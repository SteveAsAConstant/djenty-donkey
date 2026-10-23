// Compare the component-level amp against a NAM capture of a real amp.
//
//   nam_compare <model.nam> [input-dBu=11.5] [wav-out-dir or -] [our pre-gain knob 0..1] [our master knob 0..1]
//
// The NAM model (standard WaveNet, as written by the NAM trainer) runs at its own rate (48 kHz) on
// the same test signals the DSP model gets at 192 kHz. Input calibration: the capture's interface
// level in dBu is the RMS of a full-scale sine, so 1.0 at the NAM input = dBu -> Vpk volts at the
// amp input; our model takes volts directly. The output level of a capture (load box line out) is
// unknown, so outputs are compared after one global level match.
//
// Settings are for the capture used during development:
//   "6505 1992 Ld Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8" (OutmodedElectronics, TONE3000; not redistributed)
// Edit kSettings for another capture.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "dsp/Filters.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"

namespace
{
constexpr double kPi = vamp::kPi;

// ---------------------------------------------------------------- NAM WaveNet inference
// Layout and weight order of NeuralAmpModelerCore (wavenet.cpp): per layer array a 1x1 rechannel
// (no bias), per layer a dilated conv (bias) + input mixin (1x1, no bias) + 1x1 (bias), then a 1x1
// head rechannel (bias = head_bias); the last weight is head_scale. Conv weights are stored
// out-channel, in-channel, tap; tap K-1 is the current sample, tap k is delayed by d (K-1-k).
class NamWaveNet
{
public:
    bool load (const std::string& path)
    {
        std::ifstream f (path, std::ios::binary);
        if (! f) return false;
        std::stringstream ss; ss << f.rdbuf();
        const std::string js = ss.str();

        sampleRate = number (js, "\"sample_rate\"", 48000.0);
        // layer array configs, in order
        size_t pos = js.find ("\"layers\"");
        if (pos == std::string::npos) return false;
        while (true)
        {
            const size_t in = js.find ("\"input_size\"", pos);
            const size_t wpos = js.find ("\"weights\"");
            if (in == std::string::npos || in > wpos) break;
            Array a;
            a.inputSize = (int) number (js.substr (in), "\"input_size\"");
            a.conditionSize = (int) number (js.substr (in), "\"condition_size\"");
            a.headSize = (int) number (js.substr (in), "\"head_size\"");
            a.channels = (int) number (js.substr (in), "\"channels\"");
            a.kernel = (int) number (js.substr (in), "\"kernel_size\"");
            a.headBias = js.substr (in, js.find ('}', in) - in).find ("\"head_bias\": true") != std::string::npos;
            const std::string gated = js.substr (in, js.find ('}', in) - in);
            if (gated.find ("\"gated\": true") != std::string::npos) { std::printf ("gated WaveNet not supported\n"); return false; }
            if (gated.find ("\"activation\": \"Tanh\"") == std::string::npos) { std::printf ("only Tanh supported\n"); return false; }
            const size_t dl = js.find ('[', js.find ("\"dilations\"", in));
            const size_t dr = js.find (']', dl);
            std::stringstream ds (js.substr (dl + 1, dr - dl - 1));
            std::string tok;
            while (std::getline (ds, tok, ',')) a.dilations.push_back (std::stoi (tok));
            arrays.push_back (a);
            pos = dr;
        }

        // weights
        const size_t wl = js.find ('[', js.find ("\"weights\""));
        const size_t wr = js.find (']', wl);
        std::vector<double> w;
        {
            const char* p = js.c_str() + wl + 1;
            const char* end = js.c_str() + wr;
            while (p < end)
            {
                char* next = nullptr;
                const double v = std::strtod (p, &next);
                if (next == p) { ++p; continue; }
                w.push_back (v);
                p = next;
            }
        }

        size_t it = 0;
        auto take = [&] (size_t n) { std::vector<double> v (w.begin() + (long) it, w.begin() + (long) (it + n)); it += n; return v; };
        for (auto& a : arrays)
        {
            a.rechannel = take ((size_t) (a.channels * a.inputSize));
            for (int d : a.dilations)
            {
                Layer l;
                l.dilation = d;
                l.conv = take ((size_t) (a.channels * a.channels * a.kernel));
                l.convBias = take ((size_t) a.channels);
                l.mixin = take ((size_t) (a.channels * a.conditionSize));
                l.oneByOne = take ((size_t) (a.channels * a.channels));
                l.oneByOneBias = take ((size_t) a.channels);
                l.histLen = d * (a.kernel - 1) + 1;
                l.history.assign ((size_t) (a.channels * l.histLen), 0.0);
                a.layers.push_back (std::move (l));
            }
            a.headRechannel = take ((size_t) (a.headSize * a.channels));
            if (a.headBias) a.headRechannelBias = take ((size_t) a.headSize);
        }
        headScale = w[it++];
        if (it != w.size()) { std::printf ("weight count mismatch: used %zu of %zu\n", it, w.size()); return false; }
        return true;
    }

    double getSampleRate() const { return sampleRate; }

    float process (float x)
    {
        std::vector<double> in { (double) x };
        std::vector<double> head; // head accumulator handed from array to array
        for (size_t ai = 0; ai < arrays.size(); ++ai)
        {
            auto& a = arrays[ai];
            const int C = a.channels;
            std::vector<double> z ((size_t) C, 0.0);
            for (int o = 0; o < C; ++o)
                for (int i = 0; i < a.inputSize; ++i)
                    z[(size_t) o] += a.rechannel[(size_t) (o * a.inputSize + i)] * in[(size_t) i];
            if (head.empty()) head.assign ((size_t) C, 0.0);

            for (auto& l : a.layers)
            {
                // store the layer input in its history ring
                l.pos = (l.pos + 1) % l.histLen;
                for (int c = 0; c < C; ++c) l.history[(size_t) (l.pos * C + c)] = z[(size_t) c];

                std::vector<double> y (l.convBias);
                for (int k = 0; k < a.kernel; ++k)
                {
                    const int delay = l.dilation * (a.kernel - 1 - k);
                    const int idx = (l.pos - delay + l.histLen * 4) % l.histLen;
                    const double* h = &l.history[(size_t) (idx * C)];
                    for (int o = 0; o < C; ++o)
                    {
                        double s = 0.0;
                        const double* wrow = &l.conv[(size_t) (o * C * a.kernel)];
                        for (int i = 0; i < C; ++i) s += wrow[i * a.kernel + k] * h[i];
                        y[(size_t) o] += s;
                    }
                }
                for (int o = 0; o < C; ++o)
                {
                    y[(size_t) o] += l.mixin[(size_t) o] * x; // condition = the model input
                    y[(size_t) o] = std::tanh (y[(size_t) o]);
                    head[(size_t) o] += y[(size_t) o];
                }
                for (int o = 0; o < C; ++o)
                {
                    double s = l.oneByOneBias[(size_t) o];
                    for (int i = 0; i < C; ++i) s += l.oneByOne[(size_t) (o * C + i)] * y[(size_t) i];
                    z[(size_t) o] += s;
                }
            }

            std::vector<double> h2 ((size_t) a.headSize, 0.0);
            for (int o = 0; o < a.headSize; ++o)
            {
                double s = a.headBias ? a.headRechannelBias[(size_t) o] : 0.0;
                for (int i = 0; i < C; ++i) s += a.headRechannel[(size_t) (o * C + i)] * head[(size_t) i];
                h2[(size_t) o] = s;
            }
            head = h2;
            in = z;
        }
        return (float) (head[0] * headScale);
    }

private:
    struct Layer
    {
        int dilation = 1, histLen = 1, pos = 0;
        std::vector<double> conv, convBias, mixin, oneByOne, oneByOneBias, history;
    };
    struct Array
    {
        int inputSize = 1, conditionSize = 1, headSize = 1, channels = 1, kernel = 3;
        bool headBias = false;
        std::vector<int> dilations;
        std::vector<double> rechannel, headRechannel, headRechannelBias;
        std::vector<Layer> layers;
    };

    static double number (const std::string& js, const char* key, double def = 0.0)
    {
        const size_t k = js.find (key);
        if (k == std::string::npos) return def;
        return std::strtod (js.c_str() + js.find (':', k) + 1, nullptr);
    }

    std::vector<Array> arrays;
    double headScale = 1.0, sampleRate = 48000.0;
};

// ---------------------------------------------------------------- test signals (volts at the jack)
using Fn = std::function<double (double)>;

double pluck (double t, double f0, double decay, double amp)
{
    double v = 0;
    for (int h = 1; h <= 8; ++h)
        v += amp / h * std::exp (-t * decay * (1.0 + 0.3 * h)) * std::sin (2 * kPi * f0 * h * t + h);
    return v;
}
double hiss (double t) { return 0.0003 * std::sin (2 * kPi * 7919.0 * t) * std::sin (2 * kPi * 3.1 * t); }

struct Test { const char* name; double seconds; Fn f; };

std::vector<double> sample (const Test& t, double rate)
{
    std::vector<double> x ((size_t) (t.seconds * rate));
    for (size_t i = 0; i < x.size(); ++i) x[i] = t.f (i / rate);
    return x;
}

// ---------------------------------------------------------------- analysis
// 1/3-octave band energies (RBJ band-pass), 63 Hz .. 12.5 kHz
std::vector<double> bands (const std::vector<double>& x, double rate, double skip = 0.2)
{
    std::vector<double> out;
    for (double fc = 63.0; fc < 13000.0; fc *= std::pow (2.0, 1.0 / 3.0))
    {
        const double w0 = 2 * kPi * fc / rate, al = std::sin (w0) / (2 * 4.318), c = std::cos (w0), a0 = 1 + al;
        const double b0 = al / a0, b2 = -al / a0, a1 = -2 * c / a0, a2 = (1 - al) / a0;
        double z1 = 0, z2 = 0, e = 0;
        for (size_t i = 0; i < x.size(); ++i)
        {
            const double y = b0 * x[i] + z1;
            z1 = -a1 * y + z2;
            z2 = b2 * x[i] - a2 * y;
            if (i >= (size_t) (skip * rate)) e += y * y;
        }
        out.push_back (e / (x.size() - skip * rate));
    }
    return out;
}

double rms (const std::vector<double>& x, double rate, double skip = 0.2)
{
    double s = 0; size_t n = 0;
    for (size_t i = (size_t) (skip * rate); i < x.size(); ++i) { s += x[i] * x[i]; ++n; }
    return std::sqrt (s / std::max<size_t> (n, 1));
}

// amplitude of harmonic h of f0 (single-bin DFT over whole periods, after skip)
double harmonic (const std::vector<double>& x, double rate, double f0, int h, double skip = 0.3)
{
    const size_t start = (size_t) (skip * rate);
    const size_t periods = (size_t) ((x.size() - start) / rate * f0);
    const size_t n = (size_t) (periods * rate / f0);
    double re = 0, im = 0;
    for (size_t i = 0; i < n; ++i)
    {
        const double ph = 2 * kPi * f0 * h * i / rate;
        re += x[start + i] * std::cos (ph); im += x[start + i] * std::sin (ph);
    }
    return 2.0 * std::sqrt (re * re + im * im) / n;
}

double db (double v) { return 20.0 * std::log10 (std::max (v, 1e-15)); }

// 4x decimation for the WAV renders: windowed-sinc low-pass at 20 kHz
std::vector<double> decimate4 (const std::vector<double>& x)
{
    constexpr int taps = 127;
    std::vector<double> h (taps);
    const double fc = 20000.0 / 192000.0;
    double sum = 0;
    for (int i = 0; i < taps; ++i)
    {
        const double m = i - (taps - 1) / 2.0;
        const double s = m == 0 ? 2 * fc : std::sin (2 * kPi * fc * m) / (kPi * m);
        h[(size_t) i] = s * (0.42 - 0.5 * std::cos (2 * kPi * i / (taps - 1)) + 0.08 * std::cos (4 * kPi * i / (taps - 1)));
        sum += h[(size_t) i];
    }
    std::vector<double> y (x.size() / 4);
    for (size_t o = 0; o < y.size(); ++o)
    {
        double s = 0;
        for (int i = 0; i < taps; ++i)
        {
            const long k = (long) (o * 4) - i + (taps - 1) / 2;
            if (k >= 0 && k < (long) x.size()) s += h[(size_t) i] * x[(size_t) k];
        }
        y[o] = s / sum;
    }
    return y;
}

void writeWav (const std::string& path, const std::vector<double>& x, double rate, double gain)
{
    std::ofstream f (path, std::ios::binary);
    auto u32 = [&] (uint32_t v) { f.write ((const char*) &v, 4); };
    auto u16 = [&] (uint16_t v) { f.write ((const char*) &v, 2); };
    const uint32_t n = (uint32_t) x.size();
    f.write ("RIFF", 4); u32 (36 + n * 4); f.write ("WAVE", 4);
    f.write ("fmt ", 4); u32 (16); u16 (3); u16 (1); u32 ((uint32_t) rate); u32 ((uint32_t) rate * 4); u16 (4); u16 (32);
    f.write ("data", 4); u32 (n * 4);
    for (double v : x) { const float s = (float) (v * gain); f.write ((const char*) &s, 4); }
}

// ---------------------------------------------------------------- settings of the capture
struct Settings { int mode; double pre, bass, mid, treble, postGainDial, resonance, presence; };
constexpr Settings kSettings { 2 /* Lead = RED */, 0.5, 0.6, 0.4, 0.6, 0.3, 0.9, 0.8 };

// The real post-gain pot (assumed 10% audio taper) at the capture's dial position, converted to our
// knob (whose 60 dB curve is not a real pot taper): masterTaper(x) = 10^(3 (x - 1)) = taperA(dial)
double ourMasterKnob (double dial) { return 1.0 + std::log10 (vamp::taperA (dial)) / 3.0; }
} // namespace

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("usage: nam_compare <model.nam> [input-dBu=11.5] [wav-out-dir]\n"); return 1; }
    NamWaveNet nam;
    if (! nam.load (argv[1])) { std::printf ("could not load %s\n", argv[1]); return 1; }
    const double dbu = argc > 2 ? std::atof (argv[2]) : 11.5;
    const std::string wavDir = argc > 3 && std::string (argv[3]) != "-" ? argv[3] : "";
    const double preKnob = argc > 4 ? std::atof (argv[4]) : kSettings.pre; // diagnostics: override our pre-gain
    const double voltsPerFS = 0.7746 * std::pow (10.0, dbu / 20.0) * std::sqrt (2.0); // full-scale sine peak
    const double rateNam = nam.getSampleRate(), rateOurs = 4.0 * rateNam;
    const double master = argc > 5 ? std::atof (argv[5]) : ourMasterKnob (kSettings.postGainDial); // diagnostics: override
    std::printf ("NAM %.0f Hz, input %.1f dBu -> 1.0 FS = %.2f V; post-gain dial %.1f -> our master knob %.3f\n",
                 rateNam, dbu, voltsPerFS, kSettings.postGainDial * 10, master);

    auto ours = [&] (const std::vector<double>& v)
    {
        vamp::Preamp5150 pre; pre.prepare (rateOurs); pre.setMode (kSettings.mode);
        pre.setControls (preKnob, kSettings.bass, kSettings.mid, kSettings.treble, master);
        vamp::PowerAmp5150 pa; pa.setControls (kSettings.presence, kSettings.resonance); pa.prepare (rateOurs);
        const double post = kSettings.mode == 0 ? 25.0 : 1.0; // AmpSim kChannelPost
        std::vector<double> y (v.size());
        for (size_t i = 0; i < v.size(); ++i) y[i] = pa.process (pre.process (v[i]) * post);
        return y;
    };
    auto namRun = [&] (const std::vector<double>& v)
    {
        NamWaveNet m = nam; // fresh state
        std::vector<double> y (v.size());
        for (int i = 0; i < 4096; ++i) m.process (0.0f); // fill the receptive field with silence
        for (size_t i = 0; i < v.size(); ++i) y[i] = m.process ((float) (v[i] / voltsPerFS));
        return y;
    };

    const std::vector<Test> tests = {
        { "chugs", 3.0, [] (double t) {
              const double ph = std::fmod (t, 1.0); double v = 0;
              if (ph < 0.75) { const double tc = std::fmod (ph, 0.125); for (double f : { 36.7, 55.0, 73.4 }) v += pluck (tc, f, 25.0, 0.15); }
              return v + hiss (t); } },
        { "chord decay", 3.0, [] (double t) {
              double v = 0; for (double f : { 82.4, 123.5, 164.8, 207.7, 246.9, 329.6 }) v += pluck (t, f, 1.5, 0.1); return v + hiss (t); } },
        { "single notes", 3.0, [] (double t) {
              const double notes[] = { 110.0, 146.8, 196.0, 261.6, 329.6, 440.0 };
              const int n = std::min (5, (int) (t / 0.5)); return pluck (t - n * 0.5, notes[n], 4.0, 0.2) + hiss (t); } },
        { "sweep 30 mV", 3.0, [] (double t) {
              const double f0 = 40.0, f1 = 10000.0, T = 3.0, k = std::log (f1 / f0);
              return 0.03 * std::sin (2 * kPi * f0 * T / k * (std::exp (t / T * k) - 1.0)); } },
    };

    // one global level match (on "chugs"), applied to every comparison below
    double gainMatch = 1.0;
    std::printf ("\n== long-term spectrum, 1/3 octave, ours - NAM after one global level match (dB) ==\n");
    std::printf ("%-14s %7s |", "signal", "level");
    const double show[] = { 63, 125, 250, 500, 1000, 2000, 4000, 8000, 12500 };
    for (double f : show) std::printf (" %6.0f", f);
    std::printf (" | %8s\n", "mean|d|");
    for (size_t ti = 0; ti < tests.size(); ++ti)
    {
        const auto& t = tests[ti];
        const auto yo = ours (sample (t, rateOurs));
        const auto yn = namRun (sample (t, rateNam));
        if (ti == 0) gainMatch = rms (yn, rateNam) / rms (yo, rateOurs);
        const auto bo = bands (yo, rateOurs), bn = bands (yn, rateNam);
        std::printf ("%-14s %+6.1f |", t.name, db (rms (yo, rateOurs) * gainMatch / rms (yn, rateNam)));
        double meanAbs = 0; int nb = 0;
        size_t bi = 0;
        for (double fc = 63.0; fc < 13000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++bi)
        {
            const double d = 10 * std::log10 (bo[bi] * gainMatch * gainMatch / std::max (bn[bi], 1e-30));
            meanAbs += std::abs (d); ++nb;
            for (double f : show) if (std::abs (fc / f - 1.0) < 0.06) std::printf (" %+6.1f", d);
        }
        std::printf (" | %8.2f\n", meanAbs / nb);

        if (! wavDir.empty())
        {
            const double g = 0.5 / std::max (rms (yn, rateNam) * 4.0, 1e-9);
            const std::string base = wavDir + "/" + std::string (t.name) + "_";
            writeWav (base + "nam.wav", yn, rateNam, g);
            writeWav (base + "ours.wav", decimate4 (yo), rateNam, g * gainMatch);
        }
    }

    std::printf ("\n== 110 Hz sine: output level and harmonics (dB re fundamental) ==\n");
    std::printf ("%-8s %-5s %7s | %6s %6s %6s %6s %6s %6s\n", "in Vpk", "", "out dB", "H2", "H3", "H4", "H5", "H6", "H7");
    for (double a : { 0.003, 0.01, 0.03, 0.1, 0.3, 1.0 })
    {
        const Test t { "sine", 1.0, [a] (double tt) { return a * std::sin (2 * kPi * 110.0 * tt); } };
        const auto yo = ours (sample (t, rateOurs));
        const auto yn = namRun (sample (t, rateNam));
        for (int which = 0; which < 2; ++which)
        {
            const auto& y = which == 0 ? yo : yn;
            const double rate = which == 0 ? rateOurs : rateNam, g = which == 0 ? gainMatch : 1.0;
            const double h1 = harmonic (y, rate, 110.0, 1);
            std::printf ("%-8.3f %-5s %+7.1f |", a, which == 0 ? "ours" : "NAM", db (h1 * g));
            for (int h = 2; h <= 7; ++h) std::printf (" %+6.1f", db (harmonic (y, rate, 110.0, h) / h1));
            std::printf ("\n");
        }
    }
    return 0;
}
