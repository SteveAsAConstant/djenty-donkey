// Checks of the plugin's real amp path (AmpSim: JUCE 4x polyphase-IIR oversampling at 48 kHz) against
//   ideal  the same DSP at 4x / 8x / 16x with a long linear-phase decimator (16x: practically alias-free)
//   NAM    a capture of a real 6505 (optional), which has no aliasing of its own beyond its 48 kHz model
//
//   amp_path_test [model.nam] [input-dBu=11.5]
//
// 1. aliasing: inharmonic energy (everything that is not a harmonic of f0, 20 Hz..20 kHz) for HF sines
// 2. tone: 1/3-octave levels of the plugin path vs ideal on guitar-like input (oversampling filters)
// 3. pick attack: band levels above 2 kHz for plucks with a pick-noise transient, plugin vs NAM
// 4. dynamics: RMS envelope of a 110 Hz tone burst (quiet / loud / quiet), plugin vs NAM
// Settings: lead channel, gain 5, bass 6, mid 4, treble 6, presence 8, resonance 9, master 3
// (the "6505 1992 Ld Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8" capture).
#include <complex>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "dsp/AmpSim.h"
#include "dsp/FFT.h"
#include "NamWaveNet.h"

namespace
{
constexpr double kPi = vamp::kPi, fs = 48000.0;
using Signal = std::vector<double>;
using Gen = std::function<double (double)>;

Signal sample (const Gen& g, double rate, double secs)
{
    Signal x ((size_t) (rate * secs));
    for (size_t i = 0; i < x.size(); ++i) x[i] = g (i / rate);
    return x;
}

vamp::AmpSim::Settings settings()
{
    vamp::AmpSim::Settings s;
    s.channel = 2; s.gain = 0.5f; s.bass = 0.6f; s.mid = 0.4f; s.treble = 0.6f;
    s.presence = 0.8f; s.resonance = 0.9f; s.master = 0.3f;
    return s;
}

// the plugin's amp path: AmpSim at 48 kHz, 64-sample blocks (input digital = volts)
Signal pluginPath (const Gen& g, double secs)
{
    vamp::AmpSim amp;
    amp.prepare (fs, 64);
    const auto s = settings();
    std::vector<float> buf (64);
    for (int i = 0; i < 48000 / 64; ++i) { std::fill (buf.begin(), buf.end(), 0.0f); amp.process (buf.data(), 64, s); } // settle
    const auto x = sample (g, fs, secs);
    Signal y (x.size());
    for (size_t start = 0; start < x.size(); start += 64)
    {
        const int n = (int) std::min<size_t> (64, x.size() - start);
        for (int i = 0; i < n; ++i) buf[(size_t) i] = (float) x[start + (size_t) i];
        amp.process (buf.data(), n, s);
        for (int i = 0; i < n; ++i) y[start + (size_t) i] = buf[(size_t) i];
    }
    // remove the oversampler latency so the signals line up with the others
    const int lat = amp.getLatencySamples();
    y.erase (y.begin(), y.begin() + lat);
    y.resize (x.size(), 0.0);
    return y;
}

// ideal: the same DSP classes at 16x, decimated with a 2047-tap windowed sinc (20 kHz)
Signal idealPath (const Gen& g, double secs, int os = 16)
{
    const double rate = fs * os;
    const auto s = settings();
    vamp::Preamp5150 pre; pre.prepare (rate); pre.setMode (s.channel);
    pre.setControls (s.gain, s.bass, s.mid, s.treble, s.master);
    vamp::PowerAmp5150 pa; pa.setControls (s.presence, s.resonance); pa.prepare (rate);
    const auto x = sample (g, rate, secs);
    Signal y (x.size());
    for (size_t i = 0; i < x.size(); ++i) y[i] = pa.process (pre.process (x[i])) * vamp::kSpeakerToDigital;

    constexpr int taps = 2047;
    std::vector<double> h (taps);
    double sum = 0;
    for (int i = 0; i < taps; ++i)
    {
        const double m = i - (taps - 1) / 2.0, fc = 20000.0 / rate;
        h[(size_t) i] = (m == 0 ? 2 * fc : std::sin (2 * kPi * fc * m) / (kPi * m))
                      * (0.42 - 0.5 * std::cos (2 * kPi * i / (taps - 1)) + 0.08 * std::cos (4 * kPi * i / (taps - 1)));
        sum += h[(size_t) i];
    }
    Signal out (x.size() / os);
    for (size_t o = 0; o < out.size(); ++o)
    {
        double acc = 0;
        for (int i = 0; i < taps; ++i)
        {
            const long k = (long) (o * os) - i + (taps - 1) / 2;
            if (k >= 0 && k < (long) y.size()) acc += h[(size_t) i] * y[(size_t) k];
        }
        out[o] = acc / sum;
    }
    return out;
}

double voltsPerFS = 4.12;
Signal namPath (const namcmp::NamWaveNet& model, const Gen& g, double secs)
{
    namcmp::NamWaveNet m = model;
    for (int i = 0; i < 4096; ++i) m.process (0.0f);
    const auto x = sample (g, fs, secs);
    Signal y (x.size());
    for (size_t i = 0; i < x.size(); ++i) y[i] = m.process ((float) (x[i] / voltsPerFS));
    return y;
}

double db (double v) { return 20.0 * std::log10 (std::max (v, 1e-15)); }

// power spectrum of the last 2^16 samples, Blackman-Harris window
std::vector<double> spectrum (const Signal& y)
{
    constexpr int n = 1 << 16;
    vamp::FFT fft; fft.prepare (n);
    std::vector<std::complex<double>> d ((size_t) n);
    const size_t off = y.size() - n;
    for (int i = 0; i < n; ++i)
    {
        const double w = 0.35875 - 0.48829 * std::cos (2 * kPi * i / n) + 0.14128 * std::cos (4 * kPi * i / n)
                       - 0.01168 * std::cos (6 * kPi * i / n);
        d[(size_t) i] = y[off + (size_t) i] * w;
    }
    fft.transform (d.data(), false);
    std::vector<double> p ((size_t) n / 2);
    for (int k = 0; k < n / 2; ++k) p[(size_t) k] = std::norm (d[(size_t) k]);
    return p;
}

// inharmonic / total energy in 20 Hz .. 20 kHz (harmonics = bins within 6 of k*f0)
double inharmonicDb (const Signal& y, double f0)
{
    const auto p = spectrum (y);
    const double binHz = fs / (2.0 * (double) p.size());
    double tot = 0, inh = 0;
    for (size_t k = 0; k < p.size(); ++k)
    {
        const double f = k * binHz;
        if (f < 20.0 || f > 20000.0) continue;
        tot += p[k];
        const double h = std::round (f / f0);
        if (h < 1.0 || std::abs (f - h * f0) > 6.0 * binHz) inh += p[k];
    }
    return 10.0 * std::log10 (std::max (inh, 1e-30) / tot);
}

std::vector<double> bands (const Signal& x, double from = 0.2)
{
    std::vector<double> out;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0))
    {
        const double w0 = 2 * kPi * fc / fs, al = std::sin (w0) / (2 * 4.318), c = std::cos (w0), a0 = 1 + al;
        const double b0 = al / a0, b2 = -al / a0, a1 = -2 * c / a0, a2 = (1 - al) / a0;
        double z1 = 0, z2 = 0, e = 0;
        for (size_t i = 0; i < x.size(); ++i)
        {
            const double y = b0 * x[i] + z1;
            z1 = -a1 * y + z2; z2 = b2 * x[i] - a2 * y;
            if (i >= (size_t) (from * fs)) e += y * y;
        }
        out.push_back (e);
    }
    return out;
}

double pluck (double t, double f0, double decay, double amp)
{
    double v = 0;
    for (int h = 1; h <= 8; ++h) v += amp / h * std::exp (-t * decay * (1.0 + 0.3 * h)) * std::sin (2 * kPi * f0 * h * t + h);
    return v;
}

// pick attack: a 1.5 ms burst of high-passed noise at each pluck (deterministic)
double pickNoise (double t)
{
    if (t < 0.0 || t > 0.0015) return 0.0;
    const unsigned u = (unsigned) (t * 1e7) * 2654435761u;
    return 0.04 * ((u >> 8) / 8388608.0 - 1.0) * (1.0 - t / 0.0015);
}
} // namespace

int main (int argc, char** argv)
{
    namcmp::NamWaveNet nam;
    const bool haveNam = argc > 1 && nam.load (argv[1]);
    if (argc > 2) voltsPerFS = 0.7746 * std::pow (10.0, std::atof (argv[2]) / 20.0) * std::sqrt (2.0);
    std::printf ("plugin path = AmpSim at 48 kHz (JUCE 4x IIR oversampling); ideal = 16x + linear-phase decimation%s\n",
                 haveNam ? "; NAM = real 6505 capture" : "");

    // 1. aliasing
    std::printf ("\n== 1. aliasing: inharmonic energy re total, 20 Hz..20 kHz (lower is cleaner) ==\n");
    std::printf ("%-18s %10s %10s %10s %10s %10s\n", "sine", "plugin", "4x+FIR", "8x+FIR", "16x+FIR", "NAM");
    for (double f0 : { 997.0, 2003.0, 3001.0, 4999.0 })
        for (double a : { 0.05, 0.3 })
        {
            const Gen g = [f0, a] (double t) { return a * std::sin (2 * kPi * f0 * t); };
            const double p = inharmonicDb (pluginPath (g, 2.0), f0), i4 = inharmonicDb (idealPath (g, 2.0, 4), f0), i8 = inharmonicDb (idealPath (g, 2.0, 8), f0), id = inharmonicDb (idealPath (g, 2.0), f0);
            std::printf ("%5.0f Hz %5.2f V  %+9.1f  %+9.1f  %+9.1f  %+9.1f", f0, a, p, i4, i8, id);
            if (haveNam) std::printf ("  %+9.1f", inharmonicDb (namPath (nam, g, 2.0), f0));
            std::printf ("\n");
        }

    // 2. plugin path vs ideal on guitar-like input
    const Gen chug = [] (double t)
    {
        const double ph = std::fmod (t, 0.5), tc = std::fmod (ph, 0.125);
        double v = 0;
        for (double f : { 36.7, 55.0, 73.4 }) v += pluck (tc, f, 25.0, 0.15);
        return v + pickNoise (tc);
    };
    const auto yp = pluginPath (chug, 2.0), yi = idealPath (chug, 2.0);
    const auto bp = bands (yp), bi = bands (yi);
    std::printf ("\n== 2. tone of the plugin path vs ideal (chugs with pick noise), 1/3 octave, dB ==\n");
    double worst = 0; int bi_ = 0;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++bi_)
    {
        const double d = 10 * std::log10 (bp[(size_t) bi_] / bi[(size_t) bi_]);
        worst = std::max (worst, std::abs (d));
        if (bi_ % 3 == 0) std::printf ("  %5.0f Hz %+5.2f", fc, d);
    }
    std::printf ("\n  max |difference| over all bands: %.2f dB\n", worst);

    if (! haveNam) return 0;

    // 3. pick attack / fizz vs NAM: band levels re the 1 kHz band, plugin vs NAM
    const auto yn = namPath (nam, chug, 2.0);
    const auto bn = bands (yn);
    std::printf ("\n== 3. pick attack & fizz: band level re 1 kHz band, plugin vs NAM (chugs with pick noise) ==\n");
    int k1 = 0; { int k = 0; for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++k) if (std::abs (fc / 1000.0 - 1) < 0.06) k1 = k; }
    bi_ = 0;
    for (double fc = 63.0; fc < 17000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++bi_)
    {
        if (fc < 1900.0) continue;
        const double op = 10 * std::log10 (bp[(size_t) bi_] / bp[(size_t) k1]), on = 10 * std::log10 (bn[(size_t) bi_] / bn[(size_t) k1]);
        std::printf ("  %5.0f Hz: plugin %+6.1f  NAM %+6.1f  diff %+5.1f\n", fc, op, on, op - on);
    }

    // 4. dynamics: tone burst quiet (0.03 V) / loud (0.3 V) / quiet, 10 ms RMS envelope
    const Gen burst = [] (double t)
    {
        const double a = (t >= 0.5 && t < 1.0) ? 0.3 : 0.03;
        return a * std::sin (2 * kPi * 110.0 * t);
    };
    const auto ep = pluginPath (burst, 1.6), en = namPath (nam, burst, 1.6);
    auto env = [] (const Signal& y, double t) { double s = 0; const size_t n = (size_t) (4.0 * fs / 110.0), a = (size_t) (t * fs) - n / 2; for (size_t i = a; i < a + n; ++i) s += y[i] * y[i]; return std::sqrt (s / n); };
    std::printf ("\n== 4. dynamics: 110 Hz burst 0.03 V -> 0.3 V (at 0.5 s) -> 0.03 V (at 1.0 s), 4-period RMS re end of loud part (dB) ==\n");
    std::printf ("%8s %8s %8s\n", "time", "plugin", "NAM");
    const double rp = env (ep, 0.98), rn = env (en, 0.98);
    for (double t : { 0.40, 0.49, 0.505, 0.52, 0.55, 0.60, 0.70, 0.90, 0.98, 1.005, 1.02, 1.05, 1.10, 1.20, 1.40, 1.58 })
        std::printf ("%7.3fs %+8.2f %+8.2f\n", t, db (env (ep, t) / rp), db (env (en, t) / rn));
    return 0;
}
