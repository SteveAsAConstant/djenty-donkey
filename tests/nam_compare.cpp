// Compare the component-level amp against a NAM capture of a real amp.
//
//   nam_compare <model.nam> [--dbu 11.5] [--pre knob] [--drive-db d] [--wav dir] [--fit]
//
// The NAM model (standard WaveNet, as written by the NAM trainer) runs at its own rate (48 kHz) on
// the same test signals the DSP model gets at 192 kHz. Input calibration: the capture's interface
// level in dBu is the RMS of a full-scale sine, so 1.0 at the NAM input = dBu -> Vpk volts at the
// amp input; our model takes volts directly. The output level of a capture (load box line out) is
// unknown, so outputs are compared after one global level match.
//
// Knob settings are parsed from the model name, e.g. "6505 1992 Ld Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8"
// (OutmodedElectronics' 6505 pack on TONE3000, used during development; captures are not redistributed).
// --fit grid-searches our pre-gain knob and the power-amp drive that match the capture best.
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "dsp/Filters.h"
#include "dsp/Amp5150.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "NamWaveNet.h"

namespace
{
constexpr double kPi = vamp::kPi;
using namcmp::NamWaveNet;


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
// Settings are parsed from the NAM model name ("... Ld Pr-5 B-6 M-4 T-6 Po-3 R-9 P-8"); dials 0..10.
struct Settings { int mode = 2; double pre = 0.5, bass = 0.5, mid = 0.5, treble = 0.5, post = 0.3, resonance = 0.5, presence = 0.5; };

Settings parseSettings (const std::string& name)
{
    Settings s;
    s.mode = name.find (" Rhy ") != std::string::npos ? 1 : 2; // Rhythm (crunch) = BLUE, Lead = RED
    auto dial = [&] (const char* key, double def)
    {
        const size_t k = name.find (key);
        return k == std::string::npos ? def : std::atof (name.c_str() + k + std::strlen (key)) / 10.0;
    };
    s.pre = dial (" Pr-", s.pre); s.bass = dial (" B-", s.bass); s.mid = dial (" M-", s.mid); s.treble = dial (" T-", s.treble);
    s.post = dial (" Po-", s.post); s.resonance = dial (" R-", s.resonance); s.presence = dial (" P-", s.presence);
    return s;
}

struct Rig
{
    NamWaveNet nam;
    Settings s;
    double voltsPerFS = 4.12, rateNam = 48000.0, rateOurs = 192000.0;
    bool frozenSupply = false; // --frozen-supply: rails held at their DC point (no sag), to separate DC from dynamics
    double r18 = -1.0;         // --r18 ohms: V1B cathode R18 in the lead preamp (default: FullPreamp5150::r18)
    bool crunchOff = false;    // --crunch-off: Rhythm captures against the rhythm channel with crunch off (all five stages)
    bool preOnly = false;      // --pre-only: preamp output (tone stack and Post, rails at idle) against a preamp-only capture
    int bright = -1;           // --bright 0/1: the BRIGHT pushbutton (default: the plugin's default)

    std::vector<double> ours (const std::vector<double>& v, double preKnob, double drive) const
    {
        vamp::Preamp5150 pre (false, ! crunchOff);
        if (r18 >= 0.0) pre.lead.r18 = r18;
        if (bright >= 0) pre.setBright (bright != 0);
        pre.prepare (rateOurs); pre.setMode (s.mode);
        pre.setControls (preKnob, s.bass, s.mid, s.treble, vamp::masterKnobForDrive (drive));
        vamp::PowerAmp5150 pa; pa.setControls (s.presence, s.resonance); pa.prepare (rateOurs);
        if (frozenSupply) pa.beginSupplySettle();
        const double post = s.mode == 0 ? 54.0 : 1.0; // AmpSim kChannelPost
        std::vector<double> y (v.size());
        if (preOnly) for (size_t i = 0; i < v.size(); ++i) y[i] = pre.process (v[i]) * post;
        else for (size_t i = 0; i < v.size(); ++i) y[i] = vamp::processAmp5150 (pre, pa, v[i], post);
        return y;
    }
    std::vector<double> namRun (const std::vector<double>& v) const
    {
        NamWaveNet m = nam; // fresh state
        std::vector<double> y (v.size());
        for (int i = 0; i < 4096; ++i) m.process (0.0f); // fill the receptive field with silence
        for (size_t i = 0; i < v.size(); ++i) y[i] = m.process ((float) (v[i] / voltsPerFS));
        return y;
    }
};

std::vector<Test> guitarTests (double seconds)
{
    return {
        { "chugs", seconds, [] (double t) {
              const double ph = std::fmod (t, 1.0); double v = 0;
              if (ph < 0.75) { const double tc = std::fmod (ph, 0.125); for (double f : { 36.7, 55.0, 73.4 }) v += pluck (tc, f, 25.0, 0.15); }
              return v + hiss (t); } },
        { "chord decay", seconds, [] (double t) {
              double v = 0; for (double f : { 82.4, 123.5, 164.8, 207.7, 246.9, 329.6 }) v += pluck (t, f, 1.5, 0.1); return v + hiss (t); } },
        { "single notes", seconds, [seconds] (double t) {
              const double notes[] = { 110.0, 146.8, 196.0, 261.6, 329.6, 440.0 };
              const double step = seconds / 6.0;
              const int n = std::min (5, (int) (t / step)); return pluck (t - n * step, notes[n], 4.0, 0.2) + hiss (t); } },
        { "sweep 30 mV", seconds, [seconds] (double t) {
              const double f0 = 40.0, f1 = 10000.0, k = std::log (f1 / f0);
              return 0.03 * std::sin (2 * kPi * f0 * seconds / k * (std::exp (t / seconds * k) - 1.0)); } },
    };
}

// NAM renders are fixed for a capture: compute once
struct Reference
{
    std::vector<std::vector<double>> bandsNam;  // per guitar test
    std::vector<double> rmsNam;
    std::vector<double> sineLevels { 0.01, 0.1, 0.5 };
    std::vector<std::vector<double>> harmNam;   // per sine level: H1..H7 amplitudes
};

Reference makeReference (const Rig& rig, const std::vector<Test>& tests)
{
    Reference r;
    for (const auto& t : tests)
    {
        const auto y = rig.namRun (sample (t, rig.rateNam));
        r.bandsNam.push_back (bands (y, rig.rateNam));
        r.rmsNam.push_back (rms (y, rig.rateNam));
    }
    for (double a : r.sineLevels)
    {
        const Test t { "sine", 0.6, [a] (double tt) { return a * std::sin (2 * kPi * 110.0 * tt); } };
        const auto y = rig.namRun (sample (t, rig.rateNam));
        std::vector<double> h;
        for (int k = 1; k <= 7; ++k) h.push_back (harmonic (y, rig.rateNam, 110.0, k, 0.2));
        r.harmNam.push_back (h);
    }
    return r;
}

// Cost: spectra (mean |dB| over 1/3-octave bands after one global level match), harmonic structure
// (mean |dB| of H2..H7 re H1, floored at -60 dB) and compression (spread of H1 level differences)
struct Cost { double total, spectrum, harmonics, compression, highs; }; // highs: mean ours - NAM above 4 kHz (dB)

Cost evaluate (const Rig& rig, const Reference& ref, const std::vector<Test>& tests, double preKnob, double drive)
{
    std::vector<std::vector<double>> bo;
    std::vector<double> logRatios;
    for (size_t ti = 0; ti < tests.size(); ++ti)
    {
        bo.push_back (bands (rig.ours (sample (tests[ti], rig.rateOurs), preKnob, drive), rig.rateOurs));
        for (size_t b = 0; b < bo.back().size(); ++b)
            logRatios.push_back (10 * std::log10 (std::max (ref.bandsNam[ti][b], 1e-30) / std::max (bo.back()[b], 1e-30)));
    }
    double g = 0; for (double l : logRatios) g += l; g /= (double) logRatios.size(); // global level match (dB)
    double spec = 0; for (double l : logRatios) spec += std::abs (l - g); spec /= (double) logRatios.size();
    double highs = 0; int nh = 0;
    for (size_t i = 0; i < logRatios.size(); ++i)
        if (63.0 * std::pow (2.0, (double) (i % ref.bandsNam[0].size()) / 3.0) >= 3900.0) { highs -= logRatios[i] - g; ++nh; }
    highs /= std::max (nh, 1);

    double harm = 0, n = 0;
    std::vector<double> h1diff;
    for (size_t li = 0; li < ref.sineLevels.size(); ++li)
    {
        const double a = ref.sineLevels[li];
        const Test t { "sine", 0.6, [a] (double tt) { return a * std::sin (2 * kPi * 110.0 * tt); } };
        const auto y = rig.ours (sample (t, rig.rateOurs), preKnob, drive);
        const double h1 = harmonic (y, rig.rateOurs, 110.0, 1, 0.2);
        h1diff.push_back (db (h1) - db (ref.harmNam[li][0]));
        for (int k = 2; k <= 7; ++k)
        {
            const double ours = std::max (db (harmonic (y, rig.rateOurs, 110.0, k, 0.2) / h1), -60.0);
            const double nam = std::max (db (ref.harmNam[li][(size_t) k - 1] / ref.harmNam[li][0]), -60.0);
            harm += std::abs (ours - nam); ++n;
        }
    }
    harm /= n;
    double m = 0; for (double d : h1diff) m += d; m /= (double) h1diff.size();
    double comp = 0; for (double d : h1diff) comp += std::abs (d - m); comp /= (double) h1diff.size();
    return { spec + 0.5 * harm + comp, spec, harm, comp, highs };
}
} // namespace

int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::printf ("usage: nam_compare <model.nam> [--dbu 11.5] [--pre knob] [--drive-db d] [--wav dir] [--fit]\n"
                     "  --drive-db  power-amp drive relative to a real 10%% audio-taper post-gain pot at the capture's dial\n"
                     "  --fit       grid-search our pre-gain knob and the drive offset that match the capture best\n");
        return 1;
    }
    Rig rig;
    if (! rig.nam.load (argv[1])) { std::printf ("could not load %s\n", argv[1]); return 1; }
    double dbu = 11.5, driveDb = 0.0, preOverride = -1.0;
    bool driveOverride = false;
    std::string wavDir;
    bool fit = false, fitPre = false, fitDrive = false, residual = false, sweep = false;
    double smallSignal = 0.0; // --ss volts per tone: small-signal response (multitone) against the capture
    for (int i = 2; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--dbu" && i + 1 < argc) dbu = std::atof (argv[++i]);
        else if (a == "--pre" && i + 1 < argc) preOverride = std::atof (argv[++i]);
        else if (a == "--drive-db" && i + 1 < argc) { driveDb = std::atof (argv[++i]); driveOverride = true; }
        else if (a == "--wav" && i + 1 < argc) wavDir = argv[++i];
        else if (a == "--fit") fit = true;
        else if (a == "--fit-pre") fitPre = true;
        else if (a == "--residual") residual = true;
        else if (a == "--sweep") sweep = true;
        else if (a == "--frozen-supply") rig.frozenSupply = true;
        else if (a == "--r18" && i + 1 < argc) rig.r18 = std::atof (argv[++i]);
        else if (a == "--crunch-off") rig.crunchOff = true;
        else if (a == "--pre-only") rig.preOnly = true;
        else if (a == "--bright" && i + 1 < argc) rig.bright = std::atoi (argv[++i]);
        else if (a == "--ss" && i + 1 < argc) smallSignal = std::atof (argv[++i]);
        else if (a == "--fit-drive") fitDrive = true;
    }
    // dials from the model's name, or from the file name when the model's name has none (e.g. 2dor's captures,
    // whose settings are in the author's spreadsheet: copy the file to a name like "... Ld Pr-4 B-6 ... P-7.nam")
    rig.s = parseSettings (rig.nam.getName().find (" Pr-") != std::string::npos ? rig.nam.getName() : std::string (argv[1]));
    rig.voltsPerFS = 0.7746 * std::pow (10.0, dbu / 20.0) * std::sqrt (2.0); // full-scale sine peak
    rig.rateNam = rig.nam.getSampleRate();
    rig.rateOurs = 4.0 * rig.rateNam;
    // drive offsets (--drive-db, the fit) are relative to a plain 10% audio-taper post-gain pot
    const double driveRealPot = vamp::taperA (rig.s.post);
    const double driveCalibrated = vamp::masterTaper (rig.s.post); // the plugin's Master curve
    std::printf ("%s\n%s, pre %.1f bass %.1f mid %.1f treble %.1f post %.1f res %.1f pres %.1f | input %.1f dBu -> 1.0 FS = %.2f V\n",
                 rig.nam.getName().c_str(), rig.s.mode == 2 ? "RED" : "BLUE", rig.s.pre * 10, rig.s.bass * 10, rig.s.mid * 10,
                 rig.s.treble * 10, rig.s.post * 10, rig.s.resonance * 10, rig.s.presence * 10, dbu, rig.voltsPerFS);

    if (sweep)
    {
        // Distortion onset: 110 Hz sine from 1 mV to 1 V in 2 dB steps at the capture's dials. Per level the
        // output re its level at 1 V (compression curve) and H2..H4 re the fundamental, ours and NAM.
        // Last line (SWEEP): mean |ours - NAM| of the compression curve and of H2 / H3 / H4 (harmonics floored at
        // -70 dB), over all levels and over the levels below 30 mV (small signal / onset).
        std::vector<double> lv;
        for (double d = -60.0; d <= 0.01; d += 2.0) lv.push_back (std::pow (10.0, d / 20.0));
        struct Row { double h[5]; };
        std::vector<Row> ro, rn;
        for (double a : lv)
        {
            const Test t { "sine", 0.8, [a] (double tt) { return a * std::sin (2 * kPi * 110.0 * tt); } };
            const auto yo = rig.ours (sample (t, rig.rateOurs), rig.s.pre, driveCalibrated);
            const auto yn = rig.namRun (sample (t, rig.rateNam));
            Row o {}, m {};
            for (int h = 1; h <= 4; ++h)
            {
                o.h[h] = harmonic (yo, rig.rateOurs, 110.0, h);
                m.h[h] = harmonic (yn, rig.rateNam, 110.0, h);
            }
            ro.push_back (o); rn.push_back (m);
        }
        auto hdb = [] (const Row& r, int h) { return std::max (db (r.h[h] / r.h[1]), -70.0); };
        std::printf ("%8s | %7s %7s | %13s | %13s | %13s\n", "in dBV", "comp", "comp N", "H2 ours/NAM", "H3 ours/NAM", "H4 ours/NAM");
        double sum[4] {}, sumLo[4] {}; int n = 0, nLo = 0;
        for (size_t i = 0; i < lv.size(); ++i)
        {
            const double co = db (ro[i].h[1] / ro.back().h[1]) - db (lv[i]), cn = db (rn[i].h[1] / rn.back().h[1]) - db (lv[i]);
            std::printf ("%8.0f | %+7.1f %+7.1f | %6.1f %6.1f | %6.1f %6.1f | %6.1f %6.1f\n", db (lv[i]), co, cn,
                         hdb (ro[i], 2), hdb (rn[i], 2), hdb (ro[i], 3), hdb (rn[i], 3), hdb (ro[i], 4), hdb (rn[i], 4));
            const double d[4] = { std::abs (co - cn), std::abs (hdb (ro[i], 2) - hdb (rn[i], 2)),
                                  std::abs (hdb (ro[i], 3) - hdb (rn[i], 3)), std::abs (hdb (ro[i], 4) - hdb (rn[i], 4)) };
            for (int k = 0; k < 4; ++k) { sum[k] += d[k]; if (lv[i] < 0.03) sumLo[k] += d[k]; }
            ++n; if (lv[i] < 0.03) ++nLo;
        }
        std::printf ("SWEEP pre %.1f | all: comp %.2f H2 %.2f H3 %.2f H4 %.2f | <30mV: comp %.2f H2 %.2f H3 %.2f H4 %.2f\n", rig.s.pre * 10,
                     sum[0] / n, sum[1] / n, sum[2] / n, sum[3] / n, sumLo[0] / nLo, sumLo[1] / nLo, sumLo[2] / nLo, sumLo[3] / nLo);
        return 0;
    }

    if (smallSignal > 0.0)
    {
        // small-signal response: 29 tones (1/3 octave, 31 Hz .. 16 kHz, integer Hz) at `smallSignal` volts each, one
        // second analysed after 0.5 s; ours - NAM per tone after matching the level over 200 Hz .. 5 kHz
        std::vector<double> f;
        for (int k = 0; k < 29; ++k) f.push_back (std::round (31.5 * std::pow (2.0, k / 3.0)));
        const double a = smallSignal;
        const Fn mt = [f, a] (double t) { double v = 0; for (size_t k = 0; k < f.size(); ++k) v += std::sin (2 * kPi * f[k] * t + 1.7 * (double) (k * k)); return a * v; };
        const Test tst { "multitone", 1.5, mt };
        const auto yo = rig.ours (sample (tst, rig.rateOurs), preOverride >= 0.0 ? preOverride : rig.s.pre, driveCalibrated);
        const auto yn = rig.namRun (sample (tst, rig.rateNam));
        auto amp = [] (const std::vector<double>& x, double rate, double fr)
        {
            const size_t s = (size_t) (0.5 * rate), n = (size_t) rate;
            double re = 0, im = 0;
            for (size_t i = 0; i < n && s + i < x.size(); ++i) { const double ph = 2 * kPi * fr * (double) i / rate; re += x[s + i] * std::cos (ph); im += x[s + i] * std::sin (ph); }
            return 2.0 * std::sqrt (re * re + im * im) / (double) n;
        };
        std::vector<double> d;
        double off = 0; int n = 0;
        for (double fr : f) { d.push_back (db (amp (yo, rig.rateOurs, fr) / amp (yn, rig.rateNam, fr))); if (fr >= 200 && fr <= 5000) { off += d.back(); ++n; } }
        off /= n;
        std::printf ("SS %.0e V/tone, ours %+.1f dB re capture (200 Hz-5 kHz):", a, off);
        for (size_t k = 0; k < f.size(); ++k) std::printf (" %.0f:%+.1f", f[k], d[k] - off);
        std::printf ("\n");
        return 0;
    }

    if (residual)
    {
        // one machine-readable line: dials, then ours - NAM per 1/3-octave band (63 Hz..12.5 kHz),
        // averaged over the guitar signals after one global level match (for regressions over captures)
        const auto tests = guitarTests (1.5);
        const auto ref = makeReference (rig, tests);
        std::vector<double> sum (ref.bandsNam[0].size(), 0.0);
        std::vector<double> all;
        for (size_t ti = 0; ti < tests.size(); ++ti)
        {
            const auto bo = bands (rig.ours (sample (tests[ti], rig.rateOurs), preOverride >= 0.0 ? preOverride : rig.s.pre,
                                             driveCalibrated), rig.rateOurs);
            for (size_t b = 0; b < bo.size(); ++b)
            {
                const double d = 10 * std::log10 (std::max (bo[b], 1e-30) / std::max (ref.bandsNam[ti][b], 1e-30));
                sum[b] += d / (double) tests.size();
                all.push_back (d);
            }
        }
        double g = 0; for (double d : all) g += d; g /= (double) all.size();
        std::printf ("RES %.2f %.2f %.2f %.2f %.2f %.2f", rig.s.pre * 10, rig.s.bass * 10, rig.s.mid * 10, rig.s.treble * 10,
                     rig.s.presence * 10, rig.s.resonance * 10);
        for (double s : sum) std::printf (" %.2f", s - g);
        std::printf ("\n");
        return 0;
    }

    if (fitDrive)
    {
        // power-amp drive only (Gain at the capture's dial): checks the Master scale
        const auto tests = guitarTests (1.5);
        const auto ref = makeReference (rig, tests);
        Cost best { 1e9, 0, 0, 0, 0 };
        double bestDb = 0.0;
        for (double d = -15.0; d <= 15.01; d += 1.0)
        {
            const auto c = evaluate (rig, ref, tests, rig.s.pre, driveCalibrated * std::pow (10.0, d / 20.0));
            if (c.total < best.total) { best = c; bestDb = d; }
        }
        const auto atNominal = evaluate (rig, ref, tests, rig.s.pre, driveCalibrated);
        std::printf ("FITDRIVE pre_dial %.1f -> drive %+.1f dB re the Master scale | cost %.2f (spec %.2f harm %.2f comp %.2f) highs %+.1f | nominal cost %.2f (spec %.2f harm %.2f comp %.2f) highs %+.1f\n",
                     rig.s.pre * 10, bestDb, best.total, best.spectrum, best.harmonics, best.compression, best.highs,
                     atNominal.total, atNominal.spectrum, atNominal.harmonics, atNominal.compression, atNominal.highs);
        return 0;
    }

    if (fitPre)
    {
        // pre-gain only, with the power-amp drive fixed at the plugin's Master calibration: for recalibrating
        // the Gain curve (the joint fit trades pre-gain against drive where the cost is flat in one of them)
        const auto tests = guitarTests (1.5);
        const auto ref = makeReference (rig, tests);
        Cost best { 1e9, 0, 0, 0, 0 };
        double bestPre = rig.s.pre;
        // the knob's pot curve: lead (VR2) or rhythm (VR1)
        auto taper = [&] (double x) { return rig.s.mode == 2 ? vamp::leadPreTaper (x) : vamp::rhythmPreTaper (x); };
        const double lo = std::max (0.02, rig.s.pre - (rig.s.mode == 2 ? 0.15 : 0.35));
        const double hi = std::min (1.0, rig.s.pre + (rig.s.mode == 2 ? 0.25 : 0.35));
        for (double p = lo; p <= hi + 1e-9; p += 0.01)
        {
            const auto c = evaluate (rig, ref, tests, p, driveCalibrated);
            if (c.total < best.total) { best = c; bestPre = p; }
        }
        const auto atNominal = evaluate (rig, ref, tests, rig.s.pre, driveCalibrated);
        std::printf ("FITPRE pre_dial %.1f -> our_pre %.2f%s | pot %.2f dB, at dial %.2f dB -> %+.2f dB | cost %.2f (spec %.2f harm %.2f comp %.2f) | nominal cost %.2f (spec %.2f harm %.2f comp %.2f)\n",
                     rig.s.pre * 10, bestPre, bestPre >= hi - 0.005 || bestPre <= lo + 0.005 ? " (search limit)" : "",
                     db (taper (bestPre)), db (taper (rig.s.pre)), db (taper (bestPre) / taper (rig.s.pre)),
                     best.total, best.spectrum, best.harmonics, best.compression,
                     atNominal.total, atNominal.spectrum, atNominal.harmonics, atNominal.compression);
        return 0;
    }

    if (fit)
    {
        const auto tests = guitarTests (1.5);
        const auto ref = makeReference (rig, tests);
        Cost best { 1e9, 0, 0, 0, 0 };
        double bestPre = rig.s.pre, bestDb = 0;
        for (double p = std::max (0.05, rig.s.pre - 0.2); p <= std::min (1.0, rig.s.pre + 0.21); p += 0.05)
            for (double d = -30.0; d <= 0.01; d += 2.5)
            {
                const auto c = evaluate (rig, ref, tests, p, driveRealPot * std::pow (10.0, d / 20.0));
                if (c.total < best.total) { best = c; bestPre = p; bestDb = d; }
            }
        // refine
        for (double p = bestPre - 0.04; p <= bestPre + 0.041; p += 0.02)
            for (double d = bestDb - 2.0; d <= bestDb + 2.01; d += 1.0)
            {
                if (p <= 0.0 || d > 6.0) continue;
                const auto c = evaluate (rig, ref, tests, p, driveRealPot * std::pow (10.0, d / 20.0));
                if (c.total < best.total) { best = c; bestPre = p; bestDb = d; }
            }
        const auto atNominal = evaluate (rig, ref, tests, rig.s.pre, driveCalibrated); // the plugin at the capture's dials
        std::printf ("FIT pre_dial %.1f post_dial %.1f -> our_pre %.2f drive_db %+.1f | cost %.2f (spec %.2f harm %.2f comp %.2f) highs %+.1f | nominal cost %.2f (spec %.2f harm %.2f comp %.2f)\n",
                     rig.s.pre * 10, rig.s.post * 10, bestPre, bestDb, best.total, best.spectrum, best.harmonics, best.compression, best.highs,
                     atNominal.total, atNominal.spectrum, atNominal.harmonics, atNominal.compression);
        return 0;
    }

    const double preKnob = preOverride >= 0.0 ? preOverride : rig.s.pre;
    const double drive = driveOverride ? driveRealPot * std::pow (10.0, driveDb / 20.0) : driveCalibrated;
    driveDb = 20.0 * std::log10 (drive / driveRealPot);
    std::printf ("our pre %.2f, power-amp drive %+.1f dB re real pot (our master knob %.3f)\n", preKnob, driveDb, vamp::masterKnobForDrive (drive));

    const auto tests = guitarTests (3.0);
    double gainMatch = 1.0;
    std::printf ("\n== long-term spectrum, 1/3 octave, ours - NAM after one global level match (dB) ==\n");
    std::printf ("%-14s %7s |", "signal", "level");
    const double show[] = { 63, 125, 250, 500, 1000, 2000, 4000, 8000, 12500 };
    for (double f : show) std::printf (" %6.0f", f);
    std::printf (" | %8s\n", "mean|d|");
    for (size_t ti = 0; ti < tests.size(); ++ti)
    {
        const auto& t = tests[ti];
        const auto yo = rig.ours (sample (t, rig.rateOurs), preKnob, drive);
        const auto yn = rig.namRun (sample (t, rig.rateNam));
        if (ti == 0) gainMatch = rms (yn, rig.rateNam) / rms (yo, rig.rateOurs);
        const auto bo = bands (yo, rig.rateOurs), bn = bands (yn, rig.rateNam);
        std::printf ("%-14s %+6.1f |", t.name, db (rms (yo, rig.rateOurs) * gainMatch / rms (yn, rig.rateNam)));
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
            const double g = 0.5 / std::max (rms (yn, rig.rateNam) * 4.0, 1e-9);
            const std::string base = wavDir + "/" + std::string (t.name) + "_";
            writeWav (base + "nam.wav", yn, rig.rateNam, g);
            writeWav (base + "ours.wav", decimate4 (yo), rig.rateNam, g * gainMatch);
        }
    }

    std::printf ("\n== 110 Hz sine: output level and harmonics (dB re fundamental) ==\n");
    std::printf ("%-8s %-5s %7s | %6s %6s %6s %6s %6s %6s\n", "in Vpk", "", "out dB", "H2", "H3", "H4", "H5", "H6", "H7");
    for (double a : { 0.003, 0.01, 0.03, 0.1, 0.3, 1.0 })
    {
        const Test t { "sine", 1.0, [a] (double tt) { return a * std::sin (2 * kPi * 110.0 * tt); } };
        const auto yo = rig.ours (sample (t, rig.rateOurs), preKnob, drive);
        const auto yn = rig.namRun (sample (t, rig.rateNam));
        for (int which = 0; which < 2; ++which)
        {
            const auto& y = which == 0 ? yo : yn;
            const double rate = which == 0 ? rig.rateOurs : rig.rateNam, g = which == 0 ? gainMatch : 1.0;
            const double h1 = harmonic (y, rate, 110.0, 1);
            std::printf ("%-8.3f %-5s %+7.1f |", a, which == 0 ? "ours" : "NAM", db (h1 * g));
            for (int h = 2; h <= 7; ++h) std::printf (" %+6.1f", db (harmonic (y, rate, 110.0, h) / h1));
            std::printf ("\n");
        }
    }
    return 0;
}
