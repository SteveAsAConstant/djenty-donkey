// Compare the power amp (PowerAmp5150: V3A, phase inverter, 4 x 6L6GC, output transformer, speaker, NFB, supply)
// against a NAM capture of a real 5150 / 6505 power amp driven at the effects-loop return ("POW" captures).
//
//   power_compare <model.nam> [--in-dbu 18.995] [--pres 7] [--reso 7] [--wav dir]
//
// The return jack drives C11 -> V3A from the interface (here through a splitter, ~1k source impedance, set with
// setInputSourceResistance; in the plugin the power amp is fed from the Post pot wiper, ~100k).
// The preamp keeps running idle (as in the amp: only its signal is disconnected) so the supply sees its current.
// The captures' output is a tap of the speaker voltage of unknown scale: outputs are level matched once (the
// small-signal response, 200 Hz .. 5 kHz), everything else is compared after that.
//
// 1. small-signal response: a multitone (29 tones, 31 Hz .. 16 kHz) at 20 mV per tone
// 2. compression: 110 Hz and 1 kHz sines, 0.1 .. 8 V peak at the return: fundamental out, H2..H5 re fundamental
// 3. bursts (feel): 110 Hz, quiet 0.3 V -> loud -> quiet; output level every 5 ms around both transitions, each
//    re its own quiet level before the burst. The NAM models remember ~130 ms (receptive field 6350 samples), so
//    only the first ~120 ms after a transition are compared: the coupling-cap bias shift (C28 / C29 47n x 220k
//    ~10 ms) and the plate-node sag are inside it, the preamp rails' slow recovery (200-500 ms) is not.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
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
using Signal = std::vector<double>;
using Gen = std::function<double (double)>;

double db (double v) { return 20.0 * std::log10 (std::max (std::abs (v), 1e-15)); }
double dbuToVpk (double dbu) { return 0.7746 * std::sqrt (2.0) * std::pow (10.0, dbu / 20.0); }

Signal sample (const Gen& g, double rate, double secs)
{
    Signal x ((size_t) (rate * secs));
    for (size_t i = 0; i < x.size(); ++i) x[i] = g ((double) i / rate);
    return x;
}

// amplitude of frequency f (integer Hz) over exactly one second starting at `start` seconds
double tone (const Signal& x, double rate, double f, double start)
{
    const size_t s = (size_t) (start * rate), n = (size_t) rate;
    double re = 0, im = 0;
    for (size_t i = 0; i < n && s + i < x.size(); ++i)
    {
        const double ph = 2 * kPi * f * (double) i / rate;
        re += x[s + i] * std::cos (ph); im += x[s + i] * std::sin (ph);
    }
    return 2.0 * std::sqrt (re * re + im * im) / (double) n;
}

// RMS over one period of f centred on time t
double rmsAt (const Signal& x, double rate, double t, double f)
{
    const long n = (long) std::round (rate / f), c = (long) (t * rate);
    double e = 0; long m = 0;
    for (long i = c - n / 2; i < c - n / 2 + n; ++i)
        if (i >= 0 && i < (long) x.size()) { e += x[(size_t) i] * x[(size_t) i]; ++m; }
    return std::sqrt (e / std::max (m, 1L));
}

void writeWav (const std::string& path, const Signal& x, double rate, double gain)
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

struct Rig
{
    namcmp::NamWaveNet nam;
    double vIn = 9.78;              // volts at the return per NAM full scale
    double pres = 0.7, reso = 0.7;  // 0..1
    static constexpr double rateOurs = 192000.0, kReturnDrive = 1.0, kReturnSourceR = 1000.0;

    Signal ours (const Signal& v) const
    {
        vamp::Preamp5150 pre; pre.prepare (rateOurs); pre.setMode (vamp::Preamp5150::Red);
        pre.setControls (0.5, 0.5, 0.5, 0.5, 0.3);
        vamp::PowerAmp5150 pa; pa.setInputSourceResistance (kReturnSourceR); pa.setControls (pres, reso); pa.prepare (rateOurs);
        auto step = [&] (double x)
        {
            pre.setRails (pa.getRailV1(), pa.getRailV2());
            pre.process (0.0);
            double i1, i2;
            pre.railCurrents (i1, i2);
            pa.setPreampLoad (i1, i2);
            return pa.process (kReturnDrive * x);
        };
        for (int i = 0; i < (int) (0.2 * rateOurs); ++i) step (0.0);
        Signal y (v.size());
        for (size_t i = 0; i < v.size(); ++i) y[i] = step (v[i]);
        return y;
    }
    Signal namRun (const Signal& v) const
    {
        namcmp::NamWaveNet m = nam;
        for (int i = 0; i < 8192; ++i) m.process (0.0f);
        Signal y (v.size());
        for (size_t i = 0; i < v.size(); ++i) y[i] = m.process ((float) (v[i] / vIn));
        return y;
    }
    double rateNam() const { return nam.getSampleRate(); }

    // --probe: the bias of every stage during a burst: grid-cathode voltage averaged over one 110 Hz period
    // (ports: 1 V4B, 3 V4A, 5 6L6 pair 1, 7 6L6 pair 2, 9 V3A)
    void probe (const Signal& v) const
    {
        vamp::Preamp5150 pre; pre.prepare (rateOurs); pre.setMode (vamp::Preamp5150::Red);
        pre.setControls (0.5, 0.5, 0.5, 0.5, 0.3);
        vamp::PowerAmp5150 pa; pa.setInputSourceResistance (kReturnSourceR); pa.setControls (pres, reso); pa.prepare (rateOurs);
        const int ports[] = { 9, 3, 1, 5, 7 };
        std::vector<Signal> g (5, Signal (v.size()));
        Signal plate (v.size());
        for (size_t i = 0; i < v.size(); ++i)
        {
            pre.setRails (pa.getRailV1(), pa.getRailV2());
            pre.process (0.0);
            double i1, i2;
            pre.railCurrents (i1, i2);
            pa.setPreampLoad (i1, i2);
            pa.process (kReturnDrive * v[i]);
            for (int k = 0; k < 5; ++k) g[(size_t) k][i] = pa.getPort (ports[k]);
            plate[i] = pa.getSupplyVoltage();
        }
        auto mean = [&] (const Signal& x, double t)
        {
            const long n = (long) std::round (rateOurs / 110.0), c = (long) (t * rateOurs);
            double s = 0;
            for (long i = c - n / 2; i < c - n / 2 + n; ++i) s += x[(size_t) i];
            return s / (double) n;
        };
        std::printf ("%8s | %8s %8s %8s %8s %8s | %8s\n", "t (ms)", "V3A", "V4A", "V4B", "6L6 1", "6L6 2", "B+");
        for (double t : { 0.39, 0.41, 0.45, 0.5, 0.6, 0.79, 0.805, 0.81, 0.815, 0.82, 0.825, 0.83, 0.84, 0.85, 0.87, 0.9, 1.0 })
        {
            std::printf ("%8.0f |", (t - 0.4) * 1e3);
            for (int k = 0; k < 5; ++k) std::printf (" %+8.2f", mean (g[(size_t) k], t));
            std::printf (" | %8.1f\n", mean (plate, t));
        }
    }
};

std::vector<double> multitoneFreqs()
{
    std::vector<double> f;
    for (int k = 0; k < 29; ++k) f.push_back (std::round (31.5 * std::pow (2.0, k / 3.0)));
    return f;
}

Gen multitone (double perTone)
{
    const auto f = multitoneFreqs();
    return [f, perTone] (double t)
    {
        double v = 0;
        for (size_t k = 0; k < f.size(); ++k) v += std::sin (2 * kPi * f[k] * t + 1.7 * (double) (k * k));
        return perTone * v;
    };
}

std::vector<double> gains (const Signal& y, double rate, double perTone)
{
    std::vector<double> g;
    for (double f : multitoneFreqs()) g.push_back (db (tone (y, rate, f, 0.5) / perTone));
    return g;
}

const double kLevels[] = { 0.1, 0.3, 1.0, 3.0, 8.0 };
const double kFreqs[] = { 110.0, 1000.0 };

struct Sat { double fund; double h[4]; };

Sat satOf (const Signal& y, double rate, double f)
{
    Sat s;
    const double a1 = tone (y, rate, f, 0.4);
    s.fund = db (a1);
    for (int h = 2; h <= 5; ++h) s.h[h - 2] = db (tone (y, rate, f * h, 0.4) / a1);
    return s;
}

// quiet (0.3 V) 0..0.4 s, loud 0.4..0.8 s, quiet 0.8..1.2 s
Gen burst (double loud)
{
    return [loud] (double t)
    {
        const double a = t >= 0.4 && t < 0.8 ? loud : 0.3;
        return a * std::sin (2 * kPi * 110.0 * t);
    };
}
} // namespace

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("usage: power_compare <model.nam> [--in-dbu 18.995] [--pres 7] [--reso 7] [--wav dir]\n"); return 1; }
    Rig rig;
    if (! rig.nam.load (argv[1])) { std::printf ("cannot load %s\n", argv[1]); return 1; }
    double inDbu = 18.995;
    std::string wavDir;
    for (int i = 2; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&] { return i + 1 < argc ? std::atof (argv[++i]) : 0.0; };
        if (a == "--in-dbu") inDbu = next();
        else if (a == "--pres") rig.pres = next() / 10;
        else if (a == "--reso") rig.reso = next() / 10;
        else if (a == "--wav" && i + 1 < argc) wavDir = argv[++i];
        else if (a == "--probe")
        {
            for (double loud : { 3.0, 8.0 })
            {
                std::printf ("\n== probe: burst 0.3 V -> %.0f V -> 0.3 V (loud from 0 to 400 ms); mean grid-cathode V per period ==\n", loud);
                rig.probe (sample (burst (loud), Rig::rateOurs, 1.2));
            }
            return 0;
        }
    }
    rig.vIn = dbuToVpk (inDbu);
    std::printf ("%s  (NAM %.0f Hz)  return: %.3f dBu = %.2f Vpk per full scale; presence %.1f resonance %.1f\n", argv[1],
                 rig.rateNam(), inDbu, rig.vIn, rig.pres * 10, rig.reso * 10);

    // 1. small-signal response, level match
    const double perTone = 0.02;
    const auto ro = gains (rig.ours (sample (multitone (perTone), Rig::rateOurs, 1.5)), Rig::rateOurs, perTone);
    const auto rn = gains (rig.namRun (sample (multitone (perTone), rig.rateNam(), 1.5)), rig.rateNam(), perTone);
    const auto f = multitoneFreqs();
    double off = 0; int n = 0;
    for (size_t i = 0; i < f.size(); ++i) if (f[i] >= 200 && f[i] <= 5000) { off += rn[i] - ro[i]; ++n; }
    off /= n;
    double r = 0; int nr = 0;
    for (size_t i = 0; i < f.size(); ++i) if (f[i] >= 40 && f[i] <= 12000) { const double d = ro[i] + off - rn[i]; r += d * d; ++nr; }
    std::printf ("\n== small signal (%.0f mV per tone): gain dB, ours shifted by %+.2f dB; response rms %.2f dB ==\n", perTone * 1e3, off, std::sqrt (r / nr));
    for (size_t i = 0; i < f.size(); ++i)
        std::printf ("%s%6.0f:%+6.2f", i % 6 == 0 ? "\n" : "  ", f[i], ro[i] + off - rn[i]);
    std::printf ("\n(ours - capture)\n");

    // 2. compression and harmonics
    std::printf ("\n== compression: fundamental out (dB, ours shifted) and harmonics dB re fundamental, ours / capture ==\n");
    std::printf ("%6s %7s | %15s %6s | %13s %13s %13s %13s\n", "Hz", "in Vpk", "fund o / c", "diff", "H2", "H3", "H4", "H5");
    double cs = 0; int ncs = 0;
    for (double fr : kFreqs)
        for (double a : kLevels)
        {
            const Gen g = [=] (double t) { return a * std::sin (2 * kPi * fr * t); };
            const auto o = satOf (rig.ours (sample (g, Rig::rateOurs, 1.4)), Rig::rateOurs, fr);
            const auto c = satOf (rig.namRun (sample (g, rig.rateNam(), 1.4)), rig.rateNam(), fr);
            std::printf ("%6.0f %7.2f | %+7.1f %+7.1f %+6.2f |", fr, a, o.fund + off, c.fund, o.fund + off - c.fund);
            for (int h = 0; h < 4; ++h) std::printf (" %+6.1f %+6.1f", o.h[h], c.h[h]);
            std::printf ("\n");
            cs += (o.fund + off - c.fund) * (o.fund + off - c.fund); ++ncs;
        }
    std::printf ("compression rms %.2f dB\n", std::sqrt (cs / ncs));

    // 3. bursts
    for (double loud : { 3.0, 8.0 })
    {
        const auto yo = rig.ours (sample (burst (loud), Rig::rateOurs, 1.2));
        const auto yc = rig.namRun (sample (burst (loud), rig.rateNam(), 1.2));
        const double qo = rmsAt (yo, Rig::rateOurs, 0.36, 110.0), qc = rmsAt (yc, rig.rateNam(), 0.36, 110.0);
        std::printf ("\n== burst 0.3 V -> %.0f V -> 0.3 V at 110 Hz: level dB re the quiet level before, ours / capture ==\n", loud);
        for (double t0 : { 0.4, 0.8 })
        {
            std::printf ("%s:\n", t0 < 0.5 ? "attack (loud starts)" : "release (back to quiet)");
            int k = 0;
            for (double dt = 0.005; dt <= 0.12 + 1e-9; dt += 0.005, ++k)
            {
                const double t = t0 + dt;
                const double lo = db (rmsAt (yo, Rig::rateOurs, t, 110.0) / qo), lc = db (rmsAt (yc, rig.rateNam(), t, 110.0) / qc);
                std::printf ("%s%4.0f ms %+6.2f %+6.2f", k % 4 == 0 ? "\n" : "   ", dt * 1e3, lo, lc);
            }
            std::printf ("\n");
        }
        if (! wavDir.empty())
        {
            const std::string base = wavDir + "/burst" + std::to_string ((int) loud);
            Signal d48;
            for (size_t i = 0; i < yo.size(); i += 4) d48.push_back (yo[i]);
            writeWav (base + "_ours.wav", d48, 48000.0, std::pow (10.0, off / 20) * 0.5);
            writeWav (base + "_capture.wav", yc, 48000.0, 0.5);
        }
    }
    return 0;
}
