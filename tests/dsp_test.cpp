// Offline checks of the component-level DSP against the LTspice reference runs.
// Build target: dsp_test (no JUCE needed).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "dsp/FFT.h"
#include "dsp/MultibandGate.h"
#include "dsp/PitchShifter.h"
#include "dsp/Amp5150.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "dsp/PrecisionDrive.h"
#include "dsp/Urei1176.h"

using namespace vamp;

static constexpr double fs = 192000.0;

struct Stat
{
    double sum = 0, mn = 1e300, mx = -1e300; int n = 0;
    void add (double v) { sum += v; mn = std::min (mn, v); mx = std::max (mx, v); ++n; }
    double avg() const { return sum / n; }
    double pp() const { return mx - mn; }
};

static void leadPreampVsSpice()
{
    std::printf ("\n== 5150 lead preamp (factory rails 305.7 / 322.1 V, V5A loaded, pot 0.5, 1 kHz) ==\n");
    std::printf ("%-8s %8s %8s %8s %8s %8s %8s | %8s %8s %8s %8s %8s %8s\n", "amp",
                 "Vk1a", "Vk1b", "Vk2a", "Vk2b", "Vk5b", "Vk5a", "p1a_pp", "p1b_pp", "p2a_pp", "p2b_pp", "p5b_pp", "out_pp");

    for (double amp : { 0.001, 0.01, 0.1, 0.5 })
    {
        Preamp5150 pre (true); // diagnostics: all plate and cathode voltages
        pre.prepare (fs); // mode Red by default
        pre.setPreGainPot (0.5);

        Stat k[6], p[6]; // RED: the whole preamp is pre.lead (coupled), real plate voltages
        const int total = (int) (fs * 0.06), from = (int) (fs * 0.04);
        for (int i = 0; i < total; ++i)
        {
            pre.process (amp * std::sin (2.0 * kPi * 1000.0 * i / fs));
            if (i >= from)
                for (int s = 0; s < 6; ++s) { k[s].add (pre.lead.getVk (s)); p[s].add (pre.lead.getVp (s)); }
        }
        std::printf ("%-8g %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f | %8.3f %8.2f %8.2f %8.1f %8.1f %8.1f\n", amp,
                     k[0].avg(), k[1].avg(), k[2].avg(), k[3].avg(), k[4].avg(), k[5].avg(),
                     p[0].pp(), p[1].pp(), p[2].pp(), p[3].pp(), p[4].pp(), p[5].pp());
    }
}

static void toneStackResponse()
{
    std::printf ("\n== tone stack |H| dB (t, m, l raw pot fractions) ==\n");
    const double freqs[] = { 50, 100, 200, 500, 1000, 2000, 5000, 10000 };
    std::printf ("%-16s", "setting");
    for (double f : freqs) std::printf (" %7.0f", f);
    std::printf ("\n");

    const double settings[][3] = { { 0.5, 0.5, 0.5 }, { 1.0, 0.0, 1.0 }, { 0.0, 1.0, 0.0 }, { 0.7, 0.3, 0.1 } };
    for (auto& s : settings)
    {
        std::printf ("t%.1f m%.1f l%.1f   ", s[0], s[1], s[2]);
        for (double f : freqs)
        {
            Preamp5150 pre;
            pre.prepare (fs);
            pre.setToneStackRaw (s[0], s[1], s[2]);
            Stat o;
            const int total = (int) (fs * 0.5), from = (int) (fs * 0.3);
            for (int i = 0; i < total; ++i)
            {
                const double y = pre.processToneStack (std::sin (2.0 * kPi * f * i / fs));
                if (i >= from) o.add (y);
            }
            std::printf (" %7.2f", 20.0 * std::log10 (o.pp() / 2.0));
        }
        std::printf ("\n");
    }
}

static void driveSanity()
{
    std::printf ("\n== Precision Drive: 0.2 V peak 1 kHz in, volume 10 ==\n");
    for (double drv : { 0.0, 0.5, 1.0 })
        for (int atk : { 0, 5 })
        {
            PrecisionDrive pd;
            pd.prepare (fs);
            pd.setControls (drv, 0.5, atk, 1.0);
            Stat o;
            for (int i = 0; i < (int) (fs * 0.1); ++i)
            {
                const double y = pd.process (0.2 * std::sin (2.0 * kPi * 1000.0 * i / fs));
                if (i > (int) (fs * 0.05)) o.add (y);
            }
            std::printf ("drive %.1f attack %d -> out %.3f Vpp\n", drv, atk + 1, o.pp());
        }

    std::printf ("\n== TS9: 0.2 V peak 1 kHz in, level 10 ==\n");
    for (double drv : { 0.0, 0.5, 1.0 })
        for (double tone : { 0.0, 1.0 })
        {
            TubeScreamer9 ts;
            ts.prepare (fs);
            ts.setControls (drv, tone, 1.0);
            Stat o;
            for (int i = 0; i < (int) (fs * 0.1); ++i)
            {
                const double y = ts.process (0.2 * std::sin (2.0 * kPi * 1000.0 * i / fs));
                if (i > (int) (fs * 0.05)) o.add (y);
            }
            std::printf ("drive %.1f tone %.0f -> out %.3f Vpp\n", drv, tone * 10.0, o.pp());
        }
}

static void cpuBenchmark()
{
    std::printf ("\n== CPU (1 s of audio at 4x oversampling = %.0f samples) ==\n", fs);
    for (int mode : { 0, 1, 2 })
    {
        Preamp5150 pre;
        pre.prepare (fs);
        pre.setMode (mode);
        pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        double acc = 0;
        const auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < (int) fs; ++i)
        {
            const double env = std::exp (-3.0 * std::fmod (i / fs, 0.5));
            acc += pre.process (0.3 * env * (std::sin (2 * kPi * 82.4 * i / fs) + 0.5 * std::sin (2 * kPi * 123.5 * i / fs)));
        }
        const double ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
        std::printf ("mode %d: %.1f ms per second of audio (%.1f%% of one core)  [chk %.3g]\n", mode, ms, ms / 10.0, acc);
    }
}

static void channelLevels()
{
    std::printf ("\n== channel output RMS (gain 6, bass 5, mid 5, treble 6, master 5), guitar-ish 0.2 V chord ==\n");
    for (int mode : { 0, 1, 2 })
    {
        Preamp5150 pre;
        pre.prepare (fs);
        pre.setMode (mode);
        pre.setControls (0.6, 0.5, 0.5, 0.6, 0.5);
        double sq = 0; int n = 0;
        for (int i = 0; i < (int) fs; ++i)
        {
            const double t = i / fs, env = std::exp (-3.0 * std::fmod (t, 0.5));
            const double x = 0.2 * env * (std::sin (2 * kPi * 82.4 * t) + std::sin (2 * kPi * 123.5 * t) + std::sin (2 * kPi * 164.8 * t)) / 3.0;
            const double y = pre.process (x);
            if (i > (int) (fs * 0.1)) { sq += y * y; ++n; }
        }
        std::printf ("mode %d: %.3f V rms\n", mode, std::sqrt (sq / n));
    }
}

static void gateTests()
{
    constexpr double gfs = 48000.0;

    std::printf ("\n== 3-band gate: open-gate frequency response (should be flat) ==\n");
    for (double f : { 50.0, 150.0, 300.0, 1000.0, 3000.0, 8000.0, 15000.0 })
    {
        MultibandGate g;
        g.prepare (gfs);
        MultibandGate::Settings s;
        s.depth[0] = s.depth[1] = s.depth[2] = 0.0f;
        std::vector<float> a ((size_t) gfs), k ((size_t) gfs, 0.0f);
        for (size_t i = 0; i < a.size(); ++i) a[i] = (float) std::sin (2.0 * kPi * f * i / gfs);
        g.process (a.data(), k.data(), (int) a.size(), s);
        double sq = 0; int cnt = 0;
        for (size_t i = a.size() / 2; i < a.size(); ++i) { sq += (double) a[i] * a[i]; ++cnt; }
        std::printf ("%6.0f Hz: %+.3f dB\n", f, 10.0 * std::log10 (sq / cnt / 0.5)); // RMS vs unit sine
    }

    std::printf ("\n== 3-band gate: decaying note (key -10 dBFS, -30 dB/s), thresh -60 dB ==\n");
    MultibandGate g;
    g.prepare (gfs);
    MultibandGate::Settings s; // defaults: depth 30 / 60 / 100 %
    const int n = (int) (gfs * 3.0);
    std::vector<float> a ((size_t) n), k ((size_t) n);
    unsigned rng = 1;
    for (int i = 0; i < n; ++i)
    {
        const double t = i / gfs;
        k[(size_t) i] = (float) (std::pow (10.0, (-10.0 - 30.0 * t) / 20.0) * std::sin (2.0 * kPi * 110.0 * t));
        rng = rng * 1664525u + 1013904223u;
        a[(size_t) i] = (float) ((rng >> 8) / 8388608.0 - 1.0) * 0.1f; // white "amp hiss"
    }
    double closeTime[3] = { -1, -1, -1 };
    const int chunk = 64;
    for (int i = 0; i < n; i += chunk)
    {
        g.process (a.data() + i, k.data() + i, std::min (chunk, n - i), s);
        for (int b = 0; b < 3; ++b)
            if (closeTime[b] < 0 && g.getBandGain (b) < 0.5f) closeTime[b] = i / gfs;
    }
    const char* names[] = { "low ", "mid ", "high" };
    for (int b = 0; b < 3; ++b)
        std::printf ("%s band falls below -6 dB at %.3f s (final gain %.1f dB)\n", names[b], closeTime[b],
                     20.0 * std::log10 (g.getBandGain (b) + 1e-9));
}

// amplitude (peak) of the speaker voltage for a sine at the master-pot node
static long long gIter = 0, gSamples = 0, gFail = 0;

static double powerAmpPeak (double presence, double resonance, double freq, double amp, double seconds = 0.25)
{
    PowerAmp5150 pa;
    pa.setControls (presence, resonance);
    pa.prepare (fs);
    pa.statSamples = pa.statIterations = pa.statFailures = 0;
    struct Acc { PowerAmp5150& p; ~Acc() { gIter += p.statIterations; gSamples += p.statSamples; gFail += p.statFailures; } } acc { pa };
    Stat o;
    const int n = (int) (fs * seconds), from = n - (int) (fs * 0.05);
    for (int i = 0; i < n; ++i)
    {
        const double y = pa.process (amp * std::sin (2.0 * kPi * freq * i / fs));
        if (i >= from) o.add (y);
    }
    return o.pp() / 2.0;
}

static void powerAmpTests()
{
    std::printf ("\n== power amp: idle operating point ==\n");
    {
        PowerAmp5150 pa;
        pa.setControls (0.0, 0.0); // as spice/5150_power_amp.cir run 1 (VR8 is in the tail's DC path)
        pa.prepare (fs);
        for (int i = 0; i < (int) (fs * 0.2); ++i) pa.process (0.0);
        std::printf ("B+ %.1f V, bias %.2f V, output plate %.1f V, PI cathode %.2f V, PI plates %.1f / %.1f V, V3A plate %.1f V\n",
                     pa.getSupplyVoltage(), pa.getBiasVoltage(), pa.getPlate1(), pa.getPIcathode(),
                     pa.getPIplateA(), pa.getPIplateB(), pa.getV3a());
    }

    // B+ carries the bridge rectifier's 120 Hz ripple (Supply5150): the push-pull stage cancels most of it
    std::printf ("\n== idle hum at the speaker (RED, gain 7, no input), dB re full power (~53 V peak) ==\n");
    for (double master : { 0.3, 0.7 })
    {
        Preamp5150 pre; pre.prepare (fs); pre.setMode (2); pre.setControls (0.7, 0.5, 0.5, 0.5, master);
        PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (fs);
        for (int i = 0; i < (int) (0.5 * fs); ++i) processAmp5150 (pre, pa, 0.0, 1.0);
        double re[4] {}, im[4] {}, tot = 0.0;
        const int n = (int) fs; // 1 s
        for (int i = 0; i < n; ++i)
        {
            const double y = processAmp5150 (pre, pa, 0.0, 1.0);
            tot += y * y;
            for (int h = 0; h < 4; ++h)
            {
                const double ph = 2.0 * kPi * 120.0 * (h + 1) * i / fs;
                re[h] += y * std::cos (ph); im[h] += y * std::sin (ph);
            }
        }
        const double full = 53.0 / std::sqrt (2.0);
        std::printf ("master %.1f: total %6.1f dB", master, 20.0 * std::log10 (std::sqrt (tot / n) / full));
        for (int h = 0; h < 4; ++h)
            std::printf ("   %3.0f Hz %6.1f", 120.0 * (h + 1), 20.0 * std::log10 (2.0 * std::hypot (re[h], im[h]) / n / std::sqrt (2.0) / full));
        std::printf ("\n");
    }

    std::printf ("\n== power amp: small-signal response, 20 mV at master node, speaker Vpk (dB re 1 V) ==\n");
    const double freqs[] = { 50, 100, 200, 500, 1000, 2000, 5000, 10000 };
    std::printf ("%-14s", "pres/res");
    for (double f : freqs) std::printf (" %7.0f", f);
    std::printf ("\n");
    const double settings[][2] = { { 0.0, 0.0 }, { 1.0, 0.0 }, { 0.0, 1.0 }, { 0.5, 0.5 } };
    for (auto& s : settings)
    {
        std::printf ("p%.1f r%.1f     ", s[0], s[1]);
        for (double f : freqs)
            std::printf (" %7.2f", 20.0 * std::log10 (powerAmpPeak (s[0], s[1], f, 0.02)));
        std::printf ("\n");
    }

    std::printf ("\n== power amp: drive sweep at 1 kHz (presence/resonance 5) ==\n");
    for (double a : { 0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 20.0 })
    {
        gIter = gSamples = gFail = 0;
        const double vpk = powerAmpPeak (0.5, 0.5, 1000.0, a, 0.3);
        std::printf ("in %5.2f Vpk -> speaker %6.2f Vpk  (~%5.1f W into 8R)   newton avg %.2f it, %lld failures\n",
                     a, vpk, vpk * vpk / 2.0 / 8.0, (double) gIter / gSamples, gFail);
    }

    std::printf ("\n== power amp CPU ==\n");
    {
        PowerAmp5150 pa;
        pa.setControls (0.5, 0.5);
        pa.prepare (fs);
        double acc = 0;
        const auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < (int) fs; ++i)
            acc += pa.process (1.0 * std::sin (2 * kPi * 110.0 * i / fs));
        const double ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
        std::printf ("%.1f ms per second of audio (%.1f%% of one core) [chk %.3g]\n", ms, ms / 10.0, acc);
    }
    {
        // noise input (like an idle interface) is the worst case for the Newton predictor
        PowerAmp5150 pa;
        pa.setControls (0.5, 0.5);
        pa.prepare (fs);
        unsigned rng = 7;
        double acc = 0;
        const auto t0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < (int) fs; ++i)
        {
            rng = rng * 1664525u + 1013904223u;
            acc += pa.process (0.01 * ((rng >> 8) / 8388608.0 - 1.0));
        }
        const double ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
        std::printf ("noise input: %.1f ms per second of audio (%.1f%% of one core) [chk %.3g]\n", ms, ms / 10.0, acc);
    }
}

static void powerAmpDebug (double amp)
{
    PowerAmp5150 pa;
    pa.setControls (0.5, 0.5);
    pa.prepare (fs);
    int shown = 0;
    for (int i = 0; i < (int) (fs * 0.3) && shown < 6; ++i)
    {
        const double y = pa.process (amp * std::sin (2.0 * kPi * 1000.0 * i / fs));
        if (! pa.lastConverged || std::abs (y) > 80.0)
        {
            std::printf ("t=%.5f out=%.1f B+=%.1f V3A=%.1f | ports:", i / fs, y, pa.getSupplyVoltage(), pa.getV3a());
            for (int k = 0; k < PowerAmp5150::kPorts; ++k) std::printf (" %.2f", pa.getPort (k));
            std::printf (" | p0:");
            for (int k = 0; k < PowerAmp5150::kPorts; ++k) std::printf (" %.1f", pa.lastP0[k]);
            std::printf ("\n");
            ++shown;
            i += 20;
        }
    }
}

// The v0.2 transpose algorithm (blind two-tap crossfade), kept here for comparison
struct LegacyShifter
{
    std::vector<float> buf; int mask = 0, w = 0; float window = 2400, phase = 0, ratio = 1;
    void prepare (double f, int semis)
    {
        window = (float) (f * 0.05); int size = 1; while (size < (int) window * 2 + 16) size <<= 1;
        buf.assign ((size_t) size, 0.0f); mask = size - 1; ratio = std::pow (2.0f, semis / 12.0f);
    }
    float read (float d) const
    {
        float rp = (float) w - d; while (rp < 0) rp += (float) (mask + 1);
        const int i0 = (int) rp; const float fr = rp - i0;
        return buf[(size_t) (i0 & mask)] + fr * (buf[(size_t) ((i0 + 1) & mask)] - buf[(size_t) (i0 & mask)]);
    }
    float process (float x)
    {
        buf[(size_t) w] = x;
        phase += (1.0f - ratio) / window; phase -= std::floor (phase);
        float p2 = phase + 0.5f; if (p2 >= 1.0f) p2 -= 1.0f;
        const float g1 = std::sin ((float) kPi * phase), g2 = std::sin ((float) kPi * p2);
        const float y = g1 * g1 * read (phase * window + 2) + g2 * g2 * read (p2 * window + 2);
        w = (w + 1) & mask; return y;
    }
};

// The softplus^ex tables in the Koren models against the original formulas, over the whole range
// the tubes visit (cutoff to grid conduction). Errors are relative to the local current scale.
// The 6L6GC model at published operating points (single tube, class A1 / push-pull AB1 idle) and the 5150's idle
static void pentodeDatasheet()
{
    struct Point { const char* name; double vp, vg2, vg1, ip, gm, rp; }; // datasheet: mA, mA/V, kOhm (0: not given)
    const Point pts[] = {
        { "A1 250 V / 250 V / -14 V (ds 72 mA, 6.0 mA/V, 22.5k)", 250, 250, -14, 72, 6.0, 22.5 },
        { "A1 350 V / 250 V / -18 V (ds 54 mA, 5.2 mA/V, 33k)",   350, 250, -18, 54, 5.2, 33.0 },
        { "AB1 450 V / 400 V / -37 V idle (ds 58 mA per tube)",   450, 400, -37, 58, 0.0, 0.0 },
    };
    KorenPentode t;
    std::printf ("\n== 6L6GC model vs datasheets: plate current, gm, plate resistance, screen current ==\n");
    for (const auto& p : pts)
    {
        double ip, dvp, dvg;
        t.plate (p.vp, p.vg1, p.vg2, ip, dvp, dvg);
        std::printf ("%-55s ip %6.1f mA  gm %5.2f mA/V  rp %6.1f k  ig2 %5.2f mA", p.name, ip * 1e3, dvg * 1e3, 1e-3 / dvp,
                     t.screen (p.vp, p.vg1, p.vg2) * 1e3);
        if (p.gm > 0) std::printf ("   (gm %+.0f %%, rp %+.0f %%)", (dvg * 1e3 / p.gm - 1) * 100, (1.0 / dvp / (p.rp * 1e3) - 1) * 100);
        std::printf ("\n");
    }
    // push-pull class AB1 pair (450 V / 400 V / -37 V, 5600 ohm plate to plate, ideal transformer), zero and full
    // signal (70 V peak grid to grid), averaged over a cycle: v1 = B+ - x, v2 = B+ + x, x = (i1 - i2) Raa / 4
    for (double pk : { 0.0, 70.0 })
    {
        const double bp = 450.0, eg2 = 400.0, eg1 = -37.0, raa = 5600.0;
        constexpr int n = 2000;
        auto ipOf = [&] (double vp, double vg) { double i, a, b; t.plate (vp, vg, eg2, i, a, b); return i; };
        double sIp = 0, sIg2 = 0, sP = 0;
        for (int k = 0; k < n; ++k)
        {
            const double s = std::sin (2.0 * kPi * k / n), g1 = eg1 + 0.5 * pk * s, g2 = eg1 - 0.5 * pk * s;
            double lo = -bp, hi = bp;
            for (int it = 0; it < 60; ++it)
            {
                const double x = 0.5 * (lo + hi);
                (x - raa / 4 * (ipOf (bp - x, g1) - ipOf (bp + x, g2)) > 0 ? hi : lo) = x;
            }
            const double x = 0.5 * (lo + hi);
            sIp += ipOf (bp - x, g1) + ipOf (bp + x, g2);
            sIg2 += t.screen (bp - x, g1, eg2) + t.screen (bp + x, g2, eg2);
            sP += 4.0 * x * x / raa;
        }
        std::printf ("AB1 push-pull pair, %2.0f V peak grid to grid: ip %6.1f mA  ig2 %5.2f mA  %4.1f W   (ds %s)\n", pk,
                     sIp / n * 1e3, sIg2 / n * 1e3, sP / n, pk > 0.0 ? "210 mA, 22 mA, 55 W" : "116 mA, 5.6 mA");
    }
    PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (fs);
    double ip, dvp, dvg;
    t.plate (pa.getSupplyVoltage(), pa.getBiasVoltage(), pa.getScreenVoltage(), ip, dvp, dvg);
    std::printf ("5150 idle: plate %.0f V, screen %.0f V, bias %.2f V -> ip %.1f mA  gm %.2f mA/V  rp %.1f k\n",
                 pa.getSupplyVoltage(), pa.getScreenVoltage(), pa.getBiasVoltage(), ip * 1e3, dvg * 1e3, 1e-3 / dvp);
}

static void tubeTableAccuracy()
{
    std::printf ("\n== Koren tube models: tabulated vs exact formula (max relative error) ==\n");
    {
        KorenTriode t;
        double eI = 0, eP = 0, eG = 0;
        for (double vp = 0.5; vp < 500.0; vp *= 1.013)
            for (double vg = -40.0; vg < 3.0; vg += 0.0173)
            {
                double i0, p0, g0, i1, p1, g1;
                t.evalExact (vp, vg, i0, p0, g0);
                t.eval (vp, vg, i1, p1, g1);
                const double sI = std::max (std::abs (i0), 1e-9), sD = std::max ({ std::abs (p0), std::abs (g0), 1e-9 });
                eI = std::max (eI, std::abs (i1 - i0) / sI);
                eP = std::max (eP, std::abs (p1 - p0) / sD);
                eG = std::max (eG, std::abs (g1 - g0) / sD);
            }
        std::printf ("12AX7: ip %.1e  dip/dvpk %.1e  dip/dvgk %.1e  (floor 1 nA / 1 nS)\n", eI, eP, eG);
    }
    {
        KorenPentode t;
        double eI = 0, eP = 0, eG = 0;
        for (double vs : { 380.0, 450.0, 495.0 })
            for (double vp = -5.0; vp < 900.0; vp += 1.37)
                for (double vg = -200.0; vg < 20.0; vg += 0.113)
                {
                    double i0, p0, g0, i1, p1, g1;
                    t.plateExact (vp, vg, vs, i0, p0, g0);
                    t.plate (vp, vg, vs, i1, p1, g1);
                    const double sI = std::max (std::abs (i0), 1e-9), sD = std::max ({ std::abs (p0), std::abs (g0), 1e-9 });
                    eI = std::max (eI, std::abs (i1 - i0) / sI);
                    eP = std::max (eP, std::abs (p1 - p0) / sD);
                    eG = std::max (eG, std::abs (g1 - g0) / sD);
                }
        std::printf ("6L6GC: ip %.1e  dip/dvpk %.1e  dip/dvg1 %.1e  (floor 1 nA / 1 nS)\n", eI, eP, eG);
    }
}

// Per-module CPU on a guitar-like DI (palm-muted chugs + idle hiss between notes), at the rates the
// plugin runs them (non-linear amp at 4x = 192 kHz, the rest at 48 kHz). Reports the average load and
// the worst 64-sample block: on dedicated hardware the per-block deadline is what matters.
static std::vector<double> guitarDI (double rate, double seconds)
{
    std::vector<double> x ((size_t) (rate * seconds));
    unsigned rng = 12345;
    const double f[] = { 41.2, 61.7, 82.4 }; // drop-D-ish power chord
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = i / rate;
        const double ph = std::fmod (t, 1.0);
        double v = 0;
        if (ph < 0.75) // 6 chugs, then 0.25 s of idle hiss
        {
            const double env = std::exp (-std::fmod (ph, 0.125) * 30.0);
            for (double fr : f)
                for (int h = 1; h <= 6; ++h)
                    v += env * 0.25 / h * std::sin (2 * kPi * fr * h * t + h);
        }
        rng = rng * 1664525u + 1013904223u;
        v += 0.0005 * ((rng >> 8) / 8388608.0 - 1.0); // ~-66 dBFS
        x[i] = v;
    }
    return x;
}

template <typename Fn>
static void timeModule (const char* name, double rate, int block, int n, Fn&& run)
{
    using clk = std::chrono::high_resolution_clock;
    double total = 0, worst = 0;
    for (int start = 0; start < n; start += block)
    {
        const int len = std::min (block, n - start);
        const auto t0 = clk::now();
        run (start, len);
        const double us = std::chrono::duration<double, std::micro> (clk::now() - t0).count();
        total += us; worst = std::max (worst, us);
    }
    std::printf ("%-36s avg %6.2f%%   worst block %6.1f%%\n", name,
                 100.0 * total / (1e6 * n / rate), 100.0 * worst / (1e6 * block / rate));
}

static void cpuModules()
{
    constexpr double base = 48000.0, os = 4 * base, secs = 4.0;
    const auto diOs = guitarDI (os, secs);
    const auto di = guitarDI (base, secs);
    const int nOs = (int) diOs.size(), n = (int) di.size();
    volatile double sink = 0;
    auto loop = [&] (auto& fn) { return [&] (int s, int l) { double a = 0; for (int i = s; i < s + l; ++i) a += fn (i); sink = sink + a; }; };

    std::printf ("\n== CPU per module (%% of one core; 48 kHz host, 64-sample buffer = 256 samples at 4x) ==\n");

    {
        PrecisionDrive pd; pd.prepare (os); pd.setControls (0.5, 0.5, 3, 0.7);
        auto fn = [&] (int i) { return pd.process (diOs[(size_t) i]); };
        timeModule ("Precision Drive (4x)", os, 256, nOs, loop (fn));
    }
    {
        TubeScreamer9 ts; ts.prepare (os); ts.setControls (0.5, 0.5, 0.7);
        auto fn = [&] (int i) { return ts.process (diOs[(size_t) i]); };
        timeModule ("TS9 (4x)", os, 256, nOs, loop (fn));
    }

    const char* names[] = { "Preamp CLEAN gain 0.7 (4x)", "Preamp BLUE  gain 0.7 (4x)", "Preamp RED   gain 0.7 (4x)" }; // Preamp5150::Mode order
    for (int mode : { 0, 1, 2 })
    {
        Preamp5150 pre; pre.prepare (os); pre.setMode (mode); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        auto fn = [&] (int i) { return pre.process (diOs[(size_t) i]); };
        timeModule (names[mode], os, 256, nOs, loop (fn));
    }
    {
        Preamp5150 pre; pre.prepare (os); pre.setMode (2); pre.setControls (1.0, 0.5, 0.5, 0.6, 0.5);
        auto fn = [&] (int i) { return pre.process (diOs[(size_t) i]); };
        timeModule ("Preamp RED   gain 1.0 (4x)", os, 256, nOs, loop (fn));
    }
    {
        // feed the power amp the real preamp output so its operating point is realistic
        Preamp5150 pre; pre.prepare (os); pre.setMode (2); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        std::vector<double> pv ((size_t) nOs);
        for (int i = 0; i < nOs; ++i) pv[(size_t) i] = pre.process (diOs[(size_t) i]);
        PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (os);
        auto fn = [&] (int i) { return pa.process (pv[(size_t) i]); };
        timeModule ("Power amp (4x)", os, 256, nOs, loop (fn));
    }
    {
        PrecisionDrive pd; pd.prepare (os); pd.setControls (0.5, 0.5, 3, 0.7);
        Preamp5150 pre; pre.prepare (os); pre.setMode (2); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (os);
        auto fn = [&] (int i) { return processAmp5150 (pre, pa, pd.process (diOs[(size_t) i]), 1.0); };
        timeModule ("FULL amp: drive+RED+power (4x)", os, 256, nOs, loop (fn));
    }
    {
        Preamp5150 pre; pre.prepare (os); pre.setMode (0); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (os);
        auto fn = [&] (int i) { return processAmp5150 (pre, pa, diOs[(size_t) i], 25.0); }; // AmpSim kChannelPost
        timeModule ("FULL amp: CLEAN+power (4x)", os, 256, nOs, loop (fn));
    }

    std::printf ("\n-- Newton evaluations per sample (fallback %% = fast path failed, restarted with 30 its) --\n");
    for (int mode : { 0, 1, 2 })
    {
        Preamp5150 pre; pre.prepare (os); pre.setMode (mode); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        long long *ns, *ni, *nf; // the channel's coupled circuit
        if (mode == Preamp5150::Clean) { ns = &pre.clean.net.statSamples; ni = &pre.clean.net.statIterations; nf = &pre.clean.net.statFailures; }
        else
        {
            auto& p = mode == Preamp5150::Red ? pre.lead : pre.blue;
            ns = &p.net.statSamples; ni = &p.net.statIterations; nf = &p.net.statFailures;
        }
        *ns = *ni = *nf = 0;
        double a = 0;
        for (int i = 0; i < nOs; ++i) a += pre.process (diOs[(size_t) i]);
        sink = sink + a;
        std::printf ("%-6s  %s %.2f (%lld failures)\n", names[mode] + 7, mode == Preamp5150::Clean ? "V1A-V1B" : "V1A-V5A",
                     (double) *ni / std::max (*ns, 1LL), *nf);
    }
    {
        Preamp5150 pre; pre.prepare (os); pre.setMode (2); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
        PowerAmp5150 pa; pa.setControls (0.5, 0.5); pa.prepare (os);
        pa.statSamples = pa.statIterations = pa.statFailures = 0;
        double a = 0;
        for (int i = 0; i < nOs; ++i) a += processAmp5150 (pre, pa, diOs[(size_t) i], 1.0);
        sink = sink + a;
        std::printf ("Power amp DK: %.2f evals/sample, %lld failures\n", (double) pa.statIterations / pa.statSamples, pa.statFailures);
    }

    std::vector<float> buf ((size_t) n), key ((size_t) n);
    auto reload = [&] { for (int i = 0; i < n; ++i) buf[(size_t) i] = key[(size_t) i] = (float) di[(size_t) i]; };
    {
        reload(); MultibandGate g; g.prepare (base); MultibandGate::Settings st;
        timeModule ("3-band gate", base, 64, n, [&] (int s, int l) { g.process (buf.data() + s, key.data() + s, l, st); });
    }
    for (int mode : { (int) PitchShifter::Live, (int) PitchShifter::HQ })
    {
        reload(); PitchShifter ps; ps.prepare (base); ps.setParams (-2, mode);
        timeModule (mode == PitchShifter::Live ? "Transpose LIVE (-2 st)" : "Transpose HQ (-2 st)", base, 64, n,
                    [&] (int s, int l) { ps.process (buf.data() + s, l); });
    }
}

// ---- Hardware target: 8-sample blocks (dsp_test block8) ----
// Cost of each 8-sample host block at 48 kHz (deadline 166.7 us), per module, as the plugin runs them: the amp at 8x
// (64 samples a block) with AmpSim's knob smoothing, every-4th-step matrix rebuilds and channel crossfade (AmpCore
// mirrors AmpSim::process without JUCE's oversampling filters, DC block and volume), the 1176 at 2x (16 samples).
// Each case is rendered kRuns times from the same freshly prepared state and the fastest run is kept per block: the
// model is deterministic, so this drops OS interruptions but keeps its own spikes (matrix rebuilds, Newton fallbacks).
// Every module gets the same guitar DI generated at its own rate (no resampling filters between them).
namespace block8
{
constexpr double kBase = 48000.0, kOs = 8 * kBase, kCompRate = 2 * kBase, kSeconds = 4.0;
constexpr int kBlock = 8, kOsBlock = 64, kCompBlock = 16, kRuns = 5;
constexpr double kDeadlineUs = 1e6 * kBlock / kBase;
constexpr double kChannelPost[3] = { 54.0, 1.0, 1.0 }; // AmpSim.h
volatile double sink = 0;

// AmpSim::process per host block (stepKnobs / applyKnobs as there)
struct AmpCore
{
    static constexpr int kKnobs = 10, kControlInterval = 32, kHeavyEvery = 4;
    static constexpr int kHeavy[3] = { 0, 5, 6 }; // Gain, Presence, Resonance

    PrecisionDrive drive;
    TubeScreamer9 ts9;
    Preamp5150 preamp;
    PowerAmp5150 power;
    double knob[kKnobs] {}, knobTarget[kKnobs] {}, knobFrom[kKnobs] {}, heavyApplied[3] {}, knobCoef = 1.0;
    double modelMix = 0.0, modelStep = 1e-4;
    int heavyPhase = 0, controlPhase = 0, boostAttack = 3;
    bool knobsSet = false, firstBlock = true, lastBoostOn = false;

    void prepare()
    {
        drive.prepare (kOs);
        ts9.prepare (kOs);
        modelStep = 1.0 / (0.02 * kOs);
        preamp.setChannelTrims (kChannelPost[0], kChannelPost[1], kChannelPost[2]);
        preamp.prepare (kOs);
        power.prepare (kOs);
        auto settle = [this] (double seconds) { for (int i = 0; i < (int) (kOs * seconds); ++i) processAmp5150 (preamp, power, 0.0, 1.0); };
        power.beginSupplySettle();
        for (int pass = 0; pass < 3; ++pass)
        {
            settle (0.1);
            power.resetSupplyAverage();
            settle (0.1);
            power.applySupplyDC();
        }
        settle (0.15);
        power.endSupplySettle();
        knobCoef = 1.0 - std::exp (-kControlInterval / (20e-3 * kOs));
    }

    // x: kOsBlock samples at 8x; target: Gain, Bass, Mid, Treble, Master, Presence, Resonance, boost Drive, Bright, Level
    double block (const double* x, const double* target, int channel, bool boostOn, int boostModel)
    {
        if (firstBlock) preamp.setModeNow (channel);
        else preamp.setMode (channel);
        firstBlock = false;
        if (boostOn && ! lastBoostOn)
        {
            drive.setControls (knob[7], knob[8], boostAttack, knob[9]);
            ts9.setControls (knob[7], knob[8], knob[9]);
        }
        lastBoostOn = boostOn;
        const double modelTarget = boostModel == 1 ? 1.0 : 0.0;
        std::copy (knobTarget, knobTarget + kKnobs, knobFrom);
        std::copy (target, target + kKnobs, knobTarget);
        if (! knobsSet)
        {
            std::copy (target, target + kKnobs, knob);
            std::copy (target, target + kKnobs, knobFrom);
            knobsSet = true;
            applyKnobs (boostOn, true);
        }
        auto pedals = [&] (double v)
        {
            if (! boostOn) return v;
            if (modelMix != modelTarget)
                modelMix = modelTarget > modelMix ? std::min (modelTarget, modelMix + modelStep) : std::max (modelTarget, modelMix - modelStep);
            const double w = 0.5 - 0.5 * std::cos (kPi * modelMix);
            return (1.0 - w) * drive.process (v) + w * ts9.process (v);
        };
        // two-core split (Preamp5150::runAhead): the pedals first (they do not depend on the amp), then the circuits
        // fading out, solved ahead on last block's rails ("core B", timed on its own), then the rest per sample.
        // Knobs must be still: the circuits solved ahead would miss control updates within the block.
        double vpre[kOsBlock];
        aheadUs = 0.0;
        if (twoCore)
            for (int i = 0; i < kOsBlock; ++i) vpre[i] = pedals (x[i]);
        double acc = 0;
        for (int i = 0; i < kOsBlock; ++i)
        {
            if (twoCore && i % aheadChunk == 0 && preamp.fading()) // core B: the next aheadChunk samples (the lag)
            {
                const auto t0 = std::chrono::steady_clock::now();
                preamp.runAhead (vpre + i, aheadChunk);
                aheadUs += std::chrono::duration<double, std::micro> (std::chrono::steady_clock::now() - t0).count();
            }
            if (++controlPhase >= kControlInterval)
            {
                controlPhase = 0;
                stepKnobs (boostOn, (double) (i + 1) / kOsBlock);
            }
            const double v = twoCore ? vpre[i] : pedals (x[i]);
            out[i] = processAmp5150 (preamp, power, v, 1.0);
            acc += out[i];
        }
        return acc;
    }

    bool twoCore = false;  // set before the first block (with preamp.enableRunAhead)
    int aheadChunk = kOsBlock; // samples solved ahead at a time = the rail lag (a divisor of kOsBlock)
    double aheadUs = 0.0;  // the last block's run-ahead ("core B") time
    double out[kOsBlock] {}; // the last block's speaker volts

    void stepKnobs (bool boostOn, double frac)
    {
        bool moving = false;
        for (int k = 0; k < kKnobs; ++k)
        {
            const double t = knobFrom[k] + frac * (knobTarget[k] - knobFrom[k]), d = t - knob[k];
            if (d == 0.0) continue;
            knob[k] = std::abs (d) < 1e-5 ? t : knob[k] + knobCoef * d;
            moving = true;
        }
        const bool heavyStep = ++heavyPhase >= kHeavyEvery;
        if (heavyStep) heavyPhase = 0;
        bool heavyPending = false;
        for (int k : kHeavy) heavyPending = heavyPending || heavyApplied[k == 0 ? 0 : k - 4] != knob[k];
        if (moving || (heavyStep && heavyPending))
            applyKnobs (boostOn, heavyStep);
    }

    void applyKnobs (bool boostOn, bool heavy)
    {
        if (heavy)
            for (int k : kHeavy) heavyApplied[k == 0 ? 0 : k - 4] = knob[k];
        preamp.setControls (heavyApplied[0], knob[1], knob[2], knob[3], knob[4]);
        power.setControls (heavyApplied[1], heavyApplied[2]);
        if (boostOn)
        {
            drive.setControls (knob[7], knob[8], boostAttack, knob[9]);
            ts9.setControls (knob[7], knob[8], knob[9]);
        }
    }
};

// automation of one amp case at time t (s): knob targets, channel, drive on / model
struct AmpState
{
    double target[10] = { 0.6, 0.5, 0.5, 0.6, 0.3, 0.5, 0.5, 0.0, 0.5, 0.7 }; // AmpSim::Settings defaults
    int channel = Preamp5150::Red;
    bool boostOn = false;
    int boostModel = 0;
};
using Automation = std::function<void (double t, AmpState&)>;

// 0..1..0 over period p (s)
inline double tri (double t, double p) { const double f = std::fmod (t / p, 1.0); return 1.0 - std::abs (2.0 * f - 1.0); }

// drive on, every knob turning at its own rate (Master kept moderate). Spelled out: the same as a loop over k in a
// lambda crashes MSVC 14.44's optimizer (C1001).
void everyKnob (double t, AmpState& s)
{
    s.boostOn = true;
    static constexpr double period[10] = { 0.5, 0.63, 0.76, 0.89, 0.8, 1.15, 1.28, 1.41, 1.54, 1.67 };
    double* k = s.target;
    k[0] = tri (t, period[0]); k[1] = tri (t, period[1]); k[2] = tri (t, period[2]); k[3] = tri (t, period[3]);
    k[4] = 0.1 + 0.5 * tri (t, period[4]);
    k[5] = tri (t, period[5]); k[6] = tri (t, period[6]); k[7] = tri (t, period[7]); k[8] = tri (t, period[8]);
    k[9] = tri (t, period[9]);
}

using Step = std::function<void (int)>; // processes host block b

// the first 10 ms (60 blocks: the first block resets the channel and applies every knob, caches are cold) are left
// out of the statistics and the first block is shown on its own
constexpr int kSkipBlocks = 60;

void report (const char* name, const std::vector<double>& all, double rawMax)
{
    const std::vector<double> best (all.begin() + kSkipBlocks, all.end());
    auto s = best;
    std::sort (s.begin(), s.end());
    double sum = 0;
    for (double v : s) sum += v;
    const auto at = [&] (double q) { return s[(size_t) std::min ((double) s.size() - 1, q * (double) s.size())]; };
    const auto worst = std::max_element (best.begin(), best.end());
    int over = 0;
    for (double v : s) over += v > kDeadlineUs;
    std::printf ("%-38s %6.1f %6.1f %6.1f %7.1f %5.0f%%  %6.3f s %7.1f %6d %7.1f\n", name, sum / (double) s.size(), at (0.99),
                 at (0.999), *worst, 100.0 * *worst / kDeadlineUs, (double) (worst - best.begin() + kSkipBlocks) * kBlock / kBase, rawMax,
                 over, all[0]);
}

void runCase (const char* name, int numBlocks, const std::function<Step()>& make)
{
    using clk = std::chrono::steady_clock;
    std::vector<double> best ((size_t) numBlocks, 1e30);
    double rawMax = 0;
    for (int r = 0; r < kRuns; ++r)
    {
        Step step = make(); // freshly prepared state, outside the timing
        for (int b = 0; b < numBlocks; ++b)
        {
            const auto t0 = clk::now();
            step (b);
            const double us = std::chrono::duration<double, std::micro> (clk::now() - t0).count();
            best[(size_t) b] = std::min (best[(size_t) b], us);
            rawMax = std::max (rawMax, us);
        }
    }
    report (name, best, rawMax);
}

// (plain functions capturing pointers: nested lambdas capturing references crash MSVC 14.44 here)
void runAmpCase (const char* name, int numBlocks, const double* x, Automation automate)
{
    runCase (name, numBlocks, [x, automate] {
        auto amp = std::make_shared<AmpCore>();
        amp->prepare();
        return Step ([amp, x, automate] (int b) {
            AmpState st;
            automate ((double) b * kBlock / kBase, st);
            sink = sink + amp->block (x + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
        });
    });
}

using CompControls = std::function<Urei1176::Controls (double t)>;
void runCompCase (const char* name, int numBlocks, const double* x, CompControls controls, bool everyBlock)
{
    runCase (name, numBlocks, [x, controls, everyBlock] {
        auto m = std::make_shared<Urei1176>(); m->setControls (controls (0.0)); m->prepare (kCompRate);
        return Step ([m, x, controls, everyBlock] (int b) {
            if (everyBlock) m->setControls (controls ((double) b * kBlock / kBase)); // Comp1176: when a knob changed
            double a = 0;
            for (int i = 0; i < kCompBlock; ++i) a += m->processSample (x[(size_t) (b * kCompBlock + i)]);
            sink = sink + a;
        });
    });
}

// One channel switch in detail (dsp_test switch8): the full amp's cost per block around the switch, against the same
// run without it (fastest of kRuns per block both), and the setMode() call on its own (the incoming circuit takes
// over the outgoing one's state). The fade runs 20 ms = 120 blocks.
void switchProfile()
{
    using clk = std::chrono::steady_clock;
    const double secs = 1.5;
    const auto diOs = guitarDI (kOs, secs);
    const int numBlocks = (int) (kBase * secs) / kBlock, fadeBlocks = (int) (0.02 * kBase) / kBlock;
    struct Case { const char* name; double at; int from, to; bool gainMoved; bool prebuild = false; };
    const Case cases[] = {
        { "RED -> BLUE at a chug's attack",             1.0,   Preamp5150::Red,   Preamp5150::Blue,  false },
        { "RED -> BLUE 60 ms into a chug",              1.06,  Preamp5150::Red,   Preamp5150::Blue,  false },
        { "RED -> BLUE in the idle hiss",               0.8,   Preamp5150::Red,   Preamp5150::Blue,  false },
        { "RED -> CLEAN at a chug's attack",            1.0,   Preamp5150::Red,   Preamp5150::Clean, false },
        { "CLEAN -> RED at a chug's attack",            1.0,   Preamp5150::Clean, Preamp5150::Red,   false },
        { "RED -> BLUE at attack, Gain moved on RED",   1.0,   Preamp5150::Red,   Preamp5150::Blue,  true },
        // the incoming circuit's deferred matrix rebuild done 10 blocks ahead (DKCircuit::ensureBuilt)
        { "  same, BLUE's matrices built ahead",         1.0,   Preamp5150::Red,   Preamp5150::Blue,  true,  true },
        { "RED -> BLUE at attack, built ahead",          1.0,   Preamp5150::Red,   Preamp5150::Blue,  false, true },
        { "RED -> BLUE in idle hiss, built ahead",       0.8,   Preamp5150::Red,   Preamp5150::Blue,  false, true },
        { "CLEAN -> RED at attack, built ahead",         1.0,   Preamp5150::Clean, Preamp5150::Red,   false, true },
    };
    std::printf ("\n== One channel switch, full amp at 8x, 8-sample blocks (us; deadline %.1f us; fade %d blocks) ==\n",
                 kDeadlineUs, fadeBlocks);
    std::printf ("%-42s %8s %8s %8s %8s %8s %8s %8s\n", "", "setMode", "block 0", "+extra", "fade avg", "+extra",
                 "fade max", "base max");
    for (const auto& c : cases)
    {
        const int sb = (int) (c.at * kBase) / kBlock;
        auto incoming = [&] (AmpCore& a) -> DKCircuit& {
            return c.to == Preamp5150::Red ? a.preamp.lead.net.dk : c.to == Preamp5150::Blue ? a.preamp.blue.net.dk : a.preamp.clean.net.dk; };
        double tBuild = 0;
        auto render = [&] (bool doSwitch, std::vector<double>& t, double& tSet)
        {
            t.assign ((size_t) numBlocks, 1e30);
            tSet = 1e30;
            for (int r = 0; r < kRuns; ++r)
            {
                AmpCore amp;
                amp.prepare();
                for (int b = 0; b < numBlocks; ++b)
                {
                    AmpState st;
                    st.channel = doSwitch && b >= sb ? c.to : c.from;
                    if (c.gainMoved && b * kBlock >= (int) (0.5 * kBase)) st.target[0] = 0.8; // smoothed over ~20 ms
                    if (doSwitch && c.prebuild && b == sb - 10)
                    {
                        const auto tb = clk::now();
                        incoming (amp).ensureBuilt();
                        tBuild = std::chrono::duration<double, std::micro> (clk::now() - tb).count();
                    }
                    const auto t0 = clk::now();
                    if (b > 0) amp.preamp.setMode (st.channel); // block() calls it again: a no-op then
                    const auto t1 = clk::now();
                    sink = sink + amp.block (diOs.data() + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
                    const auto t2 = clk::now();
                    t[(size_t) b] = std::min (t[(size_t) b], std::chrono::duration<double, std::micro> (t2 - t0).count());
                    if (doSwitch && b == sb) tSet = std::min (tSet, std::chrono::duration<double, std::micro> (t1 - t0).count());
                }
            }
        };
        std::vector<double> sw, base;
        double tSet = 0, unused = 0;
        render (true, sw, tSet);
        render (false, base, unused);
        double fadeSum = 0, baseSum = 0, fadeMax = 0, baseMax = 0;
        for (int b = sb; b < sb + fadeBlocks; ++b)
        {
            fadeSum += sw[(size_t) b]; baseSum += base[(size_t) b];
            fadeMax = std::max (fadeMax, sw[(size_t) b]); baseMax = std::max (baseMax, base[(size_t) b]);
        }
        std::printf ("%-42s %8.2f %8.1f %+8.1f %8.1f %+8.1f %8.1f %8.1f", c.name, tSet, sw[(size_t) sb],
                     sw[(size_t) sb] - base[(size_t) sb], fadeSum / fadeBlocks, (fadeSum - baseSum) / fadeBlocks, fadeMax, baseMax);
        if (c.prebuild) std::printf ("   (rebuild ahead: %.1f us)", tBuild);
        std::printf ("\n");
        if (&c == &cases[0])
        {
            std::printf ("   per block from the switch (switched / without):");
            for (int b = sb - 2; b < sb + 8; ++b) std::printf (" %.0f/%.0f", sw[(size_t) b], base[(size_t) b]);
            std::printf (" ... last fade block %.0f/%.0f, after %.0f/%.0f\n", sw[(size_t) (sb + fadeBlocks - 1)],
                         base[(size_t) (sb + fadeBlocks - 1)], sw[(size_t) (sb + fadeBlocks + 1)], base[(size_t) (sb + fadeBlocks + 1)]);
        }
    }
    std::printf ("block 0 = the block with the switch (+extra: vs the same block without it); fade avg / max over the 20 ms "
                 "fade; base max = the same blocks without the switch\n");

    // Newton work of the incoming (BLUE) and outgoing (RED) circuits per block after RED -> BLUE
    for (double at : { 1.0, 0.8 })
    {
        const int sb = (int) (at * kBase) / kBlock;
        AmpCore amp;
        amp.prepare();
        auto& in = amp.preamp.blue.net;
        auto& out = amp.preamp.lead.net;
        std::printf ("\nRED -> BLUE at %.2f s, Newton iterations per sample in block (BLUE incoming / RED outgoing; "
                     "* = fallback restarts):\n  ", at);
        double inLate = 0, outLate = 0;
        int lateBlocks = 0;
        for (int b = 0; b < sb + fadeBlocks; ++b)
        {
            AmpState st;
            st.channel = b >= sb ? Preamp5150::Blue : Preamp5150::Red;
            const long long i0 = in.statIterations, s0 = in.statSamples, f0 = in.statRetries + in.statFailures;
            const long long o0 = out.statIterations, os0 = out.statSamples;
            sink = sink + amp.block (diOs.data() + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
            if (b < sb) continue;
            const double itIn = (double) (in.statIterations - i0) / std::max (1LL, in.statSamples - s0);
            const double itOut = (double) (out.statIterations - o0) / std::max (1LL, out.statSamples - os0);
            const long long fb = in.statRetries + in.statFailures - f0;
            if (b < sb + 6) std::printf ("b%d %.2f/%.2f%s  ", b - sb, itIn, itOut, fb > 0 ? "*" : "");
            else if (b >= sb + 10) { inLate += itIn; outLate += itOut; ++lateBlocks; }
        }
        std::printf ("... blocks 10-119 avg %.2f/%.2f\n", inLate / lateBlocks, outLate / lateBlocks);
    }
}

// The two-core split of a switch (dsp_test twocore): the fading-out circuit solved a block ahead on the rails of the
// block before (Preamp5150::runAhead) against the exact per-sample coupling. Sound: null test of the speaker volts
// (8x, so ultrasonics included), with the solver's own tolerance (reference precision vs normal) as the floor. Cost
// per fade block: core A = incoming circuit + tone stack + power amp, core B = the circuit(s) fading out; two cores
// ~ max (A + one sample of B, B) (core A waits for B's first sample, then B stays ahead), sync overhead not included.
void twoCoreTest()
{
    using clk = std::chrono::steady_clock;
    const auto diOs = guitarDI (kOs, kSeconds);
    const int numBlocks = (int) (kBase * kSeconds) / kBlock;
    struct Case { const char* name; Automation automate; };
    const Case cases[] = {
        { "RED <-> BLUE every 0.25 s", [] (double t, AmpState& s) { s.channel = (int) (t / 0.25) % 2 == 0 ? Preamp5150::Red : Preamp5150::Blue; } },
        { "RED <-> BLUE, Gain 10", [] (double t, AmpState& s) { s.target[0] = 1.0; s.channel = (int) (t / 0.25) % 2 == 0 ? Preamp5150::Red : Preamp5150::Blue; } },
        { "RED <-> BLUE + Precision Drive", [] (double t, AmpState& s) { s.boostOn = true; s.target[7] = 0.6;
                                                                         s.channel = (int) (t / 0.25) % 2 == 0 ? Preamp5150::Red : Preamp5150::Blue; } },
        { "RED -> BLUE -> CLEAN cycle", [] (double t, AmpState& s) {
              const int c[3] = { Preamp5150::Red, Preamp5150::Blue, Preamp5150::Clean }; s.channel = c[(int) (t / 0.25) % 3]; } },
    };
    auto db = [] (double r) { return 20.0 * std::log10 (std::max (r, 1e-30)); };

    std::printf ("\n== Two-core channel switch: fading-out circuit one block (8 samples) ahead on the rails of the block before ==\n");
    std::printf ("sound: residual vs the exact render, dB re the signal (whole render RMS / worst 20 ms window RMS / peak); "
                 "floor = solver tolerance (reference precision vs normal)\n");
    std::printf ("%-32s %26s %26s\n", "", "run-ahead residual", "solver floor");
    struct Cost { double serialAvg, serialMax, aMax, bMax, twoAvg, twoMax; int serialOver, twoOver, fadeBlocks; };
    std::vector<Cost> costs;
    for (const auto& c : cases)
    {
        // render: 0 exact, 1 run-ahead, 2 exact at reference precision; per block times when timed
        auto render = [&] (int kind, std::vector<double>& y, std::vector<double>* total, std::vector<double>* ahead,
                           std::vector<char>* fade)
        {
            AmpCore amp;
            if (kind == 2) amp.preamp.setReferencePrecision (true);
            amp.prepare();
            if (kind == 1) { amp.twoCore = true; amp.preamp.enableRunAhead (true); }
            y.assign ((size_t) numBlocks * kOsBlock, 0.0);
            for (int b = 0; b < numBlocks; ++b)
            {
                AmpState st;
                c.automate ((double) b * kBlock / kBase, st);
                const auto t0 = clk::now();
                if (b > 0) amp.preamp.setMode (st.channel); // as block() does: so fading() is known before it
                const bool f = amp.preamp.fading();
                sink = sink + amp.block (diOs.data() + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
                const double us = std::chrono::duration<double, std::micro> (clk::now() - t0).count();
                std::copy (amp.out, amp.out + kOsBlock, y.begin() + (ptrdiff_t) b * kOsBlock);
                if (total) (*total)[(size_t) b] = std::min ((*total)[(size_t) b], us);
                if (ahead) (*ahead)[(size_t) b] = std::min ((*ahead)[(size_t) b], amp.aheadUs);
                if (fade) (*fade)[(size_t) b] = f;
            }
        };
        std::vector<double> exact, lag, ref;
        std::vector<double> tExact ((size_t) numBlocks, 1e30), tLag ((size_t) numBlocks, 1e30), tAhead ((size_t) numBlocks, 1e30);
        std::vector<char> fade ((size_t) numBlocks, 0);
        for (int r = 0; r < kRuns; ++r)
        {
            render (0, exact, &tExact, nullptr, &fade);
            render (1, lag, &tLag, &tAhead, nullptr);
        }
        render (2, ref, nullptr, nullptr, nullptr);

        auto nullStats = [&] (const std::vector<double>& a, const std::vector<double>& b, double& whole, double& win, double& peak)
        {
            double s2 = 0, r2 = 0, sp = 0, rp = 0, worstWin = 0;
            const size_t winLen = (size_t) (0.02 * kOs);
            double w2 = 0;
            for (size_t i = 0; i < a.size(); ++i)
            {
                const double d = a[i] - b[i];
                s2 += a[i] * a[i]; r2 += d * d;
                sp = std::max (sp, std::abs (a[i])); rp = std::max (rp, std::abs (d));
                w2 += d * d;
                if ((i + 1) % winLen == 0) { worstWin = std::max (worstWin, w2 / (double) winLen); w2 = 0; }
            }
            const double sigRms = std::sqrt (s2 / (double) a.size());
            whole = db (std::sqrt (r2 / (double) a.size()) / sigRms);
            win = db (std::sqrt (worstWin) / sigRms);
            peak = db (rp / sp);
        };
        double lw, lwin, lp, fw, fwin, fp;
        nullStats (exact, lag, lw, lwin, lp);
        nullStats (exact, ref, fw, fwin, fp);
        std::printf ("%-32s %7.1f %7.1f %7.1f    %7.1f %7.1f %7.1f\n", c.name, lw, lwin, lp, fw, fwin, fp);

        Cost k {};
        for (int b = kSkipBlocks; b < numBlocks; ++b)
        {
            if (! fade[(size_t) b]) continue;
            const double B = tAhead[(size_t) b], A = tLag[(size_t) b] - B, two = std::max (A + B / kOsBlock, B);
            k.serialAvg += tExact[(size_t) b]; k.serialMax = std::max (k.serialMax, tExact[(size_t) b]);
            k.aMax = std::max (k.aMax, A); k.bMax = std::max (k.bMax, B);
            k.twoAvg += two; k.twoMax = std::max (k.twoMax, two);
            k.serialOver += tExact[(size_t) b] > kDeadlineUs; k.twoOver += two > kDeadlineUs;
            ++k.fadeBlocks;
        }
        k.serialAvg /= std::max (1, k.fadeBlocks); k.twoAvg /= std::max (1, k.fadeBlocks);
        costs.push_back (k);
    }

    std::printf ("\ncost of the fade blocks (us; deadline %.1f us; fastest of %d runs per block):\n", kDeadlineUs, kRuns);
    std::printf ("%-32s %16s %10s %10s %10s %16s %16s\n", "", "1 core avg / max", "core A max", "core B max", "", "2 cores avg / max",
                 "over: 1 / 2 core");
    for (size_t i = 0; i < costs.size(); ++i)
    {
        const auto& k = costs[i];
        std::printf ("%-32s %7.1f / %6.1f %10.1f %10.1f %10s %7.1f / %6.1f %8d / %d of %d\n", cases[i].name, k.serialAvg, k.serialMax,
                     k.aMax, k.bMax, "", k.twoAvg, k.twoMax, k.serialOver, k.twoOver, k.fadeBlocks);
    }

    // Where the residual is, and how it scales with the lag: audio band only (4th-order Butterworth low-pass at
    // 20 kHz on both renders), RMS during the fades, in the 100 ms after each, and later; dB re the whole signal's RMS
    std::printf ("\nrun-ahead residual by rail lag, audio band (<20 kHz), dB re signal RMS: whole / in fades / 0-100 ms after / "
                 "later / peak re signal peak\n");
    auto renderLag = [&] (const Automation& automate, int chunk, std::vector<double>& y, std::vector<char>& fade)
    {
        AmpCore amp;
        amp.prepare();
        if (chunk > 0) { amp.twoCore = true; amp.aheadChunk = chunk; amp.preamp.enableRunAhead (true); }
        y.assign ((size_t) numBlocks * kOsBlock, 0.0);
        fade.assign ((size_t) numBlocks, 0);
        for (int b = 0; b < numBlocks; ++b)
        {
            AmpState st;
            automate ((double) b * kBlock / kBase, st);
            if (b > 0) amp.preamp.setMode (st.channel);
            fade[(size_t) b] = amp.preamp.fading();
            sink = sink + amp.block (diOs.data() + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
            std::copy (amp.out, amp.out + kOsBlock, y.begin() + (ptrdiff_t) b * kOsBlock);
        }
        // audio band: two RBJ low-pass biquads (Q 0.5412, 1.3066) at 20 kHz
        for (double q : { 0.5412, 1.3066 })
        {
            const double w0 = 2.0 * kPi * 20000.0 / kOs, al = std::sin (w0) / (2.0 * q), cs = std::cos (w0), a0 = 1.0 + al;
            const double b0 = (1.0 - cs) / 2.0 / a0, b1 = (1.0 - cs) / a0, a1 = -2.0 * cs / a0, a2 = (1.0 - al) / a0;
            double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
            for (auto& v : y)
            {
                const double o = b0 * v + b1 * x1 + b0 * x2 - a1 * y1 - a2 * y2;
                x2 = x1; x1 = v; y2 = y1; y1 = o; v = o;
            }
        }
    };
    for (int ci : { 0, 1 })
    {
        std::vector<double> exact, lag;
        std::vector<char> fade, unused;
        renderLag (cases[ci].automate, 0, exact, fade);
        for (int chunk : { 1, 8, 64 })
        {
            renderLag (cases[ci].automate, chunk, lag, unused);
            double s2 = 0, sp = 0, rp = 0, rIn = 0, rAfter = 0, rLater = 0;
            long long nIn = 0, nAfter = 0, nLater = 0;
            int sinceFade = 1 << 30;
            const int skip = kSkipBlocks * kOsBlock;
            for (size_t i = (size_t) skip; i < exact.size(); ++i)
            {
                const size_t b = i / kOsBlock;
                if (fade[b]) sinceFade = 0; else if (i % kOsBlock == 0) sinceFade += kOsBlock;
                const double d = exact[i] - lag[i];
                s2 += exact[i] * exact[i];
                sp = std::max (sp, std::abs (exact[i])); rp = std::max (rp, std::abs (d));
                if (fade[b]) { rIn += d * d; ++nIn; }
                else if (sinceFade < (int) (0.1 * kOs)) { rAfter += d * d; ++nAfter; }
                else { rLater += d * d; ++nLater; }
            }
            const double sig = std::sqrt (s2 / (double) (exact.size() - (size_t) skip));
            auto rel = [&] (double r2, long long n) { return n > 0 ? db (std::sqrt (r2 / (double) n) / sig) : -999.0; };
            std::printf ("%-26s lag %2d samples (%5.1f us): %7.1f %7.1f %7.1f %7.1f   peak %7.1f\n", cases[ci].name, chunk,
                         1e6 * chunk / kOs, rel (rIn + rAfter + rLater, nIn + nAfter + nLater), rel (rIn, nIn), rel (rAfter, nAfter),
                         rel (rLater, nLater), db (rp / sp));
        }
    }
}

void run()
{
    const auto diOs = guitarDI (kOs, kSeconds), diComp = guitarDI (kCompRate, kSeconds), di = guitarDI (kBase, kSeconds);
    const int numBlocks = (int) (kBase * kSeconds) / kBlock;
    std::vector<float> diF (di.size());
    for (size_t i = 0; i < di.size(); ++i) diF[i] = (float) di[i];
    const double* xOs = diOs.data();
    const double* xComp = diComp.data();

    std::printf ("\n== 8-sample blocks at 48 kHz: cost per block (us; deadline %.1f us; fastest of %d runs per block) ==\n",
                 kDeadlineUs, kRuns);
    std::printf ("amp at 8x (64 samples / block, no oversampling filters), 1176 at 2x (16 samples / block)\n");
    std::printf ("%-38s %6s %6s %6s %7s %6s  %8s %7s %6s %7s\n", "", "avg", "p99", "p99.9", "max", "of dl", "max at", "raw max", "over",
                 "1st");

    // single modules, knobs still
    runCase ("Precision Drive", numBlocks, [&] {
        auto pd = std::make_shared<PrecisionDrive>(); pd->prepare (kOs); pd->setControls (0.5, 0.5, 3, 0.7);
        return Step ([pd, &diOs] (int b) { double a = 0; for (int i = 0; i < kOsBlock; ++i) a += pd->process (diOs[(size_t) (b * kOsBlock + i)]); sink = sink + a; });
    });
    runCase ("TS9", numBlocks, [&] {
        auto ts = std::make_shared<TubeScreamer9>(); ts->prepare (kOs); ts->setControls (0.5, 0.5, 0.7);
        return Step ([ts, &diOs] (int b) { double a = 0; for (int i = 0; i < kOsBlock; ++i) a += ts->process (diOs[(size_t) (b * kOsBlock + i)]); sink = sink + a; });
    });
    const char* preNames[] = { "Preamp CLEAN", "Preamp BLUE", "Preamp RED" };
    for (int mode : { 0, 1, 2 })
        runCase (preNames[mode], numBlocks, [&, mode] {
            auto pre = std::make_shared<Preamp5150>(); pre->prepare (kOs); pre->setModeNow (mode); pre->setControls (0.7, 0.5, 0.5, 0.6, 0.5);
            return Step ([pre, &diOs] (int b) { double a = 0; for (int i = 0; i < kOsBlock; ++i) a += pre->process (diOs[(size_t) (b * kOsBlock + i)]); sink = sink + a; });
        });
    {
        // the power amp on the RED preamp's output, so its operating point is realistic
        std::vector<double> pv (diOs.size());
        {
            Preamp5150 pre; pre.prepare (kOs); pre.setModeNow (Preamp5150::Red); pre.setControls (0.7, 0.5, 0.5, 0.6, 0.5);
            for (size_t i = 0; i < pv.size(); ++i) pv[i] = pre.process (diOs[i]);
        }
        runCase ("Power amp (after RED)", numBlocks, [&] {
            auto pa = std::make_shared<PowerAmp5150>(); pa->setControls (0.5, 0.5); pa->prepare (kOs);
            return Step ([pa, &pv] (int b) { double a = 0; for (int i = 0; i < kOsBlock; ++i) a += pa->process (pv[(size_t) (b * kOsBlock + i)]); sink = sink + a; });
        });
    }

    // the full amp as AmpSim runs it, with automation
    auto ampCase = [&] (const char* name, Automation automate) { runAmpCase (name, numBlocks, diOs.data(), automate); };
    ampCase ("AMP RED, knobs still", [] (double, AmpState&) {});
    ampCase ("AMP RED + Precision Drive", [] (double, AmpState& s) { s.boostOn = true; });
    ampCase ("AMP RED, Gain turning", [] (double t, AmpState& s) { s.target[0] = 0.2 + 0.8 * tri (t, 1.0); });
    ampCase ("AMP RED, Presence + Resonance turning", [] (double t, AmpState& s) { s.target[5] = tri (t, 0.7); s.target[6] = tri (t, 1.1); });
    ampCase ("AMP RED, Bass + Mid + Treble turning", [] (double t, AmpState& s) {
        s.target[1] = tri (t, 0.9); s.target[2] = tri (t, 1.3); s.target[3] = tri (t, 0.6); });
    ampCase ("AMP RED, Master turning", [] (double t, AmpState& s) { s.target[4] = 0.1 + 0.5 * tri (t, 0.8); });
    ampCase ("AMP RED + drive, every knob turning", everyKnob);
    ampCase ("AMP channel RED <-> BLUE every 0.25 s", [] (double t, AmpState& s) {
        s.channel = (int) (t / 0.25) % 2 == 0 ? Preamp5150::Red : Preamp5150::Blue; });
    ampCase ("AMP channel RED -> BLUE -> CLEAN cycle", [] (double t, AmpState& s) {
        const int c[3] = { Preamp5150::Red, Preamp5150::Blue, Preamp5150::Clean };
        s.channel = c[(int) (t / 0.25) % 3]; });
    ampCase ("AMP drive PD <-> TS9 every 0.25 s", [] (double t, AmpState& s) { s.boostOn = true; s.boostModel = (int) (t / 0.25) % 2; });

    // the 1176 at 2x
    auto compCase = [&] (const char* name, CompControls controls, bool everyBlock) {
        runCompCase (name, numBlocks, diComp.data(), controls, everyBlock);
    };
    compCase ("1176 IN 7, atk/rel 5, 4:1", [] (double) { return Urei1176::fromKnobs (7, 8.5, 5, 5, 0); }, false);
    compCase ("1176 IN 10, atk/rel 7, ALL", [] (double) { return Urei1176::fromKnobs (10, 8.5, 7, 7, 4); }, false);
    compCase ("1176 Input + Attack + Release turning", [] (double t) {
        return Urei1176::fromKnobs (3 + 7 * tri (t, 0.9), 8.5, 1 + 6 * tri (t, 0.7), 1 + 6 * tri (t, 1.1), 1); }, true);

    // 48 kHz modules
    runCase ("3-band gate", numBlocks, [&] {
        auto g = std::make_shared<MultibandGate>(); g->prepare (kBase);
        auto buf = std::make_shared<std::vector<float>> (diF);
        auto key = std::make_shared<std::vector<float>> (diF);
        return Step ([g, buf, key] (int b) { MultibandGate::Settings st; g->process (buf->data() + b * kBlock, key->data() + b * kBlock, kBlock, st); });
    });
    for (int mode : { (int) PitchShifter::Live, (int) PitchShifter::HQ })
        runCase (mode == PitchShifter::Live ? "Transpose LIVE (-2 st)" : "Transpose HQ (-2 st)", numBlocks, [&, mode] {
            auto ps = std::make_shared<PitchShifter>(); ps->prepare (kBase); ps->setParams (-2, mode);
            auto buf = std::make_shared<std::vector<float>> (diF);
            return Step ([ps, buf] (int b) { ps->process (buf->data() + b * kBlock, kBlock); });
        });

    // what one core would have to do: LIVE transpose + 1176 + amp with drive and every knob turning + gate
    runCase ("CHAIN: transpose+1176+amp+gate, knobs", numBlocks, [&] {
        auto ps = std::make_shared<PitchShifter>(); ps->prepare (kBase); ps->setParams (-2, PitchShifter::Live);
        auto m = std::make_shared<Urei1176>(); m->setControls (Urei1176::fromKnobs (7, 8.5, 5, 5, 0)); m->prepare (kCompRate);
        auto amp = std::make_shared<AmpCore>(); amp->prepare();
        auto g = std::make_shared<MultibandGate>(); g->prepare (kBase);
        auto buf = std::make_shared<std::vector<float>> (diF);
        auto key = std::make_shared<std::vector<float>> (diF);
        return Step ([=] (int b) {
            ps->process (buf->data() + b * kBlock, kBlock);
            double a = 0;
            for (int i = 0; i < kCompBlock; ++i) a += m->processSample (xComp[(size_t) (b * kCompBlock + i)]);
            AmpState st;
            everyKnob ((double) b * kBlock / kBase, st);
            a += amp->block (xOs + (size_t) b * kOsBlock, st.target, st.channel, st.boostOn, st.boostModel);
            MultibandGate::Settings gs;
            g->process (buf->data() + b * kBlock, key->data() + b * kBlock, kBlock, gs);
            sink = sink + a;
        });
    });
    std::printf ("\nof dl = max as %% of the deadline; max at = where the slowest block is; raw max = slowest single block "
                 "of all runs (incl. OS interruptions); over = blocks past the deadline (fastest-run times); 1st = the first "
                 "block (channel reset, every knob applied), left out of the other columns with the rest of the first 10 ms\n");
}
} // namespace block8

static void transposeTests()
{
    constexpr double tfs = 48000.0;
    const int n = (int) (tfs * 2.0);

    auto run = [&] (int algo, int semis, const std::vector<float>& in)
    {
        std::vector<float> out (in);
        if (algo == 0)
        {
            LegacyShifter s; s.prepare (tfs, semis);
            for (auto& v : out) v = s.process (v);
        }
        else
        {
            PitchShifter s; s.prepare (tfs); s.setParams (semis, algo == 1 ? PitchShifter::Live : PitchShifter::HQ);
            for (int i = 0; i < n; i += 256) s.process (out.data() + i, std::min (256, n - i));
        }
        return out;
    };

    auto measureFreq = [&] (const std::vector<float>& y)
    {
        // rising zero crossings with linear interpolation over the last second
        double first = -1, last = -1; int count = 0;
        for (int i = n / 2 + 1; i < n; ++i)
            if (y[(size_t) i - 1] < 0 && y[(size_t) i] >= 0)
            {
                const double t = (i - 1) + y[(size_t) i - 1] / (y[(size_t) i - 1] - y[(size_t) i]);
                if (first < 0) first = t; else ++count;
                last = t;
            }
        return count > 0 ? count * tfs / (last - first) : 0.0;
    };

    auto warble = [&] (const std::vector<float>& y, double winSec)
    {
        const int win = (int) (tfs * winSec);
        double mn = 1e9, mx = -1e9;
        for (int s = n / 2; s + win <= n; s += win)
        {
            double sq = 0; for (int i = s; i < s + win; ++i) sq += (double) y[(size_t) i] * y[(size_t) i];
            const double db = 10.0 * std::log10 (sq / win + 1e-20);
            mn = std::min (mn, db); mx = std::max (mx, db);
        }
        return mx - mn;
    };

    const char* names[] = { "legacy", "LIVE  ", "HQ    " };
    std::vector<float> sine ((size_t) n), chord ((size_t) n);
    for (int i = 0; i < n; ++i)
    {
        const double t = i / tfs;
        sine[(size_t) i] = (float) (0.5 * std::sin (2 * kPi * 110.0 * t));
        chord[(size_t) i] = (float) (0.25 * (std::sin (2 * kPi * 110.0 * t) + std::sin (2 * kPi * 164.81 * t)
                                             + std::sin (2 * kPi * 220.0 * t) + 0.5 * std::sin (2 * kPi * 277.18 * t)));
    }

    std::printf ("\n== transpose: pitch of a 110 Hz sine (expected Hz in brackets) ==\n");
    for (int semis : { -2, -5, -12, 5, 12 })
    {
        std::printf ("%+3d st [%7.2f]:", semis, 110.0 * std::pow (2.0, semis / 12.0));
        for (int a = 0; a < 3; ++a) std::printf ("  %s %7.2f", names[a], measureFreq (run (a, semis, sine)));
        std::printf ("\n");
    }

    std::printf ("\n== transpose: measured latency (onset of a 220 Hz tone at 0.5 s, -2 st) ==\n");
    {
        std::vector<float> burst ((size_t) n, 0.0f);
        for (int i = n / 4; i < n; ++i) burst[(size_t) i] = (float) (0.5 * std::sin (2 * kPi * 220.0 * i / tfs));
        for (int a = 1; a < 3; ++a)
        {
            const auto y = run (a, -2, burst);
            int onset = -1;
            for (int i = n / 4; i < n && onset < 0; ++i)
                if (std::abs (y[(size_t) i]) > 0.25f) onset = i;
            PitchShifter s; s.prepare (tfs); s.setParams (-2, a == 1 ? PitchShifter::Live : PitchShifter::HQ);
            std::printf ("%s: first output at +%.1f ms (reported latency %d samples = %.1f ms, avg delay %.1f ms)\n",
                         names[a], (onset - n / 4) * 1000.0 / tfs, s.getReportedLatency(),
                         s.getReportedLatency() * 1000.0 / tfs, s.getAverageDelay() * 1000.0 / tfs);
        }
    }

    std::printf ("\n== transpose: warble = RMS fluctuation in dB (lower is smoother) ==\n");
    std::printf ("unshifted reference: sine %.2f dB, chord %.2f dB\n", warble (sine, 0.010), warble (chord, 0.040));
    for (int semis : { -2, -5, -12 })
    {
        std::printf ("%+3d st sine :", semis);
        for (int a = 0; a < 3; ++a) std::printf ("  %s %5.2f dB", names[a], warble (run (a, semis, sine), 0.010));
        std::printf ("\n%+3d st chord:", semis);
        for (int a = 0; a < 3; ++a) std::printf ("  %s %5.2f dB", names[a], warble (run (a, semis, chord), 0.040));
        std::printf ("\n");
    }
}

static void inputGateTest()
{
    constexpr double gfs = 48000.0;
    std::printf ("\n== input gate: 80 ms chugs (-12 dBFS) every 200 ms + -60 dBFS noise, thresh -50, release 15 ms ==\n");
    InputGate g;
    g.prepare (gfs);
    const int n = (int) (gfs * 1.0);
    std::vector<float> x ((size_t) n);
    unsigned rng = 3;
    for (int i = 0; i < n; ++i)
    {
        const double t = i / gfs, ph = std::fmod (t, 0.2);
        rng = rng * 1664525u + 1013904223u;
        const double noise = 0.001 * ((rng >> 8) / 8388608.0 - 1.0);
        const double chug = ph < 0.08 ? 0.25 * std::exp (-ph * 20.0) * std::sin (2 * kPi * 82.4 * t) : 0.0;
        x[(size_t) i] = (float) (chug + noise);
    }
    std::vector<float> y (x);
    g.process (y.data(), n, -50.0f, 15.0f);
    double inGap = 0, outGap = 0, inChug = 0, outChug = 0;
    for (int i = n / 5; i < n; ++i)
    {
        const double ph = std::fmod (i / gfs, 0.2);
        if (ph > 0.12 && ph < 0.19) { inGap += x[(size_t) i] * x[(size_t) i]; outGap += y[(size_t) i] * y[(size_t) i]; }
        if (ph > 0.005 && ph < 0.05) { inChug += x[(size_t) i] * x[(size_t) i]; outChug += y[(size_t) i] * y[(size_t) i]; }
    }
    std::printf ("between chugs: %.1f dB reduction | during chugs: %.2f dB change\n",
                 10.0 * std::log10 (outGap / inGap + 1e-30), 10.0 * std::log10 (outChug / inChug));
}

static void chainLevels()
{
    std::printf ("\n== full chain: speaker V rms (gain 6, tone 5/5/6, pres/res 5), guitar-ish 0.2 V chord ==\n");
    const double post[3] = { 54.0, 1.0, 1.0 }; // keep in sync with kChannelPost in AmpSim.h
    for (int mode : { 0, 1, 2 })
    {
        std::printf ("mode %d:", mode);
        for (double master : { 0.1, 0.2, 0.3, 0.5, 0.8 })
        {
            Preamp5150 pre;
            PowerAmp5150 pa;
            pre.prepare (fs);
            pre.setMode (mode);
            pre.setControls (0.6, 0.5, 0.5, 0.6, master);
            pa.setControls (0.5, 0.5);
            pa.prepare (fs);
            double sq = 0; int n = 0;
            for (int i = 0; i < (int) (fs * 0.6); ++i)
            {
                const double t = i / fs, env = std::exp (-3.0 * std::fmod (t, 0.5));
                const double x = 0.2 * env * (std::sin (2 * kPi * 82.4 * t) + std::sin (2 * kPi * 123.5 * t) + std::sin (2 * kPi * 164.8 * t)) / 3.0;
                const double y = processAmp5150 (pre, pa, x, post[mode]);
                if (i > (int) (fs * 0.1)) { sq += y * y; ++n; }
            }
            std::printf ("  m%.1f %6.2f V", master, std::sqrt (sq / n));
        }
        std::printf ("\n");
    }
}

// Fault recovery: a single non-finite or absurd sample must not leave the circuit in a state that differs
// from an undisturbed render for longer than the sample itself. RED preamp + power amp at the plugin's
// internal rate, palm-muted chord chugs; one bad sample at 0.4 s, compared to the clean render.
static bool faultRecoveryTests()
{
    std::printf ("\n== fault recovery: one bad sample at 0.4 s (RED gain 5, master 6, 352.8 kHz) ==\n");
    constexpr double rate = 352800.0;
    const int n = (int) (rate * 1.0), at = (int) (rate * 0.4);

    auto dry = [&] (int i)
    {
        const double t = i / rate, env = std::exp (-12.0 * std::fmod (t, 0.125));
        return 0.3 * env * (std::sin (2 * kPi * 82.4 * t) + std::sin (2 * kPi * 123.5 * t)) / 2.0;
    };

    // where = 0: bad sample into the preamp input, 1: into the power amp input, 2: the input sample replaced
    // by the previous one (a lost sample can't be recovered: this is the best any fault handling can do)
    // mode: Preamp5150 channel (RED and BLUE: each the whole preamp as one circuit)
    auto render = [&] (int where, double bad, long long* failures = nullptr, int mode = Preamp5150::Red)
    {
        Preamp5150 pre;
        PowerAmp5150 pa;
        pre.prepare (rate);
        pre.setMode (mode);
        pre.setControls (0.5, 0.5, 0.5, 0.6, 0.6);
        pa.setControls (0.5, 0.5);
        pa.prepare (rate);
        pa.statFailures = 0;
        std::vector<double> y ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            const bool hit = i == at && where >= 0;
            // solve reported bad once (retry) / twice (hold): 5 / 6 power amp,
            // 7 / 8 the channel's preamp circuit
            if (hit && (where == 5 || where == 6)) pa.testFaults = where - 4;
            if (hit && (where == 7 || where == 8)) (mode == Preamp5150::Red ? pre.lead.net.testFaults : pre.blue.net.testFaults) = where - 6;
            // 9 / 10: the channel's preamp / the power amp ends a solve unconverged and 300 V off (must be caught);
            // 11 / 12: the preamp / power amp fails for 2 ms (twice the hold time: back to the idle point)
            if (hit && where == 9) (mode == Preamp5150::Red ? pre.lead.net.testCorrupt : pre.blue.net.testCorrupt) = 1;
            if (hit && where == 10) pa.testCorrupt = 1;
            if (hit && where == 11) (mode == Preamp5150::Red ? pre.lead.net.testFaults : pre.blue.net.testFaults) = (int) (4e-3 * rate);
            if (hit && where == 12) pa.testFaults = (int) (4e-3 * rate);
            double v = pre.process (hit && where == 0 ? bad : hit && where == 2 ? dry (i - 1) : dry (i));
            y[(size_t) i] = pa.process (hit && where == 1 ? bad : v);
        }
        if (failures) *failures = pa.statFailures + pre.lead.net.statFailures + pre.blue.net.statFailures;
        return y;
    };

    const std::vector<double> refs[2] = { render (-1, 0.0, nullptr, Preamp5150::Red), render (-1, 0.0, nullptr, Preamp5150::Blue) };
    const auto& ref = refs[0];
    double refPeak = 0.0;
    for (double v : ref) refPeak = std::max (refPeak, std::abs (v));

    // audio-band deviation from the clean render of the same channel: max and the time it stays above 1% of the
    // clean (RED) peak
    struct Dev { bool finite; double raw, band, ms; };
    auto deviation = [&] (const std::vector<double>& y, int mode = Preamp5150::Red)
    {
        const auto& ref = refs[mode == Preamp5150::Red ? 0 : 1];
        Biquad lp1, lp2;
        lp1.set (Biquad::Type::lowPass, rate, 20000.0, 0.5412);
        lp2.set (Biquad::Type::lowPass, rate, 20000.0, 1.3066);
        Dev r { true, 0.0, 0.0, 0.0 };
        int last = at;
        for (int i = 0; i < n; ++i)
        {
            if (! std::isfinite (y[(size_t) i])) { r.finite = false; continue; }
            const double e = y[(size_t) i] - ref[(size_t) i];
            const double d = std::abs ((double) lp2.process (lp1.process ((float) e)));
            r.raw = std::max (r.raw, std::abs (e));
            r.band = std::max (r.band, d);
            if (d > 0.01 * refPeak) last = i;
        }
        r.ms = (last - at) * 1000.0 / rate;
        return r;
    };
    const Dev best[2] = { deviation (render (2, 0.0, nullptr, Preamp5150::Red), Preamp5150::Red),
                          deviation (render (2, 0.0, nullptr, Preamp5150::Blue), Preamp5150::Blue) };

    bool ok = true;
    constexpr int R = Preamp5150::Red, B = Preamp5150::Blue;
    struct Case { const char* name; int where; double bad; int mode; };
    const Case cases[] = {
        { "NaN  -> power amp",            1, std::nan (""), R },
        { "+inf -> power amp",            1, HUGE_VAL,      R },
        { "NaN  -> preamp (RED)",         0, std::nan (""), R },
        { "+inf -> preamp (RED)",         0, HUGE_VAL,      R },
        { "NaN  -> preamp (BLUE)",        0, std::nan (""), B },
        { "RED preamp fails, retry",      7, 0.0,           R },
        { "BLUE preamp fails, retry",     7, 0.0,           B },
        { "power amp fails, retry",       5, 0.0,           R },
        { "RED preamp unconverged, 300 V off", 9, 0.0,      R },
        { "BLUE preamp unconverged, 300 V", 9, 0.0,         B },
        { "power amp unconverged, 300 V", 10, 0.0,          R },
    };
    std::printf ("clean render: peak %.1f V at the speaker (RED)\n", refPeak);
    // Deviation in the audio band (4th-order Butterworth low-pass at 20 kHz, what the downsampler keeps).
    // Pass: finite, no worse than the lost-sample floor (+10% / 0.1 V), back within 1% of the peak after 1 ms.
    std::printf ("%-28s %7s %12s %14s %15s %7s\n", "case", "finite", "raw max (V)", "<20k max (V)", "<20k > 1% (ms)", "result");
    for (int m : { R, B })
    {
        const Dev& f = best[m == R ? 0 : 1];
        std::printf ("%-28s %7s %12.2f %14.2f %15.2f %7s\n", m == R ? "(floor: input held, RED)" : "(floor: input held, BLUE)",
                     "yes", f.raw, f.band, f.ms, "");
    }
    for (const auto& c : cases)
    {
        const Dev& f = best[c.mode == R ? 0 : 1];
        const Dev r = deviation (render (c.where, c.bad, nullptr, c.mode), c.mode);
        const bool pass = r.finite && r.band < 1.1 * f.band + 0.1 && r.ms < 1.0;
        ok = ok && pass;
        std::printf ("%-28s %7s %12.2f %14.2f %15.2f %7s\n", c.name, r.finite ? "yes" : "NO", r.raw, r.band, r.ms, pass ? "PASS" : "FAIL");
    }
    // Last resort when the retry fails as well: the sample is dropped (states held). Worse than the floor (it
    // upsets the trapezoidal companions), so only finiteness and recovery within 1 ms are required.
    const Case holds[] = { { "RED preamp fails twice, hold", 8, 0.0, R },
                           { "BLUE preamp fails twice, hold", 8, 0.0, B }, { "power amp twice, hold", 6, 0.0, R } };
    for (const auto& c : holds)
    {
        const Dev r = deviation (render (c.where, c.bad, nullptr, c.mode), c.mode);
        const bool pass = r.finite && r.ms < 1.0;
        ok = ok && pass;
        std::printf ("%-24s %7s %12.2f %14.2f %15.2f %7s\n", c.name, r.finite ? "yes" : "NO", r.raw, r.band, r.ms, pass ? "PASS" : "FAIL");
    }
    // Persistent fault (2 ms of failing solves, twice the hold time): the circuit goes back to its idle point, a
    // dropout that has to rebuild (cathode / coupling caps). Required: finite, never louder than 1.2x the clean peak,
    // and back within 1% of the clean peak within 500 ms.
    const Case persistent[] = { { "RED preamp fails 2 ms", 11, 0.0, R }, { "BLUE preamp fails 2 ms", 11, 0.0, B },
                                { "power amp fails 2 ms", 12, 0.0, R } };
    for (const auto& c : persistent)
    {
        const auto y = render (c.where, c.bad, nullptr, c.mode);
        const Dev r = deviation (y, c.mode);
        double peak = 0.0;
        for (double v : y) peak = std::max (peak, std::abs (v));
        const bool pass = r.finite && peak < 1.2 * refPeak && r.ms < 500.0;
        ok = ok && pass;
        std::printf ("%-24s %7s %12.2f %14.2f %15.2f %7s   (peak %.1f V)\n", c.name, r.finite ? "yes" : "NO", r.raw, r.band, r.ms,
                     pass ? "PASS" : "FAIL", peak);
    }

    // Absurd but finite input: a 1 kV spike and 2 s of a 100 V sine at the guitar jack. Large deviations are
    // legitimate here (the circuit really is overdriven); the output must stay finite and physically bounded
    // (the speaker cannot exceed what the output stage can deliver from B+), with no Newton failures.
    std::printf ("\n== extreme input: 1 kV spike, 100 V sine (stays finite and bounded) ==\n");
    {
        long long fails = 0;
        const auto y = render (0, 1000.0, &fails);
        double peak = 0.0; bool finite = true;
        for (double v : y) { finite = finite && std::isfinite (v); peak = std::max (peak, std::abs (v)); }
        const bool pass = finite && peak < 100.0 && fails == 0;
        ok = ok && pass;
        std::printf ("1 kV spike:   finite %s, speaker peak %.1f V, %lld Newton failures  %s\n",
                     finite ? "yes" : "NO", peak, fails, pass ? "PASS" : "FAIL");
    }
    {
        Preamp5150 pre;
        PowerAmp5150 pa;
        pre.prepare (rate);
        pre.setControls (1.0, 0.5, 0.5, 0.6, 1.0);
        pa.setControls (1.0, 1.0);
        pa.prepare (rate);
        pa.statFailures = 0;
        double peak = 0.0; bool finite = true;
        for (int i = 0; i < (int) (rate * 2.0); ++i)
        {
            const double y = processAmp5150 (pre, pa, 100.0 * std::sin (2 * kPi * 110.0 * i / rate), 1.0);
            finite = finite && std::isfinite (y);
            peak = std::max (peak, std::abs (y));
        }
        const bool pass = finite && peak < 100.0 && pa.statFailures + pre.lead.net.statFailures + pre.blue.net.statFailures == 0;
        ok = ok && pass;
        std::printf ("100 V sine:   finite %s, speaker peak %.1f V, %lld Newton failures  %s\n",
                     finite ? "yes" : "NO", peak, pa.statFailures + pre.lead.net.statFailures + pre.blue.net.statFailures, pass ? "PASS" : "FAIL");
    }
    return ok;
}

// One-sided noise density (V / sqrt(Hz)) of x at rate, Welch average of Hann-windowed 2^14 blocks; band RMS helper
struct Density
{
    double rate = 1.0, binHz = 1.0;
    std::vector<double> psd; // V^2 / Hz per bin
    Density (const std::vector<double>& x, double fs) : rate (fs)
    {
        constexpr int n = 1 << 14;
        FFT fft; fft.prepare (n);
        psd.assign (n / 2, 0.0);
        std::vector<std::complex<double>> b ((size_t) n);
        double wsum = 0.0;
        for (int i = 0; i < n; ++i) { const double w = 0.5 - 0.5 * std::cos (2 * kPi * i / n); wsum += w * w; }
        int blocks = 0;
        for (size_t s = 0; s + n <= x.size(); s += n / 2, ++blocks)
        {
            for (int i = 0; i < n; ++i) b[(size_t) i] = x[s + (size_t) i] * (0.5 - 0.5 * std::cos (2 * kPi * i / n));
            fft.transform (b.data(), false);
            for (int k = 1; k < n / 2; ++k) psd[(size_t) k] += 2.0 * std::norm (b[(size_t) k]) / (wsum * fs);
        }
        for (double& v : psd) v /= std::max (blocks, 1);
        binHz = fs / n;
    }
    double at (double f) const // density around f (1/6 octave average)
    {
        double s = 0; int c = 0;
        for (size_t k = 1; k < psd.size(); ++k) if (k * binHz > f / 1.06 && k * binHz < f * 1.06) { s += psd[k]; ++c; }
        return std::sqrt (s / std::max (c, 1));
    }
    double rms (double f0, double f1) const
    {
        double s = 0;
        for (size_t k = 1; k < psd.size(); ++k) if (k * binHz >= f0 && k * binHz <= f1) s += psd[k] * binHz;
        return std::sqrt (s);
    }
};

// Circuit noise: the tube noise generator against its target, then the whole preamp's hiss
static void noiseTests()
{
    constexpr double fs = 192000.0, kT4 = 4.0 * 1.380649e-23 * 300.0;
    std::printf ("\n== tube noise (12AX7, gm 1.25 mA/V: Req 2 kOhm, flicker corner %.0f Hz), referred to the grid ==\n",
                 TubeNoise::kFlickerCorner);
    {
        TubeNoise tn; tn.prepare (fs, 1.25e-3, 12345);
        std::vector<double> x ((size_t) (fs * 20));
        for (double& v : x) v = tn.next();
        const Density d (x, fs);
        const double ew = std::sqrt (kT4 * 2000.0);
        std::printf ("%8s %12s %12s\n", "Hz", "nV/rtHz", "target");
        for (double f : { 50.0, 100.0, 1000.0, 5000.0, 20000.0 })
            std::printf ("%8.0f %12.2f %12.2f\n", f, d.at (f) * 1e9, ew * std::sqrt (1.0 + TubeNoise::kFlickerCorner / f) * 1e9);
        std::printf ("25 Hz - 10 kHz: %.2f uV rms (12AX7A / 7025 spec: 1.8 uV average, noise and hum)\n", d.rms (25, 10000) * 1e6);
    }

    std::printf ("\n== preamp hiss, input shorted (jack at 0 V through R27), tone stack + Post 5, 20 Hz - 20 kHz ==\n");
    std::printf ("%-6s %5s | %14s %16s %24s\n", "chan", "gain", "out (mV rms)", "gain at 1 kHz", "in-referred at 1 kHz");
    for (int mode : { Preamp5150::Red, Preamp5150::Blue, Preamp5150::Clean })
        for (double gain : { 0.2, 0.5, 0.8 })
        {
            auto run = [&] (bool noise, double amp)
            {
                Preamp5150 pre; pre.setNoise (noise); pre.prepare (fs); pre.setModeNow (mode);
                pre.setChannelTrims (54.0, 1.0, 1.0); // AmpSim kChannelPost
                pre.setControls (gain, 0.5, 0.5, 0.5, 0.5);
                for (int i = 0; i < (int) (fs * 0.3); ++i) pre.process (0.0);
                std::vector<double> y ((size_t) (fs * 3));
                for (size_t i = 0; i < y.size(); ++i) y[i] = pre.process (amp * std::sin (2 * kPi * 1000.0 * (double) i / fs));
                return y;
            };
            const Density dn (run (true, 0.0), fs);
            // small-signal gain at 1 kHz (noise off): 20 uV in, amplitude of the 1 kHz bin
            const auto ys = run (false, 20e-6);
            double re = 0, im = 0;
            for (size_t i = 0; i < ys.size(); ++i) { const double ph = 2 * kPi * 1000.0 * (double) i / fs; re += ys[i] * std::cos (ph); im += ys[i] * std::sin (ph); }
            const double g = 2.0 * std::sqrt (re * re + im * im) / (double) ys.size() / 20e-6;
            const char* nm[] = { "CLEAN", "BLUE", "RED" };
            std::printf ("%-6s %5.1f | %14.2f %16.1f %16.1f nV/rtHz\n", nm[mode], gain * 10, dn.rms (20, 20000) * 1e3, g,
                         dn.at (1000.0) / g * 1e9);
        }
    std::printf ("(by hand, V1A's grid alone: R27 22k || R25 1M + R22 68k + its tube, at 1 kHz: %.1f nV/rtHz)\n",
                 std::sqrt (kT4 * (21.5e3 + 68e3 + 2000.0 * (1.0 + TubeNoise::kFlickerCorner / 1000.0))) * 1e9);
}

// Tube sets: each set's draws and what they do (preamp gain and harmonics, output-pair balance, hum, H2)
static void tubeSetTests()
{
    constexpr double fs = 192000.0;
    auto tone = [&] (const std::vector<double>& y, double f, int skip)
    {
        double re = 0, im = 0; const size_t n = y.size() - (size_t) skip;
        for (size_t i = 0; i < n; ++i) { const double ph = 2 * kPi * f * (double) i / fs; re += y[(size_t) skip + i] * std::cos (ph); im += y[(size_t) skip + i] * std::sin (ph); }
        return 2.0 * std::sqrt (re * re + im * im) / (double) n;
    };
    std::printf ("\n== tube sets (RED gain 5, presence / resonance 5) ==\n");
    std::printf ("%-8s | %-34s | %-15s | %7s %12s %12s | %9s %13s\n", "set", "12AX7 current (V1A..V5A)", "6L6 pairs", "pre dB",
                 "pre H2 dBc", "pre H3 dBc", "hum dB", "out H2 dBc");
    double g0 = 0.0;
    for (int set = 0; set < TubeSet::kSets; ++set)
    {
        const auto s = TubeSet::make (set);
        // preamp alone: 1 kHz 20 uV small-signal gain; 110 Hz 30 mV harmonics at the preamp output
        auto pre = [&] (double amp, double f)
        {
            Preamp5150 p; p.setTubeSet (s); p.prepare (fs); p.setModeNow (Preamp5150::Red);
            p.setControls (0.5, 0.5, 0.5, 0.5, 0.5);
            std::vector<double> y ((size_t) (fs * 1.0));
            for (size_t i = 0; i < y.size(); ++i) y[i] = p.process (amp * std::sin (2 * kPi * f * (double) i / fs));
            return y;
        };
        const double g = 20 * std::log10 (tone (pre (20e-6, 1000.0), 1000.0, (int) (fs * 0.5)) / 20e-6);
        if (set == 0) g0 = g;
        const auto yh = pre (0.03, 110.0);
        const double h1 = tone (yh, 110.0, (int) (fs * 0.5)), h2 = tone (yh, 220.0, (int) (fs * 0.5)), h3 = tone (yh, 330.0, (int) (fs * 0.5));
        // power amp: idle hum at the speaker (dB re 53 V peak) and H2 at 1 kHz, 0.2 V at the master node
        auto pa = [&] (double amp)
        {
            PowerAmp5150 a; a.setTubeSet (s); a.setControls (0.5, 0.5); a.prepare (fs);
            std::vector<double> y ((size_t) (fs * 1.0));
            for (size_t i = 0; i < y.size(); ++i) y[i] = a.process (amp * std::sin (2 * kPi * 1000.0 * (double) i / fs));
            return y;
        };
        const auto yi = pa (0.0), yd = pa (0.2);
        const double hum = 20 * std::log10 (std::max (tone (yi, 120.0, (int) (fs * 0.3)), 1e-15) / 53.0);
        const double o1 = tone (yd, 1000.0, (int) (fs * 0.5)), o2 = tone (yd, 2000.0, (int) (fs * 0.5));
        char pres[64]; std::snprintf (pres, sizeof pres, "%.2f %.2f %.2f %.2f %.2f %.2f", s.preI[0], s.preI[1], s.preI[2], s.preI[3], s.preI[4], s.preI[5]);
        std::printf ("%-8s | %-34s | %6.3f %6.3f  | %+7.2f %12.1f %12.1f | %9.1f %13.1f\n", TubeSet::kNames[set], pres, s.sideI[0], s.sideI[1],
                     g - g0, 20 * std::log10 (h2 / h1), 20 * std::log10 (h3 / h1), hum, 20 * std::log10 (o2 / o1));
    }
}

int main (int argc, char** argv)
{
    if (argc > 1 && std::string (argv[1]) == "noise") { noiseTests(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "tubes") { tubeSetTests(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "fault") return faultRecoveryTests() ? 0 : 1;
    if (argc > 1 && std::string (argv[1]) == "cpu") { cpuModules(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "block8") { block8::run(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "switch8") { block8::switchProfile(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "twocore") { block8::twoCoreTest(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "tables") { tubeTableAccuracy(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "tube") { pentodeDatasheet(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "chain") { chainLevels(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "transpose") { transposeTests(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "ingate") { inputGateTest(); return 0; }
    inputGateTest();
    if (argc > 1 && std::string (argv[1]) == "power") { powerAmpTests(); return 0; }
    if (argc > 2 && std::string (argv[1]) == "pdebug") { powerAmpDebug (std::atof (argv[2])); return 0; }
    gateTests();
    powerAmpTests();
    channelLevels();
    leadPreampVsSpice();
    toneStackResponse();
    driveSanity();
    tubeTableAccuracy();
    const bool faultsOk = faultRecoveryTests();
    cpuBenchmark();
    return faultsOk ? 0 : 1;
}
