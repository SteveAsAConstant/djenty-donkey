// Offline checks of the component-level DSP against the LTspice reference runs.
// Build target: dsp_test (no JUCE needed).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include "dsp/MultibandGate.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "dsp/PrecisionDrive.h"

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
    std::printf ("\n== 5150 lead preamp (B+ 320V, pot 0.5, 1 kHz) ==\n");
    std::printf ("%-8s %8s %8s %8s %8s %8s %8s | %8s %8s %8s %8s %8s %8s\n", "amp",
                 "Vk1a", "Vk1b", "Vk2a", "Vk2b", "Vk5b", "Vk5a", "p1a_pp", "p1b_pp", "p2a_pp", "p2b_pp", "p5b_pp", "out_pp");

    for (double amp : { 0.001, 0.01, 0.1, 0.5 })
    {
        Preamp5150 pre;
        pre.prepare (fs); // mode Red by default
        pre.setPreGainPot (0.5);

        TriodeStage* st[] = { &pre.v1a, &pre.v1b, &pre.v2a, &pre.v2b, &pre.v5b, &pre.v5a };
        Stat k[6], p[6];
        const int total = (int) (fs * 0.06), from = (int) (fs * 0.04);
        for (int i = 0; i < total; ++i)
        {
            pre.process (amp * std::sin (2.0 * kPi * 1000.0 * i / fs));
            if (i >= from)
                for (int s = 0; s < 6; ++s) { k[s].add (st[s]->getVk()); p[s].add (st[s]->getVp()); }
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
        for (int atk : { 0, 7 })
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
        pa.setControls (0.5, 0.5);
        pa.prepare (fs);
        for (int i = 0; i < (int) (fs * 0.2); ++i) pa.process (0.0);
        std::printf ("B+ %.1f V, bias %.2f V, output plate %.1f V, PI cathode %.2f V, PI plates %.1f / %.1f V\n",
                     pa.getSupplyVoltage(), pa.getBiasVoltage(), pa.getPlate1(), pa.getPIcathode(),
                     pa.getPIplateA(), pa.getPIplateB());
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
            for (int k = 0; k < 8; ++k) std::printf (" %.2f", pa.getPort (k));
            std::printf (" | p0:");
            for (int k = 0; k < 8; ++k) std::printf (" %.1f", pa.lastP0[k]);
            std::printf ("\n");
            ++shown;
            i += 20;
        }
    }
}

static void chainLevels()
{
    std::printf ("\n== full chain: speaker V rms (gain 6, tone 5/5/6, pres/res 5), guitar-ish 0.2 V chord ==\n");
    const double post[3] = { 25.0, 1.0, 1.0 }; // keep in sync with kChannelPost in AmpSim.h
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
                const double y = pa.process (pre.process (x) * post[mode]);
                if (i > (int) (fs * 0.1)) { sq += y * y; ++n; }
            }
            std::printf ("  m%.1f %6.2f V", master, std::sqrt (sq / n));
        }
        std::printf ("\n");
    }
}

int main (int argc, char** argv)
{
    if (argc > 1 && std::string (argv[1]) == "chain") { chainLevels(); return 0; }
    if (argc > 1 && std::string (argv[1]) == "power") { powerAmpTests(); return 0; }
    if (argc > 2 && std::string (argv[1]) == "pdebug") { powerAmpDebug (std::atof (argv[2])); return 0; }
    gateTests();
    powerAmpTests();
    channelLevels();
    leadPreampVsSpice();
    toneStackResponse();
    driveSanity();
    cpuBenchmark();
    return 0;
}
