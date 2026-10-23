// Offline checks of the component-level DSP against the LTspice reference runs.
// Build target: dsp_test (no JUCE needed).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
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

int main()
{
    channelLevels();
    leadPreampVsSpice();
    toneStackResponse();
    driveSanity();
    cpuBenchmark();
    return 0;
}
