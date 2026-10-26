// The 1176 model (Source/dsp/Urei1176.h) against NAM captures of real 1176s (A2 files from TONE3000, T3K license:
// used locally, never redistributed). Captures carry no level calibration and their knob settings are only partly
// documented, so the comparison measures the same things on both (static curves, harmonics, small-signal response,
// attack and release envelopes within the network's ~130 ms memory) and fits the unknowns: the digital-to-volts
// scale, the output gain, and the front-panel settings.
// Build target: comp1176_nam (no JUCE).  comp1176_nam <capture.nam> [measure | fit ...]
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "NamWaveNet.h"
#include "dsp/Urei1176.h"

using namespace vamp;

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kFs = 48000.0; // the captures' rate; the model runs at 2x and is decimated by picking

// ------------------------------------------------------------------ test signals and measurements
struct Signal
{
    // sine of peak a (full scale = 1) at f, optional burst (on from t0) and level step (x g from t1)
    double a = 0.1, f = 1000.0, t0 = 0.0, t1 = 1e9, g = 1.0;
    double at (double t) const { return t < t0 ? 0.0 : a * std::sin (2.0 * kPi * f * (t - t0)) * (t < t1 ? 1.0 : g); }
};

// A device under test: maps a signal to output samples at kFs
struct Dut
{
    virtual ~Dut() = default;
    virtual std::vector<double> render (const Signal& s, double seconds) = 0;
};

struct NamDut : Dut
{
    namcmp::NamWaveNet proto;
    std::vector<double> render (const Signal& s, double seconds) override
    {
        namcmp::NamWaveNet m = proto; // fresh state per render
        const int n = (int) (seconds * kFs);
        std::vector<double> y ((size_t) n);
        for (int i = 0; i < n; ++i) y[(size_t) i] = m.process ((float) s.at (i / kFs));
        return y;
    }
};

struct ModelDut : Dut
{
    Urei1176::Controls ctl;
    double inScale = 1.0;  // volts per full scale at the 600 ohm source
    double outScale = 0.1; // full scale per output volt
    int os = 2;            // model rate = os x 48 kHz
    std::vector<double> render (const Signal& s, double seconds) override
    {
        Urei1176 u;
        u.setControls (ctl);
        u.prepare (kFs * os);
        const int n = (int) (seconds * kFs);
        std::vector<double> y ((size_t) n);
        for (int i = 0; i < n; ++i)
        {
            double v = 0.0;
            for (int k = 0; k < os; ++k) v = u.processSample (inScale * s.at ((i * os + k) / (kFs * os)));
            y[(size_t) i] = outScale * v; // the sample at the shared instant (signals are analytic: no aliasing in)
        }
        return y;
    }
};

static double rms (const std::vector<double>& y, double t0, double t1)
{
    double s = 0.0; int n = 0;
    for (int i = (int) (t0 * kFs); i < (int) (t1 * kFs) && i < (int) y.size(); ++i) { s += y[(size_t) i] * y[(size_t) i]; ++n; }
    return std::sqrt (s / std::max (1, n));
}

static double db (double x) { return 20.0 * std::log10 (std::max (x, 1e-12)); }

// harmonic amplitudes (dB re the fundamental) over whole periods ending at t1
static void harmonics (const std::vector<double>& y, double f, double t1, double (&h)[6])
{
    const int periods = (int) std::floor (0.1 * f);
    const int n = (int) std::round (periods * kFs / f), i0 = (int) (t1 * kFs) - n;
    double a[6] {};
    for (int k = 1; k <= 5; ++k)
    {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const double ph = 2.0 * kPi * f * k * i / kFs;
            re += y[(size_t) (i0 + i)] * std::cos (ph); im += y[(size_t) (i0 + i)] * std::sin (ph);
        }
        a[k] = std::hypot (re, im) * 2.0 / n;
    }
    for (int k = 1; k <= 5; ++k) h[k] = db (a[k] / a[1]);
    h[1] = db (a[1] / std::sqrt (2.0)); // the fundamental: rms
}

// ------------------------------------------------------------------ measurement set
struct Measured
{
    std::vector<double> levels, gain1k, gain100, h2, h3, freqs, resp, envAttack, envRelease;
};

static Measured measure (Dut& d, bool verbose)
{
    Measured m;
    for (double lev = -60.0; lev <= 0.01; lev += 3.0) m.levels.push_back (lev);
    for (double lev : m.levels)
    {
        Signal s; s.a = std::pow (10.0, lev / 20.0); s.f = 1000.0;
        auto y = d.render (s, 0.4);
        double h[6]; harmonics (y, 1000.0, 0.4, h);
        m.gain1k.push_back (h[1] - db (s.a / std::sqrt (2.0)));
        m.h2.push_back (h[2]); m.h3.push_back (h[3]);
        s.f = 100.0;
        y = d.render (s, 0.4);
        harmonics (y, 100.0, 0.4, h);
        m.gain100.push_back (h[1] - db (s.a / std::sqrt (2.0)));
    }
    for (double f : { 30.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 15000.0, 20000.0 })
    {
        Signal s; s.a = std::pow (10.0, -50.0 / 20.0); s.f = f;
        auto y = d.render (s, 0.3);
        double h[6]; harmonics (y, f, 0.3, h);
        m.freqs.push_back (f); m.resp.push_back (h[1] - db (s.a / std::sqrt (2.0)));
    }
    if (verbose)
    {
        std::printf ("%-8s %8s %8s %8s %8s\n", "in dBFS", "g 1k", "g 100", "H2 1k", "H3 1k");
        for (size_t i = 0; i < m.levels.size(); ++i)
            std::printf ("%8.1f %8.2f %8.2f %8.1f %8.1f\n", m.levels[i], m.gain1k[i], m.gain100[i], m.h2[i], m.h3[i]);
        std::printf ("small-signal response at -50 dBFS (dB):");
        for (size_t i = 0; i < m.freqs.size(); ++i) std::printf (" %.0f:%.2f", m.freqs[i], m.resp[i]);
        std::printf ("\n");
    }
    return m;
}

// envelope (rms in 1 ms windows) of a 1 kHz burst of level lev after silence, and of a 20 dB step down after 0.3 s
static void envelopes (Dut& d, double lev, std::vector<double>& att, std::vector<double>& rel)
{
    Signal s; s.a = std::pow (10.0, lev / 20.0); s.f = 1000.0; s.t0 = 0.2;
    auto y = d.render (s, 0.26);
    att.clear();
    for (int w = 0; w < 50; ++w) att.push_back (db (rms (y, 0.2 + w * 1e-3, 0.2 + (w + 1) * 1e-3)));
    Signal r; r.a = s.a; r.f = 1000.0; r.t1 = 0.4; r.g = 0.1;
    y = d.render (r, 0.52);
    rel.clear();
    for (int w = 0; w < 24; ++w) rel.push_back (db (rms (y, 0.4 + w * 5e-3, 0.4 + (w + 1) * 5e-3)));
}

// the capture's latency (samples): peak of the response to a small impulse
static int latency (namcmp::NamWaveNet proto)
{
    int best = 0; double peak = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        const double y = std::abs (proto.process (i == 0 ? 1e-3f : 0.0f));
        if (y > peak) { peak = y; best = i; }
    }
    return best;
}

// ------------------------------------------------------------------ fitting
// What the fit compares: the 1 kHz static curve (gain vs level), and the envelopes of a burst and of a 20 dB step down
// at one level above threshold (the capture's ones shifted by its latency)
struct Target
{
    std::vector<double> levels, gain, att, rel;
    double envLevel = -10.0;
};

static std::vector<double> staticCurve (Dut& d, const std::vector<double>& levels, double f = 1000.0)
{
    std::vector<double> g;
    for (double lev : levels)
    {
        Signal s; s.a = std::pow (10.0, lev / 20.0); s.f = f;
        const auto y = d.render (s, 0.35);
        double h[6]; harmonics (y, f, 0.35, h);
        g.push_back (h[1] - db (s.a / std::sqrt (2.0)));
    }
    return g;
}

static void envelopesShifted (Dut& d, double lev, int shift, std::vector<double>& att, std::vector<double>& rel)
{
    Signal s; s.a = std::pow (10.0, lev / 20.0); s.f = 1000.0; s.t0 = 0.2;
    auto y = d.render (s, 0.25);
    const double sh = shift / kFs;
    att.clear();
    for (int w = 0; w < 30; ++w) att.push_back (db (rms (y, 0.2 + sh + w * 1e-3, 0.2 + sh + (w + 1) * 1e-3)));
    Signal r; r.a = s.a; r.f = 1000.0; r.t1 = 0.3; r.g = 0.1;
    y = d.render (r, 0.4);
    rel.clear();
    for (int w = 0; w < 16; ++w) rel.push_back (db (rms (y, 0.3 + sh + w * 5e-3, 0.3 + sh + (w + 1) * 5e-3)));
}

static double rmsDiff (const std::vector<double>& a, const std::vector<double>& b, double offset = 0.0)
{
    double s = 0.0;
    for (size_t i = 0; i < a.size(); ++i) s += (a[i] - b[i] - offset) * (a[i] - b[i] - offset);
    return std::sqrt (s / (double) a.size());
}

static double meanDiff (const std::vector<double>& a, const std::vector<double>& b)
{
    double s = 0.0;
    for (size_t i = 0; i < a.size(); ++i) s += a[i] - b[i];
    return s / (double) a.size();
}

static const char* ratioName (int r) { static const char* n[] = { "4:1", "8:1", "12:1", "20:1", "ALL" }; return n[r]; }

// static fit: for each ratio and release, the input scale (and the output offset, analytically) best matching the
// capture's 1 kHz curve
static void fitStatic (const Target& t, ModelDut& m, int& bestRatio, double& bestRel, double& bestIn, double& bestOut)
{
    double bestErr = 1e9;
    for (int ratio = 0; ratio < 5; ++ratio)
        for (double rel : { 0.0, 0.5, 1.0 })
        {
            m.ctl.ratio = ratio; m.ctl.release = rel;
            auto err = [&] (double inDb, double& off)
            {
                m.inScale = std::pow (10.0, inDb / 20.0); m.outScale = 1.0;
                const auto g = staticCurve (m, t.levels);
                off = meanDiff (g, t.gain);
                return rmsDiff (g, t.gain, off);
            };
            // coarse grid, then golden-section refinement
            double lo = -10.0, hi = 40.0, bestX = 0.0, e = 1e9, off = 0.0;
            for (double x = lo; x <= hi; x += 2.5) { const double v = err (x, off); if (v < e) { e = v; bestX = x; } }
            double a = bestX - 2.5, b = bestX + 2.5;
            for (int it = 0; it < 12; ++it)
            {
                const double c = b - 0.618 * (b - a), d = a + 0.618 * (b - a);
                if (err (c, off) < err (d, off)) b = d; else a = c;
            }
            const double x = 0.5 * (a + b);
            e = err (x, off);
            std::printf ("  %-4s release %.1f: input %+6.2f dB, output %+6.2f dB, static rms error %.2f dB\n", ratioName (ratio), rel, x, -off, e);
            if (e < bestErr) { bestErr = e; bestRatio = ratio; bestRel = rel; bestIn = x; bestOut = -off; }
        }
}

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("comp1176_nam <capture.nam> [measure | fit]\n"); return 1; }
    NamDut nam;
    if (! nam.proto.load (argv[1])) { std::printf ("cannot load %s\n", argv[1]); return 1; }
    const std::string mode = argc > 2 ? argv[2] : "measure";
    std::printf ("capture: %s (%.0f Hz)\n", argv[1], nam.proto.getSampleRate());
    if (mode == "measure")
    {
        measure (nam, true);
        for (double lev : { -30.0, -20.0, -10.0 })
        {
            std::vector<double> att, rel;
            envelopes (nam, lev, att, rel);
            std::printf ("burst %.0f dBFS, 1 ms rms (dB):", lev);
            for (int w = 0; w < 30; ++w) std::printf (" %.1f", att[(size_t) w]);
            std::printf ("\nstep -20 dB, 5 ms rms (dB):");
            for (double v : rel) std::printf (" %.1f", v);
            std::printf ("\n");
        }
    }
    else if (mode == "fit")
    {
        const int lat = latency (nam.proto);
        std::printf ("capture latency %d samples\n", lat);
        Target t;
        for (double lev = -45.0; lev <= 0.01; lev += 3.0) t.levels.push_back (lev);
        t.gain = staticCurve (nam, t.levels);
        t.envLevel = argc > 3 ? std::atof (argv[3]) : -10.0;
        envelopesShifted (nam, t.envLevel, lat, t.att, t.rel);

        ModelDut m;
        m.ctl = Urei1176::fromKnobs (10.0, 7.0, 4.0, 7.0, 0); // INPUT full: the input scale stands for it
        std::printf ("static fit (1 kHz, -45..0 dBFS):\n");
        int ratio = 0; double rel = 1.0, inDb = 0.0, outDb = 0.0;
        fitStatic (t, m, ratio, rel, inDb, outDb);
        std::printf ("best: %s release %.1f, input %+.2f dB, output %+.2f dB\n", ratioName (ratio), rel, inDb, outDb);

        // dynamics: attack and release pots at the fitted static point
        m.ctl.ratio = ratio;
        m.inScale = std::pow (10.0, inDb / 20.0); m.outScale = std::pow (10.0, outDb / 20.0);
        double bestE = 1e9, bestA = 0.5, bestR = rel;
        for (double atk : { 0.0, 0.25, 0.5, 0.75, 1.0 })
            for (double r : { 0.0, 0.25, 0.5, 0.75, 1.0 })
            {
                m.ctl.attack = atk; m.ctl.release = r;
                std::vector<double> att, relv;
                envelopesShifted (m, t.envLevel, 0, att, relv);
                const double ea = rmsDiff (att, t.att, meanDiff (att, t.att)), er = rmsDiff (relv, t.rel, meanDiff (relv, t.rel));
                std::printf ("  attack %.2f release %.2f: attack envelope %.2f dB, release envelope %.2f dB rms\n", atk, r, ea, er);
                if (ea + er < bestE) { bestE = ea + er; bestA = atk; bestR = r; }
            }
        m.ctl.attack = bestA; m.ctl.release = bestR;
        std::printf ("best dynamics: attack %.2f release %.2f (dials %.1f / %.1f)\n", bestA, bestR, 1.0 + 6.0 * bestA, 1.0 + 6.0 * bestR);

        // final comparison
        const auto g = staticCurve (m, t.levels);
        std::printf ("\n%-8s %8s %8s %8s\n", "in dBFS", "capture", "model", "diff");
        for (size_t i = 0; i < t.levels.size(); ++i) std::printf ("%8.1f %8.2f %8.2f %+8.2f\n", t.levels[i], t.gain[i], g[i], g[i] - t.gain[i]);
        std::vector<double> att, relv;
        envelopesShifted (m, t.envLevel, 0, att, relv);
        std::printf ("burst %.0f dBFS, 1 ms (dB)\n capture:", t.envLevel);
        for (double v : t.att) std::printf (" %6.1f", v);
        std::printf ("\n model:  ");
        for (double v : att) std::printf (" %6.1f", v);
        std::printf ("\nstep -20 dB, 5 ms (dB)\n capture:");
        for (double v : t.rel) std::printf (" %6.1f", v);
        std::printf ("\n model:  ");
        for (double v : relv) std::printf (" %6.1f", v);
        std::printf ("\n");
        // harmonics in compression and small-signal response
        std::printf ("\nharmonics at 1 kHz (dB re fundamental), capture / model:\n");
        for (double lev : { -30.0, -20.0, -10.0, -5.0, 0.0 })
        {
            Signal s; s.a = std::pow (10.0, lev / 20.0); s.f = 1000.0;
            double hc[6], hm[6];
            harmonics (nam.render (s, 0.4), 1000.0, 0.4, hc);
            harmonics (m.render (s, 0.4), 1000.0, 0.4, hm);
            std::printf ("%6.0f dBFS: H2 %6.1f / %6.1f   H3 %6.1f / %6.1f   H4 %6.1f / %6.1f   H5 %6.1f / %6.1f\n",
                         lev, hc[2], hm[2], hc[3], hm[3], hc[4], hm[4], hc[5], hm[5]);
        }
        std::printf ("\nsmall-signal response (-50 dBFS, dB re 1 kHz), capture / model:\n");
        double c1k = 0.0, m1k = 0.0;
        for (double f : { 1000.0, 20.0, 30.0, 50.0, 100.0, 200.0, 500.0, 2000.0, 5000.0, 10000.0, 15000.0, 20000.0 })
        {
            Signal s; s.a = std::pow (10.0, -50.0 / 20.0); s.f = f;
            double hc[6], hm[6];
            const double sec = f < 40.0 ? 0.6 : 0.3;
            harmonics (nam.render (s, sec), f, sec, hc);
            harmonics (m.render (s, sec), f, sec, hm);
            if (f == 1000.0) { c1k = hc[1]; m1k = hm[1]; continue; }
            std::printf ("  %5.0f Hz: %+6.2f / %+6.2f\n", f, hc[1] - c1k, hm[1] - m1k);
        }
    }
    return 0;
}
