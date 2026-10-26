// UREI 1176LN Rev D model (Source/dsp/Urei1176.h) against the LTspice reference (spice/1176ln_revd.inc and the
// 1176_*.cir test benches, which run the factory test procedures) and the factory specifications.
// Build target: comp1176_test (no JUCE).  comp1176_test [spice | cpu | all]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "dsp/Urei1176.h"

using namespace vamp;

static constexpr double kPi = 3.14159265358979323846;
static double dBu (double vrms) { return 20.0 * std::log10 (vrms / 0.7746); }
static double peakFromDbu (double lev) { return 0.7746 * std::sqrt (2.0) * std::pow (10.0, lev / 20.0); }

struct Run
{
    std::vector<double> out, ctrl;
    double fs = 0.0;
    double rms (double t0, double t1) const
    {
        double s = 0.0; int n = 0;
        for (int i = (int) (t0 * fs); i < (int) (t1 * fs) && i < (int) out.size(); ++i) { s += out[(size_t) i] * out[(size_t) i]; ++n; }
        return std::sqrt (s / std::max (1, n));
    }
    double ctrlAt (double t) const { return ctrl[(size_t) std::min ((int) (t * fs), (int) ctrl.size() - 1)]; }
};

// tone of peak amplitude amp (EMF) from tb for bdur, x stepg from tstep (the benches' Bin source)
static Run run (const Urei1176::Controls& ctl, double fs, double seconds, double freq, double amp,
                double tb = 0.0, double bdur = 1e9, double tstep = 1e9, double stepg = 1.0, bool reference = true)
{
    Urei1176 u;
    u.setControls (ctl);
    u.prepare (fs);
    u.setReferencePrecision (reference);
    Run r; r.fs = fs;
    const int n = (int) (seconds * fs);
    r.out.resize ((size_t) n); r.ctrl.resize ((size_t) n);
    for (int i = 0; i < n; ++i)
    {
        const double t = i / fs;
        const double v = (t >= tb && t < tb + bdur) ? amp * std::sin (2.0 * kPi * freq * (t - tb)) * (t < tstep ? 1.0 : stepg) : 0.0;
        r.out[(size_t) i] = u.processSample (v);
        r.ctrl[(size_t) i] = u.controlVoltage();
    }
    return r;
}

static double thd (const std::vector<double>& x, double fs, double f0, double t0, double t1, int harmonics = 9)
{
    // DFT at the harmonics over an integer number of periods
    const int i0 = (int) (t0 * fs);
    const int periods = (int) ((t1 - t0) * f0);
    const int n = (int) std::round (periods * fs / f0);
    double h1 = 0.0, hs = 0.0;
    for (int h = 1; h <= harmonics; ++h)
    {
        double re = 0.0, im = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const double ph = 2.0 * kPi * f0 * h * i / fs;
            re += x[(size_t) (i0 + i)] * std::cos (ph);
            im += x[(size_t) (i0 + i)] * std::sin (ph);
        }
        const double a = std::hypot (re, im) * 2.0 / n;
        if (h == 1) h1 = a; else hs += a * a;
    }
    return 100.0 * std::sqrt (hs) / h1;
}

static Urei1176::Controls benchControls (double inputDb, double output, double attack, double release, int ratio, bool grOn)
{
    Urei1176::Controls c;
    c.inputDb = inputDb; c.output = output; c.attack = attack; c.release = release; c.ratio = ratio; c.grOn = grOn;
    return c;
}

// The OUTPUT pot of the benches is linear (outpos); the model's is audio taper: the rotation giving that fraction
int idleDebug();
int dcDebug();
int rateNull();
int metalStudy();
int worstBlock();
int controlStress();
int hfResponse();
int retryFailure();
static double outputForFraction (double frac) { return std::log10 (1.0 + 99.0 * frac) / 2.0; }

static void spiceParity (double fs, bool reference = true)
{
    std::printf ("\n== 1176 vs LTspice (internal rate %.0f kHz) ==\n", fs / 1000.0);

    // Q bias (1176_qbias.cir): G.R. off, 2 kHz, 5 mV EMF peak; LTspice: FET off 0.302199 V, calibrated -1.0 dB
    {
        const auto r = run (benchControls (0.0, 1.0, 1.0, 1.0, Urei1176::r20, false), fs, 0.12, 2000.0, 5e-3);
        const double v = r.rms (0.101, 0.119);
        std::printf ("Q bias: out %.6f V rms (LTspice 0.269336), drop %.3f dB vs FET off (LTspice -1.000)\n", v, 20.0 * std::log10 (v / 0.302199));
    }

    // static curves (1176_static.cir), output dBu per ratio; LTspice values
    static const double spice[5][9] = {
        { -12.36, -7.36, -2.36, 2.64, 7.64, 12.64, 17.52, 19.85, 20.87 },
        { -12.36, -7.36, -2.36, 2.64, 7.64, 12.64, 17.63, 20.95, 21.56 },
        { -12.36, -7.36, -2.36, 2.64, 7.64, 12.64, 17.63, 21.64, 22.10 },
        { -12.36, -7.36, -2.36, 2.64, 7.64, 12.64, 17.63, 22.63, 25.53 },
        { -11.37, -6.37, -1.37, 3.63, 8.63, 13.63, 18.63, 21.23, 21.53 } };
    const char* names[5] = { "4:1", "8:1", "12:1", "20:1", "all" };
    std::printf ("static curves (2 kHz, pots full CW, attack / release fastest): output dBu, C++ - LTspice\n");
    std::printf ("%-5s", "EMF");
    for (int l = 0; l < 9; ++l) std::printf (" %7d", -50 + 5 * l);
    std::printf ("\n");
    double worst = 0.0;
    for (int ratio = 0; ratio < 5; ++ratio)
    {
        std::printf ("%-5s", names[ratio]);
        for (int l = 0; l < 9; ++l)
        {
            const double lev = -50.0 + 5.0 * l;
            const auto r = run (benchControls (0.0, 1.0, 1.0, 1.0, ratio, true), fs, 1.2, 2000.0, peakFromDbu (lev), 0.0, 1e9, 1e9, 1.0, reference);
            const double d = dBu (r.rms (1.1, 1.2)) - spice[ratio][l];
            worst = std::max (worst, std::abs (d));
            std::printf (" %+7.2f", d);
        }
        std::printf ("\n");
    }
    std::printf ("worst |C++ - LTspice| %.2f dB (LTspice table to 0.01 dB)\n", worst);
}

// attack (1176_attack.cir): 20:1, OUTPUT 0.3, 0 dBu EMF 2 kHz burst after silence; CTRL (V) after the burst start
static void attackParity (double fs, bool reference = true)
{
    static const double tms[] = { 0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.7, 1, 1.5, 2, 3, 5, 10, 19.9 };
    static const double spice[3][14] = { // ATTACK 0, 0.5, 1
        { -1.45266, -1.45257, -1.45255, -1.45260, -1.44185, -1.34934, -1.34958, -1.26319, -1.20082, -1.16694, -1.13791, -1.12018, -1.11277, -1.11106 },
        { -1.45266, -1.45257, -1.45255, -1.45260, -1.43178, -1.27224, -1.27281, -1.16876, -1.13831, -1.12571, -1.11558, -1.11009, -1.10809, -1.10665 },
        { -1.45266, -1.45257, -1.45255, -1.45260, -1.25204, -1.09533, -1.09661, -1.09729, -1.09924, -1.10043, -1.10103, -1.10081, -1.10019, -1.09891 } };
    std::printf ("\nattack, CTRL after a 2 kHz burst (20:1, ~10 dB G.R.), C++ - LTspice (mV), %.0f kHz\n", fs / 1000.0);
    std::printf ("%-7s", "ms");
    for (double t : tms) std::printf (" %6.2f", t);
    std::printf ("\n");
    const double atk[3] = { 0.0, 0.5, 1.0 };
    for (int a = 0; a < 3; ++a)
    {
        const auto r = run (benchControls (0.0, outputForFraction (0.3), atk[a], 1.0, Urei1176::r20, true), fs, 0.3205,
                            2000.0, peakFromDbu (0.0), 0.3, 20e-3, 1e9, 1.0, reference);
        std::printf ("atk %.1f", atk[a]);
        for (int i = 0; i < 14; ++i) std::printf (" %+6.1f", 1e3 * (r.ctrlAt (0.3 + tms[i] * 1e-3) - spice[a][i]));
        std::printf ("\n");
    }
}

// LN trim (1176_ln.cir): 1 kHz, 4:1, ~8 dB G.R., slowest release, OUTPUT 0.4: LTspice 2.82140 V rms, THD 0.030 %
// release (1176_release.cir): 20:1, OUTPUT 0.3, 0 dBu EMF 2 kHz at ~10 dB G.R., 20 dB down at 0.5 s; CTRL (V)
// and the output (V rms over 2 ms) after the step, RELEASE 0 / 0.5 / 1
static void releaseParity (double fs, bool reference = true)
{
    static const double ts[] = { 0.01, 0.03, 0.06, 0.1, 0.2, 0.4, 0.7, 1.0, 1.5, 2.0, 3.0, 4.4 };
    static const double sc[3][12] = {
        { -1.18591, -1.19349, -1.20420, -1.21735, -1.24541, -1.28617, -1.32377, -1.34645, -1.36960, -1.38492, -1.40608, -1.42486 },
        { -1.18874, -1.20001, -1.21567, -1.23449, -1.27297, -1.32427, -1.36564, -1.38733, -1.40693, -1.41868, -1.43319, -1.44377 },
        { -1.23091, -1.29818, -1.36137, -1.40511, -1.43804, -1.44532, -1.44782, -1.44933, -1.45080, -1.45159, -1.45234, -1.45267 } };
    static const double sv[3][12] = {
        { 0.43994, 0.44686, 0.45564, 0.47135, 0.50542, 0.57326, 0.67216, 0.76691, 0.91519, 1.04838, 1.25821, 1.43584 },
        { 0.44479, 0.45774, 0.47569, 0.50361, 0.56824, 0.69302, 0.85925, 0.99873, 1.18092, 1.31244, 1.47060, 1.56815 },
        { 0.52686, 0.67940, 0.87703, 1.05109, 1.21951, 1.32412, 1.41987, 1.48470, 1.55043, 1.58643, 1.61885, 1.63243 } };
    std::printf ("\nrelease after 10 dB G.R. (20:1, 2 kHz, input 20 dB down at t = 0), %.0f kHz: CTRL C++ - LTspice (mV) | output (dB)\n", fs / 1000.0);
    std::printf ("%-7s", "s");
    for (double t : ts) std::printf (" %6.2f", t);
    std::printf ("\n");
    const double rel[3] = { 0.0, 0.5, 1.0 };
    for (int k = 0; k < 3; ++k)
    {
        const auto r = run (benchControls (0.0, outputForFraction (0.3), 1.0, rel[k], Urei1176::r20, true), fs, 4.95,
                            2000.0, peakFromDbu (0.0), 0.0, 1e9, 0.5, 0.1, reference);
        std::printf ("rel %.1f", rel[k]);
        for (int i = 0; i < 12; ++i) std::printf (" %+6.1f", 1e3 * (r.ctrlAt (0.5 + ts[i]) - sc[k][i]));
        std::printf ("\n       ");
        for (int i = 0; i < 12; ++i) std::printf (" %+6.2f", 20.0 * std::log10 (r.rms (0.5 + ts[i], 0.5 + ts[i] + 2e-3) / sv[k][i]));
        std::printf ("\n");
    }
}

static void lnParity (double fs, bool reference = true)
{
    const auto r = run (benchControls (0.0, outputForFraction (0.4), 1.0, 0.0, Urei1176::r4, true), fs, 0.6, 1000.0, peakFromDbu (-10.0), 0.0, 1e9, 1e9, 1.0, reference);
    std::printf ("\nLN null (1 kHz, 4:1, ~8 dB G.R.): out %.5f V rms (LTspice 2.82140), THD %.4f %% (LTspice 0.0296 %%), %.0f kHz\n",
                 r.rms (0.55, 0.6), thd (r.out, fs, 1000.0, 0.55, 0.6), fs / 1000.0);
}

// CPU: a 20:1 compressing burst sequence, model rate fs; % of one core at that rate
static void cpu (double fs)
{
    Urei1176 u;
    u.setControls (benchControls (-10.0, 0.5, 0.8, 0.6, Urei1176::r4, true));
    u.prepare (fs);
    const int n = (int) (4.0 * fs);
    double sink = 0.0;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i)
    {
        const double t = i / fs;
        const double env = std::fmod (t, 0.5) < 0.25 ? 1.0 : 0.05; // picked notes
        sink += u.processSample (env * 0.5 * std::sin (2.0 * kPi * 110.0 * t) + env * 0.2 * std::sin (2.0 * kPi * 880.0 * t));
    }
    const double sec = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
    const auto& a = u.netA(); const auto& b = u.netB(); const auto& c = u.netC();
    std::printf ("CPU at %.0f kHz: %.2f us/sample = %.1f %% of one core (Newton evals/sample A %.2f B %.2f C %.2f, failures %lld)%s\n",
                 fs / 1000.0, 1e6 * sec / n, 100.0 * sec / 4.0,
                 (double) a.statIterations / a.statSamples, (double) b.statIterations / b.statSamples,
                 (double) c.statIterations / c.statSamples, a.statFailures + b.statFailures + c.statFailures, sink == 1234.5 ? "" : "");
    std::printf ("  failures A %lld B %lld C %lld, held A %lld B %lld C %lld\n", a.statFailures, b.statFailures, c.statFailures, a.statHeld, b.statHeld, c.statHeld);
}

int main (int argc, char** argv)
{
    const std::string mode = argc > 1 ? argv[1] : "spice";
    if (mode == "idle") return idleDebug();
    if (mode == "dc") return dcDebug();
    if (mode == "null") return rateNull();
    if (mode == "metal") return metalStudy();
    if (mode == "worst") return worstBlock();
    if (mode == "stress") return controlStress();
    if (mode == "hf") return hfResponse();
    if (mode == "retryfail") return retryFailure();
    if (mode == "release") { releaseParity (96000.0, false); return 0; }
    if (mode == "spice" || mode == "all")
    {
        spiceParity (384000.0);
        attackParity (384000.0);
        lnParity (384000.0);
        releaseParity (96000.0, false);
    }
    if (mode == "rates" || mode == "all")
        for (double fs : { 96000.0, 192000.0 }) { spiceParity (fs, false); attackParity (fs, false); lnParity (fs, false); }
    if (mode == "cpu" || mode == "all")
        for (double fs : { 88200.0, 96000.0, 176400.0, 192000.0 }) cpu (fs);
    return 0;
}

// debug: output and control voltage with no input after prepare (should stay at rest)
int idleDebug()
{
    const auto r = run (benchControls (0.0, 1.0, 1.0, 1.0, Urei1176::r20, true), 96000.0, 2.0, 1000.0, 0.0);
    for (double t : { 0.0, 1e-4, 1e-3, 0.01, 0.03, 0.1, 0.3, 1.0, 1.99 })
        std::printf ("t %7.4f  out %+.6e  ctrl %.6f\n", t, r.out[(size_t) (t * r.fs)], r.ctrlAt (t));
    return 0;
}

int dcDebug()
{
    Urei1176 u;
    u.setControls (benchControls (0.0, 1.0, 1.0, 1.0, Urei1176::r20, true));
    u.prepare (96000.0);
    const char* nb[] = { "w", "b4", "c4", "e4", "l", "b5", "c6", "x1", "b6", "e6", "wbw", "n32", "x2", "slt", "x3", "brn", "sltw", "blu", "yel", "redw", "v30" };
    std::printf ("DC converged: A %d B %d C %d\n", u.netA().dcConverged, u.netB().dcConverged, u.netC().dcConverged);
    for (int i = 0; i < u.netB().numDcNodes() && i < 21; ++i) std::printf ("B %-5s %10.5f\n", nb[i], u.netB().dcNode (i));
    return 0;
}

// Guitar-like pick sequence (band-limited sawtooth notes with a pick transient, exponential decay, palm-muted
// chugs) at the model rates fs / 2fs / 4fs: the output at the shared sample instants, difference to the 4fs run
// (dB below the signal), and the gain reduction envelope difference (5 ms windows)
static double pickInput (double t)
{
    static const double notes[] = { 82.41, 82.41, 82.41, 110.0, 82.41, 98.0, 82.41, 123.47 };
    const double step = 0.125; // 16ths at 120 bpm
    const int k = (int) (t / step);
    const double tn = t - k * step, f = notes[k % 8];
    const double decay = (k % 4 == 3) ? 3.0 : 25.0; // every 4th note rings, the rest are chugs
    double s = 0.0;
    for (int h = 1; h <= 40 && h * f < 12000.0; ++h) s += std::sin (2.0 * kPi * h * f * tn) / h;
    return 0.6 * s * std::exp (-decay * tn) * (1.0 - std::exp (-tn / 0.0005));
}

int rateNull()
{
    for (double atk : { 1.0, 0.6 })
    {
        const double base = 96000.0, seconds = 2.0;
        std::vector<double> outs[3];
        for (int r = 0; r < 3; ++r)
        {
            const double fs = base * (1 << r);
            Urei1176 u;
            u.setControls (benchControls (-6.0, 0.25, atk, 0.8, Urei1176::r20, true));
            u.prepare (fs);
            const int n = (int) (seconds * fs);
            outs[r].resize ((size_t) n);
            for (int i = 0; i < n; ++i) outs[r][(size_t) i] = u.processSample (pickInput (i / fs));
        }
        const int n = (int) (seconds * base);
        double sig = 0.0, d1 = 0.0, d2 = 0.0, envMax1 = 0.0, envMax2 = 0.0;
        const int win = (int) (0.005 * base);
        for (int w = 0; w + win <= n; w += win)
        {
            double e0 = 0.0, e1 = 0.0, e2 = 0.0;
            for (int i = w; i < w + win; ++i)
            {
                const double y0 = outs[2][(size_t) (4 * i)], y1 = outs[0][(size_t) i], y2 = outs[1][(size_t) (2 * i)];
                e0 += y0 * y0; e1 += y1 * y1; e2 += y2 * y2;
                sig += y0 * y0; d1 += (y1 - y0) * (y1 - y0); d2 += (y2 - y0) * (y2 - y0);
            }
            if (e0 > 1e-6 * win)
            {
                envMax1 = std::max (envMax1, std::abs (10.0 * std::log10 (e1 / e0)));
                envMax2 = std::max (envMax2, std::abs (10.0 * std::log10 (e2 / e0)));
            }
        }
        std::printf ("attack %.1f, 20:1 heavy: 96k vs 384k: difference %.1f dB below the signal, envelope (5 ms) worst %.2f dB;"
                     " 192k: %.1f dB, %.2f dB\n", atk, 10.0 * std::log10 (sig / d1), envMax1, 10.0 * std::log10 (sig / d2), envMax2);
    }
    return 0;
}

// ---------------------------------------------------------------------------------------------------------------
// Metal guitar study: the plugin's knob mapping (Comp1176::toControls), the model at 96 kHz (the plugin's 2x)
static Urei1176::Controls knobs (double input, double output, double attack, double release, int ratio)
{
    return Urei1176::fromKnobs (input, output, attack, release, ratio);
}

// a plucked note: a sawtooth whose harmonic h decays (decay + 40 h) / s faster (the bright pick dulls within
// milliseconds, as a real string), so the first ms of each note has the pick's crest
static double plucked (double tn, double f, double decay, double amp)
{
    double s = 0.0;
    for (int h = 1; h <= 60 && h * f < 15000.0; ++h) s += std::sin (2.0 * kPi * h * f * tn) / h * std::exp (-40.0 * h * tn);
    return amp * s * std::exp (-decay * tn);
}

static double gainDb (const Urei1176::Controls& c, double freq, double peak)
{
    const double fs = 96000.0;
    const auto r = run (c, fs, 0.6, freq, peak, 0.0, 1e9, 1e9, 1.0, false);
    return 20.0 * std::log10 (Urei1176::kReampPad * r.rms (0.5, 0.6) / (peak / std::sqrt (2.0)));
}

int metalStudy()
{
    const double fs = 96000.0;
    std::printf ("\n== 1176 for metal guitar (model at 96 kHz; levels in volts at the guitar jack, 1 V = 0 dBFS) ==\n");

    // 1. small-signal gain through the reamp pad at Input 7, by Output (the default is unity below threshold)
    for (double out : { 6.0, 6.5, 7.0, 7.5, 8.0 })
        std::printf ("Input 7, Output %.1f: small-signal gain %+.2f dB\n", out, gainDb (knobs (7.0, out, 5.0, 5.0, 0), 1000.0, 0.01));

    // 2. static curve, 4:1 and 20:1 and ALL, Input 6 / 10, 220 Hz (A3 region of a riff)
    std::printf ("\nstatic gain reduction (dB) vs input peak (V), 220 Hz, attack 5, release 5:\n%-18s", "");
    const double peaks[] = { 0.05, 0.1, 0.2, 0.35, 0.5, 0.7, 1.0 };
    for (double p : peaks) std::printf (" %6.2f V", p);
    std::printf ("\n");
    for (int ratio : { 0, 3, 4 })
        for (double in : { 5.0, 7.0, 9.0 })
        {
            const double g0 = gainDb (knobs (in, 7.0, 5.0, 5.0, ratio), 220.0, 0.005);
            std::printf ("%-5s Input %4.1f   ", ratio == 0 ? "4:1" : ratio == 3 ? "20:1" : "ALL", in);
            for (double p : peaks) std::printf (" %8.2f", g0 - gainDb (knobs (in, 7.0, 5.0, 5.0, ratio), 220.0, p));
            std::printf ("\n");
        }

    // 3. pick transients: crest factor (peak / rms over each note) of input and output, per attack setting, and
    //    the first 2 ms of each note relative to its body (how much pick comes through)
    std::printf ("\npicked E2 notes (8ths at 120 bpm, pick transient ~1 V peak), 4:1, Input 8, release 5:\n");
    for (double atk : { 1.0, 3.0, 5.0, 7.0 })
    {
        Urei1176 u;
        u.setControls (knobs (7.0, 4.4, atk, 5.0, 0));
        u.prepare (fs);
        const int n = (int) (3.0 * fs), note = (int) (0.25 * fs);
        double pickIn = 0, bodyIn = 0, pickOut = 0, bodyOut = 0, grMax = 0, grMin = 1e9;
        for (int i = 0; i < n; ++i)
        {
            const double x = plucked ((i % note) / fs, 82.41, 6.0, 0.55), y = u.processSample (x);
            const int k = i % note;
            if (i >= note * 4)
            {
                if (k < (int) (0.002 * fs)) { pickIn = std::max (pickIn, std::abs (x)); pickOut = std::max (pickOut, std::abs (y)); }
                else if (k > (int) (0.010 * fs) && k < (int) (0.030 * fs)) { bodyIn = std::max (bodyIn, std::abs (x)); bodyOut = std::max (bodyOut, std::abs (y)); }
                grMax = std::max (grMax, u.gainReductionDb()); grMin = std::min (grMin, u.gainReductionDb());
            }
        }
        std::printf ("attack %.0f: pick peak / body peak  in %+.2f dB  out %+.2f dB   (G.R. %.1f .. %.1f dB)\n", atk,
                     20.0 * std::log10 (pickIn / bodyIn), 20.0 * std::log10 (pickOut / bodyOut), grMin, grMax);
    }

    // 4. low notes under heavy compression: THD of a sustained note, release 7 / 4 / 1, 4:1 and ALL
    std::printf ("\nTHD of a sustained note at ~10 dB G.R. (attack 5), by release:\n");
    for (int ratio : { 0, 4 })
        for (double f : { 41.2, 55.0, 110.0, 440.0 })
        {
            std::printf ("%-4s %6.1f Hz:", ratio == 0 ? "4:1" : "ALL", f);
            for (double rel : { 7.0, 4.0, 1.0 })
            {
                const auto r = run (knobs (9.0, 7.0, 5.0, rel, ratio), fs, 1.5, f, 0.7, 0.0, 1e9, 1e9, 1.0, false);
                Urei1176 probe; (void) probe;
                std::printf ("  rel %.0f %6.2f %%", rel, thd (r.out, fs, f, 1.0, 1.5, 9));
            }
            std::printf ("\n");
        }

    // 5. pumping on 16th chugs: G.R. at each pick vs just before the next one
    std::printf ("\nchugs (E2, 16ths at 120 bpm, 0.6 V), 4:1 Input 8 attack 5: G.R. range over a bar, by release\n");
    for (double rel : { 7.0, 5.0, 3.0, 1.0 })
    {
        Urei1176 u;
        u.setControls (knobs (8.0, 7.0, 5.0, rel, 0));
        u.prepare (fs);
        double lo = 1e9, hi = 0;
        for (int i = 0; i < (int) (3.0 * fs); ++i)
        {
            const double t = i / fs, tn = std::fmod (t, 0.125);
            double s = 0.0;
            for (int h = 1; h <= 40; ++h) s += std::sin (2.0 * kPi * h * 82.41 * tn) / h;
            u.processSample (0.6 * s * std::exp (-25.0 * tn) * (1.0 - std::exp (-tn / 0.0005)));
            if (t > 1.0) { lo = std::min (lo, u.gainReductionDb()); hi = std::max (hi, u.gainReductionDb()); }
        }
        std::printf ("release %.0f: G.R. %.1f .. %.1f dB (swing %.1f dB per note)\n", rel, lo, hi, hi - lo);
    }
    return 0;
}

// worst block: picked notes at the plugin's model rate in 2x-oversampled 64-sample host blocks (128 model samples),
// time-critical thread priority as a host's audio thread (Windows), as % of the block's real time
int worstBlock()
{
    const double fs = 96000.0;
    const int block = 128, blocks = (int) (8.0 * fs) / block;
    for (int ratio : { 0, 4 })
    {
        Urei1176 u;
        u.setControls (Urei1176::fromKnobs (9.0, 8.5, 7.0, 6.0, ratio));
        u.prepare (fs);
        double worst = 0.0, total = 0.0;
        int i = 0;
        for (int b = 0; b < blocks; ++b)
        {
            const auto t0 = std::chrono::steady_clock::now();
            for (int k = 0; k < block; ++k, ++i) u.processSample (pickInput (i / fs));
            const double sec = std::chrono::duration<double> (std::chrono::steady_clock::now() - t0).count();
            total += sec;
            if (b > 10) worst = std::max (worst, sec);
        }
        const double blockTime = block / fs;
        std::printf ("%s, heavy G.R.: average %.1f %%, worst 64-sample host block %.1f %% of its real time"
                     " (extra A/C passes per sample %.3f, Newton evals per sample A %.2f B %.2f C %.2f)\n",
                     ratio == 0 ? "4:1" : "ALL", 100.0 * total / (blocks * blockTime), 100.0 * worst / blockTime,
                     (double) u.statPasses / u.statSamples, (double) u.netA().statIterations / u.netA().statSamples,
                     (double) u.netB().statIterations / u.netB().statSamples, (double) u.netC().statIterations / u.netC().statSamples);
        std::printf ("   most evals in a sample A %lld B %lld C %lld, samples past 8 evals A %lld B %lld C %lld\n",
                     u.netA().statMaxIterations, u.netB().statMaxIterations, u.netC().statMaxIterations,
                     u.netA().statSlow, u.netB().statSlow, u.netC().statSlow);
    }
    return 0;
}

// knob / switch changes while playing: random controls every ~1 ms for 6 s of picked notes; the output must stay
// finite and bounded, the solver must not hold samples
int controlStress()
{
    const double fs = 96000.0;
    Urei1176 u;
    u.setControls (Urei1176::fromKnobs (7.0, 8.5, 5.0, 5.0, 0));
    u.prepare (fs);
    unsigned long long rng = 12345;
    auto rnd = [&rng] { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (double) (rng % 1000000) / 1e6; };
    double peak = 0.0;
    bool finite = true;
    for (int i = 0; i < (int) (6.0 * fs); ++i)
    {
        if (i % 96 == 0)
            u.setControls (Urei1176::fromKnobs (10.0 * rnd(), 10.0 * rnd(), 1.0 + 6.0 * rnd(), 1.0 + 6.0 * rnd(), (int) (5.0 * rnd())));
        const double y = u.processSample (2.0 * pickInput (i / fs)); // up to ~2 V peaks
        finite = finite && std::isfinite (y);
        peak = std::max (peak, std::abs (y));
    }
    const long long held = u.netA().statHeld + u.netB().statHeld + u.netC().statHeld;
    const long long failures = u.netA().statFailures + u.netB().statFailures + u.netC().statFailures;
    std::printf ("control stress: finite %s, output peak %.1f V, failed solves %lld, held samples %lld -> %s\n",
                 finite ? "yes" : "NO", peak, failures, held, finite && held == 0 && peak < 60.0 ? "PASS" : "FAIL");
    return finite && held == 0 ? 0 : 1;
}

// A / C retries of the coupled solve forced to fail (leaving a non-finite iterate behind, as a failed Newton can):
// the failed pass must not reach the capacitor states. Against an undisturbed run the output must stay finite,
// no sample may be held, and the difference stays small (a failed pass only ends that sample's CTRL iteration early).
int retryFailure()
{
    const double fs = 96000.0;
    for (int ratio : { 0, (int) Urei1176::rAll })
    {
        Urei1176 ref, u;
        for (auto* x : { &ref, &u }) { x->setControls (Urei1176::fromKnobs (9.0, 8.5, 7.0, 6.0, ratio)); x->prepare (fs); }
        const int n = (int) (2.0 * fs);
        double sumRef = 0.0, sumDiff = 0.0, maxDiff = 0.0;
        bool finite = true;
        for (int i = 0; i < n; ++i)
        {
            if (i % 61 == 0) u.testFailRetries (1, 0);
            if (i % 67 == 0) u.testFailRetries (0, 1);
            const double x = pickInput (i / fs), yr = ref.processSample (x), y = u.processSample (x);
            finite = finite && std::isfinite (y);
            if (i < n / 10) continue;
            sumRef += yr * yr;
            sumDiff += (y - yr) * (y - yr);
            maxDiff = std::max (maxDiff, std::abs (y - yr));
        }
        const long long failed = u.netA().statRetryFailures + u.netC().statRetryFailures;
        const long long held = u.netA().statHeld + u.netB().statHeld + u.netC().statHeld;
        const double errDb = 10.0 * std::log10 ((sumDiff + 1e-300) / sumRef);
        const bool pass = finite && held == 0 && failed > 100 && errDb < -40.0;
        std::printf ("retry failures, %s: %lld failed passes (A %lld C %lld), finite %s, held %lld, difference %.1f dB"
                     " (peak %.2g V) -> %s\n", ratio == 0 ? "4:1" : "ALL", failed, u.netA().statRetryFailures,
                     u.netC().statRetryFailures, finite ? "yes" : "NO", held, errDb, maxDiff, pass ? "PASS" : "FAIL");
        if (! pass) return 1;
    }
    return 0;
}

// small-signal frequency response at 384 kHz, pots full CW, G.R. off (factory test 8: -3 dB at ~55 kHz)
int hfResponse()
{
    const double fs = 384000.0;
    auto level = [&] (double f)
    {
        const auto r = run (benchControls (-20.0, 1.0, 1.0, 1.0, Urei1176::r4, false), fs, 0.25, f, 0.01, 0.0, 1e9, 1e9, 1.0, false);
        return 20.0 * std::log10 (r.rms (0.2, 0.25));
    };
    const double ref = level (1000.0);
    for (double f : { 20.0, 10000.0, 20000.0, 30000.0, 40000.0, 55000.0, 70000.0, 100000.0, 150000.0 })
        std::printf ("%7.0f Hz: %+6.2f dB\n", f, level (f) - ref);
    return 0;
}
