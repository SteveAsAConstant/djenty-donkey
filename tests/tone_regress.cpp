// Tone regression: renders the same inputs through
//   base   the frozen v0.5 DSP (tests/baseline/, namespace vamp_baseline; frozen 2026-10-24; the v0.3.0 one is the git tag
//          v0.3.0-baseline)
//   new    the current DSP (Source/dsp/, namespace vamp)
//   ref    the current DSP with setReferencePrecision(): 1000x tighter Newton residuals, no early
//          step acceptance, the original (untabulated) Koren formulas. This is the "true" solution
//          of the circuit equations the model defines.
// and reports, all as RMS error relative to the signal, band-limited to 20 kHz:
//   new-base   how far the current code moved from the baseline
//   base err   baseline vs ref: the baseline solver's own numerical error
//   new err    current vs ref
//   new peak   largest single-sample error of the current code relative to the signal peak
//   bands      largest 1/3-octave band level difference new vs ref, 50 Hz..16 kHz: the tone itself
//
// With the drive and RED in front of the power amp the waveform errors are dominated by the preamp's
// own Newton tolerance (~0.2 mV per node), amplified by the following stages: square-wave
// edges land a fraction of a sample apart. That shows in the nulls but not in the band levels.
//
// Verdicts: "exact" new-base < -100 dB (rounding level: an optimization that changes nothing);
// "pass" new err < -60 dB (inaudible); "FIXED" new err < -60 dB while the baseline was worse than -60 dB
// against ref (a solver fix); "as base" new err no worse than the baseline's own error.
// The gate is new vs ref (solver accuracy) and the band levels: an intended model change moves new-base but
// passes. "FAIL" otherwise, or any band off by 0.5 dB or more. Exit code 1 on any FAIL, so this gates optimizations.
//
// Build target: tone_regress (no JUCE). Usage: tone_regress [filter-substring]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "dsp/Amp5150.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "dsp/PrecisionDrive.h"
#include "baseline/Amp5150.h"
#include "baseline/PowerAmp5150.h"
#include "baseline/Preamp5150.h"
#include "baseline/PrecisionDrive.h"

namespace
{
constexpr double fs = 192000.0; // 4x of 48 kHz, the rate the plugin runs the amp at
constexpr double kPi = vamp::kPi;
using Signal = std::vector<double>;

// ---------------------------------------------------------------- test inputs (deterministic)
struct Rng
{
    unsigned s = 12345;
    double next() { s = s * 1664525u + 1013904223u; return (s >> 8) / 8388608.0 - 1.0; }
};

double pluck (double t, double f0, double decay, double amp)
{
    double v = 0;
    for (int h = 1; h <= 8; ++h)
        v += amp / h * std::exp (-t * decay * (1.0 + 0.3 * h)) * std::sin (2 * kPi * f0 * h * t + h);
    return v;
}

// drop-D palm-muted chugs, idle hiss between bursts
Signal chugs (double level, double secs = 1.5)
{
    Signal x ((size_t) (fs * secs));
    Rng r;
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = i / fs, ph = std::fmod (t, 0.75);
        double v = 0;
        if (ph < 0.5)
        {
            const double tc = std::fmod (ph, 0.125);
            for (double f : { 36.7, 55.0, 73.4 })
                v += pluck (tc, f, 25.0, level);
        }
        x[i] = v + 0.0005 * r.next();
    }
    return x;
}

// one open chord left to ring into the noise floor (tests blocking recovery and the sustain tail)
Signal chordDecay (double level, double secs = 2.0)
{
    Signal x ((size_t) (fs * secs));
    Rng r;
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = i / fs;
        double v = 0;
        for (double f : { 82.4, 123.5, 164.8, 207.7, 246.9, 329.6 })
            v += pluck (t, f, 1.5, level / 3.0);
        x[i] = v + 0.0005 * r.next();
    }
    return x;
}

// logarithmic sine sweep 30 Hz .. 12 kHz
Signal sweep (double level, double secs = 1.5)
{
    Signal x ((size_t) (fs * secs));
    const double f0 = 30.0, f1 = 12000.0, k = std::log (f1 / f0);
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = i / fs;
        x[i] = level * std::sin (2 * kPi * f0 * secs / k * (std::exp (t / secs * k) - 1.0));
    }
    return x;
}

// single notes picked harder and harder (0.02 .. 1 V)
Signal dynamics (double secs = 2.0)
{
    Signal x ((size_t) (fs * secs));
    Rng r;
    const int notes = 8;
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = i / fs;
        const int n = std::min (notes - 1, (int) (t / secs * notes));
        const double tn = t - n * secs / notes;
        const double amp = 0.02 * std::pow (50.0, n / (notes - 1.0));
        x[i] = pluck (tn, 110.0, 6.0, amp) + 0.0005 * r.next();
    }
    return x;
}

// ---------------------------------------------------------------- comparison
struct LP20k // 4th-order Butterworth at 20 kHz (two biquads), double precision
{
    double b[2][3], a[2][3], z[2][2] {};
    LP20k()
    {
        const double qs[2] = { 0.54119610, 1.30656296 };
        for (int s = 0; s < 2; ++s)
        {
            const double w0 = 2 * kPi * 20000.0 / fs, al = std::sin (w0) / (2 * qs[s]), c = std::cos (w0);
            const double a0 = 1 + al;
            b[s][0] = (1 - c) / 2 / a0; b[s][1] = (1 - c) / a0; b[s][2] = (1 - c) / 2 / a0;
            a[s][1] = -2 * c / a0; a[s][2] = (1 - al) / a0;
        }
    }
    double process (double x)
    {
        for (int s = 0; s < 2; ++s)
        {
            const double y = b[s][0] * x + z[s][0];
            z[s][0] = b[s][1] * x - a[s][1] * y + z[s][1];
            z[s][1] = b[s][2] * x - a[s][2] * y;
            x = y;
        }
        return x;
    }
};

double db (double ratio) { return 20.0 * std::log10 (std::max (ratio, 1e-12)); }

struct Diff { double audio, full, peak; };

Diff difference (const Signal& ref, const Signal& out)
{
    LP20k lpR, lpD;
    double sR = 0, sD = 0, sRa = 0, sDa = 0, pkR = 0, pkD = 0;
    for (size_t i = 0; i < ref.size(); ++i)
    {
        const double d = out[i] - ref[i];
        sR += ref[i] * ref[i]; sD += d * d;
        const double ra = lpR.process (ref[i]), da = lpD.process (d);
        sRa += ra * ra; sDa += da * da;
        pkR = std::max (pkR, std::abs (ref[i])); pkD = std::max (pkD, std::abs (d));
    }
    return { db (std::sqrt (sDa / std::max (sRa, 1e-30))), db (std::sqrt (sD / std::max (sR, 1e-30))),
             db (pkD / std::max (pkR, 1e-30)) };
}

// Largest 1/3-octave band level difference (dB) between two signals, 50 Hz .. 16 kHz: the long-term
// spectrum, i.e. the tone, independent of sample-exact waveform alignment
double bandDifference (const Signal& ref, const Signal& out)
{
    double worst = 0.0;
    for (double fc = 50.0; fc < 16500.0; fc *= std::pow (2.0, 1.0 / 3.0))
    {
        // RBJ band-pass (0 dB peak), Q of a 1/3-octave band
        const double w0 = 2 * kPi * fc / fs, al = std::sin (w0) / (2 * 4.318), c = std::cos (w0), a0 = 1 + al;
        const double b0 = al / a0, b2 = -al / a0, a1 = -2 * c / a0, a2 = (1 - al) / a0;
        auto energy = [&] (const Signal& x)
        {
            double z1 = 0, z2 = 0, e = 0;
            for (double v : x)
            {
                const double y = b0 * v + z1;
                z1 = -a1 * y + z2;
                z2 = b2 * v - a2 * y;
                e += y * y;
            }
            return e;
        };
        const double er = energy (ref), eo = energy (out);
        if (er > 1e-20 * ref.size()) // skip empty bands
            worst = std::max (worst, std::abs (10.0 * std::log10 (std::max (eo, 1e-300) / er)));
    }
    return worst;
}

using Proc = std::function<double (double)>;

Signal render (const Signal& in, Proc fn, double& ms)
{
    Signal out (in.size());
    const auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < in.size(); ++i) out[i] = fn (in[i]);
    ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
    return out;
}

std::string filter;
bool selected (const std::string& name) { return filter.empty() || name.find (filter) != std::string::npos; }

int failures = 0, cases = 0, fixedCases = 0, movedCases = 0;
double timeBase = 0, timeNew = 0;

// makeBase / makeNew / makeRef return a freshly prepared processor on every call
void runCase (const std::string& name, const Signal& in, const std::function<Proc()>& makeBase,
              const std::function<Proc()>& makeNew, const std::function<Proc()>& makeRef)
{
    if (! selected (name)) return;
    double ta, tb, tr;
    const auto ra = render (in, makeBase(), ta);
    const auto rb = render (in, makeNew(), tb);
    const auto rr = render (in, makeRef(), tr);
    const double moved = difference (ra, rb).audio;
    const double baseErr = difference (rr, ra).audio;
    const auto newErr = difference (rr, rb);
    const double bands = bandDifference (rr, rb);

    const bool waveOk = newErr.audio < -60.0 || newErr.audio <= baseErr + 3.0;
    const bool ok = std::isfinite (newErr.audio) && waveOk && bands < 0.5;
    const bool fixed = ok && newErr.audio < -60.0 && baseErr >= -60.0;
    ++cases;
    if (! ok) ++failures;
    if (fixed) ++fixedCases;
    if (moved >= -100.0) ++movedCases;
    timeBase += ta; timeNew += tb;
    const char* verdict = ! ok ? "FAIL" : fixed ? "FIXED" : moved < -100.0 ? "exact" : newErr.audio < -60.0 ? "pass" : "as base";
    std::printf ("%-44s %8.1f %8.1f %8.1f %8.1f %7.3f  %5.2fx  %s\n", name.c_str(), moved, baseErr, newErr.audio,
                 newErr.peak, bands, ta / tb, verdict);
}

// ---------------------------------------------------------------- cases
struct Input { const char* name; Signal x; };

// Cases are written in v0.3 knob positions. v0.4 recalibrated the Gain / Master curves, so the current
// code gets the knob positions that give the same pot settings (the plugin converts old sessions the same way).
// baseline and current take the same knob values (same knob curves since v0.5; the v0.3.0 baseline needed
// gainKnobFromV03 / masterKnobFromV03 on the current side)
template <typename P> void preampControls (P& p, double g, double b, double m, double t, double post) { p.setControls (g, b, m, t, post); }

template <typename T> auto setReference (T& t, int) -> decltype (t.setReferencePrecision (true), void()) { t.setReferencePrecision (true); }
template <typename T> void setReference (T&, long) {} // no Newton tolerances to tighten

template <typename T, typename Setup>
std::function<Proc()> maker (Setup setup, bool reference = false)
{
    return [setup, reference]
    {
        auto obj = std::make_shared<T>();
        if (reference) setReference (*obj, 0);
        setup (*obj);
        return Proc ([obj] (double v) { return obj->process (v); });
    };
}

void preampCases (const std::vector<Input>& inputs)
{
    const char* modes[] = { "CLEAN", "BLUE", "RED" };
    for (int mode : { 0, 1, 2 })
        for (double gain : { 0.3, 0.7, 1.0 })
            for (const auto& in : inputs)
            {
                char name[96];
                std::snprintf (name, sizeof name, "preamp %-5s gain %.1f  %s", modes[mode], gain, in.name);
                auto setup = [mode, gain] (auto& p) { p.prepare (fs); p.setMode (mode); preampControls (p, gain, 0.5, 0.5, 0.6, 0.5); };
                runCase (name, in.x, maker<vamp_baseline::Preamp5150> (setup), maker<vamp::Preamp5150> (setup), maker<vamp::Preamp5150> (setup, true));
            }
}

void powerAmpCases (const std::vector<Input>& inputs)
{
    struct Pc { double pres, res; const char* tag; } pcs[] = { { 0.5, 0.5, "P5 R5" }, { 1.0, 0.0, "P10 R0" }, { 0.0, 1.0, "P0 R10" } };
    for (double master : { 0.3, 0.6, 0.9 })
        for (const auto& pc : pcs)
            for (const auto& in : inputs)
            {
                char name[96];
                std::snprintf (name, sizeof name, "power  master %.1f %-6s  %s", master, pc.tag, in.name);
                if (! selected (name)) continue;
                // identical drive signal for both: the baseline RED preamp at this master setting
                vamp_baseline::Preamp5150 pre; pre.prepare (fs); pre.setMode (2); pre.setControls (0.7, 0.5, 0.5, 0.6, master);
                Signal drive (in.x.size());
                for (size_t i = 0; i < drive.size(); ++i) drive[i] = pre.process (in.x[i]);

                auto setup = [pc] (auto& p) { p.setControls (pc.pres, pc.res); p.prepare (fs); };
                runCase (name, drive, maker<vamp_baseline::PowerAmp5150> (setup), maker<vamp::PowerAmp5150> (setup), maker<vamp::PowerAmp5150> (setup, true));
            }
}

void driveCases (const std::vector<Input>& inputs)
{
    for (double drv : { 0.3, 1.0 })
        for (int atk : { 0, 7 })
            for (const auto& in : inputs)
            {
                char name[96];
                std::snprintf (name, sizeof name, "drive  %.1f attack %d  %s", drv, atk, in.name);
                auto setup = [drv, atk] (auto& p) { p.prepare (fs); p.setControls (drv, 0.5, atk, 0.7); };
                runCase (name, in.x, maker<vamp_baseline::PrecisionDrive> (setup), maker<vamp::PrecisionDrive> (setup), maker<vamp::PrecisionDrive> (setup, true));
            }
}

// preamp -> power amp on the power amp's supply rails (Amp5150.h)
inline double chainStep (vamp_baseline::Preamp5150& p, vamp_baseline::PowerAmp5150& w, double v, double post) { return vamp_baseline::processAmp5150 (p, w, v, post); }
inline double chainStep (vamp::Preamp5150& p, vamp::PowerAmp5150& w, double v, double post) { return vamp::processAmp5150 (p, w, v, post); }

template <typename Drive, typename Pre, typename Power>
std::function<Proc()> chainMaker (int mode, bool reference = false)
{
    return [mode, reference]
    {
        auto d = std::make_shared<Drive>(); d->prepare (fs); d->setControls (0.5, 0.5, 2, 0.7);
        auto p = std::make_shared<Pre>();   if (reference) setReference (*p, 0); p->prepare (fs); p->setMode (mode); preampControls (*p, 0.8, 0.5, 0.5, 0.6, 0.6);
        auto w = std::make_shared<Power>(); if (reference) setReference (*w, 0); w->setControls (0.6, 0.5); w->prepare (fs);
        const double post = mode == 0 ? 54.0 : 1.0; // AmpSim kChannelPost (CLEAN has only 2 stages)
        return Proc ([d, p, w, post] (double v) { return chainStep (*p, *w, d->process (v), post); });
    };
}

void chainCases (const std::vector<Input>& inputs)
{
    const char* modes[] = { "CLEAN", "BLUE", "RED" };
    for (int mode : { 0, 1, 2 })
        for (const auto& in : inputs)
        {
            char name[96];
            std::snprintf (name, sizeof name, "chain  drive+%-5s+power  %s", modes[mode], in.name);
            runCase (name, in.x,
                     chainMaker<vamp_baseline::PrecisionDrive, vamp_baseline::Preamp5150, vamp_baseline::PowerAmp5150> (mode),
                     chainMaker<vamp::PrecisionDrive, vamp::Preamp5150, vamp::PowerAmp5150> (mode),
                     chainMaker<vamp::PrecisionDrive, vamp::Preamp5150, vamp::PowerAmp5150> (mode, true));
        }
}
} // namespace

int main (int argc, char** argv)
{
    if (argc > 1) filter = argv[1];

    const std::vector<Input> guitar = {
        { "chugs",       chugs (0.15) },
        { "chord decay", chordDecay (0.3) },
        { "dynamics",    dynamics() },
        { "sweep 50mV",  sweep (0.05) },
    };
    const std::vector<Input> short_ = { { "chugs", chugs (0.15) }, { "dynamics", dynamics() } };

    std::printf ("%-44s %8s %8s %8s %8s %7s  %6s\n", "case", "new-base", "base err", "new err", "new peak", "bands", "speed");
    std::printf ("%-44s %8s %8s %8s %8s %7s  %6s\n", "", "dB", "dB", "dB", "dB", "max dB", "");
    preampCases (guitar);
    powerAmpCases (short_);
    driveCases (short_);
    chainCases (guitar);

    std::printf ("\n%d cases, %d failed, %d moved from the v0.5 baseline (new-base above -100 dB), %d fixed.\n"
                 "total time baseline %.0f ms, current %.0f ms -> %.2fx\n",
                 cases, failures, movedCases, fixedCases, timeBase, timeNew, timeBase / std::max (timeNew, 1e-9));
    return failures > 0 ? 1 : 0;
}
