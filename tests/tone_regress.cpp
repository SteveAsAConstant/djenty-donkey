// Tone regression: renders the same inputs through the frozen v0.3.0 DSP (tests/baseline/, namespace
// vamp_baseline) and the current DSP (Source/dsp/, namespace vamp), then reports how far apart they are
// and how much faster the current code is.
//
//   null (audio)  RMS of (current - baseline) relative to the baseline RMS, band-limited to 20 kHz
//   null (full)   same over the whole 4x-oversampled band
//   peak err      largest sample difference relative to the baseline peak
//
// PASS = audio-band null below -60 dB (no audible difference); "exact" = below -100 dB (rounding level).
// Exit code 1 if any case fails, so this can gate optimizations.
//
// Build target: tone_regress (no JUCE). Usage: tone_regress [filter-substring]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "dsp/PrecisionDrive.h"
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

int failures = 0, cases = 0;
double timeBase = 0, timeNew = 0;

void compare (const char* name, const Signal& ref, const Signal& out, double msRef, double msOut)
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
    const double nullAudio = db (std::sqrt (sDa / std::max (sRa, 1e-30)));
    const double nullFull = db (std::sqrt (sD / std::max (sR, 1e-30)));
    const double peak = db (pkD / std::max (pkR, 1e-30));
    const bool pass = nullAudio < -60.0 && std::isfinite (nullAudio);
    ++cases;
    if (! pass) ++failures;
    timeBase += msRef; timeNew += msOut;
    std::printf ("%-44s %8.1f %8.1f %8.1f  %5.2fx  %s\n", name, nullAudio, nullFull, peak, msRef / msOut,
                 ! pass ? "FAIL" : nullAudio < -100.0 ? "exact" : "pass");
}

template <typename Fn>
Signal render (const Signal& in, Fn&& fn, double& ms)
{
    Signal out (in.size());
    const auto t0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < in.size(); ++i) out[i] = fn (in[i]);
    ms = std::chrono::duration<double, std::milli> (std::chrono::high_resolution_clock::now() - t0).count();
    return out;
}

std::string filter;
bool selected (const std::string& name) { return filter.empty() || name.find (filter) != std::string::npos; }

// ---------------------------------------------------------------- cases
struct Input { const char* name; Signal x; };

void preampCases (const std::vector<Input>& inputs)
{
    const char* modes[] = { "CLEAN", "BLUE", "RED" };
    for (int mode : { 0, 1, 2 })
        for (double gain : { 0.3, 0.7, 1.0 })
            for (const auto& in : inputs)
            {
                char name[96];
                std::snprintf (name, sizeof name, "preamp %-5s gain %.1f  %s", modes[mode], gain, in.name);
                if (! selected (name)) continue;
                vamp_baseline::Preamp5150 a; a.prepare (fs); a.setMode (mode); a.setControls (gain, 0.5, 0.5, 0.6, 0.5);
                vamp::Preamp5150 b;          b.prepare (fs); b.setMode (mode); b.setControls (gain, 0.5, 0.5, 0.6, 0.5);
                double ta, tb;
                const auto ra = render (in.x, [&] (double v) { return a.process (v); }, ta);
                const auto rb = render (in.x, [&] (double v) { return b.process (v); }, tb);
                compare (name, ra, rb, ta, tb);
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
                for (size_t i = 0; i < drive.size(); ++i) drive[i] = pre.process (in.x[i]) * 25.0;

                vamp_baseline::PowerAmp5150 a; a.setControls (pc.pres, pc.res); a.prepare (fs);
                vamp::PowerAmp5150 b;          b.setControls (pc.pres, pc.res); b.prepare (fs);
                double ta, tb;
                const auto ra = render (drive, [&] (double v) { return a.process (v); }, ta);
                const auto rb = render (drive, [&] (double v) { return b.process (v); }, tb);
                compare (name, ra, rb, ta, tb);
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
                if (! selected (name)) continue;
                vamp_baseline::PrecisionDrive a; a.prepare (fs); a.setControls (drv, 0.5, atk, 0.7);
                vamp::PrecisionDrive b;          b.prepare (fs); b.setControls (drv, 0.5, atk, 0.7);
                double ta, tb;
                const auto ra = render (in.x, [&] (double v) { return a.process (v); }, ta);
                const auto rb = render (in.x, [&] (double v) { return b.process (v); }, tb);
                compare (name, ra, rb, ta, tb);
            }
}

void chainCases (const std::vector<Input>& inputs)
{
    const char* modes[] = { "CLEAN", "BLUE", "RED" };
    for (int mode : { 0, 1, 2 })
        for (const auto& in : inputs)
        {
            char name[96];
            std::snprintf (name, sizeof name, "chain  drive+%-5s+power  %s", modes[mode], in.name);
            if (! selected (name)) continue;
            vamp_baseline::PrecisionDrive da; da.prepare (fs); da.setControls (0.5, 0.5, 2, 0.7);
            vamp_baseline::Preamp5150 pa; pa.prepare (fs); pa.setMode (mode); pa.setControls (0.8, 0.5, 0.5, 0.6, 0.6);
            vamp_baseline::PowerAmp5150 wa; wa.setControls (0.6, 0.5); wa.prepare (fs);
            vamp::PrecisionDrive db_; db_.prepare (fs); db_.setControls (0.5, 0.5, 2, 0.7);
            vamp::Preamp5150 pb; pb.prepare (fs); pb.setMode (mode); pb.setControls (0.8, 0.5, 0.5, 0.6, 0.6);
            vamp::PowerAmp5150 wb; wb.setControls (0.6, 0.5); wb.prepare (fs);
            const double post = mode == 2 ? 25.0 : 1.0; // AmpSim kChannelPost
            double ta, tb;
            const auto ra = render (in.x, [&] (double v) { return wa.process (pa.process (da.process (v)) * post); }, ta);
            const auto rb = render (in.x, [&] (double v) { return wb.process (pb.process (db_.process (v)) * post); }, tb);
            compare (name, ra, rb, ta, tb);
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

    std::printf ("%-44s %8s %8s %8s  %6s\n", "case", "null dB", "full dB", "peak dB", "speed");
    std::printf ("%-44s %8s %8s %8s  %6s\n", "", "(<20k)", "", "", "");
    preampCases (guitar);
    powerAmpCases (short_);
    driveCases (short_);
    chainCases (guitar);

    std::printf ("\n%d cases, %d failed. total time baseline %.0f ms, current %.0f ms -> %.2fx\n",
                 cases, failures, timeBase, timeNew, timeBase / std::max (timeNew, 1e-9));
    return failures > 0 ? 1 : 0;
}
