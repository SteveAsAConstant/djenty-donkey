// Compare the drive pedals (OpAmpDrive: Precision Drive, TS9) against NAM captures of real pedals.
//
//   pedal_compare <model.nam> --pedal pd|ts9 [--drive d] [--tone t] [--level l] [--attack n]
//                 [--in-dbu x] [--out-dbu y] [--fit]
//
// Knobs are dial positions 0..10 (clock positions: 7:00 = 0, 12:00 = 5, 5:00 = 10), --attack 1..6.
// Calibration as on TONE3000: the interface level in dBu is the RMS of a full-scale sine, so 1.0 at the NAM input
// = dBu -> Vpk volts at the pedal input, likewise at the output. Without --out-dbu the output level is unknown and
// only shapes are compared (one global level match, reported).
//
// 1. small-signal response: a multitone (29 tones, 1/3 octave, 31 Hz .. 16 kHz) low enough to keep our clipper
//    linear, each tone's gain from a single-bin DFT over a whole number of periods
// 2. saturation: 110 Hz and 1 kHz sines, 10 mV .. 1 V peak: fundamental out and H2..H5 re fundamental
// 3. guitar: plucked notes and a chord, 1/3-octave band levels (63 Hz .. 12.5 kHz)
// --fit grid-searches Drive and Tone (and Attack) for the best match of 1 and 2, to tell a knob-scale difference
// from a circuit difference (a circuit difference leaves a residual no knob setting removes).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>
#include "dsp/PrecisionDrive.h"
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
    for (size_t i = 0; i < x.size(); ++i) x[i] = g (i / rate);
    return x;
}

// complex amplitude of frequency f (integer Hz) over exactly one second starting at `start` seconds
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

// 1/3-octave band energies (RBJ band-pass, Q 4.318), 63 Hz .. 12.5 kHz
std::vector<double> bands (const Signal& x, double rate, double skip = 0.1)
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
        out.push_back (e / (double) (x.size() - (size_t) (skip * rate)));
    }
    return out;
}

struct Knobs { double drive = 5, tone = 5, level = 5; int attack = 4; double legCap = 0; }; // legCap (F) > 0 overrides the Attack switch

struct Rig
{
    namcmp::NamWaveNet nam;
    bool ts9 = false;
    double vIn = 4.65, vOut = 4.65; // volts per NAM full scale
    bool calibratedOut = false;
    static constexpr double rateOurs = 192000.0;

    Signal ours (const Signal& v, const Knobs& k) const
    {
        Signal y (v.size());
        if (ts9)
        {
            vamp::TubeScreamer9 p; p.prepare (rateOurs); p.setControls (k.drive / 10, k.tone / 10, k.level / 10);
            for (int i = 0; i < 20000; ++i) p.process (0.0);
            for (size_t i = 0; i < v.size(); ++i) y[i] = p.process (v[i]);
        }
        else
        {
            vamp::PrecisionDrive p; p.prepare (rateOurs);
            if (k.legCap > 0) p.OpAmpDrive::setControls (k.drive / 10, k.tone / 10, k.legCap, k.level / 10);
            else p.setControls (k.drive / 10, k.tone / 10, k.attack - 1, k.level / 10);
            for (int i = 0; i < 20000; ++i) p.process (0.0);
            for (size_t i = 0; i < v.size(); ++i) y[i] = p.process (v[i]);
        }
        return y;
    }
    Signal namRun (const Signal& v) const
    {
        namcmp::NamWaveNet m = nam;
        for (int i = 0; i < 8192; ++i) m.process (0.0f);
        Signal y (v.size());
        for (size_t i = 0; i < v.size(); ++i) y[i] = m.process ((float) (v[i] / vIn)) * vOut;
        return y;
    }
    double rateNam() const { return nam.getSampleRate(); }
};

// small-signal gain clipper-linear: the clipper's mid-band gain is 1 + Rf / rLeg
double clipperGain (bool ts9, double drive)
{
    const double taper = (std::pow (10.0, 2.0 * drive / 10) - 1.0) / 99.0;
    return ts9 ? 1.0 + (51e3 + 500e3 * taper) / 4.7e3 : 1.0 + (10e3 + 500e3 * taper) / 1e3;
}

// ---------------------------------------------------------------- 1. small-signal response
std::vector<double> multitoneFreqs()
{
    std::vector<double> f;
    for (int k = 0; k < 29; ++k) f.push_back (std::round (31.5 * std::pow (2.0, k / 3.0)));
    return f;
}

struct Response { std::vector<double> ours, nam; double perTone; }; // dB gain per tone

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

std::vector<double> responseOurs (const Rig& rig, const Knobs& k, double perTone)
{
    return gains (rig.ours (sample (multitone (perTone), Rig::rateOurs, 1.5), k), Rig::rateOurs, perTone);
}

// ---------------------------------------------------------------- 2. saturation
const double kSatLevels[] = { 0.01, 0.03, 0.1, 0.3, 1.0 };
const double kSatFreqs[] = { 110.0, 1000.0 };

struct Sat { double fund; double h[4]; }; // fundamental (dBV), H2..H5 re fundamental (dB)

Sat satOf (const Signal& y, double rate, double f)
{
    Sat s;
    const double a1 = tone (y, rate, f, 0.4);
    s.fund = db (a1);
    for (int h = 2; h <= 5; ++h) s.h[h - 2] = db (tone (y, rate, f * h, 0.4) / a1);
    return s;
}

std::vector<Sat> satOurs (const Rig& rig, const Knobs& k)
{
    std::vector<Sat> out;
    for (double f : kSatFreqs)
        for (double a : kSatLevels)
            out.push_back (satOf (rig.ours (sample ([=] (double t) { return a * std::sin (2 * kPi * f * t); }, Rig::rateOurs, 1.4), k), Rig::rateOurs, f));
    return out;
}

std::vector<Sat> satNam (const Rig& rig)
{
    std::vector<Sat> out;
    for (double f : kSatFreqs)
        for (double a : kSatLevels)
            out.push_back (satOf (rig.namRun (sample ([=] (double t) { return a * std::sin (2 * kPi * f * t); }, rig.rateNam(), 1.4)), rig.rateNam(), f));
    return out;
}

// ---------------------------------------------------------------- 3. guitar
double pluck (double t, double f0, double amp)
{
    double v = 0;
    for (int h = 1; h <= 10; ++h) v += amp / h * std::exp (-t * 3.0 * (1.0 + 0.25 * h)) * std::sin (2 * kPi * f0 * h * t + h);
    return v;
}
Gen guitar()
{
    return [] (double t)
    {
        const double notes[] = { 82.41, 110.0, 146.83, 196.0, 246.94, 329.63 };
        double v = 0;
        const double t1 = std::fmod (t, 1.0);
        const int bar = (int) t;
        if (bar < 3) v = pluck (t1, notes[bar * 2], 0.25);          // single notes
        else for (double f : notes) v += pluck (t1, f, 0.08);        // chord
        return v;
    };
}

// ---------------------------------------------------------------- cost for the fit
struct Cost { double resp, sat, offset; };

Cost compare (const std::vector<double>& ro, const std::vector<double>& rn, const std::vector<Sat>& so, const std::vector<Sat>& sn)
{
    // level offset: from the small-signal response 200 Hz .. 5 kHz (what the Level knob moves); shapes are compared
    // after it, and with a calibrated output it is reported as a Level-scale difference
    double off = 0; int n = 0;
    const auto f = multitoneFreqs();
    for (size_t i = 0; i < f.size(); ++i) if (f[i] >= 200 && f[i] <= 5000) { off += rn[i] - ro[i]; ++n; }
    off /= n;
    double r = 0; int nr = 0;
    for (size_t i = 0; i < f.size(); ++i) if (f[i] >= 40 && f[i] <= 12000) { const double d = ro[i] + off - rn[i]; r += d * d; ++nr; }
    double s = 0; int ns = 0;
    for (size_t i = 0; i < so.size(); ++i)
    {
        const double d = so[i].fund + off - sn[i].fund; s += d * d; ++ns;
        for (int h : { 1, 3 }) // H3, H5: the symmetric clipper's signature (H2 / H4 are small and capture-noise limited)
            if (sn[i].h[h] > -60) { const double e = std::clamp (so[i].h[h] - sn[i].h[h], -20.0, 20.0); s += 0.25 * e * e; ++ns; }
    }
    return { std::sqrt (r / nr), std::sqrt (s / ns), off };
}
} // namespace

int main (int argc, char** argv)
{
    if (argc < 2) { std::printf ("usage: pedal_compare <model.nam> --pedal pd|ts9 [--drive d --tone t --level l --attack n] [--in-dbu x] [--out-dbu y] [--fit]\n"); return 1; }
    Rig rig;
    if (! rig.nam.load (argv[1])) { std::printf ("cannot load %s\n", argv[1]); return 1; }
    Knobs k;
    bool fit = false;
    double inDbu = 12.6, outDbu = 0.0;
    for (int i = 2; i < argc; ++i)
    {
        const std::string a = argv[i];
        auto next = [&] { return i + 1 < argc ? std::atof (argv[++i]) : 0.0; };
        if (a == "--pedal" && i + 1 < argc) rig.ts9 = std::string (argv[++i]) == "ts9";
        else if (a == "--drive") k.drive = next();
        else if (a == "--tone") k.tone = next();
        else if (a == "--level") k.level = next();
        else if (a == "--attack") k.attack = (int) next();
        else if (a == "--leg-cap") k.legCap = next() * 1e-9; // nF
        else if (a == "--in-dbu") inDbu = next();
        else if (a == "--out-dbu") { outDbu = next(); rig.calibratedOut = true; }
        else if (a == "--fit") fit = true;
    }
    rig.vIn = dbuToVpk (inDbu);
    rig.vOut = rig.calibratedOut ? dbuToVpk (outDbu) : rig.vIn;

    std::printf ("%s  (%s, NAM %.0f Hz)  in %.1f dBu = %.2f Vpk, out %s\n", argv[1], rig.ts9 ? "TS9" : "Precision Drive", rig.rateNam(),
                 inDbu, rig.vIn, rig.calibratedOut ? (std::to_string (outDbu) + " dBu").c_str() : "unknown (level matched)");
    std::printf ("knobs: drive %.2f tone %.2f level %.2f", k.drive, k.tone, k.level);
    if (! rig.ts9) std::printf (" attack %d", k.attack);
    std::printf ("\n");

    // the multitone level keeps our clipper well inside its linear range (~0.05 V rms across the diodes)
    const double perTone = std::max (1e-4, 0.05 / clipperGain (rig.ts9, k.drive) / std::sqrt (29.0 / 2.0));
    const auto rn = gains (rig.namRun (sample (multitone (perTone), rig.rateNam(), 1.5)), rig.rateNam(), perTone);
    const auto rn4 = gains (rig.namRun (sample (multitone (perTone * 0.25), rig.rateNam(), 1.5)), rig.rateNam(), perTone * 0.25);
    const auto sn = satNam (rig);

    auto report = [&] (const Knobs& kk, const char* title)
    {
        const auto ro = responseOurs (rig, kk, perTone);
        const auto so = satOurs (rig, kk);
        const auto c = compare (ro, rn, so, sn);
        std::printf ("\n== %s: drive %.2f tone %.2f level %.2f", title, kk.drive, kk.tone, kk.level);
        if (! rig.ts9) std::printf (" attack %d", kk.attack);
        std::printf ("  | response rms %.2f dB, saturation rms %.2f dB, level offset %+.2f dB ==\n", c.resp, c.sat, c.offset);
        if (rig.calibratedOut)
        {
            // the Level dial that would remove the offset (Precision Drive: A taper, TS9: linear)
            const double t0 = rig.ts9 ? kk.level / 10 : (std::pow (10.0, 2.0 * kk.level / 10) - 1.0) / 99.0, t1 = t0 * std::pow (10.0, c.offset / 20);
            const double dial = t1 >= 1.0 ? 10.0 : rig.ts9 ? 10.0 * t1 : 5.0 * std::log10 (1.0 + 99.0 * t1);
            std::printf ("calibrated output: ours is %+.2f dB re the pedal at Level %.2f; Level %.2f%s would match\n", -c.offset, kk.level,
                         dial, t1 >= 1.0 ? " (beyond full)" : "");
        }

        std::printf ("\nsmall signal (%.2f mV per tone): gain dB (ours shifted by the offset)\n%8s %8s %8s %7s %9s\n", perTone * 1e3,
                     "Hz", "ours", "pedal", "diff", "pedal/4");
        const auto f = multitoneFreqs();
        for (size_t i = 0; i < f.size(); ++i)
            std::printf ("%8.0f %+8.2f %+8.2f %+7.2f %+9.2f\n", f[i], ro[i] + c.offset, rn[i], ro[i] + c.offset - rn[i], rn4[i]);

        std::printf ("\nsaturation: fundamental out dBV (ours shifted) and harmonics dB re fundamental, ours / pedal\n");
        std::printf ("%6s %7s | %15s | %13s %13s %13s %13s\n", "Hz", "in Vpk", "fund o/p", "H2", "H3", "H4", "H5");
        size_t i = 0;
        for (double fr : kSatFreqs)
            for (double a : kSatLevels)
            {
                const auto& o = so[i]; const auto& p = sn[i]; ++i;
                std::printf ("%6.0f %7.2f | %+7.1f %+7.1f |", fr, a, o.fund + c.offset, p.fund);
                for (int h = 0; h < 4; ++h) std::printf (" %+6.1f %+6.1f", o.h[h], p.h[h]);
                std::printf ("\n");
            }

        const auto gOurs = bands (rig.ours (sample (guitar(), Rig::rateOurs, 4.0), kk), Rig::rateOurs);
        const auto gNam = bands (rig.namRun (sample (guitar(), rig.rateNam(), 4.0)), rig.rateNam());
        double tot = 0;
        for (size_t b = 0; b < gOurs.size(); ++b) tot += 10 * std::log10 (gNam[b] / gOurs[b]);
        const double goff = tot / (double) gOurs.size();
        std::printf ("\nguitar (notes + chord, 0.25 V picks): 1/3-octave bands, ours - pedal dB (offset %+.2f dB applied)\n", goff);
        int b = 0;
        for (double fc = 63.0; fc < 13000.0; fc *= std::pow (2.0, 1.0 / 3.0), ++b)
            std::printf ("%s%6.0f:%+5.1f", b % 6 == 0 ? "\n" : "  ", fc, 10 * std::log10 (gOurs[(size_t) b] / gNam[(size_t) b]) + goff);
        std::printf ("\n");
    };

    report (k, "at the capture's knob settings");

    if (fit)
    {
        // the response is compared at the capture's multitone level; Attack first (at the capture's Drive / Bright),
        // then a coarse Drive x Tone grid, then a fine one around the best
        auto cost = [&] (const Knobs& kk) { const auto c = compare (responseOurs (rig, kk, perTone), rn, satOurs (rig, kk), sn); return c.resp + c.sat; };
        Knobs best = k; double bestCost = cost (k);
        if (! rig.ts9)
        {
            std::printf ("\nAttack scan (Drive %.2f, Bright %.2f): cost", k.drive, k.tone);
            for (int at = 1; at <= vamp::PrecisionDrive::numAttack; ++at)
            {
                Knobs kk = k; kk.attack = at;
                const double c = cost (kk);
                std::printf ("  %d:%.2f", at, c);
                if (c < bestCost) { bestCost = c; best = kk; }
            }
            std::printf ("\n");
        }
        {
            const Knobs k1 = best;
            for (double d = 0; d <= 10.001; d += 1.0)
                for (double t = 0; t <= 10.001; t += 1.0)
                {
                    Knobs kk = k1; kk.drive = d; kk.tone = t;
                    const double c = cost (kk);
                    if (c < bestCost) { bestCost = c; best = kk; }
                }
        }
        // refine
        Knobs k0 = best;
        for (double d = std::max (0.0, k0.drive - 1.0); d <= std::min (10.0, k0.drive + 1.0) + 1e-9; d += 0.25)
            for (double t = std::max (0.0, k0.tone - 1.0); t <= std::min (10.0, k0.tone + 1.0) + 1e-9; t += 0.25)
            {
                Knobs kk = k0; kk.drive = d; kk.tone = t;
                const double c = cost (kk);
                if (c < bestCost) { bestCost = c; best = kk; }
            }
        report (best, "best fit");
    }
    return 0;
}
