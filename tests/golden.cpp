// Golden render: a fixed set of scenes through the component-level DSP, written to / compared against a file, for
// speed work that must not change the sound. Build target: golden (no JUCE).
//   golden write <file>   render and save (e.g. from the last committed sources)
//   golden check <file>   render and compare sample by sample: identical samples, max difference, residual in dB
// Scenes: the amp (preamp + power amp on its supply, 8x of 48 kHz) in every channel with knobs moving, channel
// switches, both drive pedals, circuit noise, another tube set and a hot input; the 1176 at 2x; the drives alone.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <tuple>
#include <vector>
#include "dsp/Amp5150.h"
#include "dsp/PitchShifter.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"
#include "dsp/PrecisionDrive.h"
#include "dsp/TubeSet.h"
#include "dsp/Urei1176.h"

using namespace vamp;

namespace
{
constexpr double kOs = 384000.0, kCompRate = 96000.0, kSecs = 1.5;

std::vector<double> guitarDI (double rate, double seconds, double level)
{
    std::vector<double> x ((size_t) (rate * seconds));
    unsigned rng = 12345;
    const double f[] = { 41.2, 61.7, 82.4 }; // drop-D-ish power chord: 6 chugs, then idle hiss
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = (double) i / rate, ph = std::fmod (t, 1.0);
        double v = 0;
        if (ph < 0.75)
        {
            const double env = std::exp (-std::fmod (ph, 0.125) * 30.0);
            for (double fr : f)
                for (int h = 1; h <= 6; ++h) v += env * 0.25 / h * std::sin (2 * 3.14159265358979323846 * fr * h * t + h);
        }
        rng = rng * 1664525u + 1013904223u;
        v += 0.0005 * ((rng >> 8) / 8388608.0 - 1.0);
        x[i] = level * v;
    }
    return x;
}

double tri (double t, double p) { const double f = std::fmod (t / p, 1.0); return 1.0 - std::abs (2.0 * f - 1.0); }

struct Scene { std::string name; std::vector<double> y; };

// knobs: gain bass mid treble master presence resonance; the amp updates them every 32 samples as AmpSim does
struct AmpScene
{
    const char* name;
    int channel0 = Preamp5150::Red;
    std::function<void (double t, double* k, int& channel)> automate;
    int drive = -1;       // -1 none, 0 Precision Drive, 1 TS9
    bool noise = false;
    int tubeSet = 0;
    double level = 1.0;
};

std::vector<double> renderAmp (const AmpScene& s)
{
    const auto di = guitarDI (kOs, kSecs, s.level);
    Preamp5150 pre;
    PowerAmp5150 pa;
    PrecisionDrive pd;
    TubeScreamer9 ts;
    const auto tubes = TubeSet::make (s.tubeSet);
    pre.setTubeSet (tubes);
    pa.setTubeSet (tubes);
    pre.setChannelTrims (54.0, 1.0, 1.0);
    double k[7] = { 0.6, 0.5, 0.5, 0.6, 0.3, 0.5, 0.5 };
    int ch = s.channel0;
    if (s.automate) s.automate (0.0, k, ch);
    pre.setControls (k[0], k[1], k[2], k[3], k[4]);
    pa.setControls (k[5], k[6]);
    pre.prepare (kOs);
    pa.prepare (kOs);
    pd.prepare (kOs); pd.setControls (0.6, 0.5, 3, 0.7);
    ts.prepare (kOs); ts.setControls (0.6, 0.5, 0.7);
    // settle preamp and power amp together on the shared supply (as AmpSim::prepare)
    auto settle = [&] (double seconds) { for (int i = 0; i < (int) (kOs * seconds); ++i) processAmp5150 (pre, pa, 0.0, 1.0); };
    pa.beginSupplySettle();
    for (int pass = 0; pass < 3; ++pass) { settle (0.1); pa.resetSupplyAverage(); settle (0.1); pa.applySupplyDC(); }
    settle (0.15);
    pa.endSupplySettle();
    pre.setModeNow (ch);
    pre.setNoise (s.noise);

    std::vector<double> y (di.size());
    for (size_t i = 0; i < di.size(); ++i)
    {
        if (i % 32 == 0 && s.automate)
        {
            int c = ch;
            s.automate ((double) i / kOs, k, c);
            if (c != ch) { ch = c; pre.setMode (ch); }
            pre.setControls (k[0], k[1], k[2], k[3], k[4]);
            pa.setControls (k[5], k[6]);
        }
        double v = di[i];
        if (s.drive == 0) v = pd.process (v);
        else if (s.drive == 1) v = ts.process (v);
        y[i] = processAmp5150 (pre, pa, v, 1.0);
    }
    return y;
}

std::vector<double> renderComp (Urei1176::Controls c0, const std::function<Urei1176::Controls (double)>& sweep, double level)
{
    const auto di = guitarDI (kCompRate, kSecs, level);
    Urei1176 m;
    m.setControls (c0);
    m.prepare (kCompRate);
    std::vector<double> y (di.size());
    for (size_t i = 0; i < di.size(); ++i)
    {
        if (sweep && i % 16 == 0) m.setControls (sweep ((double) i / kCompRate));
        y[i] = m.processSample (di[i]);
    }
    return y;
}

std::vector<Scene> renderAll()
{
    std::vector<Scene> out;
    const AmpScene amps[] = {
        { "RED static" },
        { "RED gain / presence / resonance moving", Preamp5150::Red,
          [] (double t, double* k, int&) { k[0] = 0.2 + 0.8 * tri (t, 0.7); k[5] = tri (t, 0.5); k[6] = tri (t, 0.9); } },
        { "BLUE tone stack + master moving", Preamp5150::Blue,
          [] (double t, double* k, int&) { k[1] = tri (t, 0.6); k[2] = tri (t, 0.8); k[3] = tri (t, 0.5); k[4] = 0.1 + 0.5 * tri (t, 1.1); } },
        { "CLEAN gain moving", Preamp5150::Clean, [] (double t, double* k, int&) { k[0] = 0.3 + 0.6 * tri (t, 0.9); } },
        { "channel switches RED BLUE CLEAN", Preamp5150::Red,
          [] (double t, double*, int& c) { const int cs[3] = { Preamp5150::Red, Preamp5150::Blue, Preamp5150::Clean }; c = cs[(int) (t / 0.25) % 3]; } },
        { "RED + Precision Drive", Preamp5150::Red, nullptr, 0 },
        { "RED + TS9, gain 10", Preamp5150::Red, [] (double, double* k, int&) { k[0] = 1.0; }, 1 },
        { "RED with circuit noise", Preamp5150::Red, nullptr, -1, true },
        { "BLUE, tube set C", Preamp5150::Blue, nullptr, -1, false, 3 },
        { "RED, hot input (+12 dB), master 0.8", Preamp5150::Red, [] (double, double* k, int&) { k[4] = 0.8; k[0] = 1.0; }, -1, false, 0, 4.0 },
    };
    for (const auto& s : amps) out.push_back ({ s.name, renderAmp (s) });

    out.push_back ({ "1176 IN 7 atk/rel 5 4:1", renderComp (Urei1176::fromKnobs (7, 8.5, 5, 5, 0), nullptr, 1.0) });
    out.push_back ({ "1176 IN 10 atk/rel 7 ALL", renderComp (Urei1176::fromKnobs (10, 8.5, 7, 7, 4), nullptr, 1.0) });
    out.push_back ({ "1176 knobs moving", renderComp (Urei1176::fromKnobs (7, 8.5, 5, 5, 1),
                     [] (double t) { return Urei1176::fromKnobs (3 + 7 * tri (t, 0.9), 8.5, 1 + 6 * tri (t, 0.7), 1 + 6 * tri (t, 1.1), 1); }, 1.0) });
    {
        const auto di = guitarDI (kOs, kSecs, 1.0);
        PrecisionDrive pd; pd.prepare (kOs); pd.setControls (1.0, 0.7, 1, 0.7);
        TubeScreamer9 ts; ts.prepare (kOs); ts.setControls (1.0, 0.3, 0.7);
        Scene a { "Precision Drive alone, drive 10", {} }, b { "TS9 alone, drive 10", {} };
        for (double v : di) { a.y.push_back (pd.process (v)); b.y.push_back (ts.process (v)); }
        out.push_back (a);
        out.push_back (b);
    }
    // the transposer at 48 kHz in 8-sample blocks (float signal path)
    for (const auto& [semis, mode, name] : { std::tuple<int, int, const char*> { -2, PitchShifter::Live, "Transpose LIVE -2" },
                                             { 5, PitchShifter::Live, "Transpose LIVE +5" },
                                             { -12, PitchShifter::Live, "Transpose LIVE -12" },
                                             { -2, PitchShifter::HQ, "Transpose HQ -2" } })
    {
        const auto di = guitarDI (48000.0, 3.0, 0.5);
        std::vector<float> buf (di.size());
        for (size_t i = 0; i < di.size(); ++i) buf[i] = (float) di[i];
        PitchShifter ps; ps.prepare (48000.0); ps.setParams (semis, mode);
        for (size_t i = 0; i + 8 <= buf.size(); i += 8) ps.process (buf.data() + i, 8);
        out.push_back ({ name, std::vector<double> (buf.begin(), buf.end()) });
    }
    return out;
}
} // namespace

int main (int argc, char** argv)
{
    if (argc < 3 || (std::string (argv[1]) != "write" && std::string (argv[1]) != "check"))
    {
        std::printf ("golden write <file> | golden check <file>\n");
        return 2;
    }
    const bool write = std::string (argv[1]) == "write";
    const auto scenes = renderAll();

    if (write)
    {
        FILE* f = std::fopen (argv[2], "wb");
        if (! f) { std::printf ("cannot write %s\n", argv[2]); return 1; }
        for (const auto& s : scenes)
        {
            const unsigned long long n = s.y.size();
            std::fwrite (&n, sizeof n, 1, f);
            std::fwrite (s.y.data(), sizeof (double), s.y.size(), f);
        }
        std::fclose (f);
        std::printf ("wrote %d scenes to %s\n", (int) scenes.size(), argv[2]);
        return 0;
    }

    FILE* f = std::fopen (argv[2], "rb");
    if (! f) { std::printf ("cannot read %s\n", argv[2]); return 1; }
    int changed = 0;
    std::printf ("%-40s %10s %12s %10s %10s\n", "scene", "identical", "max |diff|", "resid dB", "peak dB");
    for (const auto& s : scenes)
    {
        unsigned long long n = 0;
        std::vector<double> ref;
        if (std::fread (&n, sizeof n, 1, f) == 1) { ref.resize (n); if (std::fread (ref.data(), sizeof (double), n, f) != n) ref.clear(); }
        if (ref.size() != s.y.size()) { std::printf ("%-40s length differs\n", s.name.c_str()); ++changed; continue; }
        size_t same = 0;
        double maxD = 0, r2 = 0, s2 = 0, sp = 0;
        for (size_t i = 0; i < n; ++i)
        {
            same += std::memcmp (&ref[i], &s.y[i], sizeof (double)) == 0;
            const double d = s.y[i] - ref[i];
            maxD = std::max (maxD, std::abs (d));
            r2 += d * d; s2 += ref[i] * ref[i]; sp = std::max (sp, std::abs (ref[i]));
        }
        const double rdB = r2 > 0 ? 10.0 * std::log10 (r2 / s2) : -999.0, pdB = maxD > 0 ? 20.0 * std::log10 (maxD / sp) : -999.0;
        std::printf ("%-40s %9.4f%% %12.3g %10.1f %10.1f\n", s.name.c_str(), 100.0 * (double) same / (double) n, maxD, rdB, pdB);
        changed += same != n;
    }
    std::fclose (f);
    std::printf (changed == 0 ? "all scenes bit-identical\n" : "%d scene(s) differ\n", changed);
    return changed == 0 ? 0 : 1;
}
