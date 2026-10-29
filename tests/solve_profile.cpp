// Where the amp's solve time goes (build target solve_profile, compiled with VAMP_PROFILE): AMP RED at 8x on a
// guitar DI, cycles per sample in each phase of the 12AX7 network (TriodeNetwork::solve, all its instances) and
// the power amp (PowerAmp5150::process). The counters themselves cost ~20-40 cycles each; compare phases, not totals.
#define VAMP_PROFILE 1
#include <cmath>
#include <cstdio>
#include <vector>
#include "dsp/Amp5150.h"
#include "dsp/PowerAmp5150.h"
#include "dsp/Preamp5150.h"

using namespace vamp;

static std::vector<double> guitarDI (double rate, double seconds)
{
    constexpr double pi = 3.14159265358979323846;
    std::vector<double> x ((size_t) (rate * seconds));
    unsigned rng = 12345;
    const double f[] = { 41.2, 61.7, 82.4 }; // drop-D-ish power chord: 6 chugs, then idle hiss (as dsp_test)
    for (size_t i = 0; i < x.size(); ++i)
    {
        const double t = (double) i / rate, ph = std::fmod (t, 1.0);
        double v = 0;
        if (ph < 0.75)
        {
            const double env = std::exp (-std::fmod (ph, 0.125) * 30.0);
            for (double fr : f)
                for (int h = 1; h <= 6; ++h)
                    v += env * 0.25 / h * std::sin (2 * pi * fr * h * t + h);
        }
        rng = rng * 1664525u + 1013904223u;
        v += 0.0005 * ((rng >> 8) / 8388608.0 - 1.0);
        x[i] = v;
    }
    return x;
}

int main()
{
    constexpr double os = 8 * 48000.0;
    Preamp5150 pre;
    PowerAmp5150 pa;
    pre.setChannelTrims (54.0, 1.0, 1.0);
    pre.prepare (os);
    pre.setModeNow (Preamp5150::Red);
    pre.setControls (0.6, 0.5, 0.5, 0.6, 0.3);
    pa.setControls (0.5, 0.5);
    pa.prepare (os);
    for (int i = 0; i < (int) (0.3 * os); ++i) processAmp5150 (pre, pa, 0.0, 1.0);

    const auto di = guitarDI (os, 4.0);
    for (auto& c : prof::cycles) c = 0;
    for (auto& c : prof::calls) c = 0;
    double acc = 0;
    for (double x : di) acc += processAmp5150 (pre, pa, x, 1.0);
    const double n = (double) di.size();

    struct Row { int slot; const char* name; };
    const Row rows[] = {
        { prof::kTriSolve, "12AX7 network: whole solve" }, { prof::kTriP0, "  p0 (linear)" }, { prof::kTriEval, "  tube + grid evals" },
        { prof::kTriResidual, "  residual K i" }, { prof::kTriStep, "  port step solve" }, { prof::kTriCheck, "  fault check" },
        { prof::kTriFinish, "  finish (states, outputs)" },
        { prof::kPowProcess, "power amp: whole process" }, { prof::kPowP0, "  p0 (linear)" }, { prof::kPowEval, "  tube evals" },
        { prof::kPowStep, "  port step solve" }, { prof::kPowCheck, "  fault check" }, { prof::kPowFinish, "  finish (states, outputs)" },
        { prof::kPowSupply, "  supply step" },
    };
    std::printf ("AMP RED at 8x, %.0f samples (sum %.3g)\n%-30s %12s %10s\n", n, acc, "phase", "cycles/smp", "calls/smp");
    for (const auto& r : rows)
        std::printf ("%-30s %12.1f %10.2f\n", r.name, (double) prof::cycles[r.slot] / n, (double) prof::calls[r.slot] / n);

    // a matrix rebuild (a pot moved: Gain in the preamp, Presence / Resonance in the power amp), per phase
    auto rebuilds = [] (DKCircuit dk, const char* name) {
        for (auto& c : prof::cycles) c = 0;
        for (auto& c : prof::calls) c = 0;
        constexpr int reps = 2000;
        for (int k = 0; k < reps; ++k) { dk.markDirty(); dk.ensureBuilt(); }
        std::printf ("\n%s rebuild: N %d, ports %d, states %d, inputs %d, outputs %d\n", name, dk.numUnknowns(), dk.numPorts(),
                     dk.numStates(), dk.numInputs(), dk.numOutputs());
        const Row rr[] = { { prof::kDkRebuild, "whole rebuild" }, { prof::kDkStamp, "  stamp A" }, { prof::kDkInvert, "  invert A" },
                           { prof::kDkAinvNi, "  Ainv Ni, K" }, { prof::kDkV, "  V (node rows)" }, { prof::kDkPMO, "  P, M, O" },
                           { prof::kDkTranspose, "  transposes" }, { prof::kDkUpdate, "  pots' low-rank update" } };
        for (const auto& r : rr) std::printf ("%-30s %12.0f cycles\n", r.name, (double) prof::cycles[r.slot] / reps);
        // a pot turning: only the low-rank update (DKCircuit::markVariable)
        for (auto& c : prof::cycles) c = 0;
        for (int k = 0; k < reps; ++k) { dk.markVariableDirty(); dk.ensureBuilt(); }
        std::printf ("%-30s %12.0f cycles\n", "pot moved (update only)", (double) prof::cycles[prof::kDkUpdate] / reps);
    };
    rebuilds (pre.lead.net.dk, "preamp RED");
    rebuilds (pa.circuit(), "power amp");
    return 0;
}
