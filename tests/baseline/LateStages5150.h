#pragma once
#include <algorithm>
#include <cmath>
#include "TriodeNetwork.h"

namespace vamp_baseline
{
// V2B -> V5B -> V5A of the 5150 preamp (schematic page 2), solved simultaneously (TriodeNetwork):
//
//   V2A plate (open-circuit voltage + output impedance) -> C1 22n -> C48 1n || R11 330k -> R7 220k -> V2B grid
//   V2B: R4 220k, R13 1.82k || C5 1u.   -> R32 1M -> C57 22n -> R101 100k -> V5B grid
//   V5B: R96 220k, R102 2.2k.           -> R87 1M -> C56 22n -> R91 1M -> V5A grid
//   V5A: R86 100k, R93 2.2k, C58 22n -> out, R88 1M out -> V5A grid (shunt feedback)
//
// Solving the stages one after another (each fed by the previous stage's open-circuit plate through a
// Thevenin resistance, as TriodeStage does) misses how each stage loads the one before it: grid current through
// R32 / R87 and the R88 feedback network pull on the previous plate. Against LTspice with identical device
// equations (spice/5150_lead_harmonics.cir) the cascade put 8-17 dB too much H2 at the onset of clipping, the
// main source of the excess even harmonics vs the real amp (docs/nam_calibration.md).
// Used by the BLUE channel; RED solves its whole preamp at once (LeadPreamp5150).
class LateStages5150
{
public:
    static constexpr int kTubes = 3;

    LateStages5150()
    {
        auto& dk = net.dk;
        auto n = [&dk] { return dk.addNode(); };
        nA = n(); nC1 = n(); nOut = n();
        for (int t = 0; t < kTubes; ++t) net.tubeNodes (t);
        nR32 = n(); nR87 = n();
        const int G = DKCircuit::GND;
        const auto& v2b = net.tube[0]; const auto& v5b = net.tube[1]; const auto& v5a = net.tube[2];

        srcIn = dk.addSource (nA, 1.0);          // V2A plate output impedance: set in configure()
        cC1 = dk.addC (nA, nC1, 22e-9);           // C1
        dk.addC (nC1, G, 1e-9);                   // C48
        dk.addR (nC1, G, 330e3);                  // R11
        dk.addR (nC1, v2b.g, 220e3);              // R7
        srcB[0] = dk.addSource (v2b.p, 220e3);    // R4
        dk.addR (v2b.k, G, 1.82e3);               // R13
        cC5 = dk.addC (v2b.k, G, 1e-6);           // C5

        dk.addR (v2b.p, nR32, 1e6);               // R32
        cC57 = dk.addC (nR32, v5b.g, 22e-9);      // C57
        dk.addR (v5b.g, G, 100e3);                // R101
        srcB[1] = dk.addSource (v5b.p, 220e3);    // R96
        dk.addR (v5b.k, G, 2.2e3);                // R102

        dk.addR (v5b.p, nR87, 1e6);               // R87
        cC56 = dk.addC (nR87, v5a.g, 22e-9);      // C56
        dk.addR (v5a.g, G, 1e6);                  // R91
        srcB[2] = dk.addSource (v5a.p, 100e3);    // R86
        dk.addR (v5a.k, G, 2.2e3);                // R93
        cC58 = dk.addC (v5a.p, nOut, 22e-9);      // C58
        dk.addR (nOut, v5a.g, 1e6);               // R88

        net.addMillerAndOutputs();
        dk.markOutput (nOut);
        net.addPorts();
    }

    // vb: B+ of the three stages, rIn: output impedance of the driving (V2A) plate.
    // Allocates (matrix rebuild): call from prepare, not on the audio thread.
    void configure (double vb, double rIn, double sampleRate)
    {
        bplus = vb;
        net.dk.setSourceResistance (srcIn, rIn);
        net.dk.prepare (sampleRate);

        // start at the DC operating point (grids at 0 V, no signal), so there is no power-up transient
        const double rp[kTubes] = { 220e3, 220e3, 100e3 }, rk[kTubes] = { 1.82e3, 2.2e3, 2.2e3 };
        for (int t = 0; t < kTubes; ++t)
        {
            TriodeConfig c; c.vb = vb; c.rp = rp[t]; c.rk = rk[t];
            TriodeStage s; s.configure (c, sampleRate);
            vk[t] = s.getVk(); vp[t] = s.getVp();
        }
        net.initDC (vk, vp, sampleRate);
        net.dk.setCapVoltage (cC5, vk[0]);
        net.dk.setCapVoltage (cC57, vp[0]);
        net.dk.setCapVoltage (cC56, vp[1]);
        net.dk.setCapVoltage (cC58, vp[2]);
        double u[4];
        inputs (0.0, u);
        net.saveIdle (4, u);
        out = 0.0;
    }

    // input: V2A's open-circuit AC plate voltage. output: AC voltage after C58 (into R88 and the tone stack)
    double process (double vin)
    {
        if (std::isfinite (vin)) lastIn = vin;
        double u[4];
        inputs (lastIn, u);
        if (net.solve (u))
            out = net.dk.nodeV (nOut);
        return out;
    }

    // back to the DC operating point of configure() (channel switches; no allocation)
    void reset() { net.resetToIdle(); lastIn = 0.0; out = 0.0; }

    void setSupply (double vb) { bplus = vb; } // V2+ per sample (a source input, no rebuild)
    double railCurrent() const { return net.plateCurrent (0) + net.plateCurrent (1) + net.plateCurrent (2); }

    void setReferencePrecision (bool on) { net.setReferencePrecision (on); }

    // stage t = 0 (V2B), 1 (V5B), 2 (V5A): cathode and plate voltages (absolute), quiescent plate
    double getVk (int t) const { return net.dk.nodeV (net.tube[t].k); }
    double getVp (int t) const { return net.dk.nodeV (net.tube[t].p); }
    double getVpDC (int t) const { return vp[t]; }

    TriodeNetwork<kTubes> net; // public for the tests (statistics, testFaults)

private:
    void inputs (double vin, double (&u)[4]) const
    {
        u[srcIn] = vin;
        for (int b : srcB) u[b] = bplus;
    }

    int nA = 0, nC1 = 0, nOut = 0, nR32 = 0, nR87 = 0;
    int srcIn = 0, srcB[kTubes] {}, cC1 = 0, cC5 = 0, cC57 = 0, cC56 = 0, cC58 = 0;

    double bplus = 320.0;
    double vk[kTubes] {}, vp[kTubes] {};
    double lastIn = 0.0, out = 0.0;
};

} // namespace vamp_baseline
