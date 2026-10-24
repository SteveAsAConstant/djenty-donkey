#pragma once
#include <algorithm>
#include <cmath>
#include "TriodeNetwork.h"

namespace vamp_baseline
{
// The whole 5150 lead preamp (schematic pages 1-2), input jack to C58, solved as one circuit (TriodeNetwork, 6
// triodes, 12 ports). Every stage loads the ones next to it: grid current through the 470k / 1M feeds and the
// R88 feedback network pull on the previous plate, and the cold V2A sees its real source and load.
//
//   jack -> R27 22k, C20 39p -> C15 0.1u -> R25 1M, R22 68k -> V1A (R2 220k, R12 1.82k || C3 1u)
//   -> C6 22n -> R29 470k || C17 2.2n -> VR2 Ultra Pre 1M (R41 1M across, C19 1n || R40 1M top-wiper)
//   -> R82 470k -> V1B (R3 100k, R17 1.82k || C4 1u, C26 22u to ground via K1A in lead)
//   -> C2 22n -> R6 470k -> V2A grid, R9 1M -> V2A (R1 100k, R15 39k unbypassed: the cold clipper)
//   -> C1 22n -> C48 1n || R11 330k -> R7 220k -> V2B (R4 220k, R13 1.82k || C5 1u)
//   -> R32 1M -> C57 22n -> R101 100k -> V5B (R96 220k, R102 2.2k)
//   -> R87 1M -> C56 22n -> R91 1M -> V5A (R86 100k, R93 2.2k), C58 22n -> out, R88 1M out -> V5A grid
//   load on out: R89 470k -> R24 33k || (C59 10n + R94 47k) (schematic page 3)
class LeadPreamp5150
{
public:
    static constexpr int kTubes = 6, kInputs = 1 + kTubes;

    // diagnostics: compute every cathode and plate voltage (getVk / getVp; costs ~15% of the per-sample work)
    explicit LeadPreamp5150 (bool diagnostics = false)
    {
        auto& dk = net.dk;
        auto n = [&dk] { return dk.addNode(); };
        const int G = DKCircuit::GND;
        nIn = n(); nG1 = n(); nC6 = n(); nPot = n(); nW = n(); nC2 = n(); nC1 = n(); nR32 = n(); nR87 = n(); nOut = n();
        nTs = n(); nC59 = n();
        for (int t = 0; t < kTubes; ++t) net.tubeNodes (t);
        const auto& v1a = net.tube[0]; const auto& v1b = net.tube[1]; const auto& v2a = net.tube[2];
        const auto& v2b = net.tube[3]; const auto& v5b = net.tube[4]; const auto& v5a = net.tube[5];

        srcIn = dk.addSource (nIn, 22e3);         // R27 (guitar on the jack)
        dk.addC (nIn, G, 39e-12);                 // C20
        cC15 = dk.addC (nIn, nG1, 0.1e-6);        // C15
        dk.addR (nG1, G, 1e6);                    // R25
        dk.addR (nG1, v1a.g, 68e3);               // R22
        srcB[0] = dk.addSource (v1a.p, 220e3);    // R2
        dk.addR (v1a.k, G, 1.82e3);               // R12
        cK[0] = dk.addC (v1a.k, G, 1e-6);         // C3

        cC6 = dk.addC (v1a.p, nC6, 22e-9);        // C6
        dk.addR (nC6, nPot, 470e3);               // R29
        dk.addC (nC6, nPot, 2.2e-9);              // C17
        dk.addR (nPot, G, 1e6);                   // R41
        potTop = dk.addR (nPot, nW, 5e5);         // VR2 upper
        potBot = dk.addR (nW, G, 5e5);            // VR2 lower
        dk.addC (nPot, nW, 1e-9);                 // C19
        dk.addR (nPot, nW, 1e6);                  // R40

        dk.addR (nW, v1b.g, 470e3);               // R82
        srcB[1] = dk.addSource (v1b.p, 100e3);    // R3
        dk.addR (v1b.k, G, 1.82e3);               // R17
        cK[1] = dk.addC (v1b.k, G, 1e-6);         // C4
        cK2 = dk.addC (v1b.k, G, 22e-6);          // C26 (K1A, lead)

        cC2 = dk.addC (v1b.p, nC2, 22e-9);        // C2
        dk.addR (nC2, v2a.g, 470e3);              // R6
        dk.addR (v2a.g, G, 1e6);                  // R9
        srcB[2] = dk.addSource (v2a.p, 100e3);    // R1
        dk.addR (v2a.k, G, 39e3);                 // R15

        cC1 = dk.addC (v2a.p, nC1, 22e-9);        // C1
        dk.addC (nC1, G, 1e-9);                   // C48
        dk.addR (nC1, G, 330e3);                  // R11
        dk.addR (nC1, v2b.g, 220e3);              // R7
        srcB[3] = dk.addSource (v2b.p, 220e3);    // R4
        dk.addR (v2b.k, G, 1.82e3);               // R13
        cK[3] = dk.addC (v2b.k, G, 1e-6);         // C5

        dk.addR (v2b.p, nR32, 1e6);               // R32
        cC57 = dk.addC (nR32, v5b.g, 22e-9);      // C57
        dk.addR (v5b.g, G, 100e3);                // R101
        srcB[4] = dk.addSource (v5b.p, 220e3);    // R96
        dk.addR (v5b.k, G, 2.2e3);                // R102

        dk.addR (v5b.p, nR87, 1e6);               // R87
        cC56 = dk.addC (nR87, v5a.g, 22e-9);      // C56
        dk.addR (v5a.g, G, 1e6);                  // R91
        srcB[5] = dk.addSource (v5a.p, 100e3);    // R86
        dk.addR (v5a.k, G, 2.2e3);                // R93
        cC58 = dk.addC (v5a.p, nOut, 22e-9);      // C58
        dk.addR (nOut, v5a.g, 1e6);               // R88

        rLoad = dk.addR (nOut, nTs, 470e3);       // R89 (kV5ALoad: 1e12 removes the load, for comparisons)
        dk.addR (nTs, G, 33e3);                   // R24
        dk.addC (nTs, nC59, 10e-9);               // C59
        dk.addR (nC59, G, 47e3);                  // R94

        net.addMillerAndOutputs (diagnostics);
        dk.markOutput (nOut);
        net.addPorts();
    }

    // vb1: B+ of V1A, V1B, V2A; vb2: of V2B, V5B, V5A. Allocates (matrix rebuild): call from prepare.
    void configure (double vb1, double vb2, double sampleRate)
    {
        bplus[0] = bplus[1] = bplus[2] = vb1;
        bplus[3] = bplus[4] = bplus[5] = vb2;
        net.dk.setR (rLoad, v5aLoad ? 470e3 : 1e12);
        net.dk.prepare (sampleRate);

        // start at the DC operating point (grids at 0 V, no signal), so there is no power-up transient
        const double rp[kTubes] = { 220e3, 100e3, 100e3, 220e3, 220e3, 100e3 };
        const double rk[kTubes] = { 1.82e3, 1.82e3, 39e3, 1.82e3, 2.2e3, 2.2e3 };
        for (int t = 0; t < kTubes; ++t)
        {
            TriodeConfig c; c.vb = bplus[t]; c.rp = rp[t]; c.rk = rk[t];
            TriodeStage s; s.configure (c, sampleRate);
            vk[t] = s.getVk(); vp[t] = s.getVp();
        }
        net.initDC (vk, vp, sampleRate);
        auto& dk = net.dk;
        dk.setCapVoltage (cK[0], vk[0]);
        dk.setCapVoltage (cK[1], vk[1]);
        dk.setCapVoltage (cK2, vk[1]);
        dk.setCapVoltage (cK[3], vk[3]);
        dk.setCapVoltage (cC6, vp[0]);
        dk.setCapVoltage (cC2, vp[1]);
        dk.setCapVoltage (cC1, vp[2]);
        dk.setCapVoltage (cC57, vp[3]);
        dk.setCapVoltage (cC56, vp[4]);
        dk.setCapVoltage (cC58, vp[5]);
        double u[kInputs];
        inputs (0.0, u);
        net.saveIdle (kInputs, u);
        out = 0.0;
        lastIn = 0.0;
    }

    // VR2 Ultra Pre wiper fraction (from ground). Changes the circuit matrices: rebuilt (allocation-free) on the
    // next sample, so call it at most once per few dozen samples while the knob moves.
    void setGainPot (double f)
    {
        f = std::clamp (f, 0.0, 1.0);
        net.dk.setR (potTop, (1.0 - f) * 1e6);
        net.dk.setR (potBot, f * 1e6);
    }

    // input: volts at the guitar jack. output: AC voltage after C58 (into R88 and the tone stack feed)
    double process (double vin)
    {
        if (std::isfinite (vin)) lastIn = vin;
        double u[kInputs];
        inputs (lastIn, u);
        if (net.solve (u))
            out = net.dk.nodeV (nOut);
        return out;
    }

    // back to the DC operating point of configure() (channel switches; no allocation)
    void reset() { net.resetToIdle(); lastIn = 0.0; out = 0.0; }

    // supply rails per sample (V1+: V1A V1B V2A, V2+: V2B V5B V5A): they are source inputs, no rebuild
    void setRails (double v1, double v2)
    {
        bplus[0] = bplus[1] = bplus[2] = v1;
        bplus[3] = bplus[4] = bplus[5] = v2;
    }
    // current drawn from V1+ / V2+ at the last sample
    double railCurrent1() const { return net.plateCurrent (0) + net.plateCurrent (1) + net.plateCurrent (2); }
    double railCurrent2() const { return net.plateCurrent (3) + net.plateCurrent (4) + net.plateCurrent (5); }

    void setReferencePrecision (bool on) { net.setReferencePrecision (on); }

    // stage t = 0 V1A, 1 V1B, 2 V2A, 3 V2B, 4 V5B, 5 V5A: cathode and plate voltages (absolute; 0 without diagnostics)
    double getVk (int t) const { return net.dk.nodeV (net.tube[t].k); }
    double getVp (int t) const { return net.dk.nodeV (net.tube[t].p); }

    // V5A load through R89 -> R24 33k || C59 + R94 (factory schematic; the same feed as Preamp5150's tone stack, whose
    // own input impedance is left out here: a few % of the load). With it the NAM captures show ~1.6 dB more onset H2,
    // an area they cannot settle (tube-individual); the circuit stays as drawn. Takes effect at the next configure().
    bool v5aLoad = true;

    TriodeNetwork<kTubes> net; // public for the tests (statistics, testFaults)

private:
    void inputs (double vin, double (&u)[kInputs]) const
    {
        u[srcIn] = vin;
        for (int t = 0; t < kTubes; ++t) u[srcB[t]] = bplus[t];
    }

    int nIn = 0, nG1 = 0, nC6 = 0, nPot = 0, nW = 0, nC2 = 0, nC1 = 0, nR32 = 0, nR87 = 0, nOut = 0, nTs = 0, nC59 = 0;
    int srcIn = 0, srcB[kTubes] {}, potTop = 0, potBot = 0, rLoad = 0;
    int cC15 = 0, cK[kTubes] {}, cK2 = 0, cC6 = 0, cC2 = 0, cC1 = 0, cC57 = 0, cC56 = 0, cC58 = 0;

    double bplus[kTubes] {};
    double vk[kTubes] {}, vp[kTubes] {};
    double lastIn = 0.0, out = 0.0;
};

} // namespace vamp_baseline
