#pragma once
#include <algorithm>
#include <cmath>
#include "TriodeNetwork.h"

namespace vamp
{
// Channels as relay positions (factory schematic sheet 1, "EVH 120 preamp bd."). K1 / K2 are energized in LEAD:
// their coils are in series with the red LED and the channel switch S2A. K4 is energized with crunch: S1A grounds its
// coil. S1 is the front panel's twin pushbutton (12 pins, two DPDT buttons): S1A / S1D are CRUNCH, S1B / S1C BRIGHT
// (S1C shorts R75; the 6505's BRIGHT and CRUNCH are independent). Values as Preamp5150::Mode.
namespace channel5150 { constexpr int Clean = 0, Crunch = 1, Lead = 2; }

// The front of the preamp, the same for every channel: input jack, V1A, C6 into BOTH pre-gain networks (the one K2A
// does not select stays connected and loads C6), the selected wiper -> R82 -> V1B with its cathode network.
//
//   jack -> R27 22k, C20 39p -> C15 0.1u -> R25 1M, R22 68k -> V1A (R2 220k, R12 1.82k || C3 1u) -> C6 22n -> N1
//   lead:   N1 -> R29 470k || C17 2.2n -> R41 1M; VR2 Ultra Pre 1M (C19 1n || R40 1M top to wiper)
//   rhythm: N1 -> K4A: R19 470k || C51 270p (at rest) or R43 2.2M || C13 470p (crunch) -> A
//           A -> R21 1M || (C10 470p + R75 2.2M; S1C shorts R75 with BRIGHT) -> B: VR1 Clean Pre 1M (C27 180p top
//           to wiper); K4B at rest: R23 39k + C63 0.1u from B to ground
//   K2A: VR2 wiper (lead) or VR1 wiper (rhythm) -> R82 470k -> V1B (R3 100k, R17 1.82k || C4 1u, C26 22u)
//   K1A at rest shorts R18 47k: C26 straight to ground in rhythm, through R18 in lead (a tighter low end)
//
// Builds into net's DK circuit; the owner has created the tube nodes, tube 0 = V1A, tube 1 = V1B.
template <int NT>
struct Front5150
{
    int srcIn = 0, srcB[2] {}, cC15 = 0, cK[2] {}, cC26 = 0, cC6 = 0;

    // every capacitor of the front by name (-1: not in this channel), to carry state across a channel switch
    enum Cap { C20, C15, C3, C6, C17, C19, C51, C13, C10, C27, C63, C4, C26, kCaps };
    int cap[kCaps] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
    int leadTop = 0, leadBot = 0, rhyTop = 0, rhyBot = 0, rR18 = -1, rR75 = 0;

    void build (TriodeNetwork<NT>& net, int channel)
    {
        auto& dk = net.dk;
        auto n = [&dk] { return dk.addNode(); };
        const int G = DKCircuit::GND;
        const bool lead = channel == channel5150::Lead, crunch = channel == channel5150::Crunch;
        const auto& v1a = net.tube[0];
        const auto& v1b = net.tube[1];
        const int nIn = n(), nG1 = n(), nN1 = n(), nP = n(), nLW = n(), nA = n(), nB = n(), nRW = n();

        srcIn = dk.addSource (nIn, 22e3, true);         // R27 (guitar on the jack)
        cap[C20] = dk.addC (nIn, G, 39e-12);      // C20
        cap[C15] = cC15 = dk.addC (nIn, nG1, 0.1e-6); // C15
        dk.addR (nG1, G, 1e6, true);                    // R25
        dk.addR (nG1, v1a.g, 68e3, true);               // R22
        srcB[0] = dk.addSource (v1a.p, 220e3, true);    // R2
        dk.addR (v1a.k, G, 1.82e3);               // R12
        cap[C3] = cK[0] = dk.addC (v1a.k, G, 1e-6); // C3
        cap[C6] = cC6 = dk.addC (v1a.p, nN1, 22e-9); // C6

        // lead pre-gain network (VR2 Ultra Pre)
        dk.addR (nN1, nP, 470e3, true);                 // R29
        cap[C17] = dk.addC (nN1, nP, 2.2e-9);     // C17
        dk.addR (nP, G, 1e6, true);                     // R41
        leadTop = dk.addR (nP, nLW, 5e5, true);         // VR2 upper
        leadBot = dk.addR (nLW, G, 5e5, true);          // VR2 lower
        cap[C19] = dk.addC (nP, nLW, 1e-9);       // C19
        dk.addR (nP, nLW, 1e6, true);                   // R40

        // rhythm pre-gain network (VR1 Clean Pre)
        if (crunch)
        {
            dk.addR (nN1, nA, 2.2e6, true);             // R43 (K4A, crunch)
            cap[C13] = dk.addC (nN1, nA, 470e-12); // C13
        }
        else
        {
            dk.addR (nN1, nA, 470e3, true);             // R19 (K4A at rest)
            cap[C51] = dk.addC (nN1, nA, 270e-12); // C51
        }
        dk.addR (nA, nB, 1e6, true);                    // R21
        const int nC10 = n();
        cap[C10] = dk.addC (nA, nC10, 470e-12);   // C10
        rR75 = dk.addR (nC10, nB, 2.2e6, true);         // R75 (S1C, BRIGHT, shorts it: setBright)
        rhyTop = dk.addR (nB, nRW, 5e5, true);          // VR1 upper
        rhyBot = dk.addR (nRW, G, 5e5, true);           // VR1 lower
        cap[C27] = dk.addC (nB, nRW, 180e-12);    // C27
        if (! crunch)
        {
            const int nC63 = n();
            dk.addR (nB, nC63, 39e3, true);             // R23 (K4B at rest)
            cap[C63] = dk.addC (nC63, G, 0.1e-6); // C63
        }

        // K2A -> R82 -> V1B
        dk.addR (lead ? nLW : nRW, v1b.g, 470e3, true); // R82
        srcB[1] = dk.addSource (v1b.p, 100e3, true);    // R3
        dk.addR (v1b.k, G, 1.82e3);               // R17
        cap[C4] = cK[1] = dk.addC (v1b.k, G, 1e-6); // C4
        if (lead)
        {
            const int nC26 = n();
            cap[C26] = cC26 = dk.addC (v1b.k, nC26, 22e-6); // C26
            rR18 = dk.addR (nC26, G, 47e3);       // R18 (K1A open in lead)
        }
        else
            cap[C26] = cC26 = dk.addC (v1b.k, G, 22e-6); // C26 (K1A shorts R18)
    }

    // pot wiper fractions from ground (VR2 lead, VR1 rhythm); both are in every channel's circuit
    void setPots (DKCircuit& dk, double leadF, double rhythmF)
    {
        leadF = std::clamp (leadF, 0.0, 1.0);
        rhythmF = std::clamp (rhythmF, 0.0, 1.0);
        dk.setR (leadTop, (1.0 - leadF) * 1e6);
        dk.setR (leadBot, leadF * 1e6);
        dk.setR (rhyTop, (1.0 - rhythmF) * 1e6);
        dk.setR (rhyBot, rhythmF * 1e6);
    }

    // BRIGHT (S1C closed: R75 shorted, C10 470p straight across R21). Changes the circuit matrices like setPots.
    void setBright (DKCircuit& dk, bool on) { dk.setR (rR75, on ? kSwitchClosed : 2.2e6); }
    static constexpr double kSwitchClosed = 0.05; // pushbutton contact

    // the state of every capacitor this channel shares with o's (same part, same value; at the same sample rate)
    template <int N2>
    void copyCaps (DKCircuit& dk, const Front5150<N2>& o, const DKCircuit& odk) const
    {
        for (int c = 0; c < kCaps; ++c)
            if (cap[c] >= 0 && o.cap[c] >= 0)
                dk.setCapState (cap[c], odk.getCapState (o.cap[c]));
    }

    // DC operating point of the front's capacitors (vk / vp: V1A, V1B cathode and plate)
    void setIdleCaps (DKCircuit& dk, const double* vk, const double* vp) const
    {
        dk.setCapVoltage (cK[0], vk[0]);
        dk.setCapVoltage (cC6, vp[0]);
        dk.setCapVoltage (cK[1], vk[1]);
        dk.setCapVoltage (cC26, vk[1]);
    }
};

// DC operating point of a cathode-biased 12AX7 stage with its grid at 0 V
inline void stageDC5150 (double vb, double rp, double rk, double sampleRate, double& vk, double& vp)
{
    TriodeConfig c; c.vb = vb; c.rp = rp; c.rk = rk;
    TriodeStage s; s.configure (c, sampleRate);
    vk = s.getVk(); vp = s.getVp();
}

// The whole 5150 preamp for one channel (LEAD = RED, or rhythm with crunch = BLUE), input jack to C58, solved as one
// circuit (TriodeNetwork, 6 triodes, 12 ports): every stage loads the ones next to it (grid current through the
// 470k / 1M feeds, the R88 feedback network, the cold V2A's real source and load).
//
//   Front5150 (V1A, both gain networks, V1B) -> C2 22n -> R6 470k -> V2A grid, R9 1M (|| R20 220k: K1B at rest, rhythm)
//   -> V2A (R1 100k, R15 39k unbypassed: the cold clipper)
//   -> C1 22n -> C48 1n || R11 330k -> R7 220k -> V2B (R4 220k, R13 1.82k || C5 1u)
//   -> R32 1M -> C57 22n -> R101 100k -> V5B (R96 220k, R102 2.2k)
//   -> R87 1M -> C56 22n -> R91 1M -> V5A (R86 100k, R93 2.2k), C58 22n -> out, R88 1M out -> V5A grid
//   load on out: R89 470k -> R24 33k || (C59 10n + R94 47k) (schematic sheet 3)
class CleanPreamp5150;

class FullPreamp5150
{
    friend class CleanPreamp5150;

public:
    static constexpr int kTubes = 6, kInputs = 1 + kTubes;

    // channel: channel5150::Lead or Crunch. diagnostics: compute every cathode and plate voltage (getVk / getVp;
    // costs ~15% of the per-sample work)
    explicit FullPreamp5150 (int channel, bool diagnostics = false) : lead (channel == channel5150::Lead)
    {
        auto& dk = net.dk;
        for (int t = 0; t < kTubes; ++t) net.tubeNodes (t);
        front.build (net, channel);
        auto n = [&dk] { return dk.addNode(); };
        const int G = DKCircuit::GND;
        const auto& v1b = net.tube[1]; const auto& v2a = net.tube[2]; const auto& v2b = net.tube[3];
        const auto& v5b = net.tube[4]; const auto& v5a = net.tube[5];
        const int nC2 = n(), nC1 = n(), nR32 = n(), nR87 = n();
        nOut = n();
        const int nTs = n(), nC59 = n();

        cC2 = dk.addC (v1b.p, nC2, 22e-9);        // C2
        dk.addR (nC2, v2a.g, 470e3, true);              // R6
        dk.addR (v2a.g, G, 1e6, true);                  // R9
        if (! lead) dk.addR (v2a.g, G, 220e3, true);    // R20 (K1B at rest: rhythm)
        srcB[2] = dk.addSource (v2a.p, 100e3, true);    // R1
        dk.addR (v2a.k, G, 39e3, true);                 // R15

        cC1 = dk.addC (v2a.p, nC1, 22e-9);        // C1
        cC48 = dk.addC (nC1, G, 1e-9);            // C48
        dk.addR (nC1, G, 330e3, true);                  // R11
        dk.addR (nC1, v2b.g, 220e3, true);              // R7
        srcB[3] = dk.addSource (v2b.p, 220e3, true);    // R4
        dk.addR (v2b.k, G, 1.82e3);               // R13
        cK3 = dk.addC (v2b.k, G, 1e-6);           // C5

        dk.addR (v2b.p, nR32, 1e6, true);               // R32
        cC57 = dk.addC (nR32, v5b.g, 22e-9);      // C57
        dk.addR (v5b.g, G, 100e3, true);                // R101
        srcB[4] = dk.addSource (v5b.p, 220e3, true);    // R96
        dk.addR (v5b.k, G, 2.2e3, true);                // R102

        dk.addR (v5b.p, nR87, 1e6, true);               // R87
        cC56 = dk.addC (nR87, v5a.g, 22e-9);      // C56
        dk.addR (v5a.g, G, 1e6, true);                  // R91
        srcB[5] = dk.addSource (v5a.p, 100e3, true);    // R86
        dk.addR (v5a.k, G, 2.2e3, true);                // R93
        cC58 = dk.addC (v5a.p, nOut, 22e-9);      // C58
        dk.addR (nOut, v5a.g, 1e6, true);               // R88

        rLoad = dk.addR (nOut, nTs, 470e3);       // R89 (v5aLoad false: 1e12, for comparisons)
        dk.addR (nTs, G, 33e3);                   // R24
        cC59 = dk.addC (nTs, nC59, 10e-9);        // C59
        dk.addR (nC59, G, 47e3);                  // R94

        net.addMillerAndOutputs (diagnostics);
        dk.markOutput (nOut);
        net.addPorts();
    }

    // vb1: B+ of V1A, V1B, V2A; vb2: of V2B, V5B, V5A. Allocates (matrix rebuild): call from prepare.
    void configure (double vb1, double vb2, double sampleRate)
    {
        for (int t = 0; t < kTubes; ++t) bplus[t] = t < 3 ? vb1 : vb2;
        auto& dk = net.dk;
        dk.setR (rLoad, v5aLoad ? 470e3 : 1e12);
        if (front.rR18 >= 0) dk.setR (front.rR18, r18);
        dk.prepare (sampleRate);

        // start at the DC operating point (grids at 0 V, no signal), so there is no power-up transient
        const double rp[kTubes] = { 220e3, 100e3, 100e3, 220e3, 220e3, 100e3 };
        const double rk[kTubes] = { 1.82e3, 1.82e3, 39e3, 1.82e3, 2.2e3, 2.2e3 };
        for (int t = 0; t < kTubes; ++t) stageDC5150 (bplus[t], rp[t], rk[t], sampleRate, vk[t], vp[t]);
        net.initDC (vk, vp, sampleRate);
        front.setIdleCaps (dk, vk, vp);
        dk.setCapVoltage (cC2, vp[1]);
        dk.setCapVoltage (cC1, vp[2]);
        dk.setCapVoltage (cK3, vk[3]);
        dk.setCapVoltage (cC57, vp[3]);
        dk.setCapVoltage (cC56, vp[4]);
        dk.setCapVoltage (cC58, vp[5]);
        double u[kInputs];
        inputs (0.0, u);
        net.saveIdle (kInputs, u);
        out = 0.0;
        lastIn = 0.0;
    }

    // VR2 Ultra Pre (lead) and VR1 Clean Pre (rhythm) wiper fractions from ground. Changes the circuit matrices:
    // rebuilt (allocation-free) on the next sample, so call it at most once per few dozen samples while a knob moves.
    void setGainPots (double leadF, double rhythmF) { front.setPots (net.dk, leadF, rhythmF); }
    // BRIGHT pushbutton (S1C). Rebuilds the matrices on the next sample, like the pots.
    void setBright (bool on) { front.setBright (net.dk, on); }
    // circuit noise (resistors' thermal noise, tubes' shot and flicker noise) on / off
    void setNoise (bool on, unsigned long long seed) { net.setNoise (on, seed); }

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

    // Channel switch RED <-> BLUE: in the amp these are the same tubes and capacitors, the relays rewire the gain
    // network, V1B's cathode and V2A's grid; every capacitor keeps its charge. So this circuit takes over the other's
    // state wherever the part is the same (the rest, e.g. C13 vs C51, from the idle point): no restart transient.
    void warmFrom (const FullPreamp5150& o)
    {
        reset();
        auto& dk = net.dk;
        front.copyCaps (dk, o.front, o.net.dk);
        const int mine[] = { cC2, cC1, cC48, cK3, cC57, cC56, cC58, cC59 };
        const int theirs[] = { o.cC2, o.cC1, o.cC48, o.cK3, o.cC57, o.cC56, o.cC58, o.cC59 };
        for (int k = 0; k < 8; ++k) dk.setCapState (mine[k], o.net.dk.getCapState (theirs[k]));
        for (int t = 0; t < kTubes; ++t) net.copyTube (t, o.net, t);
        lastIn = o.lastIn;
        out = o.out;
    }
    // CLEAN -> RED / BLUE: the front (V1A, V1B, the gain networks, C2) carries over; V2A..V5A start from idle
    inline void warmFrom (const CleanPreamp5150& o);

    // supply rails per sample (V1+: V1A V1B V2A, V2+: V2B V5B V5A): they are source inputs, no rebuild
    void setRails (double v1, double v2)
    {
        for (int t = 0; t < kTubes; ++t) bplus[t] = t < 3 ? v1 : v2;
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

    // LEAD only: R18 in series with C26 (factory: K1A open in lead). v0.5.0-beta and earlier had C26 straight to
    // ground in lead (r18 = 1), the other relay position. Takes effect at the next configure().
    double r18 = 47e3;

    TriodeNetwork<kTubes> net; // public for the tests (statistics, testFaults)

private:
    void inputs (double vin, double (&u)[kInputs]) const
    {
        u[front.srcIn] = vin;
        u[front.srcB[0]] = bplus[0];
        u[front.srcB[1]] = bplus[1];
        for (int t = 2; t < kTubes; ++t) u[srcB[t]] = bplus[t];
    }

    bool lead;
    Front5150<kTubes> front;
    int nOut = 0, rLoad = 0, srcB[kTubes] {};
    int cC2 = 0, cC1 = 0, cK3 = 0, cC57 = 0, cC56 = 0, cC58 = 0, cC48 = 0, cC59 = 0;

    double bplus[kTubes] {};
    double vk[kTubes] {}, vp[kTubes] {};
    double lastIn = 0.0, out = 0.0;
};

// The plugin's CLEAN channel ("optimized" 6505 rhythm, not the original amp): the rhythm front at rest (no crunch)
// and V1B only, V2A..V5A bypassed for headroom. V1A and V1B solved together (TriodeNetwork, 2 triodes, 4 ports).
//   Front5150 (clean) -> V1B plate -> C2 22n -> out, R6 470k -> R9 1M || R20 220k (V2A's grid network as its load)
class CleanPreamp5150
{
    friend class FullPreamp5150;

public:
    static constexpr int kTubes = 2, kInputs = 1 + kTubes;

    CleanPreamp5150()
    {
        auto& dk = net.dk;
        for (int t = 0; t < kTubes; ++t) net.tubeNodes (t);
        front.build (net, channel5150::Clean);
        const int G = DKCircuit::GND;
        nOut = dk.addNode();
        const int nG = dk.addNode();
        cC2 = dk.addC (net.tube[1].p, nOut, 22e-9); // C2
        dk.addR (nOut, nG, 470e3, true);                  // R6
        dk.addR (nG, G, 1e6, true);                       // R9
        dk.addR (nG, G, 220e3, true);                     // R20 (K1B at rest)
        net.addMillerAndOutputs (false);
        dk.markOutput (nOut);
        net.addPorts();
    }

    // vb1: B+ of V1A, V1B. Allocates (matrix rebuild): call from prepare.
    void configure (double vb1, double sampleRate)
    {
        bplus = vb1;
        net.dk.prepare (sampleRate);
        stageDC5150 (vb1, 220e3, 1.82e3, sampleRate, vk[0], vp[0]);
        stageDC5150 (vb1, 100e3, 1.82e3, sampleRate, vk[1], vp[1]);
        net.initDC (vk, vp, sampleRate);
        front.setIdleCaps (net.dk, vk, vp);
        net.dk.setCapVoltage (cC2, vp[1]);
        double u[kInputs];
        inputs (0.0, u);
        net.saveIdle (kInputs, u);
        out = 0.0;
        lastIn = 0.0;
    }

    void setGainPots (double leadF, double rhythmF) { front.setPots (net.dk, leadF, rhythmF); }
    // BRIGHT pushbutton (S1C). Rebuilds the matrices on the next sample, like the pots.
    void setBright (bool on) { front.setBright (net.dk, on); }
    // circuit noise (resistors' thermal noise, tubes' shot and flicker noise) on / off
    void setNoise (bool on, unsigned long long seed) { net.setNoise (on, seed); }

    // input: volts at the guitar jack. output: V1B's AC plate voltage after C2
    double process (double vin)
    {
        if (std::isfinite (vin)) lastIn = vin;
        double u[kInputs];
        inputs (lastIn, u);
        if (net.solve (u))
            out = net.dk.nodeV (nOut);
        return out;
    }

    void reset() { net.resetToIdle(); lastIn = 0.0; out = 0.0; }

    // RED / BLUE -> CLEAN: the front (V1A, V1B, the gain networks, C2) carries over (see FullPreamp5150::warmFrom)
    void warmFrom (const FullPreamp5150& o)
    {
        reset();
        front.copyCaps (net.dk, o.front, o.net.dk);
        net.dk.setCapState (cC2, o.net.dk.getCapState (o.cC2));
        for (int t = 0; t < kTubes; ++t) net.copyTube (t, o.net, t);
        lastIn = o.lastIn; // out: set by the next solve
    }

    void setRails (double v1) { bplus = v1; }
    double railCurrent1() const { return net.plateCurrent (0) + net.plateCurrent (1); }
    void setReferencePrecision (bool on) { net.setReferencePrecision (on); }

    TriodeNetwork<kTubes> net; // public for the tests (statistics, testFaults)

private:
    void inputs (double vin, double (&u)[kInputs]) const
    {
        u[front.srcIn] = vin;
        u[front.srcB[0]] = bplus;
        u[front.srcB[1]] = bplus;
    }

    Front5150<kTubes> front;
    int nOut = 0, cC2 = 0;
    double bplus = 305.7;
    double vk[kTubes] {}, vp[kTubes] {};
    double lastIn = 0.0, out = 0.0;
};

inline void FullPreamp5150::warmFrom (const CleanPreamp5150& o)
{
    reset();
    front.copyCaps (net.dk, o.front, o.net.dk);
    net.dk.setCapState (cC2, o.net.dk.getCapState (o.cC2)); // C2 drives the same R6 -> R9 (|| R20) network in CLEAN
    for (int t = 0; t < CleanPreamp5150::kTubes; ++t) net.copyTube (t, o.net, t);
    lastIn = o.lastIn;
}

} // namespace vamp
