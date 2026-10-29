#pragma once
#include <algorithm>
#include <cmath>
#include "SemiNetwork.h"
#include "PotTapers.h"

namespace vamp
{
// UREI 1176LN Rev D limiting amplifier at component level: the factory schematic (S/N 1239-2331), transcribed in
// docs/1176_schematic_notes.md, the same circuit and device equations as the LTspice reference
// spice/1176ln_revd.inc. Voltages in and out are real volts (the plugin's 1 V = 0 dBFS).
//
// Three DK circuits (SemiNetwork), split where one part drives the next through a high impedance:
//   A  600 ohm source, bridged-T INPUT attenuator, T1, R5 + Q1 (the FET), the LN gate network, the Q2/Q3/Q14
//      preamp, C7, the OUTPUT pot and the ratio ladder as loads (8 ports)
//   B  the class-A line amp Q4 / Q5-Q6 with T2 and its feedback windings, 600 ohm load (7 ports)
//   C  ratio ladder (deck A), the G.R. control amp Q7-Q10, the CR2 / CR3 rectifier, threshold string (deck B),
//      "Q" bias, ATTACK / RELEASE and C22 (10 ports)
// A -> B: the OUTPUT pot wiper as its exact Thevenin source (A carries the pot as a load). A -> C: the preamp
// output O2 drives the ladder; A's load model of the ladder is corrected by the real ladder current one sample
// later. C -> A: the control voltage CTRL drives the gate network through R7 || C6 one sample later, and C sees
// the gate voltage through the same R7 || C6. The parts each crossing ties are 2.2 Mohm / 56k+ against nodes
// driven by emitter followers and C22, so the one-sample delays change nothing audible (LTspice parity, dsp_test).
class Urei1176
{
public:
    enum Ratio { r4 = 0, r8, r12, r20, rAll };

    struct Controls
    {
        double inputDb = -20.0; // INPUT: bridged-T attenuation (0 = full CW)
        double output = 0.5;    // OUTPUT pot rotation 0..1 (audio taper)
        double attack = 1.0;    // ATTACK rotation 0..1, 1 = full CW = fastest
        double release = 0.5;   // RELEASE rotation 0..1, 1 = full CW = fastest
        int ratio = r4;
        bool grOn = true;       // false: ATTACK full CCW (S9, G.R. disabled)
    };

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        computeValues (target); // the builders (and the DC twins) use the current control values
        a.prepare ([this] (SemiNetwork& n) { buildA (n); }, fs);
        b.prepare ([this] (SemiNetwork& n) { buildB (n); }, fs);
        c.prepare ([this] (SemiNetwork& n) { buildC (n); }, fs);
        // the predictor (SemiNetwork::begin) helps on smooth signals, but in the output stage B and the sidechain C it
        // overshoots through a transient (an attack turning the rectifiers on): there they start from the last solution
        // after a sample that took more than 2 evaluations (dsp_test iter8 compbudget: guitar DI IN 10 / ALL, worst
        // 16-sample block 382 -> 335 evaluations, IN 7 / 4:1 170 -> 155; sine bursts unchanged, bit for bit)
        a.predictorAllowed = predictorOn[0]; b.predictorAllowed = predictorOn[1]; c.predictorAllowed = predictorOn[2];
        a.predictorGate = predictorGate[0]; b.predictorGate = predictorGate[1]; c.predictorGate = predictorGate[2];
        pending = false;
        controlPhase = 0;
        initOperatingPoint();
    }

    void setControls (const Controls& ctl) { target = ctl; pending = true; }

    // The plugin's front panel: INPUT 0..10 (10 = full CW, 0 = 36 dB down), OUTPUT 0..10 (pot rotation), ATTACK /
    // RELEASE dials 1..7 (7 = fastest), ratio 0..3 = 4 / 8 / 12 / 20:1, 4 = all buttons in
    static constexpr double kInputRangeDb = 36.0;
    static Controls fromKnobs (double input, double output, double attack, double release, int ratio)
    {
        Controls c;
        c.inputDb = -kInputRangeDb * (1.0 - std::clamp (input, 0.0, 10.0) / 10.0);
        c.output = std::clamp (output, 0.0, 10.0) / 10.0;
        c.attack = (std::clamp (attack, 1.0, 7.0) - 1.0) / 6.0;
        c.release = (std::clamp (release, 1.0, 7.0) - 1.0) / 6.0;
        c.ratio = std::clamp (ratio, 0, (int) rAll);
        c.grOn = true;
        return c;
    }

    // The 1176 is a line-level unit. On a guitar it sits in a reamp loop, as in a studio: a DI box + mic preamp bring
    // the guitar up to line level in front of it (kDiGain, applied to the input by Comp1176), a reamp box's pad
    // brings it back to instrument level for the amp (kReampPad, on the output). The two cancel below the threshold;
    // without the DI gain the guitar reached the 1176 ~20 dB below where it compresses (INPUT had to be near 10).
    static constexpr double kDiGain = 10.0;   // +20 dB
    static constexpr double kReampPad = 0.1;  // -20 dB

    // tests: LTspice-tight Newton tolerance
    void setReferencePrecision (bool on) { a.setReferencePrecision (on); b.setReferencePrecision (on); c.setReferencePrecision (on); }

    // the real-time iteration bound (see processSample): off by default. Not usable yet: B (the output stage) has
    // rare samples that need far more evaluations, and a sample the bank drops jumps the output (dsp_test iter8 comp)
    bool bankOn = false;
    int bankPerSample = 16, bankCap = 96;   // A + C evaluations: refill per sample, cap (tests may change them)
    int bankBPerSample = 4, bankBCap = 32;  // B likewise

    // one sample: source EMF (V) -> output across the 600 ohm load (V)
    double processSample (double vin)
    {
        if (pending && ++controlPhase >= kControlEvery) { controlPhase = 0; applyControls(); }

        // Real-time bound: A and C share an iteration bank refilled by bankPerSample per sample up to bankCap, B
        // (the output stage, solved last) has its own, so any run of n samples costs at most cap + n * refill
        // Newton evaluations (16 samples: 64 + 16 * 12 for A + C). Off for the reference renders.
        struct Banks
        {
            Urei1176& u;
            explicit Banks (Urei1176& o) : u (o)
            {
                if (! u.bankOn) return;
                u.bankAC = std::min (u.bankAC + u.bankPerSample, u.bankCap);
                u.bankB = std::min (u.bankB + u.bankBPerSample, u.bankBCap);
                u.a.bank = u.c.bank = &u.bankAC;
                u.b.bank = &u.bankB;
            }
            ~Banks() { u.a.bank = u.b.bank = u.c.bank = nullptr; }
        } banks (*this);

        // A and C close the gain-reduction loop (gate -> preamp -> sidechain -> CTRL -> gate): solved together by
        // iterating the two within the sample until CTRL agrees (one pass while CTRL only drifts, a few while the
        // rectifier conducts), so the loop has no sample delay even at the fastest attack
        double ua[4] = { vin, kV30, ctrl + (ctrl - ctrlPrev), -iLadderErr * kNortonR };
        double uc[4] = { 0.0, kV30, kVm10, 0.0 };
        if (! a.begin (ua)) { a.hold(); return lastOut; }
        double vo2 = a.dk.nodeV (nO2), vg = a.dk.nodeV (nG);
        uc[0] = vo2; uc[3] = vg;
        if (! c.begin (uc)) { a.hold(); c.hold(); return lastOut; }
        // fixed point CTRL = C (A (CTRL)) by the secant method on g (x) = C (A (x)) - x (a plain repeat first)
        double ctrlNew = c.dk.nodeV (nCtrl);
        double xPrev = ua[2], gPrev = ctrlNew - ua[2];
        for (int it = 0; it < kMaxCoupled && std::abs (ctrlNew - ua[2]) > kCtrlTol; ++it)
        {
            const double x = ua[2], gx = ctrlNew - x;
            double next = ctrlNew;
            if (it > 0 && std::abs (gx - gPrev) > 1e-15)
                next = x - gx * (x - xPrev) / (gx - gPrev);
            xPrev = x; gPrev = gx;
            ua[2] = next;
            ++statPasses;
            if (! a.retry (ua)) break; // keeps the last solutions of both
            vo2 = a.dk.nodeV (nO2); vg = a.dk.nodeV (nG);
            uc[0] = vo2; uc[3] = vg;
            if (! c.retry (uc)) break;
            ctrlNew = c.dk.nodeV (nCtrl);
        }
        ++statSamples;
        a.commit();
        c.commit();
        ctrlPrev = ctrl;
        ctrl = ctrlNew;
        vGate = vg;
        iLadderErr = (vo2 - c.dk.nodeV (nTa)) / kR78 - vo2 / kLadderR;

        double ub[2] = { vo2 * wiperFrac, kV30 };
        if (b.solve (ub)) lastOut = b.dk.nodeV (nBlu) - b.dk.nodeV (nSlt);
        return lastOut;
    }

    // back to the settled idle point (nothing allocated: audio thread)
    void reset()
    {
        a.resetToIdle(); b.resetToIdle(); c.resetToIdle();
        ctrl = ctrlPrev = ctrlIdle;
        vGate = gateIdle;
        iLadderErr = 0.0;
        lastOut = 0.0;
    }

    // Gain reduction (dB, >= 0) as the 1176's meter shows it in GR mode: read from the control voltage. The FET's
    // channel conductance at Vds = 0 for that gate voltage (the model's own equation), against the idle point, in
    // the R5 divider; the LN feedback puts half the drain voltage across the channel, so it acts at half its value.
    double gainReductionDb() const
    {
        auto g = [] (double vgs)
        {
            const double s2 = 2.0 * kQ1.n * SemiNetwork::kVt, t = (vgs - kQ1.vto) / s2;
            if (t > 30.0) return 2.0 * kQ1.beta * s2 * t;
            const double e = std::exp (t);
            return 2.0 * kQ1.beta * s2 * std::log1p (e) * e / (1.0 + e);
        };
        constexpr double rDiv = 27.25e3; // R5 + the T1 secondary with R4
        return std::max (0.0, 20.0 * std::log10 ((1.0 + 0.5 * rDiv * g (ctrl)) / (1.0 + 0.5 * rDiv * g (ctrlIdle))));
    }

    double controlVoltage() const { return ctrl; }
    double gateVoltage() const { return vGate; }
    double idleControlVoltage() const { return ctrlIdle; }

    // Newton statistics of the three circuits (tests)
    const SemiNetwork& netA() const { return a; }
    long long statSamples = 0, statPasses = 0; // samples, and the extra A / C passes of the coupled solve
    const SemiNetwork& netB() const { return b; }
    const SemiNetwork& netC() const { return c; }
    void testFailRetries (int nA, int nC) { a.testFailRetries = nA; c.testFailRetries = nC; } // tests (SemiNetwork)
    void setFirstAttempt (int first, int retry) { for (auto* n : { &a, &b, &c }) { n->firstIts = first; n->retryIts = retry; } } // tests
    void setPredictors (bool pa, bool pb, bool pc) { predictorOn[0] = pa; predictorOn[1] = pb; predictorOn[2] = pc; } // tests; before prepare
    bool predictorOn[3] { true, true, true }; // A, B, C (prepare)
    int predictorGate[3] { 0, 2, 2 };
    void setPredictorGates (int ga, int gb, int gc) { predictorGate[0] = ga; predictorGate[1] = gb; predictorGate[2] = gc; } // tests; before prepare
    void testDampedOnly (bool running, bool dc) // tests (SemiNetwork); before prepare
    {
        for (auto* n : { &a, &b, &c }) { n->testDampedOnly = running; n->testDampedDC = dc; }
    }

    // Factory trims (calibrated in LTspice with the same equations, spice/1176_*.cir; docs/1176_schematic_notes.md):
    // "Q" bias R59 wiper position (1.0 dB drop with G.R. off, 2 kHz), R30* (Q6 33 mA: the +30 V rail at 50 mA as on
    // the schematic), R86 (LN null: least distortion in compression)
    static constexpr double kQBias = 0.30677, kR30Select = 180e3, kR86 = 35.0;

private:
    static constexpr double kV30 = 30.0, kVm10 = -10.0, kNortonR = 1e9, kSwitchOn = 1e-2, kSwitchOff = 1e9;
    static constexpr double kR78 = 56e3, kLadderR = 56e3 + 68e3 + 56e3 + 56e3 + 47e3, kR23 = 250e3;
    static constexpr double kR55 = 25e3, kR56 = 5e6, kZ0 = 620.0;
    static constexpr int kControlEvery = 32; // controls (matrix rebuilds) applied at most every 32 samples
    static constexpr int kMaxCoupled = 8;    // A / C passes per sample at most
    // iteration banks (processSample): A + C typically take ~4 evaluations a sample, ~11 in the busiest 1 % of
    // blocks; B ~2

    int bankAC = 64, bankB = 32;
    static constexpr double kCtrlTol = 2e-6;  // V: CTRL agreement between A and C

    // device models (spice/1176ln_revd.inc)
    static constexpr SemiNetwork::BjtModel kQ3391 { 6.734e-15, 400.0, 0.74, 74.0 };
    static constexpr SemiNetwork::BjtModel kQ3707 { 20e-15, 350.0, 1.0, 100.0 };
    static constexpr SemiNetwork::BjtModel kQ3053 { 50e-15, 120.0, 3.0, 100.0 };
    static constexpr SemiNetwork::JfetModel kQ1 { 1.125e-3, -1.372, 2.3e-3, 1.3, 181.3e-15 };
    static constexpr SemiNetwork::DiodeModel kD914 { 2.52e-9, 1.752 };
    static constexpr SemiNetwork::DiodeModel kFD333 { 77e-12, 1.7 };

    static constexpr int G = DKCircuit::GND;

    // ---- A: input, gain reduction element, preamp ----
    void buildA (SemiNetwork& n)
    {
        auto& d = n.dk;
        const int inp = d.addNode(), bt = d.addNode(), t1p = d.addNode(), t1pi = d.addNode(), t1si = d.addNode();
        const int sec = d.addNode(), dr = d.addNode(), g = d.addNode(), s = d.addNode(), m = d.addNode();
        const int n1 = d.addNode(), b2 = d.addNode(), n3 = d.addNode(), c2 = d.addNode(), e = d.addNode();
        const int f1 = d.addNode(), f = d.addNode(), p = d.addNode(), c3 = d.addNode(), e3 = d.addNode();
        const int o = d.addNode(), h = d.addNode(), ctl = d.addNode(), o2 = d.addNode(), v30 = d.addNode();
        nO2 = o2; nG = g; nO = o;

        d.addSource (inp, 600.0);           // u0: generator EMF, 600 ohm
        d.addSource (v30, 1e-3);            // u1: +30 V
        d.addSource (ctl, 1e-3);            // u2: CTRL (from C)
        d.addSource (o2, kNortonR);         // u3: ladder current correction (Norton)

        // bridged-T: R2 / R3 620, R1A bridge, R1B shunt (R1A R1B = 620^2)
        d.addR (inp, bt, 620.0);
        d.addR (bt, t1p, 620.0);
        rR1A = d.addR (inp, t1p, val.r1a);
        rR1B = d.addR (bt, G, val.r1b);
        // T1: 1.58:1 step-down, 9 H, 45 + 18.4 ohm
        d.addR (t1p, t1pi, 45.0);
        d.addL (t1pi, G, 9.0);
        d.addTransformer ({ { t1pi, G, 1.0 }, { t1si, G, std::sqrt (1.0 / 2.5) } });
        d.addR (t1si, sec, 18.4);
        d.addR (sec, G, 270.0);             // R4

        d.addR (sec, dr, 27e3);             // R5
        n.addJfet (g, dr, s, kQ1);          // Q1
        d.addC (g, dr, 6e-12);
        d.addC (g, s, 7e-12);
        d.addR (s, G, 10e3);                // R10
        d.addC (m, s, 50e-6);               // C28
        d.addC (dr, n1, 1e-6);              // C1
        d.addR (n1, b2, 1e3);               // R8
        d.addC (b2, G, 27e-12);             // C2
        d.addR (n3, b2, 560e3);             // R9
        n.addBjt (b2, c2, e, kQ3391);       // Q2
        d.addC (b2, c2, 3.6e-12);
        d.addR (p, c2, 1e6);                // R13
        d.addR (e, m, 180.0);               // R18
        d.addR (m, G, 180.0);               // R84
        d.addR (e, f1, 150.0);              // R85
        d.addR (f1, f, kR86);               // R86
        n.addBjt (c2, c3, e3, kQ3391);      // Q3
        d.addC (c2, c3, 3.6e-12);
        d.addR (p, c3, 22e3);               // R14
        d.addR (e3, n3, 82.0);              // R11
        d.addR (n3, G, 910.0);              // R12
        d.addC (n3, G, 80e-6);              // C5
        n.addBjt (c3, v30, o, kQ3391);      // Q14
        d.addC (c3, v30, 3.6e-12);
        d.addR (o, f, 6.8e3);               // R15
        d.addR (v30, p, 6.8e3);             // R17
        d.addC (p, G, 80e-6);               // C4
        // LN gate network
        d.addC (f, h, 0.22e-6);             // C9
        d.addR (h, g, 2.2e6);               // R6
        d.addC (h, g, 200e-12);             // C3
        d.addR (g, ctl, 2.2e6);             // R7
        d.addC (g, ctl, 200e-12);           // C6
        // output: C7, OUTPUT pot (whole, as a load) and the ratio ladder (as a load)
        d.addC (o, o2, 1e-6);               // C7
        d.addR (o2, G, kR23);               // R23
        d.addR (o2, G, kLadderR);           // R78 + R19 + R20 + R21 + R22
        d.markOutput (o2);
        d.markOutput (g);
    }

    // ---- B: line amp ----
    void buildB (SemiNetwork& n)
    {
        auto& d = n.dk;
        const int w = d.addNode(), b4 = d.addNode(), c4 = d.addNode(), e4 = d.addNode(), l = d.addNode();
        const int b5 = d.addNode(), c6 = d.addNode(), x1 = d.addNode(), b6 = d.addNode(), e6 = d.addNode();
        const int wbw = d.addNode(), n32 = d.addNode(), x2 = d.addNode(), slt = d.addNode(), x3 = d.addNode();
        const int brn = d.addNode(), sltw = d.addNode(), blu = d.addNode(), yel = d.addNode(), redw = d.addNode();
        const int v30 = d.addNode();
        nBlu = blu; nSlt = slt;

        srcWiper = d.addSource (w, val.rWiper); // u0: OUTPUT pot wiper (Thevenin)
        d.addSource (v30, 1e-3);            // u1: +30 V

        d.addC (w, b4, 0.15e-6);            // C8
        d.addR (c4, b4, 2.7e6);             // R24
        d.addR (b4, G, 2.7e6);              // R25
        d.addR (l, c4, 68e3);               // R26
        d.addR (e4, G, 1.5e3);              // R27
        n.addBjt (b4, c4, e4, kQ3391);      // Q4
        d.addC (b4, c4, 3.6e-12);
        d.addC (c4, b5, 1e-6);              // C10
        d.addR (b5, c6, 1.2e6);             // R29
        d.addC (b5, c6, 10e-12);            // C11
        d.addR (b5, x1, 150e3);             // R30
        d.addR (b5, x1, kR30Select);        // R30* (T+C)
        n.addDiode (x1, e4, kD914);         // CR1
        n.addBjt (b5, l, b6, kQ3707);       // Q5
        d.addC (b5, l, 4e-12);
        n.addBjt (b6, c6, e6, kQ3053);      // Q6
        d.addC (b6, c6, 15e-12);
        d.addR (v30, l, 8.2e3);             // R34
        d.addC (l, G, 5e-6);                // C12
        d.addC (e6, G, 0.033e-6);           // C14
        d.addR (n32, G, 39.0);              // R32
        d.addR (e4, x2, 70e3);              // R31
        d.addC (x2, n32, 270e-12);          // C13
        d.addR (e4, slt, 22e3);             // R28
        d.addR (c6, x3, 560.0);             // R33
        d.addC (x3, slt, 0.0047e-6);        // C15
        // T2 (UA-5002): primary 1, feedback winding 1 (VIO grounded), emitter winding sqrt (1.3/600),
        // secondary 0.5 + 0.5; dots at BLK, VIO, WHT/BLK, BLU. 6 H primary.
        d.addR (c6, brn, 40.0);
        d.addL (v30, brn, 6.0);
        d.addR (sltw, slt, 40.0);
        d.addR (wbw, n32, 0.2);
        d.addR (redw, slt, 30.0);
        d.addTransformer ({ { v30, brn, 1.0 }, { G, sltw, 1.0 }, { e6, wbw, std::sqrt (1.3 / 600.0) },
                            { blu, yel, 0.5 }, { yel, redw, 0.5 } });
        d.addR (blu, slt, 600.0);           // load
        d.markOutput (blu);
        d.markOutput (slt);
    }

    // ---- C: ratio ladder, G.R. control amp, rectifier, threshold, attack / release ----
    void buildC (SemiNetwork& n)
    {
        auto& d = n.dk;
        const int o2 = d.addNode(), ta = d.addNode(), t12 = d.addNode(), t8 = d.addNode(), t4 = d.addNode();
        const int sc = d.addNode(), scx = d.addNode(), b7 = d.addNode(), c7 = d.addNode(), e7 = d.addNode();
        const int rb = d.addNode(), n41 = d.addNode(), e8 = d.addNode(), b9 = d.addNode(), c9 = d.addNode();
        const int e9 = d.addNode(), n50 = d.addNode(), e10 = d.addNode(), a2 = d.addNode(), a3 = d.addNode();
        const int k = d.addNode(), m2 = d.addNode(), qa = d.addNode(), b2 = d.addNode(), s0 = d.addNode();
        const int s1 = d.addNode(), s2 = d.addNode(), s3 = d.addNode(), w55 = d.addNode(), ctl = d.addNode();
        const int w56 = d.addNode(), gt = d.addNode(), v30 = d.addNode(), vm10 = d.addNode();
        nCtrl = ctl; nTa = ta;

        d.addSource (o2, 1e-3);             // u0: preamp output O2 (from A)
        d.addSource (v30, 1e-3);            // u1: +30 V
        d.addSource (vm10, 1e-3);           // u2: -10 V
        d.addSource (gt, 1e-3);             // u3: FET gate (from A)

        // deck A: tapped ladder to ground, the pressed buttons' taps to the sidechain
        d.addR (o2, ta, kR78);              // R78
        d.addR (ta, t12, 68e3);             // R19
        d.addR (t12, t8, 56e3);             // R20
        d.addR (t8, t4, 56e3);              // R21
        d.addR (t4, G, 47e3);               // R22
        d.addR (ta, sc, 10e6);              // T+C across S4A
        rS1A = d.addR (t4, sc, val.sw[0]);
        rS2A = d.addR (t8, sc, val.sw[1]);
        rS3A = d.addR (t12, sc, val.sw[2]);
        rS4A = d.addR (ta, sc, val.sw[3]);
        rS9a = d.addR (sc, scx, val.s9a);
        rS9b = d.addR (scx, G, val.s9b);

        d.addC (scx, b7, 1e-6);             // C17
        n.addBjt (b7, c7, e7, kQ3707);      // Q7
        d.addC (b7, c7, 4e-12);
        d.addR (c7, b7, 1e6);               // R36
        d.addR (b7, G, 470e3);              // R37
        d.addR (rb, c7, 47e3);              // R38
        d.addR (v30, rb, 4.7e3);            // R51
        d.addC (rb, G, 50e-6);              // C18
        d.addR (e7, n41, 4.7e3);            // R39
        n.addBjt (c7, v30, e8, kQ3707);     // Q8
        d.addC (c7, v30, 4e-12);
        d.addR (e8, n41, 2.4e3);            // R40
        d.addR (n41, G, 270.0);             // R41
        d.addR (e8, b9, 180e3);             // R42
        d.addR (b9, G, 47e3);               // R43
        d.addR (b9, G, 390e3);              // R45
        n.addBjt (b9, c9, e9, kQ3707);      // Q9
        d.addC (b9, c9, 4e-12);
        d.addR (v30, c9, 47e3);             // R47
        d.addR (e9, n50, 8.2e3);            // R48
        n.addBjt (c9, v30, e10, kQ3707);    // Q10
        d.addC (c9, v30, 4e-12);
        d.addR (e10, n50, 2.4e3);           // R49
        d.addR (n50, G, 180.0);             // R50
        d.addR (e7, e10, 47e3);             // R46

        d.addC (e10, a2, 6.8e-6);           // C19
        n.addDiode (a2, k, kFD333);         // CR2
        d.addC (e8, a3, 6.8e-6);            // C20
        n.addDiode (a3, k, kFD333);         // CR3
        d.addR (a2, m2, 47e3);              // R52
        d.addR (a3, m2, 47e3);              // R53
        d.addC (m2, G, 80e-6);              // C21

        // "Q" bias: -10 V -> R35 10k -> R59 2k trim (wiper = B2) -> ground, R60 3.9k
        d.addR (vm10, qa, 10e3);            // R35
        d.addR (qa, G, 3.9e3);              // R60
        d.addR (qa, b2, 2e3 * (1.0 - kQBias));
        d.addR (b2, G, 2e3 * kQBias);
        // deck B: threshold string B2 -> -10 V, the pressed buttons' taps to M2
        d.addR (b2, s0, 150.0);             // R58
        d.addR (s0, s1, 470.0);             // R61
        d.addR (s1, s2, 560.0);             // R62
        d.addR (s2, s3, 1.5e3);             // R63
        d.addR (s3, vm10, 1.5e3);           // R64
        rS1B = d.addR (s0, m2, val.sw[0]);
        rS2B = d.addR (s1, m2, val.sw[1]);
        rS3B = d.addR (s2, m2, val.sw[2]);
        rS4B = d.addR (s3, m2, val.sw[3]);

        // ATTACK: R55 (CCW end at K), R54 || C27 from K to the wiper; RELEASE: R56 from CTRL, R57 wiper -> B2
        rR55a = d.addR (k, w55, val.r55a);
        rR55b = d.addR (w55, ctl, val.r55b);
        d.addR (k, w55, 470.0);             // R54
        d.addC (k, w55, 0.022e-6);          // C27
        rR56a = d.addR (ctl, w56, val.r56a);
        rR56b = d.addR (w56, b2, val.r56b);
        d.addR (w56, b2, 270e3);            // R57
        d.addC (ctl, G, 0.22e-6);           // C22
        // the gate network's R7 || C6, seen from CTRL
        d.addR (ctl, gt, 2.2e6);
        d.addC (ctl, gt, 200e-12);
        d.markOutput (ctl);
        d.markOutput (ta);
    }

    // element values for the controls (used by the builders, and by applyControls on the running circuits)
    struct Values
    {
        double r1a = 1e-3, r1b = 1e9, rWiper = 1e-3, r55a = 1e-3, r55b = 1e-3, r56a = 1e-3, r56b = 1e-3;
        double s9a = kSwitchOn, s9b = kSwitchOff;
        double sw[4] { kSwitchOff, kSwitchOff, kSwitchOff, kSwitchOff };
    };

    void computeValues (const Controls& t)
    {
        // INPUT: bridged-T, loss 1 + R1A / Z0 (R1A R1B = Z0^2)
        const double kat = std::pow (10.0, std::max (0.0, -t.inputDb) / 20.0);
        val.r1a = std::max (kZ0 * (kat - 1.0), 1e-3);
        val.r1b = std::min (kZ0 / std::max (kat - 1.0, 1e-6), 1e9);
        // OUTPUT: audio-taper pot, the wiper as a Thevenin source into B
        const double frac = std::clamp (taperA (std::clamp (t.output, 0.0, 1.0)), 0.0, 1.0);
        const double rBot = std::max (kR23 * frac, 1e-3), rTop = std::max (kR23 * (1.0 - frac), 1e-3);
        wiperFrac = rBot / (rTop + rBot);
        val.rWiper = rTop * rBot / (rTop + rBot) + 1e-3;
        const double xa = std::clamp (t.attack, 0.0, 1.0), xr = std::clamp (t.release, 0.0, 1.0);
        val.r55a = std::max (kR55 * xa, 1e-3);
        val.r55b = std::max (kR55 * (1.0 - xa), 1e-3);
        val.r56a = std::max (kR56 * (1.0 - xr), 1e-3);
        val.r56b = std::max (kR56 * xr, 1e-3);
        const int r = std::clamp (t.ratio, 0, (int) rAll);
        const bool on[4] = { r == r4 || r == rAll, r == r8 || r == rAll, r == r12 || r == rAll, r == r20 || r == rAll };
        for (int i = 0; i < 4; ++i) val.sw[i] = on[i] ? kSwitchOn : kSwitchOff;
        val.s9a = t.grOn ? kSwitchOn : kSwitchOff;
        val.s9b = t.grOn ? kSwitchOff : kSwitchOn;
    }

    void applyControls()
    {
        pending = false;
        computeValues (target);
        a.dk.setR (rR1A, val.r1a); // setR only marks a rebuild when the value changed
        a.dk.setR (rR1B, val.r1b);
        b.dk.setSourceResistance (srcWiper, val.rWiper);
        c.dk.setR (rR55a, val.r55a);
        c.dk.setR (rR55b, val.r55b);
        c.dk.setR (rR56a, val.r56a);
        c.dk.setR (rR56b, val.r56b);
        const int sa[4] = { rS1A, rS2A, rS3A, rS4A }, sb[4] = { rS1B, rS2B, rS3B, rS4B };
        for (int i = 0; i < 4; ++i) { c.dk.setR (sa[i], val.sw[i]); c.dk.setR (sb[i], val.sw[i]); }
        c.dk.setR (rS9a, val.s9a);
        c.dk.setR (rS9b, val.s9b);
    }

    // DC operating point of all three circuits (C and A depend on each other through CTRL / the gate)
    void initOperatingPoint()
    {
        double gate = -1.45;
        for (int pass = 0; pass < 3; ++pass)
        {
            const double uc[4] = { 0.0, kV30, kVm10, gate };
            c.initDC (uc, 4);
            ctrl = c.dcNode (nCtrl);
            const double ua[4] = { 0.0, kV30, ctrl, 0.0 };
            a.initDC (ua, 4);
            gate = a.dcNode (nG);
        }
        const double ub[2] = { 0.0, kV30 };
        b.initDC (ub, 2);
        const double ua[4] = { 0.0, kV30, ctrl, 0.0 }, uc[4] = { 0.0, kV30, kVm10, gate };
        a.saveIdle (4, ua);
        b.saveIdle (2, ub);
        c.saveIdle (4, uc);
        vGate = gate;
        gateIdle = gate;
        ctrlIdle = ctrl;
        ctrlPrev = ctrl;
        iLadderErr = 0.0;
        lastOut = 0.0;
    }

    SemiNetwork a, b, c;
    Controls target;
    Values val;
    bool pending = true;
    int controlPhase = 0;
    double fs = 96000.0;
    double ctrl = -1.45, ctrlPrev = -1.45, ctrlIdle = -1.45, vGate = -1.45, gateIdle = -1.45, iLadderErr = 0.0, lastOut = 0.0, wiperFrac = 0.5;
    int nO2 = 0, nG = 0, nO = 0, nBlu = 0, nSlt = 0, nCtrl = 0, nTa = 0, srcWiper = 0;
    int rR1A = 0, rR1B = 0, rR55a = 0, rR55b = 0, rR56a = 0, rR56b = 0;
    int rS1A = 0, rS2A = 0, rS3A = 0, rS4A = 0, rS1B = 0, rS2B = 0, rS3B = 0, rS4B = 0, rS9a = 0, rS9b = 0;
};

} // namespace vamp
