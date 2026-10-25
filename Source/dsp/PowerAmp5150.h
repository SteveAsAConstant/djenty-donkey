#pragma once
#include <algorithm>
#include <cmath>
#include "DKCircuit.h"
#include "PotTapers.h"
#include "Supply5150.h"
#include "TriodeStage.h"
#include "TubeSet.h"

namespace vamp
{
// Norman Koren's pentode model (6L6GC parameters from the wdv.com SPICE tube table) with a finite plate resistance.
// Koren's plate current hardly depends on the plate voltage above the knee (atan): the 6L6GC's plate resistance came
// out at ~100-280k at the datasheet operating points (datasheets: 22.5k at 250 V / 72 mA, 33k at 350 V / 54 mA) and
// ~890k at the 5150's, so the output stage had almost no source resistance of its own and followed the speaker's
// impedance too much. An Early factor (1 + Vpk / va) gives rp ~ (va + Vpk) / Ip (together with Koren's own slope): va
// 1650 V centres both datasheet points, kg1 1460 -> 1865 keeps their plate currents (dsp_test tube). Koren's screen
// current was ~1.9x the datasheets' at the two static points and did not depend on the plate voltage at all, so under
// heavy drive it was far too low (push-pull AB1 at full signal: 12.8 mA per pair, datasheet 22 mA). Screen current
// with current conservation: the cathode current Koren's plate knee withholds at low plate voltage,
// base (pi/2 - atan(Vpk / kvb)), goes to the screen, on top of Koren's screen term with kg2 12000 (least squares
// over the static 250 / 350 V points and the AB1 zero / full signal pair currents: 5.15 / 3.03 / 4.76 / 20.5 mA vs
// 5.0 / 2.5 / 5.6 / 22 mA; dsp_test tube).
struct KorenPentode
{
    static constexpr double mu = 8.7, ex = 1.35, kg1 = 1865.0, kg2 = 12000.0, kp = 48.0, kvb = 12.0, va = 1650.0;

    static const SoftplusPow& softplusPow() { static const SoftplusPow t (ex); return t; }

    bool exact = false; // reference renders: original formula instead of the table
    double currentScale = 1.0; // this tube's deviation from the nominal one (TubeSet): cathode current x

    // Terms that depend only on the screen voltage (constant during a Newton solve)
    struct Screen { double vg2k, inv, scale; };
    static Screen screenTerms (double vg2k)
    {
        // scale = 2/kg1 (vg2k/kp)^ex; vg2k <= 0 -> no plate current (as Koren's E1 <= 0)
        return { vg2k, 1.0 / vg2k, vg2k > 0.0 ? 2.0 / kg1 * std::pow (vg2k / kp, ex) : 0.0 };
    }

    // plate current of ONE tube and its derivatives w.r.t. Vpk and Vg1k
    inline void plate (double vpk, double vg1k, double vg2k, double& ip, double& dVpk, double& dVg1) const
    {
        plate (vpk, vg1k, screenTerms (vg2k), ip, dVpk, dVg1);
    }

    //   ip = 2/kg1 (vg2k/kp)^ex softplus(x)^ex atan(vpk/kvb),   x = kp (1/mu + vg1k/vg2k)
    // atan() is kept smooth through Vpk = 0 (as in Koren's original); below 0 the flyback
    // diodes dominate, and the smooth knee keeps Newton's method well behaved.
    inline void plate (double vpk, double vg1k, const Screen& s, double& ip, double& dVpk, double& dVg1) const
    {
        if (exact) { plateExact (vpk, vg1k, s.vg2k, ip, dVpk, dVg1); return; }
        constexpr double kpOverMu = kp / mu, invKvb = 1.0 / kvb;
        const double x = kpOverMu + kp * vg1k * s.inv;
        double G, dG;
        softplusPow().eval (x, G, dG);

        const double base = s.scale * G;
        const double r = vpk * invKvb;
        const double at = std::atan (r);
        ip = base * at;
        dVg1 = s.scale * dG * kp * s.inv * at;
        dVpk = base * invKvb / (1.0 + r * r);
        early (vpk, ip, dVpk, dVg1);
        ip *= currentScale; dVpk *= currentScale; dVg1 *= currentScale;
    }

    // the finite plate resistance: ip (1 + Vpk / va) above 0 V (continuous there, ip = 0)
    static inline void early (double vpk, double& ip, double& dVpk, double& dVg1)
    {
        if (vpk <= 0.0) return;
        const double f = 1.0 + vpk * (1.0 / va);
        dVpk = dVpk * f + ip * (1.0 / va);
        ip *= f;
        dVg1 *= f;
    }

    // The same with the original formula: reference for the tests
    inline void plateExact (double vpk, double vg1k, double vg2k, double& ip, double& dVpk, double& dVg1) const
    {
        constexpr double kpOverMu = kp / mu, invKvb = 1.0 / kvb, twoOverKg1 = 2.0 / kg1;
        const double x = kpOverMu + kp * vg1k / vg2k;
        double sp, sig;
        if (x > 30.0)       { sp = x; sig = 1.0; }
        else if (x < -30.0) { sp = std::exp (x); sig = sp; }
        else
        {
            const double e = std::exp (x);
            sp = std::log1p (e);
            sig = e / (1.0 + e);
        }
        const double E1 = vg2k * (1.0 / kp) * sp;
        if (E1 <= 0.0) { ip = 0.0; dVpk = 0.0; dVg1 = 0.0; return; }

        const double Ep = std::exp ((ex - 1.0) * std::log (E1)); // == pow (E1, ex - 1), faster
        const double base = twoOverKg1 * Ep * E1;
        const double r = vpk * invKvb;
        const double at = std::atan (r);
        ip = base * at;
        dVg1 = twoOverKg1 * ex * Ep * sig * at;
        dVpk = base * invKvb / (1.0 + r * r);
        early (vpk, ip, dVpk, dVg1);
        ip *= currentScale; dVpk *= currentScale; dVg1 *= currentScale;
    }

    // screen current of ONE tube: Koren's term plus the cathode current the plate knee withholds (see above)
    inline double screen (double vpk, double vg1k, double vg2k) const
    {
        if (vg2k <= 0.0) return 0.0;
        const double b = vg2k / mu + vg1k;
        const double x = kp * (1.0 / mu + vg1k / vg2k);
        const double sp = x > 30.0 ? x : (x < -30.0 ? std::exp (x) : std::log1p (std::exp (x)));
        const double base = 2.0 / kg1 * std::pow (vg2k / kp * sp, ex);
        return currentScale * ((b > 0.0 ? std::pow (b, ex) / kg2 : 0.0) + base * (1.5707963267948966 - std::atan (vpk / kvb)));
    }
};

inline void gridCurrentR (double vgk, double rgk, double& ig, double& dIg)
{
    constexpr double von = 0.25, vd = 0.03;
    const double x = (vgk - von) * (1.0 / vd);
    if (x > 30.0)  { ig = (vgk - von) / rgk; dIg = 1.0 / rgk; return; }
    if (x < -30.0) { ig = 0.0; dIg = 0.0; return; }
    const double e = std::exp (x);
    ig = vd / rgk * std::log1p (e);
    dIg = e / (1.0 + e) / rgk;
}

// Component-level model of the 5150 power section (schematic pages 3-4):
//
//   master -> V3A recovery stage -> C49 -> LTP phase inverter V4A/V4B (R46 82k / R58 100k,
//   tail R52 470 + R51 10k + R57 4.7k) -> C28/C29 47n, R54/R55 220k bias, 2.2k grid stoppers
//   -> 2 x 2 x 6L6GC push-pull -> output transformer (Raa ~2.1k) -> 4x12 speaker impedance
//   NFB: speaker -> R56 39k -> (C12 6.8n || VR8 Resonance) -> Y -> C30 22u -> tail node L (R57 4.7k),
//        presence: Y -> VR9 10k (A) -> C8 0.1u || C62 0.033u -> gnd.   V4A grid AC-referenced to L via C31.
//   Supply: the factory chain (Supply5150), every node sagging with the stages' currents.
//
// V3A + PI + output stage + transformer + speaker + NFB loop are solved simultaneously with the DK method
// (10 nonlinear ports), so the feedback loop has no artificial delay and V3A sees the PI's real load.
class PowerAmp5150
{
public:
    static constexpr int kPorts = 10, kDevices = kPorts / 2; // V4B, V4A, 6L6 pair 1, 6L6 pair 2, V3A

    // Supply: the factory chain PLATE -> SCREEN -> V3+ (phase inverter) -> V2+ (V2B V5B V5A V3A V3B) -> V1+
    // (V1A V1B V2A), Supply5150. Preamp draw when no preamp is connected (setPreampLoad): the model's idle currents.
    static constexpr double kPreampIdleI1 = 1.65e-3, kPreampIdleI2 = 2.19e-3;

    // ---- assumptions (not printed on the schematic) ----
    static constexpr double kTwoPi = 6.283185307179586;
    static constexpr double kIdlePerTube = 0.035;
    static constexpr double kRaa = 2100.0;      // plate-to-plate impedance
    static constexpr double kRload = 8.0;
    static constexpr double kLmagPrimary = 30.0; // OT magnetizing inductance (H)
    static constexpr double kCwinding = 2.5e-9;  // OT primary shunt capacitance (F)
    static constexpr double kLeakagePP = 1e-4;   // OT leakage inductance, plate to plate (H)
    // 4x12 of Celestion Vintage 30s, 8 ohm, from Thiele-Small physics: Re 7.3 ohm and Fs 75 Hz (Celestion), Qes 0.78
    // (a V30-type speaker, WGS Veteran 30); the closed cab stiffens the suspension (Vas ~19 L vs ~27-35 L of air per
    // driver): fc 80-105 Hz (owners measure 90-105 Hz), Qec = Qes fc / Fs. The total mechanical Q is far below a
    // single driver's (~12): box leaks and four drivers with spread resonances (R. Aiken). fc 85 Hz and Qmc 2 (a
    // 24 ohm peak) and a 0.7 mH voice coil (a G12H-75 measures 0.7 mH): fitted within their physical ranges to
    // power-amp-only captures of a real 6505 into three real cabinets (docs/power_amp.md). The coil's eddy-current
    // loss, 6 ohm, is an empirical fit to the same captures (Celestion publishes no impedance curve; ~13 ohm at
    // 12.8 kHz), checked on full-amp captures the fit did not use (~1 dB better at 8-12.8 kHz on all five)
    static constexpr double kSpeakerRe = 7.3, kSpeakerFc = 85.0, kSpeakerQec = 0.78 * kSpeakerFc / 75.0, kSpeakerQmc = 2.0;
    static constexpr double kVoiceCoilL = 0.7e-3, kVoiceCoilLossR = 6.0;
    // ---- from the schematic and datasheets ----
    static constexpr double kCgridFeed = 100e-12;  // C201 / C202: each side's grid feed to ground (factory power board)
    static constexpr double k6L6Cin = 10e-12, k6L6Cgp = 0.6e-12; // 6L6GC input and grid-to-plate capacitance
    static constexpr double kRscreenTube = 100.0;  // R203 / R205 / R207 / R209: each 6L6's own screen resistor

    PowerAmp5150()
    {
        auto n = [this] { return dk.addNode(); };
        using D = DKCircuit;
        const int G = D::GND;

        nA = n(); nP1 = n(); nGB = n(); nGA = n(); nK = n(); nQ = n(); nL = n(); nM = n(); nPr = n(); nY = n();
        nPB = n(); nPA = n(); nC1 = n(); nG1 = n(); nC2 = n(); nG2 = n();
        nO1 = n(); nO2 = n(); nCT = n(); nS = n(); nSp1 = n(); nSp2 = n(); nSec = n();
        nIn = n(); nG3 = n(); nK3 = n(); nP3 = n();

        // V3A recovery stage: master wiper (~100k source) -> C11 47n -> R26 1M, 1k grid stopper;
        // R5 68k from V2+, R14 3.3k || C61 4.7n
        srcIn = dk.addSource (nA, 100e3);  // master pot source impedance
        dk.addC (nA, nIn, 47e-9);          // C11
        dk.addR (nIn, G, 1e6);             // R26
        dk.addR (nIn, nG3, 1e3);           // grid stopper
        srcR5 = dk.addSource (nP3, 68e3);  // R5
        dk.addR (nK3, G, 3.3e3);           // R14
        dk.addC (nK3, G, 4.7e-9);          // C61
        dk.addC (nP3, nP1, 22e-9);         // C49
        dk.addR (nP1, nQ, 1e6);            // R50
        dk.addR (nP1, nGB, 100e3);         // R48
        dk.addR (nGA, nQ, 1e6);            // R53
        dk.addC (nGA, nL, 47e-9);          // C31
        dk.addR (nK, nQ, 470.0);           // R52
        dk.addR (nQ, nL, 10e3);            // R51
        dk.addR (nL, G, 4.7e3);            // R57

        srcR46 = dk.addSource (nPB, 82e3);  // R46
        srcR58 = dk.addSource (nPA, 100e3); // R58

        // 12AX7 interelectrode capacitances (the same as the preamp's TriodeNetwork and the LTspice model)
        const int tri[3][3] = { { nPB, nGB, nK }, { nPA, nGA, nK }, { nP3, nG3, nK3 } }; // plate, grid, cathode
        for (auto& t : tri)
        {
            dk.addC (t[1], t[2], KorenTriode::cgk); // Cgk
            dk.addC (t[1], t[0], KorenTriode::cgp); // Cgp
        }

        dk.addR (nS, nM, 39e3);            // R56
        dk.addC (nM, nY, 6.8e-9);          // C12
        rRes = dk.addR (nM, nY, 1e6);      // VR8 resonance
        dk.addC (nY, nL, 22e-6);           // C30: the feedback network is AC-coupled to the tail (no DC through VR8)
        rPres = dk.addR (nY, nPr, 10e3);   // VR9 presence (10k audio taper, wiper tied to the capacitor end)
        dk.addC (nPr, G, 0.133e-6);        // C8 0.1u || C62 0.033u

        dk.addC (nPB, nC1, 47e-9);         // C29
        srcBias1 = dk.addSource (nC1, 220e3); // R55
        dk.addC (nC1, G, kCgridFeed);      // C202
        dk.addR (nC1, nG1, 2.2e3);         // R206 / R208
        dk.addC (nPA, nC2, 47e-9);         // C28
        srcBias2 = dk.addSource (nC2, 220e3); // R54
        dk.addC (nC2, G, kCgridFeed);      // C201
        dk.addR (nC2, nG2, 2.2e3);         // R202 / R204
        // 6L6GC interelectrode capacitances (datasheet), two tubes per side: g1 to cathode + g2 + g3 (input) and g1 to
        // plate (Miller, the plate swings ~6x the grid)
        for (const auto& gp : { std::pair { nG1, nO1 }, std::pair { nG2, nO2 } })
        {
            dk.addC (gp.first, G, 2.0 * k6L6Cin);
            dk.addC (gp.first, gp.second, 2.0 * k6L6Cgp);
        }

        const double np = std::sqrt (kRaa / kRload);
        srcB = dk.addSource (nCT, 1.0);
        // primary shunt capacitance (in parallel with the reflected load, so it is damped):
        // gives the open-loop high-frequency roll-off of the output transformer
        cWinding = dk.addC (nO1, nO2, kCwinding);
        // the secondary winding drives the speaker terminal (where the feedback is taken) through the leakage
        // inductance, referred to the secondary (plate-to-plate value / np^2)
        dk.addTransformer ({ { nO1, nCT, np / 2 }, { nCT, nO2, np / 2 }, { nSec, G, 1.0 } });
        lLeak = dk.addL (nSec, nS, kLeakagePP / (np * np));
        dk.addL (nSec, G, kLmagPrimary / (np * np));

        // 4x12 cab: Re, the lossy voice coil, the motional impedance (Res || Lces || Cmes) of the in-box resonance
        rSpkRe = dk.addR (nS, nSp1, kSpeakerRe);
        lVoice = dk.addL (nSp1, nSp2, kVoiceCoilL);
        rVoiceLoss = dk.addR (nSp1, nSp2, kVoiceCoilLossR);
        {
            const double w = kTwoPi * kSpeakerFc, cmes = kSpeakerQec / (w * kSpeakerRe);
            rSpkPeak = dk.addR (nSp2, G, kSpeakerRe * kSpeakerQmc / kSpeakerQec);
            lSpkM = dk.addL (nSp2, G, 1.0 / (w * w * cmes));
            cSpkM = dk.addC (nSp2, G, cmes);
        }

        dk.markOutput (nS);  // speaker
        dk.markOutput (nK);  // diagnostics
        dk.markOutput (nO1);
        dk.markOutput (nPA);
        dk.markOutput (nPB);
        dk.markOutput (nP3);

        // nonlinear ports: (plate, grid) per device, plate current depends on both, grid current on the grid only
        dk.addPort (nPB, nK); dk.addPort (nGB, nK);   // V4B
        dk.addPort (nPA, nK); dk.addPort (nGA, nK);   // V4A
        dk.addPort (nO1, G);  dk.addPort (nG1, G);    // 6L6 pair 1
        dk.addPort (nO2, G);  dk.addPort (nG2, G);    // 6L6 pair 2
        dk.addPort (nP3, nK3); dk.addPort (nG3, nK3); // V3A
    }

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        T = 1.0 / fs;
        dk.prepare (fs);

        // supply / bias at idle (rails: the DC solution, refined by the settle loop below)
        supply = Supply5150();
        cfIk = 1.25e-3;
        std::fill (std::begin (ig2Tube), std::end (ig2Tube), 0.0);

        vBias = findBias (supply.plate, supply.screen);
        std::fill (std::begin (p), std::end (p), 0.0);
        std::fill (std::begin (pPrev), std::end (pPrev), 0.0);
        lastIn = 0.0;

        // settle at the DC operating point: rails held while the circuit settles, then set to the supply chain's DC
        // solution for the average load currents, a few times (the currents depend only weakly on the rails).
        // Speeding up the supply instead would make the supply -> phase inverter path fast enough to oscillate.
        beginSupplySettle();
        for (int pass = 0; pass < 4; ++pass)
        {
            for (int i = 0; i < (int) (fs * 0.1); ++i) process (0.0);
            resetSupplyAverage(); // average after the transient of the last rail update has died out
            for (int i = 0; i < (int) (fs * 0.1); ++i) process (0.0);
            applySupplyDC();
        }
        for (int i = 0; i < (int) (fs * 0.15); ++i) process (0.0); // settle at the final rails
        endSupplySettle();

        // settled idle point: where a persistent fault falls back to
        maxHeldRun = std::max (1, (int) (1e-3 * fs));
        dk.saveState();
        std::copy (std::begin (p), std::end (p), std::begin (pIdle));
        plateIdle = supply.plate; screenIdle = supply.screen;
        std::copy (std::begin (ig2Tube), std::end (ig2Tube), std::begin (ig2Idle));
        railIdle[0] = supply.v3; railIdle[1] = supply.v2; railIdle[2] = supply.v1;
    }

    // Supply capacitors 50x smaller (time constants <= 10 ms): for settling preamp + power amp together at prepare
    // DC settling (prepare, AmpSim with the preamp connected): while settling, the rails hold and the load currents
    // are averaged; applySupplyDC() sets the rails to the chain's DC solution for them
    void beginSupplySettle() { supplyFrozen = true; resetSupplyAverage(); }
    void resetSupplyAverage() { std::fill (std::begin (avgI), std::end (avgI), 0.0); avgN = 0; }
    void applySupplyDC()
    {
        if (avgN > 0) supply.setDC (avgI[0] / avgN, avgI[1] / avgN, avgI[2] / avgN, avgI[3] / avgN, avgI[4] / avgN);
        resetSupplyAverage();
    }
    void endSupplySettle() { supplyFrozen = false; }
    double getSupplyV (int node) const { const double v[] = { supply.plate, supply.screen, supply.v3, supply.v2, supply.v1 }; return v[node]; }

    // Preamp draw from V1+ / V2+ for the next sample (Preamp5150::railCurrents); without it the idle currents
    void setPreampLoad (double i1, double i2) { preI1 = i1; preI2 = i2; }
    // supply rails for the preamp (Preamp5150::setRails)
    double getRailV1() const { return supply.v1; }
    double getRailV2() const { return supply.v2; }
    double getRailV3() const { return supply.v3; }
    double getScreenVoltage() const { return supply.screen; }

    // Not on the schematic, for fitting / comparisons (call before prepare): the load (4x12 or load box) peak
    // damping resistance, voice-coil inductance and its loss resistance, the OT primary shunt capacitance, and Re
    void setLoadAndTransformer (double peakR, double voiceL, double voiceLossR, double cWindingF, double re = kSpeakerRe)
    {
        dk.setR (rSpkRe, re);
        dk.setR (rSpkPeak, peakR);
        dk.setL (lVoice, voiceL);
        dk.setR (rVoiceLoss, voiceLossR);
        dk.setC (cWinding, cWindingF);
    }

    // Tube set (TubeSet): V4B / V4A / V3A and the two output pairs as particular tubes. Takes effect at once (the
    // fixed bias stays where the factory set it for nominal tubes, so a hotter pair just draws more).
    void setTubeSet (const TubeSet& s)
    {
        for (int t = 0; t < 3; ++t) devTriode[t].setVariation (s.paI[t], s.paMu[t]);
        for (int t = 0; t < 2; ++t) side6L6[t].currentScale = s.sideI[t];
    }

    // OT leakage inductance, plate to plate (call before prepare)
    void setLeakage (double lpp) { dk.setL (lLeak, std::max (lpp, 1e-6) * kRload / kRaa); }

    // The speaker's motional impedance from Thiele-Small terms (call before prepare): in-box resonance fc, electrical
    // and total mechanical Q (box losses included): Cmes = Qec / (2 pi fc Re), Lces = 1 / ((2 pi fc)^2 Cmes),
    // Res = Re Qmc / Qec (the parallel R || L || C behind Re and the voice coil)
    void setSpeakerMotional (double re, double fc, double qec, double qmc)
    {
        const double w = kTwoPi * fc, cmes = qec / (w * re);
        dk.setR (rSpkRe, re);
        dk.setC (cSpkM, cmes);
        dk.setL (lSpkM, 1.0 / (w * w * cmes));
        dk.setR (rSpkPeak, re * qmc / qec);
    }

    // 0..1 knob positions
    // input source impedance (default: the Post pot wiper, ~100k); a low value for a signal at the effects return
    void setInputSourceResistance (double r) { dk.setSourceResistance (srcIn, r); }

    void setControls (double presence, double resonance)
    {
        // VR8 is POT0752 "1M 5% A", the same part as the pre-gain pots (wiper tied to the CW end: more resistance,
        // less low-end feedback, as the knob turns up)
        dk.setR (rRes, taperPot (std::clamp (resonance, 0.0, 1.0)) * 1e6 + 100.0);
        // VR9 10KA: the resistance left in circuit falls along the audio taper, so Presence acts mostly near the top
        dk.setR (rPres, (1.0 - (std::pow (10.0, 2.0 * std::clamp (presence, 0.0, 1.0)) - 1.0) / 99.0) * 10e3 + 10.0);
    }

    // input: volts at the master (post-gain) pot wiper. output: volts across the speaker
    double process (double vin)
    {
        if (! std::isfinite (vin)) vin = lastIn; // a lost input sample: the smallest disturbance (see TriodeStage)

        double u[8] {};
        u[srcIn] = vin;
        u[srcR5] = supply.v2;              // V3A on V2+
        u[srcR46] = u[srcR58] = supply.v3; // phase inverter on V3+
        u[srcBias1] = u[srcBias2] = vBias;
        u[srcB] = supply.plate;

        const auto& p0 = dk.computeP0 (u);
        // each side's screen: the supply node minus its own 100 ohm resistor's drop (last sample's screen current; a few
        // volts at most), constant during the Newton solve
        for (int t = 0; t < 2; ++t) screenNow[t] = KorenPentode::screenTerms (supply.screen - kRscreenTube * ig2Tube[t]);
        const auto& K = dk.getK();

        // predictor: linear extrapolation of the last two solutions, switched off while it does
        // not pay off (e.g. noisy input), in which case Newton starts from the previous solution
        for (int k = 0; k < kPorts; ++k)
        {
            const double cur = p[k];
            if (usePredictor) p[k] += cur - pPrev[k];
            pPrev[k] = cur;
        }

        double i[kPorts], d[kPorts][2];
        int iterations = 0;
        auto newton = [&] (int maxIt)
        {
          for (int it = 0; it < maxIt; ++it)
          {
            ++iterations;
            evalDevices (i, d);

            double J[kPorts][kPorts + 1], maxF = 0.0;
            for (int r = 0; r < kPorts; ++r)
            {
                double Ki = 0.0;
                for (int k = 0; k < kPorts; ++k) Ki += K[(size_t) r * kPorts + k] * i[k];
                const double F = p[r] - p0[(size_t) r] - Ki;
                maxF = std::max (maxF, std::abs (F) * (r % 2 == 0 ? 1.0 : 10.0)); // grids 10x stricter
                J[r][kPorts] = -F;
            }
            // converged: plates < 0.1 mV, grids < 0.01 mV. The final Newton step is still applied (a linear
            // solve, no tube evaluation): the error drops from ~residual to ~residual^2. v0.3.0 returned
            // before it, which biased the small-signal response by up to 0.3 dB at low frequencies.
            const bool converged = maxF < resTol;

            for (int r = 0; r < kPorts; ++r)
            {
                const double* Kr = &K[(size_t) r * kPorts];
                for (int dv = 0; dv < kDevices; ++dv)
                {
                    const int c0 = 2 * dv, c1 = c0 + 1;
                    // port c0 = plate (current depends on c0, c1), port c1 = grid (depends on c1 only)
                    J[r][c0] = (r == c0 ? 1.0 : 0.0) - Kr[c0] * d[c0][0];
                    J[r][c1] = (r == c1 ? 1.0 : 0.0) - Kr[c0] * d[c0][1] - Kr[c1] * d[c1][1];
                }
            }

            double dp[kPorts];
            if (! solve (J, dp)) return converged;

            // Step limit, the same for plates and grids. Grid current and the flyback diodes are softplus
            // knees (linear beyond the knee), so an overshoot corrects itself on the next step. A tighter
            // grid limit (4 V in v0.3.0) left steep preamp edges, which swing the 6L6 grids by >100 V
            // in one sample through C28/C29, unconverged after 46 iterations -> clicks at high master.
            double maxStep = 0.0;
            for (int k = 0; k < kPorts; ++k)
            {
                constexpr double lim = 40.0;
                dp[k] = std::clamp (dp[k], -lim, lim);
                p[k] += dp[k];
                maxStep = std::max (maxStep, std::abs (dp[k]) * (k % 2 == 0 ? 1.0 : kGridStepWeight));
            }

            // quadratic convergence: after a step this small the remaining error is ~step^2, far below
            // the residual tolerance, so accept without re-evaluating the tubes. The port currents are
            // moved along their (already computed) linearisation to stay consistent with p.
            if (converged || (acceptSmallStep && maxStep < kPlateStepTol))
            {
                for (int dv = 0; dv < kDevices; ++dv)
                {
                    const int c0 = 2 * dv, c1 = c0 + 1;
                    i[c0] += d[c0][0] * dp[c0] + d[c0][1] * dp[c1];
                    i[c1] += d[c1][1] * dp[c1];
                }
                return true;
            }
          }
          return false;
        };

        // fast path from the predictor, fallback from the previous solution
        bool converged = newton (6);
        if (! converged)
        {
            std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
            converged = newton (40);
        }
        // adapt: keep predicting while it converges fast, retry it every 64 samples otherwise
        predictorScore = 0.9 * predictorScore + 0.1 * (iterations <= 2 ? 1.0 : 0.0);
        if (usePredictor && predictorScore < 0.3) usePredictor = false;
        else if (! usePredictor && (++predictorRetry & 63) == 0) { usePredictor = true; predictorScore = 0.5; }

        statSamples++;
        statIterations += iterations;
        if (testCorrupt > 0) { --testCorrupt; converged = false; for (int k = 0; k < kPorts; ++k) p[k] += 300.0; }
        if (! converged) statFailures++;
        lastConverged = converged;
        for (int k = 0; k < kPorts; ++k) lastP0[k] = p0[(size_t) k];

        if (! converged)
            evalDevices (i, d);

        // A non-finite solve is retried as a lost input sample (the last accepted input, from the last
        // solution). If that fails too, the sample is dropped: finish() has not run, so the reactive states,
        // the supply and the output node still hold the last accepted sample, and its output is repeated.
        // Zeroing the states instead (v0.4) discharged C28/C29 and the supply caps mid-note: a ~20 V step at
        // the speaker for ~7 ms (tests/dsp_test fault). Only a fault that persists for 1 ms goes back to the
        // settled idle point. An unconverged solve is treated the same unless it is nearly a solution (port residual
        // <= kAcceptResidual) and physically plausible.
        auto bad = [&] (bool conv)
        {
            if (testFaults > 0) { --testFaults; return true; }
            double sum = 0.0;
            for (int k = 0; k < kPorts; ++k) sum += i[k] + p[k];
            if (! std::isfinite (sum)) return true;
            return ! conv && ! (residual (p0, K, i) <= kAcceptResidual && plausible());
        };
        if (bad (converged))
        {
            ++statRetries;
            u[srcIn] = lastIn;
            dk.computeP0 (u); // updates p0 in place
            std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
            const bool retried = newton (40);
            if (! retried)
                evalDevices (i, d);
            if (bad (retried))
            {
                ++statHeld;
                std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
                if (++heldRun > maxHeldRun)
                    restoreIdle();
                return dk.nodeV (nS);
            }
        }
        heldRun = 0;
        lastIn = u[srcIn];
        dk.finish (i);

        // ---- power supply: the whole chain sags with this sample's currents (Supply5150) ----
        const double iPlates = i[4] + i[6];
        for (int t = 0; t < 2; ++t) ig2Tube[t] = side6L6[t].screen (p[4 + 2 * t], p[5 + 2 * t], screenNow[t].vg2k);
        const double iScreens = 2.0 * (ig2Tube[0] + ig2Tube[1]);
        const double iPI = i[0] + i[2];            // V4B + V4A plates (R46, R58 from V3+)
        const double iV2 = preI2 + i[8] + cathodeFollowerCurrent(); // + V3A plate
        if (supplyFrozen)
        {
            const double cur[5] = { iPlates, iScreens, iPI, iV2, preI1 };
            for (int k = 0; k < 5; ++k) avgI[k] += cur[k];
            ++avgN;
        }
        else
            supply.step (T, iPlates, iScreens, iPI, iV2, preI1);

        return dk.nodeV (nS);
    }

    // Newton statistics (for tests)
    long long statSamples = 0, statIterations = 0, statFailures = 0, statRetries = 0, statHeld = 0;
    int testFaults = 0; // tests: the next this-many fault checks report a bad solve
    int testCorrupt = 0; // tests: the next this-many solves end unconverged, 300 V off (must not be accepted)
    bool lastConverged = true;

    // Reference renders for the tone regression: 1000x tighter residual, no early step acceptance and
    // the original (untabulated) tube formulas
    void setReferencePrecision (bool on)
    {
        resTol = on ? 1e-7 : 1e-4;
        acceptSmallStep = ! on;
        triode.exact = tube.exact = on;
        for (auto& t : devTriode) t.exact = on;
        for (auto& t : side6L6) t.exact = on;
    }
    double lastP0[kPorts] {};
    double getPort (int k) const { return p[k]; }
    double getV3a() const { return dk.nodeV (nP3); }

    double getSupplyVoltage() const { return supply.plate; }
    double getBiasVoltage() const { return vBias; }
    double getPlate1() const { return dk.nodeV (nO1); }
    double getPIcathode() const { return dk.nodeV (nK); }
    double getPIplateA() const { return dk.nodeV (nPA); }
    double getPIplateB() const { return dk.nodeV (nPB); }

private:
    // largest port-equation residual (V, grids weighted 10x) of the state p with device currents i
    template <typename P0, typename KM>
    double residual (const P0& p0, const KM& K, const double (&i)[kPorts]) const
    {
        double maxF = 0.0;
        for (int r = 0; r < kPorts; ++r)
        {
            double Ki = 0.0;
            for (int k = 0; k < kPorts; ++k) Ki += K[(size_t) r * kPorts + k] * i[k];
            maxF = std::max (maxF, std::abs (p[r] - p0[(size_t) r] - Ki) * (r % 2 == 0 ? 1.0 : 10.0));
        }
        return maxF;
    }

    // port voltages the tubes can physically have (plate, grid), with wide margins
    bool plausible() const
    {
        for (int k = 0; k < kPorts; k += 2)
            if (! (p[k] > -500.0 && p[k] < 1500.0 && p[k + 1] > -1000.0 && p[k + 1] < 100.0)) return false;
        return true;
    }

    void evalDevices (double (&i)[kPorts], double (&d)[kPorts][2])
    {
        for (int pk : { 0, 2, 8 }) // V4B, V4A, V3A
        {
            const int gk = pk + 1;
            double ip, dvpk, dvgk, ig, dig;
            devTriode[pk == 8 ? 2 : pk / 2].eval (p[pk], p[gk], ip, dvpk, dvgk);
            gridCurrent (p[gk], ig, dig);
            i[pk] = ip; d[pk][0] = dvpk; d[pk][1] = dvgk;
            i[gk] = ig; d[gk][0] = 0.0;  d[gk][1] = dig;
        }
        for (int t = 0; t < 2; ++t) // 6L6 pairs (2 tubes each)
        {
            const int pk = 4 + 2 * t, gk = pk + 1;
            double ip, dvpk, dvg1, ig, dig;
            side6L6[t].plate (p[pk], p[gk], screenNow[t], ip, dvpk, dvg1);
            gridCurrentR (p[gk], 1000.0, ig, dig);
            // CR200 / CR201 flyback diodes: conduct from ground into the plate when it swings below 0 V
            constexpr double von = 0.6, vd = 0.03, rd = 1.0;
            const double xd = (-p[pk] - von) * (1.0 / vd);
            double idio = 0.0, didio = 0.0;
            if (xd > 30.0)       { idio = (-p[pk] - von) / rd; didio = 1.0 / rd; }
            else if (xd > -30.0)
            {
                const double ed = std::exp (xd);
                idio = vd / rd * std::log1p (ed);
                didio = ed / (1.0 + ed) / rd;
            }

            i[pk] = 2.0 * ip - idio; d[pk][0] = 2.0 * dvpk + didio; d[pk][1] = 2.0 * dvg1;
            i[gk] = 2.0 * ig;        d[gk][0] = 0.0;                d[gk][1] = 2.0 * dig;
        }
    }

    // Gaussian elimination with partial pivoting; one division per pivot
    static bool solve (double (&m)[kPorts][kPorts + 1], double (&x)[kPorts])
    {
        constexpr int n = kPorts;
        double invPivot[n];
        for (int col = 0; col < n; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < n; ++r)
                if (std::abs (m[r][col]) > std::abs (m[piv][col])) piv = r;
            if (std::abs (m[piv][col]) < 1e-300) return false;
            if (piv != col)
                for (int j = 0; j <= n; ++j) std::swap (m[col][j], m[piv][j]);
            invPivot[col] = 1.0 / m[col][col];
            for (int r = col + 1; r < n; ++r)
            {
                const double f = m[r][col] * invPivot[col];
                if (f == 0.0) continue;
                for (int j = col; j <= n; ++j) m[r][j] -= f * m[col][j];
            }
        }
        for (int r = n - 1; r >= 0; --r)
        {
            double s = m[r][n];
            for (int j = r + 1; j < n; ++j) s -= m[r][j] * x[j];
            x[r] = s * invPivot[r];
        }
        return true;
    }

    // V3B (cathode follower to the effects loop, plate on V2+; signal not modelled, it is linear at normal Master
    // settings): its DC current, cathode -> R80 1k -> R16 100k -> ground, grid leak R79 to the R80 / R16 junction.
    // One Newton step per sample from the last value (V2+ moves slowly).
    double cathodeFollowerCurrent()
    {
        double ip, dp, dg;
        triode.eval (supply.v2 - cfIk * 101e3, -cfIk * 1e3, ip, dp, dg);
        const double f = ip - cfIk, df = -dp * 101e3 - dg * 1e3 - 1.0;
        cfIk = std::clamp (cfIk - f / df, 1e-6, 1e-2);
        return cfIk;
    }

    // grid bias giving the target idle current per output tube
    double findBias (double vp, double vg2) const
    {
        double lo = -100.0, hi = 0.0;
        for (int it = 0; it < 60; ++it)
        {
            const double mid = 0.5 * (lo + hi);
            double ip, a, b;
            tube.plate (vp, mid, vg2, ip, a, b);
            (ip > kIdlePerTube ? hi : lo) = mid;
        }
        return 0.5 * (lo + hi);
    }

    void restoreIdle()
    {
        dk.restoreState();
        std::copy (std::begin (pIdle), std::end (pIdle), std::begin (p));
        std::copy (std::begin (pIdle), std::end (pIdle), std::begin (pPrev));
        supply.plate = plateIdle; supply.screen = screenIdle;
        std::copy (std::begin (ig2Idle), std::end (ig2Idle), std::begin (ig2Tube));
        supply.v3 = railIdle[0]; supply.v2 = railIdle[1]; supply.v1 = railIdle[2];
        heldRun = 0;
        lastIn = 0.0;
    }

    // Newton step (V) below which a solve is accepted without a confirming evaluation: plates 30 mV,
    // grids 6 mV (weighted 5x: the grid-current knee is only 30 mV wide). Tuned with tone_regress:
    // -125 dB or better against the reference solution.
    static constexpr double kPlateStepTol = 3e-2, kGridStepWeight = 5.0;
    // an unconverged solve is kept up to this residual (V): 10 mV, still inaudible
    static constexpr double kAcceptResidual = 1e-2;
    double resTol = 1e-4;          // port-equation residual (V, grids weighted 10x)
    bool acceptSmallStep = true;

    DKCircuit dk;
    KorenTriode triode;          // nominal: V3B (cathode follower, DC only)
    KorenPentode tube;           // nominal: sets the fixed bias (findBias)
    KorenTriode devTriode[3];    // V4B, V4A, V3A (each its own tube: setTubeVariation)
    KorenPentode side6L6[2];     // the two output pairs, each the average of its two tubes

    int nA = 0, nP1 = 0, nGB = 0, nGA = 0, nK = 0, nQ = 0, nL = 0, nM = 0, nPr = 0, nY = 0;
    int nPB = 0, nPA = 0, nC1 = 0, nG1 = 0, nC2 = 0, nG2 = 0, nO1 = 0, nO2 = 0;
    int nCT = 0, nS = 0, nSp1 = 0, nSp2 = 0, nIn = 0, nG3 = 0, nK3 = 0, nP3 = 0, nSec = 0, lLeak = 0;
    int srcIn = 0, srcR5 = 0, srcR46 = 0, srcR58 = 0, srcBias1 = 0, srcBias2 = 0, srcB = 0;
    int rRes = 0, rPres = 0;
    int cWinding = 0, lVoice = 0, rVoiceLoss = 0, rSpkPeak = 0, rSpkRe = 0, lSpkM = 0, cSpkM = 0;

    double fs = 192000.0, T = 1.0 / 192000.0;
    Supply5150 supply;
    double vBias = -45.0;
    KorenPentode::Screen screenNow[2] { { 485.0, 1.0 / 485.0, 0.0 }, { 485.0, 1.0 / 485.0, 0.0 } };
    double ig2Tube[2] {}, ig2Idle[2] {}; // screen current of one tube per side (last sample)
    double p[kPorts] {}, pPrev[kPorts] {};
    double pIdle[kPorts] {}, plateIdle = 490.0, screenIdle = 485.0, lastIn = 0.0;
    double railIdle[3] { 449.9, 322.1, 305.7 };
    double preI1 = kPreampIdleI1, preI2 = kPreampIdleI2, cfIk = 1.25e-3;
    bool supplyFrozen = false;
    double avgI[5] {}; long long avgN = 0;
    int heldRun = 0, maxHeldRun = 352; // consecutive dropped samples; beyond the limit -> restoreIdle()
    bool usePredictor = true;
    double predictorScore = 1.0;
    unsigned predictorRetry = 0;
};

} // namespace vamp
