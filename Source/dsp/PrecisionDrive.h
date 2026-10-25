#pragma once
#include <algorithm>
#include <cmath>
#include "Filters.h"

namespace vamp
{
// Component-level model of the Tube Screamer family of op-amp overdrives. The Horizon Devices Precision Drive is one
// (PedalPCB's "Dwarven Hammer" build document, rev 12.29.20), the Ibanez TS9 another (tonepad "El Griton 2.2"
// TS-808 / TS-9 layout, rev Nov 28 2004): the same circuit with different values (Circuit).
//
//  in -> 27n / 510k -> Q1 2N3904 follower (R10 10k) -> 1u / 10k (to Vref) ->
//  IC1.1 non-inverting clipper:
//      feedback: (rf0 + DRIVE A500k) || cf || two 1N4148 / 1N914 antiparallel
//      to Vref: rLeg + leg cap (PD: ATTACK C10..C17 33n..470n || C9 120p; TS9: 47n)
//  -> 1k / 220n / 10k -> IC1.2 tone stage: pot (PD BRIGHT B5k, TS9 TONE B20k) between its inputs, the wiper to
//     220n + 220R to ground, feedback toneRf (PD 3k9, TS9 1k)
//  -> 1u / 1k -> VOLUME / LEVEL A100k -> 100n -> Q2 2N3904 follower (R13 10k) -> ra -> 10u -> rb (PD 100R / 10k,
//     TS9 470R / 100k) -> out
//
// The clipping stage is solved per sample with Newton's method: capacitors by the trapezoidal rule, IC1.1 (JRC4558)
// as a single-pole op-amp (100 dB, 3 MHz). Its unity-gain pole is far above Nyquist, so its state uses backward Euler
// (L-stable; the trapezoidal rule would leave a slowly decaying ring at fs/2): at full Drive the closed-loop
// bandwidth is ~6 kHz and the top end falls 3-5 dB below an ideal op-amp's (spice/precision_drive.cir). The diodes'
// junction capacitance (Cjo 4 pF each) adds to cf. The emitter followers and the Volume pot cost a fixed ~0.44 dB;
// IC1.2 (gain < 6) is taken as ideal.
class OpAmpDrive
{
public:
    struct Circuit
    {
        double rf0;        // fixed feedback resistor in series with the Drive pot
        double cf;         // feedback capacitor (without the diodes' junction capacitance)
        double rLeg;       // inverting input -> leg cap -> Vref
        double legCapExtra;// capacitor always in parallel with the leg cap
        double toneP;      // tone pot (linear)
        double toneRf;     // tone stage feedback resistor
        double ra, rb;     // output: series resistor, load to ground
        bool levelLinear;  // Volume / Level pot: B (linear) or A (audio)
    };

    // Horizon Devices Precision Drive (Dwarven Hammer): R15 10k, C8 47p, R16 1k, C9 120p, BRIGHT B5k, R8 3k9,
    // R11 100R, R14 10k, VOLUME A100k
    static constexpr Circuit precisionDrive { 10e3, 47e-12, 1e3, 120e-12, 5e3, 3.9e3, 100.0, 10e3, false };
    // Ibanez TS9: 51k, 51p, 4k7 (+ 47n), TONE B20k, 1k, output 470R / 100k (TS-808: 100R / 10k), LEVEL B100k (tonepad;
    // a calibrated capture of a real TS9 gives the same output at Level 5 and 10 only with a linear pot)
    static constexpr Circuit ts9 { 51e3, 51e-12, 4.7e3, 0.0, 20e3, 1e3, 470.0, 100e3, true };

    explicit OpAmpDrive (const Circuit& c = precisionDrive) : circ (c) {}

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        T = 1.0 / fs;
        // input: 27n into 510k || Q1's input (beta (R10 || 10k) ~ 1.5M) + 1k, then 1u into 10k
        inHP1.set (1.0 / (2.0 * kPi * (510e3 * 1.5e6 / (510e3 + 1.5e6) + 1e3) * 27e-9), fs);
        inHP2.set (1.0 / (2.0 * kPi * 10e3 * 1e-6), fs);
        // op-amp: vo' = wp (A0 (v+ - v-) - vo), backward Euler: vo = (vo_prev + h wp A0 e) / (1 + h wp)
        const double hwp = T * 2.0 * kPi * kGbw / kA0;
        opDen = 1.0 / (1.0 + hwp);
        opGain = hwp * kA0 * opDen;
        // fixed linear losses: Q1 (re ~79R into R10 || 10k), 1k into the 100k Volume pot, Q2 (re ~78R into
        // R13 || (ra + rb)), ra into rb (spice/precision_drive.cir: ~-0.44 dB for the Precision Drive)
        const double q2Load = 10e3 * (circ.ra + circ.rb) / (10e3 + circ.ra + circ.rb);
        bufferGain = 5000.0 / 5078.6 * 100.0 / 101.0 * q2Load / (q2Load + 78.0) * circ.rb / (circ.ra + circ.rb);
        updateCoeffs();
        reset();
    }

    void reset()
    {
        inHP1.reset(); inHP2.reset();
        vf = 0.0; vc = 0.0; iLeg = 0.0; vo = 0.0;
        g0 = 0.0;
        x1 = x2 = y1 = y2 = 0.0;
    }

    // drive / tone / volume 0..1 (knob rotation), legCap: the leg capacitor (F)
    void setControls (double drive, double tone, double legCap, double volume)
    {
        const double rf = circ.rf0 + 500e3 * taper (std::clamp (drive, 0.0, 1.0));
        const double ca = legCap + circ.legCapExtra;
        const double a = std::clamp (tone, 0.0, 1.0);
        if (rf != Rf || ca != Ca || a != tonePos)
        {
            Rf = rf; Ca = ca; tonePos = a;
            updateCoeffs();
        }
        volumeGain = circ.levelLinear ? std::clamp (volume, 0.0, 1.0) : taper (std::clamp (volume, 0.0, 1.0));
    }

    double process (double vin)
    {
        // input buffer coupling (two first-order high-passes), Q1's emitter-follower loss in bufferGain
        const double v = inHP2.process ((float) inHP1.process ((float) vin));

        // Unknown: the feedback voltage x = vo - vn (output minus inverting input), all relative to Vref.
        // op-amp (backward Euler):    vo = a + b vn,  a = opDen vo_prev + opGain v,  b = -opGain
        //   -> vn = (x - a) / (b - 1)
        // leg rLeg + Ca (trapezoidal, from vn to Vref): i_leg = gL vn - jL
        // KCL at vn: the leg current flows through the feedback (Rf, the diodes, cf + diode capacitance); the
        // capacitor takes the rest, i_C = i_leg - x / Rf - 2 Is sinh (x / nVt), and by the trapezoidal rule
        //   k2 (x - vf) = i_C(now) + g0      (vf, g0: last sample's x and i_C)
        const double halfT = T / (2.0 * Ca);
        const double gL = 1.0 / (circ.rLeg + halfT), jL = (vc + halfT * iLeg) * gL;
        const double a = opDen * vo + opGain * v, b = -opGain;
        const double invBm1 = 1.0 / (b - 1.0);
        const double k2 = 2.0 * (circ.cf + kCjDiodes) / T, invRf = 1.0 / Rf;
        constexpr double invNVt = 1.0 / nVt;
        double x = vf;
        for (int it = 0; it < 24; ++it)
        {
            const double e = std::exp (std::clamp (x * invNVt, -40.0, 40.0)), ie = 1.0 / e;
            const double vn = (x - a) * invBm1;
            const double F = k2 * (x - vf) + x * invRf + Is * (e - ie) - (gL * vn - jL) - g0;
            const double dF = k2 + invRf + Is * invNVt * (e + ie) - gL * invBm1;
            const double dx = std::clamp (-F / dF, -0.1, 0.1);
            x += dx;
            if (std::abs (dx) < 1e-9) break;
        }
        const double vn = (x - a) * invBm1;
        // op-amp output, limited by the 9 V rails (never reached with guitar levels: the diodes clamp x)
        vo = std::clamp (x + vn, -3.5, 3.5);
        const double e = std::exp (std::clamp (x * invNVt, -40.0, 40.0));
        const double iNew = gL * vn - jL; // leg current now
        g0 = iNew - x * invRf - Is * (e - 1.0 / e);
        vf = x;

        // leg capacitor state (trapezoidal)
        vc += halfT * (iNew + iLeg);
        iLeg = iNew;

        // passive LPF + tone stage (2nd-order, bilinear)
        const double y = b0 * vo + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = vo; y2 = y1; y1 = y;

        return y * volumeGain * bufferGain;
    }

private:
    static double taper (double x) { return (std::pow (10.0, 2.0 * x) - 1.0) / 99.0; }

    // 1k / 220n / 10k into the tone stage: pot P between the op-amp's inputs (a = wiper position from the 1k / 220n
    // node end, i.e. 1 = towards the inverting input: brightest), wiper to Ct + Rz to ground, feedback R8
    // H(s) = G7 (1 + s Ct (Rs + a R8)) / [ (G7+G3) + s((G7+G3) Ct Rs + C7 + (1-a) Ct) + s^2 C7 Ct Rs ]
    void updateCoeffs()
    {
        constexpr double R7 = 1e3, R3 = 10e3, C7 = 220e-9, Ct = 220e-9, Rz = 220.0;
        const double R8 = circ.toneRf, P = circ.toneP;
        const double a = tonePos;
        const double Rs = Rz + a * (1.0 - a) * P;
        const double G7 = 1.0 / R7, G3 = 1.0 / R3;

        const double n0 = G7, n1 = G7 * Ct * (Rs + a * R8), n2 = 0.0;
        const double d0 = G7 + G3, d1 = (G7 + G3) * Ct * Rs + C7 + (1.0 - a) * Ct, d2 = C7 * Ct * Rs;

        const double K = 2.0 * fs, K2 = K * K;
        const double B0 = n0 + n1 * K + n2 * K2, B1 = 2.0 * n0 - 2.0 * n2 * K2, B2 = n0 - n1 * K + n2 * K2;
        const double A0 = d0 + d1 * K + d2 * K2, A1 = 2.0 * d0 - 2.0 * d2 * K2, A2 = d0 - d1 * K + d2 * K2;

        b0 = B0 / A0; b1 = B1 / A0; b2 = B2 / A0;
        a1 = A1 / A0; a2 = A2 / A0;
    }

    // the two diodes' junction capacitance (Cjo 4 pF; they sit near 0 V bias: one barely forward, one barely reverse)
    static constexpr double kCjDiodes = 2.0 * 4e-12;
    static constexpr double kA0 = 1e5, kGbw = 3e6; // JRC4558: open-loop gain, gain-bandwidth product (Hz)
    static constexpr double Is = 2.52e-9, nVt = 1.752 * 0.02585; // 1N4148 (the 1N914 is the same die)

    Circuit circ;
    double fs = 192000.0, T = 1.0 / 192000.0;
    double Rf = 10e3, Ca = 100e-9, tonePos = 0.5, volumeGain = 0.5, bufferGain = 1.0;

    OnePoleHP inHP1, inHP2;
    double vf = 0.0, vc = 0.0, iLeg = 0.0, g0 = 0.0, vo = 0.0;
    double opDen = 1.0, opGain = 0.0;
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};

// Horizon Devices Precision Drive: Volume, Drive, Bright, Attack (the leg cap). The real pedal's Attack has six
// positions, 470n 330n 100n 68n 47n 33n (Neural DSP's Archetype: Misha Mansoor X, the official model), 1 the fullest
// and 6 the tightest; NAM captures of a real pedal at Attack 1 / 2 / 3 fit 470n / 330n / 100n best. The Dwarven Hammer
// clone uses an 8-way switch with 220n and 82n added (lug 1 = 470n ... lug 8 = 33n).
class PrecisionDrive : public OpAmpDrive
{
public:
    static constexpr int numAttack = 6;
    static constexpr double attackCaps[numAttack] = { 470e-9, 330e-9, 100e-9, 68e-9, 47e-9, 33e-9 };

    PrecisionDrive() : OpAmpDrive (precisionDrive) {}

    // drive / bright / volume 0..1, attack 0..5 = the knob's 1..6 (0 = fullest, 5 = tightest)
    void setControls (double drive, double bright, int attack, double volume)
    {
        OpAmpDrive::setControls (drive, bright, attackCaps[std::clamp (attack, 0, numAttack - 1)], volume);
    }
};

// Ibanez TS9 Tube Screamer: Drive, Tone, Level (the leg cap is a fixed 47n: the 720 Hz "mid hump" with 4k7)
class TubeScreamer9 : public OpAmpDrive
{
public:
    TubeScreamer9() : OpAmpDrive (ts9) {}

    // drive / tone / level 0..1
    void setControls (double drive, double tone, double level) { OpAmpDrive::setControls (drive, tone, 47e-9, level); }
};

} // namespace vamp
