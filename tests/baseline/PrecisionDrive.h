#pragma once
#include <algorithm>
#include <cmath>
#include "Filters.h"

namespace vamp_baseline
{
// Component-level model of the Horizon Devices Precision Drive circuit,
// as published in PedalPCB's "Dwarven Hammer" build document (rev 12.29.20).
//
//  in -> C2 27n / R1 510k -> Q1 buffer -> C3 1u / R2 10k ->
//  IC1.1 non-inverting clipper:
//      feedback: (R15 10k + DRIVE A500k) || C8 47p || D1/D2 1N4148 antiparallel
//      to ground: R16 1k + ATTACK cap (C10..C17: 33n..470n) || C9 120p
//  -> R7 1k / C7 220n / R3 10k -> IC1.2 "Bright" tone stage (B5k, C5 220n + R12 220R, R8 3k9)
//  -> VOLUME A100k -> Q2 buffer -> out
//
// The clipping stage is solved per sample with Newton's method (trapezoidal rule).
class PrecisionDrive
{
public:
    static constexpr double attackCaps[8] = { 33e-9, 47e-9, 68e-9, 82e-9, 100e-9, 220e-9, 330e-9, 470e-9 };

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        T = 1.0 / fs;
        inHP1.set (1.0 / (2.0 * kPi * 510e3 * 27e-9), fs);
        inHP2.set (1.0 / (2.0 * kPi * 10e3 * 1e-6), fs);
        updateCoeffs();
        reset();
    }

    void reset()
    {
        inHP1.reset(); inHP2.reset();
        vf = 0.0; vc = 0.0; iLeg = 0.0;
        g0 = 0.0;
        x1 = x2 = y1 = y2 = 0.0;
    }

    // drive/bright/volume 0..1, attack 0..7 (0 = tightest, 7 = fullest)
    void setControls (double drive, double bright, int attack, double volume)
    {
        const double rf = 10e3 + 500e3 * taper (drive);
        const double ca = attackCaps[std::clamp (attack, 0, 7)] + 120e-12;
        const double a = std::clamp (bright, 0.0, 1.0);
        if (rf != Rf || ca != Ca || a != brightPos)
        {
            Rf = rf; Ca = ca; brightPos = a;
            updateCoeffs();
        }
        volumeGain = taper (std::clamp (volume, 0.0, 1.0));
    }

    double process (double vin)
    {
        // input buffer coupling (two first-order high-passes)
        const double v = inHP2.process ((float) inHP1.process ((float) vin));

        // attack leg: R16 + Ca in series (trapezoidal), current into op-amp (-) node
        const double halfT = T / (2.0 * Ca);
        const double iNew = (v - vc - halfT * iLeg) / (R16 + halfT);
        vc += halfT * (iNew + iLeg);
        iLeg = iNew;

        // feedback voltage vf: 2Cf/T (vf - vf0) = g(vf) + g0,  g(v) = iLeg - v/Rf - 2 Is sinh(v/nVt)
        // sinh and cosh from one exp: 2 Is sinh(u) = Is (e - 1/e), 2 Is cosh(u) = Is (e + 1/e)
        const double k2 = 2.0 * Cf / T, invRf = 1.0 / Rf;
        constexpr double invNVt = 1.0 / nVt;
        double x = vf;
        for (int it = 0; it < 16; ++it)
        {
            const double e = std::exp (std::clamp (x * invNVt, -40.0, 40.0)), ie = 1.0 / e;
            const double F = k2 * (x - vf) - (iLeg - x * invRf - Is * (e - ie)) - g0;
            const double dF = k2 + invRf + Is * invNVt * (e + ie);
            const double dx = std::clamp (-F / dF, -0.1, 0.1);
            x += dx;
            if (std::abs (dx) < 1e-9) break;
        }
        vf = x;
        const double e = std::exp (std::clamp (vf * invNVt, -40.0, 40.0));
        g0 = iLeg - vf * invRf - Is * (e - 1.0 / e);

        // op-amp output (relative to Vref), limited by the 9 V rails
        const double out1 = std::clamp (v + vf, -3.5, 3.5);

        // passive LPF + Bright stage (2nd-order, bilinear)
        const double y = b0 * out1 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1; x1 = out1; y2 = y1; y1 = y;

        return y * volumeGain;
    }

private:
    static double taper (double x) { return (std::pow (10.0, 2.0 * x) - 1.0) / 99.0; }

    // H(s) = G7 (1 + s Ct (Rs + a R8)) / [ (G7+G3) + s((G7+G3) Ct Rs + C7 + (1-a) Ct) + s^2 C7 Ct Rs ]
    void updateCoeffs()
    {
        constexpr double R7 = 1e3, R3 = 10e3, C7 = 220e-9, Ct = 220e-9, Rz = 220.0, R8 = 3.9e3, P = 5e3;
        const double a = brightPos;
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

    static constexpr double R16 = 1e3, Cf = 47e-12;
    static constexpr double Is = 2.52e-9, nVt = 1.752 * 0.02585; // 1N4148

    double fs = 192000.0, T = 1.0 / 192000.0;
    double Rf = 10e3, Ca = 100e-9, brightPos = 0.5, volumeGain = 0.5;

    OnePoleHP inHP1, inHP2;
    double vf = 0.0, vc = 0.0, iLeg = 0.0, g0 = 0.0;
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};

} // namespace vamp_baseline
