#pragma once
#include <algorithm>
#include <cmath>

namespace vamp
{
// softplus(x)^ex = log(1 + e^x)^ex and its derivative, the transcendental core of Koren's tube
// models. Tabulated for |x| < 30 as a cubic Hermite spline (h = 1/32, ~30 KB, relative error < 1e-8,
// far below the Newton tolerances); beyond, softplus is e^x or x, exactly as Koren's formula is
// evaluated. Replaces 3 serial exp/log calls (~40 ns) by one lookup.
class SoftplusPow
{
public:
    explicit SoftplusPow (double exponent) : ex (exponent)
    {
        for (int k = 0; k <= kN; ++k)
        {
            const double x = kLo + k * kH;
            const double e = std::exp (x), sp = std::log1p (e), sig = e / (1.0 + e);
            const double spe1 = std::pow (sp, ex - 1.0);
            table[2 * k] = spe1 * sp;                 // sp^ex
            table[2 * k + 1] = kH * ex * spe1 * sig;  // h * d/dx sp^ex
        }
    }

    inline void eval (double x, double& G, double& dG) const
    {
        if (x >= kLo && x < kHi)
        {
            const double u = (x - kLo) * (1.0 / kH);
            const int i = (int) u;
            const double t = u - i, t2 = t * t, t3 = t2 * t;
            const double* c = &table[2 * i]; // g0, h*d0, g1, h*d1
            G = (2.0 * t3 - 3.0 * t2 + 1.0) * c[0] + (t3 - 2.0 * t2 + t) * c[1]
              + (3.0 * t2 - 2.0 * t3) * c[2] + (t3 - t2) * c[3];
            dG = ((6.0 * t2 - 6.0 * t) * (c[0] - c[2]) + (3.0 * t2 - 4.0 * t + 1.0) * c[1]
                  + (3.0 * t2 - 2.0 * t) * c[3]) * (1.0 / kH);
        }
        else if (x >= kHi) // softplus = x
        {
            const double p = std::exp ((ex - 1.0) * std::log (x));
            G = p * x;
            dG = ex * p;
        }
        else // softplus = e^x (NaN lands here too and propagates)
        {
            G = std::exp (ex * x);
            dG = ex * G;
        }
    }

private:
    static constexpr double kLo = -30.0, kHi = 30.0, kH = 1.0 / 32.0;
    static constexpr int kN = 1920; // (kHi - kLo) / kH
    double ex;
    double table[2 * (kN + 1)];
};

// Norman Koren's triode model (same equations as spice/12AX7.inc)
struct KorenTriode
{
    static constexpr double mu = 100.0, ex = 1.4, kg1 = 1060.0, kp = 600.0, kvb = 300.0;
    // interelectrode capacitances in circuit (Koren's values: tube + socket + wiring; the bare-tube datasheet
    // 1.6 / 1.7 pF fit the rhythm [PRE] capture slightly better but the full-amp captures worse, so not used)
    static constexpr double cgk = 2.3e-12, cgp = 2.4e-12;

    static const SoftplusPow& softplusPow() { static const SoftplusPow t (ex); return t; }

    bool exact = false; // reference renders: original formula instead of the table

    // This tube's deviation from the nominal one (TubeSet): plate current (and gm with it) x currentScale,
    // amplification factor x muScale. 1 / 1: exactly the nominal model.
    void setVariation (double currentScale, double muScale)
    {
        kpOverMu = kp / (mu * muScale);
        twoOverKg1 = 2.0 / kg1 * currentScale;
    }

    // Plate current and its partial derivatives w.r.t. Vpk and Vgk:
    //   ip = 2/kg1 (vpk/kp)^ex softplus(x)^ex,   x = kp (1/mu + vgk / sqrt(kvb + vpk^2))
    // (vpk/kp)^(ex-1) and the softplus table are independent, so the CPU overlaps them.
    inline void eval (double vpk, double vgk, double& ip, double& dVpk, double& dVgk) const
    {
        if (exact) { evalExact (vpk, vgk, ip, dVpk, dVgk); return; }
        if (vpk <= 0.0) { ip = 0.0; dVpk = 0.0; dVgk = 0.0; return; }
        constexpr double invKp = 1.0 / kp;
        const double r = vpk * invKp;
        const double rp = std::exp ((ex - 1.0) * std::log (r)); // (vpk/kp)^(ex-1)
        const double invS = 1.0 / std::sqrt (kvb + vpk * vpk);
        const double x = kpOverMu + kp * vgk * invS;

        double G, dG;
        softplusPow().eval (x, G, dG);

        const double a = twoOverKg1 * r * rp; // 2/kg1 (vpk/kp)^ex
        ip = a * G;
        dVgk = a * dG * kp * invS;
        dVpk = twoOverKg1 * ex * rp * invKp * G - a * dG * kp * vgk * vpk * invS * invS * invS;
    }

    // The same with the original formula (3 serial exp/log calls): reference for the tests
    inline void evalExact (double vpk, double vgk, double& ip, double& dVpk, double& dVgk) const
    {
        constexpr double invKp = 1.0 / kp;
        const double invS = 1.0 / std::sqrt (kvb + vpk * vpk);
        const double x = kpOverMu + kp * vgk * invS;

        double sp, sig;
        if (x > 30.0)       { sp = x; sig = 1.0; }
        else if (x < -30.0) { sp = std::exp (x); sig = sp; }
        else
        {
            const double e = std::exp (x);
            sp = std::log1p (e);
            sig = e / (1.0 + e);
        }

        const double E = vpk * invKp * sp;
        if (E <= 0.0) { ip = 0.0; dVpk = 0.0; dVgk = 0.0; return; }

        const double Ep = std::exp ((ex - 1.0) * std::log (E)); // == pow (E, ex - 1), faster
        ip = twoOverKg1 * Ep * E;
        const double dIdE = twoOverKg1 * ex * Ep;
        const double vs = vpk * sig * invS;
        dVgk = dIdE * vs;
        dVpk = dIdE * (sp * invKp - vpk * vgk * vs * invS * invS);
    }

private:
    double kpOverMu = kp / mu, twoOverKg1 = 2.0 / kg1; // nominal unless setVariation
};

// Grid conduction: diode-like knee (~0.25 V) into ~2k grid resistance
inline void gridCurrent (double vgk, double& ig, double& dIg)
{
    constexpr double von = 0.25, vd = 0.03, rgk = 2000.0;
    const double x = (vgk - von) * (1.0 / vd);
    if (x < -30.0)      { ig = 0.0; dIg = 0.0; return; }
    if (x > 30.0)       { ig = (vgk - von) * (1.0 / rgk); dIg = 1.0 / rgk; return; }
    const double e = std::exp (x);
    ig = vd / rgk * std::log1p (e);
    dIg = e / (1.0 + e) * (1.0 / rgk);
}

// One common-cathode 12AX7 stage with its input coupling network:
//
//   vin --[rsrc]--||cc||-- A --[rs]-- grid        plate --[rp]-- B+
//                          |           |          cathode --[rk || ck || (rk2 + ck2)]-- gnd
//                       [ra||caShunt] [rb]        optional rfb: AC plate -> grid feedback
//                          |           |
//                         gnd         gnd
struct TriodeConfig
{
    double vb = 320.0;
    double rsrc = 1.0;     // series resistance before the coupling cap
    double cc = 0.0;       // coupling cap (0 = DC coupled)
    double ra = 0.0;       // node A to ground (0 = none)
    double caShunt = 0.0;  // node A cap to ground (0 = none)
    double rs = 1000.0;    // node A to grid
    double rb = 0.0;       // grid to ground (0 = none)
    double rp = 100e3;
    double rk = 1.5e3;
    double ck = 0.0;       // cathode bypass cap (0 = unbypassed)
    double rk2 = 0.0, ck2 = 0.0; // extra series R-C cathode branch (ck2 = 0 -> none)
    double rfb = 0.0;      // plate-to-grid feedback resistor (0 = none)
};

class TriodeStage
{
public:
    void configure (const TriodeConfig& c, double sampleRate)
    {
        KorenTriode::softplusPow(); // builds the table on first use: here, not on the audio thread
        cfg = c;
        T = 1.0 / sampleRate;
        maxHeldRun = std::max (1, (int) (1e-3 * sampleRate));
        derive();
        initDC();
    }

    // vin is the AC input voltage, returns the AC plate voltage (plate - quiescent plate)
    double process (double vin)
    {
        // A non-finite input sample is treated as lost: replaced by the last accepted one, the stage then advances
        // normally. That is the smallest disturbance possible; skipping the update instead upsets the trapezoidal
        // capacitor companions (a one-sample current spike through Cgp / Cgk that the following stages amplify:
        // ~4 V at the speaker vs ~0.1 V, tests/dsp_test fault).
        if (! std::isfinite (vin)) vin = lastVin;

        double alpha = (Gs * (vin - h) + IeqA) * invGAtot;
        const double beta = betaA;

        // Newton solve of the 3 node equations (grid, cathode, plate); true when the KCL residual < 1 nA
        auto newton = [&] (double& g, double& k, double& p, int maxIt)
        {
          for (int it = 0; it < maxIt; ++it)
          {
            ++statEvals;
            double ip, ipp, ipg, ig, igd;
            tube.eval (p - k, g - k, ip, ipp, ipg);
            gridCurrent (g - k, ig, igd);

            const double pac = p - vPdc;
            const double iGP = Gcgp * (g - p) - IeqGP; // grid-plate (Miller) capacitance
            const double iGK = Gcgk * (g - k) - IeqGK; // grid-cathode capacitance
            const double F1 = Grs * (alpha + beta * g - g) - Gb * g - ig + Gfb * (pac - g) - iGP - iGK;
            const double F2 = ip + ig - (Gk + GcK) * k + IeqK - Gk2 * (k - h2) + iGK;
            const double F3 = (cfg.vb - p) * Gp - ip - Gfb * (pac - g) + iGP;

            // converged once the KCL residual is below 2 nA (~0.2 mV on a 100k node). The final Newton
            // step is still applied: it costs one 3x3 solve but no tube evaluation, and leaves an error
            // of ~residual^2 instead of ~residual (v0.3.0 returned before it, biasing small signals)
            const bool converged = std::abs (F1) + std::abs (F2) + std::abs (F3) < resTol;

            double J[3][4] = {
                { Grs * (beta - 1.0) - Gb - igd - Gfb - Gcgp - Gcgk, igd + Gcgk,                                  Gfb + Gcgp,               -F1 },
                { ipg + igd + Gcgk,                                   -ipg - ipp - igd - Gk - GcK - Gk2 - Gcgk,   ipp,                      -F2 },
                { -ipg + Gfb + Gcgp,                                  ipg + ipp,                                  -Gp - ipp - Gfb - Gcgp,   -F3 },
            };

            double d[3];
            if (! solve3 (J, d))
                return converged;

            d[0] = std::clamp (d[0], -5.0, 5.0);
            d[1] = std::clamp (d[1], -5.0, 5.0);
            d[2] = std::clamp (d[2], -50.0, 50.0);
            g += d[0]; k += d[1]; p += d[2];

            // Newton converges quadratically: after a step this small the remaining error is ~step^2
            // (far below the residual tolerance), so the confirming evaluation can be skipped
            const double tolGK = g - k < kGridOff ? kStepTolGKOff : kStepTolGK;
            if (converged || (acceptSmallStep && std::abs (d[0]) + std::abs (d[1]) < tolGK && std::abs (d[2]) < kStepTolP))
                return true;
          }
          return false;
        };

        // fast path: start from a linear extrapolation of the last two solutions;
        // fallback: restart from the previous solution with more iterations
        double g = 2.0 * vG - prevG, k = 2.0 * vK - prevK, p = 2.0 * vP - prevP;
        ++statSamples;
        if (! newton (g, k, p, 6))
        {
            ++statFallbacks;
            g = vG; k = vK; p = vP;
            if (! newton (g, k, p, 30)) ++statFailures;
        }

        // A non-finite or out-of-range solve is retried as a lost input sample (the last accepted input, from the
        // last solution). If that fails too, the sample is dropped: the states still hold the last accepted
        // sample, and its output is repeated. Resetting to the quiescent point instead (v0.4) discharged the
        // coupling / cathode caps mid-note: a step the following stages amplify, >100 V at the speaker for
        // ~25 ms. Only a fault that persists for 1 ms falls back to the quiescent point.
        auto bad = [&]
        {
            if (testFaults > 0) { --testFaults; return true; }
            return ! std::isfinite (g + k + p) || p < -50.0 || p > cfg.vb + 50.0;
        };
        if (bad())
        {
            ++statRetries;
            vin = lastVin;
            alpha = (Gs * (vin - h) + IeqA) * invGAtot;
            g = vG; k = vK; p = vP;
            newton (g, k, p, 30);
            if (bad())
            {
                ++statHeld;
                if (++heldRun > maxHeldRun)
                {
                    initDC();
                    return 0.0;
                }
                return vP - vPdc;
            }
        }
        heldRun = 0;
        lastVin = vin;
        prevG = vG; prevK = vK; prevP = vP;

        vG = g; vK = k; vP = p;
        vA = alpha + beta * g;

        // update reactive element states (trapezoidal rule)
        {
            const double i = Gs * (vin - vA - h);
            if (cfg.cc > 0.0)
            {
                const double vc = h + halfTc * i;
                h = vc + halfTc * i;
            }
        }
        if (GcA > 0.0) { const double i = GcA * vA - IeqA; IeqA = GcA * vA + i; }
        if (GcK > 0.0) { const double i = GcK * k - IeqK;  IeqK = GcK * k + i; }
        {
            const double vgp = g - p, vgk = g - k;
            const double iGP = Gcgp * vgp - IeqGP; IeqGP = Gcgp * vgp + iGP;
            const double iGK = Gcgk * vgk - IeqGK; IeqGK = Gcgk * vgk + iGK;
        }
        if (Gk2 > 0.0)
        {
            const double i = Gk2 * (k - h2);
            const double vc = h2 + halfTk2 * i;
            h2 = vc + halfTk2 * i;
        }

        return p - vPdc;
    }

    // Newton statistics (for tests)
    long long statSamples = 0, statEvals = 0, statFallbacks = 0, statFailures = 0, statRetries = 0, statHeld = 0;
    int testFaults = 0; // tests: the next this-many fault checks report a bad solve

    // Reference renders for the tone regression: 1000x tighter residual, no early step acceptance and the
    // original (untabulated) tube formula
    void setReferencePrecision (bool on) { resTol = on ? 2e-12 : 2e-9; acceptSmallStep = ! on; tube.exact = on; }

    // B+ of this stage per sample (the supply rail sags with load); the quiescent plate getVpDC() stays the
    // configure() value, so a rail change shows up as a slow offset in the AC output (blocked by the next coupling cap)
    void setSupply (double vb) { cfg.vb = vb; }
    double getPlateCurrent() const { return (cfg.vb - vP) / cfg.rp; } // drawn from the rail (through the plate resistor)

    // back to the quiescent point at the present B+ (channel switches; no allocation)
    void reset() { initDC(); }

    double getVk() const { return vK; }
    double getVp() const { return vP; }
    double getVpDC() const { return vPdc; }
    const TriodeConfig& getConfig() const { return cfg; }

private:
    // Gaussian elimination with partial pivoting; one division per pivot
    static bool solve3 (double (&m)[3][4], double (&x)[3])
    {
        double invPivot[3];
        for (int col = 0; col < 3; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < 3; ++r)
                if (std::abs (m[r][col]) > std::abs (m[piv][col])) piv = r;
            if (std::abs (m[piv][col]) < 1e-300) return false;
            if (piv != col)
                for (int j = 0; j < 4; ++j) std::swap (m[col][j], m[piv][j]);

            invPivot[col] = 1.0 / m[col][col];
            for (int r = col + 1; r < 3; ++r)
            {
                const double f = m[r][col] * invPivot[col];
                for (int j = col; j < 4; ++j) m[r][j] -= f * m[col][j];
            }
        }
        for (int r = 2; r >= 0; --r)
        {
            double s = m[r][3];
            for (int j = r + 1; j < 3; ++j) s -= m[r][j] * x[j];
            x[r] = s * invPivot[r];
        }
        return true;
    }

    void derive()
    {
        halfTc = cfg.cc > 0.0 ? T / (2.0 * cfg.cc) : 0.0;
        Gs = 1.0 / (std::max (cfg.rsrc, 1.0) + halfTc);
        Ga = cfg.ra > 0.0 ? 1.0 / cfg.ra : 0.0;
        GcA = cfg.caShunt > 0.0 ? 2.0 * cfg.caShunt / T : 0.0;
        Grs = 1.0 / std::max (cfg.rs, 1.0);
        Gb = cfg.rb > 0.0 ? 1.0 / cfg.rb : 0.0;
        Gp = 1.0 / cfg.rp;
        Gk = 1.0 / cfg.rk;
        GcK = cfg.ck > 0.0 ? 2.0 * cfg.ck / T : 0.0;
        if (cfg.ck2 > 0.0)
        {
            halfTk2 = T / (2.0 * cfg.ck2);
            Gk2 = 1.0 / (std::max (cfg.rk2, 1.0) + halfTk2);
        }
        else
        {
            halfTk2 = 0.0;
            Gk2 = 0.0;
        }
        Gfb = cfg.rfb > 0.0 ? 1.0 / cfg.rfb : 0.0;
        GAtot = Gs + Ga + GcA + Grs;
        invGAtot = 1.0 / GAtot;
        betaA = Grs * invGAtot;
        Gcgp = 2.0 * KorenTriode::cgp / T; // 12AX7 Cgp
        Gcgk = 2.0 * KorenTriode::cgk / T; // 12AX7 Cgk
    }

    // Quiescent point: grid at 0 V, caps open
    void initDC()
    {
        double k = 1.5, p = cfg.vb * 0.6;
        for (int it = 0; it < 200; ++it)
        {
            double ip, dpk, dgk;
            tube.eval (p - k, -k, ip, dpk, dgk);
            const double f1 = ip - k * Gk;
            const double f2 = (cfg.vb - p) * Gp - ip;
            const double a11 = -(dpk + dgk) - Gk, a12 = dpk;
            const double a21 = dpk + dgk,         a22 = -Gp - dpk;
            const double det = a11 * a22 - a12 * a21;
            if (std::abs (det) < 1e-300) break;
            const double dk = (-f1 * a22 + a12 * f2) / det;
            const double dp = (-a11 * f2 + a21 * f1) / det;
            k += std::clamp (dk, -1.0, 1.0);
            p += std::clamp (dp, -20.0, 20.0);
            if (std::abs (dk) + std::abs (dp) < 1e-9) break;
        }
        vK = k; vP = p; vPdc = p; vG = 0.0; vA = 0.0;
        prevG = vG; prevK = vK; prevP = vP;
        heldRun = 0;
        lastVin = 0.0;
        h = 0.0;
        IeqA = 0.0;
        IeqK = GcK * vK;
        h2 = vK;
        IeqGP = Gcgp * (0.0 - vP);
        IeqGK = Gcgk * (0.0 - vK);
    }

    // Newton step size (V) below which the solution is accepted. The remaining error is ~ step^2 * curvature:
    // tight on grid / cathode (the grid-current knee is 30 mV wide), loose on the plate (smooth, ~100 V scale).
    // Below kGridOff the grid current is exactly 0 and only the smooth Koren curve remains (volts scale).
    static constexpr double kStepTolGK = 1e-3, kStepTolGKOff = 1e-2, kStepTolP = 0.1, kGridOff = -0.6;
    double resTol = 2e-9;        // KCL residual (A), sum over the 3 nodes
    bool acceptSmallStep = true;

    KorenTriode tube;
    TriodeConfig cfg;
    double T = 1.0 / 192000.0;

    double Gs = 0, halfTc = 0, Ga = 0, GcA = 0, Grs = 0, Gb = 0, Gp = 0, Gk = 0, GcK = 0;
    double Gk2 = 0, halfTk2 = 0, Gfb = 0, GAtot = 1, invGAtot = 1, betaA = 0, Gcgp = 0, Gcgk = 0;

    double vG = 0, vK = 0, vP = 0, vA = 0, vPdc = 0;
    int heldRun = 0, maxHeldRun = 192; // consecutive dropped samples; beyond the limit -> initDC()
    double lastVin = 0.0;
    double prevG = 0, prevK = 0, prevP = 0;
    double h = 0, IeqA = 0, IeqK = 0, h2 = 0, IeqGP = 0, IeqGK = 0;
};

} // namespace vamp
