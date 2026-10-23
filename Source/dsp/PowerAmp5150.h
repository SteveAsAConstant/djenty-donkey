#pragma once
#include <algorithm>
#include <cmath>
#include "DKCircuit.h"
#include "TriodeStage.h"

namespace vamp
{
// Norman Koren's pentode model (6L6GC parameters from the wdv.com SPICE tube table)
struct KorenPentode
{
    static constexpr double mu = 8.7, ex = 1.35, kg1 = 1460.0, kg2 = 4500.0, kp = 48.0, kvb = 12.0;

    // plate current of ONE tube and its derivatives w.r.t. Vpk and Vg1k
    inline void plate (double vpk, double vg1k, double vg2k, double& ip, double& dVpk, double& dVg1) const
    {
        plateInv (vpk, vg1k, vg2k, 1.0 / vg2k, ip, dVpk, dVg1);
    }

    // same, with 1 / vg2k precomputed (the screen voltage is constant during a Newton solve)
    inline void plateInv (double vpk, double vg1k, double vg2k, double invVg2k, double& ip, double& dVpk, double& dVg1) const
    {
        constexpr double kpOverMu = kp / mu, invKvb = 1.0 / kvb, twoOverKg1 = 2.0 / kg1;
        const double x = kpOverMu + kp * vg1k * invVg2k;
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
        // atan() is kept smooth through Vpk = 0 (as in Koren's original); below 0 the flyback
        // diodes dominate, and the smooth knee keeps Newton's method well behaved.

        const double Ep = std::exp ((ex - 1.0) * std::log (E1)); // == pow (E1, ex - 1), faster
        const double base = twoOverKg1 * Ep * E1;
        const double r = vpk * invKvb;
        const double at = std::atan (r);
        ip = base * at;
        dVg1 = twoOverKg1 * ex * Ep * sig * at;
        dVpk = base * invKvb / (1.0 + r * r);
    }

    // screen current of ONE tube
    inline double screen (double vg1k, double vg2k) const
    {
        const double b = vg2k / mu + vg1k;
        return b > 0.0 ? std::pow (b, ex) / kg2 : 0.0;
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
//   NFB: speaker -> R56 39k -> (C12 6.8n || VR8 Resonance) -> tail node L,
//        presence: L -> VR9 10k -> C8 0.1u -> gnd.   V4A grid AC-referenced to L via C31.
//   Supply: B+ reservoir with sag, screen node via R210 400R.
//
// The whole PI + output stage + transformer + speaker + NFB loop is solved simultaneously
// with the DK method (8 nonlinear ports), so the feedback loop has no artificial delay.
class PowerAmp5150
{
public:
    // ---- assumptions (not printed on the schematic) ----
    static constexpr double kV0 = 500.0;        // unloaded B+ (forums: ~480-500 V at the plates)
    static constexpr double kRsupply = 60.0;    // rectifier + transformer winding resistance
    static constexpr double kCreservoir = 100e-6;
    static constexpr double kCscreen = 47e-6;
    static constexpr double kRscreen = 400.0;   // R210
    static constexpr double kV3Ratio = 0.86;    // PI supply relative to B+
    static constexpr double kIdlePerTube = 0.035;
    static constexpr double kRaa = 2100.0;      // plate-to-plate impedance
    static constexpr double kRload = 8.0;
    static constexpr double kLmagPrimary = 30.0; // OT magnetizing inductance (H)
    static constexpr double kCwinding = 2.5e-9;  // OT primary shunt capacitance (F)

    PowerAmp5150()
    {
        auto n = [this] { return dk.addNode(); };
        using D = DKCircuit;
        const int G = D::GND;

        nA = n(); nP1 = n(); nGB = n(); nGA = n(); nK = n(); nQ = n(); nL = n(); nM = n(); nPr = n();
        nPB = n(); nPA = n(); nC1 = n(); nG1 = n(); nC2 = n(); nG2 = n();
        nO1 = n(); nO2 = n(); nCT = n(); nS = n(); nSp1 = n(); nSp2 = n();

        srcIn = dk.addSource (nA, 58e3);   // V3A plate output impedance (68k || rp')
        dk.addC (nA, nP1, 22e-9);          // C49
        dk.addR (nP1, nQ, 1e6);            // R50
        dk.addR (nP1, nGB, 100e3);         // R48
        dk.addR (nGA, nQ, 1e6);            // R53
        dk.addC (nGA, nL, 47e-9);          // C31
        dk.addR (nK, nQ, 470.0);           // R52
        dk.addR (nQ, nL, 10e3);            // R51
        dk.addR (nL, G, 4.7e3);            // R57

        srcV3B = dk.addSource (nPB, 82e3);  // R46
        srcV3A = dk.addSource (nPA, 100e3); // R58

        dk.addR (nS, nM, 39e3);            // R56
        dk.addC (nM, nL, 6.8e-9);          // C12
        rRes = dk.addR (nM, nL, 1e6);      // VR8 resonance
        rPres = dk.addR (nL, nPr, 10e3);   // VR9 presence
        dk.addC (nPr, G, 0.1e-6);          // C8

        dk.addC (nPB, nC1, 47e-9);         // C29
        srcBias1 = dk.addSource (nC1, 220e3); // R55
        dk.addR (nC1, nG1, 2.2e3);         // R206 / R208
        dk.addC (nPA, nC2, 47e-9);         // C28
        srcBias2 = dk.addSource (nC2, 220e3); // R54
        dk.addR (nC2, nG2, 2.2e3);         // R202 / R204

        const double np = std::sqrt (kRaa / kRload);
        srcB = dk.addSource (nCT, 1.0);
        // primary shunt capacitance (in parallel with the reflected load, so it is damped):
        // gives the open-loop high-frequency roll-off of the output transformer
        dk.addC (nO1, nO2, kCwinding);
        dk.addTransformer ({ { nO1, nCT, np / 2 }, { nCT, nO2, np / 2 }, { nS, G, 1.0 } });
        dk.addL (nS, G, kLmagPrimary / (np * np));

        // 4x12 closed-back cab impedance: Re, lossy voice-coil inductance, ~100 Hz resonance
        dk.addR (nS, nSp1, 6.0);
        dk.addL (nSp1, nSp2, 0.5e-3);
        dk.addR (nSp1, nSp2, 30.0);
        dk.addR (nSp2, G, 39.0);
        dk.addL (nSp2, G, 20.7e-3);
        dk.addC (nSp2, G, 122e-6);

        dk.markOutput (nS);  // speaker
        dk.markOutput (nK);  // diagnostics
        dk.markOutput (nO1);
        dk.markOutput (nPA);
        dk.markOutput (nPB);

        // nonlinear ports
        dk.addPort (nPB, nK); dk.addPort (nGB, nK);   // V4B
        dk.addPort (nPA, nK); dk.addPort (nGA, nK);   // V4A
        dk.addPort (nO1, G);  dk.addPort (nG1, G);    // 6L6 pair 1
        dk.addPort (nO2, G);  dk.addPort (nG2, G);    // 6L6 pair 2
    }

    void prepare (double sampleRate)
    {
        fs = sampleRate;
        T = 1.0 / fs;
        dk.prepare (fs);

        TriodeConfig c; // V3A recovery stage: C11 47n, R26 1M, R5 68k, R14 3.3k || C61 4.7n
        c.vb = 320.0; c.rsrc = 100e3; c.cc = 47e-9; c.ra = 1e6; c.rs = 1e3;
        c.rp = 68e3; c.rk = 3.3e3; c.ck = 4.7e-9;
        v3a.configure (c, fs);

        // supply / bias at idle
        vB = kV0 - kRsupply * 0.15;
        vScreen = vB - 5.0;
        vBias = findBias (vB, vScreen);
        std::fill (std::begin (p), std::end (p), 0.0);
        std::fill (std::begin (pPrev), std::end (pPrev), 0.0);

        for (int i = 0; i < (int) (fs * 0.6); ++i)
            process (0.0);
    }

    // 0..1 knob positions
    void setControls (double presence, double resonance)
    {
        dk.setR (rRes, (std::pow (10.0, 2.0 * std::clamp (resonance, 0.0, 1.0)) - 1.0) / 99.0 * 1e6 + 100.0);
        dk.setR (rPres, (1.0 - std::clamp (presence, 0.0, 1.0)) * 10e3 + 10.0);
    }

    // input: volts at the master (post-gain) pot wiper. output: volts across the speaker
    double process (double vin)
    {
        const double v3 = v3a.process (vin);

        double u[8] {};
        u[srcIn] = v3;
        u[srcV3B] = u[srcV3A] = kV3Ratio * vB;
        u[srcBias1] = u[srcBias2] = vBias;
        u[srcB] = vB;

        const auto& p0 = dk.computeP0 (u);
        const auto& K = dk.getK();

        // predictor: linear extrapolation of the last two solutions, switched off while it does
        // not pay off (e.g. noisy input), in which case Newton starts from the previous solution
        for (int k = 0; k < 8; ++k)
        {
            const double cur = p[k];
            if (usePredictor) p[k] += cur - pPrev[k];
            pPrev[k] = cur;
        }

        double i[8], d[8][2];
        int iterations = 0;
        auto newton = [&] (int maxIt)
        {
          for (int it = 0; it < maxIt; ++it)
          {
            ++iterations;
            evalDevices (i, d);

            double J[8][9], maxF = 0.0;
            for (int r = 0; r < 8; ++r)
            {
                double Ki = 0.0;
                for (int k = 0; k < 8; ++k) Ki += K[(size_t) r * 8 + k] * i[k];
                const double F = p[r] - p0[(size_t) r] - Ki;
                maxF = std::max (maxF, std::abs (F) * (r % 2 == 0 ? 1.0 : 10.0)); // grids 10x stricter
                J[r][8] = -F;
            }
            // converged: plates < 0.1 mV, grids < 0.01 mV. The final Newton step is still applied (an 8x8
            // solve, no tube evaluation): the error drops from ~residual to ~residual^2. v0.3.0 returned
            // before it, which biased the small-signal response by up to 0.3 dB at low frequencies.
            const bool converged = maxF < resTol;

            for (int r = 0; r < 8; ++r)
            {
                for (int dv = 0; dv < 4; ++dv)
                {
                    const int c0 = 2 * dv, c1 = c0 + 1;
                    // port c0 = plate (current depends on c0, c1), port c1 = grid (depends on c1 only)
                    J[r][c0] = (r == c0 ? 1.0 : 0.0) - K[(size_t) r * 8 + c0] * d[c0][0];
                    J[r][c1] = (r == c1 ? 1.0 : 0.0) - K[(size_t) r * 8 + c0] * d[c0][1] - K[(size_t) r * 8 + c1] * d[c1][1];
                }
            }

            double dp[8];
            if (! solve (J, dp)) return converged;

            // Step limit, the same for plates and grids. Grid current and the flyback diodes are softplus
            // knees (linear beyond the knee), so an overshoot corrects itself on the next step. A tighter
            // grid limit (4 V in v0.3.0) left steep preamp edges, which swing the 6L6 grids by >100 V
            // in one sample through C28/C29, unconverged after 46 iterations -> clicks at high master.
            double maxStep = 0.0;
            for (int k = 0; k < 8; ++k)
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
                for (int dv = 0; dv < 4; ++dv)
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
        if (! converged) statFailures++;
        lastConverged = converged;
        for (int k = 0; k < 8; ++k) lastP0[k] = p0[(size_t) k];

        if (! converged)
            evalDevices (i, d);
        if (! std::isfinite (i[0] + i[4] + i[6]))
        {
            prepareStateAfterFailure();
            return 0.0;
        }
        dk.finish (i);

        // ---- power supply sag ----
        const double iPlates = i[4] + i[6];
        const double iScreens = 2.0 * (tube.screen (p[5], vScreen) + tube.screen (p[7], vScreen));
        const double iScreenFeed = (vB - vScreen) / kRscreen;
        const double iCharge = std::max (0.0, (kV0 - vB) / kRsupply);
        vB += T / kCreservoir * (iCharge - iPlates - iScreenFeed);
        vScreen += T / kCscreen * (iScreenFeed - iScreens);

        return dk.nodeV (nS);
    }

    // Newton statistics (for tests)
    long long statSamples = 0, statIterations = 0, statFailures = 0;
    bool lastConverged = true;

    // Reference renders for the tone regression: 1000x tighter residual and no early step acceptance
    void setReferencePrecision (bool on)
    {
        resTol = on ? 1e-7 : 1e-4;
        acceptSmallStep = ! on;
        v3a.setReferencePrecision (on);
    }
    double lastP0[8] {};
    double getPort (int k) const { return p[k]; }
    double getV3a() const { return v3a.getVp(); }

    double getSupplyVoltage() const { return vB; }
    double getBiasVoltage() const { return vBias; }
    double getPlate1() const { return dk.nodeV (nO1); }
    double getPIcathode() const { return dk.nodeV (nK); }
    double getPIplateA() const { return dk.nodeV (nPA); }
    double getPIplateB() const { return dk.nodeV (nPB); }

private:
    void evalDevices (double (&i)[8], double (&d)[8][2])
    {
        const double invScreen = 1.0 / vScreen;
        for (int t = 0; t < 2; ++t) // V4B, V4A
        {
            const int pk = 2 * t, gk = pk + 1;
            double ip, dvpk, dvgk, ig, dig;
            triode.eval (p[pk], p[gk], ip, dvpk, dvgk);
            gridCurrent (p[gk], ig, dig);
            i[pk] = ip; d[pk][0] = dvpk; d[pk][1] = dvgk;
            i[gk] = ig; d[gk][0] = 0.0;  d[gk][1] = dig;
        }
        for (int t = 0; t < 2; ++t) // 6L6 pairs (2 tubes each)
        {
            const int pk = 4 + 2 * t, gk = pk + 1;
            double ip, dvpk, dvg1, ig, dig;
            tube.plateInv (p[pk], p[gk], vScreen, invScreen, ip, dvpk, dvg1);
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

    static bool solve (double (&m)[8][9], double (&x)[8])
    {
        for (int col = 0; col < 8; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < 8; ++r)
                if (std::abs (m[r][col]) > std::abs (m[piv][col])) piv = r;
            if (std::abs (m[piv][col]) < 1e-300) return false;
            if (piv != col)
                for (int j = 0; j < 9; ++j) std::swap (m[col][j], m[piv][j]);
            for (int r = col + 1; r < 8; ++r)
            {
                const double f = m[r][col] / m[col][col];
                if (f == 0.0) continue;
                for (int j = col; j < 9; ++j) m[r][j] -= f * m[col][j];
            }
        }
        for (int r = 7; r >= 0; --r)
        {
            double s = m[r][8];
            for (int j = r + 1; j < 8; ++j) s -= m[r][j] * x[j];
            x[r] = s / m[r][r];
        }
        return true;
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

    void prepareStateAfterFailure()
    {
        dk.reset();
        std::fill (std::begin (p), std::end (p), 0.0);
        std::fill (std::begin (pPrev), std::end (pPrev), 0.0);
    }

    // Newton step (V) below which a solve is accepted without a confirming evaluation: plates 30 mV,
    // grids 6 mV (weighted 5x: the grid-current knee is only 30 mV wide). Tuned with tone_regress:
    // -125 dB or better against the reference solution.
    static constexpr double kPlateStepTol = 3e-2, kGridStepWeight = 5.0;
    double resTol = 1e-4;          // port-equation residual (V, grids weighted 10x)
    bool acceptSmallStep = true;

    DKCircuit dk;
    TriodeStage v3a;
    KorenTriode triode;
    KorenPentode tube;

    int nA = 0, nP1 = 0, nGB = 0, nGA = 0, nK = 0, nQ = 0, nL = 0, nM = 0, nPr = 0;
    int nPB = 0, nPA = 0, nC1 = 0, nG1 = 0, nC2 = 0, nG2 = 0, nO1 = 0, nO2 = 0;
    int nCT = 0, nS = 0, nSp1 = 0, nSp2 = 0;
    int srcIn = 0, srcV3B = 0, srcV3A = 0, srcBias1 = 0, srcBias2 = 0, srcB = 0;
    int rRes = 0, rPres = 0;

    double fs = 192000.0, T = 1.0 / 192000.0;
    double vB = 490.0, vScreen = 485.0, vBias = -45.0;
    double p[8] {}, pPrev[8] {};
    bool usePredictor = true;
    double predictorScore = 1.0;
    unsigned predictorRetry = 0;
};

} // namespace vamp
