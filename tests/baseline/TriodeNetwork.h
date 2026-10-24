#pragma once
#include <algorithm>
#include <cmath>
#include "DKCircuit.h"
#include "TriodeStage.h"

namespace vamp_baseline
{
// NT 12AX7 triodes and the circuit around them, solved simultaneously with the DK method: every stage loads
// the ones next to it (grid current, feedback networks), unlike a cascade of TriodeStages fed through Thevenin
// resistances. 2 NT nonlinear ports (per triode: plate-cathode and grid-cathode), Koren plate current + the
// TriodeStage grid current, Cgk / Cgp as real capacitors. Newton with a predictor, fault handling and statistics
// as in PowerAmp5150.
//
// The owner builds the circuit in dk: tubeNodes() for each triode's grid / cathode / plate, its own elements,
// then addMillerAndOutputs() and addPorts(). After dk.prepare() it sets the DC operating point (initDC() plus its
// own capacitors via dk.setCapVoltage()) and calls saveIdle(). Per sample: solve (u) with the source values.
template <int NT>
class TriodeNetwork
{
public:
    static constexpr int M = 2 * NT, kMaxInputs = 16;

    struct Tube { int g = 0, k = 0, p = 0, cgk = 0, cgp = 0; };

    DKCircuit dk;
    Tube tube[NT];

    void tubeNodes (int t) { tube[t].g = dk.addNode(); tube[t].k = dk.addNode(); tube[t].p = dk.addNode(); }

    // tubeOutputs: also compute each cathode and plate voltage every sample (diagnostics, tests)
    void addMillerAndOutputs (bool tubeOutputs = true)
    {
        for (auto& t : tube)
        {
            t.cgk = dk.addC (t.g, t.k, 2.3e-12); // 12AX7 Cgk
            t.cgp = dk.addC (t.g, t.p, 2.4e-12); // 12AX7 Cgp
            if (! tubeOutputs) continue;
            dk.markOutput (t.k);
            dk.markOutput (t.p);
        }
    }

    void addPorts()
    {
        for (const auto& t : tube)
        {
            dk.addPort (t.p, t.k); // port 2t: plate current
            dk.addPort (t.g, t.k); // port 2t+1: grid current
        }
    }

    // DC operating point with the grids at 0 V: port guesses and the Miller capacitances
    void initDC (const double (&vk)[NT], const double (&vp)[NT], double sampleRate)
    {
        for (int t = 0; t < NT; ++t)
        {
            dk.setCapVoltage (tube[t].cgk, -vk[t]);
            dk.setCapVoltage (tube[t].cgp, -vp[t]);
            p[2 * t] = vp[t] - vk[t];
            p[2 * t + 1] = -vk[t];
        }
        maxHeldRun = std::max (1, (int) (1e-3 * sampleRate));
    }

    // the settled / initial state a persistent fault goes back to (allocates: call from prepare)
    void saveIdle (int numInputs, const double* u)
    {
        std::copy (std::begin (p), std::end (p), std::begin (pPrev));
        std::copy (std::begin (p), std::end (p), std::begin (pIdle));
        dk.saveState();
        nU = numInputs;
        std::copy (u, u + nU, lastU);
        std::copy (u, u + nU, idleU);
        heldRun = 0;
    }

    // One sample with source values u. False when the sample had to be dropped (the states still hold the last
    // accepted sample: the caller repeats its output).
    bool solve (double* u)
    {
        const auto& p0 = dk.computeP0 (u); // updated in place by later computeP0 calls
        const auto& K = dk.getK();

        // predictor: linear extrapolation of the last two solutions while it pays off
        for (int k = 0; k < M; ++k)
        {
            const double cur = p[k];
            if (usePredictor) p[k] += cur - pPrev[k];
            pPrev[k] = cur;
        }

        double i[M], d[M][2];
        int iterations = 0;
        // damped: backtracking (a step that makes the residual worse is halved until it does not), for the last
        // fallback only. A grid driven far positive makes the plate current huge for Vpk > 0 and zero (with zero
        // slope) below, and plain Newton can cycle across Vpk = 0 forever (seen with a 1 kV spike at the jack).
        auto newton = [&] (int maxIt, bool damped = false)
        {
          double pOld[M], lastMaxF = 1e300;
          int halvings = 0;
          for (int it = 0; it < maxIt; ++it)
          {
            ++iterations;
            evalDevices (i, d);

            double J[M][M + 1], maxF = 0.0;
            for (int r = 0; r < M; ++r)
            {
                double Ki = 0.0;
                for (int k = 0; k < M; ++k) Ki += K[(size_t) r * M + k] * i[k];
                const double F = p[r] - p0[(size_t) r] - Ki;
                maxF = std::max (maxF, std::abs (F) * (r % 2 == 0 ? 1.0 : 10.0)); // grids 10x stricter
                J[r][M] = -F;
            }
            if (damped)
            {
                if (maxF > lastMaxF && halvings < 40)
                {
                    for (int k = 0; k < M; ++k) p[k] = 0.5 * (pOld[k] + p[k]);
                    ++halvings;
                    continue;
                }
                halvings = 0;
                lastMaxF = maxF;
                std::copy (std::begin (p), std::end (p), pOld);
            }
            const bool converged = maxF < resTol; // the final step is still applied (error ~ residual^2)

            for (int r = 0; r < M; ++r)
                for (int t = 0; t < NT; ++t)
                {
                    const int c0 = 2 * t, c1 = c0 + 1; // plate current depends on both ports, grid current on c1
                    J[r][c0] = (r == c0 ? 1.0 : 0.0) - K[(size_t) r * M + c0] * d[c0][0];
                    J[r][c1] = (r == c1 ? 1.0 : 0.0) - K[(size_t) r * M + c0] * d[c0][1] - K[(size_t) r * M + c1] * d[c1][1];
                }

            double dp[M];
            if (! solveLinear (J, dp)) return converged;

            double maxStep = 0.0;
            for (int k = 0; k < M; ++k)
            {
                const double lim = k % 2 == 0 ? 50.0 : 10.0; // grid current is a softplus knee: overshoot self-corrects
                dp[k] = std::clamp (dp[k], -lim, lim);
                p[k] += dp[k];
                maxStep = std::max (maxStep, std::abs (dp[k]) * (k % 2 == 0 ? 1.0 : kGridStepWeight));
            }
            // quadratic convergence: after a step this small the remaining error is ~step^2, so accept without
            // re-evaluating the tubes; the currents move along their linearisation to stay consistent with p
            if (converged || (acceptSmallStep && maxStep < kPlateStepTol))
            {
                for (int t = 0; t < NT; ++t)
                {
                    const int c0 = 2 * t, c1 = c0 + 1;
                    i[c0] += d[c0][0] * dp[c0] + d[c0][1] * dp[c1];
                    i[c1] += d[c1][1] * dp[c1];
                }
                return true;
            }
          }
          return false;
        };

        bool converged = newton (6);
        if (! converged)
        {
            std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
            converged = newton (40);
        }
        // absurd inputs (a 1 kV spike at the jack) can make plain Newton cycle: continue from where the fallback
        // stopped with backtracking (never reached on normal signals, so they are unaffected)
        if (! converged)
            converged = newton (200, true);
        predictorScore = 0.9 * predictorScore + 0.1 * (iterations <= 2 ? 1.0 : 0.0);
        if (usePredictor && predictorScore < 0.3) usePredictor = false;
        else if (! usePredictor && (++predictorRetry & 63) == 0) { usePredictor = true; predictorScore = 0.5; }

        statSamples++;
        statIterations += iterations;
        if (! converged) { statFailures++; evalDevices (i, d); }

        // A non-finite solve is retried as a lost input sample (the last accepted sources); if that fails too the
        // sample is dropped (states held); a fault lasting 1 ms goes back to the idle point. As PowerAmp5150.
        auto bad = [&]
        {
            if (testFaults > 0) { --testFaults; return true; }
            double sum = 0.0;
            for (int k = 0; k < M; ++k) sum += i[k] + p[k];
            return ! std::isfinite (sum);
        };
        if (bad())
        {
            ++statRetries;
            std::copy (lastU, lastU + nU, u);
            dk.computeP0 (u);
            std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
            if (! newton (40))
                evalDevices (i, d);
            if (bad())
            {
                ++statHeld;
                std::copy (std::begin (pPrev), std::end (pPrev), std::begin (p));
                if (++heldRun > maxHeldRun)
                    restoreIdle();
                return false;
            }
        }
        heldRun = 0;
        std::copy (u, u + nU, lastU);
        dk.finish (i);
        for (int t = 0; t < NT; ++t) plateI[t] = i[2 * t];
        return true;
    }

    // tone-regression reference renders: 1000x tighter residual, no early step acceptance, untabulated Koren
    void setReferencePrecision (bool on)
    {
        resTol = on ? 2e-8 : 2e-5;
        acceptSmallStep = ! on;
        triode.exact = on;
    }

    // plate current of triode t at the last accepted sample: what it draws from its supply rail
    double plateCurrent (int t) const { return plateI[t]; }

    // back to the idle point saved by saveIdle() (no allocation: fine on the audio thread)
    void resetToIdle()
    {
        restoreIdle();
        for (int t = 0; t < NT; ++t) plateI[t] = 0.0;
    }

    long long statSamples = 0, statIterations = 0, statFailures = 0, statRetries = 0, statHeld = 0;
    int testFaults = 0; // tests: the next this-many fault checks report a bad solve

private:
    void evalDevices (double (&i)[M], double (&d)[M][2]) const
    {
        for (int t = 0; t < NT; ++t)
        {
            const int pk = 2 * t, gk = pk + 1;
            double ip, dvpk, dvgk, ig, dig;
            triode.eval (p[pk], p[gk], ip, dvpk, dvgk);
            gridCurrent (p[gk], ig, dig);
            i[pk] = ip; d[pk][0] = dvpk; d[pk][1] = dvgk;
            i[gk] = ig; d[gk][0] = 0.0;  d[gk][1] = dig;
        }
    }

    // Gaussian elimination with partial pivoting
    static bool solveLinear (double (&m)[M][M + 1], double (&x)[M])
    {
        double invPivot[M];
        for (int col = 0; col < M; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < M; ++r)
                if (std::abs (m[r][col]) > std::abs (m[piv][col])) piv = r;
            if (std::abs (m[piv][col]) < 1e-300) return false;
            if (piv != col)
                for (int j = 0; j <= M; ++j) std::swap (m[col][j], m[piv][j]);
            invPivot[col] = 1.0 / m[col][col];
            for (int r = col + 1; r < M; ++r)
            {
                const double f = m[r][col] * invPivot[col];
                if (f == 0.0) continue;
                for (int j = col; j <= M; ++j) m[r][j] -= f * m[col][j];
            }
        }
        for (int r = M - 1; r >= 0; --r)
        {
            double s = m[r][M];
            for (int j = r + 1; j < M; ++j) s -= m[r][j] * x[j];
            x[r] = s * invPivot[r];
        }
        return true;
    }

    void restoreIdle()
    {
        dk.restoreState();
        std::copy (std::begin (pIdle), std::end (pIdle), std::begin (p));
        std::copy (std::begin (pIdle), std::end (pIdle), std::begin (pPrev));
        std::copy (idleU, idleU + nU, lastU);
        heldRun = 0;
    }

    // Newton step (V) accepted without a confirming evaluation: plates 30 mV, grids 3 mV (weighted 10x: the
    // grid-current knee is 30 mV wide). Residual tolerance in volts (grids weighted 10x).
    static constexpr double kPlateStepTol = 3e-2, kGridStepWeight = 10.0;
    double resTol = 2e-5;
    bool acceptSmallStep = true;

    KorenTriode triode;
    double p[M] {}, pPrev[M] {}, pIdle[M] {}, plateI[NT] {};
    double lastU[kMaxInputs] {}, idleU[kMaxInputs] {};
    int nU = 0;
    int heldRun = 0, maxHeldRun = 352;
    bool usePredictor = true;
    double predictorScore = 1.0;
    unsigned predictorRetry = 0;
};

} // namespace vamp_baseline
