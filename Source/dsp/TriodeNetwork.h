#pragma once
#include <algorithm>
#include <cmath>
#include "DKCircuit.h"
#include "PortSolve.h"
#include "TriodeStage.h"
#include "TubeNoise.h"

namespace vamp
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
            t.cgk = dk.addC (t.g, t.k, KorenTriode::cgk); // 12AX7 Cgk
            t.cgp = dk.addC (t.g, t.p, KorenTriode::cgp); // 12AX7 Cgp
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
            // shot / flicker noise of this tube from its transconductance at the operating point
            double ip, dvpk, gm;
            triode[t].eval (vp[t] - vk[t], -vk[t], ip, dvpk, gm);
            tubeNoise[t].prepare (sampleRate, gm, noiseSeed + 7919ull * (unsigned long long) (t + 1));
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
        if (noiseOn) for (int t = 0; t < NT; ++t) vn[t] = tubeNoise[t].next();
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

            double rhs[M], maxF = 0.0; // -F
            for (int r = 0; r < M; ++r)
            {
                double Ki = 0.0;
                for (int k = 0; k < M; ++k) Ki += K[(size_t) r * M + k] * i[k];
                const double F = p[r] - p0[(size_t) r] - Ki;
                maxF = std::max (maxF, std::abs (F) * (r % 2 == 0 ? 1.0 : 10.0)); // grids 10x stricter
                rhs[r] = -F;
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

            // (I - K D) dp = -F over the conducting ports only (plate current depends on both ports, grid current
            // on the grid's: PortSolve.h)
            double dp[M];
            if (! solvePortStep<M> (K.data(), d, rhs, dp)) return converged;

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
        if (testCorrupt > 0) { --testCorrupt; converged = false; for (int k = 0; k < M; ++k) p[k] += 300.0; }
        if (! converged) { statFailures++; evalDevices (i, d); }

        // An unconverged solve is kept only when it is nearly a solution (port residual <= kAcceptResidual) and
        // physically plausible; otherwise, like a non-finite one, it is retried as a lost input sample (the last
        // accepted sources); if that fails too the sample is dropped (states held); a fault lasting 1 ms goes back to
        // the idle point. As PowerAmp5150.
        auto bad = [&] (bool conv)
        {
            if (testFaults > 0) { --testFaults; return true; }
            double sum = 0.0;
            for (int k = 0; k < M; ++k) sum += i[k] + p[k];
            if (! std::isfinite (sum)) return true;
            return ! conv && ! (residual (p0, K, i) <= kAcceptResidual && plausible());
        };
        if (bad (converged))
        {
            ++statRetries;
            std::copy (lastU, lastU + nU, u);
            dk.computeP0 (u); // p0 refers to the same buffer
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
        for (auto& tr : triode) tr.exact = on;
    }

    // plate current of triode t at the last accepted sample: what it draws from its supply rail
    double plateCurrent (int t) const { return plateI[t]; }

    // Triode t takes over the state of triode u of another network (the same tube in another relay position):
    // its port voltages (and the last ones, for the predictor), plate current and Miller capacitor states
    template <int N2>
    void copyTube (int t, const TriodeNetwork<N2>& o, int u)
    {
        for (int k = 0; k < 2; ++k)
        {
            p[2 * t + k] = o.port (2 * u + k);
            pPrev[2 * t + k] = o.portPrev (2 * u + k);
        }
        plateI[t] = o.plateCurrent (u);
        dk.setCapState (tube[t].cgk, o.dk.getCapState (o.tube[u].cgk));
        dk.setCapState (tube[t].cgp, o.dk.getCapState (o.tube[u].cgp));
    }
    double port (int k) const { return p[k]; }
    double portPrev (int k) const { return pPrev[k]; }

    // back to the idle point saved by saveIdle() (no allocation: fine on the audio thread)
    void resetToIdle()
    {
        restoreIdle();
        for (int t = 0; t < NT; ++t) plateI[t] = 0.0;
    }

    // Tube t's deviation from the nominal 12AX7 (TubeSet): plate current x currentScale, mu x muScale. Takes effect
    // at once (as swapping a tube; the stage re-biases through its caps).
    void setTubeVariation (int t, double currentScale, double muScale) { triode[t].setVariation (currentScale, muScale); }

    // Noise (thermal noise of the circuit's noisy resistors, shot / flicker noise of the tubes) on / off. The seed
    // takes effect at the next initDC (tubes) / now (resistors).
    void setNoise (bool on, unsigned long long seed)
    {
        noiseOn = on;
        noiseSeed = seed;
        dk.setNoise (on, seed);
        if (! on) for (double& v : vn) v = 0.0;
    }
    bool getNoise() const { return noiseOn; }

    long long statSamples = 0, statIterations = 0, statFailures = 0, statRetries = 0, statHeld = 0;
    int testFaults = 0;  // tests: the next this-many fault checks report a bad solve
    int testCorrupt = 0; // tests: the next this-many solves end unconverged, 300 V off (must not be accepted)

private:
    // largest port-equation residual (V, grids weighted 10x) of the state p with device currents i
    template <typename P0, typename KM>
    double residual (const P0& p0, const KM& K, const double (&i)[M]) const
    {
        double maxF = 0.0;
        for (int r = 0; r < M; ++r)
        {
            double Ki = 0.0;
            for (int k = 0; k < M; ++k) Ki += K[(size_t) r * M + k] * i[k];
            maxF = std::max (maxF, std::abs (p[r] - p0[(size_t) r] - Ki) * (r % 2 == 0 ? 1.0 : 10.0));
        }
        return maxF;
    }

    // port voltages a real 12AX7 stage can have (plate-cathode, grid-cathode), with wide margins
    bool plausible() const
    {
        for (int t = 0; t < NT; ++t)
            if (! (p[2 * t] > -500.0 && p[2 * t] < 1500.0 && p[2 * t + 1] > -1000.0 && p[2 * t + 1] < 100.0)) return false;
        return true;
    }

    void evalDevices (double (&i)[M], double (&d)[M][2]) const
    {
        for (int t = 0; t < NT; ++t)
        {
            const int pk = 2 * t, gk = pk + 1;
            double ip, dvpk, dvgk, ig, dig;
            triode[t].eval (p[pk], p[gk] + vn[t], ip, dvpk, dvgk); // the tube's noise acts like a grid voltage
            gridCurrent (p[gk], ig, dig);
            i[pk] = ip; d[pk][0] = dvpk; d[pk][1] = dvgk;
            i[gk] = ig; d[gk][0] = 0.0;  d[gk][1] = dig;
        }
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
    // an unconverged solve is kept up to this residual (V): 10 mV, ~500x the tolerance, still inaudible
    static constexpr double kAcceptResidual = 1e-2;
    double resTol = 2e-5;
    bool acceptSmallStep = true;

    KorenTriode triode[NT]; // one per tube: their variations (setTubeVariation) differ
    TubeNoise tubeNoise[NT];
    double vn[NT] {};           // this sample's tube noise (V at the grid)
    bool noiseOn = false;
    unsigned long long noiseSeed = 0x2545F4914F6CDD1Dull;
    double p[M] {}, pPrev[M] {}, pIdle[M] {}, plateI[NT] {};
    double lastU[kMaxInputs] {}, idleU[kMaxInputs] {};
    int nU = 0;
    int heldRun = 0, maxHeldRun = 352;
    bool usePredictor = true;
    double predictorScore = 1.0;
    unsigned predictorRetry = 0;
};

} // namespace vamp
