#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include "DKCircuit.h"

namespace vamp
{
// A DK circuit with semiconductor devices (BJTs, JFETs, diodes) as its nonlinear ports, solved per sample with
// Newton (SPICE-style junction voltage limiting, a predictor, fallbacks), as TriodeNetwork does for tubes.
// The device equations are LTspice's, restricted to the parameters the 1176 reference netlist uses
// (spice/1176ln_revd.inc), so the C++ and LTspice solve the same equations:
//   BJT (Gummel-Poon with Is, Bf, Br, Vaf only):  If = Is (e^(Vbe/Vt) - 1), Ir = Is (e^(Vbc/Vt) - 1),
//       Ic = (If - Ir)(1 - Vbc/Vaf) - Ir/Br,  Ib = If/Bf + Ir/Br
//   JFET (N channel): Id = beta (f(Vgs - Vto) - f(Vgd - Vto)) (1 + lambda |Vds|), f(x) = (2 n Vt ln(1 + e^(x/2nVt)))^2
//       (Shichman-Hodges written symmetrically, the pinch-off edge smoothed into subthreshold conduction);
//       gate-source and gate-drain junctions as diodes (Is, N = 1)
//   diode: I = Is (e^(V/(N Vt)) - 1)
// Junction capacitances are ordinary capacitors in the circuit.
//
// The owner builds the circuit with a builder function (called twice: the running circuit and a DC instance of
// it), then prepare(). Per sample: solve(u) with the source values, then read nodes / port currents.
class SemiNetwork
{
public:
    static constexpr int kMaxPorts = 12, kMaxInputs = 16;
    static constexpr double kVt = 0.025865; // kT/q at 27 C (LTspice's default temperature)

    struct BjtModel { double is, bf, br, vaf; };
    struct JfetModel { double beta, vto, lambda, n, isGate; };
    struct DiodeModel { double is, n; };

    using Builder = std::function<void (SemiNetwork&)>;

    DKCircuit dk;

    // B, C, E nodes. Ports: (B, E) carries Ib + Ic, (B, C) carries -Ic.
    void addBjt (int b, int c, int e, const BjtModel& mdl)
    {
        Dev d; d.type = Dev::kBjt; d.bjt = mdl;
        d.p0 = dk.addPort (b, e);
        d.p1 = dk.addPort (b, c);
        d.vcrit = critical (kVt, mdl.is);
        devs[numDevs++] = d;
    }

    // G, D, S nodes. Ports: (G, S) carries the gate current Igs + Igd, (D, S) carries Id - Igd.
    void addJfet (int g, int d, int s, const JfetModel& mdl)
    {
        Dev v; v.type = Dev::kJfet; v.jfet = mdl;
        v.p0 = dk.addPort (g, s);
        v.p1 = dk.addPort (d, s);
        v.vcrit = critical (kVt, mdl.isGate);
        devs[numDevs++] = v;
    }

    // anode, cathode
    void addDiode (int a, int k, const DiodeModel& mdl)
    {
        Dev d; d.type = Dev::kDiode; d.diode = mdl;
        d.p0 = dk.addPort (a, k);
        d.p1 = -1;
        d.vcrit = critical (mdl.n * kVt, mdl.is);
        devs[numDevs++] = d;
    }

    // Build the circuit (and its DC twin) and prepare both. Nodes the owner wants per sample: markOutput in build.
    void prepare (const Builder& build, double sampleRate)
    {
        builder = build;
        numDevs = 0;
        dk = DKCircuit();
        build (*this);
        m = dk.numPorts();
        dk.prepare (sampleRate);
        maxHeldRun = std::max (1, (int) (1e-3 * sampleRate));
    }

    // DC operating point for the source values u: solved in a DC twin of the circuit (prepared at a very low rate,
    // so capacitors are open and inductors shorts). As SPICE: a conductance gmin from every node to ground, stepped
    // down from 1 mS to 10 pS (a node reached only through junctions, like a Darlington's middle, otherwise floats
    // at DC and Newton cannot move it), with the sources ramped up from 0 at the first gmin (source stepping). The
    // running circuit then starts there: capacitor voltages, inductor currents, port voltages. Allocates (call
    // from prepare). Returns false if the DC solve did not converge.
    bool initDC (const double* u, int numInputs)
    {
        SemiNetwork dc;
        dc.numDevs = 0;
        dc.testDampedOnly = testDampedDC;
        builder (dc);
        const int nn = dc.dk.numNodesTotal();
        std::vector<int> gminR;
        for (int n = 0; n < nn; ++n)
        {
            dc.dk.markOutput (n);
            gminR.push_back (dc.dk.addR (n, DKCircuit::GND, 1e3));
        }
        dc.m = dc.dk.numPorts();
        dc.dk.prepare (1e-6); // T = 1e6 s
        dc.resTol = 1e-7; // 100 nV: the line amp (T2 shorted by its magnetizing inductance) bottoms out at ~4e-9 V
        dc.acceptSmallStep = false;
        auto solveAt = [&dc] (const double* us) // plain Newton (junction-limited, as SPICE), backtracking if that fails
        {
            const auto& p0 = dc.dk.computeP0 (us);
            double pStart[kMaxPorts];
            std::copy (dc.p, dc.p + dc.m, pStart);
            if (dc.newton (p0, 300)) return true;
            std::copy (pStart, pStart + dc.m, dc.p);
            return dc.newtonDamped (p0, 400);
        };
        auto setGmin = [&] (double rg) { for (int r : gminR) dc.dk.setR (r, rg); };
        const double kFinalR = 1e11; // 10 pS left to every node (SPICE's GMIN is 1 pS across junctions)

        // 1. direct Newton
        setGmin (kFinalR);
        bool ok = solveAt (u);
        // 2. source stepping
        double us[kMaxInputs];
        if (! ok)
        {
            std::fill (dc.p, dc.p + dc.m, 0.0);
            ok = true;
            for (int s = 1; s <= 20 && ok; ++s)
            {
                for (int k = 0; k < numInputs; ++k) us[k] = u[k] * s / 20.0;
                ok = solveAt (us);
            }
        }
        // 3. gmin stepping (with source stepping at the largest gmin)
        if (! ok)
        {
            std::fill (dc.p, dc.p + dc.m, 0.0);
            setGmin (1e3);
            ok = true;
            for (int s = 1; s <= 20 && ok; ++s)
            {
                for (int k = 0; k < numInputs; ++k) us[k] = u[k] * s / 20.0;
                ok = solveAt (us);
            }
            for (double rg = 1e3; rg < kFinalR && ok; )
            {
                double pSave[kMaxPorts];
                std::copy (dc.p, dc.p + dc.m, pSave);
                ok = false;
                for (double next : { rg * 10.0, rg * 3.0, rg * 1.5, rg * 1.2 }) // smaller gmin steps if a big one fails
                {
                    next = std::min (next, kFinalR);
                    setGmin (next);
                    std::copy (pSave, pSave + dc.m, dc.p);
                    if ((ok = solveAt (u))) { rg = next; break; }
                }
                if (! ok)
                {
                    // the last steps can stall at round-off (a node held only by gmin): keep the last converged
                    // level once gmin is below 1 nS (nanoamps: nothing in these circuits notices)
                    setGmin (rg);
                    std::copy (pSave, pSave + dc.m, dc.p);
                    ok = rg >= 1e9;
                    break;
                }
            }
        }

        double i[kMaxPorts], d[kMaxPorts][2];
        dc.evalDevices (i, d);
        dc.dk.finish (i); // node voltages (states are discarded below)

        for (int c = 0; c < dk.numCaps(); ++c)
        {
            int a, b; dc.dk.capNodes (c, a, b);
            dk.setCapVoltage (c, dc.dk.nodeV (a) - dc.dk.nodeV (b));
        }
        for (int l = 0; l < dk.numInds(); ++l)
        {
            int a, b; dc.dk.indNodes (l, a, b);
            dk.setIndCurrent (l, dc.dk.indConductance (l) * (dc.dk.nodeV (a) - dc.dk.nodeV (b)));
        }
        for (int k = 0; k < m; ++k) { p[k] = dc.p[k]; pPrev[k] = p[k]; }
        dcNodes.assign ((size_t) nn, 0.0);
        for (int n = 0; n < nn; ++n) dcNodes[(size_t) n] = dc.dk.nodeV (n);
        dcConverged = ok;
        return ok;
    }

    // node voltage of the last DC operating point (initDC), any node
    double dcNode (int n) const { return n < 0 ? 0.0 : dcNodes[(size_t) n]; }
    int numDcNodes() const { return (int) dcNodes.size(); }
    bool dcConverged = false;

    // the state to go back to after a persistent fault (allocates: call from prepare, after initDC)
    void saveIdle (int numInputs, const double* u)
    {
        std::copy (p, p + m, pIdle);
        std::copy (p, p + m, pPrev);
        dk.saveState();
        nU = numInputs;
        std::copy (u, u + nU, lastU);
        std::copy (u, u + nU, idleU);
        heldRun = 0;
    }

    // One sample. False when the sample had to be dropped (states hold the last accepted sample).
    bool solve (double* u)
    {
        if (! begin (u)) { hold(); return false; }
        commit();
        return true;
    }

    // A sample in steps, for circuits solved together with another one (Urei1176): begin() solves this sample's
    // port equations for the sources u and computes the output nodes without advancing the states; retry() solves
    // the same sample again for other source values (from the current solution); commit() advances the states with
    // the last solution, hold() drops the sample. begin false: no solution (u may then hold the last accepted
    // sources, as solve() retries a failed sample with them); retry false: no solution for the new u, the previous
    // one is kept (commit() then advances with it).
    bool begin (double* u)
    {
        // The predictor extrapolates the last step, limited like a Newton step (pnjlim): unlimited, it can throw a
        // junction far past its knee as a rectifier turns on, and Newton then fails from there (the 1176's CR2 / CR3)
        double step[kMaxPorts] {};
        for (int k = 0; k < m; ++k)
        {
            step[k] = p[k] - pPrev[k];
            pPrev[k] = p[k];
        }
        if (usePredictor) applyStepImpl (step);
        iterations = 0;
        const bool ok = attempt (u, 8);
        predictorScore = 0.9 * predictorScore + 0.1 * (iterations <= 2 ? 1.0 : 0.0);
        if (usePredictor && predictorScore < 0.3) usePredictor = false;
        else if (! usePredictor && (++predictorRetry & 63) == 0) { usePredictor = true; predictorScore = 0.5; }
        statSamples++;
        statMaxIterations = std::max (statMaxIterations, (long long) iterations);
        if (iterations > 8) ++statSlow;
        statIterations += iterations;
        return ok;
    }

    // A failed retry goes back to the solution it started from (the ports, their currents and the circuit's
    // inputs), so that commit() advances the states with that solution and not with the failed Newton iterate
    bool retry (double* u)
    {
        double pKeep[kMaxPorts], iKeep[kMaxPorts];
        std::copy (p, p + m, pKeep);
        std::copy (cur, cur + m, iKeep);
        dk.saveTrial();
        iterations = 0;
        bool ok = attempt (u, 8);
        statIterations += iterations;
        if (testFailRetries > 0) // tests: a failure that leaves a non-finite iterate behind
        {
            --testFailRetries;
            ok = false;
            for (int k = 0; k < m; ++k) p[k] = cur[k] = std::numeric_limits<double>::quiet_NaN();
            dk.computeP0 (u);
            dk.evalOutputs (cur);
        }
        if (! ok)
        {
            ++statRetryFailures;
            std::copy (pKeep, pKeep + m, p);
            std::copy (iKeep, iKeep + m, cur);
            dk.restoreTrial();
        }
        return ok;
    }

    void commit()
    {
        heldRun = 0;
        std::copy (uNow, uNow + nU, lastU);
        dk.finish (cur); // the currents of the accepted solution (Newton leaves them in cur)
    }

    void hold()
    {
        ++statHeld;
        std::copy (pPrev, pPrev + m, p);
        if (++heldRun > maxHeldRun) restoreIdle();
    }

    // port current k of the last accepted sample
    double portCurrent (int k) const { return cur[k]; }
    double port (int k) const { return p[k]; }

    void resetToIdle() { restoreIdle(); }

    // tests: LTspice-tight convergence
    void setReferencePrecision (bool on) { resTol = on ? 1e-10 : 1e-6; acceptSmallStep = ! on; }

    long long statSamples = 0, statIterations = 0, statFailures = 0, statHeld = 0;
    long long statMaxIterations = 0, statSlow = 0; // most Newton evaluations in one sample; samples needing the fallbacks
    long long statRetryFailures = 0;
    long long statDamped = 0;      // solves that reached the line-search Newton (both plain attempts failed)
    int testFailRetries = 0;       // tests: make the next n retries fail
    bool testDampedOnly = false;   // tests: the plain Newton attempts fail at once, so every solve goes through the
                                   // line-search fallback (newtonDamped)
    bool testDampedDC = false;     // tests: likewise for the DC operating point's solves (initDC; set before prepare)

private:
    struct Dev
    {
        enum Type { kBjt, kJfet, kDiode } type = kDiode;
        int p0 = -1, p1 = -1;
        double vcrit = 0.6;
        BjtModel bjt {};
        JfetModel jfet {};
        DiodeModel diode {};
    };

    // Newton from the current p, then from the last sample's solution, then backtracking; a sample that still has
    // no solution is retried as a lost input sample (the last accepted sources, written back to u)
    bool attempt (double* u, int firstIts)
    {
        const auto& p0 = dk.computeP0 (u); // the reference stays valid: computeP0 updates the same buffer
        auto finite = [this] { double s = 0.0; for (int k = 0; k < m; ++k) s += p[k]; return std::isfinite (s); };
        bool ok = newton (p0, firstIts) && finite();
        if (! ok)
        {
            if (diag) diagNote (p0);
            std::copy (pPrev, pPrev + m, p);
            const int it0 = iterations;
            diagFallback = diag;
            ok = newton (p0, 60) && finite();
            diagFallback = false;
            if (diag) diagFallbackIts += iterations - it0;
        }
        if (! ok && bank && *bank <= 0)
        {
            // iteration bank empty: the last iterate is kept when it nearly solves the sample (as an unconverged
            // tube solve in TriodeNetwork), otherwise the sample is dropped (states held). No backtracking fallbacks.
            evalDevices (cur, dcur);
            double F[kMaxPorts];
            if (! (finite() && residual (p0, cur, F) <= kBankAcceptResidual)) { ++statBankHolds; return false; }
            ++statBankAccepts;
            ok = true;
        }
        if (! ok)
        {
            // the line-search Newton, from the last sample's solution again (the plain Newton that failed from there may
            // have left p anywhere, even non-finite); it needs a start with a finite residual
            ++statDamped;
            std::copy (pPrev, pPrev + m, p);
            ok = newtonDamped (p0, 300) && finite();
        }
        if (! ok)
        {
            statFailures++;
            std::copy (lastU, lastU + nU, u);
            dk.computeP0 (u);
            std::copy (pPrev, pPrev + m, p);
            ok = newtonDamped (p0, 300) && finite();
            if (! ok) return false;
        }
        std::copy (u, u + nU, uNow);
        dk.evalOutputs (cur);
        return true;
    }

    static double critical (double nvt, double is) { return nvt * std::log (nvt / (std::sqrt (2.0) * is)); }

    // e^x with a linear continuation above x = 80 (no overflow; Newton's limiting keeps real solves far below), 0 below -40
    static void expLin (double x, double& e, double& de)
    {
        if (x < -40000.0) { e = 0.0; de = 0.0; return; } // a junction this far reversed: e^-40 = 4e-18, below double precision of 1
        if (x < 80.0) { e = std::exp (x); de = e; }
        else { const double e80 = std::exp (80.0); e = e80 * (1.0 + (x - 80.0)); de = e80; }
    }

    // SPICE pnjlim: limit a junction voltage step so the exponential does not overshoot
    static double pnjlim (double vnew, double vold, double nvt, double vcrit)
    {
        if (vnew > vcrit && std::abs (vnew - vold) > 2.0 * nvt)
        {
            if (vold > 0.0)
            {
                const double arg = 1.0 + (vnew - vold) / nvt;
                return arg > 0.0 ? vold + nvt * std::log (arg) : vcrit;
            }
            return nvt * std::log (vnew / nvt);
        }
        return vnew;
    }

    // device currents i (per port) and their derivatives d[port][0 / 1] w.r.t. the device's first / second port
    void evalDevices (double (&i)[kMaxPorts], double (&d)[kMaxPorts][2]) const
    {
        for (int n = 0; n < numDevs; ++n)
        {
            const Dev& v = devs[n];
            if (v.type == Dev::kBjt)
            {
                const double vbe = p[v.p0], vbc = p[v.p1];
                const auto& md = v.bjt;
                double ef, def, er, der;
                expLin (vbe / kVt, ef, def);
                expLin (vbc / kVt, er, der);
                const double iF = md.is * (ef - 1.0), iR = md.is * (er - 1.0);
                const double gF = md.is * def / kVt, gR = md.is * der / kVt;
                const double k = 1.0 - vbc / md.vaf;
                const double ict = (iF - iR) * k;
                const double dIctX = gF * k, dIctY = -gR * k - (iF - iR) / md.vaf;
                const double ib = iF / md.bf + iR / md.br;
                const double ic = ict - iR / md.br;
                const double dIbX = gF / md.bf, dIbY = gR / md.br;
                const double dIcX = dIctX, dIcY = dIctY - gR / md.br;
                i[v.p0] = ib + ic; d[v.p0][0] = dIbX + dIcX; d[v.p0][1] = dIbY + dIcY;
                i[v.p1] = -ic;     d[v.p1][0] = -dIcX;       d[v.p1][1] = -dIcY;
            }
            else if (v.type == Dev::kJfet)
            {
                const double vgs = p[v.p0], vds = p[v.p1];
                const auto& md = v.jfet;
                const double s2 = 2.0 * md.n * kVt;
                auto f = [&] (double x, double& df)
                {
                    // softplus and its slope (the logistic) from one exponential, stable for large |x|
                    const double t = x / s2;
                    double sp, sig;
                    if (t > 30.0) { sp = s2 * t; sig = 1.0; }
                    else if (t < -30.0) { const double e = std::exp (t); sp = s2 * e; sig = e; }
                    else { const double e = std::exp (t); sp = s2 * std::log1p (e); sig = e / (1.0 + e); }
                    df = 2.0 * sp * sig;
                    return sp * sp;
                };
                double dfa, dfb;
                const double fa = f (vgs - md.vto, dfa), fb = f (vgs - vds - md.vto, dfb);
                const double lam = 1.0 + md.lambda * std::abs (vds);
                const double sgn = vds > 0.0 ? 1.0 : (vds < 0.0 ? -1.0 : 0.0);
                const double ich = md.beta * (fa - fb) * lam;
                const double dIchX = md.beta * (dfa - dfb) * lam;
                const double dIchW = md.beta * dfb * lam + md.beta * (fa - fb) * md.lambda * sgn;
                double egs, degs, egd, degd;
                expLin (vgs / kVt, egs, degs);
                expLin ((vgs - vds) / kVt, egd, degd);
                const double igs = md.isGate * (egs - 1.0), igd = md.isGate * (egd - 1.0);
                const double ggs = md.isGate * degs / kVt, ggd = md.isGate * degd / kVt;
                i[v.p0] = igs + igd;  d[v.p0][0] = ggs + ggd;     d[v.p0][1] = -ggd;
                i[v.p1] = ich - igd;  d[v.p1][0] = dIchX - ggd;   d[v.p1][1] = dIchW + ggd;
            }
            else
            {
                const auto& md = v.diode;
                const double nvt = md.n * kVt;
                double e, de;
                expLin (p[v.p0] / nvt, e, de);
                i[v.p0] = md.is * (e - 1.0);
                d[v.p0][0] = md.is * de / nvt;
                d[v.p0][1] = 0.0;
            }
        }
    }

    // apply the Newton step dp to p with junction limiting; returns the largest (unlimited) step
    double applyStep (const double (&dp)[kMaxPorts])
    {
        if (diagFallback) // tests: which ports' steps get limited while the fallback Newton runs
        {
            double q[kMaxPorts];
            std::copy (p, p + m, q);
            const double r = applyStepImpl (dp);
            for (int k = 0; k < m; ++k)
                if (std::abs ((p[k] - q[k]) - dp[k]) > 1e-9 + 1e-6 * std::abs (dp[k])) ++diagLimited[k];
            ++diagSteps;
            return r;
        }
        return applyStepImpl (dp);
    }

public:
    // Iteration bank (real-time bound): when set, every Newton evaluation takes one from *bank and Newton stops when
    // it is empty; the owner refills it per sample up to a cap, so a block's work is bounded (Urei1176). nullptr:
    // unlimited, as before.
    int* bank = nullptr;
    long long statBankAccepts = 0, statBankHolds = 0; // samples ended by an empty bank: kept / dropped
    static constexpr double kBankAcceptResidual = 1e-2;

    // ---- Convergence diagnostics (tests; off unless diag is set) ----
    // A first Newton attempt that fails: the port with the largest residual then (diagWorst), and in the fallback
    // Newton from the last sample's solution the steps that the junction limiting cut short, per port (diagLimited)
    bool diag = false;
    long long diagFirstFails = 0, diagFallbackIts = 0, diagSteps = 0;
    long long diagWorst[kMaxPorts] {}, diagLimited[kMaxPorts] {};
    int numPorts() const { return m; }
    // device owning port k (index in add order) and its type (0 BJT, 1 JFET, 2 diode); -1 if none
    int portDevice (int k) const { for (int n = 0; n < numDevs; ++n) if (devs[n].p0 == k || devs[n].p1 == k) return n; return -1; }
    int deviceType (int n) const { return (int) devs[n].type; }
    bool portIsSecond (int k) const { const int n = portDevice (k); return n >= 0 && devs[n].p1 == k; }

private:
    bool diagFallback = false;
    template <typename P0>
    void diagNote (const P0& p0)
    {
        ++diagFirstFails;
        double F[kMaxPorts];
        residual (p0, cur, F);
        int w = 0;
        for (int k = 1; k < m; ++k) if (std::abs (F[k]) > std::abs (F[w])) w = k;
        ++diagWorst[w];
    }

    double applyStepImpl (const double (&dp)[kMaxPorts])
    {
        double maxStep = 0.0;
        for (int n = 0; n < numDevs; ++n)
        {
            const Dev& v = devs[n];
            if (v.type == Dev::kBjt)
            {
                for (int k : { v.p0, v.p1 })
                {
                    const double nv = std::clamp (p[k] + dp[k], p[k] - 20.0, p[k] + 20.0);
                    p[k] = pnjlim (nv, p[k], kVt, v.vcrit);
                }
            }
            else if (v.type == Dev::kJfet)
            {
                // gate-source junction limited, then the gate-drain junction likewise (through Vds)
                const double vgsOld = p[v.p0], vdsOld = p[v.p1];
                const double vgs = pnjlim (std::clamp (vgsOld + dp[v.p0], vgsOld - 5.0, vgsOld + 5.0), vgsOld, kVt, v.vcrit);
                const double vds = vdsOld + std::clamp (dp[v.p1], -5.0, 5.0);
                const double vgd = pnjlim (vgs - vds, vgsOld - vdsOld, kVt, v.vcrit);
                p[v.p0] = vgs;
                p[v.p1] = vgs - vgd;
            }
            else
                p[v.p0] = pnjlim (std::clamp (p[v.p0] + dp[v.p0], p[v.p0] - 20.0, p[v.p0] + 20.0), p[v.p0],
                                  v.diode.n * kVt, v.vcrit);
            maxStep = std::max (maxStep, std::abs (dp[v.p0]));
            if (v.p1 >= 0) maxStep = std::max (maxStep, std::abs (dp[v.p1]));
        }
        return maxStep;
    }

    // residual F = p - p0 - K i(p) (largest |F|, V)
    template <typename P0>
    double residual (const P0& p0, const double (&i)[kMaxPorts], double (&F)[kMaxPorts]) const
    {
        const auto& K = dk.getK();
        double maxF = 0.0;
        for (int r = 0; r < m; ++r)
        {
            double Ki = 0.0;
            for (int k = 0; k < m; ++k) Ki += K[(size_t) r * m + k] * i[k];
            F[r] = p[r] - p0[(size_t) r] - Ki;
            const double a = std::abs (F[r]);
            if (! (a <= maxF)) maxF = a; // NaN propagates (std::max would drop it)
        }
        return maxF;
    }

    // the same for NP ports (fixed size)
    template <int NP, typename P0>
    double residualN (const P0& p0, const double (&i)[kMaxPorts], double (&F)[kMaxPorts]) const
    {
        const auto& K = dk.getK();
        double maxF = 0.0;
        for (int r = 0; r < NP; ++r)
        {
            double Ki = 0.0;
            for (int k = 0; k < NP; ++k) Ki += K[(size_t) r * NP + k] * i[k];
            F[r] = p[r] - p0[(size_t) r] - Ki;
            const double a = std::abs (F[r]);
            if (! (a <= maxF)) maxF = a; // NaN propagates (std::max would drop it)
        }
        return maxF;
    }

    // Newton, dispatched to a fixed-size instance per port count (the same operations, unrolled)
    template <typename P0>
    bool newton (const P0& p0, int maxIt)
    {
        if (testDampedOnly) return false; // tests: every solve through the line-search fallback
        switch (m)
        {
            case 1: return newtonN<1> (p0, maxIt);
            case 2: return newtonN<2> (p0, maxIt);
            case 3: return newtonN<3> (p0, maxIt);
            case 4: return newtonN<4> (p0, maxIt);
            case 5: return newtonN<5> (p0, maxIt);
            case 6: return newtonN<6> (p0, maxIt);
            case 7: return newtonN<7> (p0, maxIt);
            case 8: return newtonN<8> (p0, maxIt);
            case 9: return newtonN<9> (p0, maxIt);
            case 10: return newtonN<10> (p0, maxIt);
            case 11: return newtonN<11> (p0, maxIt);
            default: return newtonN<12> (p0, maxIt);
        }
    }

    template <int NP, typename P0>
    bool newtonN (const P0& p0, int maxIt)
    {
        const auto& K = dk.getK();
        auto& i = cur;
        auto& d = dcur;
        double F[kMaxPorts], pOld[kMaxPorts];
        for (int it = 0; it < maxIt; ++it)
        {
            if (bank && (*bank)-- <= 0) { *bank = 0; return false; }
            ++iterations;
            evalDevices (i, d);
            const double maxF = residualN<NP> (p0, i, F);
            if (! std::isfinite (maxF)) return false;
            if (maxF < resTol) return true;

            double J[kMaxPorts][kMaxPorts + 1];
            for (int r = 0; r < NP; ++r)
            {
                for (int c = 0; c < NP; ++c) J[r][c] = r == c ? 1.0 : 0.0;
                J[r][NP] = -F[r];
            }
            for (int n = 0; n < numDevs; ++n)
            {
                const Dev& v = devs[n];
                const int c0 = v.p0, c1 = v.p1;
                for (int r = 0; r < NP; ++r)
                {
                    const double k0 = K[(size_t) r * NP + c0];
                    if (c1 < 0) { J[r][c0] -= k0 * d[c0][0]; continue; }
                    const double k1 = K[(size_t) r * NP + c1];
                    J[r][c0] -= k0 * d[c0][0] + k1 * d[c1][0];
                    J[r][c1] -= k0 * d[c0][1] + k1 * d[c1][1];
                }
            }
            double dp[kMaxPorts];
            if (! solveLinearN<NP> (J, dp)) return false;
            std::copy (p, p + NP, pOld);
            applyStep (dp);
            // Quadratic convergence: after a full (unlimited) step this small the remaining error is ~step^2, so it
            // is accepted without evaluating the devices again; the currents move along their linearisation.
            if (acceptSmallStep)
            {
                double maxDp = 0.0;
                bool limited = false;
                for (int k = 0; k < NP; ++k)
                {
                    const double s = p[k] - pOld[k];
                    limited = limited || std::abs (s - dp[k]) > 1e-12 + 1e-9 * std::abs (dp[k]); // (p + dp) - p is not exactly dp
                    maxDp = std::max (maxDp, std::abs (s));
                }
                if (! limited && maxDp < kSmallStep)
                {
                    for (int n = 0; n < numDevs; ++n)
                    {
                        const Dev& v = devs[n];
                        const double s0 = p[v.p0] - pOld[v.p0], s1 = v.p1 >= 0 ? p[v.p1] - pOld[v.p1] : 0.0;
                        i[v.p0] += d[v.p0][0] * s0 + d[v.p0][1] * s1;
                        if (v.p1 >= 0) i[v.p1] += d[v.p1][0] * s0 + d[v.p1][1] * s1;
                    }
                    return true;
                }
            }
        }
        return false;
    }

    // Line-search Newton: the fallback once both plain Newton attempts have failed, and the DC operating point's
    // last resort. The junction-limited Newton step, backtracked along itself (t = 1, 1/2, 1/4 ...) until the
    // residual's 2-norm decreases sufficiently (Armijo on f = |F|^2 / 2, as Dennis-Schnabel / Kelley): the Newton
    // direction is a descent direction of f, so a decreasing step exists whenever J is right, and near the
    // solution the full step is taken (quadratic convergence, as the plain Newton). A step that still does not
    // decrease f at 2^-24 of itself means a local minimum of f that is no root, or no descent direction: the solve
    // fails rather than wandering on. (A plain "halve while the max residual grows" search cannot do this job: a
    // full step past a junction's knee makes the residual grow by orders of magnitude, so it accepts only tiny
    // fractions of each step and crawls.) maxIt counts device evaluations. Starts from p, whose residual must be
    // finite; on failure p is wherever the search was (the callers restore their own start).
    template <typename P0>
    bool newtonDamped (const P0& p0, int maxIt)
    {
        constexpr double kArmijo = 1e-4;
        const auto& K = dk.getK();
        auto& i = cur;
        auto& d = dcur;
        double F[kMaxPorts], pOld[kMaxPorts], pFull[kMaxPorts];
        auto sumSq = [this] (const double (&v)[kMaxPorts]) { double s = 0.0; for (int k = 0; k < m; ++k) s += v[k] * v[k]; return s; };
        if (bank && (*bank)-- <= 0) { *bank = 0; return false; }
        ++iterations;
        evalDevices (i, d);
        double maxF = residual (p0, i, F);
        if (! std::isfinite (maxF)) return false;
        double f = sumSq (F);
        for (int it = 1; it < maxIt;)
        {
            if (maxF < resTol) return true;

            double J[kMaxPorts][kMaxPorts + 1];
            for (int r = 0; r < m; ++r)
            {
                for (int c = 0; c < m; ++c) J[r][c] = r == c ? 1.0 : 0.0;
                J[r][m] = -F[r];
            }
            for (int n = 0; n < numDevs; ++n)
            {
                const Dev& v = devs[n];
                const int c0 = v.p0, c1 = v.p1;
                for (int r = 0; r < m; ++r)
                {
                    const double k0 = K[(size_t) r * m + c0];
                    if (c1 < 0) { J[r][c0] -= k0 * d[c0][0]; continue; }
                    const double k1 = K[(size_t) r * m + c1];
                    J[r][c0] -= k0 * d[c0][0] + k1 * d[c1][0];
                    J[r][c1] -= k0 * d[c0][1] + k1 * d[c1][1];
                }
            }
            double dp[kMaxPorts];
            if (! solveLinear (J, dp)) return false; // singular Jacobian: no step to take
            std::copy (p, p + m, pOld);
            applyStep (dp);
            std::copy (p, p + m, pFull); // the junction-limited step's end

            // 1. along the junction-limited step as the plain Newton takes it (t = 1 is accepted wherever the plain
            //    Newton would have converged from here), backtracked a few times;
            // 2. if that gives no decrease (a junction's step was cut short while the rest of the step assumed it
            //    whole: the combination need not be a descent direction), the whole Newton step scaled uniformly to
            //    the fraction the most limited junction got, backtracked as far as it takes. That is a scaled Newton
            //    direction, so a descent direction of f: the junction moves exactly as far as the limiting allows
            //    and the rest of the circuit follows it consistently.
            bool accepted = false;
            bool limited = false;
            double t = 1.0;
            for (int phase = 0; phase < 2 && ! accepted; ++phase)
            {
                if (phase == 1)
                {
                    double t0 = 1.0;
                    for (int k = 0; k < m; ++k)
                        if (dp[k] != 0.0) t0 = std::min (t0, std::abs (pFull[k] - pOld[k]) / std::abs (dp[k]));
                    limited = t0 < 1.0;
                    if (limited) // the scaled Newton step from the most limited junction's fraction
                    {
                        t = std::max (t0, 1e-9);
                        for (int k = 0; k < m; ++k) p[k] = pOld[k] + t * dp[k];
                    } // else: nothing was limited, the step is the Newton step already: backtrack further along it
                }
                for (int ls = 0; ls < (phase == 0 ? 4 : 30) && it < maxIt; ++ls)
                {
                    if (bank && (*bank)-- <= 0) { *bank = 0; return false; }
                    ++iterations;
                    ++it;
                    evalDevices (i, d);
                    const double mf = residual (p0, i, F);
                    if (std::isfinite (mf))
                    {
                        const double fn = sumSq (F);
                        if (fn <= f * (1.0 - 2.0 * kArmijo * t)) { f = fn; maxF = mf; accepted = true; break; }
                    }
                    t *= 0.5;
                    if (phase == 1 && limited) for (int k = 0; k < m; ++k) p[k] = pOld[k] + t * dp[k];
                    else for (int k = 0; k < m; ++k) p[k] = pOld[k] + t * (pFull[k] - pOld[k]);
                }
            }
            if (! accepted) return false;
        }
        return false;
    }

    // dispatched to a fixed-size instance per port count (the same operations, unrolled)
    bool solveLinear (double (&a)[kMaxPorts][kMaxPorts + 1], double (&x)[kMaxPorts]) const
    {
        switch (m)
        {
            case 1:  return solveLinearN<1> (a, x);
            case 2:  return solveLinearN<2> (a, x);
            case 3:  return solveLinearN<3> (a, x);
            case 4:  return solveLinearN<4> (a, x);
            case 5:  return solveLinearN<5> (a, x);
            case 6:  return solveLinearN<6> (a, x);
            case 7:  return solveLinearN<7> (a, x);
            case 8:  return solveLinearN<8> (a, x);
            case 9:  return solveLinearN<9> (a, x);
            case 10: return solveLinearN<10> (a, x);
            case 11: return solveLinearN<11> (a, x);
            default: return solveLinearN<12> (a, x);
        }
    }

    template <int n>
    static bool solveLinearN (double (&a)[kMaxPorts][kMaxPorts + 1], double (&x)[kMaxPorts])
    {
        double invPivot[kMaxPorts];
        for (int col = 0; col < n; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < n; ++r)
                if (std::abs (a[r][col]) > std::abs (a[piv][col])) piv = r;
            if (std::abs (a[piv][col]) < 1e-300) return false;
            if (piv != col)
                for (int j = 0; j <= n; ++j) std::swap (a[col][j], a[piv][j]);
            invPivot[col] = 1.0 / a[col][col];
            for (int r = col + 1; r < n; ++r)
            {
                const double f = a[r][col] * invPivot[col];
                if (f == 0.0) continue;
                for (int j = col; j <= n; ++j) a[r][j] -= f * a[col][j];
            }
        }
        for (int r = n - 1; r >= 0; --r)
        {
            double s = a[r][n];
            for (int j = r + 1; j < n; ++j) s -= a[r][j] * x[j];
            x[r] = s * invPivot[r];
        }
        return true;
    }

    void restoreIdle()
    {
        dk.restoreState();
        std::copy (pIdle, pIdle + m, p);
        std::copy (pIdle, pIdle + m, pPrev);
        std::copy (idleU, idleU + nU, lastU);
        heldRun = 0;
    }

    Builder builder;
    Dev devs[kMaxPorts];
    int numDevs = 0, m = 0;
    double p[kMaxPorts] {}, pPrev[kMaxPorts] {}, pIdle[kMaxPorts] {};
    double cur[kMaxPorts] {}, dcur[kMaxPorts][2] {};
    double lastU[kMaxInputs] {}, idleU[kMaxInputs] {}, uNow[kMaxInputs] {};
    std::vector<double> dcNodes;
    int nU = 0, heldRun = 0, maxHeldRun = 96, iterations = 0;
    double resTol = 1e-6;
    bool acceptSmallStep = true;
    static constexpr double kSmallStep = 1e-3; // V: a Newton step this small is accepted without re-evaluating
    bool usePredictor = true;
    double predictorScore = 1.0;
    unsigned predictorRetry = 0;
};

} // namespace vamp
