#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>
#include "Profile.h"
#include "Simd.h"

namespace vamp
{
// Nodal DK-method circuit solver (Yeh, "Automated physical modeling of nonlinear audio
// circuits", 2009-style formulation).
//
// Linear part: resistors, capacitors, inductors (trapezoidal companions), Norton voltage
// sources and ideal multi-winding transformers, all folded into one MNA matrix A that is
// inverted once per rebuild (pots change -> rebuild).
// Nonlinear part: "ports" (voltage between two nodes, current flowing a -> b). Per sample the
// caller solves only the m-dimensional port equation
//        p = p0 + K * i(p)
// with Newton's method, then calls finish(i) to advance the reactive states.
//
// Per sample everything is expressed in the reactive states s (one companion current per C / L),
// the inputs u and the port currents i, with matrices precomputed at rebuild:
//        p0 = P [s; u]        s' = M [s; u; i]        v_out = O [s; u; i]
// so only the states and the marked output nodes are ever computed (same math as solving the full
// MNA system, about half the multiply-adds).
class DKCircuit
{
public:
    static constexpr int GND = -1;

    struct Winding { int a, b; double turns; };

    int addNode() { return numNodes++; }

    // noisy: the resistor's thermal (Johnson) noise is simulated when noise is on (setNoise): a noise current of
    // spectral density 4kT/R across it, white up to Nyquist. Structural: set before prepare().
    int addR (int a, int b, double r, bool noisy = false)
    {
        resistors.push_back ({ a, b, 1.0 / std::max (r, 1e-3), std::max (r, 1e-3), noisy, -1 });
        dirty = true;
        return (int) resistors.size() - 1;
    }

    void setR (int idx, double r)
    {
        auto& e = resistors[(size_t) idx];
        const double g = 1.0 / std::max (r, 1e-3);
        if (e.g == g) return;
        e.g = g;
        e.r = std::max (r, 1e-3);
        if (e.var >= 0) varDirty = true; // a pot: low-rank update (updateVariable), not a rebuild
        else dirty = true;
    }

    // Resistor idx (addR) changes while running (a pot): setR on it then corrects the matrices by a rank-r update
    // instead of rebuilding them. It is solved in resistance form (a branch-current unknown, see rebuild), which is
    // well conditioned as a short too. A rebuild builds the matrices with every variable resistor at its reference value rRef
    // and keeps the correction's factors; each later value is applied from that same base (Woodbury identity), so
    // the matrices depend only on the present values, never on the knob's path. Structural: before prepare().
    void markVariable (int idx, double rRef)
    {
        assert (numVar < kMaxVar && resistors[(size_t) idx].var < 0);
        resistors[(size_t) idx].var = numVar;
        var[numVar].res = idx;
        var[numVar].rRef = std::max (rRef, 1e-3);
        ++numVar;
        dirty = true;
    }
    static constexpr int kMaxVar = 4;

    int addC (int a, int b, double c) { caps.push_back ({ a, b, c, 0 }); dirty = true; return (int) caps.size() - 1; }
    int addL (int a, int b, double l) { inds.push_back ({ a, b, l, 0 }); dirty = true; return (int) inds.size() - 1; }

    // change a capacitor / inductor (addC / addL index) after prepare(): the matrices are rebuilt on the next sample
    void setC (int idx, double c) { auto& e = caps[(size_t) idx]; e.c = c; e.gc = 2.0 * c / T; dirty = true; }
    void setL (int idx, double l) { auto& e = inds[(size_t) idx]; e.l = l; e.gl = T / (2.0 * l); dirty = true; }

    // node --[rs]-- u[k]; returns the input index k. noisy: rs's thermal noise (a series noise voltage, 4kT rs)
    int addSource (int node, double rs, bool noisy = false)
    {
        sources.push_back ({ node, 1.0 / rs, noisy });
        dirty = true;
        return (int) sources.size() - 1;
    }

    // Thermal noise of the noisy resistors and sources on / off (no rebuild), at kelvin; seed per circuit
    void setNoise (bool on, unsigned long long seed = 0x9e3779b97f4a7c15ull, double kelvin = 300.0)
    {
        noiseOn = on;
        rng = seed | 1ull;
        noiseKT = kBoltzmann * kelvin;
        dirty = true; // the noise amplitudes are computed in rebuild()
    }
    bool getNoise() const { return noiseOn; }

    // uniform white noise in [-1, 1) (xorshift64*): a uniform of variance 1/3, scaled by sqrt(3) sigma
    double noiseUniform()
    {
        rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
        return (double) ((rng * 0x2545F4914F6CDD1Dull) >> 11) * (2.0 / 9007199254740992.0) - 1.0;
    }

    void setSourceResistance (int k, double rs)
    {
        if (sources[(size_t) k].g != 1.0 / rs) { sources[(size_t) k].g = 1.0 / rs; dirty = true; }
    }

    void addTransformer (std::vector<Winding> w)
    {
        transformers.push_back (std::move (w));
        dirty = true;
    }

    // port voltage = v(a) - v(b); port current flows a -> b through the nonlinear device
    int addPort (int a, int b)
    {
        ports.push_back ({ a, b });
        dirty = true;
        return (int) ports.size() - 1;
    }

    int numPorts() const { return (int) ports.size(); }
    int numInputs() const { return (int) sources.size(); }
    int numNodesTotal() const { return numNodes; }
    int numUnknowns() const { return N; } // MNA size (nodes + transformer aux), after prepare()
    int numStates() const { return (int) (caps.size() + inds.size()); }
    int numOutputs() const { return (int) outputs.size(); }

    // Element topology, for a DC operating point solved in a second instance of the same circuit (SemiNetwork):
    // capacitor / inductor / port idx -> its nodes (a, b)
    int numCaps() const { return (int) caps.size(); }
    int numInds() const { return (int) inds.size(); }
    void capNodes (int idx, int& a, int& b) const { a = caps[(size_t) idx].a; b = caps[(size_t) idx].b; }
    void indNodes (int idx, int& a, int& b) const { a = inds[(size_t) idx].a; b = inds[(size_t) idx].b; }
    void portNodes (int idx, int& a, int& b) const { a = ports[(size_t) idx].a; b = ports[(size_t) idx].b; }
    // inductor idx's companion conductance T / 2L (after prepare)
    double indConductance (int idx) const { return inds[(size_t) idx].gl; }

    // Start inductor idx (addL's return value) carrying current i (a -> b) at zero voltage: the DC steady state of
    // its trapezoidal companion (i = gl v + ieq, v = 0). Call after prepare().
    void setIndCurrent (int idx, double i) { z[caps.size() + (size_t) idx] = i; }

    void prepare (double sampleRate)
    {
        T = 1.0 / sampleRate;
        for (auto& c : caps) c.gc = 2.0 * c.c / T;
        for (auto& l : inds) l.gl = T / (2.0 * l.l);
        // aux unknowns: one current per winding + one "volts per turn" per transformer
        numAux = 0;
        for (auto& t : transformers) numAux += (int) t.size() + 1;
        // one branch-current unknown per variable resistor (after the transformers' aux unknowns): see rebuild
        N = numNodes + numAux + numVar;
        for (int v = 0; v < numVar; ++v) var[v].row = numNodes + numAux + v;
        m = (int) ports.size();
        numS = (int) (caps.size() + inds.size());
        numSrc = (int) sources.size();
        numNoise = 0;
        for (auto& r : resistors) numNoise += r.noisy ? 1 : 0;
        // z = [s; u; i; noise]: the noise inputs (one current per noisy resistor) last, so that with noise off every
        // product stops before them and costs what it did without noise
        nzMain = numS + numSrc + m;
        nz = nzMain + numNoise;
        noiseAmp.assign ((size_t) (numSrc + numNoise), 0.0);
        assert (numS <= kMaxStates && (int) outputs.size() <= kMaxStates); // finish() keeps both on the stack
        x.assign ((size_t) N, 0.0);
        z.assign ((size_t) nz, 0.0);
        zTrial.assign ((size_t) nz, 0.0);
        p0.assign ((size_t) m, 0.0);
        // rebuild() reuses these (assign to the same size does not reallocate)
        bufA.assign ((size_t) N * N, 0.0);
        bufInv.assign ((size_t) N * N, 0.0);
        bufAinvNi.assign ((size_t) N * m, 0.0);
        bufV.assign ((size_t) N * nz, 0.0);
        K.assign ((size_t) m * m, 0.0);
        P.assign ((size_t) m * nz, 0.0); // over all of z, zero in the port-current columns
        M.assign ((size_t) numS * nz, 0.0);
        O.assign (outputs.size() * (size_t) nz, 0.0);
        // the same column by column (per-sample products, matVecT), rows padded to a multiple of 4 (the AVX loads)
        ldP = (m + 3) & ~3; ldM = (numS + 3) & ~3; ldO = ((int) outputs.size() + 3) & ~3;
        Pt.assign ((size_t) nz * ldP, 0.0);
        Mt.assign ((size_t) nz * ldM, 0.0);
        Ot.assign ((size_t) nz * ldO, 0.0);
        // the variable resistors' base (markVariable): the matrices at the reference values and the correction factors
        if (numVar > 0)
        {
            K0.assign (K.size(), 0.0); Pt0.assign (Pt.size(), 0.0); Mt0.assign (Mt.size(), 0.0);
            O0.assign (O.size(), 0.0);
            qP.assign ((size_t) kMaxVar * ldP, 0.0); qM.assign ((size_t) kMaxVar * ldM, 0.0); qO.assign ((size_t) kMaxVar * ldO, 0.0);
            yV.assign ((size_t) kMaxVar * nz, 0.0); gV.assign ((size_t) kMaxVar * nz, 0.0);
        }
        // built now (prepare is not on the audio thread): a circuit that only runs later (an idle channel) does not pay
        // for its first rebuild in the audio block where it comes in
        rebuild();
        reset();
    }

    void reset()
    {
        std::fill (z.begin(), z.end(), 0.0);
        std::fill (x.begin(), x.end(), 0.0);
    }

    // Start capacitor idx (addC's return value) charged to v = v(a) - v(b) with no current (DC steady state of
    // the trapezoidal companion: ieq = gc v). Call after prepare().
    void setCapVoltage (int idx, double v) { z[(size_t) idx] = caps[(size_t) idx].gc * v; }

    // Reactive state (trapezoidal companion source) of capacitor idx: copied to an identical capacitor of another
    // circuit prepared at the same rate, it carries over the charge and the current (channel switches)
    double getCapState (int idx) const { return z[(size_t) idx]; }
    void setCapState (int idx, double s) { z[(size_t) idx] = s; }

    // Remember / go back to a state (reactive states and output nodes), e.g. the settled idle point.
    // saveState() allocates (call it from prepare); restoreState() does not.
    void saveState() { zSaved = z; xSaved = x; }
    void restoreState()
    {
        if (zSaved.size() != z.size()) { reset(); return; }
        std::copy (zSaved.begin(), zSaved.end(), z.begin());
        std::copy (xSaved.begin(), xSaved.end(), x.begin());
    }

    // Remember / go back to this sample's inputs, port currents and noise (z past the reactive states): a trial
    // solution to return to when solving the same sample again fails (SemiNetwork::retry; no allocation)
    void saveTrial() { std::copy (z.begin() + numS, z.end(), zTrial.begin() + numS); }
    void restoreTrial() { std::copy (zTrial.begin() + numS, zTrial.end(), z.begin() + numS); }

    // p0 = P [s; u] for the source values u and the current reactive states
    const std::vector<double>& computeP0 (const double* u)
    {
        ensureBuilt();

        std::copy (u, u + numSrc, z.begin() + numS);
        if (noiseOn && numSrc + numNoise > 0)
        {
            double* zu = z.data() + numS;
            for (int k = 0; k < numSrc; ++k) if (noiseAmp[(size_t) k] > 0.0) zu[k] += noiseAmp[(size_t) k] * noiseUniform();
            double* zn = z.data() + nzMain;
            for (int k = 0; k < numNoise; ++k) zn[k] = noiseAmp[(size_t) (numSrc + k)] * noiseUniform();
            matVecT (Pt.data(), ldP, m, nz, z.data(), p0.data()); // (the port-current columns of P are 0)
        }
        else
            matVecT (Pt.data(), ldP, m, numS + numSrc, z.data(), p0.data());
        return p0;
    }

    // K (m x m, row-major): sensitivity of the port voltages to the port currents
    const std::vector<double>& getK() const { return K; }
    unsigned getKVersion() const { return kVersion; } // changes whenever K may have (caches of K: PortSolve.h)

    // Given the converged port currents, compute the output nodes and advance the states
    // (uses the u passed to the preceding computeP0)
    void finish (const double* iPorts)
    {
        evalOutputs (iPorts);
        const int cols = noiseOn ? nz : nzMain;
        double buf[kMaxStates];
        matVecT (Mt.data(), ldM, numS, cols, z.data(), buf);
        std::copy (buf, buf + numS, z.begin());
    }

    // The output nodes for these port currents without advancing the states: a trial solution (finish() commits)
    void evalOutputs (const double* iPorts)
    {
        std::copy (iPorts, iPorts + m, z.begin() + numS + numSrc);
        const int cols = noiseOn ? nz : nzMain;
        double buf[kMaxStates];
        const int numO = (int) outputs.size();
        if (numO <= 2) matVec (O.data(), numO, cols, nz, z.data(), buf); // one or two dot products: by row
        else matVecT (Ot.data(), ldO, numO, cols, z.data(), buf);
        for (int o = 0; o < numO; ++o) x[(size_t) outputs[(size_t) o]] = buf[o];
    }

    // Only nodes marked as outputs are computed in finish(); nodeV() of any other node is 0.
    double nodeV (int n) const { return n < 0 ? 0.0 : x[(size_t) n]; }
    void markOutput (int node) { outputs.push_back (node); dirty = true; }

    void markDirty() { dirty = true; }

    // the matrix rebuild computeP0() would do next, ahead of time (allocation-free; the same matrices either way)
    bool isDirty() const { return dirty || varDirty; }
    void ensureBuilt()
    {
        if (dirty) rebuild();
        else if (varDirty) updateVariable();
    }

    // tests: the next ensureBuilt() / computeP0() rebuilds everything, the variable resistors' values included
    // directly (no low-rank update): the reference the update is checked against (dsp_test potupdate)
    void rebuildDirect() { directOnce = true; dirty = true; ensureBuilt(); }
    void markVariableDirty() { varDirty = numVar > 0; } // tests: redo the low-rank update (solve_profile)
    // tests (golden controls): every rebuild direct, no low-rank updates; every resistor's conductance scaled
    static inline bool testAlwaysDirect = false;
    static inline double testGScale = 1.0;
    const std::vector<double>& getPt() const { return Pt; } // tests: the per-sample matrices
    const std::vector<double>& getMt() const { return Mt; }
    const std::vector<double>& getOt() const { return Ot; }
    const std::vector<double>& getO() const { return O; }

private:
    struct Res { int a, b; double g, r; bool noisy; int var; }; // var: index in var[] when variable (markVariable), else -1
    // row: its branch-current unknown; noiseSlot: its noise amplitude in noiseAmp
    struct Var { int res = -1, row = -1; double rRef = 0.0; int noiseSlot = -1; };
    struct Cap { int a, b; double c, gc; };
    struct Ind { int a, b; double l, gl; };
    struct Src { int node; double g; bool noisy; };
    struct Port { int a, b; };

public:
    // out = A v (A rows x cols, row-major). Four rows at a time: four independent accumulation chains instead of
    // one, so the adds overlap instead of waiting on each other. Each row is still summed in index order, so the
    // result is bit-identical to the plain loop.
    static void matVec (const double* A, int rows, int cols, const double* v, double* out)
    {
        matVec (A, rows, cols, cols, v, out);
    }

    // the same over the first `cols` columns of a matrix whose rows are `stride` long
    static void matVec (const double* A, int rows, int cols, int stride, const double* v, double* out)
    {
        int r = 0;
        for (; r + 4 <= rows; r += 4)
        {
            const double* a0 = A + (size_t) r * stride;
            const double* a1 = a0 + stride;
            const double* a2 = a1 + stride;
            const double* a3 = a2 + stride;
            double s0 = 0.0, s1 = 0.0, s2 = 0.0, s3 = 0.0;
            for (int j = 0; j < cols; ++j)
            {
                const double vj = v[j];
                s0 += a0[j] * vj; s1 += a1[j] * vj; s2 += a2[j] * vj; s3 += a3[j] * vj;
            }
            out[r] = s0; out[r + 1] = s1; out[r + 2] = s2; out[r + 3] = s3;
        }
        for (; r < rows; ++r)
        {
            const double* a = A + (size_t) r * stride;
            double s = 0.0;
            for (int j = 0; j < cols; ++j) s += a[j] * v[j];
            out[r] = s;
        }
    }

    // out = A v with A stored column by column: At[j * ld + r] = A[r][j] (ld a multiple of 4, the pad rows 0). Up to
    // 16 rows at a time in eight 2-wide registers (32 in eight 4-wide AVX registers where the CPU has AVX) across all
    // columns: eight independent sums, so the adds overlap instead of waiting on each other (a tail computes and
    // drops the pad rows). Each row is still summed in column order with a separate multiply and add, so the result
    // is bit-identical to matVec (and the plain loop) on every path. On arm64 32 rows at a time (sixteen sums: NEON has
    // 32 registers) with v read two columns at once and multiplied by lane (tileTLane): -20 % on the Pi 5's A76, still
    // the same operations in the same order.
    static void matVecT (const double* At, int ld, int rows, int cols, const double* v, double* out)
    {
#if VAMP_SIMD_SSE2
        if (vampHasAvx()) { matVecTAvx (At, ld, rows, cols, v, out); return; }
#endif
#if VAMP_SIMD_NEON
        for (int r = 0; r < rows; r += 32)
        {
            const int pairs = std::min (16, (rows - r + 1) / 2);
            double* o = out + r;
            double tmp[32];
            const bool tail = r + 2 * pairs > rows; // an odd count: the last pair's pad row goes to tmp
            double* d = tail ? tmp : o;
            switch (pairs)
            {
                case 16: tileTLane<16> (At + r, ld, cols, v, d); break;
                case 15: tileTLane<15> (At + r, ld, cols, v, d); break;
                case 14: tileTLane<14> (At + r, ld, cols, v, d); break;
                case 13: tileTLane<13> (At + r, ld, cols, v, d); break;
                case 12: tileTLane<12> (At + r, ld, cols, v, d); break;
                case 11: tileTLane<11> (At + r, ld, cols, v, d); break;
                case 10: tileTLane<10> (At + r, ld, cols, v, d); break;
                case 9: tileTLane<9> (At + r, ld, cols, v, d); break;
                case 8: tileTLane<8> (At + r, ld, cols, v, d); break;
                case 7: tileTLane<7> (At + r, ld, cols, v, d); break;
                case 6: tileTLane<6> (At + r, ld, cols, v, d); break;
                case 5: tileTLane<5> (At + r, ld, cols, v, d); break;
                case 4: tileTLane<4> (At + r, ld, cols, v, d); break;
                case 3: tileTLane<3> (At + r, ld, cols, v, d); break;
                case 2: tileTLane<2> (At + r, ld, cols, v, d); break;
                default: tileTLane<1> (At + r, ld, cols, v, d); break;
            }
            if (tail) std::copy (tmp, tmp + (rows - r), o);
        }
#elif VAMP_SIMD_SSE2
        for (int r = 0; r < rows; r += 16)
        {
            const int pairs = std::min (8, (rows - r + 1) / 2);
            double* o = out + r;
            double tmp[16];
            const bool tail = r + 2 * pairs > rows; // an odd count: the last pair's pad row goes to tmp
            switch (pairs)
            {
                case 8: tileT<8> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 7: tileT<7> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 6: tileT<6> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 5: tileT<5> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 4: tileT<4> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 3: tileT<3> (At + r, ld, cols, v, tail ? tmp : o); break;
                case 2: tileT<2> (At + r, ld, cols, v, tail ? tmp : o); break;
                default: tileT<1> (At + r, ld, cols, v, tail ? tmp : o); break;
            }
            if (tail) std::copy (tmp, tmp + (rows - r), o);
        }
#else
        for (int r = 0; r < rows; ++r)
        {
            double s = 0.0;
            for (int j = 0; j < cols; ++j) s += At[(size_t) j * ld + r] * v[j];
            out[r] = s;
        }
#endif
    }

#if VAMP_SIMD_SSE2 || VAMP_SIMD_NEON
#if VAMP_SIMD_SSE2
    // matVecT with 4-wide AVX registers, up to 32 rows at a time
    VAMP_AVX_FN static void matVecTAvx (const double* At, int ld, int rows, int cols, const double* v, double* out)
    {
        for (int r = 0; r < rows; r += 32)
        {
            const int quads = std::min (8, (rows - r + 3) / 4);
            double* o = out + r;
            double tmp[32];
            const bool tail = r + 4 * quads > rows; // the pad rows of the last quad go to tmp
            double* dst = tail ? tmp : o;
            switch (quads)
            {
                case 8: tileAvx<8> (At + r, ld, cols, v, dst); break;
                case 7: tileAvx<7> (At + r, ld, cols, v, dst); break;
                case 6: tileAvx<6> (At + r, ld, cols, v, dst); break;
                case 5: tileAvx<5> (At + r, ld, cols, v, dst); break;
                case 4: tileAvx<4> (At + r, ld, cols, v, dst); break;
                case 3: tileAvx<3> (At + r, ld, cols, v, dst); break;
                case 2: tileAvx<2> (At + r, ld, cols, v, dst); break;
                default: tileAvx<1> (At + r, ld, cols, v, dst); break;
            }
            if (tail) for (int k = 0; k < rows - r; ++k) o[k] = tmp[k];
        }
        _mm256_zeroupper(); // back to SSE code without the transition penalty
    }

    // 4 Q rows of matVecTAvx, all columns
    template <int Q>
    VAMP_AVX_FN static inline void tileAvx (const double* At, int ld, int cols, const double* v, double* out)
    {
        __m256d s[Q];
        for (int k = 0; k < Q; ++k) s[k] = _mm256_setzero_pd();
        for (int j = 0; j < cols; ++j, At += ld)
        {
            const __m256d vj = _mm256_set1_pd (v[j]);
            for (int k = 0; k < Q; ++k) s[k] = _mm256_add_pd (s[k], _mm256_mul_pd (_mm256_loadu_pd (At + 4 * k), vj));
        }
        for (int k = 0; k < Q; ++k) _mm256_storeu_pd (out + 4 * k, s[k]);
    }
#endif

    // 2 P rows of matVecT, all columns
    template <int P>
    static inline void tileT (const double* At, int ld, int cols, const double* v, double* out)
    {
  #if VAMP_SIMD_SSE2
        __m128d s[P];
        for (int k = 0; k < P; ++k) s[k] = _mm_setzero_pd();
        for (int j = 0; j < cols; ++j, At += ld)
        {
            const __m128d vj = _mm_set1_pd (v[j]);
            for (int k = 0; k < P; ++k) s[k] = _mm_add_pd (s[k], _mm_mul_pd (_mm_loadu_pd (At + 2 * k), vj));
        }
        for (int k = 0; k < P; ++k) _mm_storeu_pd (out + 2 * k, s[k]);
  #else
        float64x2_t s[P];
        for (int k = 0; k < P; ++k) s[k] = vdupq_n_f64 (0.0);
        for (int j = 0; j < cols; ++j, At += ld)
        {
            const float64x2_t vj = vdupq_n_f64 (v[j]);
            for (int k = 0; k < P; ++k) s[k] = vaddq_f64 (s[k], vmulq_f64 (vld1q_f64 (At + 2 * k), vj));
        }
        for (int k = 0; k < P; ++k) vst1q_f64 (out + 2 * k, s[k]);
  #endif
    }

  #if VAMP_SIMD_NEON
    // 2 P rows of matVecT on arm64: two columns per step, v[j] and v[j + 1] in the lanes of one load (the multiply by
    // lane is the multiply by the broadcast value, so each row's sum is formed exactly as in tileT)
    template <int P>
    static inline void tileTLane (const double* At, int ld, int cols, const double* v, double* out)
    {
        float64x2_t s[P];
        for (int k = 0; k < P; ++k) s[k] = vdupq_n_f64 (0.0);
        int j = 0;
        for (; j + 1 < cols; j += 2, At += 2 * ld)
        {
            const float64x2_t vv = vld1q_f64 (v + j);
            for (int k = 0; k < P; ++k) s[k] = vaddq_f64 (s[k], vmulq_laneq_f64 (vld1q_f64 (At + 2 * k), vv, 0));
            for (int k = 0; k < P; ++k) s[k] = vaddq_f64 (s[k], vmulq_laneq_f64 (vld1q_f64 (At + ld + 2 * k), vv, 1));
        }
        if (j < cols)
        {
            const float64x2_t vj = vdupq_n_f64 (v[j]);
            for (int k = 0; k < P; ++k) s[k] = vaddq_f64 (s[k], vmulq_f64 (vld1q_f64 (At + 2 * k), vj));
        }
        for (int k = 0; k < P; ++k) vst1q_f64 (out + 2 * k, s[k]);
    }
  #endif
#endif

private:
    // row-major A (rows x cols) -> At column by column with leading dimension ld (the pad rows stay 0)
    static void transpose (const std::vector<double>& A, int rows, int cols, std::vector<double>& At, int ld)
    {
        for (int r = 0; r < rows; ++r)
            for (int j = 0; j < cols; ++j)
                At[(size_t) j * ld + r] = A[(size_t) r * cols + j];
    }

    static void stamp (std::vector<double>& A, int n, int a, int b, double g)
    {
        if (a >= 0) A[(size_t) a * n + a] += g;
        if (b >= 0) A[(size_t) b * n + b] += g;
        if (a >= 0 && b >= 0) { A[(size_t) a * n + b] -= g; A[(size_t) b * n + a] -= g; }
    }

    void rebuild()
    {
        VAMP_PROF (prof::kDkRebuild);
        VAMP_PROF_LAPS;
        // all buffers are sized in prepare(): rebuilding does not allocate (audio thread)
        // with variable resistors (markVariable) the base is built with them at their reference values, and their
        // present values are applied by updateVariable() at the end (tests: rebuildDirect() takes them directly)
        const bool useBase = numVar > 0 && ! directOnce && ! testAlwaysDirect;
        auto& A = bufA;
        A.assign ((size_t) N * N, 0.0);
        for (auto& r : resistors)
        {
            if (r.var < 0) { stamp (A, N, r.a, r.b, r.g * testGScale); continue; }
            // a variable resistor in resistance form: its current i (a -> b) is an unknown, with the branch equation
            // v(a) - v(b) - R i = 0. R is one diagonal entry, so a pot's change is a rank-1 update that stays well
            // conditioned at both ends of its travel (a segment at 1 mOhm is R ~ 0, not a conductance of 1e3)
            const int k = var[r.var].row;
            if (r.a >= 0) { A[(size_t) r.a * N + k] += 1.0; A[(size_t) k * N + r.a] += 1.0; }
            if (r.b >= 0) { A[(size_t) r.b * N + k] -= 1.0; A[(size_t) k * N + r.b] -= 1.0; }
            A[(size_t) k * N + k] = -(useBase ? var[r.var].rRef : r.r) / testGScale;
        }
        for (auto& c : caps)      stamp (A, N, c.a, c.b, c.gc);
        for (auto& l : inds)      stamp (A, N, l.a, l.b, l.gl);
        for (auto& s : sources)   A[(size_t) s.node * N + s.node] += s.g;

        int aux = numNodes;
        for (auto& t : transformers)
        {
            const int eIdx = aux + (int) t.size();
            for (size_t w = 0; w < t.size(); ++w)
            {
                const int iw = aux + (int) w;
                const auto& wd = t[w];
                if (wd.a >= 0) { A[(size_t) wd.a * N + iw] += 1.0; A[(size_t) iw * N + wd.a] += 1.0; }
                if (wd.b >= 0) { A[(size_t) wd.b * N + iw] -= 1.0; A[(size_t) iw * N + wd.b] -= 1.0; }
                A[(size_t) iw * N + eIdx] -= wd.turns;    // v(a) - v(b) - n*e = 0
                A[(size_t) eIdx * N + iw] += wd.turns;    // sum n*i = 0
            }
            aux = eIdx + 1;
        }
        VAMP_PROF_LAP (prof::kDkStamp);

        invert (A, bufInv, N);
        VAMP_PROF_LAP (prof::kDkInvert);
        const auto& Ainv = bufInv;

        // Ni: port current a -> b leaves node a (-1) and enters node b (+1); Nv = -Ni^T
        auto& AinvNi = bufAinvNi;
        AinvNi.assign ((size_t) N * m, 0.0);
        for (int r = 0; r < N; ++r)
            for (int k = 0; k < m; ++k)
            {
                const auto& pt = ports[(size_t) k];
                double s = 0.0;
                if (pt.a >= 0) s -= Ainv[(size_t) r * N + pt.a];
                if (pt.b >= 0) s += Ainv[(size_t) r * N + pt.b];
                AinvNi[(size_t) r * m + k] = s;
            }

        K.assign ((size_t) m * m, 0.0);
        for (int k = 0; k < m; ++k)
        {
            const auto& pt = ports[(size_t) k];
            for (int j = 0; j < m; ++j)
            {
                double s = 0.0;
                if (pt.a >= 0) s += AinvNi[(size_t) pt.a * m + j];
                if (pt.b >= 0) s -= AinvNi[(size_t) pt.b * m + j];
                K[(size_t) k * m + j] = s;
            }
        }
        VAMP_PROF_LAP (prof::kDkAinvNi);

        // The MNA right-hand side is b = Bs s + Bu u: a capacitor companion current ieq enters node a
        // and leaves b, an inductor's the other way round, a source drives its node through g.
        // Node voltages are then  v = Ainv Bs s + Ainv Bu u + AinvNi i,  written here as one row
        // per node over z = [s; u; i; noise] (a noise current enters its resistor's node a and leaves b).
        auto& V = bufV;
        V.assign ((size_t) N * nz, 0.0);
        for (int r = 0; r < N; ++r)
        {
            double* row = &V[(size_t) r * nz];
            const double* a = &Ainv[(size_t) r * N];
            int k = 0;
            for (auto& c : caps)
            {
                if (c.a >= 0) row[k] += a[c.a];
                if (c.b >= 0) row[k] -= a[c.b];
                ++k;
            }
            for (auto& l : inds)
            {
                if (l.a >= 0) row[k] -= a[l.a];
                if (l.b >= 0) row[k] += a[l.b];
                ++k;
            }
            for (auto& s : sources)
                row[k++] = a[s.node] * s.g;
            for (int j = 0; j < m; ++j)
                row[k++] = AinvNi[(size_t) r * m + j];
            for (auto& rs : resistors)
                if (rs.noisy)
                    row[k++] = (rs.a >= 0 ? a[rs.a] : 0.0) - (rs.b >= 0 ? a[rs.b] : 0.0);
        }
        VAMP_PROF_LAP (prof::kDkV);
        auto nodeRow = [&] (int n, int j) { return n < 0 ? 0.0 : V[(size_t) n * nz + j]; };

        // p0 = (v(a) - v(b)) of each port with i = 0: over z, the port-current columns left at 0
        const int iBegin = numS + numSrc, iEnd = iBegin + m;
        P.assign ((size_t) m * nz, 0.0);
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < nz; ++j)
                if (j < iBegin || j >= iEnd)
                    P[(size_t) k * nz + j] = nodeRow (ports[(size_t) k].a, j) - nodeRow (ports[(size_t) k].b, j);

        // trapezoidal companions:  cap  ieq' = 2 gc v - ieq,   inductor  ieq' = 2 gl v + ieq
        M.assign ((size_t) numS * nz, 0.0);
        int k = 0;
        for (auto& c : caps)
        {
            for (int j = 0; j < nz; ++j)
                M[(size_t) k * nz + j] = 2.0 * c.gc * (nodeRow (c.a, j) - nodeRow (c.b, j));
            M[(size_t) k * nz + k] -= 1.0;
            ++k;
        }
        for (auto& l : inds)
        {
            for (int j = 0; j < nz; ++j)
                M[(size_t) k * nz + j] = 2.0 * l.gl * (nodeRow (l.a, j) - nodeRow (l.b, j));
            M[(size_t) k * nz + k] += 1.0;
            ++k;
        }

        O.assign (outputs.size() * (size_t) nz, 0.0);
        for (size_t o = 0; o < outputs.size(); ++o)
            for (int j = 0; j < nz; ++j)
                O[o * (size_t) nz + j] = nodeRow (outputs[o], j);

        VAMP_PROF_LAP (prof::kDkPMO);
        transpose (P, m, nz, Pt, ldP);
        transpose (M, numS, nz, Mt, ldM);
        transpose (O, (int) outputs.size(), nz, Ot, ldO);
        VAMP_PROF_LAP (prof::kDkTranspose);

        // thermal noise per sample, white up to Nyquist (one-sided density 4kT R, i.e. variance 2kT R fs), as uniform
        // samples of the same variance: a series voltage for a noisy source, a parallel current for a resistor
        {
            const double fs = 1.0 / T;
            for (int k = 0; k < numSrc; ++k)
                noiseAmp[(size_t) k] = sources[(size_t) k].noisy ? std::sqrt (3.0 * 2.0 * noiseKT * fs / sources[(size_t) k].g) : 0.0;
            int j = numSrc;
            for (auto& rs : resistors)
                if (rs.noisy)
                {
                    if (rs.var >= 0) var[rs.var].noiseSlot = j;
                    noiseAmp[(size_t) j++] = std::sqrt (3.0 * 2.0 * noiseKT * fs * rs.g);
                }
        }

        dirty = false;
        ++kVersion;
        if (numVar == 0) return;
        varDirty = false;
        if (! useBase) { directOnce = false; baseValid = false; return; } // exact for the present values; no base

        // The correction factors. The variable resistors' branch rows (U = their unit vectors) hold -R, so with the
        // change from the reference D = diag (-(R - rRef)), A = A0 + U D U^T and (Woodbury)
        //        Ainv = Ainv0 - W (I + D Z0)^-1 D U^T Ainv0,    W = Ainv0 U,  Z0 = U^T W,
        // and every per-sample matrix is a fixed row selection / scaling of Ainv times a fixed right-hand side
        // (X = L Ainv B), so X = X0 - (L W) G with G = (I + D Z0)^-1 D (U^T V0): rank numVar per matrix.
        {
            const auto& Ainv = bufInv;
            auto w = [&] (int n, int v) { return n < 0 ? 0.0 : Ainv[(size_t) n * N + var[v].row]; }; // W[n][v]
            for (int v = 0; v < numVar; ++v)
            {
                for (int l = 0; l < numVar; ++l) z0[v][l] = w (var[v].row, l);
                for (int j = 0; j < nz; ++j) yV[(size_t) v * nz + j] = nodeRow (var[v].row, j);
                for (int p = 0; p < m; ++p)
                    qP[(size_t) v * ldP + p] = w (ports[(size_t) p].a, v) - w (ports[(size_t) p].b, v);
                int s = 0;
                for (auto& c : caps) { qM[(size_t) v * ldM + s] = 2.0 * c.gc * (w (c.a, v) - w (c.b, v)); ++s; }
                for (auto& l : inds) { qM[(size_t) v * ldM + s] = 2.0 * l.gl * (w (l.a, v) - w (l.b, v)); ++s; }
                for (size_t o = 0; o < outputs.size(); ++o) qO[(size_t) v * ldO + o] = w (outputs[o], v);
            }
            std::copy (K.begin(), K.end(), K0.begin());
            std::copy (Pt.begin(), Pt.end(), Pt0.begin());
            std::copy (Mt.begin(), Mt.end(), Mt0.begin());
            std::copy (O.begin(), O.end(), O0.begin());
            baseValid = true;
        }
        updateVariable();
    }

    // out[0..len) = base - sum over v of q[v][..] g[v], in that order (base may be out); len: the padded column (the pads
    // are 0 in base and q), so the loops run over whole vectors
    static void columnUpdate (double* out, const double* base, const double* q, int len, const double* g, int r)
    {
        for (int p = 0; p < len; ++p) out[p] = base[p];
        for (int v = 0; v < r; ++v)
        {
            const double gv = g[v];
            const double* qv = q + (size_t) v * len;
            for (int p = 0; p < len; ++p) out[p] -= qv[p] * gv;
        }
    }
    static constexpr int kMaxColumn = 64; // ports (padded) in one column of P

    // The variable resistors' present values applied to the base (see rebuild): G from the numVar x numVar system,
    // then X = X0 - Q G for K, P, M and O. O(numVar nz (ports + states + outputs)) instead of the rebuild's O(N^3).
    void updateVariable()
    {
        if (testAlwaysDirect) { rebuildDirect(); return; }
        if (! baseValid) { rebuild(); return; }
        VAMP_PROF (prof::kDkUpdate);
        const int r = numVar, cols = noiseOn ? nz : nzMain; // (noise columns: see below)
        double D[kMaxVar], S[kMaxVar][kMaxVar];
        for (int v = 0; v < r; ++v) D[v] = var[v].rRef - resistors[(size_t) var[v].res].r; // -(R - rRef)
        for (int v = 0; v < r; ++v)
        {
            double sc = 0.0;
            for (int l = 0; l < r; ++l) { S[v][l] = (v == l ? 1.0 : 0.0) + D[v] * z0[v][l]; sc = std::max (sc, std::abs (S[v][l])); }
            const double inv = 1.0 / sc; // rows equilibrated: a shorted pot's row is ~1e9 times the others
            for (int l = 0; l < r; ++l) S[v][l] *= inv;
            for (int j = 0; j < cols; ++j) gV[(size_t) v * nz + j] = D[v] * inv * yV[(size_t) v * nz + j];
        }
        // (I + D Z0) G = D Y: Gaussian elimination with partial pivoting, then back substitution
        for (int c = 0; c < r; ++c)
        {
            int piv = c;
            for (int v = c + 1; v < r; ++v) if (std::abs (S[v][c]) > std::abs (S[piv][c])) piv = v;
            if (piv != c)
            {
                for (int l = 0; l < r; ++l) std::swap (S[c][l], S[piv][l]);
                std::swap_ranges (gV.begin() + (std::ptrdiff_t) c * nz, gV.begin() + (std::ptrdiff_t) (c + 1) * nz, gV.begin() + (std::ptrdiff_t) piv * nz);
            }
            if (! (std::abs (S[c][c]) > 0.0) || ! std::isfinite (S[c][c])) { rebuildDirect(); return; } // (cannot happen for a solvable circuit)
            for (int v = c + 1; v < r; ++v)
            {
                const double f = S[v][c] / S[c][c];
                if (f == 0.0) continue;
                for (int l = c; l < r; ++l) S[v][l] -= f * S[c][l];
                for (int j = 0; j < cols; ++j) gV[(size_t) v * nz + j] -= f * gV[(size_t) c * nz + j];
            }
        }
        for (int c = r - 1; c >= 0; --c)
        {
            double* gc = &gV[(size_t) c * nz];
            for (int l = c + 1; l < r; ++l)
            {
                const double s = S[c][l];
                const double* gl = &gV[(size_t) l * nz];
                for (int j = 0; j < cols; ++j) gc[j] -= s * gl[j];
            }
            const double inv = 1.0 / S[c][c];
            for (int j = 0; j < cols; ++j) gc[j] *= inv;
        }

        // X = X0 - Q G, column by column of z (Pt / Mt / Ot are stored that way; P's port-current columns are K). With
        // noise off the noise columns are not used (computeP0 / finish stop before them) and stay at the base: turning
        // noise on rebuilds (setNoise)
        const int iBegin = numS + numSrc, iEnd = iBegin + m, numO = (int) outputs.size();
        assert (ldP <= kMaxColumn);
        double col[kMaxColumn];
        for (int j = 0; j < cols; ++j)
        {
            double gj[kMaxVar];
            for (int v = 0; v < r; ++v) gj[v] = gV[(size_t) v * nz + j];
            if (j < iBegin || j >= iEnd)
                columnUpdate (&Pt[(size_t) j * ldP], &Pt0[(size_t) j * ldP], qP.data(), ldP, gj, r);
            else
            {
                const int c = j - iBegin;
                for (int p = 0; p < m; ++p) col[p] = K0[(size_t) p * m + c];
                for (int p = m; p < ldP; ++p) col[p] = 0.0;
                columnUpdate (col, col, qP.data(), ldP, gj, r);
                for (int p = 0; p < m; ++p) K[(size_t) p * m + c] = col[p];
            }
            columnUpdate (&Mt[(size_t) j * ldM], &Mt0[(size_t) j * ldM], qM.data(), ldM, gj, r);
            for (int o = 0; o < numO; ++o)
            {
                double t = O0[(size_t) o * nz + j];
                for (int v = 0; v < r; ++v) t -= qO[(size_t) v * ldO + o] * gj[v];
                O[(size_t) o * nz + j] = t;
                Ot[(size_t) j * ldO + o] = t;
            }
        }

        const double fs = 1.0 / T;
        for (int v = 0; v < r; ++v)
            if (var[v].noiseSlot >= 0)
                noiseAmp[(size_t) var[v].noiseSlot] = std::sqrt (3.0 * 2.0 * noiseKT * fs * resistors[(size_t) var[v].res].g);
        varDirty = false;
        ++kVersion;
    }

    // Gauss-Jordan with partial pivoting: A (destroyed) -> I = A^-1, in place
    static void invert (std::vector<double>& A, std::vector<double>& I, int n)
    {
        I.assign ((size_t) n * n, 0.0);
        for (int i = 0; i < n; ++i) I[(size_t) i * n + i] = 1.0;

        for (int col = 0; col < n; ++col)
        {
            int piv = col;
            for (int r = col + 1; r < n; ++r)
                if (std::abs (A[(size_t) r * n + col]) > std::abs (A[(size_t) piv * n + col])) piv = r;
            if (piv != col)
                for (int j = 0; j < n; ++j)
                {
                    std::swap (A[(size_t) col * n + j], A[(size_t) piv * n + j]);
                    std::swap (I[(size_t) col * n + j], I[(size_t) piv * n + j]);
                }
            const double d = A[(size_t) col * n + col];
            for (int j = 0; j < n; ++j) { A[(size_t) col * n + j] /= d; I[(size_t) col * n + j] /= d; }
            for (int r = 0; r < n; ++r)
            {
                if (r == col) continue;
                const double f = A[(size_t) r * n + col];
                if (f == 0.0) continue;
                for (int j = 0; j < n; ++j)
                {
                    A[(size_t) r * n + j] -= f * A[(size_t) col * n + j];
                    I[(size_t) r * n + j] -= f * I[(size_t) col * n + j];
                }
            }
        }

    }

    static constexpr int kMaxStates = 128; // capacitors + inductors, and output nodes (finish() keeps them on the stack)

    double T = 1.0 / 192000.0;
    int numNodes = 0, numAux = 0, N = 0, m = 0, numS = 0, numSrc = 0, numNoise = 0, nzMain = 0, nz = 0;
    bool dirty = true;
    unsigned kVersion = 0; // counts rebuilds: K (and the rest) changed

    static constexpr double kBoltzmann = 1.380649e-23;
    bool noiseOn = false;
    double noiseKT = kBoltzmann * 300.0;
    unsigned long long rng = 0x9e3779b97f4a7c15ull;
    std::vector<double> noiseAmp; // per input: thermal noise amplitude (uniform, peak) of a noisy source / resistor

    std::vector<Res> resistors;
    std::vector<Cap> caps;
    std::vector<Ind> inds;
    std::vector<Src> sources;
    std::vector<std::vector<Winding>> transformers;
    std::vector<Port> ports;
    std::vector<int> outputs;

    std::vector<double> K, P, M, O;
    std::vector<double> Pt, Mt, Ot; // P, M, O column by column (rows padded to ldP / ldM / ldO): the per-sample products
    int ldP = 0, ldM = 0, ldO = 0;
    std::vector<double> z;     // [s; u; i]: reactive states, then the inputs and port currents of this sample
    std::vector<double> x, p0; // x: output node voltages (others stay 0)
    std::vector<double> zSaved, xSaved, zTrial;
    std::vector<double> bufA, bufInv, bufAinvNi, bufV; // rebuild work buffers

    // variable resistors (markVariable): the base matrices at the reference values and the correction factors
    Var var[kMaxVar];
    int numVar = 0;
    bool varDirty = false, baseValid = false, directOnce = false;
    double z0[kMaxVar][kMaxVar] {};
    std::vector<double> K0, Pt0, Mt0, O0; // the per-sample matrices at the reference values
    std::vector<double> qP, qM, qO;       // rows of W for the ports, states, outputs (numVar rows, padded as Pt / Mt / Ot)
    std::vector<double> yV, gV;           // U^T V0 and the update G (numVar x nz)
};

} // namespace vamp
