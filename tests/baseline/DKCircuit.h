#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace vamp_baseline
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

    int addR (int a, int b, double r)
    {
        resistors.push_back ({ a, b, 1.0 / std::max (r, 1e-3) });
        dirty = true;
        return (int) resistors.size() - 1;
    }

    void setR (int idx, double r)
    {
        const double g = 1.0 / std::max (r, 1e-3);
        if (resistors[(size_t) idx].g != g) { resistors[(size_t) idx].g = g; dirty = true; }
    }

    int addC (int a, int b, double c) { caps.push_back ({ a, b, c, 0 }); dirty = true; return (int) caps.size() - 1; }
    int addL (int a, int b, double l) { inds.push_back ({ a, b, l, 0 }); dirty = true; return (int) inds.size() - 1; }

    // change a capacitor / inductor (addC / addL index) after prepare(): the matrices are rebuilt on the next sample
    void setC (int idx, double c) { auto& e = caps[(size_t) idx]; e.c = c; e.gc = 2.0 * c / T; dirty = true; }
    void setL (int idx, double l) { auto& e = inds[(size_t) idx]; e.l = l; e.gl = T / (2.0 * l); dirty = true; }

    // node --[rs]-- u[k]; returns the input index k
    int addSource (int node, double rs)
    {
        sources.push_back ({ node, 1.0 / rs });
        dirty = true;
        return (int) sources.size() - 1;
    }

    void setSourceResistance (int k, double rs)
    {
        sources[(size_t) k].g = 1.0 / rs;
        dirty = true;
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

    void prepare (double sampleRate)
    {
        T = 1.0 / sampleRate;
        for (auto& c : caps) c.gc = 2.0 * c.c / T;
        for (auto& l : inds) l.gl = T / (2.0 * l.l);
        // aux unknowns: one current per winding + one "volts per turn" per transformer
        numAux = 0;
        for (auto& t : transformers) numAux += (int) t.size() + 1;
        N = numNodes + numAux;
        m = (int) ports.size();
        numS = (int) (caps.size() + inds.size());
        numU = (int) sources.size();
        assert (numS <= kMaxStates && (int) outputs.size() <= kMaxStates); // finish() keeps both on the stack
        x.assign ((size_t) N, 0.0);
        z.assign ((size_t) (numS + numU + m), 0.0);
        p0.assign ((size_t) m, 0.0);
        // rebuild() reuses these (assign to the same size does not reallocate)
        const size_t nz = (size_t) (numS + numU + m);
        bufA.assign ((size_t) N * N, 0.0);
        bufInv.assign ((size_t) N * N, 0.0);
        bufAinvNi.assign ((size_t) N * m, 0.0);
        bufV.assign ((size_t) N * nz, 0.0);
        K.assign ((size_t) m * m, 0.0);
        P.assign ((size_t) m * (numS + numU), 0.0);
        M.assign ((size_t) numS * nz, 0.0);
        O.assign (outputs.size() * nz, 0.0);
        dirty = true;
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

    // Remember / go back to a state (reactive states and output nodes), e.g. the settled idle point.
    // saveState() allocates (call it from prepare); restoreState() does not.
    void saveState() { zSaved = z; xSaved = x; }
    void restoreState()
    {
        if (zSaved.size() != z.size()) { reset(); return; }
        std::copy (zSaved.begin(), zSaved.end(), z.begin());
        std::copy (xSaved.begin(), xSaved.end(), x.begin());
    }

    // p0 = P [s; u] for the source values u and the current reactive states
    const std::vector<double>& computeP0 (const double* u)
    {
        if (dirty) rebuild();

        std::copy (u, u + numU, z.begin() + numS);
        matVec (P.data(), m, numS + numU, z.data(), p0.data());
        return p0;
    }

    // K (m x m, row-major): sensitivity of the port voltages to the port currents
    const std::vector<double>& getK() const { return K; }

    // Given the converged port currents, compute the output nodes and advance the states
    // (uses the u passed to the preceding computeP0)
    void finish (const double* iPorts)
    {
        std::copy (iPorts, iPorts + m, z.begin() + numS + numU);
        const int nz = numS + numU + m;

        double buf[kMaxStates];
        const int numO = (int) outputs.size();
        matVec (O.data(), numO, nz, z.data(), buf);
        for (int o = 0; o < numO; ++o) x[(size_t) outputs[(size_t) o]] = buf[o];

        matVec (M.data(), numS, nz, z.data(), buf);
        std::copy (buf, buf + numS, z.begin());
    }

    // Only nodes marked as outputs are computed in finish(); nodeV() of any other node is 0.
    double nodeV (int n) const { return n < 0 ? 0.0 : x[(size_t) n]; }
    void markOutput (int node) { outputs.push_back (node); dirty = true; }

    void markDirty() { dirty = true; }

private:
    struct Res { int a, b; double g; };
    struct Cap { int a, b; double c, gc; };
    struct Ind { int a, b; double l, gl; };
    struct Src { int node; double g; };
    struct Port { int a, b; };

public:
    // out = A v (A rows x cols, row-major). Four rows at a time: four independent accumulation chains instead of
    // one, so the adds overlap instead of waiting on each other. Each row is still summed in index order, so the
    // result is bit-identical to the plain loop.
    static void matVec (const double* A, int rows, int cols, const double* v, double* out)
    {
        int r = 0;
        for (; r + 4 <= rows; r += 4)
        {
            const double* a0 = A + (size_t) r * cols;
            const double* a1 = a0 + cols;
            const double* a2 = a1 + cols;
            const double* a3 = a2 + cols;
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
            const double* a = A + (size_t) r * cols;
            double s = 0.0;
            for (int j = 0; j < cols; ++j) s += a[j] * v[j];
            out[r] = s;
        }
    }

private:
    static void stamp (std::vector<double>& A, int n, int a, int b, double g)
    {
        if (a >= 0) A[(size_t) a * n + a] += g;
        if (b >= 0) A[(size_t) b * n + b] += g;
        if (a >= 0 && b >= 0) { A[(size_t) a * n + b] -= g; A[(size_t) b * n + a] -= g; }
    }

    void rebuild()
    {
        // all buffers are sized in prepare(): rebuilding when a pot moves does not allocate (audio thread)
        auto& A = bufA;
        A.assign ((size_t) N * N, 0.0);
        for (auto& r : resistors) stamp (A, N, r.a, r.b, r.g);
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

        invert (A, bufInv, N);
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

        // The MNA right-hand side is b = Bs s + Bu u: a capacitor companion current ieq enters node a
        // and leaves b, an inductor's the other way round, a source drives its node through g.
        // Node voltages are then  v = Ainv Bs s + Ainv Bu u + AinvNi i,  written here as one row
        // per node over z = [s; u; i].
        const int nz = numS + numU + m;
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
        }
        auto nodeRow = [&] (int n, int j) { return n < 0 ? 0.0 : V[(size_t) n * nz + j]; };

        // p0 = (v(a) - v(b)) of each port with i = 0, over [s; u]
        const int nsu = numS + numU;
        P.assign ((size_t) m * nsu, 0.0);
        for (int k = 0; k < m; ++k)
            for (int j = 0; j < nsu; ++j)
                P[(size_t) k * nsu + j] = nodeRow (ports[(size_t) k].a, j) - nodeRow (ports[(size_t) k].b, j);

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

        dirty = false;
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

    static constexpr int kMaxStates = 64; // capacitors + inductors, and output nodes (finish() keeps them on the stack)

    double T = 1.0 / 192000.0;
    int numNodes = 0, numAux = 0, N = 0, m = 0, numS = 0, numU = 0;
    bool dirty = true;

    std::vector<Res> resistors;
    std::vector<Cap> caps;
    std::vector<Ind> inds;
    std::vector<Src> sources;
    std::vector<std::vector<Winding>> transformers;
    std::vector<Port> ports;
    std::vector<int> outputs;

    std::vector<double> K, P, M, O;
    std::vector<double> z;     // [s; u; i]: reactive states, then the inputs and port currents of this sample
    std::vector<double> x, p0; // x: output node voltages (others stay 0)
    std::vector<double> zSaved, xSaved;
    std::vector<double> bufA, bufInv, bufAinvNi, bufV; // rebuild work buffers
};

} // namespace vamp_baseline
