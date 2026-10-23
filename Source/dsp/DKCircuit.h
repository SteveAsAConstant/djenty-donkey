#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

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

    void addC (int a, int b, double c) { caps.push_back ({ a, b, c, 0 }); dirty = true; }
    void addL (int a, int b, double l) { inds.push_back ({ a, b, l, 0 }); dirty = true; }

    // node --[rs]-- u[k]; returns the input index k
    int addSource (int node, double rs)
    {
        sources.push_back ({ node, 1.0 / rs });
        dirty = true;
        return (int) sources.size() - 1;
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
        assert (numS <= kMaxStates);
        x.assign ((size_t) N, 0.0);
        z.assign ((size_t) (numS + numU + m), 0.0);
        p0.assign ((size_t) m, 0.0);
        dirty = true;
        reset();
    }

    void reset()
    {
        std::fill (z.begin(), z.end(), 0.0);
        std::fill (x.begin(), x.end(), 0.0);
    }

    // p0 = P [s; u] for the source values u and the current reactive states
    const std::vector<double>& computeP0 (const double* u)
    {
        if (dirty) rebuild();

        std::copy (u, u + numU, z.begin() + numS);
        const int nz = numS + numU;
        for (int r = 0; r < m; ++r)
        {
            const double* row = &P[(size_t) r * nz];
            double s = 0.0;
            for (int j = 0; j < nz; ++j) s += row[j] * z[(size_t) j];
            p0[(size_t) r] = s;
        }
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

        for (size_t o = 0; o < outputs.size(); ++o)
        {
            const double* row = &O[o * (size_t) nz];
            double s = 0.0;
            for (int j = 0; j < nz; ++j) s += row[j] * z[(size_t) j];
            x[(size_t) outputs[o]] = s;
        }

        double sNew[kMaxStates];
        for (int r = 0; r < numS; ++r)
        {
            const double* row = &M[(size_t) r * nz];
            double s = 0.0;
            for (int j = 0; j < nz; ++j) s += row[j] * z[(size_t) j];
            sNew[r] = s;
        }
        std::copy (sNew, sNew + numS, z.begin());
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

    static void stamp (std::vector<double>& A, int n, int a, int b, double g)
    {
        if (a >= 0) A[(size_t) a * n + a] += g;
        if (b >= 0) A[(size_t) b * n + b] += g;
        if (a >= 0 && b >= 0) { A[(size_t) a * n + b] -= g; A[(size_t) b * n + a] -= g; }
    }

    void rebuild()
    {
        std::vector<double> A ((size_t) N * N, 0.0);
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

        const auto Ainv = invert (A, N);

        // Ni: port current a -> b leaves node a (-1) and enters node b (+1); Nv = -Ni^T
        std::vector<double> AinvNi ((size_t) N * m, 0.0);
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
        std::vector<double> V ((size_t) N * nz, 0.0);
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

    static std::vector<double> invert (std::vector<double> A, int n)
    {
        std::vector<double> I ((size_t) n * n, 0.0);
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
        return I;
    }

    static constexpr int kMaxStates = 64; // capacitors + inductors (finish() keeps s' on the stack)

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
};

} // namespace vamp
