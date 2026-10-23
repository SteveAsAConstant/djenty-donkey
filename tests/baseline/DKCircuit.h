#pragma once
#include <algorithm>
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
// with Newton's method, then calls finish(i) to update all node voltages and reactive states.
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

    void addC (int a, int b, double c) { caps.push_back ({ a, b, c, 0, 0 }); dirty = true; }
    void addL (int a, int b, double l) { inds.push_back ({ a, b, l, 0, 0 }); dirty = true; }

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
        x.assign ((size_t) N, 0.0);
        bvec.assign ((size_t) N, 0.0);
        p0.assign ((size_t) m, 0.0);
        dirty = true;
        reset();
    }

    void reset()
    {
        for (auto& c : caps) c.ieq = 0;
        for (auto& l : inds) l.ieq = 0;
        std::fill (x.begin(), x.end(), 0.0);
    }

    // Build b from the source values u and the reactive histories, return p0 = E * b
    const std::vector<double>& computeP0 (const double* u)
    {
        if (dirty) rebuild();

        std::fill (bvec.begin(), bvec.end(), 0.0);
        for (size_t k = 0; k < sources.size(); ++k)
            bvec[(size_t) sources[k].node] += sources[k].g * u[k];
        for (auto& c : caps)
        {
            if (c.a >= 0) bvec[(size_t) c.a] += c.ieq;
            if (c.b >= 0) bvec[(size_t) c.b] -= c.ieq;
        }
        for (auto& l : inds)
        {
            if (l.a >= 0) bvec[(size_t) l.a] -= l.ieq;
            if (l.b >= 0) bvec[(size_t) l.b] += l.ieq;
        }

        for (int r = 0; r < m; ++r)
        {
            const double* e = &E[(size_t) r * N];
            double s = 0.0;
            for (int j = 0; j < N; ++j) s += e[j] * bvec[(size_t) j];
            p0[(size_t) r] = s;
        }
        return p0;
    }

    // K (m x m, row-major): sensitivity of the port voltages to the port currents
    const std::vector<double>& getK() const { return K; }

    // Given the converged port currents, compute all unknowns and advance the states
    void finish (const double* iPorts)
    {
        for (int r : neededRows)
        {
            const double* a = &Ainv[(size_t) r * N];
            double s = 0.0;
            for (int j = 0; j < N; ++j) s += a[j] * bvec[(size_t) j];
            const double* f = &AinvNi[(size_t) r * m];
            for (int j = 0; j < m; ++j) s += f[j] * iPorts[j];
            x[(size_t) r] = s;
        }

        for (auto& c : caps)
        {
            const double v = nodeV (c.a) - nodeV (c.b);
            const double i = c.gc * v - c.ieq;
            c.ieq = c.gc * v + i;
        }
        for (auto& l : inds)
        {
            const double v = nodeV (l.a) - nodeV (l.b);
            const double i = l.gl * v + l.ieq;
            l.ieq = i + l.gl * v;
        }
    }

    // Only nodes touching a capacitor / inductor (state updates) or marked as outputs are
    // computed in finish(); nodeV() of any other node is not kept up to date.
    double nodeV (int n) const { return n < 0 ? 0.0 : x[(size_t) n]; }
    void markOutput (int node) { outputs.push_back (node); dirty = true; }

    void markDirty() { dirty = true; }

private:
    struct Res { int a, b; double g; };
    struct Cap { int a, b; double c, gc, ieq; };
    struct Ind { int a, b; double l, gl, ieq; };
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

        Ainv = invert (A, N);

        std::vector<bool> need ((size_t) N, false);
        auto mark = [&need] (int n) { if (n >= 0) need[(size_t) n] = true; };
        for (auto& c : caps) { mark (c.a); mark (c.b); }
        for (auto& l : inds) { mark (l.a); mark (l.b); }
        for (int o : outputs) mark (o);
        neededRows.clear();
        for (int r = 0; r < N; ++r)
            if (need[(size_t) r]) neededRows.push_back (r);

        // Ni: port current a -> b leaves node a (-1) and enters node b (+1); Nv = -Ni^T
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

        E.assign ((size_t) m * N, 0.0);
        K.assign ((size_t) m * m, 0.0);
        for (int k = 0; k < m; ++k)
        {
            const auto& pt = ports[(size_t) k];
            for (int j = 0; j < N; ++j)
            {
                double s = 0.0;
                if (pt.a >= 0) s += Ainv[(size_t) pt.a * N + j];
                if (pt.b >= 0) s -= Ainv[(size_t) pt.b * N + j];
                E[(size_t) k * N + j] = s;
            }
            for (int j = 0; j < m; ++j)
            {
                double s = 0.0;
                if (pt.a >= 0) s += AinvNi[(size_t) pt.a * m + j];
                if (pt.b >= 0) s -= AinvNi[(size_t) pt.b * m + j];
                K[(size_t) k * m + j] = s;
            }
        }
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

    double T = 1.0 / 192000.0;
    int numNodes = 0, numAux = 0, N = 0, m = 0;
    bool dirty = true;

    std::vector<Res> resistors;
    std::vector<Cap> caps;
    std::vector<Ind> inds;
    std::vector<Src> sources;
    std::vector<std::vector<Winding>> transformers;
    std::vector<Port> ports;
    std::vector<int> outputs, neededRows;

    std::vector<double> Ainv, AinvNi, E, K;
    std::vector<double> x, bvec, p0;
};

} // namespace vamp_baseline
