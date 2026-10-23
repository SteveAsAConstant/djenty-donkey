#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <vector>

namespace vamp
{
constexpr int GND = -1; // ground node
constexpr int SRC = -2; // ideal input voltage source node

// Small linear R/C circuit solved by modified nodal analysis.
// Capacitors use trapezoidal companion models, so the network is exactly the
// bilinear-transform discretisation of the analog circuit.
// N = number of unknown nodes (0..N-1). Rebuild is cheap, so pots can change per block.
//
// Per sample it runs in state-space form over z = [s; input] (s = capacitor companion currents):
// the output node is one row times z and the states are s' = M z, both precomputed at build.
template <int N>
class LinearNetwork
{
public:
    int addR (int a, int b, double r)
    {
        resistors.push_back ({ a, b, 1.0 / std::max (r, 1.0) });
        dirty = true;
        return (int) resistors.size() - 1;
    }

    void setR (int index, double r)
    {
        const double g = 1.0 / std::max (r, 1.0);
        if (resistors[(size_t) index].g != g)
        {
            resistors[(size_t) index].g = g;
            dirty = true;
        }
    }

    int addC (int a, int b, double c)
    {
        caps.push_back ({ a, b, c, 0.0 });
        dirty = true;
        return (int) caps.size() - 1;
    }

    void prepare (double sampleRate)
    {
        const double T = 1.0 / sampleRate;
        for (auto& c : caps)
            c.gc = 2.0 * c.c / T;
        dirty = true;
        reset();
    }

    void reset() { z.fill (0.0); }

    double process (double input, int outNode)
    {
        if (dirty)
            build();

        const int nz = numS + 1;
        z[(size_t) numS] = input;

        const double* r = rows[(size_t) outNode].data();
        double y = 0.0;
        for (int j = 0; j < nz; ++j) y += r[j] * z[(size_t) j];

        std::array<double, kMaxCaps> sNew;
        for (int c = 0; c < numS; ++c)
        {
            const double* m = M[(size_t) c].data();
            double s = 0.0;
            for (int j = 0; j < nz; ++j) s += m[j] * z[(size_t) j];
            sNew[(size_t) c] = s;
        }
        std::copy (sNew.begin(), sNew.begin() + numS, z.begin());

        return y;
    }

private:
    static constexpr int kMaxCaps = 8;

    struct Res { int a, b; double g; };
    struct Cap { int a, b; double c, gc; };

    void stamp (std::array<std::array<double, N>, N>& G, int a, int b, double g)
    {
        if (a >= 0) G[(size_t) a][(size_t) a] += g;
        if (b >= 0) G[(size_t) b][(size_t) b] += g;
        if (a >= 0 && b >= 0)
        {
            G[(size_t) a][(size_t) b] -= g;
            G[(size_t) b][(size_t) a] -= g;
        }
        if (a >= 0 && b == SRC) srcCoef[(size_t) a] += g;
        if (b >= 0 && a == SRC) srcCoef[(size_t) b] += g;
    }

    void build()
    {
        std::array<std::array<double, N>, N> G {};
        srcCoef.fill (0.0);

        for (auto& r : resistors) stamp (G, r.a, r.b, r.g);
        for (auto& c : caps)      stamp (G, c.a, c.b, c.gc);

        // Gauss-Jordan inversion with partial pivoting
        std::array<std::array<double, N>, N> I {};
        for (int i = 0; i < N; ++i) I[(size_t) i][(size_t) i] = 1.0;

        for (int col = 0; col < N; ++col)
        {
            int pivot = col;
            for (int r = col + 1; r < N; ++r)
                if (std::abs (G[(size_t) r][(size_t) col]) > std::abs (G[(size_t) pivot][(size_t) col]))
                    pivot = r;
            std::swap (G[(size_t) col], G[(size_t) pivot]);
            std::swap (I[(size_t) col], I[(size_t) pivot]);

            const double d = G[(size_t) col][(size_t) col];
            for (int j = 0; j < N; ++j) { G[(size_t) col][(size_t) j] /= d; I[(size_t) col][(size_t) j] /= d; }

            for (int r = 0; r < N; ++r)
            {
                if (r == col) continue;
                const double f = G[(size_t) r][(size_t) col];
                if (f == 0.0) continue;
                for (int j = 0; j < N; ++j)
                {
                    G[(size_t) r][(size_t) j] -= f * G[(size_t) col][(size_t) j];
                    I[(size_t) r][(size_t) j] -= f * I[(size_t) col][(size_t) j];
                }
            }
        }

        // Node voltages v = I (srcCoef input + B s), where a capacitor's companion current enters
        // node a and leaves node b: one row per node over z = [s; input]
        assert ((int) caps.size() <= kMaxCaps);
        numS = std::min ((int) caps.size(), kMaxCaps);
        const int nz = numS + 1;
        for (int n = 0; n < N; ++n)
        {
            auto& row = rows[(size_t) n];
            row.fill (0.0);
            for (int c = 0; c < numS; ++c)
            {
                if (caps[(size_t) c].a >= 0) row[(size_t) c] += I[(size_t) n][(size_t) caps[(size_t) c].a];
                if (caps[(size_t) c].b >= 0) row[(size_t) c] -= I[(size_t) n][(size_t) caps[(size_t) c].b];
            }
            for (int j = 0; j < N; ++j)
                row[(size_t) numS] += I[(size_t) n][(size_t) j] * srcCoef[(size_t) j];
        }
        auto nodeRow = [&] (int n, int j)
        {
            if (n == GND) return 0.0;
            if (n == SRC) return j == numS ? 1.0 : 0.0;
            return rows[(size_t) n][(size_t) j];
        };

        // trapezoidal companion:  ieq' = 2 gc (v(a) - v(b)) - ieq
        for (int c = 0; c < numS; ++c)
        {
            const auto& cap = caps[(size_t) c];
            for (int j = 0; j < nz; ++j)
                M[(size_t) c][(size_t) j] = 2.0 * cap.gc * (nodeRow (cap.a, j) - nodeRow (cap.b, j));
            M[(size_t) c][(size_t) c] -= 1.0;
        }
        dirty = false;
    }

    std::vector<Res> resistors;
    std::vector<Cap> caps;
    std::array<double, N> srcCoef {};
    int numS = 0;
    std::array<std::array<double, kMaxCaps + 1>, N> rows {};
    std::array<std::array<double, kMaxCaps + 1>, kMaxCaps> M {};
    std::array<double, kMaxCaps + 1> z {}; // [s; input]
    bool dirty = true;
};

} // namespace vamp
