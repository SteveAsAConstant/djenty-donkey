#pragma once
#include <array>
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
        caps.push_back ({ a, b, c, 0.0, 0.0 });
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

    void reset()
    {
        for (auto& c : caps) c.ieq = 0.0;
        v.fill (0.0);
    }

    double process (double input, int outNode)
    {
        if (dirty)
            build();

        std::array<double, N> rhs {};
        for (int i = 0; i < N; ++i)
            rhs[(size_t) i] = srcCoef[(size_t) i] * input;

        for (auto& c : caps)
        {
            if (c.a >= 0) rhs[(size_t) c.a] += c.ieq;
            if (c.b >= 0) rhs[(size_t) c.b] -= c.ieq;
        }

        for (int i = 0; i < N; ++i)
        {
            double s = 0.0;
            for (int j = 0; j < N; ++j)
                s += inv[(size_t) i][(size_t) j] * rhs[(size_t) j];
            v[(size_t) i] = s;
        }

        for (auto& c : caps)
        {
            const double vab = nodeV (c.a, input) - nodeV (c.b, input);
            const double i = c.gc * vab - c.ieq;
            c.ieq = c.gc * vab + i;
        }

        return v[(size_t) outNode];
    }

    double getNodeVoltage (int node) const { return v[(size_t) node]; }

private:
    struct Res { int a, b; double g; };
    struct Cap { int a, b; double c, gc, ieq; };

    double nodeV (int n, double input) const
    {
        if (n == GND) return 0.0;
        if (n == SRC) return input;
        return v[(size_t) n];
    }

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

        inv = I;
        dirty = false;
    }

    std::vector<Res> resistors;
    std::vector<Cap> caps;
    std::array<std::array<double, N>, N> inv {};
    std::array<double, N> srcCoef {};
    std::array<double, N> v {};
    bool dirty = true;
};

} // namespace vamp
