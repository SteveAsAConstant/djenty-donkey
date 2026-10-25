#pragma once
#include <algorithm>
#include <cmath>

namespace vamp
{
// A set of real tubes instead of the nominal model: each position gets its own plate-current scale (current and gm
// together: Koren's kg1) and, for the 12AX7s, amplification factor, drawn from production spreads. Set 0 is the
// nominal model ("Stock"); sets 1..kSets-1 are fixed draws (a preset recalls the same tubes).
//   12AX7: current sigma 12 %, mu sigma 4 % (+-25 % / +-8 % at 2 sigma, inside the datasheets' tolerances)
//   6L6GC: current sigma 6 % per tube (a fairly matched quad); each side of the push-pull is the average of its two
// Draws are clipped at 2.5 sigma. Passive parts keep their nominal values.
struct TubeSet
{
    static constexpr int kSets = 6;
    static constexpr const char* kNames[kSets] = { "Stock", "Set A", "Set B", "Set C", "Set D", "Set E" };

    double preI[6] { 1, 1, 1, 1, 1, 1 }, preMu[6] { 1, 1, 1, 1, 1, 1 }; // V1A V1B V2A V2B V5B V5A
    double paI[3] { 1, 1, 1 }, paMu[3] { 1, 1, 1 };                     // V4B V4A V3A
    double sideI[2] { 1, 1 };                                            // 6L6 pairs (V5 + V7, V6 + V8)

    static TubeSet make (int set)
    {
        TubeSet s;
        if (set <= 0) return s;
        unsigned long long rng = 0x9e3779b97f4a7c15ull * (unsigned long long) (set + 1) | 1ull;
        auto uniform = [&rng]
        {
            rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
            return ((double) ((rng * 0x2545F4914F6CDD1Dull) >> 11) + 0.5) / 9007199254740992.0; // (0, 1)
        };
        auto gauss = [&] (double sigma) // clipped at 2.5 sigma
        {
            const double g = std::sqrt (-2.0 * std::log (uniform())) * std::cos (6.283185307179586 * uniform());
            return 1.0 + sigma * std::clamp (g, -2.5, 2.5);
        };
        for (int t = 0; t < 6; ++t) { s.preI[t] = gauss (0.12); s.preMu[t] = gauss (0.04); }
        for (int t = 0; t < 3; ++t) { s.paI[t] = gauss (0.12); s.paMu[t] = gauss (0.04); }
        for (int t = 0; t < 2; ++t) s.sideI[t] = 0.5 * (gauss (0.06) + gauss (0.06));
        return s;
    }
};
} // namespace vamp
