#pragma once
#include <cmath>

namespace vamp
{
// Noise of one triode, referred to its grid (a voltage in series with the grid, seen by the plate current).
//   shot noise: equivalent noise resistance Req = 2.5 / gm (textbook value for a triode), white
//   flicker:    1/f, corner kFlickerCorner (density 4kT Req fc / f)
// 4kT Req (1 + fc / f) gives ~1.2 uV over 25 Hz..10 kHz for a 12AX7 at 1.25 mA/V, within the 12AX7A / 7025
// "equivalent noise and hum voltage referenced to grid" of 1.8 uV average (hum included). The corner is the one
// assumption: tubes differ several-fold in flicker noise.
// The 1/f part: independent white noises through one-pole low-passes at half-decade corners from 5 Hz up, each
// with a variance ~ 1 / corner (1/f within ~+-0.3 dB over 20 Hz..20 kHz), scaled to the target at 1 kHz.
class TubeNoise
{
public:
    static constexpr double kFlickerCorner = 5000.0, kKelvin = 300.0;

    void prepare (double sampleRate, double gm, unsigned long long seed)
    {
        constexpr double kB = 1.380649e-23, twoPi = 6.283185307179586;
        rng = seed | 1ull;
        const double req = 2.5 / std::max (gm, 1e-5), fourKTR = 4.0 * kB * kKelvin * req;
        white = std::sqrt (3.0 * fourKTR * sampleRate / 2.0); // uniform amplitude for variance 4kT Req fs / 2
        // one-sided density of the bank at f for unit per-pole input variances w_k: sum 2 w_k / fs |H_k|^2
        auto bankDensity = [&] (double f)
        {
            double s = 0.0;
            for (int k = 0; k < kPoles; ++k)
            {
                const double a = pole[k], w = twoPi * f / sampleRate;
                const double h2 = (1.0 - a) * (1.0 - a) / (1.0 + a * a - 2.0 * a * std::cos (w));
                s += 2.0 * weight[k] * weight[k] / 3.0 / sampleRate * h2; // uniform of amplitude weight: var w^2 / 3
            }
            return s;
        };
        for (int k = 0; k < kPoles; ++k)
        {
            const double fk = 5.0 * std::pow (10.0, 0.5 * k);
            pole[k] = std::exp (-twoPi * fk / sampleRate);
            weight[k] = 1.0 / std::sqrt (fk);
            state[k] = 0.0;
        }
        const double scale = std::sqrt (fourKTR * kFlickerCorner / 1000.0 / bankDensity (1000.0));
        for (double& w : weight) w *= scale;
    }

    // next noise sample (volts at the grid)
    double next()
    {
        double v = white * uniform();
        for (int k = 0; k < kPoles; ++k)
        {
            state[k] = pole[k] * state[k] + (1.0 - pole[k]) * weight[k] * uniform();
            v += state[k];
        }
        return v;
    }

private:
    static constexpr int kPoles = 10; // 5 Hz .. 158 kHz

    double uniform() // [-1, 1), xorshift64*
    {
        rng ^= rng >> 12; rng ^= rng << 25; rng ^= rng >> 27;
        return (double) ((rng * 0x2545F4914F6CDD1Dull) >> 11) * (2.0 / 9007199254740992.0) - 1.0;
    }

    double white = 0.0, pole[kPoles] {}, weight[kPoles] {}, state[kPoles] {};
    unsigned long long rng = 1;
};
} // namespace vamp
