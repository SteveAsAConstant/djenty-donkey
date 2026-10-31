#pragma once
#include <algorithm>
#include <cmath>
#include <complex>
#include <functional>
#include <vector>
#include "FFT.h"
#include "Filters.h"

namespace vamp
{
// The built-in cabinet, used until an IR is loaded: a 4x12-ish curve
inline void setupFallbackCab (Biquad (&f)[5], double sampleRate)
{
    using T = Biquad::Type;
    f[0].set (T::highPass, sampleRate, 80.0, 0.7);
    f[1].set (T::peak, sampleRate, 120.0, 1.2, 3.0);
    f[2].set (T::peak, sampleRate, 400.0, 1.0, -3.0);
    f[3].set (T::peak, sampleRate, 2500.0, 1.5, 3.0);
    f[4].set (T::lowPass, sampleRate, 5000.0, 0.9);
    for (auto& b : f) b.reset();
}

// ITU-R BS.1770 K-weighting (its high shelf and RLB high-pass, as specified at 48 kHz): power gain at f (Hz)
inline double kWeighting (double f)
{
    constexpr double fs = 48000.0, pi = 3.14159265358979323846;
    const std::complex<double> z1 = std::polar (1.0, -2.0 * pi * f / fs), z2 = z1 * z1;
    auto power = [&] (double b0, double b1, double b2, double a1, double a2)
    {
        return std::norm ((b0 + b1 * z1 + b2 * z2) / (1.0 + a1 * z1 + a2 * z2));
    };
    double shelf, hp;
    {
        const double f0 = 1681.974450955533, g = 3.999843853973347, q = 0.7071752369554196;
        const double k = std::tan (pi * f0 / fs), vh = std::pow (10.0, g / 20.0), vb = std::pow (vh, 0.4996667741545416);
        const double a0 = 1.0 + k / q + k * k;
        shelf = power ((vh + vb * k / q + k * k) / a0, 2.0 * (k * k - vh) / a0, (vh - vb * k / q + k * k) / a0,
                       2.0 * (k * k - 1.0) / a0, (1.0 - k / q + k * k) / a0);
    }
    {
        const double f0 = 38.13547087602444, q = 0.5003270373238773;
        const double k = std::tan (pi * f0 / fs), a0 = 1.0 + k / q + k * k;
        hp = power (1.0, -2.0, 1.0, 2.0 * (k * k - 1.0) / a0, (1.0 - k / q + k * k) / a0);
    }
    return shelf * hp;
}

// The amp's own output spectrum ahead of the cab, the reference signal for matching: power density (dB re 1 kHz)
// at 1/3-octave centres 20 Hz * 2^(i/3), i = 0..30. Measured from the model (default settings: RED, gain 6, bass 5,
// mid 5, treble 6, presence / resonance 5, master 3; drop-A chugs 2/3 + a lead line 1/3), smoothed over an octave.
// A high-gain amp puts far more energy below 200 Hz than pink noise does, which is where cab IRs differ most
// (a pink reference left bass-heavy IRs ~4 dB loud on chugs).
inline double ampOutputDensityDb (double f)
{
    static constexpr double db[31] = { 17.9, 16.5, 12.1, 12.2, 11.1, 16.3, 18.9, 19.0, 17.3, 8.8, 8.3, 5.3, 4.3, 2.4, 0.8, 0.7,
                                       1.5, 0.0, -3.5, -7.3, -8.6, -10.0, -11.0, -12.2, -14.9, -15.9, -17.4, -19.2, -20.7, -22.7, -23.9 };
    const double x = std::clamp (3.0 * std::log2 (std::max (f, 1.0) / 20.0), 0.0, 30.0);
    const int i = std::min (29, (int) x);
    return db[i] + (x - i) * (db[i + 1] - db[i]);
}

// Loudness of a cabinet given as its impulse response h (n samples at rate) for the amp's output spectrum,
// K-weighted: the sum over 20 Hz .. 20 kHz of |H(f)|^2 K(f) S(f). Arbitrary units: only ratios are used.
inline double cabLoudness (const float* h, int n, double rate)
{
    int size = 1;
    while (size < std::max (n, 65536)) size <<= 1;
    FFT fft;
    fft.prepare (size);
    std::vector<std::complex<double>> d ((size_t) size);
    for (int i = 0; i < n; ++i) d[(size_t) i] = h[i];
    fft.transform (d.data(), false);
    double sum = 0.0;
    for (int k = 1; k < size / 2; ++k)
    {
        const double f = k * rate / size;
        if (f < 20.0 || f > 20000.0) continue;
        sum += std::norm (d[(size_t) k]) * kWeighting (f) * std::pow (10.0, ampOutputDensityDb (f) / 10.0);
    }
    return sum;
}

// The gain that makes a loaded IR (h, n samples at rate) as loud as the built-in cabinet, so switching between
// them or between IRs keeps the level (for the amp's output spectrum, K-weighted; the tone is untouched)
inline double cabMatchGain (const float* h, int n, double rate)
{
    static const double fallback = []
    {
        Biquad f[5];
        setupFallbackCab (f, 48000.0);
        std::vector<float> ir (65536);
        for (size_t i = 0; i < ir.size(); ++i)
        {
            float v = i == 0 ? 1.0f : 0.0f;
            for (auto& b : f) v = b.process (v);
            ir[i] = v;
        }
        return cabLoudness (ir.data(), (int) ir.size(), 48000.0);
    }();
    const double l = cabLoudness (h, n, rate);
    return l > 1e-30 ? std::sqrt (fallback / l) : 1.0;
}

// The same match for what is actually being played: x is the amp's output ahead of the cab (nx samples at xRate, e.g.
// the last seconds). The loudness of a cab is the sum over the bins of one FFT of x of |H(f)|^2 K(f) |X(f)|^2 (|H|^2
// of each IR at the nearest bin of its own FFT): exactly the K-weighted loudness the convolved signal has, so an IR
// loaded with this gain is as loud as the built-in cabinet for the player's own settings and playing. The fixed
// spectrum above stands for one setting (no drive pedal): with a pedal in front a dark IR came out 5.8 dB quiet
// (Neural Donkey, a real 59 s capture, docs/pi_bench.md). post: the power gain at f of what follows the cab (the EQ),
// so the match holds at the output. Returns 0 when x holds too little signal.
// |X|^2 is x's averaged spectrum (Welch: Hann segments of kSegment, half overlapping; 2.9 Hz bins at 48 kHz) rather
// than one FFT of all of x. One 60 s FFT (4M points, 64 MB) flushed the Pi's shared cache: loading an IR overran the
// audio threads, even from another core (15 such matches: ~340 xruns). 16k segments (256 kB) stay in that core's own L2
// (40 loads: ~4 overruns, 64k segments 22); the gain differs from the one-FFT match by <= 0.03 dB on 60 s of real
// playing and 0.13 dB on 20 s, five IRs (docs/pi_bench.md).
inline double cabMatchGainFor (const float* h, int n, double rate, const float* x, int nx, double xRate,
                                const std::function<double (double)>& post = {})
{
    constexpr int kSegment = 16384;
    double power = 0.0;
    for (int i = 0; i < nx; ++i) power += (double) x[i] * x[i];
    if (nx < (int) xRate || power / nx < 1e-6) return 0.0; // under a second, or quieter than -60 dBFS rms
    FFT fft;
    fft.prepare (kSegment);
    std::vector<std::complex<double>> d ((size_t) kSegment);
    auto spectrum = [&] (const float* s, int len) // len <= kSegment: one FFT, zero-padded
    {
        std::fill (d.begin(), d.end(), std::complex<double> {});
        for (int i = 0; i < len; ++i) d[(size_t) i] = s[i];
        fft.transform (d.data(), false);
        std::vector<double> p ((size_t) kSegment / 2);
        for (size_t k = 0; k < p.size(); ++k) p[k] = std::norm (d[k]);
        return p;
    };
    std::vector<double> X ((size_t) kSegment / 2, 0.0);
    {
        std::vector<double> win ((size_t) kSegment);
        for (int i = 0; i < kSegment; ++i) win[(size_t) i] = 0.5 - 0.5 * std::cos (2.0 * kPi * i / kSegment);
        const int hop = kSegment / 2;
        for (int start = 0; start == 0 || start + kSegment <= nx; start += hop) // at least one segment (zero-padded)
        {
            std::fill (d.begin(), d.end(), std::complex<double> {});
            for (int i = 0; i < kSegment && start + i < nx; ++i) d[(size_t) i] = x[start + i] * win[(size_t) i];
            fft.transform (d.data(), false);
            for (size_t k = 0; k < X.size(); ++k) X[k] += std::norm (d[k]);
        }
    }
    std::vector<float> builtin ((size_t) kSegment);
    {
        Biquad f[5];
        setupFallbackCab (f, xRate);
        for (size_t i = 0; i < builtin.size(); ++i)
        {
            float v = i == 0 ? 1.0f : 0.0f;
            for (auto& b : f) v = b.process (v);
            builtin[i] = v;
        }
    }
    // the loaded IR at its own resolution, once (up to 65536 samples, 1 MB: 1.4 s at 48 kHz), each |X|^2 bin reading its
    // nearest bin; the built-in cab (an 80 Hz high-pass and four filters, gone within 0.3 s) at the segments' size
    std::vector<double> H;
    {
        int size = kSegment;
        while (size < std::min (n, 65536)) size <<= 1;
        FFT hf;
        hf.prepare (size);
        std::vector<std::complex<double>> hd ((size_t) size);
        for (int i = 0; i < std::min (n, size); ++i) hd[(size_t) i] = h[i];
        hf.transform (hd.data(), false);
        H.resize ((size_t) size / 2);
        for (size_t k = 0; k < H.size(); ++k) H[k] = std::norm (hd[k]);
    }
    const auto B = spectrum (builtin.data(), kSegment);
    const double xBin = xRate / (2.0 * (double) X.size()), hBin = rate / (2.0 * (double) H.size()), bBin = xRate / (2.0 * (double) B.size());
    double lh = 0.0, lb = 0.0;
    for (size_t k = 1; k < X.size(); ++k)
    {
        const double f = (double) k * xBin;
        if (f < 20.0 || f > 20000.0 || f >= 0.5 * rate) continue;
        const double w = X[k] * kWeighting (f) * (post ? post (f) : 1.0);
        lh += w * H[std::min (H.size() - 1, (size_t) std::lround (f / hBin))];
        lb += w * B[std::min (B.size() - 1, (size_t) std::lround (f / bBin))];
    }
    return lh > 1e-30 ? std::sqrt (lb / lh) : 0.0;
}
} // namespace vamp
