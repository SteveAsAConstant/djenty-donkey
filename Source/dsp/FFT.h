#pragma once
#include <cmath>
#include <complex>
#include <vector>

namespace vamp
{
// Minimal in-place radix-2 complex FFT (size must be a power of two).
class FFT
{
public:
    void prepare (int size)
    {
        n = size;
        twiddle.resize ((size_t) n / 2);
        for (int k = 0; k < n / 2; ++k)
            twiddle[(size_t) k] = std::polar (1.0, -2.0 * 3.14159265358979323846 * k / n);
        rev.resize ((size_t) n);
        int bits = 0;
        while ((1 << bits) < n) ++bits;
        for (int i = 0; i < n; ++i)
        {
            int r = 0;
            for (int b = 0; b < bits; ++b) r |= ((i >> b) & 1) << (bits - 1 - b);
            rev[(size_t) i] = r;
        }
    }

    // forward: X[k] = sum x[n] e^{-j2pi kn/N};  inverse: unnormalised (caller divides by N)
    void transform (std::complex<double>* data, bool inverse) const
    {
        for (int i = 0; i < n; ++i)
            if (i < rev[(size_t) i]) std::swap (data[i], data[rev[(size_t) i]]);

        for (int len = 2; len <= n; len <<= 1)
        {
            const int half = len / 2, step = n / len;
            for (int start = 0; start < n; start += len)
                for (int k = 0; k < half; ++k)
                {
                    auto w = twiddle[(size_t) (k * step)];
                    if (inverse) w = std::conj (w);
                    const auto t = w * data[start + k + half];
                    data[start + k + half] = data[start + k] - t;
                    data[start + k] += t;
                }
        }
    }

    int size() const { return n; }

private:
    int n = 0;
    std::vector<std::complex<double>> twiddle;
    std::vector<int> rev;
};

} // namespace vamp
