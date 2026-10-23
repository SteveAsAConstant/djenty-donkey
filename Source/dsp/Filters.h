#pragma once
#include <algorithm>
#include <cmath>

namespace vamp
{
constexpr double kPi = 3.14159265358979323846;

// One-pole low-pass: y += g * (x - y)
struct OnePoleLP
{
    float g = 1.0f, y = 0.0f;

    void set (double fc, double fs)
    {
        fc = std::clamp (fc, 1.0, fs * 0.45);
        g = (float) (1.0 - std::exp (-2.0 * kPi * fc / fs));
    }
    void reset() { y = 0.0f; }
    inline float process (float x) { y += g * (x - y); return y; }
};

// One-pole high-pass, built as x - LP(x)
struct OnePoleHP
{
    OnePoleLP lp;

    void set (double fc, double fs) { lp.set (fc, fs); }
    void reset() { lp.reset(); }
    inline float process (float x) { return x - lp.process (x); }
};

// RBJ cookbook biquad, transposed direct form II. Allocation-free coefficient updates.
struct Biquad
{
    enum class Type { lowPass, highPass, peak, lowShelf, highShelf };

    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float z1 = 0, z2 = 0;

    void reset() { z1 = z2 = 0.0f; }

    inline float process (float x)
    {
        const float y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }

    void set (Type type, double fs, double f, double q, double gainDb = 0.0)
    {
        f = std::clamp (f, 10.0, fs * 0.45);
        const double A = std::pow (10.0, gainDb / 40.0);
        const double w0 = 2.0 * kPi * f / fs;
        const double cw = std::cos (w0), sw = std::sin (w0);
        const double alpha = sw / (2.0 * q);
        const double sqA = std::sqrt (A);

        double B0 = 1, B1 = 0, B2 = 0, A0 = 1, A1 = 0, A2 = 0;

        switch (type)
        {
            case Type::lowPass:
                B0 = (1 - cw) / 2; B1 = 1 - cw; B2 = (1 - cw) / 2;
                A0 = 1 + alpha;    A1 = -2 * cw; A2 = 1 - alpha;
                break;
            case Type::highPass:
                B0 = (1 + cw) / 2; B1 = -(1 + cw); B2 = (1 + cw) / 2;
                A0 = 1 + alpha;    A1 = -2 * cw;   A2 = 1 - alpha;
                break;
            case Type::peak:
                B0 = 1 + alpha * A; B1 = -2 * cw; B2 = 1 - alpha * A;
                A0 = 1 + alpha / A; A1 = -2 * cw; A2 = 1 - alpha / A;
                break;
            case Type::lowShelf:
                B0 = A * ((A + 1) - (A - 1) * cw + 2 * sqA * alpha);
                B1 = 2 * A * ((A - 1) - (A + 1) * cw);
                B2 = A * ((A + 1) - (A - 1) * cw - 2 * sqA * alpha);
                A0 = (A + 1) + (A - 1) * cw + 2 * sqA * alpha;
                A1 = -2 * ((A - 1) + (A + 1) * cw);
                A2 = (A + 1) + (A - 1) * cw - 2 * sqA * alpha;
                break;
            case Type::highShelf:
                B0 = A * ((A + 1) + (A - 1) * cw + 2 * sqA * alpha);
                B1 = -2 * A * ((A - 1) + (A + 1) * cw);
                B2 = A * ((A + 1) + (A - 1) * cw - 2 * sqA * alpha);
                A0 = (A + 1) - (A - 1) * cw + 2 * sqA * alpha;
                A1 = 2 * ((A - 1) - (A + 1) * cw);
                A2 = (A + 1) - (A - 1) * cw - 2 * sqA * alpha;
                break;
        }

        b0 = (float) (B0 / A0); b1 = (float) (B1 / A0); b2 = (float) (B2 / A0);
        a1 = (float) (A1 / A0); a2 = (float) (A2 / A0);
    }
};

inline float dbToGain (float db) { return std::pow (10.0f, db / 20.0f); }

} // namespace vamp
