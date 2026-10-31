#pragma once
#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>
#include "Filters.h"

namespace vamp
{
// Guitar tuner: the dry input, decimated to ~24 kHz on the audio thread (two low-pass biquads, then every k-th sample
// into a ring), its pitch found off it on a UI / control thread (YIN: the cumulative mean normalised difference, the
// first dip under kThreshold, parabolic interpolation). 25 Hz .. 1.5 kHz: drop tunings of a 7 / 8 string to the top of
// the neck. A 170 ms window (four periods of the lowest note); a reading every call, at the caller's rate.
class Tuner
{
public:
    struct Reading
    {
        bool valid = false;  // a pitch was found (and the input is above kMinLevelDb)
        double hz = 0.0;
        int note = 0;        // MIDI note number of the nearest note (A4 = 69)
        double cents = 0.0;  // from that note, -50..+50
        float levelDb = -120.0f;
    };
    static constexpr double kA4 = 440.0, kMinHz = 25.0, kMaxHz = 1500.0, kThreshold = 0.15;
    static constexpr float kMinLevelDb = -60.0f;

    void prepare (double sampleRate)
    {
        decim = std::max (1, (int) std::lround (sampleRate / 24000.0));
        rate = sampleRate / decim;
        for (auto& b : lp) { b.set (BiquadType::lowPass, sampleRate, std::min (0.4 * rate, 9000.0), 0.707); b.reset(); }
        window = 1;
        while (window < (int) (0.17 * rate)) window <<= 1;
        size = window * 2;
        ring.reset (new std::atomic<float>[(size_t) size]);
        for (int i = 0; i < size; ++i) ring[(size_t) i].store (0.0f, std::memory_order_relaxed);
        written.store (0);
        phase = 0;
        const std::lock_guard<std::mutex> g (analysing);
        x.assign ((size_t) window, 0.0f);
        d.assign ((size_t) window / 2, 0.0f);
    }

    // audio thread, while the tuner is on
    void push (const float* in, int n)
    {
        if (size == 0) return;
        auto w = written.load (std::memory_order_relaxed);
        for (int i = 0; i < n; ++i)
        {
            const float v = lp[1].process (lp[0].process (in[i]));
            if (++phase < decim) continue;
            phase = 0;
            ring[(size_t) (w % size)].store (v, std::memory_order_relaxed);
            ++w;
        }
        written.store (w, std::memory_order_release);
    }

    // any one thread at a time (a lock): the pitch of the last window
    Reading analyse()
    {
        Reading r;
        const std::lock_guard<std::mutex> g (analysing);
        const auto w = written.load (std::memory_order_acquire);
        if (size == 0 || w < window) return r;
        double power = 0.0;
        for (int i = 0; i < window; ++i)
        {
            x[(size_t) i] = ring[(size_t) ((w - window + i) % size)].load (std::memory_order_relaxed);
            power += (double) x[(size_t) i] * x[(size_t) i];
        }
        r.levelDb = (float) (10.0 * std::log10 (power / window + 1e-24));
        if (r.levelDb < kMinLevelDb) return r;

        // difference function over lags up to half the window, cumulative-mean normalised
        const int maxLag = std::min (window / 2 - 1, (int) (rate / kMinHz)), minLag = std::max (2, (int) (rate / kMaxHz));
        const int len = window - maxLag;
        d[0] = 1.0f;
        double running = 0.0;
        for (int tau = 1; tau <= maxLag; ++tau)
        {
            double s = 0.0;
            for (int i = 0; i < len; ++i)
            {
                const double v = (double) x[(size_t) i] - x[(size_t) (i + tau)];
                s += v * v;
            }
            running += s;
            d[(size_t) tau] = running > 0.0 ? (float) (s * tau / running) : 1.0f;
        }
        int tau = -1;
        for (int t = minLag; t < maxLag; ++t)
            if (d[(size_t) t] < kThreshold)
            {
                while (t + 1 < maxLag && d[(size_t) (t + 1)] < d[(size_t) t]) ++t; // down to the dip's bottom
                tau = t;
                break;
            }
        if (tau < 0) return r;
        const double a = d[(size_t) (tau - 1)], b = d[(size_t) tau], c = d[(size_t) (tau + 1)];
        const double den = a - 2.0 * b + c;
        const double t = tau + (std::abs (den) > 1e-12 ? 0.5 * (a - c) / den : 0.0);
        r.hz = rate / t;
        const double midi = 69.0 + 12.0 * std::log2 (r.hz / kA4);
        r.note = (int) std::lround (midi);
        r.cents = 100.0 * (midi - r.note);
        r.valid = true;
        return r;
    }

    static const char* noteName (int midi)
    {
        static const char* names[] = { "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B" };
        return names[((midi % 12) + 12) % 12];
    }
    static int octave (int midi) { return midi / 12 - 1; }

private:
    Biquad lp[2];
    int decim = 2, phase = 0, window = 0, size = 0;
    double rate = 24000.0;
    std::unique_ptr<std::atomic<float>[]> ring;
    std::atomic<long long> written { 0 };
    std::mutex analysing;
    std::vector<float> x, d;
};
} // namespace vamp
