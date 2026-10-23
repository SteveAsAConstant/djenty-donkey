#pragma once
#include <cmath>
#include <vector>
#include "Filters.h"

namespace vamp
{
// Simple low-latency delay-line pitch shifter (two crossfaded read taps).
// Good enough for drop tunings in a first version; can later be swapped for a
// phase-vocoder / PSOLA engine for cleaner chords.
class PitchShifter
{
public:
    void prepare (double sampleRate)
    {
        window = (float) (sampleRate * 0.05); // 50 ms grain
        int size = 1;
        while (size < (int) window * 2 + 16) size <<= 1;
        buffer.assign ((size_t) size, 0.0f);
        mask = size - 1;
        reset();
    }

    void reset()
    {
        std::fill (buffer.begin(), buffer.end(), 0.0f);
        writePos = 0;
        phase = 0.0f;
    }

    void setSemitones (int semis)
    {
        semitones = semis;
        ratio = std::pow (2.0f, (float) semis / 12.0f);
    }

    void process (float* data, int numSamples)
    {
        for (int i = 0; i < numSamples; ++i)
        {
            const float x = data[i];
            buffer[(size_t) writePos] = x;

            if (semitones != 0)
            {
                phase += (1.0f - ratio) / window;
                phase -= std::floor (phase);
                float phase2 = phase + 0.5f;
                if (phase2 >= 1.0f) phase2 -= 1.0f;

                // Hann crossfade: sin^2 + cos^2 = 1
                const float g1 = std::sin ((float) kPi * phase);
                const float g2 = std::sin ((float) kPi * phase2);

                data[i] = g1 * g1 * read (phase * window + 2.0f)
                        + g2 * g2 * read (phase2 * window + 2.0f);
            }

            writePos = (writePos + 1) & mask;
        }
    }

private:
    float read (float delay) const
    {
        float rp = (float) writePos - delay;
        while (rp < 0.0f) rp += (float) (mask + 1);
        const int i0 = (int) rp;
        const float frac = rp - (float) i0;
        const float a = buffer[(size_t) (i0 & mask)];
        const float b = buffer[(size_t) ((i0 + 1) & mask)];
        return a + frac * (b - a);
    }

    std::vector<float> buffer;
    int mask = 0, writePos = 0, semitones = 0;
    float window = 2400.0f, phase = 0.0f, ratio = 1.0f;
};

} // namespace vamp
