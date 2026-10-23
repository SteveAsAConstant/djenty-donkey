#pragma once
#include <juce_audio_basics/juce_audio_basics.h>
#include <vector>
#include "Filters.h"

namespace vamp
{
class StereoDelay
{
public:
    void prepare (double sampleRate)
    {
        sr = sampleRate;
        size = (int) (sampleRate * 2.5) + 4;
        bufL.assign ((size_t) size, 0.0f);
        bufR.assign ((size_t) size, 0.0f);
        delaySamples.reset (sampleRate, 0.2);
        dampL.set (4500.0, sampleRate);
        dampR.set (4500.0, sampleRate);
        reset();
    }

    void reset()
    {
        std::fill (bufL.begin(), bufL.end(), 0.0f);
        std::fill (bufR.begin(), bufR.end(), 0.0f);
        dampL.reset(); dampR.reset();
        writePos = 0;
    }

    void process (float* L, float* R, int numSamples, float timeMs, float feedback, float mix, bool pingPong)
    {
        delaySamples.setTargetValue (juce::jlimit (1.0f, (float) size - 4.0f, timeMs * 0.001f * (float) sr));

        for (int i = 0; i < numSamples; ++i)
        {
            const float d = delaySamples.getNextValue();
            const float wl = dampL.process (read (bufL, d));
            const float wr = dampR.process (read (bufR, d));

            if (pingPong)
            {
                const float in = 0.5f * (L[i] + R[i]);
                bufL[(size_t) writePos] = in + wr * feedback;
                bufR[(size_t) writePos] = wl * feedback;
            }
            else
            {
                bufL[(size_t) writePos] = L[i] + wl * feedback;
                bufR[(size_t) writePos] = R[i] + wr * feedback;
            }

            L[i] += wl * mix;
            R[i] += wr * mix;
            writePos = (writePos + 1) % size;
        }
    }

private:
    float read (const std::vector<float>& buf, float delay) const
    {
        float rp = (float) writePos - delay;
        if (rp < 0.0f) rp += (float) size;
        const int i0 = (int) rp;
        const int i1 = (i0 + 1) % size;
        const float frac = rp - (float) i0;
        return buf[(size_t) i0] + frac * (buf[(size_t) i1] - buf[(size_t) i0]);
    }

    double sr = 44100.0;
    int size = 1, writePos = 0;
    std::vector<float> bufL, bufR;
    OnePoleLP dampL, dampR;
    juce::SmoothedValue<float> delaySamples;
};

} // namespace vamp
