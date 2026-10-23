#pragma once
#include <juce_dsp/juce_dsp.h>
#include "Filters.h"
#include "Preamp5150.h"
#include "PrecisionDrive.h"

namespace vamp
{
// Signal levels inside the amp are real volts.
constexpr double kInputVolts = 1.0;       // 0 dBFS input = 1 V peak at the guitar jack
constexpr double kPowerAmpScale = 1.0 / 8.0; // volts after post-gain pot -> power amp drive units

// per-channel level trims so the three channels sit at similar loudness
// (measured with tests/dsp_test: clean ~0.05 V rms vs ~3.7 V rms for blue/red at default knobs)
constexpr double kChannelTrim[3] = { 60.0, 1.0, 1.0 };

class AmpSim
{
public:
    struct Settings
    {
        int channel = 2;
        float gain = 0.6f, bass = 0.5f, mid = 0.5f, treble = 0.6f;
        float presence = 0.5f, resonance = 0.5f, master = 0.5f; // all 0..1
        bool boostOn = false;
        float boostDrive = 0.0f, boostBright = 0.5f, boostLevel = 0.7f;
        int boostAttack = 3; // 0..7
    };

    void prepare (double sampleRate, int maxBlock)
    {
        // 4x oversampling around the non-linear circuit
        oversampler = std::make_unique<juce::dsp::Oversampling<float>> (
            1, 2, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, false);
        oversampler->initProcessing ((size_t) maxBlock);
        osRate = sampleRate * 4.0;

        drive.prepare (osRate);
        preamp.prepare (osRate);
        resonance.reset(); presence.reset(); dcBlock.reset();
        oversampler->reset();
    }

    int getLatencySamples() const
    {
        return oversampler ? juce::roundToInt (oversampler->getLatencyInSamples()) : 0;
    }

    void process (float* data, int numSamples, const Settings& s)
    {
        const int channel = juce::jlimit (0, 2, s.channel);
        preamp.setMode (channel);
        preamp.setControls (s.gain, s.bass, s.mid, s.treble, s.master);
        if (s.boostOn)
            drive.setControls (s.boostDrive, s.boostBright, s.boostAttack, s.boostLevel);

        using T = Biquad::Type;
        resonance.set (T::peak, osRate, 90.0, 1.0, s.resonance * 9.0f);
        presence.set (T::highShelf, osRate, 4500.0, 0.7, s.presence * 10.0f);
        dcBlock.set (10.0, osRate);

        const double trim = kPowerAmpScale * kChannelTrim[channel];

        float* chans[] = { data };
        juce::dsp::AudioBlock<float> block (chans, 1, (size_t) numSamples);
        auto up = oversampler->processSamplesUp (block);
        float* x = up.getChannelPointer (0);
        const int n = (int) up.getNumSamples();

        for (int i = 0; i < n; ++i)
        {
            double v = (double) x[i] * kInputVolts;

            if (s.boostOn)
                v = drive.process (v);

            v = preamp.process (v) * trim;

            // power amp (simplified for now): resonance / presence in the NFB loop, push-pull saturation
            float p = resonance.process ((float) v);
            p = presence.process (p);
            p = std::tanh (p + 0.05f) - std::tanh (0.05f);

            x[i] = dcBlock.process (p) * 0.5f;
        }

        oversampler->processSamplesDown (block);
    }

private:
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;
    double osRate = 176400.0;

    PrecisionDrive drive;
    Preamp5150 preamp;
    Biquad resonance, presence;
    OnePoleHP dcBlock;
};

} // namespace vamp
