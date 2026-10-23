#pragma once
#include <juce_dsp/juce_dsp.h>
#include "Filters.h"
#include "PowerAmp5150.h"
#include "Preamp5150.h"
#include "PrecisionDrive.h"

namespace vamp
{
// Signal levels inside the amp are real volts.
constexpr double kInputVolts = 1.0;          // 0 dBFS input = 1 V peak at the guitar jack
constexpr double kSpeakerToDigital = 1.0 / 100.0; // speaker volts -> digital (max ~56 V -> ~0.56)

// Per-channel calibration of the post-gain pot (the real amp has separate Clean / Lead Post pots).
// Measured with tests/dsp_test ("chain"): CLEAN needs far more post gain than BLUE/RED.
constexpr double kChannelPost[3] = { 25.0, 1.0, 1.0 };

class AmpSim
{
public:
    struct Settings
    {
        int channel = 2;
        float gain = 0.6f, bass = 0.5f, mid = 0.5f, treble = 0.6f;
        float presence = 0.5f, resonance = 0.5f, master = 0.3f; // all 0..1
        float volumeDb = 0.0f; // post-power-amp level (like an attenuator), does not change the tone
        bool boostOn = false;
        float boostDrive = 0.0f, boostBright = 0.5f, boostLevel = 0.7f;
        int boostAttack = 3; // 0..7
    };

    void prepare (double sampleRate, int maxBlock)
    {
        // 4x oversampling around the whole non-linear amp (preamp + power amp)
        oversampler = std::make_unique<juce::dsp::Oversampling<float>> (
            1, 2, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, false);
        oversampler->initProcessing ((size_t) maxBlock);
        osRate = sampleRate * 4.0;

        drive.prepare (osRate);
        preamp.prepare (osRate);
        power.prepare (osRate);
        volume.reset (osRate, 0.03);
        dcBlock.reset();
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
        power.setControls (s.presence, s.resonance);
        if (s.boostOn)
            drive.setControls (s.boostDrive, s.boostBright, s.boostAttack, s.boostLevel);
        dcBlock.set (10.0, osRate);

        const double post = kChannelPost[channel];
        volume.setTargetValue (std::pow (10.0, s.volumeDb / 20.0) * kSpeakerToDigital);

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

            v = preamp.process (v) * post;        // volts at the post-gain (master) pot wiper
            v = power.process (v);                // volts across the speaker
            x[i] = dcBlock.process ((float) (v * volume.getNextValue()));
        }

        oversampler->processSamplesDown (block);
    }

private:
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;
    double osRate = 176400.0;

    PrecisionDrive drive;
    Preamp5150 preamp;
    PowerAmp5150 power;
    OnePoleHP dcBlock;
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Multiplicative> volume { kSpeakerToDigital };
};

} // namespace vamp
