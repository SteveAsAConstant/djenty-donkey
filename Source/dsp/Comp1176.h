#pragma once
#include <juce_dsp/juce_dsp.h>
#include "Urei1176.h"

namespace vamp
{
// The 1176LN model (Urei1176) in the plugin: its own 2x oversampling at 44.1 / 48 kHz (none at 88.2 kHz and up)
// and the front-panel controls (Urei1176::fromKnobs). Signals are volts at the guitar jack's scale (1.0 = 1 V), as
// AmpSim's input; in a reamp loop: a DI + preamp in front (Urei1176::kDiGain), a reamp box's pad to the amp
// (Urei1176::kReampPad).
// Model rate: 88.2 kHz and up are within 0.1 dB of a 384 kHz reference on picked notes even at the fastest attack
// (tests/comp1176_test null); about 15 % of one core.
class Comp1176
{
public:
    struct Settings
    {
        float input = 5.0f;   // INPUT 0..10: 10 = full CW (no attenuation), 0 = 36 dB of attenuation
        float output = 5.3f;  // OUTPUT 0..10: pot rotation (audio taper); 5.3 = unity below threshold at INPUT 5
        float attack = 5.0f;  // ATTACK dial 1..7, 7 = fastest (20 us class), 1 = slowest (~800 us)
        float release = 5.0f; // RELEASE dial 1..7, 7 = fastest (50 ms), 1 = slowest (1.1 s)
        int ratio = 0;        // 0..3 = 4 / 8 / 12 / 20:1, 4 = all buttons in
    };

    void prepare (double sampleRate, int maxBlock)
    {
        factor = sampleRate < 80000.0 ? 2 : 1;
        oversampler.reset();
        if (factor > 1)
        {
            oversampler = std::make_unique<juce::dsp::Oversampling<float>> (
                1, 1, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, false);
            oversampler->initProcessing ((size_t) maxBlock);
        }
        model.setControls (toControls (last));
        model.prepare (sampleRate * factor);
    }

    void reset()
    {
        model.reset();
        if (oversampler) oversampler->reset();
        gr.store (0.0f);
    }

    int getLatencySamples() const { return oversampler ? juce::roundToInt (oversampler->getLatencyInSamples()) : 0; }

    // gain reduction (dB) at the end of the last block, for the meter
    float getGainReductionDb() const { return gr.load(); }

    void process (float* data, int numSamples, const Settings& s)
    {
        if (s.input != last.input || s.output != last.output || s.attack != last.attack || s.release != last.release
            || s.ratio != last.ratio)
        {
            last = s;
            model.setControls (toControls (s));
        }
        if (factor == 1)
            for (int i = 0; i < numSamples; ++i) data[i] = (float) (Urei1176::kReampPad * model.processSample (Urei1176::kDiGain * data[i]));
        else
        {
            float* chans[] = { data };
            juce::dsp::AudioBlock<float> block (chans, 1, (size_t) numSamples);
            auto up = oversampler->processSamplesUp (block);
            float* x = up.getChannelPointer (0);
            for (size_t i = 0; i < up.getNumSamples(); ++i) x[i] = (float) (Urei1176::kReampPad * model.processSample (Urei1176::kDiGain * x[i]));
            oversampler->processSamplesDown (block);
        }
        gr.store ((float) model.gainReductionDb());
    }

    static Urei1176::Controls toControls (const Settings& s)
    {
        return Urei1176::fromKnobs (s.input, s.output, s.attack, s.release, s.ratio);
    }

private:
    Urei1176 model;
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;
    int factor = 2;
    Settings last;
    std::atomic<float> gr { 0.0f };
};

} // namespace vamp
