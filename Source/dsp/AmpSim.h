#pragma once
#include <juce_dsp/juce_dsp.h>
#include "Filters.h"
#include "Amp5150.h"
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
        // Oversampling around the whole non-linear amp (preamp + power amp), to at least kMinOsRate:
        // 8x at 44.1 / 48 kHz, 4x at 88.2 / 96 kHz, 2x at 176.4 / 192 kHz. Aliasing (inharmonic energy for
        // heavily clipped 1-5 kHz sines, tests/amp_path_test): 4x -50..-55 dB, 8x -61..-70 dB, ~1.7x the CPU.
        int stages = 1;
        while (sampleRate * (1 << stages) < kMinOsRate && stages < 3)
            ++stages;
        oversampler = std::make_unique<juce::dsp::Oversampling<float>> (
            1, (size_t) stages, juce::dsp::Oversampling<float>::filterHalfBandPolyphaseIIR, true, false);
        oversampler->initProcessing ((size_t) maxBlock);
        osRate = sampleRate * (1 << stages);

        drive.prepare (osRate);
        preamp.prepare (osRate);
        power.prepare (osRate);
        // settle preamp and power amp together on the shared supply, so the rails start at the loaded DC point
        // (as PowerAmp5150::prepare: rails held, load currents averaged once each update's transient has died out)
        auto settle = [this] (double seconds) { for (int i = 0; i < (int) (osRate * seconds); ++i) processAmp5150 (preamp, power, 0.0, 1.0); };
        power.beginSupplySettle();
        for (int pass = 0; pass < 3; ++pass)
        {
            settle (0.1);
            power.resetSupplyAverage();
            settle (0.1);
            power.applySupplyDC();
        }
        settle (0.15);
        power.endSupplySettle();
        volume.reset (osRate, 0.03);
        dcBlock.reset();
        oversampler->reset();
        knobsSet = false;
        knobCoef = 1.0 - std::exp (-kControlInterval / (kKnobSmoothMs * 1e-3 * osRate));
    }

    // Knob smoothing time constant (ms); 0 = apply each block's values at once (the pre-v0.5 behaviour, tests)
    void setKnobSmoothing (double ms)
    {
        knobCoef = ms <= 0.0 ? 1.0 : 1.0 - std::exp (-kControlInterval / (ms * 1e-3 * osRate));
    }

    int getLatencySamples() const
    {
        return oversampler ? juce::roundToInt (oversampler->getLatencyInSamples()) : 0;
    }

    void process (float* data, int numSamples, const Settings& s)
    {
        const int channel = juce::jlimit (0, 2, s.channel);
        preamp.setMode (channel);
        if (s.boostOn && (! lastBoostOn || s.boostAttack != boostAttack)) // switches: no smoothing
            drive.setControls (knob[7], knob[8], s.boostAttack, knob[9]);
        lastBoostOn = s.boostOn;
        boostAttack = s.boostAttack;
        const double target[kKnobs] = { s.gain, s.bass, s.mid, s.treble, s.master, s.presence, s.resonance,
                                        s.boostDrive, s.boostBright, s.boostLevel };
        std::copy (target, target + kKnobs, knobTarget);
        if (! knobsSet)
        {
            std::copy (target, target + kKnobs, knob);
            knobsSet = true;
            applyKnobs (true);
        }
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
            if (++controlPhase >= kControlInterval)
            {
                controlPhase = 0;
                stepKnobs (s.boostOn);
            }
            double v = (double) x[i] * kInputVolts;

            if (s.boostOn)
                v = drive.process (v);

            v = processAmp5150 (preamp, power, v, post); // volts across the speaker (preamp on the power amp's rails)
            x[i] = dcBlock.process ((float) (v * volume.getNextValue()));
        }

        oversampler->processSamplesDown (block);
    }

private:
    // Knob smoothing. The knobs reach process() once per host block; applied as one jump, every block boundary
    // is a step in the circuit (zipper noise / clicks while a knob moves, coarser with larger buffers). Each knob
    // follows its target with a one-pole (kKnobSmoothMs) instead, and the circuit is updated every
    // kControlInterval oversampled samples (~0.1 ms) while one is still moving. Gain and Presence / Resonance sit
    // inside DK circuits (lead preamp, power amp): their matrices are rebuilt then (DKCircuit: allocation-free), so
    // they follow every kHeavyEvery-th step only (~0.36 ms: still ~55 steps per 20 ms ramp, a quarter of the CPU).
    static constexpr int kKnobs = 10, kControlInterval = 32, kHeavyEvery = 4;
    static constexpr double kKnobSmoothMs = 20.0;
    static constexpr int kHeavy[3] = { 0, 5, 6 }; // Gain, Presence, Resonance

    void stepKnobs (bool boostOn)
    {
        bool moving = false;
        for (int k = 0; k < kKnobs; ++k)
        {
            const double d = knobTarget[k] - knob[k];
            if (d == 0.0) continue;
            knob[k] = std::abs (d) < 1e-5 ? knobTarget[k] : knob[k] + knobCoef * d;
            moving = true;
        }
        const bool heavyStep = ++heavyPhase >= kHeavyEvery;
        if (heavyStep) heavyPhase = 0;
        bool heavyPending = false;
        for (int k : kHeavy) heavyPending = heavyPending || heavyApplied[k == 0 ? 0 : k - 4] != knob[k];
        if (moving || (heavyStep && heavyPending))
            applyKnobs (boostOn, heavyStep);
    }

    void applyKnobs (bool boostOn, bool heavy = true)
    {
        if (heavy)
            for (int k : kHeavy) heavyApplied[k == 0 ? 0 : k - 4] = knob[k];
        preamp.setControls (heavyApplied[0], knob[1], knob[2], knob[3], knob[4]);
        power.setControls (heavyApplied[1], heavyApplied[2]);
        if (boostOn)
            drive.setControls (knob[7], knob[8], boostAttack, knob[9]);
    }

    double knob[kKnobs] {}, knobTarget[kKnobs] {}, knobCoef = 1.0;
    double heavyApplied[3] {}; // Gain, Presence, Resonance as last applied
    int heavyPhase = 0;
    bool knobsSet = false, lastBoostOn = false;
    int controlPhase = 0, boostAttack = 3;

    static constexpr double kMinOsRate = 352800.0; // 8 x 44.1 kHz
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;
    double osRate = 176400.0;

    PrecisionDrive drive;
    Preamp5150 preamp;
    PowerAmp5150 power;
    OnePoleHP dcBlock;
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Multiplicative> volume { kSpeakerToDigital };
};

} // namespace vamp
