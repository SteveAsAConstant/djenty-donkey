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
// Measured with tests/dsp_test ("chain"): CLEAN needs far more post gain than BLUE/RED. 54 since the factory rhythm
// network (R21 1M ahead of the R23 / C63 shunt), 6.7 dB more attenuation than before: CLEAN as loud as in v0.5.0-beta.
constexpr double kChannelPost[3] = { 54.0, 1.0, 1.0 };

class AmpSim
{
public:
    struct Settings
    {
        int channel = 2;
        bool bright = true;    // BRIGHT pushbutton (rhythm: CLEAN and BLUE)
        bool crunch = true;    // CRUNCH pushbutton (BLUE)
        bool noise = false;    // circuit noise of the preamp (resistors, tubes)
        int tubeSet = 0;       // TubeSet: 0 nominal tubes, 1.. particular sets
        float gain = 0.6f, bass = 0.5f, mid = 0.5f, treble = 0.6f;
        float presence = 0.5f, resonance = 0.5f, master = 0.3f; // all 0..1
        float volumeDb = 0.0f; // post-power-amp level (like an attenuator), does not change the tone
        bool boostOn = false;
        float boostDrive = 0.0f, boostBright = 0.5f, boostLevel = 0.7f;
        int boostAttack = 3; // 0..5 (the knob minus one)
        int boostModel = 0;  // 0 Precision Drive, 1 TS9
    };

    // channel-switch crossfade (Preamp5150::switchFadeMs: 0 = relay switch, the default); tests, before prepare()
    void setSwitchFadeMs (double ms) { preamp.switchFadeMs = ms; }
    void setSwitchDipMs (double ms) { preamp.switchDipMs = ms; }    // relay dip (Preamp5150::switchDipMs); tests
    bool testBothPedals = false; // tests: both drive pedals always run, as before single-pedal running (never reset)

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
        ts9.prepare (osRate);
        modelStep = 1.0 / (0.02 * osRate); // drive model switch: 20 ms crossfade
        preamp.setChannelTrims (kChannelPost[0], kChannelPost[1], kChannelPost[2]); // applied inside the switch crossfade
        // the tubes of the last block (or nominal): settled with them
        const auto tubes = TubeSet::make (tubeSetApplied);
        preamp.setTubeSet (tubes);
        power.setTubeSet (tubes);
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
        volumeDbApplied = std::numeric_limits<float>::quiet_NaN(); // the dB -> gain conversion again on the first block
        dcBlock.set (10.0, osRate); // 10 Hz, on the speaker voltage
        dcBlock.reset();
        oversampler->reset();
        knobsSet = false;
        firstBlock = true;
        firstModel = true;
        bothPedals = false;
        knobCoef = 1.0 - std::exp (-kControlInterval / (kKnobSmoothMs * 1e-3 * osRate));
    }

    // Knob smoothing time constant (ms); 0 = no one-pole, each knob only ramps linearly across the host block (tests)
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
        preamp.setCrunch (s.crunch); // before the channel, so the first block starts on the right circuit
        if (firstBlock) preamp.setModeNow (channel); // nothing audible yet: no switch crossfade
        else preamp.setMode (channel);
        preamp.setBright (s.bright); // no-op unless it changed
        preamp.setNoise (s.noise);   // likewise
        if (s.tubeSet != tubeSetApplied) // other tubes: as swapping them, the stages re-bias through their caps
        {
            tubeSetApplied = std::clamp (s.tubeSet, 0, TubeSet::kSets - 1);
            const auto tubes = TubeSet::make (tubeSetApplied);
            preamp.setTubeSet (tubes);
            power.setTubeSet (tubes);
        }
        firstBlock = false;
        if (s.boostOn && (! lastBoostOn || s.boostAttack != boostAttack)) // switches: no smoothing
        {
            drive.setControls (knob[7], knob[8], s.boostAttack, knob[9]);
            ts9.setControls (knob[7], knob[8], knob[9]);
        }
        modelTarget = s.boostModel == 1 ? 1.0 : 0.0;
        if (firstModel) { modelMix = modelTarget; modelW = 0.5 - 0.5 * std::cos (kPi * modelMix); firstModel = false; }
        if (testBothPedals) bothPedals = true;
        else if (s.boostOn && modelTarget != modelMix && ! bothPedals)
        {
            // a crossfade starts: the pedal fading in has not run since it was last faded out, so it starts from rest
            // (its coupling caps and filters hold stale voltages otherwise); from rest its own start-up transient dies
            // out within the fade (tests/amp_path_test boost)
            if (modelTarget > 0.5) ts9.reset(); else drive.reset();
            bothPedals = true;
        }
        lastBoostOn = s.boostOn;
        boostAttack = s.boostAttack;
        const double target[kKnobs] = { s.gain, s.bass, s.mid, s.treble, s.master, s.presence, s.resonance,
                                        s.boostDrive, s.boostBright, s.boostLevel };
        std::copy (knobTarget, knobTarget + kKnobs, knobFrom); // the last block's values: the target ramps from there
        std::copy (target, target + kKnobs, knobTarget);
        if (! knobsSet)
        {
            std::copy (target, target + kKnobs, knob);
            std::copy (target, target + kKnobs, knobFrom);
            knobsSet = true;
            applyKnobs (true);
        }
        if (s.volumeDb != volumeDbApplied) // the dB -> gain conversion only when the knob moved
        {
            volumeDbApplied = s.volumeDb;
            volume.setTargetValue (std::pow (10.0, s.volumeDb / 20.0) * kSpeakerToDigital);
        }

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
                stepKnobs (s.boostOn, (double) (i + 1) / (double) n);
            }
            double v = (double) x[i] * kInputVolts;

            if (s.boostOn) // a model switch is a 20 ms crossfade (both pedals run); settled, only the selected one runs
            {
                if (modelMix != modelTarget)
                {
                    modelMix = modelTarget > modelMix ? std::min (modelTarget, modelMix + modelStep) : std::max (modelTarget, modelMix - modelStep);
                    modelW = 0.5 - 0.5 * std::cos (kPi * modelMix); // the crossfade weight: only while the mix moves
                }
                else if (! testBothPedals)
                    bothPedals = false; // settled (modelW exactly 0 or 1: the mix is the same as with both running)
                if (bothPedals) v = (1.0 - modelW) * drive.process (v) + modelW * ts9.process (v);
                else v = modelW > 0.5 ? ts9.process (v) : drive.process (v);
            }

            v = processAmp5150 (preamp, power, v, 1.0); // volts across the speaker (preamp on the power amp's rails; channel trim in the preamp)
            x[i] = dcBlock.process ((float) (v * volume.getNextValue()));
        }

        oversampler->processSamplesDown (block);
    }

private:
    // Knob smoothing. The knobs reach process() once per host block; applied as one jump, every block boundary
    // is a step in the circuit (zipper noise / clicks while a knob moves, coarser with larger buffers). Each knob
    // follows its target with a one-pole (kKnobSmoothMs) instead, and the circuit is updated every
    // kControlInterval oversampled samples (~0.1 ms) while one is still moving. The target itself ramps linearly
    // across the block from the last block's value: a one-pole on the raw per-block staircase still moves in
    // block-rate jerks (Resonance swept with 512-sample blocks: -47 dB of sidebands vs -59 dB with a smooth target).
    // Gain and Presence / Resonance sit inside DK circuits (lead preamp, power amp): their matrices are rebuilt then
    // (DKCircuit: allocation-free), so they follow every kHeavyEvery-th step only (~0.36 ms: still ~55 steps per
    // 20 ms ramp, a quarter of the CPU).
    static constexpr int kKnobs = 10, kControlInterval = 32, kHeavyEvery = 4;
    static constexpr double kKnobSmoothMs = 20.0;

    // frac: position in the current host block (0..1], where the target is between knobFrom and knobTarget
    void stepKnobs (bool boostOn, double frac)
    {
        bool moving = false;
        for (int k = 0; k < kKnobs; ++k)
        {
            const double t = knobFrom[k] + frac * (knobTarget[k] - knobFrom[k]), d = t - knob[k];
            if (d == 0.0) continue;
            knob[k] = std::abs (d) < 1e-5 ? t : knob[k] + knobCoef * d;
            moving = true;
        }
        // the preamp's and the power amp's rebuilds fall on different steps (half a cycle apart), so they never
        // land in the same block: the worst block carries one rebuild, not two
        if (++heavyPhase >= kHeavyEvery) heavyPhase = 0;
        const bool gainStep = heavyPhase == 0, powerStep = heavyPhase == kHeavyEvery / 2;
        const bool heavyPending = (gainStep && heavyApplied[0] != knob[0])
                               || (powerStep && (heavyApplied[1] != knob[5] || heavyApplied[2] != knob[6]));
        if (moving || heavyPending)
            applyKnobs (boostOn, gainStep, powerStep);
    }

    void applyKnobs (bool boostOn, bool gain = true, bool powerKnobs = true)
    {
        if (gain) heavyApplied[0] = knob[0];
        if (powerKnobs) { heavyApplied[1] = knob[5]; heavyApplied[2] = knob[6]; }
        preamp.setControls (heavyApplied[0], knob[1], knob[2], knob[3], knob[4]);
        power.setControls (heavyApplied[1], heavyApplied[2]);
        if (boostOn)
        {
            drive.setControls (knob[7], knob[8], boostAttack, knob[9]);
            ts9.setControls (knob[7], knob[8], knob[9]);
        }
    }

    double knob[kKnobs] {}, knobTarget[kKnobs] {}, knobFrom[kKnobs] {}, knobCoef = 1.0;
    double heavyApplied[3] {}; // Gain, Presence, Resonance as last applied
    int heavyPhase = 0;
    bool knobsSet = false, lastBoostOn = false, firstBlock = true, firstModel = true;
    int tubeSetApplied = 0;
    double modelMix = 0.0, modelTarget = 0.0, modelStep = 1e-4; // drive model crossfade, 0 = Precision Drive
    double modelW = 0.0;                                         // its raised-cosine weight (1 = TS9)
    bool bothPedals = false;      // a crossfade is in progress: both pedals run; otherwise only the selected one
    float volumeDbApplied = std::numeric_limits<float>::quiet_NaN(); // volumeDb the smoother's target was set from
    int controlPhase = 0, boostAttack = 3;

    static constexpr double kMinOsRate = 352800.0; // 8 x 44.1 kHz
    std::unique_ptr<juce::dsp::Oversampling<float>> oversampler;
    double osRate = 176400.0;

    PrecisionDrive drive;
    TubeScreamer9 ts9;
    Preamp5150 preamp;
    PowerAmp5150 power;
    OnePoleHP dcBlock;
    juce::SmoothedValue<double, juce::ValueSmoothingTypes::Multiplicative> volume { kSpeakerToDigital };
};

} // namespace vamp
