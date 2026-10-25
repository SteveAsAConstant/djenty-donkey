#pragma once
#include <juce_audio_processors/juce_audio_processors.h>

namespace ids
{
inline constexpr const char* inGain      = "inGain";
inline constexpr const char* inGateOn    = "inGateOn";
inline constexpr const char* inGateThresh = "inGateThresh";
inline constexpr const char* inGateRelease = "inGateRelease";
inline constexpr const char* gateOn      = "gateOn";
inline constexpr const char* gateThresh  = "gateThresh";
inline constexpr const char* gateRelease = "gateRelease";
inline constexpr const char* gateLow     = "gateLow";
inline constexpr const char* gateMid     = "gateMid";
inline constexpr const char* gateHigh    = "gateHigh";
inline constexpr const char* transpose   = "transpose";
inline constexpr const char* transposeMode = "transposeMode";
inline constexpr const char* compOn      = "compOn";
inline constexpr const char* compThresh  = "compThresh";
inline constexpr const char* compRatio   = "compRatio";
inline constexpr const char* compAttack  = "compAttack";
inline constexpr const char* compRelease = "compRelease";
inline constexpr const char* compMakeup  = "compMakeup";
inline constexpr const char* boostOn     = "boostOn";
inline constexpr const char* boostDrive  = "boostDrive";
inline constexpr const char* boostTone   = "boostTone";
inline constexpr const char* boostLevel  = "boostLevel";
inline constexpr const char* boostAttack = "boostAttack";
inline constexpr const char* boostModel  = "boostModel";
inline constexpr const char* channel     = "channel";
inline constexpr const char* ampBright   = "ampBright";
inline constexpr const char* ampCrunch   = "ampCrunch";
inline constexpr const char* ampNoise    = "ampNoise";
inline constexpr const char* tubeSet     = "tubeSet";
inline constexpr const char* ampGain     = "ampGain";
inline constexpr const char* bass        = "bass";
inline constexpr const char* mid         = "mid";
inline constexpr const char* treble      = "treble";
inline constexpr const char* presence    = "presence";
inline constexpr const char* resonance   = "resonance";
inline constexpr const char* master      = "master";
inline constexpr const char* ampVolume   = "ampVolume";
inline constexpr const char* cabOn       = "cabOn";
inline constexpr const char* eqOn        = "eqOn";
inline constexpr const char* phaserOn    = "phaserOn";
inline constexpr const char* phRate      = "phRate";
inline constexpr const char* phDepth     = "phDepth";
inline constexpr const char* phFeedback  = "phFeedback";
inline constexpr const char* phMix       = "phMix";
inline constexpr const char* delayOn     = "delayOn";
inline constexpr const char* dlTime      = "dlTime";
inline constexpr const char* dlFeedback  = "dlFeedback";
inline constexpr const char* dlMix       = "dlMix";
inline constexpr const char* dlPingPong  = "dlPingPong";
inline constexpr const char* reverbOn    = "reverbOn";
inline constexpr const char* rvSize      = "rvSize";
inline constexpr const char* rvDamping   = "rvDamping";
inline constexpr const char* rvWidth     = "rvWidth";
inline constexpr const char* rvMix       = "rvMix";
inline constexpr const char* outGain     = "outGain";

inline juce::String eqBand (int i) { return "eq" + juce::String (i); }
inline constexpr float eqFreqs[10] = { 31, 62, 125, 250, 500, 1000, 2000, 4000, 8000, 16000 };
inline const char* eqLabels[10] = { "31", "62", "125", "250", "500", "1k", "2k", "4k", "8k", "16k" };
} // namespace ids

namespace vamp
{
inline std::unique_ptr<juce::AudioParameterFloat> makeFloat (const juce::String& id, const juce::String& name,
                                                             float min, float max, float def,
                                                             const juce::String& suffix = {}, int decimals = 1,
                                                             float skewCentre = 0.0f)
{
    juce::NormalisableRange<float> range (min, max);
    if (skewCentre > min && skewCentre < max)
        range.setSkewForCentre (skewCentre);

    return std::make_unique<juce::AudioParameterFloat> (
        juce::ParameterID { id, 1 }, name, range, def,
        juce::AudioParameterFloatAttributes()
            .withLabel (suffix)
            .withStringFromValueFunction ([decimals, suffix] (float v, int) { return juce::String (v, decimals) + suffix; }));
}

inline std::unique_ptr<juce::AudioParameterBool> makeBool (const juce::String& id, const juce::String& name, bool def)
{
    return std::make_unique<juce::AudioParameterBool> (juce::ParameterID { id, 1 }, name, def);
}

inline juce::AudioProcessorValueTreeState::ParameterLayout createParameterLayout()
{
    juce::AudioProcessorValueTreeState::ParameterLayout layout;

    layout.add (makeFloat (ids::inGain, "Input", -24.0f, 24.0f, 0.0f, " dB"));

    layout.add (makeBool  (ids::inGateOn, "Input Gate On", false));
    layout.add (makeFloat (ids::inGateThresh, "Input Gate Threshold", -90.0f, 0.0f, -50.0f, " dB"));
    layout.add (makeFloat (ids::inGateRelease, "Input Gate Release", 2.0f, 200.0f, 20.0f, " ms", 0, 20.0f));

    layout.add (makeBool  (ids::gateOn, "Gate On", true));
    layout.add (makeFloat (ids::gateThresh, "Gate Threshold", -90.0f, 0.0f, -60.0f, " dB"));
    layout.add (makeFloat (ids::gateRelease, "Gate Release", 5.0f, 500.0f, 80.0f, " ms", 0, 80.0f));
    layout.add (makeFloat (ids::gateLow, "Gate Low Depth", 0.0f, 100.0f, 30.0f, " %", 0));
    layout.add (makeFloat (ids::gateMid, "Gate Mid Depth", 0.0f, 100.0f, 60.0f, " %", 0));
    layout.add (makeFloat (ids::gateHigh, "Gate High Depth", 0.0f, 100.0f, 100.0f, " %", 0));

    layout.add (std::make_unique<juce::AudioParameterInt> (juce::ParameterID { ids::transpose, 1 }, "Transpose", -12, 12, 0));
    layout.add (std::make_unique<juce::AudioParameterChoice> (juce::ParameterID { ids::transposeMode, 1 }, "Transpose Mode",
                                                              juce::StringArray { "Live", "HQ" }, 0));

    layout.add (makeBool  (ids::compOn, "Comp On", false));
    layout.add (makeFloat (ids::compThresh, "Comp Threshold", -60.0f, 0.0f, -20.0f, " dB"));
    layout.add (makeFloat (ids::compRatio, "Comp Ratio", 1.0f, 20.0f, 4.0f, ":1", 1, 4.0f));
    layout.add (makeFloat (ids::compAttack, "Comp Attack", 0.1f, 100.0f, 5.0f, " ms", 1, 10.0f));
    layout.add (makeFloat (ids::compRelease, "Comp Release", 10.0f, 1000.0f, 100.0f, " ms", 0, 150.0f));
    layout.add (makeFloat (ids::compMakeup, "Comp Makeup", 0.0f, 24.0f, 0.0f, " dB"));

    layout.add (makeBool  (ids::boostOn, "Boost On", false));
    layout.add (makeFloat (ids::boostDrive, "Boost Drive", 0.0f, 10.0f, 0.0f));
    layout.add (makeFloat (ids::boostTone, "Drive Bright", 0.0f, 10.0f, 5.0f));
    layout.add (makeFloat (ids::boostLevel, "Drive Volume", 0.0f, 10.0f, 7.0f));
    layout.add (std::make_unique<juce::AudioParameterInt> (juce::ParameterID { ids::boostAttack, 1 }, "Drive Attack", 1, 6, 4));
    layout.add (std::make_unique<juce::AudioParameterChoice> (juce::ParameterID { ids::boostModel, 1 }, "Drive Model",
                                                              juce::StringArray { "Precision Drive", "TS9" }, 0));

    layout.add (std::make_unique<juce::AudioParameterChoice> (juce::ParameterID { ids::channel, 1 }, "Channel",
                                                              juce::StringArray { "Clean", "Blue", "Red" }, 2));
    layout.add (makeBool (ids::ampBright, "Bright", true)); // the rhythm channel's BRIGHT pushbutton (CLEAN, BLUE)
    layout.add (makeBool (ids::ampCrunch, "Crunch", true)); // the rhythm channel's CRUNCH pushbutton (BLUE)
    layout.add (makeBool (ids::ampNoise, "Amp Noise", false)); // circuit noise: resistors' thermal, tubes' shot / flicker
    layout.add (std::make_unique<juce::AudioParameterChoice> (juce::ParameterID { ids::tubeSet, 1 }, "Tube Set",
                                                              juce::StringArray { "Stock", "Set A", "Set B", "Set C", "Set D", "Set E" }, 0));
    layout.add (makeFloat (ids::ampGain, "Gain", 0.0f, 10.0f, 6.9f));
    layout.add (makeFloat (ids::bass, "Bass", 0.0f, 10.0f, 5.0f));
    layout.add (makeFloat (ids::mid, "Mid", 0.0f, 10.0f, 5.0f));
    layout.add (makeFloat (ids::treble, "Treble", 0.0f, 10.0f, 6.0f));
    layout.add (makeFloat (ids::presence, "Presence", 0.0f, 10.0f, 5.0f));
    layout.add (makeFloat (ids::resonance, "Resonance", 0.0f, 10.0f, 5.0f));
    layout.add (makeFloat (ids::master, "Master", 0.0f, 10.0f, 4.6f));
    layout.add (makeFloat (ids::ampVolume, "Amp Volume", -40.0f, 12.0f, 0.0f, " dB"));

    layout.add (makeBool (ids::cabOn, "Cab On", true));

    layout.add (makeBool (ids::eqOn, "EQ On", false));
    for (int i = 0; i < 10; ++i)
        layout.add (makeFloat (ids::eqBand (i), juce::String ("EQ ") + ids::eqLabels[i], -12.0f, 12.0f, 0.0f, " dB"));

    layout.add (makeBool  (ids::phaserOn, "Phaser On", false));
    layout.add (makeFloat (ids::phRate, "Phaser Rate", 0.05f, 5.0f, 0.5f, " Hz", 2, 0.8f));
    layout.add (makeFloat (ids::phDepth, "Phaser Depth", 0.0f, 1.0f, 0.6f, "", 2));
    layout.add (makeFloat (ids::phFeedback, "Phaser Feedback", -0.9f, 0.9f, 0.3f, "", 2));
    layout.add (makeFloat (ids::phMix, "Phaser Mix", 0.0f, 1.0f, 0.5f, "", 2));

    layout.add (makeBool  (ids::delayOn, "Delay On", false));
    layout.add (makeFloat (ids::dlTime, "Delay Time", 20.0f, 2000.0f, 380.0f, " ms", 0, 400.0f));
    layout.add (makeFloat (ids::dlFeedback, "Delay Feedback", 0.0f, 0.95f, 0.35f, "", 2));
    layout.add (makeFloat (ids::dlMix, "Delay Mix", 0.0f, 1.0f, 0.3f, "", 2));
    layout.add (makeBool  (ids::dlPingPong, "Ping Pong", false));

    layout.add (makeBool  (ids::reverbOn, "Reverb On", false));
    layout.add (makeFloat (ids::rvSize, "Reverb Size", 0.0f, 1.0f, 0.5f, "", 2));
    layout.add (makeFloat (ids::rvDamping, "Reverb Damping", 0.0f, 1.0f, 0.5f, "", 2));
    layout.add (makeFloat (ids::rvWidth, "Reverb Width", 0.0f, 1.0f, 1.0f, "", 2));
    layout.add (makeFloat (ids::rvMix, "Reverb Mix", 0.0f, 1.0f, 0.25f, "", 2));

    layout.add (makeFloat (ids::outGain, "Output", -24.0f, 24.0f, 0.0f, " dB"));

    return layout;
}
} // namespace vamp
