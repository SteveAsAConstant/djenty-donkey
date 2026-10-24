#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "Parameters.h"
#include "dsp/Preamp5150.h"

// Gain / Master knob curves were recalibrated against a real amp in v0.4 (see Preamp5150.h).
// States and preset files without this marker are from v0.3: their Gain and Master values are
// converted so that they keep their sound.
inline constexpr int kKnobCurvesVersion = 2;
inline constexpr const char* kKnobCurvesAttr = "knobCurves";

inline void upgradeKnobCurves (juce::XmlElement& xml)
{
    if (xml.getIntAttribute (kKnobCurvesAttr, 1) >= kKnobCurvesVersion)
        return;
    bool lead = true; // the Gain curve differs per channel (lead pre-gain low end, see Preamp5150.h)
    for (auto* p : xml.getChildWithTagNameIterator ("PARAM"))
        if (p->getStringAttribute ("id") == ids::channel)
            lead = juce::roundToInt (p->getDoubleAttribute ("value")) == vamp::Preamp5150::Red;
    for (auto* p : xml.getChildWithTagNameIterator ("PARAM"))
    {
        const auto id = p->getStringAttribute ("id");
        const double v = p->getDoubleAttribute ("value") * 0.1;
        if (id == ids::ampGain) p->setAttribute ("value", 10.0 * vamp::gainKnobFromV03 (v, lead));
        else if (id == ids::master) p->setAttribute ("value", 10.0 * vamp::masterKnobFromV03 (v));
    }
    xml.setAttribute (kKnobCurvesAttr, kKnobCurvesVersion);
}

// Factory presets (built into the plugin) + user presets (XML files in
// Documents/Djenty Donkey/Presets, easy to share). Call everything from the message thread.
class PresetManager
{
public:
    using Values = std::vector<std::pair<const char*, float>>;
    struct Factory { const char* name; Values values; };

    PresetManager (juce::AudioProcessorValueTreeState& state,
                   std::function<void (const juce::File&)> loadIRFn,
                   std::function<juce::String()> getIRPathFn)
        : apvts (state), loadIR (std::move (loadIRFn)), getIRPath (std::move (getIRPathFn))
    {
        refresh();
    }

    static juce::File getUserFolder()
    {
        auto dir = juce::File::getSpecialLocation (juce::File::userDocumentsDirectory)
                       .getChildFile ("Djenty Donkey").getChildFile ("Presets");
        dir.createDirectory();
        return dir;
    }

    void refresh()
    {
        userFiles = getUserFolder().findChildFiles (juce::File::findFiles, false, "*.xml");
        userFiles.sort();
    }

    int getNumFactory() const { return (int) factory().size(); }
    int getNumPresets() const { return getNumFactory() + userFiles.size(); }
    bool isUser (int index) const { return index >= getNumFactory(); }

    juce::String getName (int index) const
    {
        if (index < 0 || index >= getNumPresets()) return {};
        return isUser (index) ? userFiles[index - getNumFactory()].getFileNameWithoutExtension()
                              : juce::String (factory()[(size_t) index].name);
    }

    int indexOf (const juce::String& name) const
    {
        for (int i = 0; i < getNumPresets(); ++i)
            if (getName (i) == name) return i;
        return -1;
    }

    juce::String getCurrentName() const { return currentName; }
    void setCurrentName (const juce::String& n) { currentName = n; }

    void load (int index)
    {
        if (index < 0 || index >= getNumPresets()) return;

        if (! isUser (index))
        {
            resetAllToDefault();
            for (auto& [id, value] : factory()[(size_t) index].values)
                setParam (id, value);
        }
        else
        {
            std::unique_ptr<juce::XmlElement> xml (juce::XmlDocument::parse (userFiles[index - getNumFactory()]));
            if (xml == nullptr) return;
            upgradeKnobCurves (*xml);
            resetAllToDefault();
            for (auto* p : xml->getChildWithTagNameIterator ("PARAM"))
                setParam (p->getStringAttribute ("id"), (float) p->getDoubleAttribute ("value"));

            const juce::File ir (xml->getStringAttribute ("irPath"));
            if (juce::File::isAbsolutePath (ir.getFullPathName()) && ir.existsAsFile())
                loadIR (ir);
        }
        currentName = getName (index);
    }

    bool save (const juce::String& rawName)
    {
        const auto name = juce::File::createLegalFileName (rawName.trim());
        if (name.isEmpty()) return false;

        auto state = apvts.copyState();
        std::unique_ptr<juce::XmlElement> xml (state.createXml());
        if (xml == nullptr) return false;
        xml->setTagName ("DjentyDonkeyPreset");
        xml->setAttribute ("name", name);
        xml->setAttribute ("version", JucePlugin_VersionString);
        xml->setAttribute (kKnobCurvesAttr, kKnobCurvesVersion);
        xml->setAttribute ("irPath", getIRPath());

        const bool ok = xml->writeTo (getUserFolder().getChildFile (name + ".xml"));
        refresh();
        if (ok) currentName = name;
        return ok;
    }

    bool remove (int index)
    {
        if (! isUser (index) || index >= getNumPresets()) return false;
        const bool ok = userFiles[index - getNumFactory()].deleteFile();
        refresh();
        return ok;
    }

private:
    void setParam (const juce::String& id, float value)
    {
        if (auto* p = apvts.getParameter (id))
        {
            p->beginChangeGesture();
            p->setValueNotifyingHost (p->convertTo0to1 (value));
            p->endChangeGesture();
        }
    }

    void resetAllToDefault()
    {
        for (auto* p : apvts.processor.getParameters())
            if (auto* rp = dynamic_cast<juce::RangedAudioParameter*> (p))
            {
                rp->beginChangeGesture();
                rp->setValueNotifyingHost (rp->getDefaultValue());
                rp->endChangeGesture();
            }
    }

    // Starting points tuned from the circuit's behaviour - adjust by ear.
    static const std::vector<Factory>& factory()
    {
        static const std::vector<Factory> presets = {
            { "Default", {} },
            { "Djent Rhythm", {
                { ids::channel, 2 }, { ids::ampGain, 6.1f }, { ids::bass, 4.5f }, { ids::mid, 6.0f }, { ids::treble, 6.5f },
                { ids::presence, 5.5f }, { ids::resonance, 4.0f }, { ids::master, 5.1f },
                { ids::boostOn, 1 }, { ids::boostDrive, 0.0f }, { ids::boostLevel, 9.0f }, { ids::boostTone, 6.0f }, { ids::boostAttack, 2 },
                { ids::inGateOn, 1 }, { ids::inGateThresh, -50.0f }, { ids::inGateRelease, 15.0f },
                { ids::gateThresh, -55.0f }, { ids::gateRelease, 60.0f }, { ids::gateLow, 40.0f }, { ids::gateMid, 70.0f }, { ids::gateHigh, 100.0f } } },
            { "Djent Drop -2 (LIVE)", {
                { ids::channel, 2 }, { ids::ampGain, 6.1f }, { ids::bass, 4.0f }, { ids::mid, 6.0f }, { ids::treble, 6.5f },
                { ids::presence, 5.5f }, { ids::resonance, 3.5f }, { ids::master, 5.1f },
                { ids::boostOn, 1 }, { ids::boostDrive, 0.0f }, { ids::boostLevel, 9.0f }, { ids::boostTone, 6.0f }, { ids::boostAttack, 1 },
                { ids::transpose, -2 }, { ids::transposeMode, 0 },
                { ids::inGateOn, 1 }, { ids::inGateThresh, -50.0f }, { ids::inGateRelease, 15.0f },
                { ids::gateThresh, -55.0f }, { ids::gateRelease, 60.0f } } },
            { "5150 Lead", {
                { ids::channel, 2 }, { ids::ampGain, 7.7f }, { ids::bass, 5.0f }, { ids::mid, 6.5f }, { ids::treble, 6.0f },
                { ids::presence, 6.0f }, { ids::resonance, 5.0f }, { ids::master, 5.7f },
                { ids::boostOn, 1 }, { ids::boostDrive, 2.0f }, { ids::boostLevel, 7.0f }, { ids::boostAttack, 4 },
                { ids::gateThresh, -62.0f }, { ids::gateRelease, 120.0f },
                { ids::delayOn, 1 }, { ids::dlTime, 420.0f }, { ids::dlFeedback, 0.3f }, { ids::dlMix, 0.2f }, { ids::dlPingPong, 1 },
                { ids::reverbOn, 1 }, { ids::rvSize, 0.55f }, { ids::rvMix, 0.18f } } },
            { "Blue Crunch", {
                { ids::channel, 1 }, { ids::ampGain, 6.9f }, { ids::bass, 5.5f }, { ids::mid, 5.5f }, { ids::treble, 6.0f },
                { ids::presence, 5.0f }, { ids::resonance, 5.0f }, { ids::master, 4.6f },
                { ids::gateThresh, -62.0f }, { ids::gateLow, 20.0f }, { ids::gateMid, 40.0f }, { ids::gateHigh, 80.0f } } },
            { "Big Clean", {
                { ids::channel, 0 }, { ids::ampGain, 5.3f }, { ids::bass, 5.0f }, { ids::mid, 5.0f }, { ids::treble, 6.5f },
                { ids::presence, 4.0f }, { ids::resonance, 3.0f }, { ids::master, 5.7f },
                { ids::gateOn, 0 },
                { ids::compOn, 1 }, { ids::compThresh, -24.0f }, { ids::compRatio, 3.0f }, { ids::compMakeup, 4.0f },
                { ids::delayOn, 1 }, { ids::dlTime, 380.0f }, { ids::dlFeedback, 0.25f }, { ids::dlMix, 0.15f },
                { ids::reverbOn, 1 }, { ids::rvSize, 0.6f }, { ids::rvMix, 0.3f } } },
        };
        return presets;
    }

    juce::AudioProcessorValueTreeState& apvts;
    std::function<void (const juce::File&)> loadIR;
    std::function<juce::String()> getIRPath;
    juce::Array<juce::File> userFiles;
    juce::String currentName { "Default" };
};
