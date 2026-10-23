#pragma once
#include "PluginProcessor.h"
#include "UI/Components.h"

// Cabinet IR loader: Load button, prev/next IR in folder, built-in cab, drag & drop
class IRPanel : public juce::Component,
                public juce::FileDragAndDropTarget,
                private juce::Timer
{
public:
    explicit IRPanel (VibeAmpProcessor&);

    void resized() override;
    void paint (juce::Graphics&) override;

    bool isInterestedInFileDrag (const juce::StringArray& files) override;
    void filesDropped (const juce::StringArray& files, int, int) override;
    void fileDragEnter (const juce::StringArray&, int, int) override { dragOver = true; repaint(); }
    void fileDragExit (const juce::StringArray&) override { dragOver = false; repaint(); }

private:
    void timerCallback() override;

    VibeAmpProcessor& proc;
    juce::TextButton loadButton { "LOAD IR" }, prevButton { "<" }, nextButton { ">" }, builtinButton { "BUILT-IN" };
    juce::Label nameLabel;
    std::unique_ptr<juce::FileChooser> chooser;
    juce::File lastDirectory;
    bool dragOver = false;
};

class VibeAmpEditor : public juce::AudioProcessorEditor
{
public:
    explicit VibeAmpEditor (VibeAmpProcessor&);
    ~VibeAmpEditor() override;

    void paint (juce::Graphics&) override;
    void resized() override;

private:
    void setAmpChannelColour (int channel);

    VibeAmpProcessor& proc;
    ui::LookAndFeel lnf;
    juce::TooltipWindow tooltipWindow { this, 400 };

    std::unique_ptr<ui::Section> inSec, gateSec, pitchSec, compSec, boostSec;
    std::unique_ptr<ui::Section> ampSec;
    std::unique_ptr<ui::Section> cabSec, eqSec;
    std::unique_ptr<ui::Section> phaserSec, delaySec, reverbSec, outSec;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (VibeAmpEditor)
};
