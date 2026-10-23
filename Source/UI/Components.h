#pragma once
#include <functional>
#include <juce_gui_basics/juce_gui_basics.h>
#include <juce_audio_processors/juce_audio_processors.h>

namespace ui
{
using APVTS = juce::AudioProcessorValueTreeState;

namespace colours
{
inline const juce::Colour background { 0xff141418 };
inline const juce::Colour panel      { 0xff222228 };
inline const juce::Colour track      { 0xff3a3a42 };
inline const juce::Colour text       { 0xffd8d8e0 };
inline const juce::Colour clean      { 0xff3ddc84 };
inline const juce::Colour blue       { 0xff3d8bff };
inline const juce::Colour red        { 0xffff3b3b };
inline const juce::Colour orange     { 0xffffa53d };
inline const juce::Colour purple     { 0xffb26bff };
inline const juce::Colour teal       { 0xff3dd6d0 };
inline const juce::Colour yellow     { 0xfff5d33d };
} // namespace colours

inline juce::Font font (float h, bool bold = false)
{
    return juce::Font (juce::FontOptions (h, bold ? juce::Font::bold : juce::Font::plain));
}

// ---------------------------------------------------------------------------
class LookAndFeel : public juce::LookAndFeel_V4
{
public:
    LookAndFeel()
    {
        setColour (juce::Slider::textBoxTextColourId, colours::text);
        setColour (juce::Slider::textBoxOutlineColourId, juce::Colours::transparentBlack);
        setColour (juce::Slider::trackColourId, colours::orange);
        setColour (juce::Slider::backgroundColourId, colours::track);
        setColour (juce::Label::textColourId, colours::text);
        setColour (juce::TextButton::buttonColourId, colours::track);
        setColour (juce::TextButton::textColourOffId, colours::text);
        setColour (juce::TextButton::textColourOnId, juce::Colours::black);
    }

    void drawRotarySlider (juce::Graphics& g, int x, int y, int w, int h, float pos,
                           float startAngle, float endAngle, juce::Slider& s) override
    {
        auto bounds = juce::Rectangle<int> (x, y, w, h).toFloat().reduced (4.0f);
        const float r = juce::jmin (bounds.getWidth(), bounds.getHeight()) * 0.5f;
        const auto c = bounds.getCentre();
        const auto accent = s.findColour (juce::Slider::rotarySliderFillColourId);
        const float angle = startAngle + pos * (endAngle - startAngle);

        g.setColour (juce::Colour (0xff101014));
        g.fillEllipse (juce::Rectangle<float> (r * 1.5f, r * 1.5f).withCentre (c));
        g.setColour (juce::Colour (0xff2e2e36));
        g.drawEllipse (juce::Rectangle<float> (r * 1.5f, r * 1.5f).withCentre (c), 1.5f);

        juce::Path track;
        track.addCentredArc (c.x, c.y, r - 2.0f, r - 2.0f, 0.0f, startAngle, endAngle, true);
        g.setColour (colours::track);
        g.strokePath (track, juce::PathStrokeType (3.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));

        juce::Path value;
        value.addCentredArc (c.x, c.y, r - 2.0f, r - 2.0f, 0.0f, startAngle, angle, true);
        g.setColour (accent);
        g.strokePath (value, juce::PathStrokeType (3.0f, juce::PathStrokeType::curved, juce::PathStrokeType::rounded));

        const auto p1 = c.getPointOnCircumference (r * 0.2f, angle);
        const auto p2 = c.getPointOnCircumference (r * 0.7f, angle);
        g.setColour (juce::Colours::white);
        g.drawLine ({ p1, p2 }, 2.5f);
    }

    void drawToggleButton (juce::Graphics& g, juce::ToggleButton& b, bool highlighted, bool) override
    {
        auto r = b.getLocalBounds().toFloat().reduced (2.0f);
        const bool on = b.getToggleState();
        const auto accent = b.findColour (juce::ToggleButton::tickColourId);

        g.setColour (on ? accent : colours::track.brighter (highlighted ? 0.2f : 0.0f));
        g.fillRoundedRectangle (r, r.getHeight() * 0.5f);
        g.setColour (on ? juce::Colours::black : colours::text);
        g.setFont (font (11.0f, true));
        g.drawText (b.getButtonText(), r, juce::Justification::centred);
    }
};

// ---------------------------------------------------------------------------
class Knob : public juce::Component
{
public:
    Knob (APVTS& state, const juce::String& paramID, const juce::String& name, juce::Colour accent)
    {
        slider.setSliderStyle (juce::Slider::RotaryHorizontalVerticalDrag);
        slider.setTextBoxStyle (juce::Slider::TextBoxBelow, false, 70, 16);
        setAccent (accent);
        addAndMakeVisible (slider);

        label.setText (name, juce::dontSendNotification);
        label.setJustificationType (juce::Justification::centred);
        label.setFont (font (12.0f, true));
        addAndMakeVisible (label);

        attachment = std::make_unique<APVTS::SliderAttachment> (state, paramID, slider);

        if (auto* p = state.getParameter (paramID))
            slider.setDoubleClickReturnValue (true, p->convertFrom0to1 (p->getDefaultValue()));
    }

    void setAccent (juce::Colour c)
    {
        slider.setColour (juce::Slider::rotarySliderFillColourId, c);
        repaint();
    }

    void resized() override
    {
        auto b = getLocalBounds();
        label.setBounds (b.removeFromTop (16));
        slider.setBounds (b);
    }

private:
    juce::Slider slider;
    juce::Label label;
    std::unique_ptr<APVTS::SliderAttachment> attachment;
};

// ---------------------------------------------------------------------------
class VSlider : public juce::Component
{
public:
    VSlider (APVTS& state, const juce::String& paramID, const juce::String& name, juce::Colour accent)
    {
        slider.setSliderStyle (juce::Slider::LinearVertical);
        slider.setTextBoxStyle (juce::Slider::NoTextBox, false, 0, 0);
        slider.setColour (juce::Slider::trackColourId, accent);
        slider.setColour (juce::Slider::thumbColourId, juce::Colours::white);
        slider.setPopupDisplayEnabled (true, true, nullptr);
        addAndMakeVisible (slider);

        label.setText (name, juce::dontSendNotification);
        label.setJustificationType (juce::Justification::centred);
        label.setFont (font (11.0f, true));
        addAndMakeVisible (label);

        attachment = std::make_unique<APVTS::SliderAttachment> (state, paramID, slider);
        slider.setDoubleClickReturnValue (true, 0.0);
    }

    void resized() override
    {
        auto b = getLocalBounds();
        label.setBounds (b.removeFromBottom (16));
        slider.setBounds (b);
    }

private:
    juce::Slider slider;
    juce::Label label;
    std::unique_ptr<APVTS::SliderAttachment> attachment;
};

// ---------------------------------------------------------------------------
// Vertical peak meter (dBFS) with peak hold, clip LED and an optional
// green "target zone" showing the recommended input level.
class LevelMeter : public juce::Component, public juce::SettableTooltipClient, private juce::Timer
{
public:
    static constexpr float minDb = -60.0f, maxDb = 6.0f;
    static constexpr float targetLow = -18.0f, targetHigh = -6.0f;

    LevelMeter (std::function<float()> peakSource, bool showTargetZone)
        : source (std::move (peakSource)), showTarget (showTargetZone)
    {
        setTooltip (showTarget ? "Strum hard: peaks should land in the green zone (-18 to -6 dBFS). Adjust the Input knob."
                               : "Output level. Keep it below 0 dBFS (red LED = clipping).");
        startTimerHz (30);
    }

    void paint (juce::Graphics& g) override
    {
        auto b = getLocalBounds().reduced (2, 0);
        auto text = b.removeFromBottom (16);
        auto led = b.removeFromTop (8).withSizeKeepingCentre (22, 6);
        b.removeFromTop (3);
        auto bar = b.withSizeKeepingCentre (juce::jmin (16, b.getWidth() / 3), b.getHeight()).toFloat();

        // clip LED
        g.setColour (clipTicks > 0 ? colours::red : colours::track);
        g.fillRoundedRectangle (led.toFloat(), 2.0f);

        // track
        g.setColour (juce::Colour (0xff101014));
        g.fillRoundedRectangle (bar, 3.0f);

        // recommended zone
        if (showTarget)
        {
            const auto top = yFor (bar, targetHigh), bottom = yFor (bar, targetLow);
            g.setColour (colours::clean.withAlpha (0.18f));
            g.fillRect (juce::Rectangle<float> (bar.getX() - 4, top, bar.getWidth() + 8, bottom - top));
        }

        // level bar
        if (level > minDb)
        {
            const auto y = yFor (bar, level);
            const auto c = level > -1.0f ? colours::red
                         : level > targetHigh ? colours::yellow
                         : level >= targetLow ? colours::clean
                                              : colours::teal;
            g.setColour (c);
            g.fillRoundedRectangle (bar.withTop (y), 3.0f);
        }

        // peak hold line
        if (holdDb > minDb)
        {
            g.setColour (juce::Colours::white);
            g.fillRect (bar.getX(), yFor (bar, holdDb) - 1.0f, bar.getWidth(), 2.0f);
        }

        // scale
        g.setFont (font (9.0f));
        for (float db : { 0.0f, -6.0f, -18.0f, -36.0f })
        {
            const auto y = yFor (bar, db);
            g.setColour (colours::text.withAlpha (0.5f));
            g.drawHorizontalLine (juce::roundToInt (y), bar.getRight() + 1.0f, bar.getRight() + 5.0f);
            g.drawText (juce::String ((int) db), juce::Rectangle<float> (bar.getRight() + 6.0f, y - 6.0f, 24.0f, 12.0f),
                        juce::Justification::centredLeft);
        }

        // numeric peak-hold readout
        g.setColour (colours::text);
        g.setFont (font (11.0f, true));
        g.drawText (holdDb > minDb ? juce::String (holdDb, 1) + " dB" : juce::String ("-inf"), text,
                    juce::Justification::centred);
    }

    void mouseDown (const juce::MouseEvent&) override
    {
        clipTicks = 0;
        holdDb = minDb;
        repaint();
    }

private:
    float yFor (juce::Rectangle<float> bar, float db) const
    {
        const float n = (juce::jlimit (minDb, maxDb, db) - minDb) / (maxDb - minDb);
        return bar.getBottom() - n * bar.getHeight();
    }

    void timerCallback() override
    {
        const float peak = source ? source() : 0.0f;
        const float db = juce::Decibels::gainToDecibels (peak, minDb);

        level = juce::jmax (db, level - 1.0f); // ~30 dB/s fall
        if (db >= holdDb || --holdTicks <= 0)
        {
            holdDb = db;
            holdTicks = 45; // 1.5 s hold
        }
        if (peak >= 1.0f) clipTicks = 60;
        else if (clipTicks > 0) --clipTicks;

        repaint();
    }

    std::function<float()> source;
    bool showTarget;
    float level = minDb, holdDb = minDb;
    int holdTicks = 0, clipTicks = 0;
};

// ---------------------------------------------------------------------------
class Toggle : public juce::Component
{
public:
    Toggle (APVTS& state, const juce::String& paramID, const juce::String& text, juce::Colour accent)
    {
        button.setButtonText (text);
        button.setColour (juce::ToggleButton::tickColourId, accent);
        addAndMakeVisible (button);
        attachment = std::make_unique<APVTS::ButtonAttachment> (state, paramID, button);
    }

    void resized() override
    {
        button.setBounds (getLocalBounds().withSizeKeepingCentre (juce::jmin (getWidth(), 80), 24));
    }

private:
    juce::ToggleButton button;
    std::unique_ptr<APVTS::ButtonAttachment> attachment;
};

// ---------------------------------------------------------------------------
// A titled panel with an optional on/off switch and a row of controls.
class Section : public juce::Component
{
public:
    Section (const juce::String& titleText, juce::Colour accentColour, APVTS* state = nullptr,
             const juce::String& enableParamID = {})
        : title (titleText), accent (accentColour)
    {
        if (state != nullptr && enableParamID.isNotEmpty())
        {
            power = std::make_unique<juce::ToggleButton> ("ON");
            power->setColour (juce::ToggleButton::tickColourId, accent);
            addAndMakeVisible (*power);
            powerAttachment = std::make_unique<APVTS::ButtonAttachment> (*state, enableParamID, *power);
        }
    }

    template <typename T, typename... Args>
    T* add (Args&&... args)
    {
        auto* c = new T (std::forward<Args> (args)...);
        items.add (c);
        addAndMakeVisible (c);
        return c;
    }

    void setAccent (juce::Colour c)
    {
        accent = c;
        if (power) power->setColour (juce::ToggleButton::tickColourId, c);
        for (auto* item : items)
            if (auto* k = dynamic_cast<Knob*> (item))
                k->setAccent (c);
        repaint();
    }

    juce::Colour getAccent() const { return accent; }
    int getNumItems() const { return items.size(); }

    void paint (juce::Graphics& g) override
    {
        auto r = getLocalBounds().toFloat().reduced (1.0f);
        g.setColour (colours::panel);
        g.fillRoundedRectangle (r, 8.0f);
        g.setColour (accent.withAlpha (0.55f));
        g.drawRoundedRectangle (r, 8.0f, 1.5f);

        g.setColour (accent);
        g.setFont (font (13.0f, true));
        g.drawText (title, getLocalBounds().reduced (10, 6).removeFromTop (20), juce::Justification::centredLeft);
    }

    void resized() override
    {
        auto b = getLocalBounds().reduced (8, 6);
        auto top = b.removeFromTop (22);
        if (power)
            power->setBounds (top.removeFromRight (46).reduced (0, 1));

        if (items.isEmpty())
            return;

        const int w = b.getWidth() / items.size();
        for (auto* item : items)
            item->setBounds (b.removeFromLeft (w).reduced (2, 0));
    }

private:
    juce::String title;
    juce::Colour accent;
    std::unique_ptr<juce::ToggleButton> power;
    std::unique_ptr<APVTS::ButtonAttachment> powerAttachment;
    juce::OwnedArray<juce::Component> items;
};

// ---------------------------------------------------------------------------
class ChannelSelector : public juce::Component
{
public:
    ChannelSelector (APVTS& state, const juce::String& paramID, std::function<void (int)> onChannelChanged)
        : onChange (std::move (onChannelChanged))
    {
        const char* names[] = { "CLEAN", "BLUE", "RED" };
        const juce::Colour cols[] = { colours::clean, colours::blue, colours::red };

        for (int i = 0; i < 3; ++i)
        {
            auto& b = buttons[i];
            b.setButtonText (names[i]);
            b.setClickingTogglesState (true);
            b.setRadioGroupId (515);
            b.setColour (juce::TextButton::buttonOnColourId, cols[i]);
            b.onClick = [this, i]
            {
                if (buttons[i].getToggleState() && attachment != nullptr)
                    attachment->setValueAsCompleteGesture ((float) i);
            };
            addAndMakeVisible (b);
        }

        if (auto* param = state.getParameter (paramID))
        {
            attachment = std::make_unique<juce::ParameterAttachment> (*param, [this] (float v)
            {
                const int idx = juce::jlimit (0, 2, juce::roundToInt (v));
                buttons[idx].setToggleState (true, juce::dontSendNotification);
                if (onChange) onChange (idx);
            });
            attachment->sendInitialUpdate();
        }
    }

    void resized() override
    {
        auto b = getLocalBounds().reduced (2, 4);
        const int h = b.getHeight() / 3;
        for (auto& btn : buttons)
            btn.setBounds (b.removeFromTop (h).reduced (0, 3));
    }

private:
    std::function<void (int)> onChange;
    juce::TextButton buttons[3];
    std::unique_ptr<juce::ParameterAttachment> attachment;
};

} // namespace ui
