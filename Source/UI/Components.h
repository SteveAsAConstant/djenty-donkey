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

    void setLabel (const juce::String& name) { label.setText (name, juce::dontSendNotification); }

    // greyed out and not editable (a control the current mode does not use)
    void setActive (bool active)
    {
        slider.setEnabled (active);
        setAlpha (active ? 1.0f : 0.35f);
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
// Shows how far each noise-gate band is closed (L / M / H).
class GateBandMeter : public juce::Component, public juce::SettableTooltipClient, private juce::Timer
{
public:
    explicit GateBandMeter (std::function<float (int)> bandGainSource) : source (std::move (bandGainSource))
    {
        setTooltip ("Gate activity per band: full bar = open, empty = closed. "
                    "Highs close first to remove hiss, lows close last to keep the sustain.");
        startTimerHz (30);
    }

    void paint (juce::Graphics& g) override
    {
        auto b = getLocalBounds().reduced (2, 0);
        g.setFont (font (12.0f, true));
        g.setColour (colours::text);
        g.drawText ("Bands", b.removeFromTop (16), juce::Justification::centred);
        auto labels = b.removeFromBottom (16);
        b.reduce (0, 6);

        const char* names[] = { "L", "M", "H" };
        const int w = b.getWidth() / 3;
        for (int i = 0; i < 3; ++i)
        {
            auto col = b.removeFromLeft (w).toFloat().withSizeKeepingCentre (10.0f, (float) b.getHeight());
            g.setColour (juce::Colour (0xff101014));
            g.fillRoundedRectangle (col, 3.0f);
            // 0 dB .. -40 dB of gain reduction
            const float db = juce::Decibels::gainToDecibels (gains[i], -40.0f);
            const float n = 1.0f + db / 40.0f;
            g.setColour (colours::teal.withAlpha (0.35f + 0.65f * n));
            g.fillRoundedRectangle (col.withTop (col.getBottom() - n * col.getHeight()), 3.0f);
            g.setColour (colours::text);
            g.drawText (names[i], labels.removeFromLeft (w), juce::Justification::centred);
        }
    }

private:
    void timerCallback() override
    {
        for (int i = 0; i < 3; ++i) gains[i] = source ? source (i) : 1.0f;
        repaint();
    }

    std::function<float (int)> source;
    float gains[3] { 1.0f, 1.0f, 1.0f };
};

// ---------------------------------------------------------------------------
class Toggle : public juce::Component, public juce::SettableTooltipClient
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
// Two or more toggles stacked in one column, each with its own tooltip.
class ToggleStack : public juce::Component
{
public:
    struct Item { juce::String paramID, text, tooltip; };

    // choice: optionally a choice parameter below the toggles, as a drop-down
    ToggleStack (APVTS& state, const std::vector<Item>& items, juce::Colour accent, const Item& choice = {})
    {
        for (const auto& it : items)
        {
            auto* b = buttons.add (new juce::ToggleButton (it.text));
            b->setColour (juce::ToggleButton::tickColourId, accent);
            b->setTooltip (it.tooltip);
            addAndMakeVisible (b);
            attachments.add (new APVTS::ButtonAttachment (state, it.paramID, *b));
        }
        if (auto* p = dynamic_cast<juce::AudioParameterChoice*> (state.getParameter (choice.paramID)))
        {
            combo = std::make_unique<juce::ComboBox>();
            combo->addItemList (p->choices, 1);
            combo->setTooltip (choice.tooltip);
            addAndMakeVisible (*combo);
            comboAttachment = std::make_unique<APVTS::ComboBoxAttachment> (state, choice.paramID, *combo);
            comboLabel.setText (choice.text, juce::dontSendNotification);
            comboLabel.setJustificationType (juce::Justification::centred);
            comboLabel.setFont (font (11.0f, true));
            addAndMakeVisible (comboLabel);
        }
    }

    void resized() override
    {
        auto b = getLocalBounds();
        // natural sizes, shrunk together (down to a floor) when the panel is short (small window)
        int h = 24, gap = 6, labelH = combo ? 16 : 0, comboBoxH = combo ? 24 : 0;
        const int n = buttons.size();
        auto total = [&] { return n * h + (n - 1) * gap + (combo ? gap + labelH + comboBoxH : 0); };
        while (total() > b.getHeight() && h > 16)
        {
            --h;
            if (gap > 2) --gap;
            if (comboBoxH > 18) --comboBoxH;
            if (labelH > 12) --labelH;
        }
        b = b.withSizeKeepingCentre (juce::jmin (b.getWidth(), 92), total());
        for (auto* btn : buttons) { btn->setBounds (b.removeFromTop (h)); b.removeFromTop (gap); }
        if (combo)
        {
            comboLabel.setBounds (b.removeFromTop (labelH));
            combo->setBounds (b.removeFromTop (comboBoxH));
        }
    }

private:
    juce::OwnedArray<juce::ToggleButton> buttons;
    juce::OwnedArray<APVTS::ButtonAttachment> attachments;
    std::unique_ptr<juce::ComboBox> combo;
    std::unique_ptr<APVTS::ComboBoxAttachment> comboAttachment;
    juce::Label comboLabel;
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
    void setTitle (const juce::String& t) { title = t; repaint(); }

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
// Vertical stack of radio buttons bound to a choice parameter.
class ChoiceButtons : public juce::Component, public juce::SettableTooltipClient
{
public:
    ChoiceButtons (APVTS& state, const juce::String& paramID, const juce::String& title,
                   const juce::StringArray& labels, juce::Colour accent)
    {
        static int nextGroup = 1000;
        const int group = ++nextGroup;

        label.setText (title, juce::dontSendNotification);
        label.setJustificationType (juce::Justification::centred);
        label.setFont (font (12.0f, true));
        addAndMakeVisible (label);

        for (int i = 0; i < labels.size(); ++i)
        {
            auto* b = buttons.add (new juce::TextButton (labels[i]));
            b->setClickingTogglesState (true);
            b->setRadioGroupId (group);
            b->setColour (juce::TextButton::buttonOnColourId, accent);
            b->onClick = [this, i]
            {
                if (buttons[i]->getToggleState() && attachment != nullptr)
                    attachment->setValueAsCompleteGesture ((float) i);
            };
            addAndMakeVisible (b);
        }

        if (auto* param = state.getParameter (paramID))
        {
            attachment = std::make_unique<juce::ParameterAttachment> (*param, [this] (float v)
            {
                const int idx = juce::jlimit (0, buttons.size() - 1, juce::roundToInt (v));
                buttons[idx]->setToggleState (true, juce::dontSendNotification);
                current = idx;
                if (onChange) onChange (idx);
            });
            attachment->sendInitialUpdate();
        }
    }

    // called on the message thread whenever the choice changes (UI, automation, preset); setOnChange calls it once
    // with the current choice
    void setOnChange (std::function<void (int)> f)
    {
        onChange = std::move (f);
        if (onChange) onChange (current);
    }

    void resized() override
    {
        auto b = getLocalBounds();
        label.setBounds (b.removeFromTop (16));
        b = b.reduced (4, 6);
        const int h = juce::jmin (30, b.getHeight() / juce::jmax (1, buttons.size()));
        b = b.withSizeKeepingCentre (b.getWidth(), h * buttons.size());
        for (auto* btn : buttons)
            btn->setBounds (b.removeFromTop (h).reduced (0, 2));
    }

private:
    juce::Label label;
    juce::OwnedArray<juce::TextButton> buttons;
    std::unique_ptr<juce::ParameterAttachment> attachment;
    std::function<void (int)> onChange;
    int current = 0;
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
