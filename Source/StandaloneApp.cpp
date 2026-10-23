// Standalone application shell for Djenty Donkey.
// Same as JUCE's default StandaloneFilterApp, except the "Mute audio input"
// feedback protection is disabled: a guitar amp sim with muted input is silent.
#include <juce_audio_devices/juce_audio_devices.h>
#include <juce_gui_extra/juce_gui_extra.h>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_audio_plugin_client/Standalone/juce_StandaloneFilterWindow.h>

namespace
{
class DjentyStandaloneApp final : public juce::JUCEApplication
{
public:
    DjentyStandaloneApp()
    {
        juce::PropertiesFile::Options options;
        options.applicationName = juce::CharPointer_UTF8 (JucePlugin_Name);
        options.filenameSuffix = ".settings";
        options.osxLibrarySubFolder = "Application Support";
        options.folderName = "";
        appProperties.setStorageParameters (options);
    }

    const juce::String getApplicationName() override           { return juce::CharPointer_UTF8 (JucePlugin_Name); }
    const juce::String getApplicationVersion() override        { return JucePlugin_VersionString; }
    bool moreThanOneInstanceAllowed() override                 { return true; }
    void anotherInstanceStarted (const juce::String&) override {}

    void initialise (const juce::String&) override
    {
        auto holder = std::make_unique<juce::StandalonePluginHolder> (appProperties.getUserSettings(),
                                                                      false,
                                                                      juce::String {},
                                                                      nullptr,
                                                                      juce::Array<juce::StandalonePluginHolder::PluginInOuts> {},
                                                                      false);

        // Never mute the input and hide the toggle / warning banner
        holder->processorHasPotentialFeedbackLoop = false;
        holder->getMuteInputValue().setValue (false);

        if (juce::Desktop::getInstance().getDisplays().displays.isEmpty())
        {
            pluginHolder = std::move (holder);
            return;
        }

        mainWindow = std::make_unique<juce::StandaloneFilterWindow> (
            getApplicationName(),
            juce::LookAndFeel::getDefaultLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId),
            std::move (holder));

        mainWindow->setVisible (true);
    }

    void shutdown() override
    {
        pluginHolder = nullptr;
        mainWindow = nullptr;
        appProperties.saveIfNeeded();
    }

    void systemRequestedQuit() override
    {
        if (pluginHolder != nullptr)
            pluginHolder->savePluginState();

        if (mainWindow != nullptr)
            mainWindow->pluginHolder->savePluginState();

        if (juce::ModalComponentManager::getInstance()->cancelAllModalComponents())
        {
            juce::Timer::callAfterDelay (100, []
            {
                if (auto app = juce::JUCEApplicationBase::getInstance())
                    app->systemRequestedQuit();
            });
        }
        else
        {
            quit();
        }
    }

private:
    juce::ApplicationProperties appProperties;
    std::unique_ptr<juce::StandaloneFilterWindow> mainWindow;
    std::unique_ptr<juce::StandalonePluginHolder> pluginHolder;
};
} // namespace

juce::JUCEApplicationBase* juce_CreateApplication() { return new DjentyStandaloneApp(); }
