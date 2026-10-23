cmake_minimum_required(VERSION 3.22)
project(DjentyDonkey VERSION 0.3.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Static MSVC runtime: the exe / VST3 run on other PCs without installing the VC++ redistributable
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# JUCE is downloaded automatically as a zip (no git required)
include(FetchContent)
FetchContent_Declare(JUCE
    URL https://github.com/juce-framework/JUCE/archive/refs/tags/8.0.4.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(JUCE)

# Steinberg ASIO SDK (official download, dual licensed proprietary / GPLv3) so the
# standalone app can talk to ASIO drivers such as the Neural DSP Quad Cortex.
if(WIN32)
    FetchContent_Declare(asiosdk
        URL https://download.steinberg.net/sdk_downloads/ASIO-SDK_2.3.4_2025-10-15.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(asiosdk)
endif()

juce_add_plugin(DjentyDonkey
    VERSION 0.3.0
    COMPANY_NAME "Djenty Donkey"
    PLUGIN_MANUFACTURER_CODE Djdk
    PLUGIN_CODE Djdn
    FORMATS VST3 Standalone
    PRODUCT_NAME "Djenty Donkey"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    VST3_CATEGORIES Fx Distortion
    COPY_PLUGIN_AFTER_BUILD FALSE)

target_sources(DjentyDonkey PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp)

# Custom standalone app shell (no "mute audio input" - an amp sim needs its input)
target_sources(DjentyDonkey_Standalone PRIVATE Source/StandaloneApp.cpp)

target_include_directories(DjentyDonkey PRIVATE Source)

target_compile_definitions(DjentyDonkey PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
    JUCE_USE_CUSTOM_PLUGIN_STANDALONE_APP=1)

if(WIN32)
    target_compile_definitions(DjentyDonkey PUBLIC JUCE_ASIO=1)
    target_include_directories(DjentyDonkey PRIVATE "${asiosdk_SOURCE_DIR}/common")
endif()

target_link_libraries(DjentyDonkey
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)

# Offline DSP checks against the LTspice reference (no JUCE)
add_executable(dsp_test tests/dsp_test.cpp)
target_include_directories(dsp_test PRIVATE Source)

# Tone regression: current DSP vs the frozen v0.3.0 DSP in tests/baseline (null test + speed)
add_executable(tone_regress tests/tone_regress.cpp)
target_include_directories(tone_regress PRIVATE Source tests)
