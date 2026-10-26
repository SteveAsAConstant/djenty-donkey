cmake_minimum_required(VERSION 3.22)

# macOS: one universal binary (Apple Silicon + Intel), runs on 10.13 and later.
# Must be set before project(); override on the command line for a single-arch dev build.
if(CMAKE_HOST_APPLE)
    set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "Target architectures")
    set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "Minimum macOS version")
endif()

project(DjentyDonkey VERSION 1.0.1)

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

# Audio Unit (Logic Pro, GarageBand, ...) only exists on macOS
set(DD_FORMATS VST3 Standalone)
if(APPLE)
    list(APPEND DD_FORMATS AU)
endif()

juce_add_plugin(DjentyDonkey
    VERSION 1.0.1
    COMPANY_NAME "Djenty Donkey"
    BUNDLE_ID com.djentydonkey.djentydonkey
    PLUGIN_MANUFACTURER_CODE Djdk
    PLUGIN_CODE Djdn
    FORMATS ${DD_FORMATS}
    PRODUCT_NAME "Djenty Donkey"
    AU_MAIN_TYPE kAudioUnitType_Effect
    # macOS asks the user before an app may record: without this the standalone's guitar input is silent
    MICROPHONE_PERMISSION_ENABLED TRUE
    MICROPHONE_PERMISSION_TEXT "Djenty Donkey needs the audio input to hear your guitar."
    HARDENED_RUNTIME_ENABLED TRUE
    HARDENED_RUNTIME_OPTIONS com.apple.security.device.audio-input
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

# Tone regression: current DSP vs the frozen v0.5 DSP in tests/baseline (null test + speed)
add_executable(tone_regress tests/tone_regress.cpp)
target_include_directories(tone_regress PRIVATE Source tests)

# Comparison against a NAM capture of a real amp (pass the .nam file; captures are not redistributed)
add_executable(nam_compare tests/nam_compare.cpp)
target_include_directories(nam_compare PRIVATE Source)

# The drive pedals against NAM captures of real pedals
add_executable(pedal_compare tests/pedal_compare.cpp)
target_include_directories(pedal_compare PRIVATE Source)

# The power amp against NAM captures of a real power amp (driven at the effects return)
add_executable(power_compare tests/power_compare.cpp)
target_include_directories(power_compare PRIVATE Source)

# The plugin's real amp path (JUCE oversampling) vs an alias-free reference and a NAM capture
juce_add_console_app(amp_path_test PRODUCT_NAME "amp_path_test")
target_sources(amp_path_test PRIVATE tests/amp_path_test.cpp)
target_include_directories(amp_path_test PRIVATE Source tests)
target_compile_definitions(amp_path_test PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
target_link_libraries(amp_path_test PRIVATE juce::juce_dsp juce::juce_recommended_config_flags)

# The 1176LN model against its LTspice reference and the factory test procedures
add_executable(comp1176_test tests/comp1176_test.cpp)
target_include_directories(comp1176_test PRIVATE Source)

# The 1176LN model against NAM captures of real 1176s (pass the .nam file; captures are not redistributed)
add_executable(comp1176_nam tests/comp1176_nam.cpp)
target_include_directories(comp1176_nam PRIVATE Source tests)

# The whole processor driven like a host / auval (rates, block sizes, layouts, random parameters).
# -DDD_ASAN=ON builds it with AddressSanitizer (only this test; the plugin is unaffected).
option(DD_ASAN "Build host_test with AddressSanitizer" OFF)
juce_add_console_app(host_test PRODUCT_NAME "host_test")
target_sources(host_test PRIVATE tests/host_test.cpp Source/PluginProcessor.cpp Source/PluginEditor.cpp)
target_include_directories(host_test PRIVATE Source)
target_compile_definitions(host_test PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0
    JucePlugin_Name="Djenty Donkey" JucePlugin_VersionString="${PROJECT_VERSION}")
target_link_libraries(host_test PRIVATE juce::juce_audio_utils juce::juce_dsp juce::juce_recommended_config_flags)
if(DD_ASAN)
    if(MSVC)
        target_compile_options(host_test PRIVATE /fsanitize=address /Zi)
        target_link_options(host_test PRIVATE /INCREMENTAL:NO /DEBUG)
    else()
        target_compile_options(host_test PRIVATE -fsanitize=address -fno-omit-frame-pointer -g)
        target_link_options(host_test PRIVATE -fsanitize=address)
    endif()
endif()

# Golden render for speed work: golden write <file> from the last commit, golden check <file> after the change
# (bit-identical, or rounding-level only: tests/golden.cpp)
add_executable(golden tests/golden.cpp)
target_include_directories(golden PRIVATE Source)

# The whole plugin's CPU at 48 kHz per buffer size and per stage (plugin_cpu [ir.wav ...]; pin it to a P-core)
juce_add_console_app(plugin_cpu PRODUCT_NAME "plugin_cpu")
target_sources(plugin_cpu PRIVATE tests/plugin_cpu.cpp Source/PluginProcessor.cpp Source/PluginEditor.cpp)
target_include_directories(plugin_cpu PRIVATE Source)
target_compile_definitions(plugin_cpu PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0
    JucePlugin_Name="Djenty Donkey" JucePlugin_VersionString="${PROJECT_VERSION}")
target_link_libraries(plugin_cpu PRIVATE juce::juce_audio_utils juce::juce_dsp juce::juce_recommended_config_flags)
