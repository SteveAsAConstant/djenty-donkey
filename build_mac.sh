#!/bin/sh
# Configure + build Djenty Donkey on macOS (Release, universal arm64 + x86_64). First run downloads JUCE.
# Needs Xcode or the Command Line Tools (xcode-select --install) and CMake >= 3.22 (brew install cmake).
set -e
cd "$(dirname "$0")"
BUILD_DIR="${BUILD_DIR:-build-mac}"

cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" --config Release --parallel "$(sysctl -n hw.ncpu)"

ART="$BUILD_DIR/DjentyDonkey_artefacts/Release"
echo
echo "Done:"
echo "  Standalone: $ART/Standalone/Djenty Donkey.app"
echo "  VST3:       $ART/VST3/Djenty Donkey.vst3"
echo "  AU:         $ART/AU/Djenty Donkey.component"
echo "  DSP tests:  $BUILD_DIR/dsp_test"
echo
echo "Install for this user only (no package):"
echo "  cp -R \"$ART/VST3/Djenty Donkey.vst3\" ~/Library/Audio/Plug-Ins/VST3/"
echo "  cp -R \"$ART/AU/Djenty Donkey.component\" ~/Library/Audio/Plug-Ins/Components/"
