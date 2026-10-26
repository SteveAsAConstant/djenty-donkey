#!/bin/sh
# Builds the distributable macOS package for Djenty Donkey.
# Run from the project root after build_mac.sh:  sh installer/make_package_mac.sh
#
# Code signing: ad-hoc by default (runs locally; downloaded copies need right-click -> Open once, see
# README_MAC_CN.txt). With an Apple Developer account set
#   CODESIGN_IDENTITY="Developer ID Application: ..."  INSTALLER_IDENTITY="Developer ID Installer: ..."
# and notarize the resulting .pkg with xcrun notarytool.
set -e

VERSION="${VERSION:-1.0.0}"
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build-mac}"
ART="$BUILD_DIR/DjentyDonkey_artefacts/Release"
DIST="$ROOT/dist"
STAGE="$DIST/DjentyDonkey-$VERSION-macOS"
WORK="$DIST/mac-work"
IDENTITY="${CODESIGN_IDENTITY:--}"
APP="Djenty Donkey.app"
VST3="Djenty Donkey.vst3"
AU="Djenty Donkey.component"

for b in "Standalone/$APP" "VST3/$VST3" "AU/$AU"; do
    [ -d "$ART/$b" ] || { echo "missing $ART/$b (run build_mac.sh first)"; exit 1; }
done

rm -rf "$STAGE" "$WORK"
mkdir -p "$STAGE/Portable" "$STAGE/source" "$WORK/app" "$WORK/vst3" "$WORK/au/root" "$WORK/au/scripts"

cp -R "$ART/Standalone/$APP" "$WORK/app/"
cp -R "$ART/VST3/$VST3" "$WORK/vst3/"
cp -R "$ART/AU/$AU" "$WORK/au/root/"

# 1) sign (hardened runtime; the standalone needs the audio-input entitlement to hear the guitar)
cat > "$WORK/app.entitlements" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict><key>com.apple.security.device.audio-input</key><true/></dict></plist>
EOF
TS=""; [ "$IDENTITY" != "-" ] && TS="--timestamp"
codesign --force --sign "$IDENTITY" $TS --options runtime --entitlements "$WORK/app.entitlements" "$WORK/app/$APP"
codesign --force --sign "$IDENTITY" $TS --options runtime "$WORK/vst3/$VST3"
codesign --force --sign "$IDENTITY" $TS --options runtime "$WORK/au/root/$AU"
for b in "$WORK/app/$APP" "$WORK/vst3/$VST3" "$WORK/au/root/$AU"; do codesign --verify --strict "$b"; done

# 2) installer: one component package per format, so the user can untick what they do not want
component_pkg() # <root> <id> <install location> <out> [scripts dir]
{
    pkgbuild --analyze --root "$1" "$WORK/components.plist" >/dev/null
    # never "relocate" into an older copy found elsewhere on the disk
    /usr/libexec/PlistBuddy -c "Set :0:BundleIsRelocatable false" "$WORK/components.plist"
    if [ -n "$5" ]; then
        pkgbuild --root "$1" --component-plist "$WORK/components.plist" --identifier "$2" --version "$VERSION" \
                 --install-location "$3" --scripts "$5" "$4"
    else
        pkgbuild --root "$1" --component-plist "$WORK/components.plist" --identifier "$2" --version "$VERSION" \
                 --install-location "$3" "$4"
    fi
}

# Logic / GarageBand cache Audio Units: make them rescan
cat > "$WORK/au/scripts/postinstall" <<'EOF'
#!/bin/sh
killall -9 AudioComponentRegistrar 2>/dev/null || true
exit 0
EOF
chmod +x "$WORK/au/scripts/postinstall"

component_pkg "$WORK/app"     com.djentydonkey.djentydonkey.app  /Applications                     "$WORK/app.pkg"
component_pkg "$WORK/vst3"    com.djentydonkey.djentydonkey.vst3 /Library/Audio/Plug-Ins/VST3       "$WORK/vst3.pkg"
component_pkg "$WORK/au/root" com.djentydonkey.djentydonkey.au   /Library/Audio/Plug-Ins/Components "$WORK/au.pkg" "$WORK/au/scripts"

cat > "$WORK/distribution.xml" <<EOF
<?xml version="1.0" encoding="utf-8"?>
<installer-gui-script minSpecVersion="2">
    <title>Djenty Donkey $VERSION</title>
    <options customize="allow" require-scripts="false" hostArchitectures="arm64,x86_64"/>
    <domains enable_localSystem="true"/>
    <os-version min="10.13"/>
    <choices-outline>
        <line choice="app"/>
        <line choice="vst3"/>
        <line choice="au"/>
    </choices-outline>
    <choice id="app" title="Standalone app" description="Djenty Donkey.app in /Applications">
        <pkg-ref id="com.djentydonkey.djentydonkey.app"/>
    </choice>
    <choice id="vst3" title="VST3 plug-in" description="/Library/Audio/Plug-Ins/VST3 (Cubase, Reaper, Ableton Live, Studio One, ...)">
        <pkg-ref id="com.djentydonkey.djentydonkey.vst3"/>
    </choice>
    <choice id="au" title="Audio Unit plug-in" description="/Library/Audio/Plug-Ins/Components (Logic Pro, GarageBand, ...)">
        <pkg-ref id="com.djentydonkey.djentydonkey.au"/>
    </choice>
    <pkg-ref id="com.djentydonkey.djentydonkey.app" version="$VERSION">app.pkg</pkg-ref>
    <pkg-ref id="com.djentydonkey.djentydonkey.vst3" version="$VERSION">vst3.pkg</pkg-ref>
    <pkg-ref id="com.djentydonkey.djentydonkey.au" version="$VERSION">au.pkg</pkg-ref>
</installer-gui-script>
EOF

PKG="$STAGE/DjentyDonkey-$VERSION.pkg"
if [ -n "$INSTALLER_IDENTITY" ]; then
    productbuild --distribution "$WORK/distribution.xml" --package-path "$WORK" --sign "$INSTALLER_IDENTITY" "$PKG"
else
    productbuild --distribution "$WORK/distribution.xml" --package-path "$WORK" "$PKG"
fi

# 3) portable (the signed bundles, to copy by hand)
cp -R "$WORK/app/$APP" "$WORK/vst3/$VST3" "$WORK/au/root/$AU" "$STAGE/Portable/"

# 4) docs
cp "$ROOT/installer/README_MAC_CN.txt" "$STAGE/安装说明-macOS.txt"
cp "$ROOT/installer/README_CN.txt" "$STAGE/使用说明.txt"
cp "$ROOT/installer/THIRD-PARTY-NOTICES.txt" "$STAGE/"

# 5) source code (required by the AGPL license of JUCE)
SRC="$WORK/DjentyDonkey-source-$VERSION"
mkdir -p "$SRC"
for item in CMakeLists.txt build.bat build_mac.sh README.md LICENSE Source tests docs spice installer; do
    cp -R "$ROOT/$item" "$SRC/"
done
# LTspice runs leave waveform / database files next to the netlists (can be GBs); ship the netlists only
find "$SRC/spice" \( -name '*.raw' -o -name '*.db' -o -name '*.log' \) -delete
(cd "$WORK" && zip -qry "$STAGE/source/DjentyDonkey-source-$VERSION.zip" "DjentyDonkey-source-$VERSION")

# 6) final zip (ditto keeps the bundles' signatures and symlinks intact)
ZIP="$DIST/DjentyDonkey-$VERSION-macOS.zip"
rm -f "$ZIP"
ditto -c -k --sequesterRsrc --keepParent "$STAGE" "$ZIP"
rm -rf "$WORK"

ls -lh "$ZIP" "$PKG"
