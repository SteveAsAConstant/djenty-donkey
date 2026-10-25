; Inno Setup script for Djenty Donkey (Windows x64)
; Build: installer\make_package.ps1  (or ISCC.exe installer\DjentyDonkey.iss)

#define AppName "Djenty Donkey"
#define AppVersion "0.5.6-beta"
#define AppExe "Djenty Donkey.exe"
#define BuildDir GetEnv("USERPROFILE") + "\vab-build\DjentyDonkey_artefacts\Release"

[Setup]
AppId={{8F3C2B1A-5D4E-4F6A-9B7C-1D2E3F4A5B6C}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher=Djenty Donkey
DefaultDirName={autopf}\{#AppName}
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
OutputDir=..\dist
OutputBaseFilename=DjentyDonkey-Setup-{#AppVersion}
Compression=lzma2
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
PrivilegesRequired=admin
WizardStyle=modern
UninstallDisplayName={#AppName}
UninstallDisplayIcon={app}\{#AppExe}
CloseApplications=yes

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"
Name: "vst3"; Description: "Install the VST3 plugin for DAWs (Reaper, Cubase, FL Studio, ...)"; GroupDescription: "Plugin:"

[Files]
Source: "{#BuildDir}\Standalone\{#AppExe}"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#BuildDir}\VST3\Djenty Donkey.vst3\*"; DestDir: "{commoncf64}\VST3\Djenty Donkey.vst3"; Flags: ignoreversion recursesubdirs createallsubdirs; Tasks: vst3
Source: "README_CN.txt"; DestDir: "{app}"; DestName: "使用说明.txt"; Flags: ignoreversion
Source: "THIRD-PARTY-NOTICES.txt"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{group}\使用说明"; Filename: "{app}\使用说明.txt"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "{cm:LaunchProgram,{#AppName}}"; Flags: nowait postinstall skipifsilent
