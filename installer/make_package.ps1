# Builds the distributable package for Djenty Donkey.
# Run from the project root after build.bat:  powershell -ExecutionPolicy Bypass -File installer\make_package.ps1
$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem

$version  = "0.5.6-beta"
$root     = Split-Path -Parent $PSScriptRoot
$build    = Join-Path $env:USERPROFILE "vab-build\DjentyDonkey_artefacts\Release"
$dist     = Join-Path $root "dist"
$stageDir = Join-Path $dist "DjentyDonkey-$version-Windows"
$iscc     = @("$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe", "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe") |
            Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $iscc) { throw "Inno Setup (ISCC.exe) not found" }

# Inno Setup needs UTF-8 *with BOM* for the Chinese strings
$utf8Bom = New-Object System.Text.UTF8Encoding($true)
foreach ($f in "DjentyDonkey.iss", "README_CN.txt") {
    $p = Join-Path $PSScriptRoot $f
    [IO.File]::WriteAllText($p, [IO.File]::ReadAllText($p), $utf8Bom)
}

# Zip a folder with standard "/" entry separators and UTF-8 names
function New-Zip ([string] $sourceDir, [string] $zipPath, [bool] $includeBaseDir) {
    if (Test-Path $zipPath) { Remove-Item $zipPath }
    $base = if ($includeBaseDir) { Split-Path $sourceDir } else { $sourceDir }
    $zip = [IO.Compression.ZipFile]::Open($zipPath, [IO.Compression.ZipArchiveMode]::Create, [Text.Encoding]::UTF8)
    try {
        Get-ChildItem $sourceDir -Recurse -File | ForEach-Object {
            $name = $_.FullName.Substring($base.Length).TrimStart('\').Replace('\', '/')
            [IO.Compression.ZipFileExtensions]::CreateEntryFromFile($zip, $_.FullName, $name, [IO.Compression.CompressionLevel]::Optimal) | Out-Null
        }
    } finally { $zip.Dispose() }
}

if (Test-Path $stageDir) { Remove-Item $stageDir -Recurse -Force }
New-Item -ItemType Directory -Force $stageDir, "$stageDir\Portable", "$stageDir\source" | Out-Null

# 1) installer
& $iscc /Qp (Join-Path $PSScriptRoot "DjentyDonkey.iss")
if ($LASTEXITCODE -ne 0) { throw "ISCC failed" }
Copy-Item (Join-Path $dist "DjentyDonkey-Setup-$version.exe") $stageDir

# 2) portable
Copy-Item (Join-Path $build "Standalone\Djenty Donkey.exe") "$stageDir\Portable"
Copy-Item (Join-Path $build "VST3\Djenty Donkey.vst3") "$stageDir\Portable" -Recurse

# 3) docs
Copy-Item (Join-Path $PSScriptRoot "README_CN.txt") (Join-Path $stageDir "使用说明.txt")
Copy-Item (Join-Path $PSScriptRoot "THIRD-PARTY-NOTICES.txt") $stageDir

# 4) source code (required by the AGPL/GPL licenses of JUCE and the ASIO SDK)
$srcStage = Join-Path $dist "src-stage\DjentyDonkey-source-$version"
if (Test-Path (Split-Path $srcStage)) { Remove-Item (Split-Path $srcStage) -Recurse -Force }
New-Item -ItemType Directory -Force $srcStage | Out-Null
foreach ($item in "CMakeLists.txt", "build.bat", "README.md", "Source", "tests", "docs", "spice", "installer") {
    Copy-Item (Join-Path $root $item) $srcStage -Recurse
}
# LTspice runs leave waveform / database files next to the netlists (can be GBs); ship the netlists only
Get-ChildItem (Join-Path $srcStage "spice") -Include *.raw, *.db -Recurse | Remove-Item
New-Zip $srcStage "$stageDir\source\DjentyDonkey-source-$version.zip" $true
Remove-Item (Split-Path $srcStage) -Recurse -Force

# 5) final zip
$zip = Join-Path $dist "DjentyDonkey-$version-Windows.zip"
New-Zip $stageDir $zip $true

Get-Item $zip, (Join-Path $dist "DjentyDonkey-Setup-$version.exe") | ForEach-Object { "{0}  {1:N1} MB" -f $_.Name, ($_.Length / 1MB) }
