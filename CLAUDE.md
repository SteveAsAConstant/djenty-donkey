# Djenty Donkey: notes for Claude Code

JUCE VST3 + standalone (Windows) with component-level circuit models: 5150 / 6505 preamp and power amp, Precision
Drive and TS9 pedals, UREI 1176LN Rev D compressor, plus gate, transpose, IR loader and effects. Overview and
signal chain: README.md. The user writes in Chinese; reply in Chinese. Docs in `docs/` are Chinese, README and code
comments English.

## Principles (the user's)

- Physics first: the factory schematic and LTspice are ground truth. Every circuit change gets an LTspice reference
  and a parity check (`spice/`, the `*_test` tools). Do not bend a circuit to fit a capture.
- NAM captures of real gear are only for spectra, saturation and knob calibration; fit only what is genuinely
  unknown (knob laws, levels, a part's value within its physical range) and document it as fitted.
- The feel and response of real hardware matter more than small frequency-response differences.
- High CPU is acceptable; accuracy comes first. Still report CPU (`dsp_test cpu`, `comp1176_test cpu / worst`).
- Signal scale everywhere: 1.0 = 1 V at the guitar jack (0 dBFS = 1 V peak).

## Build

```
cmake --build %USERPROFILE%\vab-build --config Release
```
(`build.bat` configures first; its echo lines still show the old "VibeAmp 515" names.) Build dir is
`%USERPROFILE%\vab-build` (short path, not under %TEMP%). Targets: `DjentyDonkey_All` (VST3 + standalone),
`dsp_test`, `tone_regress`, `nam_compare`, `pedal_compare`, `power_compare`, `amp_path_test`, `comp1176_test`, `host_test`,
`comp1176_nam`, `golden`, `plugin_cpu`, `solve_profile`. Plugin output: `vab-build\DjentyDonkey_artefacts\Release\{Standalone,VST3}`; tests in `vab-build\Release`.

## Tests (run after DSP changes)

- `tone_regress`: must stay at 0 failed (null against the frozen baseline).
- `dsp_test [cpu | fault | noise | tubes | tube | chain | power | transpose | ingate | tables | portsolve]`: LTspice comparisons,
  fault recovery, CPU per module. `dsp_test block8 | switch8 | twocore`: cost per 8-sample block at 48 kHz (hardware
  target), one channel switch in detail, the two-core switch split; pin to a P-core (`start /high /affinity 1000`).
  `block8 <n>`: the same with n-sample blocks. `costs`: one 8-sample block split per stage. `iter8 [comp]`: Newton
  evaluations per sample / block in the amp (or the 1176), worst samples and why. `solve_profile` (own target, built
  with VAMP_PROFILE, `Source/dsp/Profile.h`): rdtsc cycles per phase of the tube solves; compare phases, not totals.
  `portsolve`: the port step's AVX version (PortSolve.h, off by default: measured slower at these sizes) stays
  bit-identical to the scalar one on random circuits.
- `golden write <file>` (built from the last commit) / `golden check <file>`: null test for speed work; a change must
  be bit-identical, or rounding-level only (far below the solver's ~-100 dB own error).
- `comp1176_test [spice | rates | release | null | metal | cpu | worst | stress | hf | retryfail | damped | all]`: 1176 vs
  LTspice and the factory test procedures. `worst`: run at real-time priority for meaningful numbers. `retryfail`:
  forced failures in the coupled A / C solve must not reach the states. `damped`: the line-search Newton fallback
  (SemiNetwork::newtonDamped) forced on every solve and on the DC operating point must reproduce the plain path.
- `host_test [auval | random | ir | all]`: the whole processor driven like a host / auval (rates, block sizes, layouts,
  random parameters; `ir`: IRs loaded from another thread while rendering, user presets restoring their cab). Configure with `-DDD_ASAN=ON` for AddressSanitizer (Windows: the VS `bin\Hostx64\x64` dir must
  be on PATH for the ASan DLL); slow under ASan. The macOS CI runs it.
- `plugin_cpu [ir.wav ...]`: the whole plugin's CPU at 48 kHz / 64-256 buffers, per stage (pin to a P-core; close any
  running standalone first: it competes for the core). `plugin_cpu levels [ir.wav ...]`: output level with each IR vs
  the built-in cab (IRs are loudness-matched to it, `dsp/CabMatch.h`).
- `amp_path_test [knobs | switch | boost [dir] | render <dir> | sustain | prepare]`: the plugin's amp path (AmpSim):
  aliasing and tone vs an ideal path, knob zipper noise, channel switches (relay, relay + 4 ms dip as in the plugin,
  the old 20 ms crossfade; `render <dir>`: the same as WAVs from a played riff, for listening), the drive pedal switch (`boost`: only the
  selected pedal runs; the other starts from rest when faded in, checked against both always running; `dir`: WAVs).
- `nam_compare`, `power_compare`, `pedal_compare`, `comp1176_nam`: against NAM captures (pass the .nam path).
- Captures live in `D:\Downloads From Web\` (6505 lead / rhythm / POW / AMP / PRE, two 1176s). They are T3K-licensed:
  use locally only, never commit, upload or put them in a package.

## LTspice

`"C:\Program Files\ADI\LTspice\LTspice.exe" -b file.cir`, results via `.meas` in the `.log` (outputs are gitignored).
- `.step ... list` values are run in ascending order, whatever order you write them in.
- `-ascii` breaks `.meas` when `.tran` has a start time; use the binary raw for `.meas`, `-ascii` only to dump waves.
- `.four` needs a node: make a B-source for a differential voltage.
- 1176: circuit in `spice/1176ln_revd.inc`, benches `spice/1176_*.cir` (each sets every `.param` the include needs).

## Commits and releases

- Commit trailer: `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.
- Work happens on the local branch `dev`; `main` is fast-forwarded to it. Push only `main`
  (never the working branch).
- Release only when the user asks (or gives explicit approval). Steps:
  1. Version in three places: `CMakeLists.txt` (project and juce_add_plugin), `installer/DjentyDonkey.iss`
     (AppVersion), `installer/make_package.ps1` ($version); commit "vX.Y.Z-beta".
  2. Build `DjentyDonkey_All`, then `powershell -ExecutionPolicy Bypass -File installer\make_package.ps1` (Inno
     Setup) -> `dist/DjentyDonkey-X.Y.Z-beta-Windows.zip` and `dist/DjentyDonkey-Setup-X.Y.Z-beta.exe`.
  3. `git branch -f main HEAD`, `git push origin main`, then
     `gh release create vX.Y.Z-beta --target main --prerelease --title "..." --notes-file notes.md <zip> <setup>`.
     Notes: English, short sections (what changed, what you will hear, still open), as earlier releases.
  4. Local install: the user's copy is in Program Files; run the Setup exe (`/SILENT`), it asks for admin. The user
     also keeps each release zip in `Desktop\donkeys`.
- User guide shipped with the installer: `installer/README_CN.txt` (update it for user-facing changes).

## Where things are

- `Source/dsp/`: `DKCircuit.h` (nodal DK solver), `TriodeNetwork.h` (tubes), `SemiNetwork.h` (BJT / JFET / diode
  ports, DC operating point), `FullPreamp5150.h` / `Preamp5150.h`, `PowerAmp5150.h`, `PrecisionDrive.h`,
  `Urei1176.h` + `Comp1176.h` (compressor), `AmpSim.h` (oversampling, chain).
- `Source/PluginProcessor.*` (chain, parameters), `Source/Parameters.h` (IDs; changing an ID drops old settings),
  `Source/PresetManager.h` (factory presets), `Source/PluginEditor.cpp` + `Source/UI/Components.h`.
- Docs: `docs/5150_schematic_notes.md`, `docs/power_amp.md`, `docs/nam_calibration.md`, `docs/noise.md`,
  `docs/precision_drive.md`, `docs/1176_schematic_notes.md`, `docs/compressor_1176.md`.

## Windows gotchas

- Shell: Git Bash and PowerShell 5.1. Inline `sed` / `perl` replacements containing `\n` inside C string literals
  insert real newlines; use the Edit tool for those.
- Restarting the standalone right after killing it can crash Steinberg's built-in ASIO driver (`asiobtin.dll`):
  close it cleanly and wait a few seconds.
- Screenshots of the standalone: the display is high-DPI, so call `SetProcessDPIAware` before sizing / capturing
  the window (the editor is wider than a DPI-unaware window).
