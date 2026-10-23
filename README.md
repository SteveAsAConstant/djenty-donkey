# Djenty Donkey 🫏

**A component-level high-gain guitar amp plugin** — 5150 / 6505-style preamp solved circuit-by-circuit, a Precision Drive–style pedal, IR loader and effects. VST3 + standalone for Windows.

![Djenty Donkey](docs/screenshot.png)

> 5150, 6505, Peavey, EVH, Horizon Devices and Precision Drive are trademarks of their respective owners.
> They are only used to describe the circuits that inspired this model. This project is not affiliated with or endorsed by them.

## Download

Grab the latest `DjentyDonkey-x.y.z-Windows.zip` from **[Releases](../../releases)**.

- `DjentyDonkey-Setup-x.y.z.exe` — installer (desktop shortcut + VST3)
- `Portable/` — no install needed
- No VC++ runtime required

Windows SmartScreen may warn because the exe is not code-signed: click *More info → Run anyway*.

## Signal chain

```
Input ─ Gate ─ Transpose ─ Compressor ─┬─ [8x oversampled] ───────────────────────────────┐
                                       │  Precision Drive → 5150 preamp → tone stack → power │
                                       └──────────────────────────────────────────────────┘
      ─ Cabinet IR ─ 10-band EQ ─ Phaser ─ Delay ─ Reverb ─ Output
```

| Block | What it is |
|---|---|
| **Preamp** | Each 12AX7 stage is solved per sample (Koren triode model + Newton–Raphson): coupling caps, cathode bypass caps, grid current and Miller capacitance are real circuit elements, so cold-clipper bias shift ("blocking distortion") happens naturally. |
| **Channels** | **RED** = 5150 lead (V1A→V1B→V2A→V2B→V5B→V5A). **BLUE** = rhythm with crunch. **CLEAN** = 6505 rhythm, *optimized*: bypasses V2A–V5A for headroom (not in the original amp). |
| **Tone stack** | Marshall-style TMB solved by nodal analysis, matches LTspice within 0.01 dB. |
| **Precision Drive** | Op-amp + 1N4148 clipper solved per sample; 8-position *Attack* cap switch (33n–470n) for tightness. Based on the public PedalPCB "Dwarven Hammer" build doc. |
| **Power amp** | V3A recovery + long-tail-pair phase inverter + 2×2 6L6GC push-pull + output transformer + 4×12 speaker impedance, solved simultaneously with a nodal DK solver. Presence and Resonance are the real NFB-loop parts (VR9/C8, VR8/C12). Supply sag, flyback diodes, grid current. Matches LTspice within ~0.5 dB small-signal and ~1 % at clipping. |
| **Cab** | Load any `.wav` IR (drag & drop, prev/next in folder) or use the built-in cab. |
| **3-band gate** | Keyed by the clean input, applied after the amp on LR4 bands (300 Hz / 3 kHz): highs close first/fastest/deepest (kills hiss), lows last/slowest/shallowest (keeps the sustain). |
| **Transpose** | *LIVE*: waveform-aligned WSOLA, ~6 ms attack latency. *HQ*: phase-vocoder time-stretch + resampling with identity phase locking and transient phase reset, ~43 ms (reported to the host). Both accurate to a few cents. |

Validation against LTspice: [docs/spice_reference_lead.md](docs/spice_reference_lead.md), [docs/power_amp.md](docs/power_amp.md).
Circuit notes: [docs/5150_schematic_notes.md](docs/5150_schematic_notes.md), [docs/precision_drive.md](docs/precision_drive.md), [docs/gate_and_transpose.md](docs/gate_and_transpose.md).

## Setting the input level

Strum hard and set **Input** so the meter's peak lands in the **green zone (-18 to -6 dBFS)**. The model treats 0 dBFS as 1 V at the guitar jack, which is roughly what real pickups produce.

## Build from source

Requirements: Windows 10/11 x64, **Visual Studio 2022 Build Tools** (Desktop development with C++), **CMake ≥ 3.22**.
JUCE 8.0.4 and the Steinberg ASIO SDK are downloaded automatically by CMake.

```
build.bat
```

Outputs go to `%USERPROFILE%\vab-build\DjentyDonkey_artefacts\Release\` (short path outside `%TEMP%` on purpose: Windows 260-char limit and MSBuild incremental builds).

- DSP regression tests (no JUCE): `%USERPROFILE%\vab-build\Release\dsp_test.exe` (`dsp_test.exe cpu` for per-module CPU)
- Tone regression against the frozen v0.3.0 DSP and a high-precision reference: `%USERPROFILE%\vab-build\Release\tone_regress.exe`
- Package (installer + portable + source zip, needs [Inno Setup 6](https://jrsoftware.org/isinfo.php)):
  `powershell -ExecutionPolicy Bypass -File installer\make_package.ps1`
- LTspice reference netlists: `spice/` (`LTspice.exe -b <file>.cir`)

## Project layout

```
Source/
  dsp/TriodeStage.h     12AX7 stage, Koren model, Newton solver
  dsp/LinearNetwork.h   small MNA solver for R/C networks (tone stack, gain networks)
  dsp/Preamp5150.h      5150 / 6505 preamp assembled from the schematic
  dsp/PrecisionDrive.h  drive pedal
  dsp/AmpSim.h          oversampling, power amp, channel trims
  PluginProcessor.*     signal chain, parameters, IR loading
  PluginEditor.*, UI/   interface
  StandaloneApp.cpp     standalone shell (input never muted)
tests/dsp_test.cpp      comparisons against LTspice + CPU benchmark
tests/tone_regress.cpp  null / band-level comparison vs tests/baseline/ (frozen v0.3.0 DSP)
spice/                  LTspice reference circuits
installer/              Inno Setup script + packaging
```

## Roadmap

- [x] Component-level phase inverter + 4×6L6 power amp with NFB, sag and output transformer
- [x] 3-band noise gate
- [x] LIVE / HQ pitch shifter
- [ ] Cathode follower (V3B) compression
- [ ] CPU: "eco" mode (lower oversampling for the power amp)
- [ ] Preset browser

## License

**GNU AGPLv3** — see [LICENSE](LICENSE). Built with [JUCE](https://juce.com) (AGPLv3) and the Steinberg ASIO SDK (GPLv3 option); see [installer/THIRD-PARTY-NOTICES.txt](installer/THIRD-PARTY-NOTICES.txt).
ASIO and VST are trademarks of Steinberg Media Technologies GmbH.
