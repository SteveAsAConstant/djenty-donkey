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
| **Preamp** | Each 12AX7 stage is solved per sample (Koren triode model + Newton–Raphson): coupling caps, cathode bypass caps, grid current and Miller capacitance are real circuit elements, so cold-clipper bias shift ("blocking distortion") happens naturally. In the lead channel the whole preamp (V1A → V5A with every network between them) is solved as one circuit, so each stage loads the ones next to it: that sets the even harmonics at the edge of breakup and the cold clipper's bias shift (matches LTspice within ~0.5 dB at every plate). Knob moves are smoothed (~20 ms): zipper noise 15–22 dB lower, the same at any buffer size. |
| **Channels** | **RED** = 5150 lead (V1A→V1B→V2A→V2B→V5B→V5A). **BLUE** = rhythm with crunch. **CLEAN** = 6505 rhythm, *optimized*: bypasses V2A–V5A for headroom (not in the original amp). |
| **Tone stack** | The factory network as drawn, including its feed (R89 470k into R24 33k and C59/R94), the 570 pF treble cap, the mid pot wired as a divider and the 500k Post-pot load; solved by nodal analysis, matches LTspice within 0.01 dB. |
| **Precision Drive** | Op-amp + 1N4148 clipper solved per sample; 8-position *Attack* cap switch (33n–470n) for tightness. Based on the public PedalPCB "Dwarven Hammer" build doc. |
| **Power amp** | V3A recovery + long-tail-pair phase inverter + 2×2 6L6GC push-pull + output transformer + 4×12 speaker impedance, solved simultaneously with a nodal DK solver. Presence and Resonance are the real NFB-loop parts (VR9/C8, VR8/C12). Flyback diodes, grid current. Matches LTspice within 0.05 dB small-signal and 0.2 % at clipping. |
| **Supply** | The factory high-voltage chain (reservoir → screen → phase inverter → preamp rails through R59 4k, R60 22k and R61 10k), every node sagging with the real current of the stages it feeds: the preamp sits at ~306 / 322 V and breathes with the power amp. Within 0.01 V of LTspice. |
| **Cab** | Load any `.wav` IR (drag & drop, prev/next in folder) or use the built-in cab. |
| **3-band gate** | Keyed by the clean input, applied after the amp on LR4 bands (300 Hz / 3 kHz): highs close first/fastest/deepest (kills hiss), lows last/slowest/shallowest (keeps the sustain). |
| **Transpose** | *LIVE*: waveform-aligned WSOLA, ~6 ms attack latency. *HQ*: phase-vocoder time-stretch + resampling with identity phase locking and transient phase reset, ~43 ms (reported to the host). Both accurate to a few cents. |

Validation against a real amp (v0.4): 14 [NAM](https://github.com/sdatkinson/neural-amp-modeler) captures of a Peavey 6505 (lead channel, various gain / EQ settings) — with the knobs at the capture's dial positions the odd harmonics match within ~2–3 dB and the knobs' effect on the spectrum within a fraction of a dB. The 1/3-octave spectra differ by ~2 dB on average, almost all of it one fixed, knob-independent curve (+3 dB highs, −2.5 dB mids) that looks like the capture chain's colouring. The model keeps the factory circuit rather than fitting it. See [docs/nam_calibration.md](docs/nam_calibration.md).

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
  dsp/TriodeNetwork.h   N triodes + their circuit solved together (DK method, Newton)
  dsp/LeadPreamp5150.h  the whole lead preamp as one circuit
  dsp/LateStages5150.h  V2B -> V5B -> V5A solved together (BLUE channel)
  dsp/DKCircuit.h       nodal DK solver (power amp, coupled preamp stages)
  dsp/PowerAmp5150.h    phase inverter + 6L6 power amp + NFB
  dsp/PrecisionDrive.h  drive pedal
  dsp/AmpSim.h          oversampling, power amp, channel trims
  PluginProcessor.*     signal chain, parameters, IR loading
  PluginEditor.*, UI/   interface
  StandaloneApp.cpp     standalone shell (input never muted)
tests/dsp_test.cpp      comparisons against LTspice, fault recovery, CPU benchmark (dsp_test fault / cpu)
tests/nam_compare.cpp   real-amp comparison with NAM captures (--sweep, --fit, --fit-pre)
tests/tone_regress.cpp  null / band-level comparison vs tests/baseline/ (frozen v0.3.0 DSP)
spice/                  LTspice reference circuits
installer/              Inno Setup script + packaging
```

## Roadmap

- [x] Component-level phase inverter + 4×6L6 power amp with NFB, sag and output transformer
- [x] 3-band noise gate
- [x] LIVE / HQ pitch shifter
- [x] Cathode follower (V3B): checked on the schematic, it sits after the Post pot (FX send), linear at normal Master settings
- [x] Gain / Master / tone path calibrated against NAM captures of a real 6505
- [x] Even harmonics at the edge of breakup vs the real amp (coupled V2B–V5A solve, Gain recalibrated on 14 captures)
- [x] Lead channel: whole preamp solved as one circuit; knob smoothing
- [x] Tone-stack feed network and V5A load as on the factory schematic; preamp B+ supply chain and sag
- [x] V3A recovery stage inside the power-amp circuit, with the 12AX7 interelectrode capacitances (LTspice within 0.05 dB)
- [ ] CPU: "eco" mode / hardware port (Raspberry Pi class)
- [ ] Preset browser

## License

**GNU AGPLv3** — see [LICENSE](LICENSE). Built with [JUCE](https://juce.com) (AGPLv3) and the Steinberg ASIO SDK (GPLv3 option); see [installer/THIRD-PARTY-NOTICES.txt](installer/THIRD-PARTY-NOTICES.txt).
ASIO and VST are trademarks of Steinberg Media Technologies GmbH.
