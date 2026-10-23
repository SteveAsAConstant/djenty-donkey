# Djenty Donkey 🫏

**A component-level high-gain guitar amp plugin** — 5150 / 6505-style preamp solved circuit-by-circuit, a Precision Drive–style pedal, IR loader and effects. VST3 + standalone for Windows.

**元件级模拟的高增益吉他箱头插件**：参照 5150 / 6505 原理图逐个元件求解的前级、Precision Drive 风格的 Drive、IR 加载和周边效果。Windows VST3 + 独立程序。

![Djenty Donkey](docs/screenshot.png)

> 5150, 6505, Peavey, EVH, Horizon Devices and Precision Drive are trademarks of their respective owners.
> They are only used to describe the circuits that inspired this model. This project is not affiliated with or endorsed by them.

## Download / 下载

Grab the latest `DjentyDonkey-x.y.z-Windows.zip` from **[Releases](../../releases)**.
到 **[Releases](../../releases)** 下载最新的 `DjentyDonkey-x.y.z-Windows.zip`：

- `DjentyDonkey-Setup-x.y.z.exe` — installer (desktop shortcut + VST3) / 安装版（自动建桌面快捷方式、安装 VST3）
- `Portable/` — no install needed / 免安装版
- No VC++ runtime required / 不需要额外安装运行库

Windows SmartScreen may warn because the exe is not code-signed: click *More info → Run anyway*.
程序没有数字签名，Windows 可能会提示，点"更多信息 → 仍要运行"即可。

## Signal chain / 信号链

```
Input ─ Gate ─ Transpose ─ Compressor ─┬─ [4x oversampled] ───────────────────────────────┐
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
| **Power amp** | Simplified for now (presence/resonance + push-pull saturation). Component-level PI + 6L6 is on the roadmap. |
| **Cab** | Load any `.wav` IR (drag & drop, prev/next in folder) or use the built-in cab. |

Validation against LTspice: [docs/spice_reference_lead.md](docs/spice_reference_lead.md).
Circuit notes: [docs/5150_schematic_notes.md](docs/5150_schematic_notes.md), [docs/precision_drive.md](docs/precision_drive.md).

## Setting the input level / 输入电平怎么调

Strum hard and set **Input** so the meter's peak lands in the **green zone (-18 to -6 dBFS)**. The model treats 0 dBFS as 1 V at the guitar jack, which is roughly what real pickups produce.

用力扫弦，调 **Input** 让电平表峰值落在**绿色区域（-18 ~ -6 dBFS）**。

## Build from source / 从源码编译

Requirements: Windows 10/11 x64, **Visual Studio 2022 Build Tools** (Desktop development with C++), **CMake ≥ 3.22**.
JUCE 8.0.4 and the Steinberg ASIO SDK are downloaded automatically by CMake.

```
build.bat
```

Outputs go to `%USERPROFILE%\vab-build\DjentyDonkey_artefacts\Release\` (short path outside `%TEMP%` on purpose: Windows 260-char limit and MSBuild incremental builds).

- DSP regression tests (no JUCE): `%USERPROFILE%\vab-build\Release\dsp_test.exe`
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
spice/                  LTspice reference circuits
installer/              Inno Setup script + packaging
```

## Roadmap

- [ ] Component-level phase inverter + 4×6L6 power amp with NFB, sag and output transformer
- [ ] Cathode follower (V3B) compression
- [ ] CPU optimisation (lookup tables, fewer Newton iterations)
- [ ] Preset browser
- [ ] Better pitch shifter for chords

## License

**GNU AGPLv3** — see [LICENSE](LICENSE). Built with [JUCE](https://juce.com) (AGPLv3) and the Steinberg ASIO SDK (GPLv3 option); see [installer/THIRD-PARTY-NOTICES.txt](installer/THIRD-PARTY-NOTICES.txt).
ASIO and VST are trademarks of Steinberg Media Technologies GmbH.
