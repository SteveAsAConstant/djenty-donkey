# 过载块：Precision Drive 与 TS9

插件里的 DRIVE 块可以选两个踏板（MODEL 按钮）：

- **PRECISION**：Horizon Devices Precision Drive。来源是 PedalPCB 公开的 **Dwarven Hammer** 构建文档（克隆板，Rev 12.29.20）
  <https://docs.pedalpcb.com/project/PedalPCB-DwarvenHammer.pdf>
- **TS9**：Ibanez TS9 Tube Screamer。来源是 tonepad 的 **El Griton 2.2** TS-808 / TS-9 布局（rev Nov 28 2004），取 TS-9 的元件值

两者是同一类电路（Tube Screamer 家族），只是元件值不同，所以共用一个模型 `OpAmpDrive`（`Source/dsp/PrecisionDrive.h`），各自带一组元件值（`Circuit`）。

## 拓扑

```
IN → C2 27n (R9) → Q1 2N3904 射极跟随 (R1 510k 偏置, R10 10k) → C3 1µ / R2 10k
   → IC1.1 (JRC4558) 同相削波级
        反馈：(Rf0 + DRIVE A500k) ‖ Cf ‖ D1/D2 反并联 (1N4148 / 1N914)
        对 Vref：R_leg + 支路电容
   → R7 1k / C7 220n / R3 10k
   → IC1.2 音色级：电位器跨在运放两个输入之间，滑臂经 220n + 220Ω 到地，反馈 R8
   → C1 1µ / R4 1k → VOLUME / LEVEL A100k → Q2 2N3904 缓冲 → Ra → C6 10µ → Rb → OUT
```

| 元件 | Precision Drive | TS9 |
|---|---|---|
| Rf0（与 DRIVE 串联） | R15 10k | 51k |
| Cf | C8 47p | 51p |
| 削波级对地支路 | R16 1k + **ATTACK 470n–33n**（6 档）‖ C9 120p | 4.7k + 47n |
| 音色电位器 / 反馈 | BRIGHT B5k / R8 3k9 | TONE B20k / 1k |
| 输出 Ra / Rb | 100Ω / 10k | 470Ω / 100k |
| 最小 / 最大增益（中高频） | 11 / **511 倍** | 12 / 118 倍 |
| 对地支路的低切 | 约 340 Hz–4.8 kHz（ATTACK 可选） | 约 720 Hz |

Precision Drive 更紧：对地支路的电阻只有 1k，ATTACK 选小电容时几乎只放大中高频，而且最大增益是 TS9 的 4 倍多。TS9 是经典的"中频凸起"：低频在削波前就被切掉，音色级把高频收掉，声音更圆润。ATTACK 和真机一样是 6 档：1 = 470n（最饱满），2 = 330n，3 = 100n，4 = 68n，5 = 47n，6 = 33n（最紧）。档位和电容值有两个来源：Neural DSP 的官方模型（Archetype: Misha Mansoor X）和真机 NAM 捕获（见下文）。Dwarven Hammer 克隆板是 8 档，多了 220n 和 82n（旋转开关 1 号脚 470n …… 8 号脚 33n）。TS9 没有 ATTACK，界面上这个旋钮会变暗。

旧版本按克隆板做了 8 档，而且方向反了（1 = 33n …… 8 = 470n）。旧工程和预设加载时会自动换算到同一个电容，声音不变。只有旧的 82n 和 220n 两档真机上没有，分别换到最接近的 68n（4 档）和 330n（2 档）。

两个踏板在 DRIVE 块打开时同时运行，切换型号时做 20 ms 交叉淡化，没有爆音。旧工程和预设没有型号参数，加载后是 PRECISION（和以前一样）。

## 模型

- **削波级**：每个采样用牛顿法求解反馈电压 x = vo − v−。电容用梯形法离散，二极管取 Is = 2.52 nA、n = 1.752，结电容（每只 Cjo 4 pF）并入 Cf
- **运放是真实的 JRC4558**：单极点模型（开环 100 dB，GBW 3 MHz）。它的单位增益极点远高于 Nyquist，所以运放状态用后向欧拉（L 稳定），否则在 fs/2 会留下慢慢衰减的振铃。满 Drive 时闭环带宽只有约 6 kHz，高频比理想运放低 3–5 dB，这是原机真实的声音
- 输出限制在 ±3.5 V（9 V 供电，JRC4558 离电源轨约 1 V）
- **输入**：27n 进 510k ‖ Q1 输入阻抗的高通，再加 1µ / 10k 高通
- **固定损耗**：两个射极跟随器、1k 进 100k 音量电位器、输出 Ra / Rb 分压，合计 PD 约 0.44 dB
- 后面的低通和音色级合并成一个精确的二阶传递函数，再用双线性变换离散化
- 电位器锥度：Drive 和 Precision Drive 的 Volume 为 A（对数，Dwarven Hammer 物料表 A100K），Bright / Tone 为 B（线性），TS9 的 Level 为 B（线性，按 tonepad；真机捕获在 Level 5 和 10 的输出电平只有线性电位器才对得上）
- 原机内置的噪声门（Precision Drive）没有建模，插件前面已经有独立的 Gate
- 2N3904 跟随器的偶次谐波（−55…−62 dB）没有建模，可以忽略

## 与 LTspice 对比

参考网表：`spice/precision_drive.cir` 和 `spice/ts9.cir`（JRC4558 用 LTspice 的 UniversalOpAmp2 level 2：100 dB、3 MHz、1 V/µs、离电源轨 1.5 V）。

| | Precision Drive | TS9 |
|---|---|---|
| 小信号频响（各 Drive / Attack / Tone 组合，20 Hz–20 kHz） | 0.31 dB 以内（以前最大 0.8 dB，高频最大 5 dB） | 0.14 dB 以内 |
| 大信号基波 H1（110 Hz / 1 kHz） | 0.07 dB 以内 | 0.05 dB 以内 |
| 大信号 H3 / H5 | 0.1–0.6 dB | 0.4 dB 以内 |

大信号时，JRC4558 和接近理想的运放结果几乎一样；GBW 主要影响小信号和高 Drive 时的高频。

CPU（4× 过采样，48 kHz）：每个踏板约 1.5% 单核。

## 与真机 NAM 捕获对比

工具：`tests/pedal_compare.cpp`（目标 `pedal_compare`）。在相同输入下比较：
- 小信号频响：29 个 1/3 倍频程单音，电平低到我们的削波级保持线性；
- 饱和：110 Hz 和 1 kHz 正弦，10 mV–1 V 峰值，看基波和 H2–H5；
- 吉他：单音和和弦的 1/3 倍频程频谱。

输入、输出按捕获自带的接口标定（dBu）换算成伏特。`--fit` 搜索最匹配的 Drive / Tone（和 Attack）。能被旋钮消掉的差别是旋钮刻度问题，消不掉的才可能是电路问题。

```
pedal_compare <model.nam> --pedal pd|ts9 --drive d --tone t --level l [--attack n] --in-dbu x [--out-dbu y] [--fit] [--leg-cap nF]
```

捕获来自 TONE3000，许可不允许再分发，仓库里不包含：
- **Precision Drive**：[HB Nam A2 Horizon Devices Precision Drive Boost Pack v2](https://www.tone3000.com/tones/hb-nam-a2-horizon-devices-precision-drive-boost-pack-v2-87039)（hblu2024），5 个设置，标定 +12.6 dBu。
- **TS9（现代版，TA75558）**：[Ibanez TS9 Tube Screamer](https://www.tone3000.com/tones/ibanez-ts9-tube-screamer-86314)（madhatteramplab），5 个设置，输入 16.3 dBu，输出 21.1 dBu。
- **TS9（1981 日本版，JRC4558）**：[IBANEZ TS9 1981 JAPAN PEDAL](https://www.tone3000.com/tones/ibanez-ts9-1981-japan-pedal-80480)（analog），3 个设置，没有标定。

旋钮按捕获说明换算：钟点 7:00 = 0、12:00 = 5、5:00 = 10。

**注意：** 这些 A2 模型用分段线性的 LeakyReLU，小信号时完全没有谐波（H3 显示为 −170 dB），然后在某个电平突然出现。所以"开始削波的位置"没有可比性，饱和只比基波压缩和明显削波时的谐波。

### Precision Drive

**ATTACK 已按真机修正。** 真机三个 ATTACK 档位（1、2、3）的捕获显示：数字越大，低频越少、频响峰越往上移。之前模型按克隆板的元件编号排成 1 = 33n，方向正好反了。

用 `--leg-cap` 逐个电容值扫描，得到的最佳匹配：

| 真机档位 | 最佳电容（频响误差均方根） | 克隆板原理图 | Neural DSP 官方模型 |
|---|---|---|---|
| 1 | 470n（0.34 dB） | 470n | 470n |
| 2 | 330n（0.47 / 0.48 dB） | 330n | 330n |
| 3 | **100n**（0.41 / 0.74 dB） | 220n | **100n** |

第 3 档的捕获和官方模型一致，所以真机是 6 档：470n、330n、100n、68n、47n、33n。

修正后，五个捕获在标称旋钮位置：
- 小信号频响 32 Hz–10 kHz 误差 ±0.9 dB 以内（均方根 0.34–0.74 dB）；
- 吉他频谱到 6 kHz 误差 ±0.6 dB 以内；
- 1 V 输入重度削波时，ATTACK 1 / 2 的 H3 / H5 误差 0.1–0.7 dB。

剩下的差别：
- 12 kHz 以上真机亮 1–3 dB；
- 重度削波时真机输出电平高约 1.3 dB（削波电压高约 15%），属于 1N4148 不同厂家之间的离散。这和输入标定无关：把输入标定从 12.6 dBu 往上调只会更差。

输出电平没有参考价值：五个捕获的输出电平几乎一样，和 Level 旋钮（7–10）无关，像是被归一化过。

### TS9

**Level 电位器是线性的，已修正。** 现代版 TS9 有输出标定：
- 用 A 锥度时，我们在 Level 10 比真机高 3 dB，在 Level 5 低 10–13 dB；
- 换成 B（线性）后，五个设置都高约 3 dB（+1.9 到 +4.5 dB）。

固定 3 dB 的差别属于标定或个体差异，和 tonepad 标的 B100K 一致。

**频响形状：**
- 现代版在标称旋钮位置误差均方根 0.9–1.6 dB，满 Drive 2.5 dB。
- `--fit` 后全部降到 0.4–0.7 dB，所以电路结构没问题，差别在旋钮刻度：这台的 Tone 比标称亮很多，像反对数（C）锥度；Drive 增益偏小。
- 1981 版（和我们一样用 JRC4558）在标称 Tone 位置 5 kHz 以下误差约 ±1 dB，按线性 Tone 就对得上。

两台真机的 Tone 规律不一样，所以保留 tonepad 的 B20K。1981 版的捕获在 5 kHz 以上急剧衰减（16 kHz −44 dB），这是捕获链路造成的，不是踏板本身。

**削波：** 1 V 输入时 H3 / H5 误差 1–3 dB，基波压缩误差 1–2 dB（满 Drive 时更大）。
