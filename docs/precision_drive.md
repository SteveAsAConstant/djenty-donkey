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
| 削波级对地支路 | R16 1k + **ATTACK 33n–470n**（8 档）‖ C9 120p | 4.7k + 47n |
| 音色电位器 / 反馈 | BRIGHT B5k / R8 3k9 | TONE B20k / 1k |
| 输出 Ra / Rb | 100Ω / 10k | 470Ω / 100k |
| 最小 / 最大增益（中高频） | 11 / **511 倍** | 12 / 118 倍 |
| 对地支路的低切 | 约 340 Hz–4.8 kHz（ATTACK 可选） | 约 720 Hz |

Precision Drive 更紧：对地支路的电阻只有 1k，ATTACK 选小电容时几乎只放大中高频，而且最大增益是 TS9 的 4 倍多。TS9 是经典的"中频凸起"：低频在削波前就被切掉，音色级把高频收掉，声音更圆润。ATTACK 1 = 33n（最紧），ATTACK 8 = 470n（最饱满）。TS9 没有 ATTACK，界面上这个旋钮会变暗。

两个踏板在 DRIVE 块打开时同时运行，切换型号时做 20 ms 交叉淡化，没有爆音。旧工程和预设没有型号参数，加载后是 PRECISION（和以前一样）。

## 模型

- **削波级**：每个采样用牛顿法求解反馈电压 x = vo − v−。电容用梯形法离散，二极管取 Is = 2.52 nA、n = 1.752，结电容（每只 Cjo 4 pF）并入 Cf
- **运放是真实的 JRC4558**：单极点模型（开环 100 dB，GBW 3 MHz）。它的单位增益极点远高于 Nyquist，所以运放状态用后向欧拉（L 稳定），否则在 fs/2 会留下慢慢衰减的振铃。满 Drive 时闭环带宽只有约 6 kHz，高频比理想运放低 3–5 dB，这是原机真实的声音
- 输出限制在 ±3.5 V（9 V 供电，JRC4558 离电源轨约 1 V）
- **输入**：27n 进 510k ‖ Q1 输入阻抗的高通，再加 1µ / 10k 高通
- **固定损耗**：两个射极跟随器、1k 进 100k 音量电位器、输出 Ra / Rb 分压，合计 PD 约 0.44 dB
- 后面的低通和音色级合并成一个精确的二阶传递函数，再用双线性变换离散化
- 电位器锥度：Drive / Volume / Level 为 A（对数），Bright / Tone 为 B（线性）。TS9 的 Level 资料不一致（tonepad 标 B，很多改装资料标 A），这里取 A，转起来更均匀
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
