# Precision Drive 电路

来源：PedalPCB 公开的 **Dwarven Hammer** 构建文档（Horizon Devices Precision Drive 的克隆板，Rev 12.29.20）
<https://docs.pedalpcb.com/project/PedalPCB-DwarvenHammer.pdf>

## 拓扑

```
IN → C2 27n (R9 1M) → Q1 2N3904 射极跟随 (R1 510k 偏置, R10 10k) → C3 1µ / R2 10k
   → IC1.1 (JRC4558) 同相削波级
        反馈：(R15 10k + DRIVE A500k) ‖ C8 47p ‖ D1/D2 1N4148 反并联
        对地：R16 1k + ATTACK 电容 (C10–C17: 33n 47n 68n 82n 100n 220n 330n 470n) ‖ C9 120p
   → R7 1k / C7 220n / R3 10k (低通 ≈ 795Hz)
   → IC1.2 Bright 级：B5k 电位器，C5 220n + R12 220Ω，R8 3k9
   → C1 1µ / R4 1k → VOLUME A100k → Q2 2N3904 缓冲 → C6 10µ → OUT
```

## 为什么它比 TS9 紧

| | TS9 | Precision Drive |
|---|---|---|
| 削波级对地支路 | 4.7k + 47n → 约 720Hz 起切低频 | **1k + 33n~470n** → 约 340Hz~4.8kHz 可选（ATTACK 旋钮） |
| 最大增益 | 1 + 551k/4.7k ≈ 118 倍 | 1 + 510k/1k ≈ **511 倍** |
| 最小增益 | 1 + 51k/4.7k ≈ 12 倍 | 1 + 10k/1k ≈ 11 倍 |

ATTACK 1 = 33n（最紧，几乎只放大中高频），ATTACK 8 = 470n（最饱满）。

## 模型（`Source/dsp/PrecisionDrive.h`）

- 削波级：反馈电压 vf 满足 `Cf·dvf/dt = i_leg − vf/Rf − 2·Is·sinh(vf/nVt)`，每个采样用梯形法加牛顿迭代求解；1N4148 参数取 Is = 2.52nA，n = 1.752
- 运放理想化，输出限制在 ±3.5V（9V 供电）
- 后面的低通和 Bright 级合并成一个精确的二阶传递函数，再用双线性变换离散化
- 电位器锥度：Drive / Volume 为 A（对数），Bright 为 B（线性）
- 原机内置的噪声门没有建模（插件前面已经有独立的 Gate）
