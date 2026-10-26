* UREI 1176LN Rev D (S/N 1239-2331) limiting amplifier, from the factory schematic (manual p. 52).
* Reference for the C++ model Source/dsp/Urei1176.h: same circuit, same device equations
* (BJTs: Is / Bf / Br / Vaf only; JFET: Shichman-Hodges smoothed into subthreshold; diodes: Is / N; junction capacitances as fixed
* capacitors). Transcription and assumptions: docs/1176_schematic_notes.md.
*
* Knobs (.param): ratio 1=4:1 2=8:1 3=12:1 4=20:1 5=all buttons in; att = INPUT attenuation (dB, 0 = full CW);
* outpos = OUTPUT pot rotation (1 = full CW); atk / rel = ATTACK / RELEASE rotation (1 = full CW = fastest);
* gr = 0: GR disable switch S9 (ATTACK full CCW), sidechain input grounded.

* (the test benches set: ratio att outpos atk rel gr freq amp tb bdur tstep stepg q r30s r86 jn)
* calibration (T+C / P.C.B. trims): q = "Q" bias trim position, r30s = R30* (output stage bias), r86 = R86 (LN trim)

* ---- supplies (regulated, ideal) ----
V30 v30 0 30
Vm10 vm10 0 -10

* ---- input: 600 ohm source -> bridged-T attenuator (R1A / R1B = 600 ohm dual pot, R2 / R3 620) -> T1 ----
* sine of peak amp (V) at freq, on from tb for bdur (a tone burst; tb=0, bdur=1e9 for a steady tone), level x stepg from tstep
Bin src 0 V={amp*sin(2*pi*freq*(time-tb))*(time>=tb)*(time<tb+bdur)*if(time<tstep,1,stepg)}
Rsrc src inp 600
.param z0=620 kat={10**(att/20)}
.param r1a={max(z0*(kat-1),1m)} r1b={min(z0/max(kat-1,1u),1G)}
R2 inp bt 620
R3 bt t1p 620
R1A inp t1p {r1a}
R1B bt 0 {r1b}
* T1 (UTC O-12 class, 500:200 ohm = 1.58:1 step-down, 9 H, 91 ohm DCR referred to the primary)
Rt1p t1p t1pi 45
L1p t1pi 0 9
L1s t1si 0 {9/2.5}
K1 L1p L1s 1
Rt1s t1si sec 18.4
R4 sec 0 270

* ---- gain reduction element + preamp ----
R5 sec d 27k
* Q1 (UREI 13-0027, selected N-JFET; 2N5457 data): Shichman-Hodges written symmetrically,
*   Id = beta (f(Vgs-Vto) - f(Vgd-Vto)) (1 + lambda |Vds|),  f(x) = x^2 for x > 0 (level 1),
* with the pinch-off edge smoothed into subthreshold (exponential) conduction: f(x) = (2 n Vt ln(1 + e^(x / 2 n Vt)))^2.
* The 1 dB "Q" bias puts the channel at ~5 uS, deep in that region. Gate junctions as diodes.
.param jb=1.125m jvto=-1.372 jlam=2.3m jvt=0.025865
.func jsp(x) {2*jn*jvt*ln(1+exp(x/(2*jn*jvt)))}
Bq1 d s I={jb*(jsp(V(g,s)-jvto)*jsp(V(g,s)-jvto) - jsp(V(g,d)-jvto)*jsp(V(g,d)-jvto))*(1+jlam*abs(V(d,s)))}
Dgs g s DGJ
Dgd g d DGJ
Cgd1 g d 6p
Cgs1 g s 7p
R10 s 0 10k
C28 m s 50u
C1 d n1 1u
R8 n1 b2 1k
C2 b2 0 27p
R9 n3 b2 560k
Q2 c2 b2 e Q3391
Cbc2 b2 c2 3.6p
R13 p c2 1Meg
R18 e m 180
R84 m 0 180
R85 e f1 150
R86 f1 f {max(r86,1m)}
Q3 c3 c2 e3 Q3391
Cbc3 c2 c3 3.6p
R14 p c3 22k
R11 e3 n3 82
R12 n3 0 910
C5 n3 0 80u
Q14 v30 c3 o Q3391
Cbc14 c3 v30 3.6p
R15 o f 6.8k
R17 v30 p 6.8k
C4 p 0 80u
* LN: gate fed half from the control voltage, half from the feedback node F
C9 f h 0.22u
R6 h g 2.2Meg
C3 h g 200p
R7 g ctrl 2.2Meg
C6 g ctrl 200p
C22 ctrl 0 0.22u
* output gain pot R23 (250k, front panel) after C7
C7 o o2 1u
.param r23=250k
R23a o2 w {max(r23*(1-outpos),1m)}
R23b w 0 {max(r23*outpos,1m)}

* ---- compression ratio deck A: tapped divider to ground, the pressed button's tap to the sidechain ----
R78 o2 ta 56k
R19 ta t12 68k
R20 t12 t8 56k
R21 t8 t4 56k
R22 t4 0 47k
Rtc ta sc 10Meg
.param on4={if(ratio==1|ratio==5,1,0)} on8={if(ratio==2|ratio==5,1,0)}
.param on12={if(ratio==3|ratio==5,1,0)} on20={if(ratio==4|ratio==5,1,0)}
RS1A t4 sc {if(on4,10m,1G)}
RS2A t8 sc {if(on8,10m,1G)}
RS3A t12 sc {if(on12,10m,1G)}
RS4A ta sc {if(on20,10m,1G)}
* S9 (GR disable): normally sc -> C17; disabled: C17 input to ground
RS9a sc scx {if(gr,10m,1G)}
RS9b scx 0 {if(gr,1G,10m)}

* ---- G.R. control amp ----
C17 scx b7 1u
Q7 c7 b7 e7 Q3707
Cbc7 b7 c7 4p
R36 c7 b7 1Meg
R37 b7 0 470k
R38 rb c7 47k
R51 v30 rb 4.7k
C18 rb 0 50u
R39 e7 n41 4.7k
Q8 v30 c7 e8 Q3707
Cbc8 c7 v30 4p
R40 e8 n41 2.4k
R41 n41 0 270
R42 e8 b9 180k
R43 b9 0 47k
R45 b9 0 390k
Q9 c9 b9 e9 Q3707
Cbc9 b9 c9 4p
R47 v30 c9 47k
R48 e9 n50 8.2k
Q10 v30 c9 e10 Q3707
Cbc10 c9 v30 4p
R49 e10 n50 2.4k
R50 n50 0 180
R46 e7 e10 47k

* ---- full-wave rectifier, threshold bias (deck B), attack / release ----
C19 e10 a2 6.8u
D2 a2 k DFD
C20 e8 a3 6.8u
D3 a3 k DFD
R52 a2 m2 47k
R53 a3 m2 47k
C21 m2 0 80u
* "Q" bias (P.C.B.): -10 V -> R35 10k -> R59 2k trim -> R83 (T+C, taken as 0) -> ground, R60 3.9k across to ground;
* the wiper is B2. q = wiper position from the ground end (calibrated: 1.0 dB drop with GR off)
R35 vm10 qa 10k
R60 qa 0 3.9k
R59a qa b2b {max(2k*(1-q),1m)}
R59b b2b 0 {max(2k*q,1m)}
R58 b2b s0 150
R61 s0 s1 470
R62 s1 s2 560
R63 s2 s3 1.5k
R64 s3 vm10 1.5k
RS1B s0 m2 {if(on4,10m,1G)}
RS2B s1 m2 {if(on8,10m,1G)}
RS3B s2 m2 {if(on12,10m,1G)}
RS4B s3 m2 {if(on20,10m,1G)}
* attack: R55 25k (CCW end at K) with R54 470 || C27 .022 from K to the wiper; CW = fastest
.param r55=25k
R55a k w55 {max(r55*atk,1m)}
R55b w55 ctrl {max(r55*(1-atk),1m)}
R54 k w55 470
C27 k w55 0.022u
* release: R56 5M from CTRL to the wiper, R57 270k from the wiper to B2 in parallel with the rest; CW = fastest
.param r56=5Meg
R56a ctrl w56 {max(r56*(1-rel),1m)}
R56b w56 b2b {max(r56*rel,1m)}
R57 w56 b2b 270k

* ---- class-A line amp ----
C8 w b4 0.15u
R24 c4 b4 2.7Meg
R25 b4 0 2.7Meg
R26 l c4 68k
R27 e4 0 1.5k
Q4 c4 b4 e4 Q3391
Cbc4 b4 c4 3.6p
C10 c4 b5 1u
R29 b5 c6 1.2Meg
C11 b5 c6 10p
R30 b5 x1 150k
R30s b5 x1 {r30s}
D1 x1 e4 D914
Q5 l b5 b6 Q3707
Cbc5 b5 l 4p
Q6 c6 b6 e6 Q3053
Cbc6 b6 c6 15p
R34 v30 l 8.2k
C12 l 0 5u
C14 e6 0 0.033u
R32 n32 0 39
R31 e4 x2 70k
C13 x2 n32 270p
R28 e4 slt 22k
R33 c6 x3 560
C15 x3 slt 0.0047u
* T2 (UA-5002: 600 : 150+150 : 600 feedback : 1.3 ohm emitter winding, Sowter 1327 data), 6 H primary.
* Dots: BLK (primary, +30 end), WHT/BLK (emitter), VIO (feedback, grounded), BLU (output).
Rpri c6 brn 40
Lpri v30 brn 6
Lfb 0 sltw 6
Le e6 wbw {6*1.3/600}
Rwe wbw n32 0.2
Rfbw sltw slt 40
Lsa blu yel 1.5
Lsb yel redw 1.5
Rsec redw red 30
Rtie red slt 1m
K2 Lpri Lfb Le Lsa Lsb 1
Rload blu red 600

.model Q3391 NPN(Is=6.734f Bf=400 Br=0.74 Vaf=74)
* Q7-Q10 selected for hFE 250 or above (schematic note 1); Q5 the same type
.model Q3707 NPN(Is=20f Bf=350 Br=1 Vaf=100)
.model Q3053 NPN(Is=50f Bf=120 Br=3 Vaf=100)
.model DGJ D(Is=181.3f N=1)
.model D914 D(Is=2.52n N=1.752)
* FD333 (Fairchild low-leakage, Process 1M): VF ~ 0.72 V at 1 mA
.model DFD D(Is=77p N=1.7)

