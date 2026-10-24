* 12AX7 exactly as the C++ TriodeStage: Koren plate current, Cgk 2.3p, Cgp 2.4p,
* grid current 0.03/2k * ln(1 + exp((Vgk - 0.25) / 0.03)) (Source/dsp/TriodeStage.h gridCurrent)
.SUBCKT 12AX7C P G K
B1 E 0 V=V(P,K)/600*log(1+exp(600*(1/100+V(G,K)/sqrt(300+V(P,K)*V(P,K)))))
RE E 0 1G
B2 P K I=(pwr(V(E),1.4)+pwrs(V(E),1.4))/1060
RCP P K 1G
C1 G K 2.3p
C2 G P 2.4p
BG G K I=0.03/2000*log(1+exp((V(G,K)-0.25)/0.03))
.ENDS 12AX7C
