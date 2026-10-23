* 12AX7 triode - Norman Koren model (1996), LTspice B-source form
* pins: P G K
.SUBCKT 12AX7 P G K
B1 E 0 V=V(P,K)/600*log(1+exp(600*(1/100+V(G,K)/sqrt(300+V(P,K)*V(P,K)))))
RE E 0 1G
B2 P K I=(pwr(V(E),1.4)+pwrs(V(E),1.4))/1060
RCP P K 1G
C1 G K 2.3p
C2 G P 2.4p
C3 P K 0.9p
* grid current (conducts when grid goes positive vs cathode)
R1 G GI 2k
D3 GI K DX
.MODEL DX D(IS=1n RS=1 CJO=10p TT=1n)
.ENDS 12AX7
