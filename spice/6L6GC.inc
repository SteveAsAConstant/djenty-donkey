* 6L6GC beam tetrode - Norman Koren pentode model (wdv.com parameters), LTspice B-sources
* pins: P G1 K G2
.SUBCKT 6L6GC P G1 K G2
B1 E 0 V=V(G2,K)/48*log(1+exp(48*(1/8.7+V(G1,K)/V(G2,K))))
RE E 0 1G
B2 P K I=(pwr(V(E),1.35)+pwrs(V(E),1.35))/1460*atan(V(P,K)/12)
B3 G2 K I=pwr(max(V(G2,K)/8.7+V(G1,K),0),1.35)/4500
RCP P K 1G
R1 G1 GI 1k
D1 GI K DX
.MODEL DX D(IS=1n RS=1 CJO=10p TT=1n)
.ENDS 6L6GC
