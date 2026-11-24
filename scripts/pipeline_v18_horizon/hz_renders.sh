#!/bin/bash
# [hz] v18 HORIZON stage-1 render set (and the same views of Orbit f3 for comparison).
# usage: ./hz_renders.sh GROUP OUTDIR [PARAMS NPZ PREFIX]      (inside design/; ../rv.sh waits while 2 Blender processes run)
#   P  product views   hero face34 front side rear rear2 top lidopen
#   C  close-ups       stopkey (closed lid, STOP key) nose spine (band landing on K1) pupil tail haunch (light end on the orbit, base line) joint (K1)
#   S  sections        secK1 (x -200: K1 joint, labyrinth, deck land)  secwheel (x 120: body rail, cover, deck over the chassis rim)
#                      secnose (y -30: nose wall, light line, slot, spine)  secbase (x -150: K2 base step)
# defaults: PARAMS h.json, NPZ h.npz, PREFIX h; Orbit: ./hz_renders.sh P OUT f3.json ../../base3/f3.npz o
D=$(cd "$(dirname "$0")" && pwd); M=$(cd "$D/.." && pwd); R="$M/rv.sh"; PY=$M/venv/bin/python; cd $D
G=$1; O=$2; P=${3:-h.json}; N=${4:-h.npz}; X=${5:-h}; mkdir -p $O; T=$O/tmp_${X}_${G}; RES=${RES:-1600}; SPP=${SPP:-64}
case $G in
P) CUSTOM_VIEWS="rear2=-1.15:-1.0:0.62:-0.06:0.0:0.13:85" $R $P $N $O/$X hero,face34,front,side,rear,rear2,top,lidopen $RES $SPP;;
C) CUSTOM_VIEWS="stopkey=0.24:-0.16:0.46:0.0044:0.0:0.283:220;nose=0.62:-0.40:0.30:0.30:-0.08:0.178:120;spine=0.62:-0.24:0.27:0.316:-0.094:0.184:300;pupil=0.75:-0.08:0.19:0.317:0.0:0.1705:420;tail=-0.66:-0.45:0.30:-0.31:-0.10:0.175:120;haunch=-0.25:-0.75:0.30:-0.05:-0.175:0.16:110;joint=-0.10:-0.42:0.25:-0.20:-0.187:0.180:300" \
     $R $P $N $O/$X stopkey,nose,spine,pupil,tail,haunch,joint $RES $SPP;;
S) $PY section.py $N ${T}_a.npz x -0.20 - --cap-deck > /dev/null
   CLIP=x:-0.20:- HEADLIGHT=30 CUSTOM_VIEWS="secK1=0.05:-0.187:0.184:-0.20:-0.187:0.184:300" $R $P ${T}_a.npz $O/$X secK1 $RES $SPP
   $PY section.py $N ${T}_b.npz x 0.12 - --cap-deck > /dev/null
   CLIP=x:0.12:- HEADLIGHT=30 CUSTOM_VIEWS="secwheel=0.36:-0.165:0.205:0.12:-0.165:0.205:150" $R $P ${T}_b.npz $O/$X secwheel $RES $SPP
   $PY section.py $N ${T}_c.npz y -0.03 + --cap-deck > /dev/null
   CLIPY=-0.03 HEADLIGHT=30 CUSTOM_VIEWS="secnose=0.31:-0.30:0.172:0.31:-0.03:0.172:200" $R $P ${T}_c.npz $O/$X secnose $RES $SPP
   $PY section.py $N ${T}_d.npz x -0.15 - --cap-deck > /dev/null
   CLIP=x:-0.15:- HEADLIGHT=30 CUSTOM_VIEWS="secbase=0.05:-0.188:0.125:-0.15:-0.188:0.125:420" $R $P ${T}_d.npz $O/$X secbase $RES $SPP
   rm -f ${T}_*.npz;;
esac
echo HZRENDERS $G DONE
