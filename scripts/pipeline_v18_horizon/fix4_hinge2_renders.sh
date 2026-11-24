#!/bin/bash
# [fix4-hinge2] hinge renders of the geared lid drive (lid.h4): sections at the hinge plane (gear mesh), through the spring cartridge and
# the damper (closed / 30 deg / open), the drive in 3D with the cover hidden, the open lid. Section caps: section.py (metal grey, dark
# mouldings in the body colour, the lid moulding in the cover colour).
# (view names start with "lid": design_render.py applies LIDDEG only to those)
# usage (inside design/): ./fix4_hinge2_renders.sh PARAMS NPZ OUTDIR [RES] [SPP]      needs ../rv.sh (waits while 2 Blender processes run)
D=$(cd "$(dirname "$0")" && pwd); M=$(cd "$D/.." && pwd); R="$M/rv.sh"; PY=$M/venv/bin/python; cd $D
P=$1; N=$2; O=$(mkdir -p "$3" && cd "$3" && pwd); RES=${4:-1600}; SPP=${5:-64}; T=$O/tmp_h4
sec() { $PY section.py $N ${T}_$1.npz $2 $3 $4 > /dev/null; }      # name axis value sign
YH=-0.054; YS=-0.0422; YD=-0.0338      # hinge plane (tongue / pinion), spring cartridge, damper (left hinge, seen from -y)
CUSTOM_VIEWS="lid75F=0.75:-0.6:0.65:-0.05:0:0.28:100" $R $P $N $O/h4 lid75F $RES $SPP
HIDE=TopCover,CoverKit HEADLIGHT=8 CUSTOM_VIEWS="liddrive=0.03:-0.17:0.38:-0.137:-0.045:0.264:115" LIDDEG=75 $R $P $N $O/h4 liddrive $RES $SPP
HIDE=TopCover,CoverKit,Lid,LidKit,StopKey HEADLIGHT=10 CUSTOM_VIEWS="drive3dB=-0.02:-0.30:0.36:-0.137:-0.045:0.262:150" $R $P $N $O/h4 drive3dB $RES $SPP
sec H y $YH +
for A in 0 30 75; do
  CLIPY=$YH HEADLIGHT=30 LIDDEG=$A CUSTOM_VIEWS="lidgear$A=-0.136:-0.40:0.266:-0.136:$YH:0.266:120" $R $P ${T}_H.npz $O/h4 lidgear$A $RES $SPP
done
CLIPY=$YH HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="lidgearZ=-0.1365:-0.40:0.2635:-0.1365:$YH:0.2635:420" $R $P ${T}_H.npz $O/h4 lidgearZ $RES $SPP
sec S y $YS +
for A in 0 75; do
  CLIPY=$YS HEADLIGHT=30 LIDDEG=$A CUSTOM_VIEWS="lidspring$A=-0.136:-0.40:0.262:-0.136:$YS:0.262:260" $R $P ${T}_S.npz $O/h4 lidspring$A $RES $SPP
done
sec D y $YD +
CLIPY=$YD HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="liddamper=-0.136:-0.40:0.264:-0.136:$YD:0.264:200" $R $P ${T}_D.npz $O/h4 liddamper $RES $SPP
sec X z 0.2613 -
CLIP=z:0.2613:- HEADLIGHT=25 LIDDEG=0 CUSTOM_VIEWS="plan=-0.137:-0.044:0.60:-0.137:-0.044:0.2613:150" $R $P ${T}_X.npz $O/h4 plan $RES $SPP
rm -f ${T}_*.npz
echo FIX4 HINGE RENDERS DONE
