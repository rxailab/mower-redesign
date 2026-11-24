#!/bin/bash
# [fix3-hinge] hinge render set (final_set.sh group B hinge views + close-ups). usage: ./r3.sh PARAMS NPZ TAG [RES SPP]
D=$(cd "$(dirname "$0")/.." && pwd); O=$D/r; mkdir -p $O; P=$1; N=$2; T=$3; RES=${4:-1600}; SPP=${5:-64}; R="$D/rv.sh"; PY=$D/venv/bin/python
# final_set.sh group B views
CUSTOM_VIEWS="lid75F=0.75:-0.6:0.65:-0.05:0:0.28:100" $R $P $N $O/${T} lid75F $RES $SPP
HIDE=TopCover,CoverKit HEADLIGHT=8 CUSTOM_VIEWS="lidhingeX=-0.36:-0.30:0.42:-0.140:-0.050:0.282:150" $R $P $N $O/${T} lidhingeX $RES $SPP
cd $D/design; $PY section.py $N $O/tmp_${T}_secH.npz y -0.054 + > /dev/null; cd $D
CLIPY=-0.054 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="lidhingesec=-0.155:-0.34:0.29:-0.150:-0.054:0.272:170;lidhingesecW=-0.128:-0.40:0.268:-0.128:-0.054:0.262:95" $R $P $O/tmp_${T}_secH.npz $O/${T} lidhingesec,lidhingesecW $RES $SPP
CLIPY=-0.054 HEADLIGHT=30 LIDDEG=75 CUSTOM_VIEWS="lidsecW75=-0.140:-0.40:0.285:-0.140:-0.054:0.272:85" $R $P $O/tmp_${T}_secH.npz $O/${T} lidsecW75 $RES $SPP
# stop detail: cut through the inboard cassette plate (y -49.0), lid at the open stop, seen from inboard
cd $D/design; $PY section.py $N $O/tmp_${T}_secS.npz y -0.0482 - > /dev/null; cd $D
HIDE=HingePod CLIP=y:-0.0482:- HEADLIGHT=30 LIDDEG=75 CUSTOM_VIEWS="lidstop=-0.150:0.24:0.285:-0.149:-0.049:0.266:150;lidstopZ=-0.146:0.24:0.2735:-0.146:-0.049:0.2655:300" $R $P $O/tmp_${T}_secS.npz $O/${T} lidstop,lidstopZ $RES $SPP
# lid underside at the open stop: keel, spine rib, tongue
HEADLIGHT=10 CUSTOM_VIEWS="lidkeel=-0.04:-0.13:0.34:-0.140:-0.054:0.300:60" $R $P $N $O/${T} lidkeel $RES $SPP
rm -f $O/tmp_${T}_sec*.npz
echo R3 DONE $T
