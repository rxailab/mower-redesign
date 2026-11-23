#!/bin/bash
# One-piece body with snap-on top cover (v17): generate, check, render.  usage: ./run_cov.sh [NAME=a_cov] [RES=1600] [SPP=64]
set -e; cd "$(dirname "$0")"; N=${1:-a_cov}; RES=${2:-1600}; SPP=${3:-64}; B=~/blender-4.2.3-linux-x64/blender; O=$PWD/out_$N; mkdir -p $O
../venv/bin/python design_gen.py $N.json $N.npz 2>$N.log | tee $N.check.json; ../venv/bin/python check_cover.py $N.npz | tee $N.cover.txt
G() { grep -E "Error|Traceback|line [0-9]+|WARNING" || true; }
CV="rear2=-1.15:-1.0:0.62:-0.06:0.0:0.13:85;rearstr=-1.6:0:0.42:-0.1:0:0.14:100;sideup=0.25:-1.3:0.75:0.1:-0.1:0.2:100"
CV="$CV;seamN=0.8:-0.3:0.42:0.29:-0.03:0.195:150;seamT=-0.9:-0.45:0.4:-0.31:-0.08:0.175:150;seamW=0.12:-0.7:0.42:0.12:-0.17:0.225:170;seamR=-0.45:-0.8:0.6:-0.15:-0.19:0.235:150"
CUSTOM_VIEWS="$CV" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N hero,face34,front,rear,side,top,dockhero,rear2,rearstr,sideup,seamN,seamT,seamW,seamR,lidopen,lidhinge $RES $SPP 2>&1 | G
LIFT=0.2 LIFT2=0.42 HEADLIGHT=25 CUSTOM_VIEWS="explode=-1.5:-1.55:0.95:0.0:0.0:0.4:60;explode2=-0.2:-2.2:0.62:0.0:0.0:0.4:70" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N explode,explode2 $RES $SPP 2>&1 | G
HC=TopCover,CoverKit,Lid,LidKit,LidWell
HIDE=$HC CUSTOM_VIEWS="deck=-0.55:-0.75:0.95:-0.05:-0.05:0.22:70;deckN=0.75:-0.3:0.5:0.27:-0.02:0.2:120;deckT=-0.75:-0.35:0.55:-0.28:-0.02:0.215:110;plan=0.0:0.0:2.3:0.0:0.0:0.1:100" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N deck,deckN,deckT,plan $RES $SPP 2>&1 | G
HIDE=Backdrop,Chassis,Wheel_L,Wheel_R,Caster_L,Caster_R,RingDecal_L,RingDecal_R,Skirt,Plinth,NewShell,Mount,Camera,PortSleeves,Controls HEADLIGHT=260 CUSTOM_VIEWS="coverin=-0.15:-0.3:-0.75:-0.02:0.0:0.25:55" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N coverin $RES $SPP 2>&1 | G
../venv/bin/python mount_map.py $O/${N}_plan.png mounts.json $O/fastening_map.png $N.npz cover; ../venv/bin/python sheets_cov.py $O $N
echo RENDERED $O
