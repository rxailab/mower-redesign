#!/bin/bash
# One-piece body (v16): generate, check, render all views.  usage: ./run_one.sh [NAME=a_one] [RES=1600] [SPP=64]
set -e; cd "$(dirname "$0")"; N=${1:-a_one}; RES=${2:-1600}; SPP=${3:-64}; B=~/blender-4.2.3-linux-x64/blender; O=$PWD/out_$N; mkdir -p $O
../venv/bin/python design_gen.py $N.json $N.npz 2>$N.log | tee $N.check.json
G() { grep -E "Error|Traceback|line [0-9]+|WARNING" || true; }
CV="rear2=-1.15:-1.0:0.62:-0.06:0.0:0.13:85;rearstr=-1.6:0:0.42:-0.1:0:0.14:100;rearlow=-1.25:-0.75:0.32:-0.1:0:0.13:85"
CV="$CV;slotL=-0.9:-0.5:0.32:-0.325:-0.10:0.157:170;valL=-0.85:-0.75:0.22:-0.29:-0.16:0.09:150;wellA=-0.25:-0.42:0.52:-0.152:-0.152:0.272:230"
CV="$CV;wellB=0.177:-0.5:0.42:0.177:-0.161:0.232:230;wellN=0.75:-0.28:0.5:0.27:-0.04:0.21:130;plan=0.0:0.0:2.3:0.0:0.0:0.1:100;haunch=-0.9:-1.0:0.5:-0.06:-0.19:0.12:110"
CUSTOM_VIEWS="$CV" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N hero,face34,front,rear,side,top,dockhero,rear2,rearstr,rearlow,tailclose,slotL,valL,wellA,wellB,wellN,plan,haunch,lidopen,lidhinge,lidunder,lidside,lidmech $RES $SPP 2>&1 | G
LIFT=0.27 HEADLIGHT=25 CUSTOM_VIEWS="explode=-1.3:-1.35:0.7:0.0:0.0:0.3:60;explode2=-0.2:-1.9:0.5:0.0:0.0:0.3:75" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N explode,explode2 $RES $SPP 2>&1 | G
HH=Backdrop,Chassis,Wheel_L,Wheel_R,Caster_L,Caster_R,RingDecal_L,RingDecal_R,Skirt,Plinth
HIDE=$HH HEADLIGHT=260 CUSTOM_VIEWS="under=-0.15:-0.3:-0.95:-0.02:0.0:0.2:55" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N under $RES $SPP 2>&1 | G
HIDE=$HH HEADLIGHT=32 CUSTOM_VIEWS="tailin=0.02:0.06:-0.16:-0.32:0.0:0.115:35" $B -b --factory-startup -P design_render.py -- $N.json $N.npz $O/$N tailin $RES $SPP 2>&1 | G
../venv/bin/python mount_map.py $O/${N}_plan.png mounts.json $O/fastening_map.png $N.npz; ../venv/bin/python sheets_one.py $O $N
echo RENDERED $O
