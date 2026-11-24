#!/bin/bash
# [fix2] after-renders of f2 at the reviewers' exact views (same resolution / spp as their f1 images) -> r/after/
D=<scratch>/wf4/fix2; S=<scratch>/wf4/fix2/..; source $S/r1_aesthetic/views.sh; O=$D/r/after; mkdir -p $O; R=$D/rv.sh; P=f2.json; N=f2.npz
CV="rear2=-1.15:-1.0:0.62:-0.06:0.0:0.13:85"
LG="o_side=0.0:-6.0:0.165:0.0:0.0:0.165:240;o_front=6.0:0.0:0.165:0.0:0.0:0.165:400;o_rear=-6.0:0.0:0.165:0.0:0.0:0.165:400;o_top=0.25:0.0:6.0:0.0:0.0:0.15:240;plinthc=0.62:-0.52:0.17:0.29:-0.17:0.118:110;risec=-0.30:-0.70:0.20:-0.256:-0.2:0.158:150;rearlow=-0.95:-0.45:0.10:-0.30:-0.05:0.085:85;statusc=0.30:-0.18:0.45:0.075:0.0:0.276:200;orbitc=0.35:-0.65:0.25:0.24:-0.17:0.2:150"
CUSTOM_VIEWS="$CV;$OV;$MV" $R $P $N $O/f2 hero,face34,rear2,top,eS,qRR,qFR,eR,mShoulder,mShoulderR,mPort,mLobe,mStatus,mTailCorner,mRise 1600 64
CUSTOM_VIEWS="$LG" $R $P $N $O/rv o_side,o_front,o_rear,o_top,plinthc,risec,rearlow,statusc,orbitc 1600 64
ZV="zRearQ=-1.1:-1.0:0.7:-0.12:0:0.18:70;zSide=0:-1.6:0.4:0:0:0.17:70"
ZEBRA=14 ZEBRA_AXIS=Z CUSTOM_VIEWS="$ZV" NODENOISE=1 $R $P $N $O/z zRearQ,zSide 1600 32
ZEBRA=20 ZEBRA_AXIS=X CUSTOM_VIEWS="zSideX=0:-1.6:0.4:0:0:0.17:70" NODENOISE=1 $R $P $N $O/zx zSideX 1600 32
LIDDEG=75 CUSTOM_VIEWS="$(cat $S/r2_defects/views_lid.txt)" $R $P $N $O/a75 lidD,lidF 1200 64
CUSTOM_VIEWS="$(cat $S/r2_defects/views_H.txt)" $R $P $N $O/g gH0900 1200 64
CUSTOM_VIEWS="$LV" $R $P $N $O/v lidopenF 1600 64
LIFT=0.2 LIFT2=0.42 HEADLIGHT=25 CUSTOM_VIEWS="explodeup=0.75:-1.1:0.22:0.0:0.0:0.48:55;explodec=-0.58:-0.66:0.78:-0.22:-0.16:0.37:105" $R $P $N $O/e explodeup,explodec 1200 64
echo AFTER DONE
