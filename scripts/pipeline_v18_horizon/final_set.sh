#!/bin/bash
# [fix2] FINAL RENDER SET (A product, B lid, C construction, D details), 1600 px, 64 spp unless noted. usage: ./final_set.sh GROUP
# [fix4-int] renders the integrated f4 (P / N below) -> <round>/final_set4/renders; needs ../rv.sh; stoppad aimed at the STOP key axis (4.4, 0)
D=$(cd "$(dirname "$0")/.." && pwd)      # [fix4-int] the round folder (design/..), was a hard-coded f2 folder
O=${FS_OUT:-$D/final_set4/renders}; mkdir -p $O; P=${FS_P:-f4.json}; N=${FS_N:-f4.npz}; R="$D/rv.sh"; PY=$D/venv/bin/python; cd $D/design      # [fix4-int]
HC=TopCover,CoverKit,Lid,LidKit,LidWell,HingePod
case $1 in
A) CUSTOM_VIEWS="rear2=-1.15:-1.0:0.62:-0.06:0.0:0.13:85;rearstr=-1.6:0:0.42:-0.1:0:0.14:100;sideup=0.25:-1.3:0.75:0.1:-0.1:0.2:100;plan=0.0005:0.0:2.3:0.0:0.0:0.1:100" \
     $R $P $N $O/p hero,face34,front,side,rear,rear2,rearstr,sideup,top,plan,dockhero 1600 64;;
B) CUSTOM_VIEWS="closedtop=0.30:-0.36:0.56:-0.040:0.0:0.276:105;stoppad=0.24:-0.16:0.46:0.0044:0.0:0.283:220" $R $P $N $O/l closedtop,stoppad 1600 64
   LIDDEG=30 CUSTOM_VIEWS="lid30=-0.75:-0.95:1.05:-0.03:0.0:0.2:85;lidlatch30=0.30:-0.24:0.40:0.040:0.0:0.270:150" $R $P $N $O/l lid30,lidlatch30 1600 64
   CUSTOM_VIEWS="lid75F=0.75:-0.6:0.65:-0.05:0:0.28:100" $R $P $N $O/l lidopen,lid75F 1600 64
   HIDE=TopCover,CoverKit HEADLIGHT=8 CUSTOM_VIEWS="lidhingeX=-0.36:-0.30:0.42:-0.140:-0.050:0.282:150" $R $P $N $O/l lidhingeX 1600 64
   cd $D/design; $PY section.py $N $O/../tmp_$1_secH.npz y -0.054 + > /dev/null
   CLIPY=-0.054 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="lidhingesec=-0.155:-0.34:0.29:-0.150:-0.054:0.272:170" $R $P $O/../tmp_$1_secH.npz $O/l lidhingesec 1600 64
   $PY section.py $N $O/../tmp_$1_secL.npz y 0.0 + > /dev/null
   CLIPY=0.0 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="latchsec=0.045:-0.30:0.285:0.035:0.0:0.268:190" $R $P $O/../tmp_$1_secL.npz $O/l latchsec 1600 64;;
C) LIFT=0.2 LIFT2=0.42 HEADLIGHT=25 CUSTOM_VIEWS="explode=-1.5:-1.55:0.95:0.0:0.0:0.4:60;explodec=-0.58:-0.66:0.78:-0.22:-0.16:0.37:105" \
     $R $P $N $O/c explode,explodec 1600 64
   LIFT=0.2 LIFT2=0.42 HEADLIGHT=60 HIDE=Backdrop CUSTOM_VIEWS="explodelow=0.55:-1.05:-0.30:0.0:0.0:0.40:52" $R $P $N $O/c explodelow 1600 64
   HIDE=$HC CUSTOM_VIEWS="deck=-0.55:-0.75:0.95:-0.05:-0.05:0.22:70;deckN=0.75:-0.3:0.5:0.27:-0.02:0.2:120;deckT=-0.75:-0.35:0.55:-0.28:-0.02:0.215:110" $R $P $N $O/c deck,deckN,deckT 1600 64
   HIDE=Backdrop,Chassis,Wheel_L,Wheel_R,Caster_L,Caster_R,RingDecal_L,RingDecal_R,Skirt,Plinth,NewShell,Mount,Camera,PortSleeves,Controls,ControlsSI,CtlTower,StopLegend HEADLIGHT=260 CUSTOM_VIEWS="coverin=-0.15:-0.3:-0.75:-0.02:0.0:0.25:55" $R $P $N $O/c coverin 1600 64
   cd $D/design; $PY section.py $N $O/../tmp_$1_secY.npz y -0.06 + > /dev/null
   CLIPY=-0.06 HEADLIGHT=30 CUSTOM_VIEWS="secnose=0.32:-0.34:0.21:0.305:-0.06:0.190:220;sectail=-0.33:-0.34:0.21:-0.317:-0.06:0.190:220;secY=0.0:-1.6:0.25:0.0:-0.06:0.19:70" $R $P $O/../tmp_$1_secY.npz $O/c secnose,sectail,secY 1600 64
   $PY section.py $N $O/../tmp_$1_secX.npz x 0.12 - > /dev/null
   CLIP=x:0.12:- HEADLIGHT=30 CUSTOM_VIEWS="secorbit=0.33:-0.21:0.27:0.12:-0.170:0.2425:220;secX=1.3:0.0:0.25:0.12:0.0:0.18:70" $R $P $O/../tmp_$1_secX.npz $O/c secorbit,secX 1600 64
   $PY section.py $N $O/../tmp_$1_secR.npz x -0.20 - > /dev/null
   CLIP=x:-0.20:- HEADLIGHT=30 CUSTOM_VIEWS="secrise=-0.03:-0.30:0.24:-0.20:-0.19:0.155:150" $R $P $O/../tmp_$1_secR.npz $O/c secrise 1600 64;;
D) CUSTOM_VIEWS="fband=0.95:0.0:0.19:0.30:0.0:0.165:75;pupil=0.75:-0.08:0.19:0.317:0:0.174:200;rband=-0.95:0.0:0.19:-0.32:0.0:0.165:75;orbitc=0.35:-0.65:0.25:0.24:-0.17:0.2:150;orbfil=0.42:-0.62:0.25:0.235:-0.17:0.19:180;risec=-0.12:-0.75:0.22:-0.21:-0.18:0.16:120;shut=0.20:-0.22:0.38:0.045:-0.072:0.277:220;tailcorner=-0.75:-0.62:0.26:-0.29:-0.17:0.18:170" \
     $R $P $N $O/d fband,pupil,rband,orbitc,orbfil,risec,shut,tailcorner 1600 64
   # [fix4-front2] A3: the dark close-ups get the f2 headlight pass back (final_set_D_headlight.txt; the sheet labels them 'headlight' = HL12)
   HEADLIGHT=12 CUSTOM_VIEWS="portc=0.55:0.10:0.20:0.318:0.006:0.147:230;port=0.62:0.16:0.24:0.316:0.020:0.146:170;floor=0.62:-0.52:0.17:0.29:-0.17:0.118:110" $R $P $N $O/d portc,port,floor 1600 64;;
esac
rm -f $O/../tmp_$1_sec*.npz 2>/dev/null
echo FINALSET $1 DONE
