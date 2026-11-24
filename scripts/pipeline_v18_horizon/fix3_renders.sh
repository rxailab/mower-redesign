#!/bin/bash
# [fix3-int] fix-round-3 detail renders of the integrated f3 (the views each branch used, on the merged model) -> ../final_set/renders/<prefix>_*.png
# usage: ./fix3_renders.sh GROUP   S STOP key + halo (s_), H hinge (h_), P charger ports (pt_), T chassis struts (st_), L labyrinth (lb_), G lights (lt_)
# needs ../rv.sh (renders design_render.py, waits while 2 Blender processes run); 1600 px, 64 spp.
D=$(cd "$(dirname "$0")" && pwd); M=$(cd "$D/.." && pwd); O=$M/final_set/renders; R="$M/rv.sh"; PY=$M/venv/bin/python; mkdir -p $O; cd $D
P=f3.json; N=f3.npz; T=$M/final_set/tmp_$1; HC=TopCover,CoverKit,Lid,LidKit,LidWell,HingePod
sec() { $PY section.py $N ${T}_$1.npz $2 $3 $4 $5 > /dev/null; }      # name axis value sign [--cap-deck]
case $1 in
S) CUSTOM_VIEWS="stopside=0.16:-0.115:0.300:0.0044:0.0:0.2845:150" $R $P $N $O/s stopside 1600 64
   CUSTOM_VIEWS="lidkey=0.42:-0.30:0.52:0.000:0.0:0.262:120" $R $P $N $O/s lidkey 1600 64
   sec Y0 y 0.0 +
   CLIPY=0.0 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="stopsec=-0.02:-0.32:0.29:-0.025:0.0:0.274:95;stopsec2=0.004:-0.30:0.29:0.004:0.0:0.275:200;sechalo=0.022:-0.30:0.272:0.022:0.0:0.268:170;sechaloZ=0.024:-0.20:0.262:0.024:0.0:0.2565:420" $R $P ${T}_Y0.npz $O/s stopsec,stopsec2,sechalo,sechaloZ 1600 64
   STOPSTROKE=0.005 CLIPY=0.0 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="stopsec2P=0.004:-0.30:0.29:0.004:0.0:0.275:200;sechaloP=0.022:-0.30:0.272:0.022:0.0:0.268:170" $R $P ${T}_Y0.npz $O/s stopsec2P,sechaloP 1600 64
   LIDDEG=-0.438 CLIPY=0.0 HEADLIGHT=30 CUSTOM_VIEWS="lidsechaloOT=0.024:-0.20:0.262:0.024:0.0:0.2565:420" $R $P ${T}_Y0.npz $O/s lidsechaloOT 1600 64;;
H) CUSTOM_VIEWS="lid75F=0.75:-0.6:0.65:-0.05:0:0.28:100" $R $P $N $O/h lid75F 1600 64
   HIDE=TopCover,CoverKit HEADLIGHT=8 CUSTOM_VIEWS="lidhingeX=-0.36:-0.30:0.42:-0.140:-0.050:0.282:150" $R $P $N $O/h lidhingeX 1600 64
   sec H y -0.054 +
   CLIPY=-0.054 HEADLIGHT=30 LIDDEG=0 CUSTOM_VIEWS="lidhingesec=-0.155:-0.34:0.29:-0.150:-0.054:0.272:170;lidhingesecW=-0.128:-0.40:0.268:-0.128:-0.054:0.262:95" $R $P ${T}_H.npz $O/h lidhingesec,lidhingesecW 1600 64
   CLIPY=-0.054 HEADLIGHT=30 LIDDEG=75 CUSTOM_VIEWS="lidsecW75=-0.140:-0.40:0.285:-0.140:-0.054:0.272:85" $R $P ${T}_H.npz $O/h lidsecW75 1600 64
   sec S y -0.0482 -
   HIDE=HingePod CLIP=y:-0.0482:- HEADLIGHT=30 LIDDEG=75 CUSTOM_VIEWS="lidstop=-0.150:0.24:0.285:-0.149:-0.049:0.266:150;lidstopZ=-0.146:0.24:0.2735:-0.146:-0.049:0.2655:300" $R $P ${T}_S.npz $O/h lidstop,lidstopZ 1600 64
   HEADLIGHT=10 CUSTOM_VIEWS="lidkeel=-0.04:-0.13:0.34:-0.140:-0.054:0.300:60" $R $P $N $O/h lidkeel 1600 64;;
P) HEADLIGHT=12 CUSTOM_VIEWS="portc3=0.55:0.10:0.205:0.318:0.004:0.152:230;portm=0.62:-0.16:0.24:0.316:-0.020:0.146:170" $R $P $N $O/pt portc3,portm 1600 64
   CUSTOM_VIEWS="dockport=0.352:-0.30:0.205:0.318:0.0:0.152:120;dockportL=0.352:0.30:0.205:0.318:0.0:0.152:120" $R $P $N $O/pt dockport,dockportL 1600 64
   sec P y 0.0285 +
   CLIPY=0.0285 HEADLIGHT=30 CUSTOM_VIEWS="docksec=0.30:-0.30:0.158:0.30:0.0285:0.152:110;slotlamp=0.312:-0.20:0.170:0.312:0.0285:0.1655:330" $R $P ${T}_P.npz $O/pt docksec,slotlamp 1600 64
   sec X x 0.3105 -
   CLIP=x:0.3105:- HEADLIGHT=20 CUSTOM_VIEWS="secweb=0.55:0.0:0.152:0.31:0.0:0.152:150" $R $P ${T}_X.npz $O/pt secweb 1600 64;;
T) VS="wheel34=0.62:-0.66:0.24:0.12:-0.17:0.15:80;seatside=0.12:-0.62:0.20:0.12:-0.17:0.165:105;chwheel=0.36:-0.52:0.40:0.15:-0.16:0.18:62;pinin=0.06:-0.07:0.33:0.118:-0.163:0.219:110;hookc=0.40:-0.36:0.22:0.268:-0.165:0.157:150;pinc=0.20:-0.36:0.33:0.119:-0.163:0.218:150"
   HB=NewShell,$HC,Mount,PortSleeves,Skirt,Plinth,Controls,ControlsSI,CtlTower,StopLegend
   CUSTOM_VIEWS="$VS" $R $P $N $O/st wheel34 1600 64
   HIDE=Wheel_R,RingDecal_R CUSTOM_VIEWS="$VS" $R $P $N $O/st seatside 1600 64
   HIDE=$HC,Mount HEADLIGHT=15 STRUTS_HL=1 CUSTOM_VIEWS="$VS" $R $P $N $O/st pinin 1600 64
   HIDE=$HB,Wheel_R,RingDecal_R HEADLIGHT=12 STRUTS_HL=1 CUSTOM_VIEWS="$VS" $R $P $N $O/st chwheel,hookc,pinc 1600 64;;
L) sec LY y -0.06 + --cap-deck
   CLIPY=-0.06 HEADLIGHT=30 CUSTOM_VIEWS="secnoseZ=0.3100:-0.30:0.1905:0.3100:-0.06:0.1905:480;sectailZ=-0.3163:-0.30:0.1905:-0.3163:-0.06:0.1905:480" $R $P ${T}_LY.npz $O/lb secnoseZ,sectailZ 1600 64
   sec LX12 x 0.12 - --cap-deck
   CLIP=x:0.12:- HEADLIGHT=30 CUSTOM_VIEWS="secorbitZ=0.36:-0.1688:0.2430:0.12:-0.1688:0.2430:480" $R $P ${T}_LX12.npz $O/lb secorbitZ 1600 64
   sec LXm20 x -0.20 - --cap-deck
   CLIP=x:-0.20:- HEADLIGHT=30 CUSTOM_VIEWS="secriseZ=0.04:-0.1932:0.1660:-0.20:-0.1932:0.1660:480" $R $P ${T}_LXm20.npz $O/lb secriseZ 1600 64
   sec LXm10 x -0.10 - --cap-deck
   CLIP=x:-0.10:- HEADLIGHT=30 CUSTOM_VIEWS="secbaseZ=0.14:-0.1731:0.1240:-0.10:-0.1731:0.1240:480" $R $P ${T}_LXm10.npz $O/lb secbaseZ 1600 64
   sec LX24 x 0.24 - --cap-deck
   CLIP=x:0.24:- HEADLIGHT=30 CUSTOM_VIEWS="secbrowZ=0.48:-0.1688:0.1912:0.24:-0.1688:0.1912:480" $R $P ${T}_LX24.npz $O/lb secbrowZ 1600 64
   CUSTOM_VIEWS="gapZ=0.20:-0.40:0.30:0.12:-0.1705:0.2434:400;gapnoseZ=0.55:-0.12:0.25:0.3114:-0.06:0.1910:400" $R $P $N $O/lb gapZ,gapnoseZ 1600 64;;
G) CUSTOM_VIEWS="lensend=0.62:-0.42:0.22:0.300:-0.120:0.174:330;pupilZ=0.75:-0.08:0.19:0.317:0.0:0.174:420" $R $P $N $O/lt lensend,pupilZ 1600 64
   sec G7 y 0.07 +
   CLIPY=0.07 HEADLIGHT=30 CUSTOM_VIEWS="secbarF=0.305:-0.20:0.185:0.305:0.07:0.174:200" $R $P ${T}_G7.npz $O/lt secbarF 1600 64
   sec G0 y 0.0004 +
   CLIPY=0.0004 HEADLIGHT=30 CUSTOM_VIEWS="secpupil=0.305:-0.20:0.185:0.305:0.0:0.172:180" $R $P ${T}_G0.npz $O/lt secpupil 1600 64
   sec G3 y 0.03 +
   CLIPY=0.03 HEADLIGHT=30 CUSTOM_VIEWS="secslot=0.311:-0.20:0.170:0.311:0.03:0.1645:300" $R $P ${T}_G3.npz $O/lt secslot 1600 64
   sec GR y -0.07 +
   CLIPY=-0.07 HEADLIGHT=30 CUSTOM_VIEWS="secbarR=-0.310:-0.27:0.185:-0.310:-0.07:0.174:200" $R $P ${T}_GR.npz $O/lt secbarR 1600 64
   HIDE=$HC,Mount,Chassis,StopKey HEADLIGHT=25 CUSTOM_VIEWS="kitF=0.16:-0.02:0.33:0.295:-0.06:0.172:70;harnF=0.17:0.30:0.33:0.265:0.09:0.200:75" $R $P $N $O/lt kitF,harnF 1600 64;;
esac
rm -f ${T}_*.npz
echo FIX3RENDERS $1 DONE
