#!/bin/bash
# [fix4-cover2] detail renders of the cover-kit / deck / labyrinth / seat fixes -> ../r/c2_*.png   usage: ./fix4_cover2_renders.sh P.json NPZ GROUP
#   (GY / TT: single re-renders of legsecY / the TPE views)
#   O overall (hero, face34, seatside)   G G1 legs (sections at the moved site x 157 and the old site x 119, chassis cut faces orange)
#   D deck edge + barb (G4) and the x-locator   T m12 TPE seat pad over the drive housing   L M5 labyrinth at the nose / tail ends
D=$(cd "$(dirname "$0")" && pwd); M=$(cd "$D/.." && pwd); O=$M/r; R="$M/rv.sh"; PY=$M/venv/bin/python; mkdir -p $O; cd $D
P=$1; N=$2; T=$M/r/tmp_$3; HC=TopCover,CoverKit,Lid,LidKit,LidWell,HingePod
sec() { $PY section.py $N ${T}_$1.npz $2 $3 $4 $5 > /dev/null; }      # name axis value sign [--cap-deck]
case $3 in
GY) sec GY y -0.1632 + --cap-deck
   CLIPY=-0.1632 HEADLIGHT=30 STRUTS_HL=1 CUSTOM_VIEWS="legsecY=0.138:-0.45:0.212:0.138:-0.1632:0.212:140" $R $P ${T}_GY.npz $O/c2 legsecY 1600 64;;
O) CUSTOM_VIEWS="seatside=0.12:-0.62:0.20:0.12:-0.17:0.165:105" $R $P $N $O/c2 hero,face34 1600 64
   HIDE=Wheel_R,RingDecal_R CUSTOM_VIEWS="seatside=0.12:-0.62:0.20:0.12:-0.17:0.165:105" $R $P $N $O/c2 seatside 1600 64;;
G) sec G157 x 0.1572 - --cap-deck
   CLIP=x:0.1572:- HEADLIGHT=30 STRUTS_HL=1 CUSTOM_VIEWS="legsecR=0.42:-0.163:0.216:0.1572:-0.163:0.216:260" $R $P ${T}_G157.npz $O/c2 legsecR 1600 64
   sec GY y -0.1632 + --cap-deck
   CLIPY=-0.1632 HEADLIGHT=30 STRUTS_HL=1 CUSTOM_VIEWS="legsecY=0.138:-0.45:0.212:0.138:-0.1632:0.212:140" $R $P ${T}_GY.npz $O/c2 legsecY 1600 64
   sec G119 x 0.1193 - --cap-deck
   CLIP=x:0.1193:- HEADLIGHT=30 STRUTS_HL=1 CUSTOM_VIEWS="oldsiteR=0.38:-0.163:0.218:0.1193:-0.163:0.218:260" $R $P ${T}_G119.npz $O/c2 oldsiteR 1600 64
   HIDE=TopCover,Lid,LidKit,LidWell,HingePod HEADLIGHT=15 STRUTS_HL=1 CUSTOM_VIEWS="legin=0.10:-0.07:0.33:0.150:-0.163:0.214:95" $R $P $N $O/c2 legin 1600 64;;
D) sec D98 x -0.0979 - --cap-deck
   CLIP=x:-0.0979:- HEADLIGHT=30 CUSTOM_VIEWS="barbsec=0.16:-0.162:0.222:-0.0979:-0.162:0.222:330" $R $P ${T}_D98.npz $O/c2 barbsec 1600 64
   sec D35 x 0.0348 - --cap-deck
   CLIP=x:0.0348:- HEADLIGHT=30 CUSTOM_VIEWS="deckrim=0.30:0.170:0.222:0.0348:0.170:0.222:200" $R $P ${T}_D35.npz $O/c2 deckrim 1600 64
   sec DL y -0.1607 + --cap-deck
   CLIPY=-0.1607 HEADLIGHT=30 CUSTOM_VIEWS="xloc=0.002:-0.42:0.2245:0.002:-0.1607:0.2245:300" $R $P ${T}_DL.npz $O/c2 xloc 1600 64;;
T) sec T118 x 0.118 - --cap-deck
   MATCOL="Dark=#D9702A:matte" CLIP=x:0.118:- HEADLIGHT=45 STRUTS_HL=1 CUSTOM_VIEWS="tpesec=0.40:-0.1665:0.1560:0.118:-0.1665:0.1560:520" $R $P ${T}_T118.npz $O/c2 tpesec 1600 64
   MATCOL="Dark=#D9702A:matte" HIDE=Wheel_R,RingDecal_R HEADLIGHT=20 CUSTOM_VIEWS="tpeunder=0.17:-0.33:0.105:0.118:-0.166:0.156:150" $R $P $N $O/c2 tpeunder 1600 64;;
TT) MATCOL="Dark=#D9702A:matte" HIDE=Wheel_R,RingDecal_R HEADLIGHT=20 CUSTOM_VIEWS="tpeunder=0.17:-0.33:0.105:0.118:-0.166:0.156:150" $R $P $N $O/c2 tpeunder 1600 64
   sec T118 x 0.118 - --cap-deck
   MATCOL="Dark=#D9702A:matte" CLIP=x:0.118:- HEADLIGHT=45 STRUTS_HL=1 CUSTOM_VIEWS="tpesec=0.40:-0.1665:0.1560:0.118:-0.1665:0.1560:520" $R $P ${T}_T118.npz $O/c2 tpesec 1600 64;;
L) sec LY y -0.085 + --cap-deck
   CLIPY=-0.085 HEADLIGHT=30 CUSTOM_VIEWS="secnoseZ=0.3095:-0.30:0.1915:0.3095:-0.085:0.1915:480;sectailZ=-0.3160:-0.30:0.1915:-0.3160:-0.085:0.1915:480" $R $P ${T}_LY.npz $O/c2 secnoseZ,sectailZ 1600 64;;
esac
rm -f ${T}_*.npz
echo C2RENDERS $3 DONE
