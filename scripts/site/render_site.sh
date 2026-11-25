#!/bin/bash
# Product-site renders of Orbit (default f9 files: PARAMS / NPZ / WEB_PARAMS / WEB_NPZ override; P = design folder for rv.sh) (transparent film + shadow catcher; see CHANGES.txt section 8 for WEBBG / LIDMAP / MATVIEWS / EXMAP).
# f9: the visor end (-x, casters) is the front, the charging port end (+x, drive wheels) the back; every 3/4 view looks at the visor. Output: $O (default scripts/site/r), then run bake_site.py.
HERE=$(cd "$(dirname "$0")" && pwd); O=${O:-$HERE/r}; mkdir -p $O
export WEBBG=1
# 1 Orbit stills + colours + close-ups
CUSTOM_VIEWS="hero=-1.35:-1.25:0.95:0.0:0.0:0.11:70;stopkey=-0.2512:-0.18:0.47:0.0044:0.0:0.284:200;pupil=-1.02:-0.52:0.32:-0.28:-0.05:0.165:50;c_orbit=-1.35:-1.25:0.95:0.0:0.0:0.11:70;c_lunar=-1.35:-1.25:0.95:0.0:0.0:0.11:70;c_graphite=-1.35:-1.25:0.95:0.0:0.0:0.11:70;c_glacier=-1.35:-1.25:0.95:0.0:0.0:0.11:70;c_sage=-1.35:-1.25:0.95:0.0:0.0:0.11:70;c_sand=-1.35:-1.25:0.95:0.0:0.0:0.11:70" MATVIEWS="c_orbit|Body=#F7F7F4:gloss,Lid=#F7F7F4:gloss,Core=#6E6D69:satin;c_lunar|Body=#CDD0CF:satincoat,Lid=#CDD0CF:satincoat,Core=#3A3B3E:texture;c_graphite|Body=#3B3E44:metallic,Lid=#3B3E44:metallic,Core=#17181B:texture;c_glacier|Body=#A9BCCB:satincoat,Lid=#A9BCCB:satincoat,Core=#2E3A45:texture;c_sage|Body=#9AA58F:satincoat,Lid=#9AA58F:satincoat,Core=#3A4236:texture;c_sand|Body=#C9BB9E:satincoat,Lid=#C9BB9E:satincoat,Core=#5A5144:texture" $HERE/rv.sh ${PARAMS:-f9.json} ${NPZ:-f9.npz} $O/o hero,stopkey,pupil,c_orbit,c_lunar,c_graphite,c_glacier,c_sage,c_sand 2400 64
recolour_back=1
# 2 turntable
V=""; N=""; for i in $(seq 0 71); do n=$(printf 't%02d' $i); V="$V$n=$(python3 -c "import math;a=math.radians(-137.2+5*$i);print(f'{1.9*math.cos(a):.4f}:{1.9*math.sin(a):.4f}')"):0.80:0.0:0.0:0.12:70;"; N="$N$n,"; done      # starts on the visor 3/4 view
CUSTOM_VIEWS="$V" $HERE/rv.sh ${PARAMS:-f9.json} ${NPZ:-f9.npz} $O/tt "${N%,}" 1600 32
# 3 lid sequence
CUSTOM_VIEWS="lid00=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid01=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid02=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid03=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid04=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid05=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid06=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid07=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid08=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid09=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid10=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid11=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid12=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid13=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid14=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid15=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid16=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid17=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid18=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid19=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid20=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid21=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid22=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid23=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid24=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid25=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid26=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid27=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid28=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid29=-1.30:-1.05:0.98:-0.05:0.0:0.30:80;lid30=-1.30:-1.05:0.98:-0.05:0.0:0.30:80" LIDMAP="lid00=0.0,lid01=2.5,lid02=5.0,lid03=7.5,lid04=10.0,lid05=12.5,lid06=15.0,lid07=17.5,lid08=20.0,lid09=22.5,lid10=25.0,lid11=27.5,lid12=30.0,lid13=32.5,lid14=35.0,lid15=37.5,lid16=40.0,lid17=42.5,lid18=45.0,lid19=47.5,lid20=50.0,lid21=52.5,lid22=55.0,lid23=57.5,lid24=60.0,lid25=62.5,lid26=65.0,lid27=67.5,lid28=70.0,lid29=72.5,lid30=75.0" $HERE/rv.sh ${PARAMS:-f9.json} ${NPZ:-f9.npz} $O/lid lid00,lid01,lid02,lid03,lid04,lid05,lid06,lid07,lid08,lid09,lid10,lid11,lid12,lid13,lid14,lid15,lid16,lid17,lid18,lid19,lid20,lid21,lid22,lid23,lid24,lid25,lid26,lid27,lid28,lid29,lid30 1600 32
# 4 dock-port close-up: opaque (no WEBBG), lit by a light travelling with the camera
WEBBG= HEADLIGHT=12 CUSTOM_VIEWS="dockport=0.352:-0.30:0.205:0.318:0.0:0.152:120" $HERE/rv.sh ${PARAMS:-f9.json} ${NPZ:-f9.npz} $O/pt dockport 1600 64
# 5 exploded sequence (3D, 24 frames): body / cover / lid lifted 110 / 220 / 330 mm at full explode, smoothstep eased
V=""; E=""; L=""
for i in $(seq 0 23); do n=$(printf 'ex%02d' $i); q=$(python3 -c "t=$i/23;print(t*t*(3-2*t))")
  V="$V$n=-1.9:-1.95:1.2:0.0:0.0:0.30:70;"; E="$E$n=$(python3 -c "q=$q;print(f'{0.11*q:.5f}:{0.22*q:.5f}:{0.33*q:.5f}')"),"; L="$L$n,"; done
CUSTOM_VIEWS="$V" EXMAP="$E" $HERE/rv.sh ${WEB_PARAMS:-f9_web.json} ${WEB_NPZ:-f9_web.npz} $O/exs "${L%,}" 1600 48
# 6 docking sequence (36 frames): the mower reverses onto the dock plate (drive wheels up the 17 mm ramp first), pins into the port at the back
python3 $HERE/poses.py dock 36 > $O/dock.txt
CUSTOM_VIEWS="$(sed -n 1p $O/dock.txt)" POSEMAP="$(sed -n 2p $O/dock.txt)" $HERE/rv.sh ${WEB_PARAMS:-f9_web.json} ${WEB_NPZ:-f9_web.npz} $O/dk "$(sed -n 3p $O/dock.txt)" 1600 40
# 7 slope sequence (36 frames): 35 % (19.3 deg) lawn block, the mower climbs 1.2 m visor first; RAMP_TEX = top-down short-grass photo
python3 $HERE/poses.py slope 36 > $O/slope.txt
RAMP=19.29 RAMP_L=2.4 RAMP_T=0.22 RAMP_SOIL='#9C7A56' RAMP_TEX=${RAMP_TEX:?top-down grass photo} CUSTOM_VIEWS="$(sed -n 1p $O/slope.txt)" POSEMAP="$(sed -n 2p $O/slope.txt)" $HERE/rv.sh ${WEB_PARAMS:-f9_web.json} ${WEB_NPZ:-f9_web.npz} $O/sl "$(sed -n 3p $O/slope.txt)" 1600 40
# 8 plan views for the lawn animation: orthographic, 1.2 m wide, centred 0.15 m from the mower origin towards +x (the back / charging port points left in the image)
CUSTOM_VIEWS="mplan=0.1505:0.0:3.0:0.15:0.0:0.0:50" ORTHO="mplan=1.2" $HERE/rv.sh ${WEB_PARAMS:-f9_web.json} ${WEB_NPZ:-f9_web.npz} $O/p mplan 1200 48
ONLY=Dock CUSTOM_VIEWS="dockplan=0.1505:0.0:3.0:0.15:0.0:0.0:50" ORTHO="dockplan=1.2" $HERE/rv.sh ${WEB_PARAMS:-f9_web.json} ${WEB_NPZ:-f9_web.npz} $O/p dockplan 1200 48
echo SITE RENDERS DONE
