#!/bin/bash
# usage: ./run_concept.sh NAME [views] [res] [samples]   -> reads NAME.json, writes NAME.npz, NAME_<view>.png, NAME_sheet.png
set -e; cd "$(dirname "$0")"; N=$1; VIEWS=${2:-hero,face,face34,rear,side,lidopen}; RES=${3:-900}; SPP=${4:-32}
../venv/bin/python design_gen.py $N.json $N.npz 2>$N.log | tee $N.check.json
~/blender-4.2.3-linux-x64/blender -b --factory-startup -P design_render.py -- $N.json $N.npz $PWD/$N $VIEWS $RES $SPP 2>&1 | grep -E "Error|Traceback|line [0-9]+" || true
../venv/bin/python - <<PY
from PIL import Image
v='$VIEWS'.split(','); ims=[Image.open(f'$N_{k}.png'.replace('\$N','$N')).convert('RGB') for k in v] if False else [Image.open('$N'+'_'+k+'.png').convert('RGB') for k in v]
w,h=ims[0].size; c=3 if len(ims)>4 else 2; r=(len(ims)+c-1)//c
o=Image.new('RGB',(c*w,r*h),(255,255,255))
for i,im in enumerate(ims): o.paste(im,((i%c)*w,(i//c)*h))
o.save('$N'+'_sheet.png'); print('sheet', '$N'+'_sheet.png')
PY
