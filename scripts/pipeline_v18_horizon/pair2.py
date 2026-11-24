# usage: pair.py BEFORE AFTER OUT x0 y0 x1 y1 [--w W] [--gain G] [--label TEXT]  -> side-by-side before | after crop (same box), labelled
import sys
from PIL import Image, ImageDraw, ImageEnhance
a = sys.argv[1:]; W = 900; G = None; LB = ''
if '--w' in a: i = a.index('--w'); W = int(a[i + 1]); del a[i:i + 2]
if '--gain' in a: i = a.index('--gain'); G = float(a[i + 1]); del a[i:i + 2]
if '--label' in a: i = a.index('--label'); LB = a[i + 1]; del a[i:i + 2]
box = tuple(int(v) for v in a[3:7]) if len(a) >= 7 else None
ims = []
for p, t in ((a[0], 'BEFORE f1 (fix 1)'), (a[1], 'AFTER f2 (fix 2)')):
    im = Image.open(p).convert('RGB')
    if box: im = im.crop(box)
    if G: im = ImageEnhance.Brightness(im).enhance(G)
    im = im.resize((W, int(im.height * W / im.width)), Image.LANCZOS); d = ImageDraw.Draw(im); d.rectangle((0, 0, 560, 26), fill=(0, 0, 0)); d.text((6, 6), t + ('  ' + LB if LB else ''), fill=(255, 255, 255)); ims.append(im)
out = Image.new('RGB', (2 * W + 8, max(i.height for i in ims)), (255, 255, 255)); out.paste(ims[0], (0, 0)); out.paste(ims[1], (W + 8, 0)); out.save(a[2]); print(a[2], out.size)
