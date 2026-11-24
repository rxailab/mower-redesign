# usage: cmp.py OUT.png W label1=img1 label2=img2 ... [--crop x0,y0,x1,y1 (fractions)]
import sys
from PIL import Image, ImageDraw, ImageFont
out = sys.argv[1]; W = int(sys.argv[2]); items = []; crop = None
args = sys.argv[3:]
if '--crop' in args: i = args.index('--crop'); crop = [float(t) for t in args[i + 1].split(',')]; args = args[:i] + args[i + 2:]
for a in args:
    l, p = a.split('=', 1); im = Image.open(p).convert('RGB')
    if crop: w, h = im.size; im = im.crop((int(crop[0] * w), int(crop[1] * h), int(crop[2] * w), int(crop[3] * h)))
    im = im.resize((W, int(im.size[1] * W / im.size[0])), Image.LANCZOS); items.append((l, im))
H = max(im.size[1] for _, im in items); S = Image.new('RGB', (W * len(items), H + 34), (250, 250, 250)); dr = ImageDraw.Draw(S)
try: fnt = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 22)
except Exception: fnt = ImageFont.load_default()
for k, (l, im) in enumerate(items): S.paste(im, (k * W, 34)); dr.text((k * W + 8, 6), l, fill=(20, 20, 20), font=fnt)
S.save(out); print(out, S.size)
