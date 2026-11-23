# Annotated fastening map on top of the "plan" render.  usage: python mount_map.py PLAN.png mounts.json OUT.png SHELL.npz [cover]   (cover: PLAN.png was rendered with the top cover taken off)
import json, sys
from PIL import Image, ImageDraw, ImageFont
im = Image.open(sys.argv[1]).convert('RGB'); MJ = json.load(open(sys.argv[2])); W, H = im.size; S = W / 1600
F = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
import os, glob
if not os.path.exists(F): F = (glob.glob('/usr/share/fonts/**/NotoSansCJK*.ttc', recursive=True) + glob.glob('/usr/share/fonts/**/*CJK*', recursive=True))[0]
f1 = ImageFont.truetype(F, int(21 * S), index=2); f2 = ImageFont.truetype(F, int(27 * S), index=2); f3 = ImageFont.truetype(F, int(38 * S), index=2)
CZ, LENS = 2.3, 100.0
import numpy as np
VV = np.load(sys.argv[4])['v']; COVER = len(sys.argv) > 5
def skin(x, y, z0, r=0.006):          # height of the outer surface above a point (markers sit on the surface, so they line up in perspective)
    m = (np.hypot(VV[:, 0] - x, VV[:, 1] - y) < r) & (VV[:, 2] > z0); return float(np.median(VV[m, 2])) if m.any() else z0 + 0.03
def px(x, y, z): k = LENS / 36.0 * W / (CZ - z); return W / 2 + x * k, H / 2 - y * k
d = ImageDraw.Draw(im, 'RGBA'); OR = (255, 122, 40); BL = (60, 170, 255); GR = (90, 220, 120)
def mark(p, n, col, r=15, side=1):
    r *= S; d.ellipse([p[0] - r, p[1] - r, p[0] + r, p[1] + r], outline=col + (255,), width=int(3 * S))
    t = str(n); tw = d.textlength(t, font=f1); q = (p[0] + side * (r + 16 * S), p[1])
    d.ellipse([q[0] - 14 * S, q[1] - 14 * S, q[0] + 14 * S, q[1] + 14 * S], fill=col + (255,)); d.text((q[0] - tw / 2, q[1] - 15 * S), t, font=f1, fill=(15, 15, 15))
n = 0
for x, y, z, r in MJ['posts']: n += 1; mark(px(x, y, z + 0.004 if COVER else skin(x, y, z)), n, OR, side=(-1 if x < -0.2 and abs(y) < 0.05 else (1 if y > 0.02 else -1) if abs(y) > 0.05 else 1))
for u in MJ['under']: n += 1; mark(px(u['x'], u['y'], 0.2 if COVER else skin(u['x'], u['y'], u['z'] + 0.03, 0.012)), n, BL, 12, side=-1 if u['kind'] == 'front' else (1 if u['y'] > 0 else -1))
for c in MJ['clips']: n += 1; mark(px(c['x_below'] - 0.012, (c['y0'] + c['y1']) / 2, 0.10), n, GR, 10, side=-1)
d.rectangle([0, H - 150 * S, W, H], fill=(18, 19, 22, 225)); d.text((34 * S, 26 * S), '固定点位置（俯视，车头朝右' + ('，上盖已取下）' if COVER else '）'), font=f3, fill=(245, 245, 245))
y0 = H - 135 * S
for col, txt in ((OR, '1–14  从上方拧入的螺丝：' + ('在上盖下面的台面上，装上上盖后全部看不见' if COVER else '壳体上的沉孔，对准底盘槽体边缘的 14 根螺丝柱')), (BL, '15–19  从底部拧入的螺丝：壳体内侧的螺丝柱，落在底盘前翼（3 处）和后角承座（2 处）上'), (GR, '20–24  卡扣：尾部裙边内侧的 5 个卡钩，扣进底盘后围的 5 个卡口')):
    d.ellipse([34 * S, y0 + 9 * S, 56 * S, y0 + 31 * S], fill=col + (255,)); d.text((70 * S, y0), txt, font=f2, fill=(238, 238, 238)); y0 += 42 * S
im.save(sys.argv[3]); print('saved', sys.argv[3])
