# [fix3-struts] Annotated plane sections (y-z at x = const): body, cover, body kit, chassis as cut by the npz's struts_spec (or --legacy: the f2 deletion in red), wheels.
# usage: python secfig.py DESIGN.npz OUT.png x0 u0 u1 z0 z1 pxmm "title" [--legacy] [--notes "a|b|c"]
import numpy as np, sys, json
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, '.')
import strutskit as sk
def sl(V, F, ax, v0):
    T = V[F]; d = T[:, :, ax] - v0; hit = (d.min(1) < 0) & (d.max(1) > 0); T = T[hit]; d = d[hit]; out = []
    for (i, j) in ((0, 1), (1, 2), (2, 0)):
        a, b = d[:, i], d[:, j]; m = (a * b) < 0; t = np.where(m, a / np.where(m, a - b, 1), 0)
        out.append((m, T[:, i] + (T[:, j] - T[:, i]) * t[:, None]))
    return np.concatenate([np.stack([pa[ma & mb], pb[ma & mb]], 1) for (ma, pa), (mb, pb) in ((out[0], out[1]), (out[1], out[2]), (out[2], out[0]))])
a = [t for t in sys.argv[1:]]; LEG = '--legacy' in a; NOTES = []
if '--notes' in a: i = a.index('--notes'); NOTES = a[i + 1].split('|'); del a[i:i + 2]
a = [t for t in a if t != '--legacy']
NPZ, OUT = a[0], a[1]; x0, u0, u1, z0, z1, pm = [float(t) for t in a[2:8]]; TITLE = a[8] if len(a) > 8 else ''
d = np.load(NPZ); B = np.load('dev/base_parts.npz'); X = B['Chassis_v'].astype(float); CF = B['Chassis_f'].astype(np.int64)
SKS = sk.from_npz(d); V = d['v']; F = d['f']; part = d['part']; side = d['side']; lab = d['lab']
W = int((u1 - u0) * 1000 * pm); H = int((z1 - z0) * 1000 * pm) + 70
im = Image.new('RGB', (W, H), (255, 255, 255)); dr = ImageDraw.Draw(im)
try: fnt = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15); fsm = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 12)
except Exception: fnt = fsm = ImageFont.load_default()
P = lambda q: ((q[0] - u0) * 1000 * pm, (z1 - q[1]) * 1000 * pm)
for g in range(int(np.ceil(u0 * 1000)), int(u1 * 1000) + 1):
    if g % 5 == 0: dr.line([P((g / 1000, z0)), P((g / 1000, z1))], fill=(236, 236, 236) if g % 10 else (205, 205, 205))
for g in range(int(np.ceil(z0 * 1000)), int(z1 * 1000) + 1):
    if g % 5 == 0: dr.line([P((u0, g / 1000)), P((u1, g / 1000))], fill=(236, 236, 236) if g % 10 else (205, 205, 205))
def pre(Vx, Fx):
    T = Vx[Fx]; lo = T.min(1); hi = T.max(1)
    return (lo[:, 0] < x0) & (hi[:, 0] > x0) & (hi[:, 1] > min(u0, u1) - 0.01) & (lo[:, 1] < max(u0, u1) + 0.01) & (hi[:, 2] > z0 - 0.01) & (lo[:, 2] < z1 + 0.01)
def draw(Vx, Fx, col, w=1):
    for q in sl(Vx, Fx, 0, x0): dr.line([P(q[0][[1, 2]]), P(q[1][[1, 2]])], fill=col, width=w)
for wn in ('Wheel_L', 'Wheel_R'):
    Wv = B[wn + '_v']; Wf = B[wn + '_f']; draw(Wv, Wf[pre(Wv, Wf)], (70, 160, 70))
mb = pre(V, F)
draw(V, F[mb & (part == 0) & (lab != 4)], (30, 70, 220), 2); draw(V, F[mb & (part == 1) & (lab != 4)], (220, 130, 0), 2)
if 'mount_v' in d.files:
    mv = d['mount_v']; mf = d['mount_f']; draw(mv, mf[pre(mv, mf)], (150, 0, 200), 2)
mc = pre(X, CF)
if LEG:
    kl = sk.legacy_kill(X, json.load(open('mounts.json'))['posts']); fk = kl[CF].any(1)
    draw(X, CF[mc & ~fk], (0, 0, 0), 2); draw(X, CF[mc & fk], (235, 40, 40), 1)
elif SKS and SKS['mode'] == 'cut':
    S = sl(X, CF[mc], 0, x0); rm = sk.inside(S.reshape(-1, 3), SKS).reshape(-1, 2)
    for q, r in zip(S, rm):
        if r.all(): dr.line([P(q[0][[1, 2]]), P(q[1][[1, 2]])], fill=(200, 200, 200), width=1)        # removed by the cut (shown thin grey)
        else: dr.line([P(q[0][[1, 2]]), P(q[1][[1, 2]])], fill=(0, 0, 0), width=2)
    CV, CFc, CI = sk.cap_mesh(X, CF, SKS)
    if len(CFc): draw(CV, CFc, (255, 100, 30), 3)
    for c in SKS['cuts']:                      # the cut planes where they meet this section
        b = np.array(c['box'])
        if b[0, 0] <= x0 <= b[0, 1]:
            i = int(np.argmax(np.abs(c['n'])))
            if i == 1: dr.line([P((c['p'][1], b[2, 0])), P((c['p'][1], b[2, 1]))], fill=(255, 100, 30), width=1)
            if i == 2: dr.line([P((b[1, 0], c['p'][2])), P((b[1, 1], c['p'][2]))], fill=(255, 100, 30), width=1)
else: draw(X, CF[mc], (0, 0, 0), 2)
dr.rectangle((0, H - 70, W, H), fill=(250, 250, 250))
dr.text((8, H - 66), '%s   x = %.1f mm   grid 5 mm' % (TITLE, x0 * 1000), fill=(0, 0, 0), font=fnt)
leg = [((30, 70, 220), 'body'), ((220, 130, 0), 'cover'), ((150, 0, 200), 'body kit (deck, seat flange)'), ((0, 0, 0), 'chassis'), ((70, 160, 70), 'wheel')]
leg += [((235, 40, 40), 'deleted by the f2 renderer')] if LEG else [((255, 100, 30), 'cut face (cap) / cut plane'), ((200, 200, 200), 'removed by the cut')]
xx = 8
for col, t in leg: dr.rectangle((xx, H - 40, xx + 14, H - 30), fill=col); dr.text((xx + 18, H - 43), t, fill=(0, 0, 0), font=fsm); xx += 30 + int(dr.textlength(t, font=fsm))
y_ = 8
for t in NOTES: dr.text((W - 8 - int(dr.textlength(t, font=fnt)), y_), t, fill=(20, 20, 20), font=fnt); y_ += 20
im.save(OUT); print('saved', OUT, im.size)
