# Assets for the product page's lawn-path animation.
# usage: lawn_assets.py GARDEN.png SHORT_GRASS.png OUTDIR [MOWER_PLAN.png DOCK_PLAN.png]
#   GARDEN.png       top-down photo of the garden (generated with an image model; the lawn is found by colour)
#   SHORT_GRASS.png  top-down photo of freshly mowed grass (texture of the mowed stripes)
#   *_PLAN.png       orthographic plan renders (render_site.sh: mplan / dockplan, 1.2 m wide, mower origin 0.15 m right of the
#                    image centre, nose pointing left)
# Writes OUTDIR/garden.webp, mowed_a.webp, mowed_b.webp (light / dark stripes, lawn only), mower.webp, dock.webp and lawn.json:
#   {"size": [w, h], "pxm": px per metre, "mower": {...}, "dock": {...}, "segs": [{"d": svg path, "mow": bool, "rev": bool, "cls": "a"|"b", "w": px}]}
import sys, json, os
import numpy as np
from PIL import Image, ImageFilter
from scipy import ndimage as nd

PXM = 160.0                 # image px per metre (paving slabs ~0.8 m, mower 0.67 m long)
CUT = 0.20 * PXM            # cutting width
ROW = 0.18 * PXM            # row pitch: 2 cm overlap
EDGE = 0.13 * PXM           # blade centre from the lawn edge on the edge pass
PLAN_W, PLAN_PX, PLAN_CX = 1.2, 1200, 0.15      # plan renders: width (m), width (px), image centre in mower x (m)


def lawn_mask(im):
    hsv = np.asarray(im.convert('HSV')).astype(np.float32); H = hsv[..., 0] * 360 / 255; S = hsv[..., 1] / 255; V = hsv[..., 2] / 255
    g = (H > 55) & (H < 120) & (S > 0.30) & (V > 0.18)
    g = nd.binary_opening(g, structure=np.ones((7, 7)))
    lab, _ = nd.label(g); h, w = g.shape; m = lab == lab[h * 6 // 10, w * 45 // 100]
    return nd.binary_fill_holes(nd.binary_closing(m, structure=np.ones((9, 9))))


def smooth1(a, k=9):
    if len(a) < k: return a
    p = np.r_[np.repeat(a[0], k // 2), a, np.repeat(a[-1], k // 2)]; return np.convolve(p, np.ones(k) / k, 'valid')


def extents(dist, level, step=3):
    ys, L, R = [], [], []
    for y in range(0, dist.shape[0], step):
        r = np.nonzero(dist[y] >= level)[0]
        if len(r) > 2: ys.append(y); L.append(r.min()); R.append(r.max())
    return np.array(ys, float), smooth1(np.array(L, float)), smooth1(np.array(R, float))


def dstr(pts): return 'M' + ' L'.join(f'{x:.1f} {y:.1f}' for x, y in pts)


def bez(a, c, b, n=24):
    t = np.linspace(0, 1, n)[:, None]; a, c, b = map(np.array, (a, c, b)); return list(map(tuple, (1 - t) ** 2 * a + 2 * (1 - t) * t * c + t * t * b))


def main():
    G, SH, OUT = sys.argv[1:4]; os.makedirs(OUT, exist_ok=True)
    im = Image.open(G).convert('RGB'); W, H = im.size; m = lawn_mask(im); dist = nd.distance_transform_edt(m)
    # --- dock on the left edge, part way down; the pillar stands just off the lawn, the mower faces left into it
    yd = int(H * 0.585); xe = int(np.nonzero(m[yd])[0].min())
    O = (xe - 4 + 0.25 * PXM, float(yd)); A = (O[0] + 0.50 * PXM, float(yd))
    # --- edge loop (clockwise on screen), starting on the left side above the dock
    ys, L, R = extents(dist, EDGE)
    loop = [(L[0], ys[0])] + [(R[i], ys[i]) for i in range(len(ys))] + [(L[i], ys[i]) for i in range(len(ys) - 1, -1, -1)]
    js = int(np.argmin(np.abs(ys - (yd - 0.55 * PXM)))); S = (L[js], ys[js])
    edge = [(L[i], ys[i]) for i in range(js, -1, -1)] + loop[1:]          # up to the top-left corner, once round, back at the top-left
    # --- rows inside the edge band, alternating direction
    lev = EDGE + 0.5 * ROW; yr, Lr, Rr = extents(dist, lev, step=1)          # rows reach into the edge band: no long-grass corners left
    rows = []; y = yr.min() + 1; k = 0
    while y <= yr.max() - 1:
        i = int(np.argmin(np.abs(yr - y))); a, b = (Lr[i], yr[i]), (Rr[i], yr[i]); rows.append((a, b) if k % 2 == 0 else (b, a)); y += ROW; k += 1
    segs = [{'d': dstr([O, A]), 'mow': False, 'rev': True}]
    segs.append({'d': dstr(bez(A, (S[0] + 0.25 * PXM, A[1]), S)), 'mow': True, 'cls': 'a', 'w': CUT})
    segs.append({'d': dstr(edge), 'mow': True, 'cls': 'a', 'w': CUT})
    prev = edge[-1]
    for k, (a, b) in enumerate(rows):
        segs.append({'d': dstr([prev, a]), 'mow': False, 'rev': False})          # the turn: over the mowed headland, no new stripe
        segs.append({'d': dstr([a, b]), 'mow': True, 'cls': 'a' if k % 2 == 0 else 'b', 'w': CUT + 1}); prev = b
    # --- home: along the bottom / left edge (already mowed), then a hook into the dock, nose first
    jb = int(np.argmin(np.abs(ys - (yd + 0.6 * PXM)))); home = [prev]
    if prev[0] > W / 2: home += [(R[-1], ys[-1])]
    home += [(L[-1], ys[-1])] + [(L[i], ys[i]) for i in range(len(ys) - 1, jb - 1, -1)]
    P1 = home[-1]; segs.append({'d': dstr(home + bez(P1, (A[0] + 0.35 * PXM, P1[1] - 0.1 * PXM), A)[1:]), 'mow': False, 'rev': False})
    segs.append({'d': dstr([A, O]), 'mow': False, 'rev': False})
    # --- mowed lawn: the same photo, the long-grass detail smoothed away, a fine short-grass texture added; light / dark stripe
    base = np.asarray(im).astype(np.float32)
    sm = np.asarray(im.filter(ImageFilter.GaussianBlur(1.6))).astype(np.float32)
    detail = base - np.asarray(im.filter(ImageFilter.GaussianBlur(1.0))).astype(np.float32)          # the photo's own fine grain, kept
    sh = Image.open(SH).convert('L').resize((300, 300), Image.LANCZOS); tile = np.asarray(sh).astype(np.float32)
    tex = np.tile(tile, (H // 300 + 1, W // 300 + 1))[:H, :W]; tex = tex - nd.gaussian_filter(tex, 4)
    lum = sm.mean(-1, keepdims=True); low = nd.gaussian_filter(lum[..., 0], 40)[..., None]           # keep the photo's light falloff, even out its patches
    flat = sm * (0.45 + 0.55 * low / np.maximum(lum, 1))
    alpha = (nd.gaussian_filter(m.astype(np.float32), 0.7) * 255).astype(np.uint8)
    for name, gain, tint, tg in (('mowed_a', 1.10, (1.03, 1.04, 0.97), 0.10), ('mowed_b', 0.93, (0.985, 1.0, 0.99), 0.08)):
        rgb = flat * gain * np.array(tint) + detail * 0.55 + tex[..., None] * tg * np.array([0.8, 1.0, 0.6])
        g_ = rgb.mean(-1, keepdims=True); rgb = g_ + (rgb - g_) * 0.92                                   # a touch less saturated than the long grass
        Image.fromarray(np.dstack([np.clip(rgb, 0, 255).astype(np.uint8), alpha]), 'RGBA').save(f'{OUT}/{name}.webp', 'WEBP', quality=82, method=6)
    im.save(f'{OUT}/garden.webp', 'WEBP', quality=82, method=6)
    # --- the flower bed (no-go zone): the planted / mulched region the lawn wraps round, outlined a little outside its steel edge
    hsv = np.asarray(im.convert('HSV')).astype(np.float32); Sa = hsv[..., 1] / 255; Va = hsv[..., 2] / 255
    bed = ~m & ((Sa > 0.20) | (Va < 0.30)); bed = nd.binary_opening(bed, structure=np.ones((5, 5)))
    yy_, xx_ = np.nonzero(m); box_ = np.zeros_like(m); box_[yy_.min():yy_.max() + 1, xx_.min():xx_.max() + 1] = True
    notch_ = nd.distance_transform_edt(box_ & ~m); cy_, cx_ = np.unravel_index(np.argmax(notch_), notch_.shape)      # deepest non-lawn point inside the lawn's box = in the bed
    lab_, _ = nd.label(bed)
    bed = nd.binary_fill_holes(nd.binary_closing(lab_ == lab_[cy_, cx_], structure=np.ones((15, 15)))); bed = nd.binary_dilation(bed, iterations=6)
    by, bL, bR = extents(nd.distance_transform_edt(bed), 1, step=4)
    bedpts = [(bL[0], by[0])] + [(bR[i], by[i]) for i in range(len(by))] + [(bL[i], by[i]) for i in range(len(by) - 1, -1, -1)]
    info = {'size': [W, H], 'pxm': PXM, 'segs': segs, 'dock': {'x': O[0], 'y': O[1]}, 'bed': dstr(bedpts) + ' Z'}
    if len(sys.argv) > 5:           # plan renders -> image element geometry: the mower origin sits 0.15 m right of the image centre
        sc = PXM * PLAN_W / PLAN_PX
        for key, src in (('mower', sys.argv[4]), ('dockimg', sys.argv[5])):
            p = Image.open(src).convert('RGBA'); w_, h_ = p.size
            p.resize((int(w_ * sc * 2), int(h_ * sc * 2)), Image.LANCZOS).save(f'{OUT}/{"mower" if key == "mower" else "dock"}.webp', 'WEBP', quality=85, method=6)
            info[key] = {'w': w_ * sc, 'h': h_ * sc, 'ox': -(w_ / 2 + PLAN_CX * PLAN_PX / PLAN_W) * sc, 'oy': -h_ / 2 * sc}
    json.dump(info, open(f'{OUT}/lawn.json', 'w'))
    print('rows', len(rows), 'segments', len(segs), 'dock', [round(v) for v in O])


if __name__ == '__main__':
    main()
