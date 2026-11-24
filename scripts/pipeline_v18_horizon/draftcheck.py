# [hz] Draft of the visible (outer) skins against a straight +Z pull: the body's cavity and the cover's cavity both open upwards,
# the cover / body inner skins are formed by cores pulled down. Per part (0 body, 1 cover, lid = lab 4) the area of the outer skin
# by draft angle (asin of the outward normal's z component): undercut (< 0), 0-0.5, 0.5-1, 1-2, >= 2 deg, and where the faces
# below a limit lie (x / z ranges of clusters). Rim faces (side 2: rolled edges, edge faces, labyrinth) are not A-surface and are
# excluded, as are the faces of openings' walls that face sideways inside a window (reported separately as 'window walls').
# usage: ../venv/bin/python draftcheck.py NPZ [--json OUT] [--limit 1.0] [--zmin 0.0]
import numpy as np, json, sys
a = sys.argv[1:]; NPZ = a[0]; OUT = a[a.index('--json') + 1] if '--json' in a else None
LIM = float(a[a.index('--limit') + 1]) if '--limit' in a else 1.0; ZMIN = float(a[a.index('--zmin') + 1]) if '--zmin' in a else 0.0
d = np.load(NPZ); V = d['v'].astype(float); F = d['f']; lab = d['lab']; side = d['side'] if 'side' in d.files else np.zeros(len(F), np.uint8)
part = d['part'] if 'part' in d.files else np.zeros(len(F), np.uint8)
P = V[F]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); ar = 0.5 * np.linalg.norm(n, axis=1); n = n / np.maximum(2 * ar[:, None], 1e-30)
c = P.mean(1); dr = np.degrees(np.arcsin(np.clip(n[:, 2], -1, 1)))
BINS = [(-90, 0), (0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 90.01)]
def clusters(m, cell=0.01):
    """x / z boxes of the low-draft faces (1 cm cells, 8-connected)."""
    if not m.any(): return []
    from scipy.ndimage import label
    cx = np.floor(c[m, 0] / cell).astype(int); cz = np.floor(c[m, 2] / cell).astype(int); cy = np.sign(c[m, 1]).astype(int)
    out = []
    for sg in (-1, 0, 1):
        k = cy == sg
        if not k.any(): continue
        x0, z0 = cx[k].min(), cz[k].min(); G = np.zeros((cx[k].max() - x0 + 1, cz[k].max() - z0 + 1)); A = np.zeros_like(G)
        np.add.at(A, (cx[k] - x0, cz[k] - z0), ar[m][k]); G = A > 0; L, nl = label(G, np.ones((3, 3)))
        for i in range(1, nl + 1):
            ii = np.argwhere(L == i); s = float(A[L == i].sum())
            if s < 2e-5: continue
            out.append(dict(side='L' if sg > 0 else ('R' if sg < 0 else 'C'), area_cm2=round(s * 1e4, 2), x_mm=[int((ii[:, 0].min() + x0) * cell * 1000), int((ii[:, 0].max() + x0 + 1) * cell * 1000)],
                            z_mm=[int((ii[:, 1].min() + z0) * cell * 1000), int((ii[:, 1].max() + z0 + 1) * cell * 1000)]))
    return sorted(out, key=lambda q: -q['area_cm2'])[:12]
rep = {}
for nm, m in (('body', (part == 0) & (side == 0) & (lab != 4)), ('cover', (part == 1) & (side == 0) & (lab != 4)), ('lid', (lab == 4) & (side == 0))):
    m = m & (c[:, 2] > ZMIN) & (ar > 0)
    if not m.any(): continue
    r = {'area_cm2': round(float(ar[m].sum()) * 1e4, 1)}
    for lo, hi in BINS: r['%g..%g' % (lo, hi)] = round(float(ar[m & (dr >= lo) & (dr < hi)].sum()) * 1e4, 2)
    r['min_deg'] = round(float(dr[m].min()), 2); r['p0.1_deg'] = round(float(np.percentile(dr[m], 0.1)), 2)
    r['below_limit_cm2'] = round(float(ar[m & (dr < LIM)].sum()) * 1e4, 2); r['undercut_cm2'] = r['-90..0']
    r['below_limit_where'] = clusters(m & (dr < LIM)); r['undercut_where'] = clusters(m & (dr < 0))
    rep[nm] = r
rep['limit_deg'] = LIM
print(json.dumps(rep, indent=1))
if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
