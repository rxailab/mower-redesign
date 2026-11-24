# [orbit] check_cover.py plus: non-finite vertices skipped, cover kit split into snap legs (ckit_lab 0) and the visual edge wall (ckit_lab 1)
import numpy as np, sys
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f']; part = d['part']; lab = d['lab']
if 'side' in d.files: so_ = d['side'] == 0; F = F[so_]; part = part[so_]; lab = lab[so_]      # [v18c] real wall: the outer skin decides (inner skins face the cavity)
Fn = F[lab != 4]; e = np.sort(np.concatenate([Fn[:, [0, 1]], Fn[:, [1, 2]], Fn[:, [2, 0]]]), 1); ue, cnt = np.unique(e, axis=0, return_counts=True)
BT = cKDTree(V[np.unique(ue[cnt == 1].ravel())]); SEAM = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0035
def side(Fs, pts, tag):
    pts = pts[np.isfinite(pts).all(1)]; pts = pts[BT.query(pts)[0] > SEAM]      # the rolled part-line edges themselves are judged by eye, not by this test
    P = V[Fs]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20); c = P.mean(1)
    dist, i = cKDTree(c).query(pts); out = ((pts - c[i]) * n[i]).sum(1)          # > 0: on the outer side of the nearest face
    bad = (out > 0.0002) & (dist < 0.004); print('%s: %d of %d vertices outside the skin (worst %.2f mm)' % (tag, bad.sum(), len(pts), out[bad].max() * 1000 if bad.any() else 0.0)); return pts[bad]
ck = d['ckit_v']; cl = d['ckit_lab']; cf = d['ckit_f']; leg = np.unique(cf[cl == 0].ravel())
b1 = side(F[(part == 1)], d['mount_v'][d['mount_v'][:, 2] > 0.15], 'body deck / screws vs cover')
b2 = side(F[(part == 0) & (lab != 4)], ck[leg], 'cover snap legs vs body')
b3 = side(F[(part == 0) & (lab != 4)], d['mount_v'], 'body inner parts vs body skin')
print('non-finite mount vertices:', int((~np.isfinite(d['mount_v']).all(1)).sum()))
for b in (b1, b2, b3):
    if len(b): print('   e.g.', np.round(b[:6], 4).tolist())
