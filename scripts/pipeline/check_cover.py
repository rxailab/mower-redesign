# Do the hidden parts stay inside?  usage: python check_cover.py SHELL.npz
# Every vertex of the body's deck / screws (mount_*) must lie under the cover, every vertex of the cover's legs (ckit_*) inside the body skin.
import numpy as np, sys
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f']; part = d['part']; lab = d['lab']
def side(Fs, pts, tag):
    P = V[Fs]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20); c = P.mean(1)
    dist, i = cKDTree(c).query(pts); out = ((pts - c[i]) * n[i]).sum(1)          # > 0: on the outer side of the nearest face
    bad = (out > 0.0002) & (dist < 0.004); print('%s: %d of %d vertices outside the skin (worst %.2f mm)' % (tag, bad.sum(), len(pts), out[bad].max() * 1000 if bad.any() else 0.0)); return pts[bad]
b1 = side(F[(part == 1)], d['mount_v'][d['mount_v'][:, 2] > 0.15], 'body deck / screws vs cover')
b2 = side(F[(part == 0) & (lab != 4)], d['ckit_v'], 'cover legs / flange vs body')
b3 = side(F[(part == 0) & (lab != 4)], d['mount_v'], 'body inner parts vs body skin')
for b in (b1, b2, b3):
    if len(b): print('   e.g.', np.round(b[:4], 4).tolist())
