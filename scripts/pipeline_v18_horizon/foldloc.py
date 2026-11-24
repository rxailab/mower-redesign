# [hz] where are the folded inner-skin faces / open edges of the wall solids? usage: foldloc.py NPZ
import numpy as np, sys
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; side = d['side']; lab = d['lab']
P = V[F]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); a = np.linalg.norm(n, axis=1); n = n / np.maximum(a[:, None], 1e-30); c = P.mean(1)
for pa in (0, 1):
    o = (part == pa) & (side == 0) & (lab != 4); i = (part == pa) & (side == 1)
    t = cKDTree(c[o]); _, k = t.query(c[i]); dot = (n[i] * n[o][k]).sum(1)
    bad = np.nonzero(i)[0][(dot > 0.2) & (a[i] > 1e-12)]
    print('part', pa, 'folded inner faces', len(bad))
    if len(bad):
        cb = c[bad] * 1000; key = np.round(cb / 10).astype(int); u, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        for j in np.argsort(-cnt)[:12]: print('   ~%s mm: %d' % ((u[j] * 10).tolist(), cnt[j]))
    m = part == pa; Fs = F[m]; e = np.sort(np.concatenate([Fs[:, [0, 1]], Fs[:, [1, 2]], Fs[:, [2, 0]]]), 1)
    u, cnt = np.unique(e, axis=0, return_counts=True); be = u[cnt == 1]
    print('part', pa, 'open edges', len(be))
    if len(be):
        cb = V[be].mean(1) * 1000; key = np.round(cb / 10).astype(int); uu, cn = np.unique(key, axis=0, return_counts=True)
        for j in np.argsort(-cn)[:8]: print('   ~%s mm: %d' % ((uu[j] * 10).tolist(), cn[j]))
