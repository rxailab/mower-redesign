# [fix1] D8 check (Blender python): non-adjacent self-intersections of each part's rim faces (side 2) and rim x outer / inner faces
import sys, json, numpy as np
from mathutils.bvhtree import BVHTree
a = sys.argv[sys.argv.index('--') + 1:]; d = np.load(a[0]); V = d['v']; F = d['f'].astype(np.int64); side = d['side']; part = d['part']; lab = d['lab']; R = {}
Vg = V.tolist()
for pn in (0, 1):
    g = {s_: np.nonzero((part == pn) & (side == s_) & (lab != 4))[0] for s_ in (0, 1, 2)}
    T = {s_: BVHTree.FromPolygons(Vg, F[g[s_]].tolist(), all_triangles=True) for s_ in g}
    for s1, s2 in ((2, 2), (0, 2), (1, 2)):
        pr = np.array(T[s1].overlap(T[s2]), np.int64).reshape(-1, 2)
        if len(pr):
            fa = F[g[s1][pr[:, 0]]]; fb = F[g[s2][pr[:, 1]]]; keep = ~(fa[:, :, None] == fb[:, None, :]).any((1, 2))
            if s1 == s2: keep &= pr[:, 0] != pr[:, 1]
            pr = pr[keep]
        ca = V[F[g[s1][pr[:, 0]]]].mean(1) * 1000 if len(pr) else np.zeros((0, 3))
        key = np.round(ca / 5).astype(np.int64); uk, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True) if len(ca) else (np.zeros((0, 3)), np.zeros(0, int), np.zeros(0, int))
        R['p%d_s%dxs%d' % (pn, s1, s2)] = dict(pairs=int(len(pr)), clusters=[dict(n=int(cnt[k]), at_mm=ca[inv.ravel() == k].mean(0).round(1).tolist()) for k in np.argsort(-cnt)[:8]])
        print(pn, s1, s2, len(pr), R['p%d_s%dxs%d' % (pn, s1, s2)]['clusters'][:4], flush=True)
json.dump(R, open(a[1], 'w'), indent=1)
