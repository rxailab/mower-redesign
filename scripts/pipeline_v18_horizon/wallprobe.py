# [v18c] Where does the wall misbehave? usage: python wallprobe.py SHELL.npz
# inner vertices outside the outer skin (poke-through), wall thinner than 60 % of nominal, inner faces facing the same way as the outer skin near them
import numpy as np, sys
sys.path.insert(0, '.'); from shell_wall import SurfDist
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f'].astype(np.int64); side = d['side']; part = d['part']; lab = d['lab']
for pa in (0, 1):
    mo = (part == pa) & (side == 0) & (lab != 4); mi = (part == pa) & (side == 1)
    sd = SurfDist(V, F[mo]); iv = np.unique(F[mi]); s_, _ = sd(V[iv], 0.02)
    out = iv[s_ > 0]; thin = iv[(s_ < 0) & (-s_ < 0.0015)]
    print('part', pa, 'inner verts', len(iv), 'outside outer', len(out), 'wall < 1.5 mm', len(thin))
    for nm, ids in (('outside', out), ('thin', thin)):
        if not len(ids): continue
        P = V[ids]; key = np.floor(P / 0.01).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True)
        for k in range(min(len(u), 12)):
            m = inv.ravel() == k; print('   %s %3d verts near %s  s %.2f..%.2f mm' % (nm, m.sum(), np.round(P[m].mean(0) * 1000, 1), s_[np.isin(iv, ids[m])].min() * 1000, s_[np.isin(iv, ids[m])].max() * 1000))
    # folded: inner face normal vs nearest outer face normal (inner faces point into the cavity, i.e. opposite to the outer skin)
    Pi = V[F[mi]]; ni = np.cross(Pi[:, 1] - Pi[:, 0], Pi[:, 2] - Pi[:, 0]); ai = np.linalg.norm(ni, axis=1); ni /= np.maximum(ai, 1e-30)[:, None]
    Po = V[F[mo]]; no = np.cross(Po[:, 1] - Po[:, 0], Po[:, 2] - Po[:, 0]); no /= np.maximum(np.linalg.norm(no, axis=1), 1e-30)[:, None]
    _, j = cKDTree(Po.mean(1)).query(Pi.mean(1)); dot = (ni * no[j]).sum(1); bad = (dot > 0.2) & (ai > 1e-13)
    print('   inner faces facing outwards (dot > 0.2 with the nearest outer face):', int(bad.sum()))
    if bad.any():
        C = Pi[bad].mean(1); key = np.floor(C / 0.01).astype(int); u, inv = np.unique(key, axis=0, return_inverse=True)
        for k in range(min(len(u), 12)): m = inv.ravel() == k; print('      %3d faces near %s' % (m.sum(), np.round(C[m].mean(0) * 1000, 1)))
