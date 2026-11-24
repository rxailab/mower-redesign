# [fix2] R2E2 straight-lift test: can the snap-on cover be lifted straight up (+Z) off the body? Every cover vertex (both skins and the
# rims, lid excluded) within 8 mm of the body solid casts a ray +Z; a hit on a body face above it within 40 mm is a collision at that lift.
# usage: python lifttest.py DESIGN.npz [--kit]   (--kit: the cover's hidden kit (legs, pins, pads) is tested too, against the body's deck)
import numpy as np, sys, json
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; lab = d['lab']
body = F[(part == 0) & (lab != 4)]; cov = np.unique(F[(part == 1) & (lab != 4)])
P = V[cov]; bt = cKDTree(V[np.unique(body)]); dd, _ = bt.query(P, distance_upper_bound=0.008); P = P[np.isfinite(dd)]
tests = [('cover skin', P)]
if '--kit' in sys.argv and 'ckit_v' in d.files:
    K = d['ckit_v']; mv = d['mount_v']; mf = d['mount_f']; mk = d['mount_kind']; names = [str(x) for x in d['mount_kinds']]
    tests.append(('cover kit vs deck', K)); deck = mf[mk == names.index('deck')] if 'deck' in names else np.zeros((0, 3), int)
def lift_hits(P, Vb, Fb, maxlift=0.04):
    T = Vb[Fb]; lo = T[:, :, :2].min(1); hi = T[:, :, :2].max(1); cs = 0.002
    gx0, gy0 = lo.min(0) - cs
    i0 = ((lo - [gx0, gy0]) / cs).astype(int); i1 = ((hi - [gx0, gy0]) / cs).astype(int); buck = {}
    for f_ in range(len(T)):
        for a in range(i0[f_, 0], i1[f_, 0] + 1):
            for b in range(i0[f_, 1], i1[f_, 1] + 1): buck.setdefault((a, b), []).append(f_)
    free = np.full(len(P), np.inf)
    ci = ((P[:, :2] - [gx0, gy0]) / cs).astype(int)
    for k in range(len(P)):
        fl = buck.get((ci[k, 0], ci[k, 1]))
        if not fl: continue
        t = T[fl]; p = P[k]; e1 = t[:, 1, :2] - t[:, 0, :2]; e2 = t[:, 2, :2] - t[:, 0, :2]; den = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]
        ok = np.abs(den) > 1e-16; dn = np.where(ok, den, 1.0); da = p[:2] - t[:, 0, :2]
        u = (da[:, 0] * e2[:, 1] - da[:, 1] * e2[:, 0]) / dn; v = (e1[:, 0] * da[:, 1] - e1[:, 1] * da[:, 0]) / dn
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1); z = t[:, 0, 2] + u * (t[:, 1, 2] - t[:, 0, 2]) + v * (t[:, 2, 2] - t[:, 0, 2])
        up = hit & (z > p[2] + 1e-6) & (z < p[2] + maxlift)
        if up.any(): free[k] = (z[up] - p[2]).min()
    return free
out = {}
for nm, Pt in tests:
    Fb = body if nm == 'cover skin' else deck; Vb = V if nm == 'cover skin' else mv
    if not len(Fb): continue
    free = lift_hits(Pt, Vb, Fb); hit = np.isfinite(free)
    r = dict(tested=int(len(Pt)), colliding=int(hit.sum()))
    if hit.any():
        j = np.argsort(free[hit])[:5]; r.update(min_free_lift_mm=round(float(free[hit].min()) * 1000, 2), median_free_lift_mm=round(float(np.median(free[hit])) * 1000, 2),
                                             worst_at_mm=(Pt[hit][j] * 1000).round(1).tolist())
    out[nm] = r; print(nm, r)
print(json.dumps(out))
