# [fix3-labyrinth] Labyrinth check of the cover / body joint.   usage: python labcheck.py DESIGN.npz [OUT.json] [--step MM]
#  1. static clearance: every cover-solid vertex near the joint -> body solid (wall + upstand) and deck solid; every body vertex near the
#     joint -> cover solid (exact point-to-triangle signed distances, closed solids: < 0 = inside the plastic)
#  2. straight lift: the same, with the cover moved up by s = 0 .. 12 mm (both directions), min clearance over the whole stroke
#  3. per station along the part line (every ~5 mm): rays from the gap mouth into the joint (along the inward skin normal, tilted
#     +-15 / +-30 deg across the line and +-20 deg along it) are marched until they enter either solid: the depth reached. A joint with a
#     labyrinth stops every ray within the wall stack; without one the level rays run straight into the cavity (> 10 mm)
#  4. per station: upstand overlap = how far the body upstand rises above the cover's lower edge (along the in-surface normal of the line)
import numpy as np, sys, json
sys.path.insert(0, '.'); from shell_wall import boundary_loops


from labyrinthkit import RobustSurfDist as SurfDist      # exact distance, vertical-ray-parity sign
from scipy.spatial import cKDTree
a = sys.argv[1:]; STEP = 0.00025
if '--step' in a: i = a.index('--step'); STEP = float(a[i + 1]) / 1000; del a[i:i + 2]
d = np.load(a[0]); OUT = a[1] if len(a) > 1 else None
V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; side = d['side']; lab = d['lab']
mv = d['mount_v'].astype(float); mf = d['mount_f'].astype(np.int64); MKN = [str(x) for x in d['mount_kinds']]; Fd = mf[d['mount_kind'] == MKN.index('deck')]
Fb = F[(part == 0) & (lab != 4)]; Fc = F[(part == 1) & (lab != 4)]
SB, SC, SDk = SurfDist(V, Fb), SurfDist(V, Fc), SurfDist(mv, Fd, sign_within=0.0014)
rep = {}
# ---- the two seam loops (free boundary of each outer skin: cover -> the longest loop; body -> the highest long loop)
def loops(m):
    out = []
    for lp, _ in boundary_loops(F[m], len(V)):
        if len(lp) > 50: out.append(lp)
    return out
Lc = max(loops((part == 1) & (side == 0) & (lab != 4)), key=len)
Lb = max([l_ for l_ in loops((part == 0) & (side == 0)) if len(l_) > 1000], key=lambda l_: V[l_, 2].mean())
# ---- 1 + 2: clearance at rest and over the lift stroke
cv_ = np.unique(Fc); bv_ = np.unique(Fb)
nb_, _ = cKDTree(V[bv_]).query(V[cv_], distance_upper_bound=0.008); Pc = V[cv_[np.isfinite(nb_)]]
nc_, _ = cKDTree(V[cv_]).query(V[bv_], distance_upper_bound=0.008); Pb = V[bv_[np.isfinite(nc_)]]
dk_ = np.unique(Fd); nd_, _ = cKDTree(Pc).query(mv[dk_], distance_upper_bound=0.012); Pd = mv[dk_[np.isfinite(nd_)]]
strokes = np.arange(0.0, 0.012 + 1e-9, STEP); lift = []
for s in strokes:
    e = np.array([0, 0, s]); r = {}
    for nm, X, S in (('cover->body', Pc + e, SB), ('body->cover', Pb - e, SC), ('cover->deck', Pc + e, SDk), ('deck->cover', Pd - e, SC)):
        q, _ = S(X, 0.003); q = q[np.isfinite(q)]; r[nm] = (float(q.min()) if len(q) else np.inf, int((q < 0).sum()), int((q < 0.0005).sum()))
    lift.append((s, r))
def summ(nm):
    mins = [(r[nm][0], s) for s, r in lift]; m, s = min(mins); return dict(min_mm=round(m * 1000, 3), at_lift_mm=round(s * 1000, 2), inside_total=sum(r[nm][1] for _, r in lift),
                                                                       under_0p5_total=sum(r[nm][2] for _, r in lift), at_rest_min_mm=round(lift[0][1][nm][0] * 1000, 3))
rep['clearance'] = {nm: summ(nm) for nm in ('cover->body', 'body->cover', 'cover->deck', 'deck->cover')}
rep['clearance']['tested'] = dict(cover_verts=int(len(Pc)), body_verts=int(len(Pb)), deck_verts=int(len(Pd)), stroke_mm=[0, 12], step_mm=STEP * 1000)
rep['clearance']['min_over_stroke_mm'] = round(min(v['min_mm'] for k, v in rep['clearance'].items() if k != 'tested'), 3)
# ---- 3 + 4: stations
Q = V[Lc]; sl = np.linalg.norm(np.roll(Q, -1, 0) - Q, axis=1); s_ = np.r_[0, np.cumsum(sl)[:-1]]; Ltot = sl.sum()
st = np.searchsorted(s_, np.arange(0, Ltot, 0.005)); P0 = Q[st]; tg = Q[(st + 3) % len(Q)] - Q[(st - 3) % len(Q)]; tg /= np.linalg.norm(tg, axis=1, keepdims=True)
Fo = F[(part == 1) & (side == 0) & (lab != 4)]; Po = V[Fo]; no = np.cross(Po[:, 1] - Po[:, 0], Po[:, 2] - Po[:, 0]); no /= np.maximum(np.linalg.norm(no, axis=1, keepdims=True), 1e-30)
_, fi = cKDTree(Po.mean(1)).query(P0, k=12); ne = no[fi].mean(1); ne -= (ne * tg).sum(1, keepdims=True) * tg; ne /= np.linalg.norm(ne, axis=1, keepdims=True)
bq, _ = cKDTree(V[Lb]).query(P0); bb = np.cross(tg, ne); sg = np.sign(((P0 - V[Lb][cKDTree(V[Lb]).query(P0)[1]]) * bb).sum(1)); bb *= np.where(sg == 0, 1, sg)[:, None]   # b: body -> cover
Pbq = V[Lb][cKDTree(V[Lb]).query(P0)[1]]
M = 0.5 * (P0 + Pbq) + ne * 0.0006                                                   # gap mouth, just outside both skins
dirs = []
for al in (-30, -15, 0, 15, 30):
    for be in (-20, 0, 20):
        dirs.append((al, be))
depth = np.zeros((len(M), len(dirs)))
ts = np.arange(0.0, 0.0102, 0.00005)
for j, (al, be) in enumerate(dirs):
    dv = -ne * np.cos(np.radians(al)) + bb * np.sin(np.radians(al)); dv = dv * np.cos(np.radians(be)) + tg * np.sin(np.radians(be)); dv /= np.linalg.norm(dv, axis=1, keepdims=True)
    hit = np.full(len(M), np.inf)
    for t in ts:
        live = ~np.isfinite(hit)
        if not live.any(): break
        X = M[live] + dv[live] * t; q1, _ = SB(X, 0.002); q2, _ = SC(X, 0.002); q3, _ = SDk(X, 0.002)
        h = (q1 < 0) | (q2 < 0) | (q3 < 0); idx = np.nonzero(live)[0][h]; hit[idx] = t
    depth[:, j] = hit
dmax = np.where(np.isfinite(depth), depth, 0.0102).max(1); level = dirs.index((0, 0))
# overlap: body points near the station plane (tg) furthest along b, minus the cover's lowest point along b
Bv = V[np.unique(F[(part == 0) & (side == 2)])]; Cv = V[np.unique(F[(part == 1) & (side == 2) & (lab != 4)])]
tb = cKDTree(Bv); tc = cKDTree(Cv); ov = np.full(len(M), np.nan)
for k in range(len(M)):
    ib = tb.query_ball_point(M[k], 0.007); ic = tc.query_ball_point(M[k], 0.004)
    if not ib or not ic: continue
    B_ = Bv[ib]; C_ = Cv[ic]; B_ = B_[np.abs((B_ - M[k]) @ tg[k]) < 0.0004]; C_ = C_[np.abs((C_ - M[k]) @ tg[k]) < 0.0004]
    if len(B_) and len(C_): ov[k] = ((B_ - M[k]) @ bb[k]).max() - ((C_ - M[k]) @ bb[k]).min()
rep['stations'] = dict(n=int(len(M)), spacing_mm=5.0, rays_per_station=len(dirs),
                       level_ray_depth_mm=dict(max=round(float(np.where(np.isfinite(depth[:, level]), depth[:, level], 0.0102).max()) * 1000, 2), median=round(float(np.median(np.where(np.isfinite(depth[:, level]), depth[:, level], 0.0102))) * 1000, 2)),
                       any_ray_depth_mm=dict(max=round(float(dmax.max()) * 1000, 2), p95=round(float(np.percentile(dmax, 95)) * 1000, 2), median=round(float(np.median(dmax)) * 1000, 2)),
                       stations_with_a_ray_past_6mm=int((dmax > 0.006).sum()), stations_with_a_ray_past_10mm=int((dmax >= 0.0101).sum()),
                       upstand_overlap_above_cover_edge_mm=dict(min=round(float(np.nanmin(ov)) * 1000, 2), p5=round(float(np.nanpercentile(ov, 5)) * 1000, 2), median=round(float(np.nanmedian(ov)) * 1000, 2), measured=int(np.isfinite(ov).sum())))
worst = np.argsort(-dmax)[:6]; rep['stations']['deepest_at_mm'] = [dict(at=(M[k] * 1000).round(1).tolist(), depth=round(float(dmax[k]) * 1000, 2)) for k in worst]
lo = np.argsort(np.where(np.isfinite(ov), ov, 9))[:6]; rep['stations']['lowest_overlap_at_mm'] = [dict(at=(M[k] * 1000).round(1).tolist(), overlap=round(float(ov[k]) * 1000, 2)) for k in lo]
print(json.dumps(rep, indent=1))
if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
