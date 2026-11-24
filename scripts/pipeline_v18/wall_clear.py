# Wall vs the fixed product (chassis with the mudguard struts removed as design_render.py does, wheels):
# usage: python wall_clear.py SHELL.npz PARTS.npz [--json OUT]   (PARTS.npz = meshes of base.glb in world coordinates: <Name>_v, <Name>_f)
# Reports every chassis region that lies inside the plastic of the body / cover (penetration depth) or closer than `clear` to it, clustered,
# with the outer-skin room at that spot (= what the wall had to fit in).
import numpy as np, sys, json, collections
sys.path.insert(0, '.'); from shell_wall import SurfDist
d = np.load(sys.argv[1]); B = np.load(sys.argv[2]); V = d['v'].astype(float); F = d['f']; part = d['part']; lab = d['lab']
side = d['side'] if 'side' in d.files else np.zeros(len(F), np.uint8); lid = lab == 4
# --kill-z Z: the mudguard struts are removed above Z (design_render.py uses 0.135; 0.07 = the struts gone entirely)
KZ = float(sys.argv[sys.argv.index('--kill-z') + 1]) if '--kill-z' in sys.argv else 0.135
PO = json.load(open('mounts.json'))['posts']; X = B['Chassis_v']
kill = (np.abs(X[:, 1]) > 0.160) & (X[:, 2] > KZ) & (X[:, 0] > -0.06) & (X[:, 0] < 0.29)
for p_ in PO: kill &= (X[:, 0] - p_[0]) ** 2 + (X[:, 1] - p_[1]) ** 2 > 0.0068 ** 2
import strutskit as _sk; SKS = _sk.from_npz(d)      # [fix3-struts] cut spec written by the generator: no vertex deletion, the cut boxes are applied to the samples
if SKS and SKS['mode'] == 'cut': kill[:] = False      # [fix3-struts]
Fc = B['Chassis_f'][~kill[B['Chassis_f']].any(1)]
# sample the chassis faces near the body densely (0.5 mm), so a large face crossing the wall is not missed between its vertices
from scipy.spatial import cKDTree as _T
near = np.isfinite(_T(V[::7]).query(X[Fc].mean(1), distance_upper_bound=0.03, workers=-1)[0]); Fc = Fc[near]; P3 = X[Fc]
ar = 0.5 * np.linalg.norm(np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]), axis=1); ns = np.clip((ar / 0.25e-6).astype(int), 1, 4000)
rng = np.random.default_rng(1); fi = np.repeat(np.arange(len(Fc)), ns); u = rng.random((len(fi), 2)); fl = u.sum(1) > 1; u[fl] = 1 - u[fl]
Xs = np.r_[X[~kill], P3[fi, 0] + (P3[fi, 1] - P3[fi, 0]) * u[:, :1] + (P3[fi, 2] - P3[fi, 0]) * u[:, 1:]]
if SKS and SKS['mode'] == 'cut': Xs = np.r_[Xs[~_sk.inside(Xs, SKS)], _sk.cap_samples(X, B['Chassis_f'].astype(np.int64), SKS)]      # [fix3-struts] cut + capped
print('chassis samples', len(Xs), *(('struts cut:', ', '.join(c_['id'] for c_ in SKS['cuts'])) if SKS and SKS['mode'] == 'cut' else ('kill z', KZ)))      # [fix3-struts]
CLR = 0.001; out = {}
for pa, nm in ((0, 'body'), (1, 'cover')):
    sol = SurfDist(V, F[(part == pa) & ~lid]); so = SurfDist(V, F[(part == pa) & ~lid & (side == 0)])
    s_, fi = sol(Xs, 0.006); m = np.isfinite(s_) & (s_ < CLR)
    o_, _ = so(Xs[m], 0.02)
    P = Xs[m]; dep = s_[m]; room = -o_                                                 # room: how far behind the outer skin the chassis point is
    key = np.floor(P / 0.012).astype(int); cs = collections.defaultdict(list)
    for k, p, dp, rm in zip(map(tuple, key), P, dep, room): cs[k].append((dp, rm, p))
    rows = sorted((min(v, key=lambda q: q[0]) for v in cs.values()), key=lambda q: q[0]); seen = []; lst = []
    for dp, rm, p in rows:
        if any(np.linalg.norm(p - q) < 0.02 for q in seen): continue
        seen.append(p); lst.append(dict(at_mm=(p * 1000).round(1).tolist(), wall_gap_mm=round(float(dp) * 1000, 2), outer_room_mm=round(float(rm) * 1000, 2)))
    out[nm] = dict(chassis_points_inside_wall=int((dep < 0).sum()), min_gap_mm=round(float(dep.min()) * 1000, 2) if len(dep) else None, spots=lst[:20])
    print('%s: %d chassis points inside the wall, %d closer than %.1f mm; tightest spots (gap < 0 = inside the plastic):' % (nm, (dep < 0).sum(), len(dep), CLR * 1000))
    for r in lst[:20]: print('   ', r)
if 'ports_wp' in d.files and len(d['sleeve_f']):      # [fix3-ports] the port frame (closed tunnel / web / ducts behind the slot) vs the chassis (k 100: its caps are long fan triangles)
    SDp_ = SurfDist(d['sleeve_v'].astype(float), d['sleeve_f'], k=100); s_, _ = SDp_(Xs, 0.006); m = np.isfinite(s_) & (np.abs(s_) < CLR)      # [fix3-ports] [fix4-front2] |s|: the
    if m.any(): s_[m] = SurfDist(d['sleeve_v'].astype(float), d['sleeve_f'], k=2000)(Xs[m], 0.03)[0]; m = np.isfinite(s_) & (s_ < CLR)      # [fix3-ports] candidates re-measured exactly  (face-centroid sign is unreliable off the face)
    P = Xs[m]; dep = s_[m]      # [fix3-ports]
    sa_, _ = SurfDist(d['sleeve_v'].astype(float), d['sleeve_f'], k=100)(Xs[np.abs(Xs[:, 1]) < 0.07], 0.02)      # [fix3-ports]
    sa_ = np.abs(sa_)      # [fix4-front2] unsigned (inside-ness is the exact k 2000 count above): the f4 scarfed duct ends gave a -11.3 'gap' 11 mm away (sign of a far face)
    out['ports'] = dict(chassis_points_inside_wall=int((dep < 0).sum()), closer_than_clear=int(len(dep)), min_gap_mm=round(float(np.nanmin(np.where(np.isfinite(sa_), sa_, np.nan))) * 1000, 2),      # [fix3-ports]
                        tightest_at_mm=(Xs[np.abs(Xs[:, 1]) < 0.07][int(np.nanargmin(np.where(np.isfinite(sa_), sa_, np.nan)))] * 1000).round(1).tolist())      # [fix3-ports]
    print('ports: %d chassis points inside the port frame, %d closer than %.1f mm; min gap %.2f mm at %s' % (out['ports']['chassis_points_inside_wall'], len(dep), CLR * 1000, out['ports']['min_gap_mm'], out['ports']['tightest_at_mm']))      # [fix3-ports]
if '--json' in sys.argv: json.dump(out, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
