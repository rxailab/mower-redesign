# [fix3-lights] Checks of the light kit (lightkit.py arrays lights_*) in a design npz.
# usage: python lightcheck.py DESIGN.npz dev/base_parts.npz [--ref base_f2.npz] [--json OUT]
#  1. topology: lens solids and every kit / harness component closed (0 boundary / 0 non-manifold edges)
#  2. lens: flush with the f2 A-surface (--ref), wall 2.5, gap to the body opening (min / max over the outline)
#  3. clearances of every kit / harness kind to: body solid, cover solid, deck, other body kit, cover kit, lid kits, sleeves, nose floor, real chassis
#     (struts cut as in the renderer), wheels, casters, controls; designed contacts are listed and checked against their own limits
#  4. sight lines: rays from outside through the 0.4 mm gap (and through the lens windows) - what is the first thing seen?
#  5. halo feed (moves with the lid): clearance to the control panel / emitter / well over the lid stroke -0.44 .. 75 deg
import numpy as np, sys, json, collections
from scipy.spatial import cKDTree
sys.path.insert(0, '.'); from shell_wall import SurfDist
import lidmech
a = sys.argv; d = np.load(a[1]); B = np.load(a[2]); OUT = a[a.index('--json') + 1] if '--json' in a else None
REF = np.load(a[a.index('--ref') + 1]) if '--ref' in a else None
V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; side = d['side']; lab = d['lab']
KINDS = [str(k) for k in d['lights_kinds']]; rep = {}
def topo(Vv, Ff):
    e = np.concatenate([Ff[:, [0, 1]], Ff[:, [1, 2]], Ff[:, [2, 0]]]); key = np.minimum(e[:, 0], e[:, 1]) * (len(Vv) + 1) + np.maximum(e[:, 0], e[:, 1])
    _, c = np.unique(key, return_counts=True); return int((c == 1).sum()), int((c > 2).sum())
def comps(Ff):
    from scipy.sparse import coo_matrix; from scipy.sparse.csgraph import connected_components
    n = int(Ff.max()) + 1; e = np.r_[Ff[:, [0, 1]], Ff[:, [1, 2]]]; _, cl = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)), directed=False)
    return cl[Ff[:, 0]]
def samples(Vv, Ff, dens=0.25e-6, rng=np.random.default_rng(3)):
    P = Vv[Ff]; ar = 0.5 * np.linalg.norm(np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]), axis=1); ns = np.clip((ar / dens).astype(int), 0, 200)
    fi = np.repeat(np.arange(len(Ff)), ns); u = rng.random((len(fi), 2)); fl = u.sum(1) > 1; u[fl] = 1 - u[fl]
    return np.r_[Vv[np.unique(Ff)], P.mean(1), P[fi, 0] + (P[fi, 1] - P[fi, 0]) * u[:, :1] + (P[fi, 2] - P[fi, 0]) * u[:, 1:]]
from shell_wall import _pt_tri
class SignedDist:
    """Exact unsigned distance (closest point among the k faces with the nearest centroids) with a robust sign: angle-weighted pseudo-normals of the
    closest feature (face / edge / vertex). Every closed connected component is first oriented outward (signed volume), so the target's own winding
    does not matter; open components (the chassis scan) give an unsigned distance (sign +)."""
    def __init__(s, V, F, k=24, trust=0.0013):      # trust: the sign is used only this close to the surface (every target near the kit is a thin wall)
        import lightkit; V = np.asarray(V, float); F = lightkit.consistent(np.asarray(F, np.int64)); cl = comps(F); closed = np.zeros(len(F), bool)
        for c in np.unique(cl):
            m = cl == c; b, nm = topo(V, F[m])
            if b == 0:
                P = V[F[m]]; vol = np.einsum('ij,ij->i', P[:, 0], np.cross(P[:, 1], P[:, 2])).sum(); closed[m] = True
                if vol < 0: F[m] = F[m][:, ::-1]
        s.V = V; s.F = F; s.closed = closed; P = V[F]; fn = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); fa = np.linalg.norm(fn, axis=1); s.fn = fn / np.maximum(fa, 1e-30)[:, None]
        ev = np.roll(P, -1, 1) - P; ev /= np.maximum(np.linalg.norm(ev, axis=2, keepdims=True), 1e-30); ang = np.arccos(np.clip(-(ev * np.roll(ev, 1, 1)).sum(2), -1, 1))
        ang = ang * (fa > 2e-12)[:, None]                                                     # slivers (kept in the topology by the final weld) carry no weight
        vn = np.zeros((len(V), 3))
        for c in range(3): np.add.at(vn, F[:, c], s.fn * ang[:, c:c + 1])
        s.vn = vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-30)
        e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]); key = np.minimum(e[:, 0], e[:, 1]) * (len(V) + 1) + np.maximum(e[:, 0], e[:, 1])
        uk, inv = np.unique(key, return_inverse=True); en = np.zeros((len(uk), 3)); np.add.at(en, inv, np.tile(s.fn * (fa > 2e-12)[:, None], (3, 1))); s.ekey = uk; s.en = en / np.maximum(np.linalg.norm(en, axis=1, keepdims=True), 1e-30)
        s.tr = cKDTree(P.mean(1)); s.k = min(k, len(F)); s.P = P; s.trust = trust
    def __call__(s, X, upper=0.03):
        out = np.full(len(X), np.inf)
        for s0 in range(0, len(X), 50000):
            x = X[s0:s0 + 50000]; dd, ii = s.tr.query(x, k=s.k, distance_upper_bound=upper, workers=-1); ok = np.isfinite(dd); ii = np.where(ok, ii, 0)
            xx = np.repeat(x, s.k, 0); T = s.P[ii.ravel()]; q = _pt_tri(xx, T[:, 0], T[:, 1], T[:, 2]); dist = np.linalg.norm(xx - q, axis=1).reshape(-1, s.k)
            dist = np.where(ok, dist, np.inf); j = np.argmin(dist, 1); r = np.arange(len(x)); fi = ii[r, j]; dm = dist[r, j]; Q = q.reshape(-1, s.k, 3)[r, j]
            T = s.P[fi]; e0 = T[:, 1] - T[:, 0]; e1 = T[:, 2] - T[:, 0]; v2 = Q - T[:, 0]; d00 = (e0 * e0).sum(1); d01 = (e0 * e1).sum(1); d11 = (e1 * e1).sum(1)
            d20 = (v2 * e0).sum(1); d21 = (v2 * e1).sum(1); den = np.maximum(d00 * d11 - d01 * d01, 1e-30); bv = (d11 * d20 - d01 * d21) / den; bw = (d00 * d21 - d01 * d20) / den; bu = 1 - bv - bw
            Bc = np.c_[bu, bv, bw]; eps = 1e-6; small = Bc < eps; ns = small.sum(1); n = s.fn[fi].copy()
            m1 = ns == 1                                                                         # on an edge: opposite the small coordinate
            if m1.any():
                k0 = np.argmax(small[m1], 1); Fm = s.F[fi[m1]]; a_ = Fm[np.arange(m1.sum()), (k0 + 1) % 3]; b_ = Fm[np.arange(m1.sum()), (k0 + 2) % 3]
                key = np.minimum(a_, b_) * (len(s.V) + 1) + np.maximum(a_, b_); n[m1] = s.en[np.searchsorted(s.ekey, key)]
            m2 = ns >= 2                                                                         # on a vertex
            if m2.any(): k1 = np.argmax(~small[m2], 1); n[m2] = s.vn[s.F[fi[m2]][np.arange(m2.sum()), k1]]
            sg = np.sign(((x - Q) * n).sum(1)); sg = np.where(sg == 0, 1, sg); sg = np.where(s.closed[fi] & (dm < s.trust), sg, 1)
            out[s0:s0 + 50000] = np.where(np.isfinite(dm), dm * sg, np.inf)
        return out
# ---- 1. topology
LV, LF, LL, LS, LI = d['lights_lens_v'], d['lights_lens_f'].astype(np.int64), d['lights_lens_lab'], d['lights_lens_side'], d['lights_lens_id']
rep['lens_topology'] = {('front', 'rear')[i]: dict(zip(('boundary_edges', 'nonmanifold_edges'), topo(LV, LF[LI == i]))) for i in np.unique(LI)}
def comp_topo(pre):
    Vv, Ff, kk = d[pre + '_v'], d[pre + '_f'].astype(np.int64), d[pre + '_kind']; cl = comps(Ff); bad = []; n = 0
    for c in np.unique(cl):
        m = cl == c; b, nm = topo(Vv, Ff[m]); n += 1; Fm = Ff[m]; e = np.concatenate([Fm[:, [0, 1]], Fm[:, [1, 2]], Fm[:, [2, 0]]]); dk = e[:, 0] * (len(Vv) + 1) + e[:, 1]
        nm += int((~np.isin(e[:, 1] * (len(Vv) + 1) + e[:, 0], dk)).sum()) if not b else 0      # + edges not paired with their reverse (inconsistent winding)
        if b or nm: bad.append(dict(kind=KINDS[int(kk[m][0])], faces=int(m.sum()), boundary=b, nonmanifold=nm, at_mm=(Vv[Ff[m]].mean((0, 1)) * 1000).round(1).tolist()))
    return dict(components=n, open_components=len(bad), open=bad[:12])
rep['kit_topology'] = comp_topo('lights_kit'); rep['harness_topology'] = comp_topo('lights_harn')
print('topology', json.dumps(rep['lens_topology']), json.dumps({k: rep[k]['open_components'] for k in ('kit_topology', 'harness_topology')}), flush=True)
# ---- 2. lens: flush, gap
lo = np.unique(LF[LS == 0])
if REF is not None:      # the lens outer face against the f2 body outer skin in the band (lab 2 / 3 / 6 / 9): same A-surface
    Rv = REF['v'].astype(float); Rf = REF['f'].astype(np.int64); m = np.isin(REF['lab'], [2, 3, 6, 9]) & (REF['side'] == 0)
    Pl = LV[lo]; s_ = None
    # distance of each lens vertex to its outline (in the band's end view) to exclude the rolled edge / old groove zone
    import shapely; from shapely.geometry import LineString
    dd = np.zeros(len(Pl))
    for sg in (1, -1):      # [fix4-front2] the outline from the lens itself (band z / h are parameters; f3: 0.174 / 0.020)
        mm = np.sign(Pl[:, 0]) == sg; z0_, z1_ = Pl[mm][:, 2].min(), Pl[mm][:, 2].max(); y1_ = np.abs(Pl[mm][:, 1]).max(); zc = 0.5 * (z0_ + z1_); hh_ = 0.5 * (z1_ - z0_)
        o = LineString([(-(y1_ - hh_), zc), (y1_ - hh_, zc)]).buffer(hh_, quad_segs=48); dd[mm] = shapely.distance(o.boundary, shapely.points(Pl[mm][:, 1:3]))
    # [fix4-front2] the f2 reference skin carries a 0.6 x 0.3 groove round ITS band outline (z 0.174, h 0.020, w 0.26): where the f4 lens outline differs
    # (band 17.5 raised) the lens crosses that old groove line, so the lens points within 0.8 of the f2 outline are excluded too (reported separately)
    zr_, hr_, wr_ = [float(x_) for x_ in (a[a.index('--ref-band') + 1].split(',') if '--ref-band' in a else (0.174, 0.020, 0.26))]
    o2_ = LineString([(-(wr_ / 2 - hr_ / 2), zr_), (wr_ / 2 - hr_ / 2, zr_)]).buffer(hr_ / 2, quad_segs=48); dr_ = shapely.distance(o2_.boundary, shapely.points(Pl[:, 1:3]))      # [fix4-front2]
    # [fix4-front2] the pupil rings / windows / light windows (labs != 2) sit at the band centre: f2 skin at z 0.174, the f4 lens at its own centre. Where the
    # two differ, the f2 skin is no reference: lens points within 0.8 of the f2 feature faces or of the lens' own feature faces (band (y, z) view) are excluded
    from shapely.geometry import Polygon as _Pg      # [fix4-front2]
    def _foot(Vv, Ff, sel):      # [fix4-front2] (y, z) footprint of the faces sel, per x side, grown 0.8
        out = {}
        for sg in (1, -1):
            T = Vv[Ff[sel & (np.sign(Vv[Ff[:, 0], 0]) == sg)]][:, :, 1:3]; e1_, e2_ = T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]; T = T[np.abs(e1_[:, 0] * e2_[:, 1] - e1_[:, 1] * e2_[:, 0]) > 1e-14]
            out[sg] = shapely.union_all(shapely.polygons(T)).buffer(0.0008, quad_segs=4) if len(T) else None
        return out
    cR_ = Rv[Rf].mean(1); fR_ = _foot(Rv, Rf, (REF['side'] == 0) & np.isin(REF['lab'], [3, 6, 9]) & (np.abs(cR_[:, 0]) > 0.25) & (cR_[:, 2] > 0.155) & (cR_[:, 2] < 0.195))      # [fix4-front2]
    fL_ = _foot(LV, LF, (LS == 0) & (LL != 2))      # [fix4-front2]
    fz_ = np.zeros(len(Pl), bool)      # [fix4-front2]
    for sg in (1, -1):
        mm = np.sign(Pl[:, 0]) == sg
        for g_ in (fR_[sg], fL_[sg]):
            if g_ is not None: fz_[mm] |= shapely.contains_xy(g_, Pl[mm][:, 1], Pl[mm][:, 2])
    s_ = SignedDist(Rv, Rf[m], k=160, trust=0.0)(Pl, 0.005); ok = np.isfinite(s_); far = ok & (dd > 0.0008) & (dr_ > 0.0008) & ~fz_      # unsigned exact distance to the f2 A-surface  [fix4-front2] & dr_ & ~fz_; k 24 -> 160: the f2 band skin has long slivers whose centroids lie far from their closest point (0.3 mm false deviations)
    rep['lens_flush_mm'] = dict(max_abs_dev_away_from_edge=round(float(np.abs(s_[far]).max()) * 1000, 4), n=int(far.sum()),
                                p99_9=round(float(np.percentile(np.abs(s_[far]), 99.9)) * 1000, 4), n_over_0_02=int((np.abs(s_[far]) > 2e-5).sum()),      # [fix4-front2]
                                feature_zone_max_dev=round(float(np.abs(s_[ok & (dd > 0.0008) & fz_]).max()) * 1000, 3) if (ok & (dd > 0.0008) & fz_).any() else None,      # [fix4-front2]
                                edge_zone_max_dev=round(float(np.abs(s_[ok & (dd <= 0.0008)]).max()) * 1000, 3),      # [fix4-front2] own outline only
                                ref_groove_zone_max_dev=round(float(np.abs(s_[ok & (dd > 0.0008) & (dr_ <= 0.0008)]).max()) * 1000, 3) if (ok & (dd > 0.0008) & (dr_ <= 0.0008)).any() else None)      # [fix4-front2]
rim = np.unique(LF[LS == 2]); rep['lens_gap_to_body_mm'] = 'see below'
print('lens', rep.get('lens_flush_mm'), rep['lens_gap_to_body_mm'], flush=True)
# ---- 3. clearances
X = B['Chassis_v'].astype(float); PO = json.load(open('mounts.json'))['posts']
kill = (np.abs(X[:, 1]) > 0.160) & (X[:, 2] > 0.135) & (X[:, 0] > -0.06) & (X[:, 0] < 0.29)
for p_ in PO: kill &= (X[:, 0] - p_[0]) ** 2 + (X[:, 1] - p_[1]) ** 2 > 0.0068 ** 2
Fc = B['Chassis_f'][~kill[B['Chassis_f']].any(1)]
mv, mf, mk = d['mount_v'].astype(float), d['mount_f'].astype(np.int64), d['mount_kind']; MKN = [str(x) for x in d['mount_kinds']]
c_ = np.load('control.npz')
TG = {'body': (V, F[(part == 0) & (lab != 4)]), 'cover': (V, F[(part == 1) & (lab != 4)]), 'deck': (mv, mf[mk == MKN.index('deck')]),
      'body_kit': (mv, mf[mk != MKN.index('deck')]), 'cover_kit': (d['ckit_v'], d['ckit_f']), 'lid_well': (d['kitf_v'], d['kitf_f']), 'lid_kit': (d['kitm_v'], d['kitm_f']),
      'sleeves': (d['sleeve_v'], d['sleeve_f']), 'nose_floor': (d['plinth_v'], d['plinth_f']), 'chassis': (X, Fc), 'wheels': None, 'controls': (c_['v'], c_['f']),
      'ctl_standin': (d['ctlsi_v'], d['ctlsi_f']), 'lens': (LV, LF)}
WV = np.r_[B['Wheel_L_v'], B['Wheel_R_v'], B['Caster_L_v'], B['Caster_R_v']]; o_ = np.cumsum([0, len(B['Wheel_L_v']), len(B['Wheel_R_v']), len(B['Caster_L_v'])])
TG['wheels'] = (WV, np.r_[B['Wheel_L_f'], B['Wheel_R_f'] + o_[1], B['Caster_L_f'] + o_[2], B['Caster_R_f'] + o_[3]])
SD = {}
def _gap():      # lens rim (roll + edge face + inner round) to the body solid: the joint gap (designed 0.4)
    gs = sdist('body', LV[rim]); ok = np.isfinite(gs); j = np.argmin(np.where(ok, gs, np.inf))
    rep['lens_gap_to_body_mm'] = dict(min=round(float(gs[ok].min()) * 1000, 3), at_mm=(LV[rim][j] * 1000).round(1).tolist(), p1=round(float(np.percentile(gs[ok], 1)) * 1000, 3),
                                      median=round(float(np.median(gs[ok])) * 1000, 3)); print('lens gap', rep['lens_gap_to_body_mm'], flush=True)
def sdist(tg, Pq):
    if tg not in SD: SD[tg] = SignedDist(TG[tg][0], TG[tg][1])
    return SD[tg](Pq, 0.03)
# designed contacts: (kit kind, target) -> (lower limit mm, why)
DESIGNED = {('boss', 'body'): (None, 'moulded on the wall (fused)'), ('gasket', 'body'): (-0.3, 'foam (1.3 free, 0.8 installed) on the inner skin: local extra squeeze <= 0.3'),
            ('housing', 'lens'): (-0.4, 'laser weld land (collapse 0.05 nominal, <= 0.4 local)'), ('foam', 'lens'): (-0.1, 'anti-flare / crosstalk foam on the lens'), ('gasket', 'lens'): (0.05, 'gasket beside the lens edge'),
            ('ear', 'body'): (0.3, 'ear behind the wall'), ('screw', 'body'): (None, 'screw in the boss (tip 1.5 above the inner skin: checked separately)'),
            ('housing', 'body'): (0.25, 'frame 0.8 behind the inner skin (gasket), light-trap step 0.3 beside the body edge'),
            ('ear', 'lens'): (0.5, ''), ('grommet', 'deck'): (None, 'grommet in the deck hole'), ('cable', 'deck'): (None, 'cable through the grommet'),
            ('clip', 'deck'): (None, 'moulded on the deck'), ('cradle', 'deck'): (None, 'moulded on the deck'), ('svc', 'deck'): (0.0, 'connector sits in the cradle'),
            ('chassis_lead', 'deck'): (0.3, 'lead lies in the open J-hooks'), ('emitter', 'controls'): (None, 'emitter board on the panel (supplier part)'),
            ('emitter', 'ctl_standin'): (None, 'on the panel'), ('chassis_lead', 'chassis'): (None, 'lead enters the chassis grommet at its end'),
            ('cable', 'body'): (0.5, 'cable along the wall in P-clips'), ('clip', 'body'): (None, 'clip on the wall'), ('plug', 'body'): (0.5, ''),
            ('housing', 'sleeves'): (0.45, '[fix3-int] port frame (fix-ports) = part of the body moulding, under the trimmed bottom of the front housing: 0.5 nominal (port_clear) like the housing to the body; module location +-0.09 RSS'),
            ('pad', 'body'): (-0.25, '[fix4-front2] crush pad (flush datum) bears on the inner skin; local skin deviation of the band map <= 0.25'), ('pad', 'lens'): (0.3, ''),      # [fix4-front2]
            ('bar', 'lens'): (0.15, 'air gap 0.25'), ('carrier', 'lens'): (0.05, 'carrier lip at the lens'), ('pipe', 'lens'): (0.05, 'pipe at the lens'), ('tof', 'lens'): (0.2, 'ToF 0.3 air'),
            ('camera', 'lens'): (0.1, 'barrel 0.25 air'), ('led', 'lens'): (0.5, ''), ('pcb', 'lens'): (0.5, '')}
_gap(); LIMIT = 1.0
res = collections.OrderedDict(); worst = []
for pre in ('lights_kit', 'lights_harn'):
    Vv, Ff, kk = d[pre + '_v'].astype(float), d[pre + '_f'].astype(np.int64), d[pre + '_kind']
    for k in np.unique(kk):
        kn = KINDS[int(k)]; Pk = samples(Vv, Ff[kk == k]); row = {}
        for tg in TG:
            if tg == 'lens' and pre == 'lights_harn': continue
            if tg in ('controls', 'ctl_standin', 'lid_kit', 'lid_well') and kn != 'emitter':
                c0 = np.asarray(TG[tg][0]).mean(0)
                if np.abs(Pk[:, 0] - c0[0]).min() > 0.25: continue
            s_ = sdist(tg, Pk); s_ = s_[np.isfinite(s_)]
            if not len(s_): continue
            mn = float(s_.min()) * 1000; lim = DESIGNED.get((kn, tg), (LIMIT, ''))[0]
            row[tg] = round(mn, 2)
            if lim is not None and mn < lim: worst.append(dict(kind=kn, target=tg, min_mm=round(mn, 2), limit_mm=lim, at_mm=(Pk[np.isfinite(sdist(tg, Pk))][np.argmin(s_)] * 1000).round(1).tolist()))
        res[kn] = row
        print('%-13s' % kn, row, flush=True)
# chassis points inside any kit solid (the chassis scan is not a closed solid: test its dense samples against the closed kit parts)
KS = SignedDist(np.r_[d['lights_kit_v'], d['lights_harn_v'] if len(d['lights_harn_v']) else np.zeros((0, 3))].astype(float),
                np.r_[d['lights_kit_f'], d['lights_harn_f'] + len(d['lights_kit_v'])].astype(np.int64), trust=0.006)
Pc = samples(X, Fc[np.isfinite(cKDTree(np.r_[d['lights_kit_v'], d['lights_harn_v']]).query(X[Fc].mean(1), distance_upper_bound=0.03)[0])], dens=0.1e-6)
sc = KS(Pc, 0.02); sc = sc[np.isfinite(sc)]
rep['chassis_samples_vs_kit'] = dict(samples=int(len(Pc)), inside=int((sc < 0).sum()), min_gap_mm=round(float(sc.min()) * 1000, 2) if len(sc) else None)
# boss / screw sanity: no boss / screw point outside the body's outer skin; screw tips above the inner skin
SO = SurfDist(V, F[(part == 0) & (side == 0) & (lab != 4)])
kk = d['lights_kit_kind']; Vk = d['lights_kit_v'].astype(float); Fk = d['lights_kit_f'].astype(np.int64)
for kn in ('boss', 'screw'):
    Pb = Vk[np.unique(Fk[kk == KINDS.index(kn)])]; so, _ = SO(Pb, 0.01); so = so[np.isfinite(so)]
    rep[kn + '_behind_outer_skin_mm'] = round(float(-so.max()) * 1000, 2)
rep['clearance_mm'] = res; rep['violations'] = worst
print('chassis vs kit', rep['chassis_samples_vs_kit'], 'boss/screw behind outer skin', rep['boss_behind_outer_skin_mm'], rep['screw_behind_outer_skin_mm'], flush=True)
print('violations', json.dumps(worst, indent=0), flush=True)
# ---- 4. sight lines through the gap and the windows
def tri_hit(O, Dr, T, tmax):
    e1 = T[:, 1] - T[:, 0]; e2 = T[:, 2] - T[:, 0]; p = np.cross(Dr, e2); det = (e1 * p).sum(1); ok = np.abs(det) > 1e-18; inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    s = O - T[:, 0]; u = (s * p).sum(1) * inv; q = np.cross(s, e1); v = (q @ Dr) * inv; t = (e2 * q).sum(1) * inv
    h = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-7) & (t < tmax); return np.where(h, t, np.inf)
G = [('body', V, F[(part == 0) & (lab != 4)], None), ('lens', LV, LF, LL), ('kit', d['lights_kit_v'], d['lights_kit_f'], d['lights_kit_kind']),
     ('cover', V, F[(part == 1) & (lab != 4)], None), ('chassis', X, Fc, None)] + ([('ports', d['sleeve_v'], d['sleeve_f'], None)] if 'sleeve_v' in d.files and len(d['sleeve_f']) else [])      # [fix3-int] the port frame is opaque body plastic
GT = [(nm, np.asarray(Vv, float)[np.asarray(Ff, np.int64)], lb) for nm, Vv, Ff, lb in G]; GK = [cKDTree(T.mean(1)) for _, T, _ in GT]
def first_hit(O, Dr, tmax=0.03, skip_lens=False):
    best = (np.inf, None, None)
    for (nm, T, lb), kd in zip(GT, GK):
        if skip_lens and nm == 'lens': continue
        ids = kd.query_ball_point(O + Dr * tmax / 2, tmax / 2 + 0.006)
        if not ids: continue
        ids = np.asarray(ids); t = tri_hit(O, Dr, T[ids], tmax); j = np.argmin(t)
        if t[j] < best[0]: best = (t[j], nm, (int(lb[ids[j]]) if lb is not None else None))
    return best
rng = np.random.default_rng(7); sight = {}
for li, nm in ((0, 'front'), (1, 'rear')):
    sgn = 1 if li == 0 else -1; rv = np.unique(LF[(LI == li) & (LS == 2)]); Pr = LV[rv]
    pick = rng.choice(len(Pr), 400, replace=False); hits = collections.Counter(); leak = []
    for p in Pr[pick]:
        n_out = np.array([sgn * 0.976, 0.0, 0.215]); base = p + n_out * 0.0015
        for k in range(6):
            th = np.radians(rng.uniform(0, 65)); ph = rng.uniform(0, 2 * np.pi); e1 = np.cross(n_out, [0, 0, 1.0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n_out, e1)
            Dr = -n_out * np.cos(th) + (e1 * np.cos(ph) + e2 * np.sin(ph)) * np.sin(th); t, what, lb = first_hit(base, Dr)
            key = what if what != 'kit' else 'kit:' + KINDS[lb]
            if what == 'lens' and lb in (3, 6): key = 'lens window'
            hits[key] += 1
            if what in ('kit',) and KINDS[lb] not in ('housing', 'gasket') or what in ('chassis', None): leak.append(dict(hit=key, at_mm=(base * 1000).round(1).tolist()))
    # through the windows: rays straight in through lens window faces (lab 3 light, 6 glass) -> what lies behind the lens
    wf = np.nonzero((LI == li) & (LS == 0) & np.isin(LL, [3, 6]))[0]; behind = collections.Counter()
    for fi in rng.choice(wf, min(300, len(wf)), replace=False):
        T = LV[LF[fi]]; c = T.mean(0); n = np.cross(T[1] - T[0], T[2] - T[0]); n /= np.linalg.norm(n)
        if n[0] * sgn < 0: n = -n
        t, what, lb = first_hit(c - n * 0.0027, -n, 0.03, skip_lens=True); behind[(what if what != 'kit' else 'kit:' + KINDS[lb]) + ('/light' if LL[fi] == 3 else '/glass')] += 1
    sight[nm] = dict(gap_rays=dict(hits), gap_leaks=len(leak), leak_examples=leak[:5], behind_windows=dict(behind))
    print('sight', nm, sight[nm], flush=True)
rep['sight'] = sight
# ---- 5. halo feed over the lid stroke
if len(d['lights_halo_f']):
    Q = d['lid_pivot']; Hv = d['lights_halo_v'].astype(float); Ph = samples(Hv, d['lights_halo_f'].astype(np.int64))
    emi = d['lights_kit_kind'] == KINDS.index('emitter'); TG['emitter'] = (Vk, Fk[emi])
    rows = {}
    for deg in (-0.438, 0.0, 2.0, 5.0, 10.0, 20.0, 40.0, 75.0):
        q = np.radians(-deg); c, s = np.cos(q), np.sin(q); P2 = Ph.copy(); x = P2[:, 0] - Q[0]; z = P2[:, 2] - Q[1]
        P2[:, 0] = Q[0] + c * x + s * z; P2[:, 2] = Q[1] - s * x + c * z
        rows[str(deg)] = {tg: round(float(np.nanmin(np.where(np.isfinite(sdist(tg, P2)), sdist(tg, P2), np.nan))) * 1000, 2) for tg in ('controls', 'ctl_standin', 'emitter', 'lid_well')}
    rep['halo_sweep_mm'] = rows; print('halo sweep', rows, flush=True)
    sl = SignedDist(V, F[lab == 4], trust=0.0)(Ph, 0.03); rep['halo_feed_top_under_lid_A_surface_mm'] = round(float(np.nanmin(np.where(np.isfinite(sl), sl, np.nan))) * 1000, 2)      # lid skin is open: unsigned
    lu = d['kitm_part'] == lidmech.PM['underside']; su = SignedDist(d['kitm_v'], d['kitm_f'][lu], trust=0.0)(Ph, 0.03); rep['halo_feed_to_lid_underside_mm'] = round(float(np.nanmin(np.where(np.isfinite(su), su, np.nan))) * 1000, 2)
rep['ok'] = bool(all(v['boundary_edges'] == 0 and v['nonmanifold_edges'] == 0 for v in rep['lens_topology'].values()) and rep['kit_topology']['open_components'] == 0
                 and rep['harness_topology']['open_components'] == 0 and not worst and rep['chassis_samples_vs_kit']['inside'] == 0
                 and rep['boss_behind_outer_skin_mm'] > 0.3 and all(s['gap_leaks'] == 0 for s in sight.values()))
print('LIGHTS OK:', rep['ok'])
if OUT: json.dump(rep, open(OUT, 'w'), indent=1, default=float)
