# [fix3-ports] Charger port checks: docking stroke of the dock pins through the slot (body + port frame), chassis seat, mirror
# symmetry and closed topology of the port frame, deck-drain drop paths, window vs chassis funnel mouths.
# usage: python portcheck.py SHELL.npz PARAMS.json [--json OUT] [--seat-dy M]   (PARAMS: the json the npz was made from; its ports3 block
#        gives the pin model; without ports3 (f2) the defaults of portskit.py are used)
import numpy as np, sys, json
sys.path.insert(0, '.'); import portskit
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
d = np.load(sys.argv[1]); P = json.load(open(sys.argv[2])); Q = portskit.params(P.get('ports3')); out = {}
if '--seat-dy' in sys.argv: Q['seat_dy'] = float(sys.argv[sys.argv.index('--seat-dy') + 1])
V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; lab = d['lab']
BOX = (np.array([0.27, -0.075, 0.12]), np.array([0.345, 0.075, 0.18]))
def sample(Vs, Fs, step=0.00015, box=BOX, seed=0):
    c = Vs[Fs].mean(1); m = np.all((c > box[0]) & (c < box[1]), axis=1); P3 = Vs[Fs[m]]
    ar = 0.5 * np.linalg.norm(np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]), axis=1); ns = np.clip((ar / step ** 2).astype(int) + 1, 1, 2000)
    fi = np.repeat(np.arange(len(P3)), ns); u = np.random.default_rng(seed).random((len(fi), 2)); f_ = u.sum(1) > 1; u[f_] = 1 - u[f_]
    return np.r_[P3.reshape(-1, 3), P3[fi, 0] + (P3[fi, 1] - P3[fi, 0]) * u[:, :1] + (P3[fi, 2] - P3[fi, 0]) * u[:, 1:]]
Xb = sample(V, F[(part == 0) & (lab != 4)]); Xs = sample(d['sleeve_v'], d['sleeve_f']) if 'sleeve_v' in d.files else np.zeros((0, 3))
B = np.load('dev/base_parts.npz'); Xc = sample(B['Chassis_v'], B['Chassis_f'], 0.0002)
print('samples: body %d, port frame %d, chassis %d' % (len(Xb), len(Xs), len(Xc)))

# ---- 1. port frame: closed solids, orientation, mirror symmetry
if len(Xs):
    sv, sf = d['sleeve_v'], d['sleeve_f'].astype(np.int64); n = len(sv); e = np.r_[sf[:, [0, 1]], sf[:, [1, 2]], sf[:, [2, 0]]]
    nc, cc = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)))
    comps = []
    for c_ in range(nc):
        fm = cc[sf[:, 0]] == c_
        if not fm.any(): continue
        Fc = sf[fm]; ee = np.r_[Fc[:, [0, 1]], Fc[:, [1, 2]], Fc[:, [2, 0]]]; key = np.minimum(ee[:, 0], ee[:, 1]) * n + np.maximum(ee[:, 0], ee[:, 1]); _, cnt = np.unique(key, return_counts=True)
        dk = ee[:, 0] * n + ee[:, 1]; rk = ee[:, 1] * n + ee[:, 0]; Pc = sv[Fc]; vol = np.einsum('ij,ij->i', Pc[:, 0], np.cross(Pc[:, 1], Pc[:, 2])).sum() / 6
        comps.append(dict(faces=int(fm.sum()), boundary=int((cnt == 1).sum()), nonmanifold=int((cnt > 2).sum()), unpaired=int((~np.isin(rk, dk)).sum()), vol_mm3=round(vol * 1e9, 1)))
    from shell_wall import SurfDist
    mir = sv * np.array([1, -1, 1]); dm = np.abs(SurfDist(sv, sf, k=40)(mir, 0.003)[0]); dm[~np.isfinite(dm)] = 0.003      # mirrored vertices -> original surface
    dm = np.minimum(dm, cKDTree(sv).query(mir, workers=-1)[0])                                                      # (or onto a mirrored vertex)
    tun = sv[:, 0] > (float(d['ports_xw'][0]) if 'ports_xw' in d.files else 0.3135)                # tunnel part (from the body's rim points)
    out['frame'] = dict(components=comps, all_closed=all(c_['boundary'] == 0 and c_['nonmanifold'] == 0 and c_['unpaired'] == 0 for c_ in comps),
                        all_positive=all(c_['vol_mm3'] > 0 for c_ in comps), mirror_dev_mm=dict(web_ducts_max=round(float(dm[~tun].max()) * 1e3, 4) if (~tun).any() else None,
                        tunnel_p99=round(float(np.percentile(dm[tun], 99)) * 1e3, 3) if tun.any() else None, tunnel_max=round(float(dm[tun].max()) * 1e3, 3) if tun.any() else None))
    print('frame', json.dumps(out['frame']))
if 'ports_wp' in d.files:
    from shapely.geometry import Polygon
    Wp = Polygon(d['ports_wp']); Wm = Polygon(d['ports_wm']); out['windows'] = dict(mirror_xor_mm2=round(Wp.symmetric_difference(portskit.mirror(Wm)).area * 1e6, 5),
        plus_y_mm=np.round(np.array(Wp.bounds) * 1e3, 2).tolist(), minus_y_mm=np.round(np.array(Wm.bounds) * 1e3, 2).tolist())
    print('windows', out['windows'])

# ---- 2. docking stroke: the pins (capsules on the fitted axes) travel -x from 70 mm before the seat to the seat, at every misalignment case
def axis_pts(p, dy, dz, s):
    """pin axis segment (tip-centre, base) at stroke s (pins s ahead of their seated x)"""
    xt = Q['tip_x'] + Q['pin_r'] + s; xb = 0.3500 + s; y0, z0 = portskit.pin_axis(p, xt - s, Q['seat_dy'] + dy); y1, z1 = portskit.pin_axis(p, xb - s, Q['seat_dy'] + dy)
    return np.array([xt, y0, z0 + dz]), np.array([xb, y1, z1 + dz])
def seg_dist(X, a, b):
    ab = b - a; t = np.clip(((X - a) @ ab) / (ab @ ab), 0, 1); return np.linalg.norm(X - a - t[:, None] * ab, axis=1)
def stroke(X, dy, dz, r, s_list):
    """min clearance (mm) over the stroke and where; X prefiltered to the neighbourhood of the pin paths"""
    best = (np.inf, None, None, None)
    for p in Q['pins']:
        a0, b0 = axis_pts(p, dy, dz, 0.0); m = (np.abs(X[:, 1] - 0.5 * (a0[1] + b0[1])) < r + 0.006) & (np.abs(X[:, 2] - 0.5 * (a0[2] + b0[2])) < r + 0.006)
        Y = X[m]
        if not len(Y): continue
        for s in s_list:
            a, b = axis_pts(p, dy, dz, s); c = seg_dist(Y, a, b) - r; i = int(np.argmin(c))
            if c[i] < best[0]: best = (float(c[i]), Y[i], s, p['y'] > 0)
    return best
S_ALL = np.arange(0.070, -1e-9, -0.00025)
cases = [('nominal (seated pose)', 0.0, 0.0), ('asm_dock pose', -Q['seat_dy'], 0.0)]
for dy_n, dy in (('capture min', Q['capture'][0]), ('capture max', Q['capture'][1]), ('lateral 0', 0.0)):
    for dz_n, dz in (('dz min', Q['dz'][0]), ('dz max', Q['dz'][1])): cases.append(('%s + %s' % (dy_n, dz_n), dy, dz))
cases += [('capture min', Q['capture'][0], 0.0), ('capture max', Q['capture'][1], 0.0)]
Xbs = np.r_[Xb, Xs]; rows = []
for nm, dy, dz in cases:
    for r in (Q['pin_r'], 0.00397):
        c, X_, s_, side = stroke(Xbs, dy, dz, r, S_ALL)
        rows.append(dict(case=nm, dy_mm=round(dy * 1e3, 2), dz_mm=round(dz * 1e3, 2), pin_r_mm=round(r * 1e3, 2), min_clear_mm=round(c * 1e3, 3),
                         at_mm=(X_ * 1e3).round(1).tolist() if X_ is not None else None, stroke_mm=round(s_ * 1e3, 2) if s_ is not None else None, pin='+y' if side else '-y'))
        print('stroke %-34s r %.2f: body/frame clearance %7.3f mm at %s (stroke %s mm, pin %s)' % (nm, r * 1e3, c * 1e3, rows[-1]['at_mm'], rows[-1]['stroke_mm'], rows[-1]['pin']))
out['stroke'] = rows
# ---- 2b. [fix4-front2] S9: heading error. Both pins rotated by yaw about a vertical axis through the seated tip mid-point (it moves with the lateral
#      offset), two motion models: 'heading' = the mower drives along its own x (rotated pins translate along x), 'crab' = the pins travel along their
#      own rotated axes (review safety S9, dockyaw.py). Design range +-Q['yaw_deg'] (windows built for it); +-3 deg reported for information.
def axis_pts_yaw(p, dy, dz, s, yaw, model):
    a, b = axis_pts(p, dy, dz, 0.0 if model == 'heading' else s); tips = [portskit.pin_axis(q, Q['tip_x'], Q['seat_dy'] + dy)[0] for q in Q['pins']]
    px, py = Q['tip_x'], 0.5 * (tips[0] + tips[1]); c, sn = np.cos(yaw), np.sin(yaw)
    R = lambda q: np.array([px + (q[0] - px) * c - (q[1] - py) * sn, py + (q[0] - px) * sn + (q[1] - py) * c, q[2]])
    a, b = R(a), R(b)
    if model == 'heading': a = a + np.array([s, 0, 0]); b = b + np.array([s, 0, 0])
    return a, b
def stroke_yaw(X, dy, dz, r, s_list, yaw, model):
    best = (np.inf, None, None, None)
    for p in Q['pins']:
        a0, b0 = axis_pts_yaw(p, dy, dz, 0.0, yaw, model); m = (np.abs(X[:, 1] - 0.5 * (a0[1] + b0[1])) < r + 0.008) & (np.abs(X[:, 2] - 0.5 * (a0[2] + b0[2])) < r + 0.006); Y = X[m]
        if not len(Y): continue
        for s in s_list:
            a, b = axis_pts_yaw(p, dy, dz, s, yaw, model); c = seg_dist(Y, a, b) - r; i = int(np.argmin(c))
            if c[i] < best[0]: best = (float(c[i]), Y[i], s, p['y'] > 0)
    return best
yrows = []; YD = float(Q.get('yaw_deg', 0.0)); S_Y = np.arange(0.070, -1e-9, -0.0005)
for yd in sorted(set([-3.0, -YD, YD, 3.0]) - {0.0}):
    for nm, dy, dz in [c_ for c_ in cases if c_[0] != 'asm_dock pose']:
        for model in ('heading', 'crab'):
            c, X_, s_, side = stroke_yaw(Xbs, dy, dz, Q['pin_r'], S_Y, np.radians(yd), model)
            yrows.append(dict(yaw_deg=yd, case=nm, model=model, min_clear_mm=round(c * 1e3, 3), at_mm=(X_ * 1e3).round(1).tolist() if X_ is not None else None,
                              stroke_mm=round(s_ * 1e3, 2) if s_ is not None else None, pin='+y' if side else '-y', design=abs(yd) <= YD + 1e-9))
    w_ = min((r_ for r_ in yrows if r_['yaw_deg'] == yd), key=lambda r_: r_['min_clear_mm'])
    print('yaw %+.1f deg: worst %.3f mm (%s, %s, pin %s at %s)' % (yd, w_['min_clear_mm'], w_['case'], w_['model'], w_['pin'], w_['at_mm']), flush=True)
out['stroke_yaw'] = yrows
dsg = [r_['min_clear_mm'] for r_ in rows if r_['pin_r_mm'] == round(Q['pin_r'] * 1e3, 2)] + [r_['min_clear_mm'] for r_ in yrows if r_['design']]
out['worst_design_mm'] = dict(value=min(dsg), yaw_range_deg=YD, target=0.5, ok=min(dsg) >= 0.5); print('worst over the design box (lateral x dz x yaw +-%.1f): %.3f mm' % (YD, min(dsg)))
# chassis: where the pins first touch it (the seat) at the nominal and the extreme lateral cases
ch = []
for nm, dy in (('seated', 0.0), ('asm_dock', -Q['seat_dy']), ('capture min', Q['capture'][0]), ('capture max', Q['capture'][1])):
    first = None
    for s in S_ALL:
        c, X_, _, side = stroke(Xc, dy, 0.0, 0.00397, [s])
        if c < 0.0: first = (s, c, X_, side); break
    ch.append(dict(case=nm, first_contact_stroke_mm=round(first[0] * 1e3, 2) if first else None, tip_x_mm=round((Q['tip_x'] + first[0]) * 1e3, 2) if first else None,
                   at_mm=(first[2] * 1e3).round(1).tolist() if first else None, pin=('+y' if first[3] else '-y') if first else None))
    print('chassis', ch[-1])
out['chassis_contact'] = ch

# ---- 3. deck drains at the nose: vertical drop path, first surface hit below the drain
def ray_down(o, Vs, Fs):
    P3 = Vs[Fs]; mn = P3.min(1); mx = P3.max(1); m = (mn[:, 0] <= o[0]) & (mx[:, 0] >= o[0]) & (mn[:, 1] <= o[1]) & (mx[:, 1] >= o[1]) & (mn[:, 2] < o[2])
    best = None
    for T in P3[m]:
        a, b, c = T; e1 = b - a; e2 = c - a; dvec = np.array([0, 0, -1.0]); h = np.cross(dvec, e2); det = e1 @ h
        if abs(det) < 1e-14: continue
        f_ = 1 / det; s_ = o - a; u = f_ * (s_ @ h)
        if u < 0 or u > 1: continue
        q = np.cross(s_, e1); v = f_ * (dvec @ q)
        if v < 0 or u + v > 1: continue
        t = f_ * (e2 @ q)
        if t > 1e-6 and (best is None or t < best): best = t
    return best
drains = []
try:
    rp = json.load(open(sys.argv[1].replace('.npz', '.check.json')))['mounts']['cover']['location']['deck_drains']
except Exception: rp = []
for q in [q for q in rp if q[0] > 250]:
    o = np.array(q) / 1e3 - np.array([0, 0, 0.003]); hits = {}
    for nm, Vs, Fs in (('body', V, F[(part == 0) & (lab != 4)]), ('port frame', d['sleeve_v'], d['sleeve_f']), ('chassis', B['Chassis_v'], B['Chassis_f'])):
        t = ray_down(o, Vs, Fs.astype(np.int64)); hits[nm] = round((o[2] - t) * 1e3, 1) if t is not None else None
    first = max((v_, k_) for k_, v_ in hits.items() if v_ is not None) if any(v_ is not None for v_ in hits.values()) else None
    drains.append(dict(drain_mm=q, hit_z_mm=hits, first=first[1] if first else None)); print('drain', drains[-1])
out['drains'] = drains
# ---- 4. windows vs the chassis funnel mouths (y extents at x 302, z 152.6) and the pins
if 'ports_wp' in d.files:
    out['mouths_mm'] = dict(plus=[8.7, 52.0], minus=[-45.4, -4.6]); print('mouths', out['mouths_mm'])
if '--json' in sys.argv: json.dump(out, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
