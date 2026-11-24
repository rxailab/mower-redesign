# [fix4-front2] M6: tool actions of the one-piece body moulding at the nose and the tail - every face of the port frame and of the lamp
# bosses (+ gussets) must be formed by ONE declared action of the body tool, and that action must be free to move:
#   core          -z  main core (the body is drawn vertically: cavity +z forms the A-surface, core -z the inside)
#   slider_front  +x  front slider on the cavity side: slot, tunnel bore, front of the web, windows, duct interiors (fingers through the windows)
#   islide_front  -x  internal slide in the core (travel --travel-front, before the core withdraws): outside of the tunnel / ducts / hood, duct
#                     ends, back of the web, the four front lamp bosses + gussets (horizontal axes since f4), inner skin round the band
#   islide_rear   +x  internal slide in the core at the tail: the four rear lamp bosses + gussets
# A face is formed by an action when its outward normal n has n.d >= -sin(0.5 deg) (|n.d| < sin 0.5 deg = parallel: needs 0.5..1 deg draft
# on the tool drawing) and a ray from it along d meets no other surface of the moulding within the action's travel (slides) or at all
# (core / slider). Faces lying inside another solid of the moulding (fused primitives, the tunnel buried in the wall, boss feet in the wall)
# are not part of the moulded surface and are skipped (ray parity). The moulding = body + deck + its body kit + nose floor + port frame +
# lamp bosses. Faces no declared action can form are TRAPPED (undercut / needs another action); the report lists them as clusters.
# usage (inside design/):  ../venv/bin/python toolcheck.py NPZ [--json OUT.json] [--travel-front 0.024] [--travel-rear 0.012]
#                          [--extra NAME:dx,dy,dz:travel:FEATURES]   (e.g. the f3 module-axis lifter: axis_front:-0.979,0,-0.205:0.012:front_bosses)
#   (re-executes itself in Blender; or: blender -b -P toolcheck.py -- NPZ ...)   exit 0 = no trapped face, 1 = trapped faces
import sys, os, json, time, math
HERE_ = os.path.dirname(os.path.abspath(sys.argv[sys.argv.index('-P') + 1] if '-P' in sys.argv else __file__)); sys.path.insert(0, HERE_)
import qa_parts as QP
QP.ensure_blender(os.path.join(HERE_, 'toolcheck.py'))
import numpy as np
from mathutils.bvhtree import BVHTree
from mathutils import Vector

a = QP.script_args(); NPZ = a[0]
def opt(k, dv=None): return a[a.index(k) + 1] if k in a else dv
OUT = opt('--json'); TF = float(opt('--travel-front', 0.024)); TR = float(opt('--travel-rear', 0.012)); PAR = math.sin(math.radians(0.5))
ACT = {'core': (np.array([0.0, 0.0, -1.0]), None), 'slider_front': (np.array([1.0, 0.0, 0.0]), None),
       'islide_front': (np.array([-1.0, 0.0, 0.0]), TF), 'islide_rear': (np.array([1.0, 0.0, 0.0]), TR)}
FEAT = [('port_frame', lambda p: p.group == 'portframe', ['slider_front', 'islide_front', 'core']),
        ('front_bosses', lambda p: p.group == 'lamp.boss' and p.c[0] > 0, ['islide_front', 'core']),
        ('rear_bosses', lambda p: p.group == 'lamp.boss' and p.c[0] < 0, ['islide_rear', 'core'])]
for e_ in [a[i + 1] for i, x in enumerate(a) if x == '--extra']:      # extra action for a comparison run: NAME:dx,dy,dz:travel:feature[+feature]
    nm_, dv_, tr_, fs_ = e_.split(':'); dv_ = np.array([float(x) for x in dv_.split(',')]); ACT[nm_] = (dv_ / np.linalg.norm(dv_), float(tr_))
    for f_ in FEAT:
        if f_[0] in fs_.split('+'): f_[2].append(nm_)
MOULD = ('body', 'portframe', 'nosefloor', 'lamp.boss', 'mount.deck', 'mount.boss', 'mount.rib', 'mount.bracket', 'mount.clip', 'mount.well',
         'mount.hem_lip', 'mount.other', 'mount.seat', 'mount.drain')
os.chdir(QP.HERE); t0 = time.time()
d, parts, info = QP.build(NPZ, None, None, hardware=False)
mould = [p for p in parts if p.group in MOULD]
print('moulding: %d components, %d faces (%.1f s)' % (len(mould), sum(len(p.F) for p in mould), time.time() - t0), flush=True)
for p in mould: p.bvh = BVHTree.FromPolygons(p.V.tolist(), p.F.tolist(), all_triangles=True, epsilon=0.0)
RAYS = [Vector(v).normalized() for v in ((0.31, 0.77, 0.56), (-0.62, 0.21, 0.76), (0.12, -0.85, -0.51))]


def inside(p, x):
    """ray parity of point x in the closed component p (2 of 3 rays)"""
    if np.any(x < p.lo - 1e-4) or np.any(x > p.hi + 1e-4): return False
    odd = 0
    for r in RAYS:
        o = Vector(x); n = 0
        for _ in range(64):
            h = p.bvh.ray_cast(o, r, 10.0)
            if h[0] is None: break
            n += 1; o = h[0] + r * 1e-7
        odd += n % 2
    return odd >= 2


def tri(p):
    T = p.V[p.F]; n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]); ar = 0.5 * np.linalg.norm(n, axis=1)
    return T.mean(1), n / np.maximum(2 * ar, 1e-30)[:, None], ar


def altitude(p):      # smallest height of each triangle (2 area / longest edge): < SLIVER = a zero-width sliver whose normal means nothing
    T = p.V[p.F]; L = np.max(np.linalg.norm(T - np.roll(T, 1, 1), axis=2), 1); ar = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
    return 2 * ar / np.maximum(L, 1e-30)
SLIVER = 5e-6      # 5 um: slivers where the un-booleaned solids of one moulding meet / coincident rim points are reported apart, not as trapped


# ---- moulded surface: drop the faces that lie inside another component of the moulding (fused primitives / buried parts)
feat_parts = {f_[0]: [p for p in mould if f_[1](p)] for f_ in FEAT}
FP = [p for v in feat_parts.values() for p in v]
EXT = {}
for p in mould:
    c, n, ar = tri(p); keep = np.ones(len(c), bool)
    if p in FP: tests = [(np.arange(len(c)), [q for q in mould if q is not p])]      # feature faces: against every other solid of the moulding
    else:      # other parts: only their faces close to a feature part, against that part (boss feet / buried tunnel cover the body skin)
        tests = [(np.nonzero(np.all((c > q.lo - 0.001) & (c < q.hi + 0.001), 1))[0], [q]) for q in FP]
    for idx, others in tests:
        for i in idx:
            if not keep[i]: continue
            x = c[i] + n[i] * 2e-6      # just outside the face: the face is moulded surface unless that point lies inside another solid
            for q in others:
                if inside(q, x): keep[i] = False; break
    EXT[id(p)] = keep
V_ = []; F_ = []; o_ = 0
for p in mould:
    k = EXT[id(p)]; V_.append(p.V); F_.append(p.F[k] + o_); o_ += len(p.V)
BV = BVHTree.FromPolygons(np.concatenate(V_).tolist(), np.concatenate(F_).tolist(), all_triangles=True, epsilon=0.0)
print('moulded surface BVH: %d faces (%.1f s)' % (sum(len(f) for f in F_), time.time() - t0), flush=True)


def free(x, dvec, L):
    h = BV.ray_cast(Vector(x), Vector(dvec), L if L else 2.0)
    return h[0] is None, (None if h[0] is None else h[3])


rep = dict(info=dict(npz=os.path.abspath(NPZ), travel_front_mm=TF * 1000, travel_rear_mm=TR * 1000, parallel_deg=0.5),
           actions={k: dict(dir=v[0].round(4).tolist(), travel_mm=None if v[1] is None else v[1] * 1000) for k, v in ACT.items()}, features={})
LAB = []; ntrap = 0; PX = {}      # PX: x of the faces each action forms, per component (travel each slide needs)
for nm, _, acts in FEAT:
    R = dict(faces=0, area_mm2=0.0, internal_faces=0, by_action={k: 0.0 for k in acts}, parallel_mm2={k: 0.0 for k in acts}, trapped_mm2=0.0, trapped=[], trapped_slivers=0, trapped_slivers_mm2=0.0)
    for p in feat_parts[nm]:
        c, n, ar = tri(p); keep = EXT[id(p)]; R['internal_faces'] += int((~keep).sum()); alt = altitude(p)
        for i in np.nonzero(keep)[0]:
            if ar[i] < 1e-14: continue
            R['faces'] += 1; R['area_mm2'] += ar[i] * 1e6; best = None
            for k in acts:
                dv, L = ACT[k]; nd = float(n[i] @ dv)
                if nd < -PAR: continue
                ok, _ = free(c[i] + n[i] * 2e-5 + dv * 1e-5, dv, L)
                if ok and (best is None or nd > best[1]): best = (k, nd)
            if best is None and alt[i] < SLIVER:
                R['trapped_slivers'] += 1; R['trapped_slivers_mm2'] += ar[i] * 1e6; LAB.append((c[i], n[i], -2)); continue
            if best is None:
                R['trapped_mm2'] += ar[i] * 1e6; R['trapped'].append((c[i] * 1000).round(2).tolist()); LAB.append((c[i], n[i], -1)); continue
            R['by_action'][best[0]] += ar[i] * 1e6
            if best[1] < PAR: R['parallel_mm2'][best[0]] += ar[i] * 1e6
            LAB.append((c[i], n[i], list(ACT).index(best[0]))); PX.setdefault((best[0], id(p)), []).append(c[i][0])
    # trapped faces -> clusters (5 mm grid)
    if R['trapped']:
        T = np.array(R['trapped']); key = np.round(T / 5).astype(int); u, inv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True)
        R['trapped_clusters'] = sorted([dict(n=int(cnt[j]), at_mm=T[inv.ravel() == j].mean(0).round(1).tolist()) for j in range(len(u))], key=lambda r: -r['n'])[:12]
    ntrap += len(R['trapped']); R['trapped'] = len(R['trapped'])
    for k in ('area_mm2', 'trapped_mm2'): R[k] = round(R[k], 1)
    R['trapped_slivers_mm2'] = round(R['trapped_slivers_mm2'], 4)
    R['by_action'] = {k: round(v, 1) for k, v in R['by_action'].items()}; R['parallel_mm2'] = {k: round(v, 1) for k, v in R['parallel_mm2'].items()}
    rep['features'][nm] = R
    print('%-13s faces %6d (internal %d), area %8.1f mm2: %s  parallel (draft on the drawing) %s  TRAPPED %d faces %.1f mm2 %s  (+ %d sliver faces < %.0f um high, %.4f mm2)' % (
        nm, R['faces'], R['internal_faces'], R['area_mm2'], R['by_action'], R['parallel_mm2'], R['trapped'], R['trapped_mm2'], R.get('trapped_clusters', '')[:3] if R['trapped'] else '',
        R['trapped_slivers'], SLIVER * 1e6, R['trapped_slivers_mm2']), flush=True)
# slide extents and travel actually needed (x extent of what each internal slide forms)
for k in ('islide_front', 'islide_rear'):
    m = [x for x in LAB if x[2] == list(ACT).index(k)]
    if m:
        X = np.array([x[0] for x in m]); ex_ = max(np.ptp(v) for (k_, _), v in PX.items() if k_ == k)
        rep['actions'][k].update(formed_box_mm=[(X.min(0) * 1000).round(1).tolist(), (X.max(0) * 1000).round(1).tolist()], component_x_extent_max_mm=round(float(ex_) * 1000, 1))
        print(k, 'forms a region', rep['actions'][k]['formed_box_mm'], 'longest formed component along x %.1f mm (travel %.0f mm)' % (rep['actions'][k]['component_x_extent_max_mm'], rep['actions'][k]['travel_mm']), flush=True)
rep['trapped_faces'] = ntrap; rep['ok'] = ntrap == 0; rep['time_s'] = round(time.time() - t0, 1)
if LAB:
    np.savez(os.path.splitext(OUT)[0] + '_faces.npz' if OUT else 'toolcheck_faces.npz', c=np.array([x[0] for x in LAB]), n=np.array([x[1] for x in LAB]),
             act=np.array([x[2] for x in LAB]), names=np.array(list(ACT)))
if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
print('TOOLCHECK %s: %d trapped faces (%.0f s)' % ('OK' if rep['ok'] else 'FAIL', ntrap, time.time() - t0), flush=True)
sys.exit(0 if rep['ok'] else 1)
