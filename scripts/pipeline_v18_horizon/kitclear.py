# [fix4-cover2] G1 standing check: the cover's hidden kit (snap legs, barbs, datum pins, seat pads, stakes) against the REAL chassis with the
# fix-struts cuts (npz caps, as the renderer / interfere.py), over the whole straight lift of the cover (0 = seated .. --lift mm), with the
# snap legs both at rest and deflected inward by the hook depth (the leg flexes while its hook passes the deck window: linear from 0 at the
# flex root, z_ret + L_flex, to `hook` at the retention face and below; an envelope - it is applied at every lift).
# Distances are unsigned surface-to-surface (kit area samples -> chassis faces, chassis area samples + cap samples -> kit faces) plus an
# inside test of the chassis samples against the closed kit parts (ray parity). PASS: min gap >= --clear (0.5 mm) at every lift.
# usage: python kitclear.py NPZ PARAMS.json [--json OUT] [--lift 30] [--step 0.25] [--clear 0.5]
import numpy as np, sys, json, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.spatial import cKDTree
from shell_wall import SurfDist
from labyrinthkit import RobustSurfDist
import strutskit as SK
a = sys.argv[1:]
def opt(k, dv): return float(a[a.index(k) + 1]) if k in a else dv
d = np.load(a[0]); P = json.load(open(a[1])); OUT = a[a.index('--json') + 1] if '--json' in a else None
LIFT, STEP, CLR = opt('--lift', 30.0) / 1000, opt('--step', 0.25) / 1000, opt('--clear', 0.5) / 1000
LG = P['cover']['legs']; HK = LG.get('hook', 0.0012); LFX = LG.get('L_flex', 0.016)
B = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dev', 'base_parts.npz')); X = B['Chassis_v'].astype(float); Fx = B['Chassis_f'].astype(np.int64)
KV = d['ckit_v'].astype(float); KF = d['ckit_f'].astype(np.int64); KK = d['ckit_kind']; KN = [str(k) for k in d['ckit_kinds']]
use = np.isin(KK, [KN.index(k) for k in ('leg', 'barb', 'pin', 'seatpad', 'stake') if k in KN]); KF = KF[use]; KK = KK[use]
# ---- the cut chassis near the kit: faces with every vertex inside a cut box removed (conservative), + the npz caps
sp = SK.from_npz(d); lo = KV[np.unique(KF)].min(0) - 0.006; hi = KV[np.unique(KF)].max(0) + np.array([0.006, 0.006, 0.006])
lo[2] -= 0.002; hi[2] += LIFT
Fx = Fx[((X[Fx] >= lo) & (X[Fx] <= hi)).all(2).any(1)]
if sp and sp.get('mode') == 'cut': Fx = Fx[~SK.inside(X[Fx].reshape(-1, 3), sp).reshape(-1, 3).all(1)]
CHV, CHF = X, Fx
if 'struts_cap_v' in d.files and len(d['struts_cap_f']):
    cv_ = d['struts_cap_v'].astype(float); CHV = np.r_[X, cv_]; CHF = np.r_[Fx, d['struts_cap_f'].astype(np.int64) + len(X)]
def area_samples(V, F, h):
    T = V[F]; ar = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1); n = np.clip((ar / (h * h)).astype(int), 1, 20000)
    rng = np.random.default_rng(3); fi = np.repeat(np.arange(len(F)), n); u = rng.random((len(fi), 2)); f = u.sum(1) > 1; u[f] = 1 - u[f]
    return np.r_[V[np.unique(F)], T[fi, 0] + (T[fi, 1] - T[fi, 0]) * u[:, :1] + (T[fi, 2] - T[fi, 0]) * u[:, 1:]]
Sch = area_samples(CHV, CHF, 0.0004)
# ---- leg sites: leg + barb components clustered in plan; outward = barb centroid - leg centroid (the hook points out)
from coverkit4 import _components, _weld
w = _weld(KV); comp = _components(int(w.max()) + 1, w[KF]); kind_c = {}; cen = {}
for c in np.unique(comp):
    fm = comp == c; kind_c[c] = KN[int(np.bincount(KK[fm]).argmax())]; cen[c] = KV[KF[fm]].reshape(-1, 3).mean(0)
barbs = [c for c in cen if kind_c[c] == 'barb']; legs = [c for c in cen if kind_c[c] == 'leg']
DEF = np.zeros_like(KV); sites = []
for b in barbs:
    lg = [c for c in legs if np.linalg.norm(cen[c][:2] - cen[b][:2]) < 0.012]
    if not lg: continue
    vb = np.unique(KF[comp == b]); zr = float(KV[vb, 2].max())                                  # retention face top = deck underside
    vl = np.unique(KF[np.isin(comp, lg)]); low = vl[KV[vl, 2] < zr + LFX]
    out = cen[b][:2] - KV[low, :2].mean(0); out /= max(np.linalg.norm(out), 1e-9); inw = np.r_[-out, 0.0]
    for vs in (vb, vl):
        r = np.clip((zr + LFX - KV[vs, 2]) / LFX, 0.0, 1.0); DEF[vs] = (HK * r)[:, None] * inw[None, :]
    sites.append(dict(at_mm=np.round(cen[b] * 1000, 1).tolist(), inward=np.round(inw[:2], 3).tolist(), z_ret_mm=round(zr * 1000, 2)))
SDch = SurfDist(CHV, CHF, 12)
res = {}; worst = None
for st, KVs in (('rest', KV), ('deflected', KV + DEF)):
    Sk = area_samples(KVs, KF, 0.0003); kc = np.unique(comp); SDk = RobustSurfDist(KVs, KF, sign_within=0.003)
    tk = cKDTree(Sch); rows = []
    for dz in np.arange(0.0, LIFT + 1e-9, STEP):
        q = Sk + [0, 0, dz]; dn, _ = tk.query(q, distance_upper_bound=0.003, workers=-1); m = np.isfinite(dn); g1 = np.inf; at = None
        if m.any():
            s_, _ = SDch(q[m], 0.006); s_ = np.abs(s_); i = int(np.argmin(s_)); g1 = float(s_[i]); at = q[m][i]
        c_ = Sch - [0, 0, dz]; dn2, _ = cKDTree(Sk).query(c_, distance_upper_bound=0.003, workers=-1); m2 = np.isfinite(dn2); g2 = np.inf; at2 = None; nin = 0
        if m2.any():
            s2, _ = SDk(c_[m2], 0.006); nin = int((s2 < 0).sum()); i2 = int(np.argmin(s2)); g2 = float(s2[i2]); at2 = c_[m2][i2] + [0, 0, dz]
        g = min(g1, g2); rows.append(dict(lift_mm=round(dz * 1000, 2), min_gap_mm=round(g * 1000, 3) if np.isfinite(g) else None, chassis_in_kit=nin,
                                          at_mm=(np.round((at if g1 <= g2 else at2) * 1000, 1).tolist() if np.isfinite(g) else None)))
        if np.isfinite(g) and (worst is None or g < worst[0]): worst = (g, st, dz, rows[-1]['at_mm'])
    fin = [r for r in rows if r['min_gap_mm'] is not None]
    res[st] = dict(min_gap_mm=min((r['min_gap_mm'] for r in fin), default=None), inside_total=int(sum(r['chassis_in_kit'] for r in rows)),
                   tightest=sorted(fin, key=lambda r: r['min_gap_mm'])[:6], lifts=len(rows))
    print(st, json.dumps({k: v for k, v in res[st].items() if k != 'tightest'}), 'tightest:', [(r['lift_mm'], r['min_gap_mm'], r['at_mm']) for r in res[st]['tightest'][:4]], flush=True)
ok = all((r['min_gap_mm'] is None or r['min_gap_mm'] >= CLR * 1000) and r['inside_total'] == 0 for r in res.values())
out = dict(clear_mm=CLR * 1000, lift_mm=LIFT * 1000, step_mm=STEP * 1000, hook_mm=HK * 1000, L_flex_mm=LFX * 1000, leg_sites=sites, chassis_faces=int(len(CHF)),
           chassis_samples=int(len(Sch)), cases=res, worst=dict(gap_mm=round(worst[0] * 1000, 3), state=worst[1], lift_mm=round(worst[2] * 1000, 2), at_mm=worst[3]) if worst else None, ok=bool(ok))
print('leg sites:', len(sites), ' worst:', out['worst'], ' KITCLEAR', 'OK' if ok else 'FAIL', '(>= %.2f mm over 0..%.0f mm lift)' % (CLR * 1000, LIFT * 1000))
if OUT: json.dump(out, open(OUT, 'w'), indent=1)
