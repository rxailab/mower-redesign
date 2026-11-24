# [fix2] R2E5 / F-S5: the deck as a solid. usage: python deckcheck.py DESIGN.npz dev/base_parts.npz
#  - deck topology (boundary / non-manifold edges: 0 = closed solid), thickness probe
#  - cover kit (legs, barbs, pins, seat pads) through the deck: vertices + face centroids inside the deck solid (must be 0 except pad feet resting on it)
#  - chassis (struts cut as in the renderer) inside the deck solid; nose floor: closed, distance to the bumper / body / chassis
import numpy as np, sys, json
sys.path.insert(0, '.'); from shell_wall import SurfDist
d = np.load(sys.argv[1]); B = np.load(sys.argv[2]); out = {}
mv = d['mount_v'].astype(float); mf = d['mount_f'].astype(np.int64); mk = d['mount_kind']; MKN = [str(x) for x in d['mount_kinds']]
Fd = mf[mk == MKN.index('deck')]
e = np.concatenate([Fd[:, [0, 1]], Fd[:, [1, 2]], Fd[:, [2, 0]]]); key = np.minimum(e[:, 0], e[:, 1]) * len(mv) + np.maximum(e[:, 0], e[:, 1]); _, c = np.unique(key, return_counts=True)
out['deck_topology'] = dict(faces=int(len(Fd)), boundary_edges=int((c == 1).sum()), nonmanifold_edges=int((c > 2).sum()))
SD = SurfDist(mv, Fd)
from labyrinthkit import RobustSurfDist as _RSD; SD = _RSD(mv, Fd, sign_within=0.004)      # [fix4-cover2] sign by ray parity (the nearest face's plane gave false 'inside' at the nose-pin slot ends: 7 samples, 1.83 mm)
def samples(V_, F_):
    return np.r_[V_[np.unique(F_)], V_[F_].mean(1)]
cv = d['ckit_v'].astype(float); cf = d['ckit_f'].astype(np.int64); ck = d['ckit_kind']; CKN = [str(x) for x in d['ckit_kinds']]
for nm in ('leg', 'barb', 'pin', 'seatpad'):
    if nm not in CKN: continue
    m = ck == CKN.index(nm)
    if not m.any(): continue
    X = samples(cv, cf[m]); s, _ = SD(X, 0.01); fin = np.isfinite(s)
    r = dict(samples=int(len(X)), inside_deck=int((fin & (s < -1e-5)).sum()), min_gap_mm=round(float(s[fin & (s >= -1e-5)].min()) * 1000, 3) if (fin & (s >= -1e-5)).any() else None)
    if r['inside_deck']: r['worst_at_mm'] = (X[fin & (s < -1e-5)][np.argsort(s[fin & (s < -1e-5)])[:3]] * 1000).round(1).tolist(); r['depth_mm'] = round(float(-s[fin].min()) * 1000, 3)
    out['ckit_' + nm] = r
PO = json.load(open('mounts.json'))['posts']; X = B['Chassis_v'].astype(float)
kill = (np.abs(X[:, 1]) > 0.160) & (X[:, 2] > 0.135) & (X[:, 0] > -0.06) & (X[:, 0] < 0.29)
for p_ in PO: kill &= (X[:, 0] - p_[0]) ** 2 + (X[:, 1] - p_[1]) ** 2 > 0.0068 ** 2
import strutskit as _sk; SKS = _sk.from_npz(d)      # [fix3-struts]
if SKS and SKS['mode'] == 'cut':      # [fix3-struts] the real cut chassis: every vertex / face centroid outside the cut boxes + the caps
    kill[:] = False; Xc = samples(X, B['Chassis_f']); Xc = np.r_[Xc[~_sk.inside(Xc, SKS)], _sk.cap_samples(X, B['Chassis_f'].astype(np.int64), SKS)]      # [fix3-struts]
else: Fc = B['Chassis_f'][~kill[B['Chassis_f']].any(1)]; Xc = samples(X, Fc)      # [fix3-struts] legacy (f2)
s, _ = SD(Xc, 0.01); fin = np.isfinite(s)      # [fix3-struts] (was on the same line as the f2 kill)
if (fin & (s < -1e-5)).any():      # [hz] SurfDist signs by the nearest face's plane: wrong next to the hole walls of a sloped deck (false 'inside' 10 mm under it);
    from labyrinthkit import RobustSurfDist      # [hz] re-sign the suspects by ray parity on the closed deck solid
    q_ = fin & (s < -1e-5); r_, _ = RobustSurfDist(mv, Fd, sign_within=0.012)(Xc[q_], 0.02); s[q_] = np.where(np.isfinite(r_), r_, np.abs(s[q_]))
out['chassis_vs_deck'] = dict(inside=int((fin & (s < -1e-5)).sum()), min_gap_mm=round(float(s[fin].min()) * 1000, 2) if fin.any() else None)
if (fin & (s < -1e-5)).any(): out['chassis_vs_deck']['worst_at_mm'] = (Xc[fin & (s < -1e-5)][:3] * 1000).round(1).tolist()
# thickness: from top-facing deck faces, ray down along -n to the next deck face
from scipy.spatial import cKDTree as _KD
P = mv[Fd]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); a = np.linalg.norm(n, axis=1); n /= np.maximum(a, 1e-30)[:, None]
top = (n[:, 2] > 0.2) & (a > 1e-10); bot = (n[:, 2] < -0.2) & (a > 1e-10); dd_, _ = _KD(P[bot].mean(1)).query(P[top].mean(1)[::10])
if '--no-selfx' not in sys.argv:      # [fix4-cover2] G4: self-intersection of the deck solid (solidaudit.py, BVH, faces sharing a vertex excluded); target 0
    import subprocess, tempfile, os
    tj_ = os.path.join(tempfile.mkdtemp(), 'sa.json'); subprocess.run([sys.executable, 'solidaudit.py', sys.argv[1], '--only', 'mount.deck', '--no-hw', '--json', tj_], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        cs_ = json.load(open(tj_))['components']; c_ = cs_.get('mount.deck') or next(iter(cs_.values()))
        out['deck_selfx'] = dict(pairs=int(c_.get('selfx_pairs') or 0), closed=bool(c_.get('closed')), inverted=bool(c_.get('inverted')), oriented=bool(c_.get('oriented')), mode=c_.get('selfx_mode'))
    except Exception as e_: out['deck_selfx'] = dict(error=str(e_))
out['deck_thickness_probe_mm'] = dict(p1=round(float(np.percentile(dd_, 1)) * 1000, 2), median=round(float(np.median(dd_)) * 1000, 2))
if 'plinth_v' in d.files:
    pv = d['plinth_v'].astype(float); pf = d['plinth_f'].astype(np.int64)
    e = np.concatenate([pf[:, [0, 1]], pf[:, [1, 2]], pf[:, [2, 0]]]); key = np.minimum(e[:, 0], e[:, 1]) * len(pv) + np.maximum(e[:, 0], e[:, 1]); _, c = np.unique(key, return_counts=True)
    k = np.load('keep.npz'); kb = k['f'][k['lab'] == 7]; SB = SurfDist(k['v'].astype(float), kb); sb, _ = SB(pv, 0.02)
    from scipy.spatial import cKDTree
    Vb = d['v'][np.unique(d['f'][(d['part'] == 0) & (d['side'] == 0)])]; db, _ = cKDTree(Vb).query(pv); dc, _ = cKDTree(Xc if SKS and SKS['mode'] == 'cut' else X[~kill]).query(pv)      # [fix3-struts]
    out['nose_floor'] = dict(faces=int(len(pf)), boundary_edges=int((c == 1).sum()), nonmanifold_edges=int((c > 2).sum()), bbox_mm=(np.r_[pv.min(0), pv.max(0)] * 1000).round(1).tolist(),
                             to_bumper_mm=round(float(np.nanmin(np.abs(sb))) * 1000, 2), to_body_outer_mm=round(float(db.min()) * 1000, 2), to_chassis_mm=round(float(dc.min()) * 1000, 2),
                             mirror_asym_mm=round(float(abs(pv[:, 1].max() + pv[:, 1].min())) * 1000, 3))
print(json.dumps(out, indent=1))
