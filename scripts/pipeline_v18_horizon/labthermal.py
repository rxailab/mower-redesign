# [fix4-cover2] M5: labyrinth / deck clearances under differential thermal growth of the light cover vs the dark body (standing check;
# after the mechanics review's lab_thermal.py). The cover is scaled by (1 - eps) and the body + deck by (1 + eps) about the cover's
# thermal datum, eps = body strain - cover strain (CTE body PC/ASA 75e-6, cover ASA 90e-6 /K: -10 C -> +0.5e-3, uniform 60 C -> -0.56e-3,
# dark body 75 C / light cover 50 C -> +1.47e-3). Clearances as labcheck (RobustSurfDist, cover / body vertices within 8 mm of the other
# part, deck vertices within 12 mm of the cover).
# Datum: x from the cover's x-locators (generator report mounts.cover.fix4_cover2.x_locators) or the round nose pin (f3), y = 0 (nose /
# tail pins on the centre line), z = 195.6 (as the review). usage: python labthermal.py NPZ CHECK.json [OUT.json] [--datum-x MM]
import numpy as np, sys, json
sys.path.insert(0, '.')
from labyrinthkit import RobustSurfDist as SurfDist
from scipy.spatial import cKDTree
a = sys.argv[1:]; d = np.load(a[0]); rep = json.load(open(a[1])); out_p = a[2] if len(a) > 2 and not a[2].startswith('--') else None
cov = rep['mounts']['cover']; loc = (cov.get('fix4_cover2') or {}).get('x_locators')
x0 = float(np.mean([l_['at_mm'][0] for l_ in loc])) if loc else float(cov['location']['datum_pins'][0]['at_mm'][0])
if '--datum-x' in a: x0 = float(a[a.index('--datum-x') + 1])
p0 = np.array([x0 / 1000, 0.0, 0.1956]); src = 'x-locators' if loc and '--datum-x' not in a else ('given' if '--datum-x' in a else 'round nose pin')
V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; lab = d['lab']
mv = d['mount_v'].astype(float); mf = d['mount_f'].astype(np.int64); MKN = [str(x) for x in d['mount_kinds']]; Fd = mf[d['mount_kind'] == MKN.index('deck')]
Fb = F[(part == 0) & (lab != 4)]; Fc = F[(part == 1) & (lab != 4)]
SB, SC, SDk = SurfDist(V, Fb), SurfDist(V, Fc), SurfDist(mv, Fd, sign_within=0.0014)
cv_ = np.unique(Fc); bv_ = np.unique(Fb)
nb_, _ = cKDTree(V[bv_]).query(V[cv_], distance_upper_bound=0.008); Pc = V[cv_[np.isfinite(nb_)]]
nc_, _ = cKDTree(V[cv_]).query(V[bv_], distance_upper_bound=0.008); Pb = V[bv_[np.isfinite(nc_)]]
dk_ = np.unique(Fd); nd_, _ = cKDTree(Pc).query(mv[dk_], distance_upper_bound=0.012); Pd = mv[dk_[np.isfinite(nd_)]]
out = dict(datum_mm=np.round(p0 * 1000, 2).tolist(), datum_from=src, cases={})
print('thermal datum (mm):', out['datum_mm'], src)
for eps in (0.0, -0.56e-3, 0.5e-3, 1.0e-3, 1.47e-3):
    r = {}
    Pc_ = p0 + (Pc - p0) * (1 - eps); Pb_ = p0 + (Pb - p0) * (1 + eps); Pd_ = p0 + (Pd - p0) * (1 + eps)
    for nm, X, S in (('cover->body', Pc_, SB), ('body->cover', Pb_, SC), ('cover->deck', Pc_, SDk), ('deck->cover', Pd_, SC)):
        q, _ = S(X, 0.003); ok = np.isfinite(q); i = np.argmin(np.where(ok, q, np.inf))
        r[nm] = dict(min_mm=round(float(q[i]) * 1000, 3), at_mm=np.round(X[i] * 1000, 1).tolist(), n_inside=int((q[ok] < 0).sum()), n_lt_0p3=int((q[ok] < 0.0003).sum()))
        reg = {'nose': X[:, 0] > 0.2, 'tail': X[:, 0] < -0.2, 'flanks': np.abs(X[:, 0]) <= 0.2}      # [fix4-cover2] per region (nose / tail ends vs the flanks)
        r[nm]['regions'] = {k: round(float(np.where(ok & m, q, np.inf).min()) * 1000, 3) if (ok & m).any() else None for k, m in reg.items()}
    out['cases']['%+.2e' % eps] = r; print('%+.2e' % eps, json.dumps({k: (v['min_mm'], v['at_mm'], v['n_inside'], v['regions']) for k, v in r.items()}), flush=True)
mins = {k: min(c[k]['min_mm'] for c in out['cases'].values()) for k in ('cover->body', 'body->cover', 'cover->deck', 'deck->cover')}
out['min_over_cases_mm'] = mins; out['ok'] = bool(min(mins.values()) > 0.0)
print('min over the cases (mm):', mins, ' NO CONTACT' if out['ok'] else ' CONTACT')
if out_p: json.dump(out, open(out_p, 'w'), indent=1)
