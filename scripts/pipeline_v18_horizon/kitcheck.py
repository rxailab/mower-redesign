# [v18b] Static checks of the FIXED lid kit (kitf: well, pods, pockets, latch ...) and the closed moving kit against the parts around it.
# usage: python kitcheck.py DESIGN.npz [OUT.json]
#  - controls: distance of every kit face sample to the control panel surface (dense samples) and whether it lies inside the panel
#  - cover: depth of every kit vertex under the cover's outer surface (must leave the 2.5 mm wall + 0.5 mm; the parts that ARE the
#    opening wall / the cover's inside face are exempt), and the body deck / screws vs the kit
#  - chassis: height over the chassis top (height map, 5 mm cells, 3x3 max)
import sys, json, numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import maximum_filter
from scipy.interpolate import LinearNDInterpolator
import lidmech as L
NPZ = sys.argv[1]; OUT = sys.argv[2] if len(sys.argv) > 2 else None
d = np.load(NPZ); V, F, lab = d['v'], d['f'], d['lab']; part = d['part']
PFI = {v: k for k, v in L.PF.items()}; PMI = {v: k for k, v in L.PM.items()}
def samples(Vs, Fs, n=4):
    w = np.array([[a, b, n - a - b] for a in range(n + 1) for b in range(n + 1 - a)], float) / n
    return np.einsum('kj,fjc->fkc', w, Vs[Fs]).reshape(-1, 3)
c = np.load('control.npz'); CS = samples(c['v'], c['f'], 4); CT = cKDTree(CS)
cP = c['v'][c['f']]; cn = np.cross(cP[:, 1] - cP[:, 0], cP[:, 2] - cP[:, 0]); cn /= np.maximum(np.linalg.norm(cn, axis=1, keepdims=True), 1e-20); cc = cP.mean(1); CC = cKDTree(cc)
lp = d['lid_poly']; x0, x1, y1 = lp[:, 0].min() - 0.06, lp[:, 0].max() + 0.03, lp[:, 1].max() + 0.03
side = d['side'] if 'side' in d.files else np.zeros(len(F), np.uint8)      # [v18c] real wall: the A-surface lookup uses the outer skin only
Cf = V[F].mean(1); m_ = (part == 1) & (lab != 4) & (side == 0) & (Cf[:, 0] > x0) & (Cf[:, 0] < x1) & (np.abs(Cf[:, 1]) < y1) & (Cf[:, 2] > 0.24)
cvv = np.unique(F[m_]); ZT = cKDTree(V[cvv][:, :2]); ZZ = V[cvv][:, 2]
ZI = lambda xy: np.where(ZT.query(xy)[0] < 0.002, ZZ[ZT.query(xy)[1]], np.nan)        # cover's outer surface: nearest vertex (mesh ~0.5 mm)
H = maximum_filter(np.load('chassis_top.npy'), 3)
import shapely
LPG = shapely.Polygon(lp)
POLY_D = lambda xy: shapely.distance(LPG.boundary, shapely.points(xy)) * np.where(shapely.contains_xy(LPG, xy[:, 0], xy[:, 1]), -1, 1)
MV = d['mount_v']; MT = cKDTree(MV) if len(MV) else None
rep = {'npz': NPZ, 'fixed': {}, 'moving_closed': {}}
SOLC = None
if (side == 1).any():
    sys.path.insert(0, '.'); from shell_wall import SurfDist; SOLC = SurfDist(V, F[(part == 1) & (lab != 4)])
EXEMPT_WALL = ('cover', 'edge', 'seal', 'drain', 'ear')      # [fix1] ear: the well's staked fixing ears register under the cover's inner skin like the gutter flange
def one(Vs, Fs, name):
    S = samples(Vs, Fs, 2); dd, _ = CT.query(S, workers=16); _, ii = CC.query(S, workers=16); inside = ((S - cc[ii]) * cn[ii]).sum(1) < -1e-4
    inside &= (dd < 0.004) & np.all((S > c['v'].min(0) + 1e-4) & (S < c['v'].max(0) - 1e-4), axis=1)      # only inside the panel's box
    zA = ZI(S[:, :2]); dep = zA - S[:, 2]; ok = np.isfinite(dep) & (POLY_D(S[:, :2]) > 0.003)        # away from the rolled opening edge (the gutter seats on the cover there)
    gx = np.clip(((S[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1); gy = np.clip(((S[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)
    r = {'controls_min_mm': round(float(dd.min()) * 1000, 2), 'inside_controls': int(inside.sum()),
         'under_cover_min_mm': round(float(dep[ok].min()) * 1000, 2) if ok.any() else None,
         'over_chassis_min_mm': round(float((S[:, 2] - H[gx, gy]).min()) * 1000, 1)}
    if MT is not None: r['deck_min_mm'] = round(float(MT.query(S)[0].min()) * 1000, 2)
    if SOLC is not None:      # [v18c] gap to the cover SOLID (outer + inner skin + rims): < 0 = inside the cover wall
        sv_, _ = SOLC(np.unique(Vs[Fs.ravel()], axis=0), 0.01); r['cover_solid_gap_min_mm'] = round(float(np.nanmin(np.where(np.isfinite(sv_), sv_, np.nan))) * 1000, 2) if np.isfinite(sv_).any() else None
    return r
for k in np.unique(d['kitf_part']):
    nm = PFI[int(k)]; rep['fixed'][nm] = one(d['kitf_v'], d['kitf_f'][d['kitf_part'] == k], nm)
for k in np.unique(d['kitm_part']):
    nm = PMI[int(k)]; rep['moving_closed'][nm] = one(d['kitm_v'], d['kitm_f'][d['kitm_part'] == k], nm)
if 'stop_v' in d.files:      # [fix3-stop] STOP key parts (move with the lid, cap at rest)
    import stopkit as _sk
    for k in np.unique(d['stop_part']): nm = 'stop_' + _sk.NAME[int(k)]; rep['moving_closed'][nm] = one(d['stop_v'], d['stop_f'][d['stop_part'] == k], nm)
wall = min(v['under_cover_min_mm'] for k, v in rep['fixed'].items() if k not in EXEMPT_WALL and v['under_cover_min_mm'] is not None)
ctl = min(v['controls_min_mm'] for k, v in rep['fixed'].items() if k != 'lip')
ins = sum(v['inside_controls'] for k, v in rep['fixed'].items() if k != 'lip') + sum(v['inside_controls'] for v in rep['moving_closed'].values())
rep['summary'] = {'fixed_kit_min_under_cover_A_mm (needs >= 3.0: 2.5 wall + 0.5)': wall, 'fixed_kit_min_to_controls_mm (lip = designed contact)': ctl,
                  'kit_samples_inside_controls': ins, 'kit_min_over_chassis_mm': min(v['over_chassis_min_mm'] for v in list(rep['fixed'].values()) + list(rep['moving_closed'].values())),
                  'kit_min_to_body_deck_mm': min(v.get('deck_min_mm', 99) for v in rep['fixed'].values())}
if SOLC is not None:
    rep['summary']['fixed_kit_min_gap_to_cover_solid_mm (registration parts well/edge/seal/drain/cover exempt)'] = min(v['cover_solid_gap_min_mm'] for k_, v in rep['fixed'].items() if k_ not in EXEMPT_WALL + ('well',) and v.get('cover_solid_gap_min_mm') is not None)
    rep['summary']['well_registration_gap_to_cover_inner_mm'] = rep['fixed'].get('well', {}).get('cover_solid_gap_min_mm')
    if 'ear' in rep['fixed']: rep['summary']['ear_registration_gap_to_cover_inner_mm'] = rep['fixed']['ear'].get('cover_solid_gap_min_mm')
rep['ok'] = bool(wall >= 3.0 and ctl >= 1.0 and ins == 0)
for k, v in rep['fixed'].items(): print('fixed  %-8s' % k, v)
for k, v in rep['moving_closed'].items(): print('moving %-8s' % k, v)
print(json.dumps(rep['summary'])); print('KIT OK:', rep['ok'])
if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
