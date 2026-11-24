# Kinematic clearance check of the v18 lid on the generated meshes (exact y-slices; the lid opens by a pure rotation about the
# axis lid_pivot, parallel to Y, so every plane y = const maps onto itself).
# usage: python lidsim.py DESIGN.npz [OUT.json] [workers]
# Moving: lid skin (lab 4) + every kitm part. Fixed: cover A-surface round the opening + every kitf part + controls (control.npz)
# + chassis top (chassis_top.npy). Angles: latch over-travel .. 0 .. open_deg (0.05 deg steps to 3 deg, then 0.5 deg).
import sys, json, numpy as np, shapely
from shapely import STRtree
from multiprocessing import Pool
from scipy.ndimage import maximum_filter
from slicekit import slice_y, rot
import lidmech as LMK

NPZ = sys.argv[1]; OUT = sys.argv[2] if len(sys.argv) > 2 else None; NW = int(sys.argv[3]) if len(sys.argv) > 3 else 16
d = np.load(NPZ); V, F, lab = d['v'], d['f'], d['lab']
Q = np.array(d['lid_pivot'], float); OPEN = float(d['lid_open'][0]); YC = float(d['lid_hinge_y'][0])
lp = d['lid_poly']; x_rear, x_front, HY = lp[:, 0].min(), lp[:, 0].max(), lp[:, 1].max()
PMI = {v: k for k, v in LMK.PM.items()}; PFI = {v: k for k, v in LMK.PF.items()}
C = V[F].mean(1); crop = (C[:, 0] > x_rear - 0.045) & (C[:, 0] < x_front + 0.03) & (np.abs(C[:, 1]) < HY + 0.03) & (C[:, 2] > 0.245)
G = {'lidskin': (V, F[crop & (lab == 4)], True), 'cover': (V, F[crop & (lab != 4)], False)}
for k, nm in PMI.items():
    m = d['kitm_part'] == k
    if m.any(): G[nm] = (d['kitm_v'], d['kitm_f'][m], True)
for k, nm in PFI.items():
    m = d['kitf_part'] == k
    if m.any(): G['f_' + nm] = (d['kitf_v'], d['kitf_f'][m], False)
if 'stop_v' in d.files:      # [fix3-stop] STOP key parts (stopkit.py) move with the lid, cap at rest
    import stopkit as _sk
    for k, nm in _sk.NAME.items():
        m = d['stop_part'] == k
        if m.any(): G['stop_' + nm] = (d['stop_v'], d['stop_f'][m], True)
if 'lights_halo_v' in d.files and len(d['lights_halo_f']):      # [fix3-int] status-halo feed pipe (moves with the lid) and its emitter on the control panel (fixed)
    G['halo_feed'] = (d['lights_halo_v'], d['lights_halo_f'], True)      # [fix3-int]
    _ke = d['lights_kit_kind'] == [str(k_) for k_ in d['lights_kinds']].index('emitter')      # [fix3-int]
    if _ke.any(): G['halo_emitter'] = (d['lights_kit_v'], d['lights_kit_f'][_ke], False)      # [fix3-int]
c = np.load('control.npz'); G['controls'] = (c['v'], c['f'], False)
MOV = [k for k, g in G.items() if g[2]]; FIX = [k for k, g in G.items() if not g[2]]
# designed contacts / running fits (reported separately): (moving, fixed, rule)
DESIGNED = {('tongue', 'f_guide'): 'running fit (0.2 radial / 0.3 axial)', ('lug', 'f_guide'): 'lug in the stop slot (0.3 radial)',
            ('lug', 'f_bumper'): 'open stop: contact at open_deg', ('tongue', 'f_spring'): 'spring band lies on the tongue (moves with it)',
            ('lug', 'f_spring'): 'spring band (moves with the tongue)', ('striker', 'f_claw'): 'latch: rotor holds the bar; turns open on push (push-push)',
            ('tongue', 'f_pod'): 'running fit in the pod mouth / channel (0.2 radial / 0.3 axial)', ('lug', 'f_pod'): 'lug in the stop slot through the pod wall (0.3)',
            ('plunger', 'controls'): '[fix2] STOP plunger: 0.5 above the STOP key closed, presses it at the latch over-travel (designed contact)',
            ('stop_tip', 'controls'): '[fix3-stop] spring-backed TPE tip of the STOP key: 1.45 over the key closed, may touch it at the lid over-travel (designed contact)'}
DESIGNED.update(getattr(LMK, 'hingekit').DESIGNED3)                                 # [fix3-hinge] tape idler over the tongue
SEAL = [('lidskin', 'f_seal'), ('underside', 'f_seal'), ('edge', 'f_seal')]            # compressed at closed (designed), must lift off
xl_ = float(d['lid_latch_x'][0]) if 'lid_latch_x' in d.files else x_front + LMK.DEFAULTS['latch_dx']      # [v18b] striker position from the npz
lever = float(xl_ - Q[0]); OVER = -np.degrees(LMK.DEFAULTS['latch_over'] / lever)
ANG = np.unique(np.round(np.r_[np.arange(OVER, 0, 0.05), [OVER], np.arange(0, 3.0, 0.05), np.arange(3.0, OPEN, 0.5), [OPEN]], 3))
r_ = LMK.params({})
ys = np.r_[np.arange(-HY - 0.001, HY + 0.0011, 0.001)]
for s_ in (1, -1):
    ys = np.r_[ys, s_ * YC + np.arange(-0.0095, 0.0096, 0.00025), s_ * (HY - np.arange(0, 0.012, 0.00025))]
ys = np.unique(np.round(np.r_[ys, np.arange(-0.009, 0.0091, 0.00025)], 6))
FAR = 0.010

def bbox(seg): return np.r_[seg[..., 0].min(), seg[..., 1].min(), seg[..., 0].max(), seg[..., 1].max()]
def bdist(a, b):
    dx = max(0.0, max(a[0], b[0]) - min(a[2], b[2])); dz = max(0.0, max(a[1], b[1]) - min(a[3], b[3])); return float(np.hypot(dx, dz))

def work(y):
    sl = {k: slice_y(v_, f_, y) for k, (v_, f_, mv) in G.items()}
    sl = {k: s for k, s in sl.items() if len(s)}
    trees = {k: STRtree(shapely.linestrings(sl[k])) for k in FIX if k in sl}; fb = {k: bbox(sl[k]) for k in trees}
    out = {}
    for th in ANG:
        for a in MOV:
            if a not in sl: continue
            sr = rot(sl[a], Q, th); ab = bbox(sr); ga = None
            for b in trees:
                bd = bdist(ab, fb[b])
                if bd > FAR: dd, hit = bd, False
                else:
                    if ga is None: ga = shapely.linestrings(sr)
                    tr = trees[b]; hit = tr.query(ga, predicate='intersects').shape[1] > 0
                    _, dist = tr.query_nearest(ga, return_distance=True, all_matches=False); dd = 0.0 if hit else float(dist.min())
                key = (a, b, float(th)); cur = out.get(key)
                if cur is None or dd < cur[0]: out[key] = (dd, hit)
    return y, out

if __name__ == '__main__':
    with Pool(NW) as pool: RR = pool.map(work, ys, chunksize=2)
    RES = {}
    for y, out in RR:
        for (a, b, th), (dd, hit) in out.items():
            k = (a, b); row = RES.setdefault(k, {}); cur = row.get(th)
            if cur is None or dd < cur[0]: row[th] = (dd, hit, y)
    # chassis top (height map, 5 mm cells, 3x3 max filter) under every moving vertex
    H = maximum_filter(np.load('chassis_top.npy'), 3)
    mv = np.concatenate([V[np.unique(G['lidskin'][1])], d['kitm_v']])
    chs = {}
    for th in ANG:
        p = rot(mv[:, [0, 2]][:, None, :], Q, th)[:, 0]; gx = np.clip(((p[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1); gy = np.clip(((mv[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)
        chs[float(th)] = float((p[:, 1] - H[gx, gy]).min())
    def classify(a, b, th):
        if (a, b) in DESIGNED: return 'designed'
        if (a, b) in SEAL: return 'designed'                                     # the bulb is drawn compressed; it follows the lid up
        return 'check'
    show = [OVER, 0.0, 0.6, 1.0, 2.0] + list(np.arange(5, OPEN + 0.01, 5.0)) + ([OPEN] if OPEN % 5 else [])
    show = sorted(set(round(float(t), 3) for t in show))
    near = lambda row, t: min(row, key=lambda q: abs(q - t))
    rep = {'npz': NPZ, 'Q': Q.tolist(), 'open_deg': OPEN, 'over_travel_deg': round(OVER, 3), 'n_slices': int(len(ys)), 'n_angles': int(len(ANG)), 'pairs': {}, 'summary': {}}
    lines = ['%-10s %-11s ' % ('moving', 'fixed') + ' '.join('%6s' % ('%g' % t) for t in show) + '   min (deg, y mm)   class']
    worst = {'check': (np.inf, None)}
    for (a, b), row in sorted(RES.items()):
        mn = min((row[t][0], t, row[t][2]) for t in row if classify(a, b, t) == 'check') if any(classify(a, b, t) == 'check' for t in row) else None
        if mn and mn[0] >= FAR and (a, b) not in DESIGNED: continue
        cls = 'designed' if (a, b) in DESIGNED else ('seal' if (a, b) in SEAL else 'check')
        vals = []
        for t in show:
            dd, hit, _ = row[near(row, t)]
            vals.append('     X' if hit else ('   >10' if dd >= FAR else '%6.2f' % (dd * 1000)))
        lines.append('%-10s %-11s ' % (a, b) + ' '.join(vals) + ('   %.3f mm @ %.2f deg, y %.1f' % (mn[0] * 1000, mn[1], mn[2] * 1000) if mn else '') + '   ' + cls)
        rep['pairs']['%s|%s' % (a, b)] = {'class': cls, 'min_mm': None if not mn else round(mn[0] * 1000, 3), 'at_deg': None if not mn else mn[1], 'y_mm': None if not mn else round(mn[2] * 1000, 2),
                                          'by_angle_mm': {('%g' % t): (None if row[near(row, t)][0] >= FAR else round(row[near(row, t)][0] * 1000, 3)) for t in show},
                                          'hits_deg': [t for t in sorted(row) if row[t][1]][:5]}
        if cls != 'designed' and mn and mn[0] < worst['check'][0]: worst['check'] = (mn[0], (a, b, mn[1], mn[2]))
    # summary per angle: lid vs cover opening edge, vs controls, vs chassis, vs the rest of the kit
    groups = {'cover_edge': lambda a, b: b in ('cover', 'f_edge', 'f_cover'), 'controls': lambda a, b: b == 'controls',
              'kit': lambda a, b: b.startswith('f_') and b not in ('f_edge', 'f_cover')}
    summ = {}
    for gname, fn in groups.items():
        per = {}
        for (a, b), row in RES.items():
            if not fn(a, b): continue
            for t, (dd, hit, y) in row.items():
                if classify(a, b, t) != 'check': continue
                if t not in per or dd < per[t][0]: per[t] = (dd, a + '|' + b, y)
        summ[gname] = per
    summ['chassis'] = {t: (v, 'moving|chassis_top', None) for t, v in chs.items()}
    lines.append(''); lines.append('per-angle minimum (mm) of everything that is not a designed contact / running fit:')
    lines.append('%-11s ' % 'vs' + ' '.join('%6s' % ('%g' % t) for t in show) + '   overall min')
    for gname, per in summ.items():
        vals = ['%6.2f' % (min(per[near(per, t)][0], 0.0999) * 1000) if per[near(per, t)][0] < FAR else '   >10' for t in show]
        mn = min(per.items(), key=lambda kv: kv[1][0])
        lines.append('%-11s ' % gname + ' '.join(vals) + '   %.3f @ %.2f deg (%s%s)' % (mn[1][0] * 1000, mn[0], mn[1][1], '' if mn[1][2] is None else ', y %.1f' % (mn[1][2] * 1000)))
        rep['summary'][gname] = {'min_mm': round(mn[1][0] * 1000, 3), 'at_deg': mn[0], 'pair': mn[1][1], 'by_angle_mm': {('%g' % t): round(min(per[near(per, t)][0], 10) * 1000, 3) for t in show}}
    ok = all(v['min_mm'] >= 0.5 for v in rep['summary'].values())
    # designed contacts: seal must be compressed at 0 and lift off when opening, the lug must land on the bumper exactly at open_deg
    seal_lift = min((t for t in ANG if t > 0 and all(RES[k][t][0] > 0 for k in SEAL if k in RES and t in RES[k])), default=None)
    stop_hit = [t for t in sorted(RES.get(('lug', 'f_bumper'), {})) if RES[('lug', 'f_bumper')][t][1] or RES[('lug', 'f_bumper')][t][0] < 1e-5]
    run = {k: round(min(v[0] for v in RES[k].values()) * 1000, 3) for k in DESIGNED if k in RES}
    rep['designed'] = {'seal_lifts_off_at_deg': seal_lift, 'stop_contact_deg': stop_hit[:3], 'running_fit_min_mm': {'%s|%s' % k: v for k, v in run.items()},
                       'running_fit_hits': {'%s|%s' % k: [t for t in sorted(RES[k]) if RES[k][t][1]][:3] for k in DESIGNED if k in RES}}
    rep['ok_0.5mm'] = bool(ok)
    lines.append(''); lines.append('designed: ' + json.dumps(rep['designed'])); lines.append('ALL >= 0.5 mm: %s' % ok)
    print('\n'.join(lines)); rep['table'] = lines
    if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
