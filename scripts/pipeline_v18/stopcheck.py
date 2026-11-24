# [fix3-stop] STOP key compliance + stroke check on a generated design.   usage: python stopcheck.py DESIGN.npz PARAMS.json [OUT.json]
#  A  EN 50636-2-107:2015/A3:2021 20.101.4 geometry, measured on the meshes: minor dimension, share of the actuator raised >= 5 mm over the
#     immediate surrounding area (band 15 mm round the cap outline, its HIGHEST point is the reference), clear width of that area, top
#     surface / highest point, legend size and colour contrast (7.9).
#  B  force / travel / tolerance budget (stopkit.budget, worst case + Monte Carlo) and the latch order (latch cannot release before the cap
#     is on its stop, i.e. before the real key is pressed).
#  C  stroke simulation by y-slices (exact 2-D, every motion is in planes y = const): cap stroke 0 .. stroke in 0.25 mm steps (parallel
#     arms: carrier translates on the arc, arms turn about their rear pins), lid closed and at the latch over-travel; the tip stops on
#     the key and the compliance spring takes the rest. Clearances of every moving STOP part to the lid-side parts, the lid, the fixed
#     kit and the controls; designed contacts reported separately.
#  D  lid open: the exposed real key (fixed hardware) against the same 20.101.4 geometry rules, and the STOP parts far from it.
#  [fix4-stop2] v4 key (stop4.py): S1 true tip gap measured on the meshes and asserted against the budget gap (0.05); S5 flat-face
#     diameters; S2 grit + edge-roll case for the skirt; S6 worst-stack rows (rigid parts vs the real key); G2 / G3 static lid-frame pass
#     (lid-fixed STOP parts and the halo pipe vs the lid moulding, >= 0.5 except the designed fusions); designed contacts of the v4 parts.
import sys, json, numpy as np, shapely
from shapely.geometry import Polygon, Point
sys.path.insert(0, '.')
import stopkit as SK, lidmech as LM, stop4 as S4      # [fix4-stop2]
from slicekit import slice_y, rot, clearance

NPZ, PJ = sys.argv[1], sys.argv[2]; OUT = sys.argv[3] if len(sys.argv) > 3 else None
d = np.load(NPZ); P = json.load(open(PJ)); SKP = P['lid']['stop_key']; V4 = bool(SKP.get('v4')); p = (S4.params4 if V4 else SK.params)(SKP)      # [fix4-stop2]
V, F, lab = d['v'], d['f'], d['lab']; side = d['side'] if 'side' in d.files else np.zeros(len(F), np.uint8)
sv, sf, sp, sl = d['stop_v'], d['stop_f'], d['stop_part'], d['stop_lab']; xk, yk, z0, S = [float(t) for t in d['stop_axis']]
Q = np.array(d['lid_pivot'], float); OPEN = float(d['lid_open'][0]); xl = float(d['lid_latch_x'][0]); lever = xl - Q[0]
OVER = -np.degrees(LM.DEFAULTS['latch_over'] / lever)
R = p['r']; rep = {'npz': NPZ}

# ---------------------------------------------------------------- A: geometry rules on the closed machine
cap = sf[(sp == SK.PART['cap'])]; Pc = sv[cap]; nrm = np.cross(Pc[:, 1] - Pc[:, 0], Pc[:, 2] - Pc[:, 0]); A3 = 0.5 * np.linalg.norm(nrm, axis=1)
nz = nrm[:, 2] / np.maximum(2 * A3, 1e-30); cen = Pc.mean(1)
up = (nz > 0.2) & (np.hypot(cen[:, 0] - xk, cen[:, 1] - yk) <= R + 1e-5) & (cen[:, 2] > z0 + p['stroke'] + p['clear'] + p['side'] * 0.5)
outline = shapely.MultiPoint(sv[np.unique(cap)][:, :2]).convex_hull
mrr = np.array(outline.minimum_rotated_rectangle.exterior.coords); ed = np.linalg.norm(np.diff(mrr, axis=0), axis=1)
ang = np.radians(np.arange(0, 180, 1.0)); hull = np.array(outline.exterior.coords)
feret = min(float(np.ptp(hull @ np.array([np.cos(a), np.sin(a)]))) for a in ang)
# immediate surrounding area: every outer visible surface (lid skin, cover outer skin, the STOP parts that are not the cap) in the
# annulus R .. R + 15 mm round the cap outline (plan)
Cf = V[F].mean(1); rr = np.hypot(Cf[:, 0] - xk, Cf[:, 1] - yk)
outer = (lab == 4) | (side == 0)
band = outer & (rr > R + 0.0002) & (rr < R + 0.015)
zb = V[F[band]][:, :, 2]; z_ref = float(zb.max()); z_bmin = float(zb.min())
Cs = sv[sf].mean(1); rs = np.hypot(Cs[:, 0] - xk, Cs[:, 1] - yk); other = (sp != SK.PART['cap'])
zs_band = float(sv[sf[other & (rs > R + 0.0002) & (rs < R + 0.015)]][:, :, 2].max()) if (other & (rs > R + 0.0002) & (rs < R + 0.015)).any() else -1
z_ref = max(z_ref, zs_band)
pa = A3 * np.abs(nz)                                                           # plan area of each up-facing cap face
zt = cen[:, 2]; plan_total = np.pi * R ** 2
raised = up & (zt >= z_ref + 0.005)
o = np.argsort(-zt[up]); cum = np.cumsum(pa[up][o]) / plan_total; z20 = float(zt[up][o][np.searchsorted(cum, 0.2)])
# clear width of the surrounding area: along 72 rays, the distance from the cap outline to the first outer surface higher than z20 - 5 mm
Cu = Cf[outer]; Zu = Cu[:, 2]; widths = []
for a in np.radians(np.arange(0, 360, 5.0)):
    dvec = np.array([np.cos(a), np.sin(a)]); rel = Cu[:, :2] - [xk, yk]; along = rel @ dvec; perp = np.abs(rel @ np.array([-dvec[1], dvec[0]]))
    m = (perp < 0.0015) & (along > R) & (along < R + 0.06) & (Zu > z20 - 0.005); widths.append(float((along[m].min() - R) * 1000) if m.any() else 60.0)
body_top = float(V[F[outer & (lab != 4)]][:, :, 2].max()); lid_top = float(V[F[lab == 4]][:, :, 2].max())
def lum(h):
    c = [int(h.lstrip('#')[i:i + 2], 16) / 255 for i in (0, 2, 4)]; c = [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]; return 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
col = P.get('colors', {}); red, white, lidc = col.get('stop', '#C8102E'), '#F4F4F2', col.get('lid', '#CDD0CF')
cr = lambda a, b: round((max(lum(a), lum(b)) + 0.05) / (min(lum(a), lum(b)) + 0.05), 2)
leg = SK.legend_polys(p, xk, yk)[1]
rep['A_geometry_20.101.4'] = dict(
    minor_dimension_mm=round(feret * 1000, 2), min_rect_mm=sorted(np.round(ed[:2] * 1000, 2).tolist()), req_minor_dimension_mm=35.0,
    surround_band_z_mm=[round(z_bmin * 1000, 2), round(z_ref * 1000, 2)], cap_top_mm=round(float(zt[up].max()) * 1000, 2), cap_z_at_20pct_area_mm=round(z20 * 1000, 2),
    raised_ge_5mm_plan_fraction=round(float(pa[raised].sum() / plan_total), 3), req_fraction=0.20, height_of_20pct_area_over_band_max_mm=round((z20 - z_ref) * 1000, 2),
    surround_clear_width_min_mm=round(min(widths), 1), req_surround_width_mm=15.0, highest_point_of_machine=bool(zt.max() >= max(body_top, lid_top)),
    machine_top_without_key_mm=round(max(body_top, lid_top) * 1000, 2), legend=leg, legend_reads='front' if p.get('legend_front', True) else 'rear',
    colour=dict(stop=red, legend=white, contrast_legend_on_red=cr(red, white), contrast_red_on_lid=cr(red, lidc)))
g = rep['A_geometry_20.101.4']
if V4: g['colour']['iso3864_4'] = S4.iso3864_red(red)      # [fix4-stop2] A6: the STOP red against the ISO 3864-4 safety-red chromaticity box
rr_c = np.hypot(cen[:, 0] - xk, cen[:, 1] - yk)      # [fix4-stop2] S5: the actuating face (top faces flatter than 10 / 30 deg), not only the outline
g['flat_face_d_mm'] = {'slope_lt_10deg': round(2 * float(rr_c[up & (nz > np.cos(np.radians(10)))].max()) * 1000, 2), 'slope_lt_30deg': round(2 * float(rr_c[up & (nz > np.cos(np.radians(30)))].max()) * 1000, 2)}
g['pass'] = bool(g['minor_dimension_mm'] >= 35 and g['raised_ge_5mm_plan_fraction'] >= 0.2 and g['surround_clear_width_min_mm'] >= 15 and g['highest_point_of_machine']
                 and (not V4 or (g['flat_face_d_mm']['slope_lt_10deg'] >= 36.0 and g['colour']['iso3864_4']['inside'])))      # [fix4-stop2] >= 1 mm margin on the flat face too

# ---------------------------------------------------------------- B: budget (as written by the generator) - recomputed for the record
LKR = json.load(open(NPZ.replace('.npz', '.check.json'))).get('lid', {}).get('stop_key', {}) if __import__('os').path.exists(NPZ.replace('.npz', '.check.json')) else {}
ot_cap = LM.DEFAULTS['latch_over'] * (xk - Q[0]) / lever * 1000
B = SK.budget(SKP, ot_cap, lid=LKR.get('budget', {}).get('latch_order'), roll=S4.roll_loss(p) if V4 else 0.0)      # [fix4-stop2] edge-press roll loss
lo = (B.get('latch_order') or {}).get('F_lid_over_at_cap_N', {})
B['latch_order_ok'] = bool(lo and lo['min'] > B['worst_case']['F_ret_max'])
rep['B_budget'] = B
# [fix4-stop2] S1: the budget gap must be the TRUE gap - minimum vertical distance between the tip mesh (closed lid, cap at rest) and the scanned key
tipT = sv[sf[sp == SK.PART['tip']]]; tipP = np.concatenate([tipT.reshape(-1, 3), tipT.mean(1)]); kT_ = S4.key_tris('.')
tb_ = tipP[tipP[:, 2] < tipP[:, 2].min() + 0.0013]; zk_ = S4.zkey_at(kT_, tb_[:, :2]); okk_ = np.isfinite(zk_)
mesh_gap = float((tb_[okk_, 2] - zk_[okk_]).min()) * 1000 if okk_.any() else None
rep['B_budget']['true_gap_check'] = dict(budget_gap_mm=round(p['gap'] * 1000, 3), mesh_gap_mm=round(mesh_gap, 3) if mesh_gap is not None else None,
                                         ok=bool(mesh_gap is not None and abs(mesh_gap - p['gap'] * 1000) <= 0.05), samples=int(okk_.sum()))
# [fix4-stop2] S6 / QA 'stop.carrier x stop.tip': the rigid stem must never land on the TPE tip (it would drive the real key rigidly) and the
#   compliance spring must not go solid: room stem bottom -> tip top (meshes) and the spring's travel to block against the worst-case
#   compliance travel (cap on its stop + full lid over-travel, gap at its worst-case minimum)
cT_ = sv[sf[sp == SK.PART['carrier']]].reshape(-1, 3); rT_ = np.hypot(cT_[:, 0] - xk, cT_[:, 1] - yk); stem_bot_ = float(cT_[rT_ < p['stem_r'] + 1e-4, 2].min())
ctm_ = B['worst_case']['compliance_travel_max']; room_ = (stem_bot_ - float(tipP[:, 2].max())) * 1000; blk_ = (LKR.get('compliance_spring') or {}).get('travel_to_block_mm')
rep['B_budget']['stem_tip_room'] = dict(room_mm=round(room_, 2), compliance_travel_max_mm=ctm_, margin_mm=round(room_ - ctm_, 2), spring_travel_to_block_mm=blk_,
                                        spring_block_margin_mm=round(blk_ - ctm_, 2) if blk_ is not None else None,
                                        ok=bool(room_ - ctm_ >= 0.5 and (blk_ is None or blk_ >= ctm_)))

# ---------------------------------------------------------------- C: stroke simulation (y-slices)
PN = SK.NAME; ck = np.load('control.npz')
groups = {}                                                                    # name -> (V, F, kind) kind: 'mov' (cap stroke), 'arm_u', 'arm_l', 'lid' (lid-side), 'world'
for k, nm in PN.items():
    m = sp == k
    if not m.any(): continue
    if nm in ('cap', 'carrier', 'rod', 'tip', 'cspring'): groups['stop_' + nm] = (sv, sf[m], 'mov')
    elif nm == 'pin':        # front pins move with the carrier, rear pins are lid-side
        cf_ = sv[sf[m]].mean(1); fr_ = np.abs(cf_[:, 0] - xk) < 0.004
        groups['stop_pin_front'] = (sv, sf[m][fr_], 'mov'); groups['stop_pin_rear'] = (sv, sf[m][~fr_], 'lid')
    elif nm == 'arm':
        cf_ = sv[sf[m]].mean(1); zmid = 0.5 * (cf_[:, 2].max() + cf_[:, 2].min()); u_ = cf_[:, 2] > zmid
        groups['stop_arm_upper'] = (sv, sf[m][u_], 'arm_u'); groups['stop_arm_lower'] = (sv, sf[m][~u_], 'arm_l')
    elif nm in ('boot',): continue                                              # deforms (rolling boot)
    else: groups['stop_' + nm] = (sv, sf[m], 'lid')
C = V[F].mean(1); near = (np.abs(C[:, 0] - xk) < 0.09 + p['arm_L']) & (np.abs(C[:, 1] - yk) < 0.04)
groups['lidskin'] = (V, F[near & (lab == 4)], 'lid')
for k, nm in {v: k for k, v in LM.PM.items()}.items():
    m = d['kitm_part'] == k
    if m.any(): groups['kitm_' + nm] = (d['kitm_v'], d['kitm_f'][m], 'lid')
for k, nm in {v: k for k, v in LM.PF.items()}.items():
    m = d['kitf_part'] == k
    if m.any(): groups['f_' + nm] = (d['kitf_v'], d['kitf_f'][m], 'world')
groups['controls'] = (ck['v'], ck['f'], 'world'); groups['cover'] = (V, F[near & (lab != 4)], 'world')
if 'lights_halo_v' in d.files and len(d['lights_halo_f']): groups['halo_feed'] = (d['lights_halo_v'], d['lights_halo_f'], 'lid')      # [fix3-int] the halo's feed pipe (on the lid) vs the moving STOP parts
zu = float(z0 + p['arm_top'] - p['arm_h'] / 2); zl = zu - p['arm_d']; xP = xk - p['arm_L']
DESIGNED = {('stop_tip', 'controls'): 'tip presses the key (compliance spring)', ('stop_carrier', 'stop_halo'): 'down-stop at full stroke',
            ('stop_carrier', 'stop_socket'): '[fix4-stop2] down-stop: the shroud rim on the socket at full stroke', ('stop_carrier', 'stop_bush'): '[fix4-stop2] stem in the POM guide bush (0.15 running clearance in y)',
            ('stop_carrier', 'stop_bumper'): '[fix4-stop2] TPE lip seal on the stem', ('stop_arm_lower', 'stop_rspring'): '[fix4-stop2] return-spring leg under the lower link (the leg follows the link; static in the model)',
            ('stop_arm_upper', 'stop_bush'): '[fix4-stop2] tines under the bush / up-stop bumper',
            ('stop_arm_upper', 'stop_bumper'): 'up-stop at rest', ('stop_pin_front', 'stop_arm_upper'): 'pivot', ('stop_pin_front', 'stop_arm_lower'): 'pivot',
            ('stop_arm_upper', 'stop_pin_rear'): 'pivot', ('stop_arm_lower', 'stop_pin_rear'): 'pivot', ('stop_arm_upper', 'stop_rspring'): 'spring leg on the arm',
            ('stop_pin_front', 'stop_carrier'): 'pin in the stem lug', ('stop_rod', 'stop_carrier'): 'rod in its bushing', ('stop_cspring', 'stop_carrier'): 'spring seat',
            ('stop_cspring', 'stop_rod'): 'spring seat', ('stop_tip', 'stop_rod'): 'tip on the rod', ('stop_rod', 'stop_tip'): 'tip on the rod', ('stop_arm_upper', 'stop_bore'): 'up-stop through the 0.2 TPE bumper',
            ('stop_pin_front', 'stop_bumper'): 'front pin head 0.2 under the up-stop bumper (rigid with the arm that sits on it)', ('stop_pin_front', 'stop_bore'): 'front pin head under the up-stop (as above)'}
INTERNAL = {'stop_cap', 'stop_carrier', 'stop_rod', 'stop_tip', 'stop_cspring', 'stop_pin_front'}       # one moving assembly
ys = np.unique(np.round(np.r_[np.arange(-0.022, 0.0221, 0.0005), yk + np.array([-6.9, -6.3, -5.6, 5.6, 6.3, 6.9]) / 1000], 6))
key_top = lambda xx: None
svals = np.round(np.arange(0, S + 1e-9, 0.00025), 6)
def motion(kind, seg, s, th_lid):
    """seg (k, 2, 2) in (x, z), lid frame, cap stroke s -> world (lid rotated by th_lid about Q)."""
    th = np.arcsin(min(s / p['arm_L'], 1.0))
    if kind == 'mov': out = seg + np.array([-p['arm_L'] * (1 - np.cos(th)), -p['arm_L'] * np.sin(th)])
    elif kind == 'arm_u': out = rot(seg, np.array([xP, zu]), -np.degrees(th))
    elif kind == 'arm_l': out = rot(seg, np.array([xP, zl]), -np.degrees(th))
    else: out = seg
    return out if kind == 'world' or th_lid == 0 else rot(out, Q, th_lid)
def work(y):
    out = {}
    sl = {k: slice_y(v_, f_, y) for k, (v_, f_, kd) in groups.items()}; sl = {k: s_ for k, s_ in sl.items() if len(s_)}
    movs = [k for k in sl if groups[k][2] in ('mov', 'arm_u', 'arm_l')]
    for th_lid in (0.0, OVER):
        for s in svals:
            for a in movs:
                sa = motion(groups[a][2], sl[a], s, th_lid)
                for b in sl:
                    if b == a or (a in INTERNAL and b in INTERNAL) or (b in movs and b <= a): continue
                    if a in ('stop_tip', 'stop_rod') and b == 'controls': continue            # held by the key (compliance): analytic, part B
                    sb = motion(groups[b][2], sl[b], s if b in movs else 0.0, th_lid)
                    if max(0.0, max(sa[..., 0].min(), sb[..., 0].min()) - min(sa[..., 0].max(), sb[..., 0].max())) > 0.012: continue
                    if max(0.0, max(sa[..., 1].min(), sb[..., 1].min()) - min(sa[..., 1].max(), sb[..., 1].max())) > 0.012: continue
                    dd, hit = clearance(sa, sb); key = (a, b)
                    cur = out.setdefault(key, {}).get((th_lid, s))
                    if cur is None or dd < cur[0] or hit: out[key][(th_lid, s)] = (0.0 if hit else dd, hit or (cur[1] if cur else False), round(y * 1000, 2))
    return out
from multiprocessing import Pool
with Pool(int(__import__('os').environ.get('NW', 12))) as pool: RR = pool.map(work, ys, chunksize=1)
res = {}
for out in RR:
    for key, row in out.items():
        for kk, v in row.items():
            cur = res.setdefault(key, {}).get(kk)
            if cur is None or v[0] < cur[0] or v[1]: res[key][kk] = (min(v[0], cur[0]) if cur else v[0], v[1] or (cur[1] if cur else False), v[2] if (cur is None or v[0] <= cur[0]) else cur[2])
summary = {}; worst = (np.inf, None)
for key, row in res.items():
    mn = min(row.items(), key=lambda kv: kv[1][0]); cls = 'designed' if key in DESIGNED or (key[1], key[0]) in DESIGNED else 'check'
    summary['%s|%s' % key] = dict(min_mm=round(mn[1][0] * 1000, 3), at_lid_deg=round(mn[0][0], 3), at_stroke_mm=round(mn[0][1] * 1000, 2), y_mm=mn[1][2], hit=bool(any(v[1] for v in row.values())), cls=cls)
    if cls == 'check' and mn[1][0] < worst[0]: worst = (mn[1][0], '%s|%s' % key)
# analytic: stem bottom vs the key (the key is NOT pressed down - conservative), arms over the key, tip compliance
zk = float(SK.layout(SKP, lambda q: np.full(len(q), z0))['zkey'])
rep['C_stroke'] = dict(slices=int(len(ys)), stroke_steps=int(len(svals)), lid_angles=[0.0, round(OVER, 3)],
                       min_clearance_check_mm=round(worst[0] * 1000, 3), worst_pair=worst[1], ok_0_5mm=bool(worst[0] >= 0.0005),
                       pairs={k: v for k, v in sorted(summary.items(), key=lambda kv: kv[1]['min_mm']) if v['min_mm'] < 3.0 or v['cls'] == 'designed'})
# [fix4-stop2] S6: rigid STOP parts vs the real key in the WORST stack (nominal minimum over strokes and lid angles - stack_wc - |thermal low|)
wcs = B['worst_case']['stack_wc'] + abs(B['worst_case']['thermal'][0])
rep['C_stroke']['worst_stack_vs_key_mm'] = {k: dict(nominal=v['min_mm'], worst=round(v['min_mm'] - wcs, 2), at_stroke_mm=v['at_stroke_mm'], at_lid_deg=v['at_lid_deg'])
                                             for k, v in summary.items() if k.endswith('|controls') and v['cls'] == 'check'}
# [fix4-stop2] S2: grit under the skirt + edge-press roll: skirt over the socket / lid at full stroke >= 1.0 mm grit + r sin(1.5 deg)
sk_ = [v['min_mm'] for k, v in summary.items() if k.startswith('stop_cap|') and k.split('|')[1] in ('stop_socket', 'stop_halo', 'lidskin')]
rep['C_stroke']['grit_case'] = dict(skirt_min_over_socket_lid_mm=min(sk_) if sk_ else None, need_mm=round(1.0 + R * 1000 * np.sin(np.radians(1.5)), 2),
                                    ok=bool(sk_ and min(sk_) >= 1.0 + R * 1000 * np.sin(np.radians(1.5))))
# [fix4-stop2] G2 / G3: static lid-frame pass - lid-fixed STOP parts and the halo pipe against the lid moulding (closed lid), >= 0.5 except fusions
FUSED = {('stop_socket', 'kitm_underside'), ('stop_socket', 'lidskin'), ('stop_socket', 'kitm_rib'), ('stop_post', 'kitm_underside'), ('stop_post', 'kitm_rib'),
         ('stop_bracket', 'kitm_underside'), ('stop_pcover', 'kitm_underside'), ('stop_bore', 'kitm_underside'), ('stop_halo', 'kitm_underside'), ('halo_feed', 'stop_socket'),
         ('halo_feed', 'stop_post'), ('halo_feed', 'stop_halo'), ('stop_halo', 'lidskin'), ('stop_bush', 'kitm_underside')}
LIDF = ['kitm_underside', 'kitm_rib', 'lidskin', 'halo_feed']; STAT = [k for k in groups if k.startswith('stop_') and groups[k][2] == 'lid']
def work_static(y):
    sl = {k: slice_y(groups[k][0], groups[k][1], y) for k in STAT + LIDF}; out = {}
    for a in STAT + ['halo_feed']:
        for b in LIDF:
            if a == b or not len(sl[a]) or not len(sl[b]): continue
            dd, hit = clearance(sl[a], sl[b]); out[(a, b)] = (0.0 if hit else dd, hit)
    return out
with Pool(int(__import__('os').environ.get('NW', 12))) as pool: RS = pool.map(work_static, ys, chunksize=1)
stat = {}
for out in RS:
    for k, (dd, hit) in out.items():
        cur = stat.get(k); stat[k] = (min(dd, cur[0]) if cur else dd, hit or (cur[1] if cur else False))
stat_rep = {'%s|%s' % k: dict(min_mm=round(v[0] * 1000, 3), cross=bool(v[1]), cls='fused' if (k in FUSED or (k[1], k[0]) in FUSED) else 'check') for k, v in sorted(stat.items(), key=lambda kv: kv[1][0])}
# [fix4-int] closed lid moulding (fix4-hinge2 h4, npz lid_pinion): the STOP skin cut (r 24.9) is a bore wall through the whole plate, filled by the socket
# (2nd shot, -2.5 .. +0.3). The pipe drops from the ring's front point inside the socket, so its wall to the 1st-shot plate is the same 0.4 shadow-gap
# wall of socket material as the ring's (stop_halo|lidskin, fused 0.4): limit 0.35 (designed 0.4 - 0.05) for that pair instead of 0.5.
LIM_ = {'halo_feed|kitm_underside': 0.35} if 'lid_pinion' in d.files else {}
for k_, v_ in stat_rep.items():
    if k_ in LIM_ and v_['cls'] == 'check': v_.update(cls='designed', limit_mm=LIM_[k_], ok=bool(v_['min_mm'] >= LIM_[k_] and not v_['cross']), why='[fix4-int] pipe in the socket: 0.4 shadow-gap wall to the 1st-shot plate (bore wall of the h4 closed lid)')
chk_ = [v['min_mm'] for v in stat_rep.values() if v['cls'] == 'check']
rep['C2_static_lid_frame'] = dict(pairs={k: v for k, v in stat_rep.items() if v['min_mm'] < 3.0}, min_check_mm=min(chk_) if chk_ else None,
                                  ok_0_5mm=bool((not chk_ or min(chk_) >= 0.5) and all(v.get('ok', True) for v in stat_rep.values() if v['cls'] == 'designed')))      # [fix4-int] + designed limits

# ---------------------------------------------------------------- D: lid open: the exposed real key (fixed hardware)
K = SK.key_info('.'); Pk = K['P']; cv_, cf_, cl_ = ck['v'], ck['f'], ck['lab']
Pt = cv_[cf_]; n_ = np.cross(Pt[:, 1] - Pt[:, 0], Pt[:, 2] - Pt[:, 0]); a_ = 0.5 * np.linalg.norm(n_, axis=1); nzk = n_[:, 2] / np.maximum(2 * a_, 1e-30)
ku = (cl_ == 1) & (nzk > 0.7); zz = Pt.mean(1)[ku, 2]; aa = a_[ku] * nzk[ku]; o_ = np.argsort(-zz); kz20 = float(zz[o_][np.searchsorted(np.cumsum(aa[o_]) / aa.sum(), 0.2)])
khull = shapely.MultiPoint(Pk[:, :2]).convex_hull; kb = khull.buffer(0.015).difference(khull)
kc = d['kitf_v'][d['kitf_f']].mean(1); inb = shapely.contains_xy(kb, kc[:, 0], kc[:, 1]); kfp = d['kitf_part']
cc = Pt.mean(1); inb2 = shapely.contains_xy(kb, cc[:, 0], cc[:, 1]) & (cl_ != 1)
PFI = {v: k for k, v in LM.PF.items()}
intr = {PFI[int(k)]: round(float(d['kitf_v'][d['kitf_f'][inb & (kfp == k)]][:, :, 2].max()) * 1000, 2) for k in np.unique(kfp[inb])}
# STOP parts with the lid open: distance to the key
so = rot(sv[:, [0, 2]][:, None, :], Q, OPEN)[:, 0]; dmin = float(np.min(np.hypot(so[:, 0, None] - Pk[::7, 0][None], so[:, 1, None] - Pk[::7, 2][None]))) if len(sv) < 200000 else None
rep['D_lid_open_real_key'] = dict(key_minor_dimension_mm=round(min(float(np.ptp(np.array(khull.exterior.coords) @ np.array([np.cos(a), np.sin(a)]))) for a in ang) * 1000, 2),
    key_z_at_20pct_area_mm=round(kz20 * 1000, 2), panel_in_band_max_mm=round(float(cc[inb2, 2].max()) * 1000, 2), kit_in_band_max_mm=intr,
    rise_over_panel_mm=round((kz20 - float(cc[inb2, 2].max())) * 1000, 2), rise_over_well_floor_mm=round((kz20 - intr.get('well', 0) / 1000) * 1000, 2) if 'well' in intr else None,
    latch_housing_above_key_mm=round((intr.get('latch', 0) / 1000 - kz20) * 1000, 2) if 'latch' in intr else None,
    stop_parts_min_xz_distance_to_key_open_mm=round(dmin * 1000, 1) if dmin is not None else None)
rep['ok'] = bool(g['pass'] and B['worst_case']['always_actuates'] and B['worst_case']['no_false_stop'] and B['worst_case']['F_operate_max'] <= 30
                 and B['latch_order_ok'] and rep['C_stroke']['ok_0_5mm'])
if V4: rep['ok'] = bool(rep['ok'] and B['worst_case']['F_to_cap_stop_max'] <= 30 and B['worst_case']['F_on_real_key_max'] <= 25.0 and B['true_gap_check']['ok']      # [fix4-stop2]
                        and B['worst_case']['plunger_yield_before_switch_mm'] == 0 and B['stem_tip_room']['ok'] and all(v['worst'] >= 0.5 for v in rep['C_stroke']['worst_stack_vs_key_mm'].values())
                        and rep['C_stroke']['grit_case']['ok'] and rep['C2_static_lid_frame']['ok_0_5mm'])
print(json.dumps({k: v for k, v in rep.items() if k not in ('C_stroke', 'C2_static_lid_frame')}, indent=1)); print(json.dumps({k: v for k, v in rep['C_stroke'].items() if k != 'pairs'}))
if 'C2_static_lid_frame' in rep: print('C2 static lid frame:', json.dumps(rep['C2_static_lid_frame']))      # [fix4-stop2]
for k, v in rep['C_stroke']['pairs'].items(): print('  %-34s %s' % (k, v))
print('STOPCHECK OK:', rep['ok'])
if OUT: json.dump(rep, open(OUT, 'w'), indent=1)
