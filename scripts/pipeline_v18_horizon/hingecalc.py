# [fix3-hinge] Load cases, closed-form strength / spring / damper checks of the lid hinge, f2 vs f3 (FEA results merged from fea/*.json).
# usage: python hingecalc.py F2.check.json F3.check.json [OUT.json]       (units in the printout: N, mm, MPa, Nm, J)
import sys, json, glob, os, numpy as np
sys.path.insert(0, '.')
import lidmech as LM
R2, R3 = json.load(open(sys.argv[1]))['lid'], json.load(open(sys.argv[2]))['lid']
OUT = sys.argv[3] if len(sys.argv) > 3 else None
P2 = LM.params({k: v for k, v in json.load(open('f2.json'))['lid'].items()}); P3 = LM.params({k: v for k, v in json.load(open(os.environ.get('HC_PARAMS', 'f3.json')))['lid'].items()})      # [fix4-hinge2] HC_PARAMS=f4.json for the geared lid
H4 = R3.get('h4'); FT = 'f4' if H4 else 'f3'      # [fix4-hinge2] lid.h4: the geared drive (hingekit4 / hingebal) - the 'f3' keys below then describe the f4 lid
C3 = LM.hingekit.cfg(P3)
g = 9.81; L_EDGE = 0.205                    # free edge to Q (lid 202 + axis 3 mm behind the rear edge)

# ---------------------------------------------------------------- materials (60 C design temperature in the sun; dark well / pod)
MAT = {
 'MIM 17-4PH H900 (tongue, lugs, blade)': dict(Rp=970.0, src='MPIF Std 35 minimum YS 970 MPa (typ. 1090); ~-3% at 60 C ignored'),
 '1.4310 +C1000 strip (cassette plates)': dict(Rp=700.0, src='EN 10151 Rm 1000-1150; Rp0.2 >= 700 assumed - confirm on mill cert'),
 '1.4310 pin, hardened (f2 lug)': dict(Rp=1000.0, src='spring-hard pin'),
 'POM-C (f2 guide / f3 rail liners)': dict(Rp=45.0, Rc=55.0, src='tensile yield ~64 MPa at 23 C, ~45 at 60 C; compressive yield ~1.2x'),
 'PC/ABS (lid, keel, boss)': dict(Rp=40.0, Rc=48.0, src='tensile yield ~55 MPa at 23 C, ~40 at 60 C (datasheet curves)'),
 'PC/ABS well / pod moulding': dict(Rp=40.0, Rc=48.0, src='as lid'),
}
SF_REQ = 2.0

def lid_section(x_s, keel=None, blade=None, E_pl=2000.0, E_st=196e3):
    """Bending section of the lid across its full width at x_s mm from the rear edge (closed pose): skin 2.2, perimeter skirt rib, ring ribs
    (offsets 30 / 50) and diagonals, optional keels (2x, plastic) with the steel blade (transformed). Returns I (mm^4, PC/ABS units), z of the
    extreme bottom fibre below the neutral axis (mm) and of the extreme steel fibre."""
    W = 164.0; tl = 2.2; parts = [(W * tl, 0.0, W * tl ** 3 / 12)]                      # (area, centroid z (skin mid = 0), own I)
    def rib(b, h, n):                                                                  # rib below the skin (from the skin's underside)
        zc = -(tl / 2 + h / 2); return [(n * b * h, zc, n * b * h ** 3 / 12)]
    parts += rib(1.4, 7.5 - 2.2, 2)
    n_ring = sum(1 for o in (30.0, 50.0) if x_s > o + 2)
    parts += rib(1.2, 4.0, 2 * n_ring)
    if 30.0 < x_s < 50.0: parts += rib(1.2, 4.0, 2)                                    # corner diagonals between the rings
    zmin = -(tl / 2 + (7.5 - 2.2))
    if keel: parts += rib(keel[0], keel[1], 2); zmin = min(zmin, -(tl / 2 + keel[1]))
    zst = None
    if blade:                                                                          # steel blade, transformed width x E_st / E_pl
        b_, h_, top_ = blade; n_ = E_st / E_pl; zc = -(tl / 2 + top_ + h_ / 2); parts += [(2 * n_ * b_ * h_, zc, 2 * n_ * b_ * h_ ** 3 / 12)]
        zst = -(tl / 2 + top_ + h_)
    A = sum(p_[0] for p_ in parts); zna = sum(p_[0] * p_[1] for p_ in parts) / A
    I = sum(p_[2] + p_[0] * (p_[1] - zna) ** 2 for p_ in parts)
    return I, zmin - zna, (None if zst is None else zst - zna)

rep = {'materials': MAT, 'sf_required': SF_REQ}
# ---------------------------------------------------------------- geometry / mass
geo = {}
for nm, P, Rr in (('f2', P2, R2), ('f3', P3, R3)):
    R = P['arc_r']; t = P['arc_t']; w = P['arc_w']
    geo[nm] = dict(R_mm=R * 1e3, t_mm=t * 1e3, w_mm=w * 1e3, lug_d_mm=P['lug_d'] * 1e3, lugs_per_side=2 if (nm == 'f3' and C3['lug_both']) else 1,
                   engagement_open_deg=Rr['guide_engagement_deg']['open'], engagement_open_mm=round(np.radians(Rr['guide_engagement_deg']['open']) * R * 1e3, 1),
                   lever_ratio_edge_to_stop=round(L_EDGE / R, 1))
m2, cx2 = R2['mass_g'] / 1e3, R2['com_rel_Q_mm'][0] / 1e3
G2 = {int(k): v for k, v in R2['gravity_torque_Nm'].items()}
h3 = R3['h3'] if not H4 else dict(mass_new_g=R3['h3']['mass_new_g'], inertia_kgm2=H4['inertia_kgm2'], mass_g=H4['mass_g'], com_rel_Q_mm=H4['com_rel_Q_mm'], gravity_Nm=H4['gravity_Nm'])      # [fix4-hinge2] ONE lid report (m10)
m3 = h3['mass_g'] / 1e3; I3 = h3['inertia_kgm2']
# f2 inertia: same skin / ribs / striker; remove the f3 blades + keels (point masses at ~30 mm from Q)
I2 = I3 - (h3['mass_new_g']['blades_lugs'] + h3['mass_new_g']['keels']) / 1e3 * (0.030 ** 2 + 0.006 ** 2)
_sk3 = (R3.get('stop_key') or {}).get('mass_mesh')      # [fix3-int] the f2 lid has no STOP key parts either (the f3 inertia includes them via lidmech extra_mass)
if _sk3: I2 -= sum(v_['g'] / 1e3 * ((v_['x_mm'] - (R3['latch_x_mm'] - R3['lever_latch_mm'])) / 1e3) ** 2 for v_ in _sk3['parts_g'].values())      # [fix3-int]
if _sk3 and _sk3.get('skin_correction_g') and 'cap' in _sk3['parts_g']:      # [fix4-int] the lid skin the STOP socket replaces (negative mass at the key axis) belongs to the f2 lid
    I2 -= _sk3['skin_correction_g'] / 1e3 * ((_sk3['parts_g']['cap']['x_mm'] - (R3['latch_x_mm'] - R3['lever_latch_mm'])) / 1e3) ** 2      # [fix4-int]
rep['geometry'] = geo
rep['lid'] = {'f2': dict(mass_g=R2['mass_g'], com_mm=R2['com_rel_Q_mm'], I_kgm2=round(I2, 6), G0_Nm=G2[0], G75_Nm=G2[75]),
              'f3': dict(mass_g=h3['mass_g'], com_mm=h3['com_rel_Q_mm'], I_kgm2=I3, G0_Nm=h3['gravity_Nm']['0'], G75_Nm=h3['gravity_Nm']['75'])}

# ---------------------------------------------------------------- lid bending stiffness (first mode, root clamped) for the impact case
I_sec, zb, _ = lid_section(60.0)
EI = 2000.0 * I_sec * 1e-12 * 1e6            # N m^2 (E 2.0 GPa at 60 C; I in mm^4 -> m^4)
SKM3 = (R3.get('stop_key') or {}).get('mass_mesh')      # [fix3-int] the STOP key parts are concentrated masses on the lid (lidmech extra_mass): Rayleigh with the static
if SKM3:                                                 # [fix3-int] cantilever shape phi(xi) = xi^2 (3 - xi) / 2 -> equivalent uniform mass m_dist + sum m_i phi_i^2 / 0.236
    Qx3 = R3['latch_x_mm'] - R3['lever_latch_mm']; m_key = SKM3['g'] / 1e3
    phi_ = lambda x_mm: (lambda xi: xi ** 2 * (3 - xi) / 2)(min(max((x_mm - Qx3) / (L_EDGE * 1e3), 0.0), 1.0))
    m_eq = (m3 - m_key) + sum(v_['g'] / 1e3 * phi_(v_['x_mm']) ** 2 for v_ in SKM3['parts_g'].values()) / 0.236
    if SKM3.get('skin_correction_g') and 'cap' in SKM3['parts_g']: m_eq += SKM3['skin_correction_g'] / 1e3 * phi_(SKM3['parts_g']['cap']['x_mm']) ** 2 / 0.236      # [fix4-int] m_key is net of the skin the socket replaces: the concentrated masses must be too (was + 4.9 g at the key)
else: m_key = 0.0; m_eq = m3
m_line = m_eq / 0.202; w1 = 1.875 ** 2 * np.sqrt(EI / (m_line * 0.202 ** 4))
rep['lid_dynamics'] = dict(EI_Nm2=round(EI, 3), first_bending_Hz=round(w1 / 2 / np.pi, 1), stop_key_g=round(m_key * 1e3, 1), equivalent_uniform_mass_g=round(m_eq * 1e3, 1))      # [fix3-int]

# ---------------------------------------------------------------- spring / friction / damper (f2 vs f3)
def friction(P, F_band_side, beta_deg, m):
    """rail friction torque about Q (both hinges): the tape / band wrapped beta on the tongue presses it on the inner rail with 2 F sin(beta/2),
    plus the lid weight; mu 0.15 (POM on PVD steel), contact radius r_in."""
    r_in = P['arc_r'] - P['arc_t'] / 2; N = 2 * (2 * F_band_side * np.sin(np.radians(beta_deg) / 2)) + m * g
    return 0.15 * N * r_in
spr = {}
th = np.arange(0, 76, 5)
# f2: constant-force band 4 x 0.1 mm, 4.2 N, on a 9 mm drum
E_ss = 193e3; b2, t2, F2 = P2['band_w'] * 1e3, P2['band_t'] * 1e3, P2['spring_N']; Dd2 = 2 * P2['coil'][2] * 1e3
Rn_req = np.sqrt(E_ss * b2 * t2 ** 3 / (26.4 * F2)); s_req = E_ss * t2 / (2 * Rn_req)
Dn_drum = Dd2 / 1.17; F_drum = E_ss * b2 * t2 ** 3 / (26.4 * (Dn_drum / 2) ** 2); s_drum = E_ss * t2 / Dn_drum
cat = dict(source='Lee Spring constant-force catalogue, 0.10 x 6.35 mm 301 SS', LCF025=dict(cycles=2500, ID_mm=7.54, drum_mm=8.86, load_N=2.94),
           LCF130=dict(cycles=13000, ID_mm=11.13, drum_mm=13.54, load_N=1.42), LCF250=dict(cycles=25000, ID_mm=13.46, drum_mm=14.99, load_N=1.02))
# exit kink of the f2 band: arc tangent at exit_a (+theta) vs straight line to the coil
ea = P2['exit_a']; Q0 = np.zeros(2); rb2 = R2['spring']['band_r_mm']; Cd2 = np.array(R2['spring']['coil_centre_rel_Q_mm'])
tg = np.array([-np.sin(np.radians(ea)), np.cos(np.radians(ea))]); Pe = rb2 * np.array([np.cos(np.radians(ea)), np.sin(np.radians(ea))]); u2 = (Cd2 - Pe) / np.linalg.norm(Cd2 - Pe)
kink = np.degrees(np.arccos(np.clip(tg @ u2, -1, 1)))
T2 = 2 * F2 * rb2 / 1e3
G2a = np.interp(th, sorted(G2), [G2[k] for k in sorted(G2)])
fr2 = np.array([friction(P2, F2, (ea - (P2['arc_a'][0] + q)), m2) for q in th])
spr['f2'] = dict(type='constant-force band 4 x 0.1 mm, 301 SS, on a 9 mm drum', F_N=F2, torque_Nm=round(T2, 4),
                 required_natural_radius_mm=round(Rn_req, 2), bending_stress_for_4N2_MPa=round(s_req), on_9mm_drum=dict(Dn_mm=round(Dn_drum, 2), F_N=round(F_drum, 2), stress_MPa=round(s_drum)),
                 catalogue=cat, catalogue_force_per_mm_2500cyc=round(2.94 / 6.35, 3), asked_force_per_mm=round(F2 / b2, 3),
                 exit_kink_deg=round(kink, 1), exit_kink_strain=round(t2 / (2 * 0.2 + t2), 3),
                 net_open_Nm={int(q): round(float(T2 - G2a[i] - fr2[i]), 4) for i, q in enumerate(th)},
                 slam_energy_no_damper_J=round(float(np.trapezoid(T2 - G2a - fr2, np.radians(th))), 4))
if H4:      # [fix4-hinge2] geared drive: the balance of hingebal (tolerances, rail model with the tooth force, detent, two-way damper)
    B4 = H4['balance']; T3 = np.array([B4['spring_at_lid_Nm'][str(q)] for q in th]); G3 = np.array([H4['gravity_Nm'][str(q)] for q in th])
    fr3 = np.array([B4['rail_friction_Nm_nominal'][str(q)] for q in th]); net3 = np.array([B4['net_open_nominal_Nm'][str(q)] for q in th]); sp4 = B4['spiral']
    spr['f3'] = dict(design='f4 (fix4-hinge2): tongue sector gear m %.1f z %d -> pinion z %d -> spiral spring cartridge + two-way rotary damper (lid.h4, hingebal.py)' % (B4['gear']['module_mm'] * 1e3, B4['gear']['z_tongue'], B4['gear']['z_pinion']),
                     type='spiral power spring %.1f x %.2f x %.0f mm (1.4310) in a closed steel cartridge on the pinion shaft (ratio %.2f), one per hinge' % (sp4['strip_mm'][0], sp4['strip_mm'][1], sp4['length_mm'], B4['gear']['ratio']),
                     torque_Nm={int(q): round(float(T3[i]), 4) for i, q in enumerate(th)}, gravity_Nm={int(q): round(float(G3[i]), 4) for i, q in enumerate(th)},
                     friction_Nm={int(q): round(float(fr3[i]), 4) for i, q in enumerate(th)}, net_open_Nm={int(q): round(float(net3[i]), 4) for i, q in enumerate(th)},
                     net_min_Nm=round(float(net3.min()), 4), net_min_at_deg=int(th[np.argmin(net3)]),
                     net_open_worst_Nm=B4['net_open_worst_Nm'], net_worst_min_Nm=B4['net_open_worst_min_Nm'], net_worst_min_at_deg=B4['net_open_worst_min_at_deg'], worst_case=B4['worst_case'],
                     net_worst_mu_dry_0p25_min_Nm=B4['net_open_worst_mu_dry_0p25_min_Nm'], tolerances=B4['tolerances'], self_opening_ok=B4['self_opening_ok'],
                     net_min_relaxed_5pct_Nm=B4['net_open_relax5_only_min_Nm'], spiral=sp4, gear=B4['gear'], damper=B4['damper'], detent=B4['detent'], hold_open=B4['hold_open'],
                     cartridge=B4['cartridge'], load_cases_h4=B4['load_cases'],
                     spiral_check='sigma_max %d MPa = %.2f Rm, stress ratio %.2f (Rm ~1800 for 1.4310 +C1700 strip; <= 0.55 Rm with stress ratio 0.5 -> >= 1e5 cycles per spring-maker practice; confirm with the vendor)' % (sp4['sigma_MPa'][0], sp4['sigma_MPa'][0] / 1800, sp4['sigma_MPa'][1] / sp4['sigma_MPa'][0]),
                     slam_energy_no_damper_J=round(float(np.trapezoid(net3, np.radians(th))), 4),
                     slam_energy_note='only if the damper failed: the two-way damper limits the self-opening speed to net / c (< 1.5 rad/s) and damps closing',
                     closing_force_at_edge_N=B4['closing_force_edge_N'])
else:
    T3 = np.array([h3['spring_Nm'][str(q)] for q in th]); G3 = np.array([h3['gravity_Nm'][str(q)] for q in th]); Fb3 = T3 / (2 * h3['tape']['r_band_mm'] / 1e3)
    fr3 = np.array([friction(P3, Fb3[i], (P3['exit_a'] - (P3['arc_a'][0] + q)), m3) for i, q in enumerate(th)])
    net3 = T3 - G3 - fr3
    # damper: soft-open to 75 deg in ~2.5 s -> viscous coefficient about Q, per-side drum torque at the drum speed
    t_open = 2.5; om = np.radians(75) / t_open; c_Q = float(np.mean(net3)) / om
    n_ratio = h3['tape']['r_band_mm'] / h3['drum']['r_mm']; c_drum = c_Q / (2 * n_ratio ** 2); om_d = om * n_ratio
    spr['f3'] = dict(type='spiral power spring in the tape drum (1.4310 strip %.1f x %.2f mm, %.0f mm long) + UHMWPE tape 4 x 0.2 over an idler' % (h3['spiral']['strip_mm'][0], h3['spiral']['strip_mm'][1], h3['spiral']['length_mm']),
                     torque_Nm={int(q): round(float(T3[i]), 4) for i, q in enumerate(th)}, gravity_Nm={int(q): round(float(G3[i]), 4) for i, q in enumerate(th)},
                     friction_Nm={int(q): round(float(fr3[i]), 4) for i, q in enumerate(th)}, net_open_Nm={int(q): round(float(net3[i]), 4) for i, q in enumerate(th)},
                     net_min_Nm=round(float(net3.min()), 4), net_min_at_deg=int(th[np.argmin(net3)]),
                     tape_F_N=[round(float(Fb3[0]), 2), round(float(Fb3[-1]), 2)], tape_stress_MPa=round(float(Fb3[0]) / (4 * 0.2), 1),
                     spiral=h3['spiral'], drum=h3['drum'],
                     spiral_check='sigma_max %d MPa = %.2f Rm, stress ratio %.2f (Rm ~1800 for 1.4310 +C1700 strip; <= 0.55 Rm with stress ratio 0.5 -> >= 1e5 cycles per spring-maker practice; confirm with the vendor)' % (h3['spiral']['sigma_MPa'][0], h3['spiral']['sigma_MPa'][0] / 1800, h3['spiral']['sigma_MPa'][1] / h3['spiral']['sigma_MPa'][0]),      # [fix3-int] ratio
                     net_min_relaxed_5pct_Nm=round(float((0.95 * T3 - G3 - np.array([friction(P3, 0.95 * Fb3[i], (P3['exit_a'] - (P3['arc_a'][0] + q)), m3) for i, q in enumerate(th)])).min()), 4),      # [fix3-int]
                     damper=dict(target_open_time_s=t_open, mean_net_torque_Nm=round(float(np.mean(net3)), 4), c_about_Q_Nms=round(c_Q, 4), drum_speed_rpm=round(om_d * 60 / 2 / np.pi, 1),
                                 per_side_damper_torque_Ncm=round(c_drum * om_d * 100, 3), one_way='damps opening only; closing free'),
                     slam_energy_no_damper_J=round(float(np.trapezoid(net3, np.radians(th))), 4),
                     closing_force_at_edge_N={0: round(float((T3[0] - G3[0] + fr3[0]) / L_EDGE), 2), 75: round(float((T3[-1] - G3[-1] + fr3[-1]) / L_EDGE), 2)})
rep['spring'] = spr

# ---------------------------------------------------------------- load cases
v_tip = 2.5
KE2 = 0.5 * rep['lid']['f2']['I_kgm2'] * (v_tip / L_EDGE) ** 2; KE3 = 0.5 * I3 * (v_tip / L_EDGE) ** 2
def impact_M(KE, I, k_stop=686.0):
    """peak hinge moment of a lid that hits the stop with kinetic energy KE: lid-flex limited (first mode, w1) in series with the stop chain"""
    om0 = np.sqrt(2 * KE / I); k_lid = I * w1 ** 2; k = 1 / (1 / k_lid + 1 / k_stop); return om0 * np.sqrt(I * k), om0 * np.sqrt(I * k_stop)
LC = {
 'LC1': dict(name='spring holds the lid open on the stop (continuous, 60 C)', M_total_Nm={'f2': round(T2 - G2[75], 4), 'f3': round(float(T3[-1] - G3[-1]), 4)},
             rationale='static; creep of the stop interfaces at 60 C (sun on a dark well) is the criterion'),
 'LC2': dict(name='lid flung open by hand to the stop', v_tip_m_s=v_tip, KE_J={'f2': round(KE2, 4), 'f3': round(KE3, 4)},
             spring_slam_no_damper_J={'f2': spr['f2']['slam_energy_no_damper_J'], 'f3': spr['f3']['slam_energy_no_damper_J']},
             M_peak_Nm={'f2': [round(x, 2) for x in impact_M(max(KE2, spr['f2']['slam_energy_no_damper_J']), rep['lid']['f2']['I_kgm2'])],
                        'f3': [round(x, 2) for x in impact_M(KE3, I3)]},
             rationale='2.5 m/s at the free edge = a brisk wrist flick (lid ~12 rad/s); the lid is a 18 Hz flexible plate, so the hinge sees '
                       'M = w0*sqrt(I*k_eff) [lid-flex limited, rigid-lid bound in 2nd value]; both values for both hinges together, TPE pre-stage ignored'),
 'LC3': dict(name='hand pushing / leaning on the open lid, opening sense, at any point of the free edge', F_N=20.0,
             rationale='20 N ~ forearm + hand resting (2.2% of 80 kg = 17 N, Winter segment data) or a deliberate push to the stop; 4x a 20 m/s gust from behind '
                       '(q 245 Pa x Cd 1.2 x 0.033 m2 = 9.7 N at mid-lid = 4.8 N at the edge); ~ the elastic limit of the 2.2 mm PC/ABS lid itself at 60 C. '
                       'Corner load: ONE hinge carries the full moment (steel stops, flexible lid in twist). SF >= 2 to 60 C yield on every hinge part.',
             M_side_Nm={'centre': round(20 * L_EDGE / 2, 2), 'corner': round(20 * L_EDGE, 2)}),
 'LC4': dict(name='misuse: pulling the open lid past the stop (child / dragging the robot by the lid)', F_N=60.0,
             rationale='no-hazard case: report what yields first; the STOP key stays reachable even if the lid breaks off',
             M_side_Nm={'corner': round(60 * L_EDGE, 2)}),
 'excluded': 'lid closed on a finger (task scope); STOP-pad slap (latch / plunger issue, separate branch)',
}
rep['load_cases'] = LC

# ---------------------------------------------------------------- element checks at LC3 corner (M_side) and LC4
Mc = 20 * L_EDGE * 1e3                          # N.mm on one hinge
def chk(stress, allow, note=''):
    return dict(stress_MPa=round(float(stress), 1), allowable_yield_MPa=allow, SF=round(float(allow / max(stress, 1e-9)), 2), ok=bool(allow / max(stress, 1e-9) >= SF_REQ), note=note)
E = {}
for nm, P in (('f2', P2), ('f3', P3)):
    R = P['arc_r'] * 1e3; t = P['arc_t'] * 1e3; w = P['arc_w'] * 1e3; d = P['lug_d'] * 1e3; nl = geo[nm]['lugs_per_side']
    Fs = Mc / R; Fl = Fs / nl
    eng = P['guide_cheek'] * 1e3 * (C3['lug_eng'] if nm == 'f3' else 0.85); arm = P['clr_a'] * 1e3 + eng / 2
    e = {}
    e['stop_force_N'] = round(Fs, 1); e['force_per_lug_N'] = round(Fl, 1)
    e['lug_shear'] = chk(Fl / (np.pi * d ** 2 / 4) * 4 / 3, (970.0 if nm == 'f3' else 1000.0) * 0.577, 'max shear 4V/3A vs 0.577 Rp')
    e['lug_bending'] = chk(Fl * arm / (np.pi * d ** 3 / 32) * (1.5 if nm == 'f3' else 1.0), 970.0 if nm == 'f3' else 1000.0, 'Kt 1.5 at the R0.25 fillet (f3); f2 pressed pin: fit edge')
    e['lug_bearing_on_cheek'] = chk(Fl / (d * eng), 700.0 * 1.5 if nm == 'f3' else MAT['POM-C (f2 guide / f3 rail liners)']['Rc'],
                                    'projected bearing; f3 steel plate (bearing allowable 1.5 Rp), f2 POM-C compressive yield 60 C')
    Z = w * t ** 2 / 6; e['tongue_section_at_mouth'] = chk(Mc / Z, 970.0 if nm == 'f3' else 760.0, 'M / Z, %.0f x %.0f section (f2: 1.4310 1/2-hard Rp ~760)' % (w, t))
    if nm == 'f3' and H4:      # [fix4-hinge2] at the open stop the mouth section lies in the toothed zone: two full side lands + the middle band down to the tooth root circle
        G4c = LM.hingekit4.cfg4(P3)['gear']; bb_ = G4c['band'] * 1e3; hr_ = (G4c['z_w'] * G4c['m'] / 2 - 1.25 * G4c['m']) * 1e3 - (R - t / 2)      # band depth below the root
        A1_, A2_ = (w - bb_) * t, bb_ * hr_; z1_, z2_ = t / 2, hr_ / 2; zc_ = (A1_ * z1_ + A2_ * z2_) / (A1_ + A2_)
        I_ = (w - bb_) * t ** 3 / 12 + A1_ * (z1_ - zc_) ** 2 + bb_ * hr_ ** 3 / 12 + A2_ * (z2_ - zc_) ** 2; Zt_ = I_ / max(zc_, t - zc_); Kt_ = 1.7
        e['tongue_section_toothed'] = chk(Kt_ * Mc / Zt_, 970.0, 'band %.1f wide cut to the root circle (depth %.2f of %.1f): Z %.1f mm3 (full %.1f) x Kt %.1f (tooth-root fillet 0.38 m, periodic notch)' % (bb_, hr_, t, Zt_, Z, Kt_))
    if nm == 'f2':
        Le = np.radians(P['arc_a'][1] - P['web_a']) * R                               # tongue length inside the boss wedge
        p_root = 6 * Mc / (w * Le ** 2)
        e['root_in_boss'] = chk(p_root, MAT['PC/ABS (lid, keel, boss)']['Rc'], 'tongue end embedded %.1f mm in the PC/ABS wedge: p = 6M/(w Le^2) (embedded-post)' % Le)
        e['root_capacity_Nm_at_SF1'] = round(MAT['PC/ABS (lid, keel, boss)']['Rc'] * w * Le ** 2 / 6 / 1e3, 3)
    E[nm] = e
rep['elements_LC3_corner'] = E
if H4:      # [fix4-hinge2] gear teeth (MIM 17-4PH H900 both): Lewis bending and Hertz contact at the largest tooth force (spring x 1.1 back-driven + damper slip torque)
    B4 = H4['balance']; G4c = LM.hingekit4.cfg4(P3)['gear']; m_ = G4c['m'] * 1e3; zw_, zp_ = G4c['z_w'], G4c['z_p']; Rw_, rp_ = zw_ * m_ / 2, zp_ * m_ / 2
    Tsp_ = 1.10 * B4['spring_at_lid_Nm']['0'] / B4['gear']['eta_worst']; Tdm_ = B4['load_cases']['LC2_flick']['damper_Nm']; Ft_ = (Tsp_ + Tdm_) / 2 / (Rw_ / 1e3)
    Yw_, Yp_ = 0.42, 0.30; Ze_, Zh_ = 189.8, 2.49; u_ = zw_ / zp_
    sH_ = Ze_ * Zh_ * np.sqrt(Ft_ / (G4c['pinion_w'] * 1e3 * 2 * rp_) * (u_ + 1) / u_)
    rep['gear_teeth'] = dict(tooth_force_N=round(Ft_, 1), note='per hinge: 1.1 x spring at closed / worst efficiency + the damper slip torque (3 x rated) of a slam',
                             tongue_lewis=chk(Ft_ / (G4c['band'] * 1e3 * m_ * Yw_), 970.0, 'Lewis Y %.2f (z %d), face %.1f' % (Yw_, zw_, G4c['band'] * 1e3)),
                             pinion_lewis=chk(Ft_ / (G4c['pinion_w'] * 1e3 * m_ * Yp_), 970.0, 'Lewis Y %.2f (z %d, x +0.1), face %.1f' % (Yp_, zp_, G4c['pinion_w'] * 1e3)),
                             hertz=chk(sH_, 2.0 * 1000.0, 'ISO 6336 form, ZE 189.8, ZH 2.49; allowable ~1000 MPa contact (17-4PH H900, ~35 HRC; low cycle count: 2e4 openings x 15 teeth)'))

# lid plate (the load path in front of the hinge) and well -> cover
Fe = 20.0
lid = {}
S3 = C3['spine']; sx0, sx1 = S3['x0'] * 1e3 - 3, S3['x1'] * 1e3 - 3                    # spine rib in rear-edge coordinates
sh = lambda xs_: float(np.interp(xs_, [sx0, sx1 - S3['taper'] * 1e3, sx1], [S3['h'][0] * 1e3, S3['h'][1] * 1e3, 0.5]))
kx1 = C3['keel']['x1'] * 1e3 - 3 - C3['keel']['taper'] * 1e3
for nm, xs_, keel, blade in (('f2 at x 40 (from the rear edge)', 40.0, None, None), ('f2 at x 60', 60.0, None, None), ('f2 at x 95', 95.0, None, None),
                             ('f3 at x 40 (keel + blade)', 40.0, (11.0, 6.8), (8.0, 4.5, 0.2)), ('f3 at x %.0f (keel end, spine starts)' % (kx1 + 12), kx1 + 12, (S3['t'] * 1e3, sh(kx1 + 12)), None),
                             ('f3 at x 95 (spine)', 95.0, (S3['t'] * 1e3, sh(95.0)), None), ('f3 at x %.0f (spine end)' % sx1, sx1, None, None)):
    I_, zb_, zs_ = lid_section(xs_, keel, blade); Mx = Fe * (L_EDGE * 1e3 - xs_ - 3.0); s_ = Mx * abs(zb_) / I_
    lid[nm] = chk(s_, MAT['PC/ABS (lid, keel, boss)']['Rp'], 'rib tips in tension, LC3 20 N (full-width section carries F x lever)')
    lid[nm]['yield_at_edge_load_N'] = round(Fe * MAT['PC/ABS (lid, keel, boss)']['Rp'] / s_, 1)
rep['lid_plate_LC3'] = lid
# f3 lid-side root (blade in keel): bearing couple, blade section, keel floor / walls (LC3 corner on one hinge); FEA load split if available
fl = os.path.join('fea', FT + '_tongue_20N.json')      # [fix4-hinge2] FT
if os.path.exists(fl) and 'load' in json.load(open(fl)):
    ld = json.load(open(fl))['load']; P1, P2 = ld['skin_bearing_N'], ld['keel_bearing_N']; x1, x2 = ld['x_bearing_mm']; p1, p2 = ld['p_skin_MPa'], ld['p_keel_MPa']
else:
    x1, x2 = 16.5, 31.0; P1 = (Mc - x2 * 20) / (x2 - x1); P2 = P1 + 20; p1, p2 = P1 / 44.0, P2 / 128.0
h21 = 21.0 * np.tan(np.radians(-P3['web_a'])) - 3.2 - 0.3; Mb = P2 * (x2 - 21.0)
root = dict(skin_bearing_N=round(P1, 1), keel_bearing_N=round(P2, 1), lever_mm=round(x2 - x1, 1),
            skin_bearing=chk(p1, MAT['PC/ABS (lid, keel, boss)']['Rc'], 'skin / boss on the root top'),
            keel_floor_bearing=chk(p2, MAT['PC/ABS (lid, keel, boss)']['Rc'], 'keel floor under the blade deep part'),
            keel_floor_bending=chk(p2 * 8.0 ** 2 / (2 * 2.0 ** 2), MAT['PC/ABS (lid, keel, boss)']['Rp'], '2.0 mm floor spanning the 8 mm blade between the 1.5 mm keel walls (fixed-fixed strip)'),
            keel_wall_shear=chk(P2 / (2 * 1.5 * 14.0), 0.5 * MAT['PC/ABS (lid, keel, boss)']['Rp'], 'two 1.5 mm walls over the 14 mm deep zone carry the floor load into the skin'),
            blade_section_x21=chk(Mb / (8.0 * h21 ** 2 / 6), 970.0, 'blade depth %.1f mm between web plane and skin at x 21 (M = P2 x lever to the keel bearing)' % h21))
rep['root_f3_LC3'] = root
# LC4 (60 N corner): order of failure (stresses scale x3 from LC3)
k4 = 3.0
rep['LC4_failure_order'] = sorted([(nm_, round(v['allowable_yield_MPa'] / (k4 * v['stress_MPa']), 2)) for nm_, v in
    [('lid plate in front of the spine end', lid['f3 at x %.0f (spine end)' % sx1]), ('lid plate at the keel end', lid['f3 at x %.0f (keel end, spine starts)' % (kx1 + 12)]),
     ('tongue section at the mouth', E['f3']['tongue_section_at_mouth']), ('lug bending', E['f3']['lug_bending']), ('lug bearing on the cassette plate', E['f3']['lug_bearing_on_cheek']),
     ('blade section x21', root['blade_section_x21']), ('skin bearing on the root', root['skin_bearing']), ('keel floor bending', root['keel_floor_bending'])]], key=lambda q: q[1])
M_tot = Fe * L_EDGE
rep['well_to_cover_LC3'] = dict(moment_Nm=round(M_tot, 2), rear_stake_pull_N=round(M_tot / 0.1515, 1), per_stake_N=round(M_tot / 0.1515 / 2, 1),
                                stake_capacity_N=70.0, note='well tray bears on the cover along its front flange (x +54), rear side stakes (x -97.5) hold it: lever 151.5 mm; '
                                                            'stake d1.5 PC/ABS-ASA, head d3: tension 1.77 mm2 x 40 MPa = 71 N')
rep['cassette_to_pod_LC3'] = dict(lever_mm=39.0, bearing_force_N=round(Mc / 39.0, 1), p_MPa=round(Mc / 39.0 / 48.0, 2), note='cassette ends bear on the pod / pocket walls (6 x 8 mm faces)')

# Hertz: f3 lug on the conformal (R lug + 0.3) steel slot end, after the TPE pad has bottomed (line contact over the engaged 1.2 mm)
Ee = 196e3 / (2 * (1 - 0.28 ** 2)); r1 = P3['lug_d'] * 1e3 / 2; r2 = r1 + P3['lug_clr'] * 1e3; Reff = r1 * r2 / (r2 - r1)
q = E['f3']['force_per_lug_N'] / (P3['guide_cheek'] * 1e3 * C3['lug_eng']); pH = np.sqrt(q * Ee / (np.pi * Reff)); pH_flat = np.sqrt(q * Ee / (np.pi * r1))
rep['hertz_f3_lug_on_plate_LC3'] = dict(line_load_N_per_mm=round(q, 1), R_eff_mm=round(Reff, 2), p_max_MPa=round(pH, 0), tau_max_MPa=round(0.30 * pH, 0),
                                        tau_allow_MPa=round(0.577 * 700.0, 0), SF=round(0.577 * 700.0 / (0.30 * pH), 2),
                                        flat_end_for_comparison_p_max_MPa=round(pH_flat, 0), note='conformal semicircular slot end (hingekit.slot3) instead of the flat radial end')
# FEA results + safety factors (node-projected von Mises vs yield; quadrature-point maxima reported alongside)
fea = {}
for f in sorted(glob.glob('fea/*.json')):
    fea[os.path.basename(f)[:-5]] = json.load(open(f))
rep['fea'] = fea
sfe = {}
TF_ = FT + '_tongue_20N' if (FT + '_tongue_20N') in fea else 'f3_tongue_20N'      # [fix4-hinge2]
if TF_ in fea:
    for k_, v_ in fea[TF_]['regions'].items():
        sfe['f3 ' + k_] = dict(vm_node_MPa=v_['vm_node_max_MPa'], vm_qp_MPa=v_['vm_qp_max_MPa'], SF_node=round(970.0 / v_['vm_node_max_MPa'], 2), SF_qp=round(970.0 / v_['vm_qp_max_MPa'], 2))
        if 'full_depth' in k_: sfe['f3 ' + k_].update(Kt=1.7, SF_node_with_Kt=round(970.0 / (1.7 * v_['vm_node_max_MPa']), 2))      # [fix4-hinge2] teeth not modelled: tooth-root fillet notch
        if 'runouts' in k_: sfe['f3 ' + k_].update(note='peak at the floor exit corner (138 deg, in the FEA geometry, no tooth space there); with a 1.3 notch factor SF %.2f' % (970.0 / (1.3 * v_['vm_node_max_MPa'])))      # [fix4-hinge2]
    for r_ in fea[TF_]['rails']:
        sfe['f3 rail ' + r_['rail']] = dict(force_N=r_['force_N'], p_max_MPa=r_['p_max_MPa'], SF=round(MAT['POM-C (f2 guide / f3 rail liners)']['Rc'] / max(r_['p_max_MPa'], 1e-9), 2))
CF_ = FT + '_cheek_20N' if (FT + '_cheek_20N') in fea else 'f3_cheek_20N'      # [fix4-hinge2]
if CF_ in fea:
    c_ = fea[CF_]; sfe['f3 cassette plate (slot end)'] = dict(vm_MPa=c_['vm_max_MPa'], SF=round(700.0 / c_['vm_max_MPa'], 2))
if 'f2_cheek_20N' in fea:
    c_ = fea['f2_cheek_20N']; sfe['f2 POM cheek (slot end)'] = dict(vm_MPa=c_['vm_max_MPa'], min_principal_MPa=c_['min_principal_MPa'], SF=round(MAT['POM-C (f2 guide / f3 rail liners)']['Rc'] / abs(c_['min_principal_MPa']), 2))
if 'f2_tongue_20N' in fea:
    for k_, v_ in fea['f2_tongue_20N']['regions'].items():
        sfe['f2 ' + k_] = dict(vm_node_MPa=v_['vm_node_max_MPa'], vm_qp_MPa=v_['vm_qp_max_MPa'], SF_node=round((1000.0 if 'lug' in k_ else 760.0) / v_['vm_node_max_MPa'], 2))
rep['fea_SF_LC3'] = sfe
print(json.dumps(rep, indent=1, default=float))
if OUT: json.dump(rep, open(OUT, 'w'), indent=1, default=float)
