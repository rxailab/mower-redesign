# [fix4-hinge2] Lid balance of the geared hinge under tolerances (closed form; numpy only). Used by hingekit4.report (lid.h4 in the
# generator report) and by hingecalc.py. Units: SI inside, the report rounds to N m / N / mm / MPa.
#
# Drive: tongue sector gear (MIM, m 0.5, z_w 72 -> one tooth per 5 deg) -> pinion (z_p) on a shaft that carries, outboard, the spiral
# power spring in a closed steel cartridge and, inboard, a two-way rotary damper. Spring and damper act on the lid through the mesh in
# BOTH directions (no tape, no slack).
#
# Self-opening criterion (task / review M1): at every angle 0..OPEN, with the spring torque -10 % (vendor) and -5 % (relaxation), the
# rail friction at its upper bound, the drive efficiency at its worst and a total lid mass anywhere in M_RANGE, the net opening torque
#     net = k * eta * T(th) - G(th; M) - F_rail(th) [- detent entry torque near the stop]  >= NET_MIN.
# Rail friction: the lid / tongue of each hinge is a rigid body in a circular channel with radial play: the rails react the applied force
# (tooth force of that hinge's pinion + half the lid weight) with radial contact forces at the engaged range (one contact if the reaction
# direction lies inside the engaged range on the inner or outer rail, else two contacts at the range ends - the corner of the play region);
# friction torque = mu * sum(N_i * r_i). The f3 scalar model (sum of magnitudes at r_in) is reported alongside; the larger is used.
import numpy as np

g = 9.81
TOL = dict(k_vendor=0.90, k_relax=0.95, k_high=1.10, mu_nom=0.12, mu_ub=0.20, mu_dry=0.25, mu_low=0.06,
           mu_mesh=(0.10, 0.20), mu_journal=(0.12, 0.20), M_RANGE=(0.160, 0.180), NET_MIN=0.010)
DAMP_T = {'-10C': 2.2, '23C': 1.0, '60C': 0.55}          # rotary damper (silicone oil) torque factor vs 23 C, typical catalogue curves


def unit(a):
    a = np.radians(a); return np.array([np.cos(a), np.sin(a)])


def mesh_eta(z1, z2, mu, eps=1.6):
    """spur mesh efficiency (sliding loss, Buckingham / Niemann approximation)"""
    return 1.0 - 0.5 * np.pi * mu * (1.0 / z1 + 1.0 / z2) * eps


def contact_ratio(R1, R2, m, a, alpha=20.0):
    """transverse contact ratio of an external pair (pitch radii R1, R2, module m, addendum m) at centre distance a"""
    al = np.radians(alpha); rb1, rb2 = R1 * np.cos(al), R2 * np.cos(al); ra1, ra2 = R1 + m, R2 + m
    alw = np.arccos(min(1.0, (R1 + R2) * np.cos(al) / a))
    return float((np.sqrt(ra1 ** 2 - rb1 ** 2) + np.sqrt(ra2 ** 2 - rb2 ** 2) - a * np.sin(alw)) / (np.pi * m * np.cos(al)))


class Lid:
    """geometry + mass of the lid / hinge needed by the balance. th in deg."""
    def __init__(s, M0, com, com_extra, I, arc_a, guide_a, r_in, r_out, R_g, phi_p, alpha=20.0, L_edge=0.205, open_deg=75.0):
        s.M0, s.xc, s.zc = M0, com[0], com[1]; s.xs, s.zs = com_extra; s.I = I; s.a0, s.a1 = arc_a; s.g0, s.g1 = guide_a
        s.r_in, s.r_out, s.R_g, s.phi_p, s.alpha, s.L, s.OPEN = r_in, r_out, R_g, phi_p, alpha, L_edge, open_deg

    def G(s, th, M=None, mode='extra'):
        """gravity closing torque about Q; the mass delta sits at the STOP parts' COM ('extra') or scales the lid ('scale')"""
        t = np.radians(th); M = s.M0 if M is None else M; G0 = s.M0 * g * (s.xc * np.cos(t) - s.zc * np.sin(t))
        if mode == 'scale': return G0 * M / s.M0
        return G0 + (M - s.M0) * g * (s.xs * np.cos(t) - s.zs * np.sin(t))

    def Gmax(s, th, Ms): return np.max([s.G(th, M, md) for M in Ms for md in ('extra', 'scale')], axis=0)

    def Gmin(s, th, Ms): return np.min([s.G(th, M, md) for M in Ms for md in ('extra', 'scale')], axis=0)

    def rail(s, th, T_side, M, mu, sense=+1):
        """rail friction torque about Q (both hinges) and the contact forces of one hinge. T_side: drive torque one hinge delivers to the
        lid (opening sense > 0). sense: direction of motion (+1 opening) - friction opposes it (magnitude only is returned)."""
        er = unit(s.phi_p); et = np.array([-er[1], er[0]]); Ft = T_side / s.R_g
        F = Ft * et - abs(Ft) * np.tan(np.radians(s.alpha)) * er + np.array([0.0, -M * g / 2])
        lo, hi = max(s.a0 + th, s.g0), min(s.a1 + th, s.g1); R = -F; nR = float(np.hypot(*R))
        if nR < 1e-12: return 0.0, 0.0, []
        psi = np.degrees(np.arctan2(R[1], R[0]))
        inr = lambda p, l, h: ((p - l) % 360.0) <= (h - l) + 1e-9
        if inr(psi, lo, hi): C = [(nR, s.r_in, psi, 'inner')]
        elif inr(psi - 180.0, lo, hi): C = [(nR, s.r_out, psi - 180.0, 'outer')]
        else:
            if inr(psi, hi, lo + 180.0): d = [(unit(hi), s.r_in, hi, 'inner'), (unit(lo + 180.0), s.r_out, lo, 'outer')]
            else: d = [(unit(hi + 180.0), s.r_out, hi, 'outer'), (unit(lo), s.r_in, lo, 'inner')]
            N = np.linalg.solve(np.c_[d[0][0], d[1][0]], R); C = [(float(N[0]), d[0][1], d[0][2], d[0][3]), (float(N[1]), d[1][1], d[1][2], d[1][3])]
        vec = 2 * mu * sum(n * r for n, r, _, _ in C)
        sca = 2 * mu * (abs(Ft) * np.sqrt(1 + np.tan(np.radians(s.alpha)) ** 2) + M * g / 2) * s.r_in      # f3-style scalar (magnitudes)
        return max(vec, sca), vec, C


class Drive:
    def __init__(s, z_w, z_p, m, T0, T75, r_journal, detent_in=0.0, detent_out=0.0, detent_deg=1.5, c_Q=0.07):
        s.z_w, s.z_p, s.m = z_w, z_p, m; s.R_g, s.r_p = z_w * m / 2, z_p * m / 2; s.n = z_w / z_p
        s.T0, s.T75 = T0, T75; s.rj = r_journal; s.det_in, s.det_out, s.det_deg = detent_in, detent_out, detent_deg; s.c_Q = c_Q

    def T(s, th): return s.T0 + (s.T75 - s.T0) * np.asarray(th) / 75.0               # spring torque reflected to the lid (both, no loss)

    def eta(s, worst):
        mm, mj = (TOL['mu_mesh'][1], TOL['mu_journal'][1]) if worst else (TOL['mu_mesh'][0], TOL['mu_journal'][0])
        return mesh_eta(s.z_w, s.z_p, mm) * (1.0 - mj * s.rj / (s.r_p * np.cos(np.radians(20.0))))


def net_open(L, D, th, k, worst_eta, mu, M, mode='extra', detent=True):
    """net opening torque (N m) along th (deg) for one tolerance corner"""
    eta = D.eta(worst_eta); out = []
    for t in th:
        Td = k * eta * D.T(t); fr = L.rail(t, Td / 2, M, mu)[0]
        n = Td - L.G(t, M, mode) - fr
        if detent and t >= L.OPEN - D.det_deg - 1e-9: n -= D.det_in
        out.append(n)
    return np.array(out)


def analyse(L, D, spiral=None, th_step=1.0):
    """the ONE lid balance table: nominal, worst self-opening corners, hold-open, closing force, damper, detent"""
    th = np.arange(0.0, L.OPEN + 1e-9, th_step); M0 = L.M0; Ms = TOL['M_RANGE']; kw = TOL['k_vendor'] * TOL['k_relax']
    nom = net_open(L, D, th, 1.0, False, TOL['mu_nom'], M0)
    worst = {}
    for M in Ms:
        for md in ('extra', 'scale'):
            worst['M%.0f_%s' % (M * 1e3, md)] = net_open(L, D, th, kw, True, TOL['mu_ub'], M, md)
    W = np.min(list(worst.values()), axis=0); wk = min(worst, key=lambda k_: worst[k_].min())
    dry = np.min([net_open(L, D, th, kw, True, TOL['mu_dry'], M, md) for M in Ms for md in ('extra', 'scale')], axis=0)
    relax_only = net_open(L, D, th, TOL['k_relax'], False, TOL['mu_nom'], M0)
    # hold-open against wind at the stop: weakest spring, no friction / no gear loss credited, heaviest lid, minimum detent
    hold = kw * D.T(L.OPEN) - L.Gmax(L.OPEN, Ms) + 0.5 * D.det_out
    hold_nom = D.T(L.OPEN) - L.G(L.OPEN) + D.det_out
    wind = lambda Tq: float(np.sqrt(max(Tq, 0.0) / (0.5 * 1.2 * 1.2 * 0.033 * 0.100)))    # q * Cd 1.2 * A 0.033 m2 * lever 0.1 m (review m13)
    # closing force at the free edge (slow, quasi-static): strongest spring back-driven through the worst gear losses, upper-bound rail
    # friction, lightest lid; at the stop add the detent exit torque
    etab = D.eta(True); Fc = []; Fcn = []
    for t in th:
        Tb = TOL['k_high'] * D.T(t) / etab; fr = L.rail(t, Tb / 2, Ms[0], TOL['mu_ub'], -1)[0]
        Fc.append((Tb + fr - L.Gmin(t, [Ms[0]]) + (D.det_out if t >= L.OPEN - 1e-9 else 0.0)) / L.L)
        Tn = D.T(t) / D.eta(False); frn = L.rail(t, Tn / 2, M0, TOL['mu_nom'], -1)[0]
        Fcn.append((Tn + frn - L.G(t) + (D.det_out if t >= L.OPEN - 1e-9 else 0.0)) / L.L)
    Fc, Fcn = np.array(Fc), np.array(Fcn)
    # damper (two-way, linear in speed): self-opening time = int c / net dth (overdamped: I / c ~ 0.03 s)
    def t_open(netv, c): n_ = np.maximum(netv, 1e-4); return float(np.trapezoid(c / n_, np.radians(th)))
    dmp = dict(c_Q_Nms_per_rad=round(D.c_Q, 4), c_pinion_Nms_per_rad_per_side=float('%.4g' % (D.c_Q / 2 / D.n ** 2)),
               pinion_torque_Ncm_at_20rpm=round(D.c_Q / 2 / D.n ** 2 * (20 * 2 * np.pi / 60) * 100, 3),
               open_time_s={k_: round(t_open(nom, D.c_Q * f_), 2) for k_, f_ in DAMP_T.items()}, open_time_worst_s_23C=round(t_open(W, D.c_Q), 1),
               soft_close_Nm_at_60dps=round(D.c_Q * np.radians(60.0), 4), soft_close_edge_N_at_60dps=round(D.c_Q * np.radians(60.0) / L.L, 2),
               time_constant_I_over_c_s=round(L.I / D.c_Q, 4))
    # flick (LC2): the user drives the lid at 2.5 m/s at the edge; the damper torque (saturating at 3x its 20 rpm rating per catalogue
    # practice) and the spring set the tooth force; released early the lid stops within w * I / c
    w2 = 2.5 / L.L; Tdmp = min(D.c_Q * w2, 3 * D.c_Q * (20 * 2 * np.pi / 60) / D.n)      # lid torque, both dampers; each limited to 3 x its 20 rpm torque (slip / shear thinning)
    Ft_flick = (Tdmp / 2) / D.R_g
    lc = dict(LC1_on_stop_Nm=dict(nominal=round(float(D.T(L.OPEN) - L.G(L.OPEN)), 4), high=round(float(TOL['k_high'] * D.T(L.OPEN) - L.Gmin(L.OPEN, [Ms[0]])), 4)),
              LC2_flick=dict(w_rad_s=round(w2, 1), damper_Nm=round(Tdmp, 3), tooth_force_N=round(Ft_flick, 1), coast_after_release_deg=round(float(np.degrees(w2 * L.I / D.c_Q)), 1),
                             damper_limit_pinion_Ncm=round(3 * D.c_Q / 2 / D.n ** 2 * (20 * 2 * np.pi / 60) * 100, 2), KE_at_stop_if_pushed_through_J=round(0.5 * L.I * w2 ** 2, 4)))
    gear = dict(module_mm=D.m, z_tongue=D.z_w, z_pinion=D.z_p, ratio=round(D.n, 3), R_tongue_mm=round(D.R_g * 1e3, 3), r_pinion_mm=round(D.r_p * 1e3, 3),
                eta_nominal=round(D.eta(False), 4), eta_worst=round(D.eta(True), 4),
                tooth_force_N=dict(closed_nominal=round(D.T(0) / 2 / D.R_g, 1), open_nominal=round(D.T(L.OPEN) / 2 / D.R_g, 1), flick=round(Ft_flick, 1)))
    lewis = lambda F_, b_: F_ / (b_ * D.m * 0.30)                                        # Lewis, Y 0.30 (z 16, x +0.1) -> Pa
    gear['lewis_MPa_flick_b4'] = round(lewis(Ft_flick, 0.004) / 1e6, 1)
    tab = lambda a_: {str(int(round(t))): round(float(v), 4) for t, v in zip(th, a_) if abs(t % 5) < 1e-9}
    out = dict(model='hingebal.py (fix4-hinge2): rigid tongue in a channel with radial play, contacts at the engaged range; drive efficiency = '
                     'spur mesh x pinion journals; spring torque linear; detent entry torque in the last %.1f deg' % D.det_deg,
               tolerances=dict(TOL, worst_k=round(kw, 4)),
               spring_at_lid_Nm=tab(D.T(th)), gravity_nominal_Nm=tab(L.G(th)),
               net_open_nominal_Nm=tab(nom), net_open_nominal_min_Nm=round(float(nom.min()), 4),
               net_open_worst_Nm=tab(W), net_open_worst_min_Nm=round(float(W.min()), 4), net_open_worst_min_at_deg=float(th[np.argmin(W)]), worst_case=wk,
               net_open_relax5_only_min_Nm=round(float(relax_only.min()), 4),
               net_open_worst_mu_dry_0p25_min_Nm=round(float(dry.min()), 4), net_open_worst_mu_dry_0p25_min_at_deg=float(th[np.argmin(dry)]),
               self_opening_ok=bool(W.min() >= TOL['NET_MIN']),
               rail_friction_Nm_nominal=tab([L.rail(t, D.eta(False) * D.T(t) / 2, M0, TOL['mu_nom'])[0] for t in th]),
               hold_open=dict(worst_Nm=round(float(hold), 4), nominal_Nm=round(float(hold_nom), 4), wind_worst_m_s=round(wind(hold), 1), wind_nominal_m_s=round(wind(hold_nom), 1),
                              detent_exit_Nm=round(D.det_out, 4), detent_entry_Nm=round(D.det_in, 4), note='worst: spring x %.3f, heaviest lid, detent at half its value, no friction credited' % kw),
               closing_force_edge_N=dict(nominal=tab(Fcn), nominal_max=round(float(Fcn.max()), 2), worst_high=tab(Fc), worst_high_max=round(float(Fc.max()), 2),
                                         note='slow push at the free edge (%.0f mm from Q); + damper %.2f N at 60 deg/s' % (L.L * 1e3, D.c_Q * np.radians(60) / L.L)),
               damper=dmp, load_cases=lc, gear=gear)
    if spiral: out['spiral'] = spiral
    return out


def spiral_design(T_p0, T_p75, phi_stroke, b, t, r_in, r_a, E=190e9, sig_allow=950e6, over_phi=0.0):
    """spiral power spring in a cartridge, torques at the pinion (N m), stroke angle (rad). Returns the design table."""
    k = (T_p0 - T_p75) / phi_stroke; L_ = E * b * t ** 3 / (12 * k); pre = T_p75 / k
    s0 = 6 * (T_p0 + k * over_phi) / (b * t ** 2); s75 = 6 * T_p75 / (b * t ** 2)
    A = np.pi * (r_in ** 2 - r_a ** 2); fill = L_ * t / A
    n1 = (r_in - np.sqrt(max(r_in ** 2 - L_ * t / np.pi, 0.0))) / t; n2 = (np.sqrt(r_a ** 2 + L_ * t / np.pi) - r_a) / t
    avail = n2 - n1; used = (pre + phi_stroke + over_phi) / (2 * np.pi)
    return dict(strip_mm=[round(b * 1e3, 2), round(t * 1e3, 3)], length_mm=round(L_ * 1e3, 1), rate_Nmm_per_rad=round(k * 1e3, 3),
                torque_pinion_Nmm=[round(T_p0 * 1e3, 2), round(T_p75 * 1e3, 2)], preload_turns_open=round(pre / (2 * np.pi), 3),
                stroke_turns=round(phi_stroke / (2 * np.pi), 3), sigma_MPa=[round(s0 / 1e6), round(s75 / 1e6)], stress_ratio=round(s75 / s0, 3),
                sigma_allow_MPa=round(sig_allow / 1e6), cup_r_in_mm=round(r_in * 1e3, 2), arbor_r_mm=round(r_a * 1e3, 2), arbor_d_over_t=round(2 * r_a / t, 1),
                fill=round(fill, 3), turns_available=round(avail, 2), turns_used=round(used, 3), turns_used_pct=round(100 * used / avail, 1),
                coils_on_cup_relaxed=round(n1, 2), coils_on_arbor_fully_wound=round(n2, 2),
                ok=bool(s0 <= sig_allow and fill <= 0.62 and used / avail <= 0.55 and s75 / s0 >= 0.45 and 2 * r_a / t >= 15))


def detent(delta, lr=0.00175, clr=0.0003, E=5e6, b=0.0012, mu=0.20, n_lugs=4, R=0.017, ramp_in_deg=5.0):
    """TPE lips on both flanks of each cassette-plate slot, just behind the lug's seat at the open stop. Each lip protrudes delta into
    the lug's path (Winkler layer clr + delta thick on the steel flank). The lug (radius lr) enters over a long ramp (ramp_in_deg) and
    leaves over the lip crest, where the contact angle is set by its own radius: beta = acos((lr - delta) / lr). Returns (T_in, T_out,
    F_n per lip) in N m / N for the whole lid (n_lugs lugs, 2 lips each)."""
    c = 2 * np.sqrt(2 * lr * delta); F = E / (clr + delta) * b * (2.0 / 3.0) * c * delta
    phi = np.arctan(mu); b_out = np.arccos((lr - delta) / lr)
    T_out = n_lugs * 2 * F * np.tan(b_out + phi) * R; T_in = n_lugs * 2 * F * np.tan(np.radians(ramp_in_deg) + phi) * R
    return float(T_in), float(T_out), float(F)
