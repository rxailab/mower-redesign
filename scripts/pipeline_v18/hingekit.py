# [fix3-hinge] Open-stop / root / spring kit of the v18 arc-track lid hinge (geometry called from lidmech.build, numbers in hingecalc.py).
#
# What it changes (all guarded by lid.h3 in the parameters; without it lidmech builds the f2 hinge unchanged):
#  - STOP: the 2 mm pin pressed into one side of the tongue (bearing 1.1 mm deep in a 1.3 mm POM cheek) becomes TWO integral MIM lugs
#    (d 3.5, R0.25 root fillet), one on each side face, running in arc slots of BOTH cheeks. The cheeks are the stainless side plates of a
#    guide cassette (1.4310, 1.3 mm) that carries the POM rail liners; a 1.2 mm TPE pad in each slot end is the soft first stage.
#  - TONGUE: MIM 17-4PH H900, radial section 3 -> 4 mm (lid.arc_t), PVD dark.
#  - ROOT: the tongue root no longer ends 1.5 mm inside a 4 mm plastic wedge. It carries a BLADE (same MIM part, part 'blade') that runs
#    forward under the lid skin; in front of the pod it drops into a deep KEEL (the lid's hinge boss, part 'boss') that ends in ring rib 2.
#    The lid moment enters the steel as a couple: skin bearing on the root, keel floor under the blade's deep end (lever ~24 mm).
#  - SPRING: the 4.2 N 'constant-force' band (0.1 x 4 mm on a 9 mm drum = 3.7 GPa bending stress, and a ~100 deg kink over the exit
#    edge) becomes a flat UHMWPE tape that only transmits force: it wraps the tongue (constant lever), leaves the guide tangentially at
#    exit_a, turns over an idler pulley (part 'idler') and winds onto a drum with a spiral power spring inside (torque falls with opening,
#    matched to the lid's gravity moment); the one-way rotary damper stays on the drum axle.
import numpy as np, shapely
from shapely.geometry import Polygon, Point, LineString
from meshkit import lathe, tube, stack, sweep, rrect

PM3 = dict(blade=31)                       # kitm part ids of this branch (31+)
PF3 = dict(idler=41, pinion=42, cartridge=43)      # kitf part ids of this branch (41+); [fix4-hinge2] pinion 42, cartridge 43 (hingekit4)
DESIGNED3 = {('tongue', 'f_idler'): '[fix3-hinge] tape idler: 0.4 over the tongue face, the tape runs between (designed running clearance)'}
DEF3 = dict(lug_both=True, lug_fillet=0.00025, lug_eng=0.92, cheek_metal=True, rail_split=[-9.0, 13.0],
            blade=dict(x1=0.040, z_deep=-0.0080, drop_clr=0.0008, top=0.0002, fillet=0.0010),
            keel=dict(w=0.011, x1=0.068, z_deep=-0.0100, taper=0.012, rib=0.004),
            spine=dict(t=0.0015, x0=0.062, x1=0.122, h=[0.008, 0.005], taper=0.012),
            idler=dict(r=0.003, w=0.0044, pin_r=0.00075, dz=0.0098),
            drum=dict(r=0.005, w=0.0055, flange=0.0005),
            spring=dict(T0=0.165, T75=0.075, strip=[0.0045, 0.00018], arbor_r=0.0016, turns=7.0))


def cfg(p):
    H = p.get('h3')
    if not H: return None
    C = {k: (dict(v, **H.get(k, {})) if isinstance(v, dict) else H.get(k, v)) for k, v in DEF3.items()}
    return C


def setup(p, Q, rbnd):
    """Place the idler and the drum for the tape (closed pose, x-z plane). Overrides p['coil'] so that lidmech's damper box follows."""
    C = cfg(p); I, D = C['idler'], C['drum']; bt = p['band_t']; ea = p['exit_a']; e = np.array([np.cos(np.radians(ea)), np.sin(np.radians(ea))])
    Ci = Q + (rbnd + bt / 2 + I['r']) * e                                       # idler tangent to the tape's outer face at the exit
    ri_, rd_ = I['r'] + bt / 2, D['r'] + bt / 2                                  # tape centre radius on idler / drum
    Cd = Ci + np.array([ri_ - rd_, -I['dz']])                                    # tape leaves the idler's front vertically, lands on the drum's front
    v = Cd - Q; p['coil'] = [float(np.hypot(*v)), float(np.degrees(np.arctan2(v[1], v[0]))), D['r']]
    C['Ci'], C['Cd'] = Ci, Cd
    return C


def tape_path(C, Q, rbnd, a_from, ea, n=4):
    """Tape centre line (closed pose): wrap on the tongue from the tail pin to the exit, wrap the idler clockwise to its front point,
    straight down to the drum's front point, then 110 deg wound on the drum."""
    Ci, Cd = C['Ci'], C['Cd']; bt = 0.0002; ri_, rd_ = C['idler']['r'] + bt / 2, C['drum']['r'] + bt / 2
    a = np.radians(np.linspace(a_from, ea, max(8, int(abs(ea - a_from) / 2))))
    P = [np.c_[Q[0] + rbnd * np.cos(a), Q[1] + rbnd * np.sin(a)]]
    b = np.radians(np.linspace(ea + 180.0, 0.0, 40))[1:]; P.append(np.c_[Ci[0] + ri_ * np.cos(b), Ci[1] + ri_ * np.sin(b)])
    P.append(np.linspace(Ci + [ri_, 0], Cd + [rd_, 0], 8)[1:-1])
    c = np.radians(np.linspace(0.0, -110.0, 30)); P.append(np.c_[Cd[0] + rd_ * np.cos(c), Cd[1] + rd_ * np.sin(c)])
    return np.concatenate(P)


def lugs(KM, p, Q, R, sg, y0, w, ca, wch, K_ARM, part, dirv):
    """Two integral lugs (one per side face), d lug_d, root fillet in the 0.3 axial gap, engaging lug_eng of the cheek thickness."""
    C = cfg(p); lr = p['lug_d'] / 2; f = C['lug_fillet']; lp = Q + R * dirv(p['lug_a'])
    L = ca + wch * C['lug_eng'] + 0.0002                                        # from 0.2 mm inside the side face to the tip
    q = np.linspace(0, np.pi / 2, 5)
    prof = [(0.0, 0.0), (0.0, lr + f)] + [(0.0002 + f * (1 - np.cos(t)), lr + f * (1 - np.sin(t))) for t in q[1:]] + [(L - 0.0002, lr), (L, lr - 0.0002), (L, 0.0)]
    for s in ((sg, -sg) if C['lug_both'] else (sg,)):
        lv_, lf_ = lathe((lp[0], y0 + s * (w / 2 - 0.0002), lp[1]), (0, s, 0), prof, 28)
        if p.get('h4'): import prismkit; lv_, lf_ = prismkit.weld_clean(lv_, lf_)      # [fix4-hinge2] G5: the lathe's pole points welded (closed lug)
        KM.add(lv_, lf_, K_ARM, part)


def slot3(Q, R, rs, as0, as1, sector):
    """Arc slot whose ends are semicircles of radius rs (conformal with the lug + clearance): line contact -> conformal contact."""
    hs = np.degrees(rs / R); c = lambda a: Point(*(Q + R * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]))).buffer(rs, quad_segs=24)
    return sector(Q, R - rs, R + rs, as0 + hs, as1 - hs).union(c(as0 + hs)).union(c(as1 - hs))


def pad3(Q, R, lr, slot, a_open, as1, sector):
    """TPE first-stage pad: the crescent of the slot end in front of the lug at the open angle (the lug lands on it at open_deg and bottoms on
    the steel end after the pad's compression)."""
    lc = Q + R * np.array([np.cos(np.radians(a_open)), np.sin(np.radians(a_open))])
    pad = slot.intersection(sector(Q, 0.0, R + 0.01, a_open, as1 + 1.0)).difference(Point(*lc).buffer(lr, quad_segs=24))
    return max(shapely.get_parts(pad.buffer(-0.00015).buffer(0.00015, quad_segs=8)), key=lambda g_: g_.area)      # no slivers beside the lug


def blade_keel(p, Q, zu, y0, xf_, ra, wa):
    """Profiles (x-z, world) of the steel root blade and of the plastic keel (the lid's hinge boss). xf_: front face of the hinge pod."""
    C = cfg(p); B, Kc = C['blade'], C['keel']; ta = np.tan(np.radians(wa)); zw = lambda x: Q[1] + (x - Q[0]) * ta
    r_in = p['arc_r'] - p['arc_t'] / 2
    xa = Q[0] + ra * np.cos(np.radians(wa)); xd = xf_ + B['drop_clr']; x1k = Q[0] + Kc['x1']; x1b = Q[0] + B['x1']
    zkd = Q[1] + Kc['z_deep']; zbd = Q[1] + B['z_deep']
    xs = np.linspace(xa, x1k, 40); top = [(x, zu(x, y0) + 0.0003) for x in xs]
    zr = zu(x1k, y0) - Kc['rib']
    keel = Polygon(top + [(x1k, zr), (x1k - Kc['taper'], zkd), (xd + 0.002, zkd), (xd, zw(xd)), (xa, zw(xa))])
    keel = keel.buffer(-0.0006, join_style=1).buffer(0.0006, quad_segs=6)
    xb0 = Q[0] + r_in * np.cos(np.radians(wa)) - 0.0003; xs = np.linspace(xb0, x1b, 30)
    blade = Polygon([(x, zu(x, y0) - B['top']) for x in xs] + [(x1b, zbd), (xd + 0.0025, zbd), (xd + 0.0005, zw(xd + 0.0005) + 0.0001), (xb0, zw(xb0) + 0.0001)])
    blade = blade.buffer(-0.0003, join_style=1).buffer(0.0003, quad_segs=4)
    fr = B.get('fillet', 0.0)
    if fr > 0:                                                                   # R fr fillet where the tongue's outer edge meets the blade underside
        R_, t_ = p['arc_r'], p['arc_t']; a0_, a1_ = p['arc_a']
        arc = Polygon(np.r_[[[Q[0] + (R_ + t_ / 2) * np.cos(x), Q[1] + (R_ + t_ / 2) * np.sin(x)] for x in np.radians(np.linspace(a0_ * 0 + wa - 25, a1_, 60))],
                            [[Q[0] + (R_ - t_ / 2) * np.cos(x), Q[1] + (R_ - t_ / 2) * np.sin(x)] for x in np.radians(np.linspace(a1_, wa - 25, 60))]])
        U = arc.union(blade); Cl = U.buffer(fr, quad_segs=12).buffer(-fr, quad_segs=12)
        web = Point(*(Q + (R_ + t_ / 2) * np.array([np.cos(np.radians(wa)), np.sin(np.radians(wa))]))).buffer(3 * fr)
        blade = max(shapely.get_parts(blade.union(Cl.difference(U).intersection(web)).buffer(1e-7)), key=lambda g_: g_.area)
    # spine rib (lid side, hinge line): continues the keel forward so the lid's own rib section in front of the hinge keeps SF 2 at LC3
    S = C.get('spine'); spine = None
    if S:
        xs0, xs1 = Q[0] + S['x0'], Q[0] + S['x1']; xs = np.linspace(xs0, xs1, 40); h0, h1 = S['h']
        hh = np.interp(xs, [xs0, xs1 - S['taper'], xs1], [h0, h1, 0.0005])
        spine = Polygon([(x, zu(x, y0) + 0.0003) for x in xs] + [(x, zu(x, y0) - h) for x, h in zip(xs[::-1], hh[::-1])])
        spine = spine.buffer(-0.0003, join_style=1).buffer(0.0003, quad_segs=4)
    return blade, keel, dict(x_drop=xd, x_keel_end=x1k, x_blade_end=x1b, spine=spine)


def spring_unit(KF, p, Q, sg, y0, rbnd, C, K, PF, extrude, XZ):
    """Tape, idler (pulley + pin), drum with the spiral power spring inside, damper and axle (closed pose). K: material slots dict."""
    Ci, Cd = C['Ci'], C['Cd']; bw, bt = p['band_w'], 0.0002; I, D, S = C['idler'], C['drum'], C['spring']
    P = tape_path(C, Q, rbnd, p['arc_a'][0] + 2.0, p['exit_a'])
    P3 = np.c_[P[:, 0], np.full(len(P), y0), P[:, 1]]
    KF.add(*sweep(P3, np.array([[-bw / 2, -bt / 2], [bw / 2, -bt / 2], [bw / 2, bt / 2], [-bw / 2, bt / 2]]), up=(0, 1, 0)), K['tape'], PF['spring'])
    # idler: crowned POM pulley on a stainless pin held in the pod walls
    iw = I['w']; ri = I['r']
    KF.add(*lathe((Ci[0], y0 - iw / 2, Ci[1]), (0, 1, 0), [(0, 0), (0, ri - 0.0002), (0.0002, ri), (iw - 0.0002, ri), (iw, ri - 0.0002), (iw, 0)], 40), K['idler'], PF3['idler'])
    gwp = p['arc_w'] / 2 + p['clr_a'] + p['guide_cheek'] + p['pod_gap'] + p['pod_wall'] - 0.0003
    KF.add(*tube((Ci[0], y0 - gwp, Ci[1]), (Ci[0], y0 + gwp, Ci[1]), I['pin_r'], 16), K['steel'], PF3['idler'])
    # drum (tape winding radius D.r) with flanges; the spiral power spring lives inside it (drawn: it shows in the hinge section)
    hw = D['w'] / 2; rd = D['r']; fl = D['flange']
    KF.add(*lathe((Cd[0], y0 - hw, Cd[1]), (0, 1, 0), [(0, 0), (0, rd + fl - 0.0002), (0.0002, rd + fl), (0.0006, rd + fl), (0.0008, rd), (2 * hw - 0.0008, rd), (2 * hw - 0.0006, rd + fl),
                                                       (2 * hw - 0.0002, rd + fl), (2 * hw, rd + fl - 0.0002), (2 * hw, 0)], 56), K['drum'], PF['damper'])
    b_, t_ = S['strip']; r0 = S['arbor_r']; r1 = rd - 0.0006; nt = S['turns']; ph = np.linspace(0, 2 * np.pi * nt, int(nt * 72)); rr = r0 + (r1 - r0) * ph / ph[-1]
    sp = np.c_[Cd[0] + rr * np.cos(-ph), np.full(len(ph), y0), Cd[1] + rr * np.sin(-ph)]
    KF.add(*sweep(sp, np.array([[-b_ / 2, -t_ / 2], [b_ / 2, -t_ / 2], [b_ / 2, t_ / 2], [-b_ / 2, t_ / 2]]), up=(0, 1, 0)), K['steel'], PF['damper'])
    KF.add(*tube((Cd[0], y0 - hw - 0.0002, Cd[1]), (Cd[0], y0 + hw + 0.0002, Cd[1]), r0 - 0.0002, 20), K['steel'], PF['damper'])      # arbor
    return hw


def report(p, Q, R, C, parts, poly, zi, blade, keel, rbnd, OPEN, lugs_n):
    """Mass / inertia / gravity moment of the lid group with the fix-3 parts, and the spring + stop load table (closed-form; the FEA is
    in hingefea.py). parts: lidmech's [(m, x, z), ...] (skin, ribs, bosses + tongues, striker) - the old boss term is replaced."""
    rho_pl, rho_st = 1150.0, 7700.0; w = p['arc_w']; Kc = C['keel']
    m_keel = 2 * keel.area * Kc['w'] * rho_pl - 2 * blade.area * w * rho_pl + (2 * C['_spine'].area * C['spine']['t'] * rho_pl if C.get('_spine') is not None else 0.0)      # keels + spine ribs
    m_blade = 2 * blade.area * w * rho_st; m_lug = lugs_n * 2 * np.pi * (p['lug_d'] / 2) ** 2 * 0.0016 * rho_st
    kc, bc = keel.centroid, blade.centroid
    P = list(parts[:2]) + [(parts[2][0], parts[2][1], parts[2][2])] + list(parts[3:]) + [(m_keel, kc.x, kc.y), (m_blade + m_lug, bc.x, bc.y)]
    M = sum(a[0] for a in P); cx = sum(a[0] * a[1] for a in P) / M; cz = sum(a[0] * a[2] for a in P) / M
    # rotary inertia about Q: skin + ribs as the plan area (uniform), the rest as point masses
    A_ = poly.area; xs = np.linspace(*poly.bounds[0::2], 400); ys = np.linspace(*poly.bounds[1::2], 300); gx, gy = np.meshgrid(xs, ys)
    ins = shapely.contains_xy(poly, gx, gy); r2 = ((gx - Q[0]) ** 2 + (zi(np.c_[gx.ravel(), gy.ravel()]).reshape(gx.shape) - Q[1]) ** 2)[ins].mean()
    I = (P[0][0] + P[1][0]) * r2 + sum(a[0] * ((a[1] - Q[0]) ** 2 + (a[2] - Q[1]) ** 2) for a in P[2:])
    th = np.radians(np.arange(0, OPEN + 0.1, 5.0)); G = M * 9.81 * ((cx - Q[0]) * np.cos(th) - (cz - Q[1]) * np.sin(th))
    S = C['spring']; T = S['T0'] + (S['T75'] - S['T0']) * np.degrees(th) / 75.0
    F_band = T / (2 * rbnd); rd = C['drum']['r'] + 0.0001; Td = F_band * rd; travel = np.radians(OPEN) * rbnd; phi = travel / rd
    b_, t_ = S['strip']; E = 193e9; kd = (Td[0] - Td[-1]) / phi; L = E * b_ * t_ ** 3 / (12 * kd); sig = 6 * Td / (b_ * t_ ** 2)
    W = float(np.trapezoid(T - G, th))
    out = dict(mass_g=round(M * 1e3, 1), com_rel_Q_mm=[round((cx - Q[0]) * 1e3, 1), round((cz - Q[1]) * 1e3, 1)], inertia_kgm2=float('%.4g' % I),
               mass_new_g=dict(keels=round(m_keel * 1e3, 1), blades_lugs=round((m_blade + m_lug) * 1e3, 1)),
               gravity_Nm={str(int(round(a))): round(float(b), 4) for a, b in zip(np.degrees(th), G)},
               spring_Nm={str(int(round(a))): round(float(b), 4) for a, b in zip(np.degrees(th), T)},
               net_open_Nm_min=round(float((T - G).min()), 4), net_open_at_stop_Nm=round(float(T[-1] - G[-1]), 4), spring_work_surplus_J=round(W, 4),
               tape=dict(r_band_mm=round(rbnd * 1e3, 2), F_N=[round(float(F_band[0]), 2), round(float(F_band[-1]), 2)], travel_mm=round(travel * 1e3, 1), exit_deg=p['exit_a']),
               drum=dict(r_mm=round(rd * 1e3, 2), turn_deg=round(np.degrees(phi), 1), torque_Nmm=[round(float(Td[0]) * 1e3, 2), round(float(Td[-1]) * 1e3, 2)],
                         centre_rel_Q_mm=np.round((C['Cd'] - Q) * 1e3, 2).tolist()),
               idler_centre_rel_Q_mm=np.round((C['Ci'] - Q) * 1e3, 2).tolist(),
               spiral=dict(strip_mm=[b_ * 1e3, t_ * 1e3], length_mm=round(L * 1e3, 1), rate_Nmm_per_rad=round(kd * 1e3, 3), preload_turns_open=round(Td[-1] / kd / (2 * np.pi), 2),
                           sigma_MPa=[round(float(sig[0]) / 1e6), round(float(sig[-1]) / 1e6)]))
    return out
