# [fix4-hinge2] Geared spring / damper drive of the v18 arc-track lid hinge (called from lidmech.build when lid.h4 is set; needs lid.h3).
#
# Review round 4 (M1 margin, M2 drum not buildable, M3 slack tape with the damper on the drum, m13 hold-open, S8 undamped closing,
# n14 spring drawing, G5 open / inverted parts): the tape + idler + drum of fix-hinge are replaced by gears.
#  - TONGUE: the MIM 17-4PH tongue gets a sector gear in the middle 4.6 mm of its outer face (m 0.5, z 72 -> R 18.0, tips 0.5 below the
#    rail lands, so the POM outer rails keep running on the two 1.7 mm side lands). z 72 = one tooth per 5 deg of lid travel.
#  - PINION (MIM, z 16, x +0.1, 4.0 wide) meshes with it through the slot in the outer rail (rail_split round exit_a = phi_p). Its shaft
#    (d 2.0) runs in bores of two lobes of the stainless cassette plates (the plates also carry the rails: centre distance held by one
#    part). Ratio 4.5: the pinion turns 337.5 deg for 75 deg of lid = 15 teeth -> the static pinion meshes at every 5 deg.
#  - SPRING: spiral power spring 8.0 x 0.19 in a closed steel cartridge (deep-drawn 1.4310 cup 0.3 + cover, r_in 5.8, 0.2 axial play per
#    side), outer end hooked in the cup (keyed in the pod by a tab), inner end on the cartridge's arbor sleeve (d 3.2, D-bore on the shaft's
#    flat). Drawn wound (closed lid): packed coils on the arbor and on the cup + one free transition turn, 0.03 off the arbor and the cup.
#  - DAMPER: two-way rotary damper (d 12.2 x 7, silicone, catalogue part keyed by a tab) on the inner shaft end (rotor D-bore). It damps the
#    LID in both directions through the mesh: soft-open, soft-close, nothing can go slack.
#  - HOUSING: cartridge and damper sit INBOARD of the pod, in one bore (r 6.3) of a housing moulded on the pod (wall 1.0, snap end cap);
#    outboard there is no room (the well's rear corner, r 16, starts at |y| 61.5). [round-4 resume] Assembly from inboard: shaft through
#    the inboard plate lobe, the pinion (D-bore) and the outboard lobe; cartridge onto the flat (tab into the key slot); damper; end cap.
#  - DETENT: TPE lips (0.05 into the lug path) on both flanks of each plate slot just behind the lug's seat at the open stop.
#  - The pod becomes ONE closed solid (prismkit.slab_union) with real cavities for pinion, plate lobes, shaft, cartridge, damper.
# Numbers (spring, efficiency, friction, tolerances, damper, detent): hingebal.py.
import numpy as np, shapely
from shapely.geometry import Polygon, Point
import prismkit as PK, hingebal as HB

DESIGNED4 = {('tongue', 'f_pinion'): '[fix4-hinge2] gear mesh: tongue sector gear x pinion (backlash 0.05..0.14 at the pitch point); the pinion turns '
                                     '-4.5 x the lid angle - the static pinion of a kinematic slice check is not its real pose (interfere.py turns it)'}
DEF4 = dict(gear=dict(m=0.0005, z_w=72, z_p=16, alpha=20.0, x_p=0.1, band=0.0030, pinion_w=0.0024, backlash=0.00004, zone=[-87.0, 12.0], runout=5.5),      # zone: past the pinion's tip circle (+-10 deg) at both ends + over-travel
            shaft=dict(r=0.0010, arbor_r=0.0016, lobe_r=0.0026, journal_clr=0.00002, hole_clr=0.0003, damper_in=0.0045, end_out=0.0003, blind=0.0006, rotor_clr=0.00003),
            spring=dict(T0=0.36, T75=0.21, strip=[0.0080, 0.00019], cup_r_in=0.0058, cup_t=0.0003, ax_clr=0.0002, hole_r=0.0013, coil_clr=0.00003, E=190e9, sig_allow=950e6),
            cart=dict(gap=0.0002, wall=0.0010, cap=0.0010, tab=[0.0024, 0.0008], tab_clr=0.0001),
            damper=dict(r=0.0061, l=0.0070, c_Q=0.12, gap=0.0002, key=[0.0024, 0.0008]),
            detent=dict(delta=0.00005, E=5e6, mu=0.20, ramp_in=5.0, deg=1.5),
            pod_bev=0.0010)


def cfg4(p):
    H = p.get('h4')
    if not H: return None
    return {k: (dict(v, **H.get(k, {})) if isinstance(v, dict) else H.get(k, v)) for k, v in DEF4.items()}


# ------------------------------------------------------------------------------------------------------------------- 2D helpers
def gear_poly(z, m, alpha=20.0, x=0.0, centre=(0.0, 0.0), rot_deg=0.0, n_inv=7, n_arc=3):
    """external involute spur gear outline (CCW), tooth 0 centred at rot_deg. Root: circle rf (radial line down from the base circle)."""
    r = z * m / 2; al = np.radians(alpha); rb = r * np.cos(al); ra = r + m * (1 + x); rf = r - m * (1.25 - x)
    inv = lambda a: np.tan(a) - a
    s = m * (np.pi / 2 + 2 * x * np.tan(al)); psi = s / (2 * r)                                  # half tooth angle at the pitch circle
    rs = np.linspace(max(rb, rf), ra, n_inv); th = psi + inv(al) - inv(np.arccos(np.clip(rb / rs, -1, 1)))   # flank half angle at radius rs
    half_root = np.pi / z                                                                        # tooth pitch / 2
    th_b = th[0]; pts = []
    for k in range(z):
        c = np.radians(rot_deg) + 2 * np.pi * k / z
        a_r0 = c - half_root; ra_ = np.linspace(a_r0, c - th_b, n_arc + 1)[1:]                    # root arc (second half of the gap before tooth k)
        if k == 0: pts.append((rf * np.cos(a_r0), rf * np.sin(a_r0)))
        for a_ in ra_[:-1]: pts.append((rf * np.cos(a_), rf * np.sin(a_)))
        if rf < rb: pts.append((rf * np.cos(c - th_b), rf * np.sin(c - th_b)))
        for rr, tt in zip(rs, th): pts.append((rr * np.cos(c - tt), rr * np.sin(c - tt)))         # leading flank (base -> tip)
        ta = np.linspace(c - th[-1], c + th[-1], n_arc + 1)[1:-1]
        for a_ in ta: pts.append((ra * np.cos(a_), ra * np.sin(a_)))
        for rr, tt in zip(rs[::-1], th[::-1]): pts.append((rr * np.cos(c + tt), rr * np.sin(c + tt)))
        if rf < rb: pts.append((rf * np.cos(c + th_b), rf * np.sin(c + th_b)))
        for a_ in np.linspace(c + th_b, c + half_root, n_arc + 1)[1:]: pts.append((rf * np.cos(a_), rf * np.sin(a_)))
    P = np.array(pts)[:-1] + np.asarray(centre)
    return Polygon(P).buffer(0), dict(r=r, rb=rb, ra=ra, rf=rf)


def disc(c, r, n=72): return Point(*c).buffer(r, quad_segs=max(4, n // 4))


def setup4(p, Q, C4):
    """pinion centre (world x, z) on the ray exit_a (= phi_p) at the working centre distance"""
    G = C4['gear']; R_g = G['z_w'] * G['m'] / 2; r_p = G['z_p'] * G['m'] / 2
    co = p['clr_r'] + p['band_t'] + 0.0001; ci = p['clr_r']                                     # outer / inner running clearance of the tongue
    a = R_g + r_p + G['x_p'] * G['m'] + G['backlash'] + co                                       # backlash + tip clearance with the tongue ON the outer rail
    e = np.array([np.cos(np.radians(p['exit_a'])), np.sin(np.radians(p['exit_a']))])
    C4.update(Cp=Q + a * e, a=a, R_g=R_g, r_p=r_p, phi_p=p['exit_a'], co=co, ci=ci)
    return C4


# ------------------------------------------------------------------------------------------------------------------- lid side
def zone_cut(Q, r0, r1, a_lo, a_hi, ru, out=0.001, step=0.1):
    """region cut from the tongue's middle band (minus the gear outline = the tooth spaces): above r0 over the zone, the floor rising from r0
    to the full radius r1 over ru deg at both ends (run-out: no step at the root-side end, the tongue's most loaded section). Units of Q."""
    u = np.unique(np.r_[np.linspace(a_lo, a_hi, max(20, int((a_hi - a_lo) / step))), a_lo + ru, a_hi - ru]); f = np.clip(np.maximum((u - (a_hi - ru)) / ru, ((a_lo + ru) - u) / ru), 0.0, 1.0)
    rc = r0 + 1.2 * (r1 - r0) * f; ro = r1 + out; ur = np.radians(u)      # the floor leaves the outer face at a finite angle (5/6 of the run-out): no sliver
    return Polygon(np.r_[np.c_[Q[0] + ro * np.cos(ur), Q[1] + ro * np.sin(ur)], np.c_[Q[0] + rc[::-1] * np.cos(ur[::-1]), Q[1] + rc[::-1] * np.sin(ur[::-1])]])


def tongue(p, Q, C4, tg_full, y0, frame):
    """toothed tongue: full section on the two side lands, sector gear in the middle band (closed, oriented solid)"""
    G = C4['gear']; w = p['arc_w']; hb = G['band'] / 2; ph = C4['phi_p']; z0, z1 = G['zone']
    gw, gi = gear_poly(G['z_w'], G['m'], G['alpha'], 0.0, Q, ph + 180.0 / G['z_w'])               # tooth 0 centred half a pitch past phi_p: a GAP on the line of centres
    a_lo, a_hi = ph + z0, ph + z1
    from lidmech import sector
    zone = zone_cut(Q, gi['rf'] - 0.0003, p['arc_r'] + p['arc_t'] / 2, a_lo, a_hi, G.get('runout', 5.5))
    tg_t = tg_full.difference(zone.difference(gw))
    C4['tongue_gear'] = gi; C4['tongue_t_area'] = float(tg_t.area)
    return PK.slab_union([(tg_full, y0 - w / 2, y0 - hb), (tg_t, y0 - hb, y0 + hb), (tg_full, y0 + hb, y0 + w / 2)], frame, bev=0.0004, nb=3)


# ------------------------------------------------------------------------------------------------------------------- fixed side
def lobe(C4):
    """cheek-plate lobe round the pinion shaft (journal bore inside)"""
    S = C4['shaft']; return disc(C4['Cp'], S['lobe_r']).difference(disc(C4['Cp'], S['r'] + S['journal_clr']))


def layout(C4, gw_half, gw_pod):
    """offsets e along the hinge's outboard direction (m; inboard < 0): pod -gw_pod..gw_pod, then INBOARD the spring cartridge, the
    damper and the housing's end cap in one bore (the outboard side is the well's rear corner - no room)"""
    Sp, Ca, Dm = C4['spring'], C4['cart'], C4['damper']
    cw = Sp['strip'][0] + 2 * Sp['ax_clr'] + 2 * Sp['cup_t']; r_cup = Sp['cup_r_in'] + Sp['cup_t']
    e_c0 = -(gw_pod + Ca['gap']); e_c1 = e_c0 - cw; e_d0 = e_c1 - Ca['gap']; e_d1 = e_d0 - Dm['l']; e_k1 = e_d1 - Ca['gap']; e_k2 = e_k1 - Ca['cap']
    r_cav = max(r_cup, Dm['r']) + Ca['gap']
    L = dict(cw=cw, e_c0=e_c0, e_c1=e_c1, e_d0=e_d0, e_d1=e_d1, e_k1=e_k1, e_k2=e_k2, r_cup=r_cup, r_cav=r_cav, r_h=r_cav + Ca['wall'])
    C4['L'] = L; return L


def _yr(y0, sg, e0, e1):
    a, b = y0 + sg * e0, y0 + sg * e1; return (min(a, b), max(a, b))


def tab_box(c, r, w, h, clr=0.0, ang=-135.0):
    """anti-rotation key on a circle of radius r at angle ang (deg, x-z; -135 = bottom-rear, where the pod housing has its square corner):
    w wide, h high (radial), overlapping 0.4 into the circle"""
    a = np.radians(ang); er = np.array([np.cos(a), np.sin(a)]); et = np.array([-er[1], er[0]])
    P = [c + er * (r - 0.0004) - et * (w / 2 + clr), c + er * (r + h + clr) - et * (w / 2 + clr), c + er * (r + h + clr) + et * (w / 2 + clr), c + er * (r - 0.0004) + et * (w / 2 + clr)]
    return Polygon(P)


def drive_unit(KF, p, Q, sg, y0, C4, K, PF, frame, gw_half, gw_pod):
    """pinion + shaft (+ the cartridge's arbor sleeve, keyed on the shaft: it turns with it), spring cartridge, spiral spring (wound: lid
    closed), rotary damper (closed pose). K: material slots."""
    G, S, Sp, Ca, Dm = C4['gear'], C4['shaft'], C4['spring'], C4['cart'], C4['damper']; Cp = C4['Cp']; L = layout(C4, gw_half, gw_pod)
    # pinion (tooth on the line of centres pointing at Q) on the shaft; the shaft ends in a blind hole of the outboard pod wall and in the
    # damper's rotor bore; the arbor sleeve inside the cartridge
    gp, gi = gear_poly(G['z_p'], G['m'], G['alpha'], G['x_p'], Cp, C4['phi_p'] + 180.0)
    C4['pinion_gear'] = gi; hp = G['pinion_w'] / 2
    sh = disc(Cp, S['r'], 32); arb = disc(Cp, S['arbor_r'], 40)
    e_end = L['e_d0'] - S['damper_in']; e_a1 = L['e_c1'] + Sp['cup_t'] + 0.0001; e_a0 = L['e_c0'] - Sp['cup_t'] - 0.0001
    sl = [(sh, *_yr(y0, sg, e_end, e_a1)), (arb, *_yr(y0, sg, e_a1, e_a0)), (sh, *_yr(y0, sg, e_a0, -hp)), (gp, *_yr(y0, sg, -hp, hp)), (sh, *_yr(y0, sg, hp, gw_half + S['end_out']))]
    KF.add(*PK.slab_union(sl, frame, bev=0.0001, nb=1), K['pinion'], PF['pinion'])
    # spring cartridge: cup + cover (one closed shell with the spring room inside), keyed by a tab at the bottom
    tab = tab_box(Cp, L['r_cup'] - 0.0001, Ca['tab'][0], Ca['tab'][1])
    ring_o = disc(Cp, L['r_cup'], 96).union(tab); hole = disc(Cp, Sp['hole_r'], 40)
    cup = [(ring_o.difference(hole), *_yr(y0, sg, L['e_c1'], L['e_c1'] + Sp['cup_t'])),
           (ring_o.difference(disc(Cp, Sp['cup_r_in'], 96)), *_yr(y0, sg, L['e_c1'] + Sp['cup_t'], L['e_c0'] - Sp['cup_t'])),
           (ring_o.difference(hole), *_yr(y0, sg, L['e_c0'] - Sp['cup_t'], L['e_c0']))]
    KF.add(*PK.slab_union(cup, frame, bev=0.0001, nb=1), K['cart'], PF['cartridge'])
    # spiral strip, drawn wound (closed lid): packed coils on the arbor, a free transition, packed coils on the cup (coil_clr off both)
    b_, t_ = Sp['strip']; sp = C4['spiral']; Lstrip = sp['length_mm'] / 1e3; wound = sp['preload_turns_open'] + sp['stroke_turns']
    f_ = wound / max(sp['turns_available'], 1e-6); ra_ = S['arbor_r'] + t_ / 2 + Sp['coil_clr']; rc_ = Sp['cup_r_in'] - t_ / 2 - Sp['coil_clr']
    La = f_ * Lstrip; ka = (-ra_ + np.sqrt(ra_ ** 2 + t_ * La / np.pi)) / t_                     # coils on the arbor: 2 pi (ra k + t k^2 / 2) = La
    Lt_ = 2 * np.pi * 0.5 * (ra_ + ka * t_ + rc_) * 1.0; Lc = max(Lstrip - La - Lt_, 0.0)
    kc = (rc_ - np.sqrt(max(rc_ ** 2 - t_ * Lc / np.pi, 0.0))) / t_
    r_a1, r_c1 = ra_ + ka * t_, rc_ - kc * t_
    ph1 = np.linspace(0, 2 * np.pi * ka, max(8, int(ka * 96))); ph2 = np.linspace(0, 2 * np.pi, 96)[1:]; ph3 = np.linspace(0, 2 * np.pi * kc, max(8, int(kc * 96)))[1:]
    R1 = ra_ + t_ * ph1 / (2 * np.pi); R2 = r_a1 + (r_c1 - r_a1) * (0.5 - 0.5 * np.cos(np.pi * ph2 / (2 * np.pi))); R3 = r_c1 + t_ * ph3 / (2 * np.pi)
    PH = np.r_[ph1, ph1[-1] + ph2, ph1[-1] + 2 * np.pi + ph3]; RR = np.r_[R1, R2, np.minimum(R3, rc_)]
    ang = -PH                                                                                   # wound clockwise (x-z view)
    ym = y0 + sg * 0.5 * (L['e_c0'] + L['e_c1'])
    path = np.c_[Cp[0] + RR * np.cos(ang), np.full(len(RR), ym), Cp[1] + RR * np.sin(ang)]
    from meshkit import sweep
    sec_ = np.array([[-b_ / 2, -t_ / 2 * 0.96], [b_ / 2, -t_ / 2 * 0.96], [b_ / 2, t_ / 2 * 0.96], [-b_ / 2, t_ / 2 * 0.96]])
    KF.add(*PK.orient(*sweep(path, sec_, up=(0, 1, 0))), K['spring'], PF['spring'])
    sp.update(drawn_coils=dict(arbor=round(float(ka), 2), transition=1.0, cup=round(float(kc), 2), r_arbor_pack_mm=round(r_a1 * 1e3, 2), r_cup_pack_mm=round(r_c1 * 1e3, 2),
                               clearance_to_arbor_and_cup_mm=Sp['coil_clr'] * 1e3))
    # two-way rotary damper: body keyed in the housing bore (same key slot as the cartridge tab), rotor bore (D) on the shaft end
    key = tab_box(Cp, Dm['r'] - 0.0001, Dm['key'][0], Dm['key'][1])
    dmp = disc(Cp, Dm['r'], 96).union(key); bore = disc(Cp, S['r'] + S['rotor_clr'], 40); e_b = e_end - 0.0003
    dsl = [(dmp, *_yr(y0, sg, L['e_d1'], e_b)), (dmp.difference(bore), *_yr(y0, sg, e_b, L['e_d0'] - 0.0006)),
           (disc(Cp, Dm['r'] - 0.0006, 96).union(key).difference(bore), *_yr(y0, sg, L['e_d0'] - 0.0006, L['e_d0']))]
    KF.add(*PK.slab_union(dsl, frame, bev=0.0003, nb=2), K['damper'], PF['damper'])
    return L


def housings(C4, x_min, x_lim, z_bot):
    """x-z profile of the cartridge / damper housing: round top (cylinder about the shaft), square below down to z_bot (it stands on the
    pod's floor level like the pod itself); front edge of the square part at x_lim (the well floor's hole edge over the control panel), rear >= x_min"""
    Cp = C4['Cp']; r_ = C4['L']['r_h']; z_bot = min(z_bot, Cp[1] - r_ - 0.0002)      # the bottom wall >= the others (0.75 below the pod floor; not tangent to the round top)
    g_ = disc(Cp, r_, 96).union(shapely.box(Cp[0] - r_, z_bot, min(Cp[0] + r_, x_lim - 0.0002), Cp[1]))      # 0.2 behind the pod's front face (no collinear faces)
    return {'cart': g_.intersection(shapely.box(x_min, z_bot, Cp[0] + r_ + 0.001, Cp[1] + r_ + 0.001))}


def pod(KF, p, Q, sg, y0, C4, prof, chan, slot, CAS3, gw_half, gw_pod, yc_, K, part, frame, H):
    """the hinge pod as ONE closed solid: main block (the f3 profile) with the cassette envelope, the tongue channel, the pinion cavity
    (between the rail side lands: through the outer-rail slot), the shaft holes, the plate-lobe pockets; inboard the cartridge / damper
    housing (one bore with the key slot, end cap)."""
    S, Sp, Ca, Dm = C4['shaft'], C4['spring'], C4['cart'], C4['damper']; Cp = C4['Cp']; L = C4['L']; G = C4['gear']
    pin_cav = disc(Cp, C4['pinion_gear']['ra'] + 0.0003); lobe_cav = disc(Cp, S['lobe_r'] + 0.0003); hole = disc(Cp, S['r'] + S['hole_clr'], 40)
    cav = disc(Cp, L['r_cav'], 96).union(tab_box(Cp, L['r_cup'] - 0.0001, Ca['tab'][0], Ca['tab'][1], Ca['tab_clr'])).union(
        tab_box(Cp, Dm['r'] - 0.0001, Dm['key'][0], Dm['key'][1], Ca['tab_clr']))
    hc = H['cart']
    hp = G['pinion_w'] / 2 + 0.0003; tp = p['guide_cheek']; e_pl0 = gw_half - tp                # cassette plates: e in [gw_half - t, gw_half]
    E = [(L['e_k2'], L['e_k1'], hc), (L['e_k1'], -gw_pod, hc.difference(cav)),
         (-gw_pod, -gw_half, prof.difference(hole)), (-gw_half, -e_pl0, prof.difference(CAS3).difference(lobe_cav)),
         (-e_pl0, -hp, prof.difference(chan).difference(CAS3).difference(hole)), (-hp, hp, prof.difference(chan).difference(CAS3).difference(pin_cav)),
         (hp, e_pl0, prof.difference(chan).difference(CAS3).difference(hole)), (e_pl0, gw_half, prof.difference(CAS3).difference(lobe_cav)),
         (gw_half, gw_half + S['blind'], prof.difference(hole)), (gw_half + S['blind'], gw_pod, prof)]
    sl = [(g_, *_yr(y0, sg, e0, e1)) for e0, e1, g_ in E]                                     # no simplify: neighbouring slabs must share their vertices
    bh_ = 0.0003; KF.add(*PK.slab_union(sl, frame, bev=(bh_, C4['pod_bev']) if sg > 0 else (C4['pod_bev'], bh_), nb=3), K, part)      # 0.3 edge round on the housing's end cap (concave corner), 1.0 on the pod's outboard face
    return [(hc.bounds, _yr(y0, sg, L['e_k2'], -gw_pod))]


def pad4(Q, R, lr, clr, slot, a_open, as1, sector, C4):
    """TPE first-stage pad + detent: the crescent ahead of the lug's seat, a thin liner on both slot flanks beside the seated lug that hugs
    its rear quadrant, and behind it the two detent lips that narrow the lug's path by delta per side (entry ramp over 'deg', crest at
    the point where the seated lug's circle is delta in from the flank -> exit contact angle acos((r - delta) / r))."""
    Dt = C4['detent']; dl = Dt['delta']; Dc = np.degrees(np.sqrt(2 * lr * dl - dl * dl) / R); a_s = a_open - Dc - Dt['deg'] - 0.6
    angs = np.linspace(a_s - 0.3, a_open, 240); s_ = np.radians(a_open - angs) * R
    hug = lr - np.sqrt(np.clip(lr * lr - s_ * s_, 0.0, None)) - 0.00001                        # just clear of the seated lug
    ramp = dl * np.clip((angs - (a_open - Dc - Dt['deg'])) / Dt['deg'], 0.0, 1.0)
    pr = np.clip(np.minimum(ramp, hug), 0.0, dl); u = np.radians(angs)
    rin = R - lr + pr; rout = R + lr - pr
    core = Polygon(np.r_[np.c_[Q[0] + rin * np.cos(u), Q[1] + rin * np.sin(u)], np.c_[Q[0] + rout[::-1] * np.cos(u[::-1]), Q[1] + rout[::-1] * np.sin(u[::-1])]])
    lc = Q + R * np.array([np.cos(np.radians(a_open)), np.sin(np.radians(a_open))])
    pad = slot.intersection(sector(Q, 0.0, R + 0.01, a_s, as1 + 1.0)).difference(core).difference(Point(*lc).buffer(lr + 0.00001, quad_segs=32))
    parts = [g for g in shapely.get_parts(pad) if g.area > 1e-10]
    C4['detent_engage_deg'] = float(Dc + Dt['deg'])                                              # lid angle before the stop where the lips start
    return PK.clean(shapely.union_all(parts), 5e-7) if parts else Polygon()


# ------------------------------------------------------------------------------------------------------------------- report
def spiral_of(C4, OPEN):
    G, Sp, S = C4['gear'], C4['spring'], C4['shaft']; n = G['z_w'] / G['z_p']
    Tp0, Tp75 = Sp['T0'] / 2 / n, Sp['T75'] / 2 / n
    return HB.spiral_design(Tp0, Tp75, np.radians(OPEN) * n, Sp['strip'][0], Sp['strip'][1], Sp['cup_r_in'], S['arbor_r'], Sp['E'], Sp['sig_allow'])


def report(p, Q, C4, h3, rep, OPEN, tongue_area_removed_m2):
    """ONE lid report: lidmech / hingekit.report mass model (+ the STOP parts in extra_mass) corrected for the tongue teeth, the balance
    of the geared drive under tolerances (hingebal), the gear / spring / damper / detent design tables."""
    G, Sp, S, Dm, Dt = C4['gear'], C4['spring'], C4['shaft'], C4['damper'], C4['detent']
    dm = -2 * tongue_area_removed_m2 * G['band'] * 7700.0                                       # both tongues (MIM 7.7 g/cm3)
    M = h3['mass_g'] / 1e3 + dm; cx = (h3['mass_g'] / 1e3 * h3['com_rel_Q_mm'][0] / 1e3 + dm * 0.018 * np.cos(np.radians(C4['phi_p'] - 40))) / M
    cz = (h3['mass_g'] / 1e3 * h3['com_rel_Q_mm'][1] / 1e3 + dm * 0.018 * np.sin(np.radians(C4['phi_p'] - 40))) / M
    xs = rep.get('_extra_com_rel_Q', (0.126, -0.0025))
    rg = lambda q: Q + q
    Lb = HB.Lid(M, (cx, cz), xs, h3['inertia_kgm2'], p['arc_a'], (p['guide_a'][0], p['guide_a'][1] - 0.6), p['arc_r'] - p['arc_t'] / 2, p['arc_r'] + p['arc_t'] / 2,
                C4['R_g'], C4['phi_p'], G['alpha'], L_edge=rep.get('_L_edge', 0.205), open_deg=OPEN)
    ti, to, Fl = HB.detent(Dt['delta'], lr=p['lug_d'] / 2, clr=p['lug_clr'], E=Dt['E'], b=p['guide_cheek'] * h3.get('_lug_eng', 0.92), mu=Dt['mu'], n_lugs=4, R=p['arc_r'], ramp_in_deg=Dt['ramp_in'])
    D = HB.Drive(G['z_w'], G['z_p'], G['m'], Sp['T0'], Sp['T75'], S['r'], detent_in=ti, detent_out=to, detent_deg=Dt['deg'], c_Q=Dm['c_Q'])
    sp = C4['spiral']; bal = HB.analyse(Lb, D, spiral=sp)
    th = np.arange(0, OPEN + 0.1, 5.0); Gv = Lb.G(th)
    a_nom = C4['a']; gi, pi_ = C4['tongue_gear'], C4['pinion_gear']
    bal['gear'].pop('lewis_MPa_flick_b4', None); bal['gear']['lewis_pinion_MPa_flick'] = round(bal['gear']['tooth_force_N']['flick'] / (G['pinion_w'] * G['m'] * 0.30) / 1e6, 1)      # face = the pinion width
    bal['gear'].update(centre_distance_mm=round(a_nom * 1e3, 3), pinion_centre_rel_Q_mm=np.round((C4['Cp'] - Q) * 1e3, 2).tolist(), mesh_angle_deg=C4['phi_p'],
                       tongue_tip_root_mm=[round(gi['ra'] * 1e3, 3), round(gi['rf'] * 1e3, 3)], pinion_tip_root_mm=[round(pi_['ra'] * 1e3, 3), round(pi_['rf'] * 1e3, 3)],
                       band_mm=G['band'] * 1e3, pinion_w_mm=G['pinion_w'] * 1e3, profile_shift_pinion=G['x_p'],
                       contact_ratio_tongue_on_outer_rail=round(HB.contact_ratio(C4['R_g'], C4['r_p'] + G['x_p'] * G['m'], G['m'], a_nom - C4['co']), 3),
                       contact_ratio_tongue_centred=round(HB.contact_ratio(C4['R_g'], C4['r_p'] + G['x_p'] * G['m'], G['m'], a_nom), 3),
                       contact_ratio_tongue_on_inner_rail=round(HB.contact_ratio(C4['R_g'], C4['r_p'] + G['x_p'] * G['m'], G['m'], a_nom + C4['ci']), 3),
                       tip_clearance_min_mm=round((a_nom - C4['co'] - pi_['ra'] - gi['rf']) * 1e3, 3), radial_play_mm=round((C4['co'] + C4['ci']) * 1e3, 3),
                       phases_match_every_deg=round(360.0 / G['z_w'], 3), shaft_d_mm=S['r'] * 2e3, journal='stainless cassette-plate lobes (both sides of the pinion), greased')
    bal['detent'] = dict(delta_mm=Dt['delta'] * 1e3, entry_Nm=round(ti, 4), exit_Nm=round(to, 4), lip_force_N=round(Fl, 3), engaged_last_deg=Dt['deg'],
                         model='TPE lips (E %.0f MPa) on both slot flanks; exit crest angle acos((r - delta) / r) of the lug; mu %.2f' % (Dt['E'] / 1e6, Dt['mu']))
    bal['damper'].update(type='two-way rotary damper on the pinion shaft (silicone, d %.1f x %.1f), keyed in the housing; damps the lid through the mesh in both directions' % (Dm['r'] * 2e3, Dm['l'] * 1e3))
    L_ = rep.get('_L_edge', 0.205); Fc_ = bal['closing_force_edge_N']; r_z = (0.0083, 0.0259)      # S8: hinge-side crush zone of the side edges (review: r 8.3..25.9 mm from Q)
    bal['pinch'] = dict(zone_r_mm=[r_z[0] * 1e3, r_z[1] * 1e3], gap_at_75deg_mm=[round(0.966 * r_z[0] * 1e3, 1), round(0.966 * r_z[1] * 1e3, 1)],
                        force_amplification_edge_to_zone=[round(L_ / r_z[1], 1), round(L_ / r_z[0], 1)],
                        push_needed_at_edge_N=dict(nominal_max=Fc_['nominal_max'], worst_max=Fc_['worst_high_max']),
                        self_releasing=bool(bal['net_open_worst_min_Nm'] > 0), lid_cannot_fall_or_slam=True,
                        soft_close=dict(damper_Nm_at_60dps=bal['damper']['soft_close_Nm_at_60dps'], edge_N_at_60dps=bal['damper']['soft_close_edge_N_at_60dps']),
                        note='the spring opens the lid at every angle under every tolerance (net opening torque > 0): a finger is only held while the user '
                             'pushes, and is released as soon as the push stops; the two-way damper damps closing (no free fall, no slam). The geometry '
                             'of the 8-26 mm side-edge gap is unchanged (cover-side corner lip recommended, see interfaces); risk assessment + manual note.')
    bal['cartridge'] = dict(cup='deep-drawn 1.4310, %.1f wall, r_in %.1f, + snap cover; keyed in the pod by a tab' % (Sp['cup_t'] * 1e3, Sp['cup_r_in'] * 1e3),
                            width_mm=round(C4['L']['cw'] * 1e3, 2), axial_play_per_side_mm=Sp['ax_clr'] * 1e3, housing_wall_mm=C4['cart']['wall'] * 1e3,
                            housing_gap_mm=C4['cart']['gap'] * 1e3, housing='inboard of the pod, one bore r %.1f (wall %.1f) for cartridge + damper, key slot at the bottom rear, '
                            'snap end cap %.1f; %.1f .. %.1f mm inboard of the hinge plane' % (C4['L']['r_cav'] * 1e3, C4['cart']['wall'] * 1e3, C4['cart']['cap'] * 1e3, -C4['L']['e_c0'] * 1e3 - C4['cart']['gap'] * 1e3, -C4['L']['e_k2'] * 1e3),
                            assembly='1 pinion (D-bore) between the cassette plates; 2 shaft from inboard through the open housing, the inboard plate lobe, the pinion and '
                            'the outboard lobe into its blind hole; 3 cartridge (cup + arbor sleeve + spring pre-wound to the open-pose preload + cover, one unit) '
                            'onto the shaft flat, tab into the key slot; 4 damper (rotor D-bore on the shaft end, body key in the same slot); 5 end cap snaps on',
                            arbor='sleeve d %.1f inside the cartridge (captured by the cup holes d %.1f), D-bore on the shaft d %.1f' % (C4['shaft']['arbor_r'] * 2e3, C4['spring']['hole_r'] * 2e3, C4['shaft']['r'] * 2e3))
    out = dict(mass_g=round(M * 1e3, 1), com_rel_Q_mm=[round(cx * 1e3, 1), round(cz * 1e3, 1)], inertia_kgm2=h3['inertia_kgm2'],
               mass_change_g=dict(tongue_teeth=round(dm * 1e3, 2)), mass_range_g=[TOL_ * 1e3 for TOL_ in HB.TOL['M_RANGE']],
               gravity_Nm={str(int(t)): round(float(v), 4) for t, v in zip(th, Gv)}, balance=bal)
    return out
