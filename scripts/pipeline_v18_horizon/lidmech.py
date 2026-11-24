# v18 control-panel lid mechanism (called by design_gen.py; numpy / scipy / shapely only).
#
# The lid is a moulded part (2.2 mm skin, rolled top edge, perimeter rib + two concentric ring ribs + radial ribs, two hinge bosses,
# striker boss). It opens by a PURE ROTATION about a virtual axis Q parallel to Y that lies just above the roof surface, 3 mm behind
# the rear shut line (laptop-lid kinematics: the rear edge only turns in place, the gap never closes). The rotation is produced by two
# concealed stainless arc tongues (centre line = circle about Q) that run in POM guides inside sealed pockets of the well moulding.
#   - open stop: a lug on each tongue runs in a slot of the guide cheek and lands on a TPE bumper at open_deg (load at R 25 in the
#     pocket, never on a cosmetic edge); the other end of the slot is the bottom stop for the latch over-travel.
#   - soft-open: a constant-force spring band pinned to the tail of each tongue runs along the guide and winds onto a coil drum
#     beside it; a rotary damper on the drum axle slows the opening (the tongues themselves stay plain polished arcs).
#   - push-push latch at the front centre: housing with a mouth and a rotary catch (fork) on the well wall, steel loop striker on the lid.
#   - continuous hollow bulb seal on a dam, compressed by the lid underside; drained gutter between the cover edge and the dam;
#     the well floor is a frame round the control panel with a TPE skirt lip on the panel top (the well never cuts the control box).
# [v18b] Compact layout for the lower v18 roof (the well floor is only 20 mm under the axis): arc R 17, every part of the hinge stays
#   between the dam and the control box. In front of the dam each hinge sits in a moulded HINGE POD (dark, same colour as the well):
#   its top is the guide's end face, its back a concave arc concentric with Q, and the tongue leaves it through a mouth slot. The pod
#   also hides the spring-band exit, the coil drum and (under the well floor) the damper. Behind the dam the pocket ceiling steps up
#   behind the gutter so the guide can run far enough back for 43 deg of engagement at the open stop.
# Everything is generated in the CLOSED pose. kitm_* moves with the lid, kitf_* is fixed to the cover; kitm_part / kitf_part give the
# part of every face (codes PM / PF below). lid_pivot = (Qx, Qz), lid_open = open_deg: the renderer / viewer rotate the lid group about it.
import os, numpy as np, shapely
from shapely.geometry import Polygon, Point, LineString, MultiPoint
from shapely.geometry.polygon import orient
from scipy.spatial import cKDTree, Delaunay
from meshkit import Bag, stack, lathe, tube, sweep, rrect

K_D, K_RUB, K_STEEL, K_ARM, K_GREY, K_GLOSS, K_LID = range(7)            # renderer material slots (KITM order)
K_GUIDE = K_D                                                              # guides: black POM (only the stainless arcs read as mechanism)
PM = dict(underside=1, rib=2, boss=3, tongue=4, lug=5, striker=6, edge=7, plunger=8)      # [fix2] plunger: STOP plunger under the STOP pad (R2E4)                        # kitm_part
PF = dict(cover=10, well=11, seal=12, guide=13, pocket=14, damper=15, latch=16, claw=17, bumper=18, drain=19, lip=20, spring=21, edge=22, pod=23, ear=24)      # [fix1] ear: staked fixing ears of the well (E3)   # kitf_part
import hingekit; PM.update(hingekit.PM3); PF.update(hingekit.PF3)          # [fix3-hinge] blade (kitm 31), idler (kitf 41): open-stop / root / spring kit
import hingekit4, prismkit      # [fix4-hinge2] geared spring / damper drive (pinion 42, cartridge 43 in hingekit.PF3); closed, oriented kit parts
import wellkit4      # [fix4-int] closed solids of the well kit (G5 leftovers: pocket, TPE lip)
hingekit.DESIGNED3.update(hingekit4.DESIGNED4)      # [fix4-hinge2] lidsim: tongue teeth x pinion = the gear mesh (the pinion turns with the lid)
ORIENT = [False]      # [fix4-hinge2] PBag.add winds every added part consistently and outward (prismkit.orient) when lid.h4 is on

DEFAULTS = dict(
    gap=0.0008, open_deg=75.0, edge_r=0.0004, hole_r=0.0004, n_roll=4, fine=0.0004,     # shut line: gap, rolled A-edge radii (lid / opening)
    t_lid=0.0022, t_cover=0.0025, cover_wall=True, chamfer=0.0003, undercut_deg=20.0,  # walls; edge faces lean back under the rolled edge (dark gap)
    axis_dx=-0.003, axis_dz=0.001, hinge_y=None,                                         # virtual axis Q rel. to the lid's top rear edge
    arc_r=0.017, arc_t=0.003, arc_w=0.008, arc_a=[-140.0, -12.0], guide_a=[-146.0, -22.0], web_a=-17.0,          # [v18b] compact (eng_lid v18: R 25, -129..-8, -132..-15, web -12)
    guide_wall=0.0013, guide_cheek=0.0013, clr_r=0.0002, clr_a=0.0003, boss_r=[0.0125, 0.026], boss_w=0.010,
    lug_a=-133.0, lug_d=0.002, lug_clr=0.0003, over_deg=1.5, bumper=0.0015,
    band_w=0.004, band_t=0.0001, exit_a=-57.0, coil=[0.0313, -63.43, 0.0045], spring_N=4.2, damper=[0.004, 0.007],   # soft-open: constant-force spring + damper
    pod=True, pod_wall=0.0012, pod_gap=0.0003, pod_round=0.002, pod_edge=0.0015, ceil_clr=0.0006,                                   # [v18b] hinge pod in front of the dam; stepped pocket ceiling
    rib_in=[0.0070, 0.0084], rib_depth=0.0075, rib_t=0.0012, rib_h=0.004, ring_off=[0.024, 0.048],
    gutter_floor=[0.0070, 0.0085], gutter_wall=0.0015, dam=[0.0025, 0.0045], dam_top=0.0052,
    seal_d=0.004, seal_c=0.0035, latch_dx=-0.009, latch_over=0.0015, latch_rotor=[0.0036, -0.0024, -0.0015],   # [v18b] striker 9 mm in from the front edge, rotor R3.6 (eng_lid 11.5, R4.4)
    floor_gap=0.0035, floor_t=0.002, lip=0.0015, drains=[0.0035, 0.0025])


def undercuts(p):
    """Horizontal set-back of the bottom of the lid edge face (inboard) and of the opening's edge face (outboard)."""
    tb = np.tan(np.radians(p['undercut_deg'])); return (p['t_lid'] - p['edge_r'] - p['chamfer']) * tb, (p['t_cover'] - p['hole_r']) * tb

def params(L):
    p = dict(DEFAULTS); p.update({k: v for k, v in L.items() if v is not None})
    for k in ('gap', 'open_deg'): p[k] = float(L.get(k, DEFAULTS[k]))
    return p


# ---------------------------------------------------------------------------------------------------------------- 2D / mesh helpers
class PBag(Bag):
    """Bag with a part id per face."""
    def __init__(s): super().__init__(); s.P = []
    def add(s, v, f, lab, part=0):
        if ORIENT[0] and len(f): v, f = prismkit.orient(v, f)      # [fix4-hinge2]
        n0 = len(s.F); super().add(v, f, lab)
        if len(s.F) > n0: s.P.append(np.full(len(s.F[-1]), part, np.int16))
    def out(s):
        v, f, l = super().out(); return v, f, l, (np.concatenate(s.P) if s.P else np.zeros(0, np.int16))

def resample(ring, step):
    ring = ring.exterior if hasattr(ring, 'exterior') else ring
    n = max(8, int(np.ceil(ring.length / step))); s = np.linspace(0, ring.length, n, endpoint=False)
    return shapely.get_coordinates(shapely.line_interpolate_point(ring, s))

def lnormals(P):
    """Left normals (mitred) of a closed polyline. CCW outline -> inward; CW hole -> into the material."""
    t = np.roll(P, -1, 0) - P; t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-15); n1 = np.c_[-t[:, 1], t[:, 0]]; n0 = np.roll(n1, 1, 0)
    m = n0 + n1; m /= np.maximum(np.linalg.norm(m, axis=1, keepdims=True), 1e-15); c = np.clip((m * n1).sum(1), 0.5, None); return m / c[:, None]

def tri_polygon(pg):
    """Constrained Delaunay triangulation of a polygon (holes allowed): points (k, 2), triangles (m, 3). Uses only the input vertices."""
    T = shapely.get_parts(shapely.constrained_delaunay_triangles(pg))
    C = shapely.get_coordinates(shapely.get_exterior_ring(T)).reshape(-1, 4, 2)[:, :3].reshape(-1, 2)
    pts, inv = np.unique(C, axis=0, return_inverse=True); return pts, inv.reshape(-1, 3)

RES = [1.0]                                                                   # kit resolution scale (2 for the light web model)
def sector(c, r0, r1, a0, a1, da=None):
    """Annular sector about c (x, z) between radii r0..r1 and angles a0..a1 (deg, 0 = +x, positive = up)."""
    da = 0.75 * RES[0] if da is None else da; n = max(2, int(abs(a1 - a0) / da) + 1); a = np.radians(np.linspace(a0, a1, n))
    return Polygon(np.r_[np.c_[c[0] + r1 * np.cos(a), c[1] + r1 * np.sin(a)], np.c_[c[0] + r0 * np.cos(a[::-1]), c[1] + r0 * np.sin(a[::-1])]])

def dirv(a): a = np.radians(a); return np.array([np.cos(a), np.sin(a)])

def extrude(pg, w0, w1, frame, bev=0.0, nb=3, step=None, ends=(True, True)):
    """Prism of the polygon pg (coordinates (u, v), holes allowed) from w0 to w1 along w. frame = (origin, eu, ev, ew).
    Each end can get a rounded edge of radius bev (quarter circle, nb segments). Returns V, F (closed)."""
    o, eu, ev, ew = [np.asarray(a, float) for a in frame]
    pg = orient(pg, 1.0)
    if step: pg = shapely.segmentize(pg, step)
    loops = [np.asarray(pg.exterior.coords)[:-1]] + [np.asarray(r_.coords)[:-1] for r_ in pg.interiors]
    loops = [L_[np.r_[True, np.linalg.norm(np.diff(L_, axis=0), axis=1) > 1e-9]] for L_ in loops]
    nrm = [lnormals(L_) for L_ in loops]
    ph = np.radians(np.linspace(0, 90, nb + 1)) if bev > 0 else np.array([np.pi / 2])
    b0 = bev if ends[0] else 0.0; b1 = bev if ends[1] else 0.0
    prof = [(w0 + b0 * (1 - np.cos(q)), b0 * (1 - np.sin(q))) for q in (ph if b0 > 0 else [np.pi / 2])]
    prof += [(w1 - b1 * (1 - np.cos(q)), b1 * (1 - np.sin(q))) for q in (ph[::-1] if b1 > 0 else [np.pi / 2])]
    to3 = lambda uv, w: o + uv[:, :1] * eu + uv[:, 1:2] * ev + w * ew
    Vs, Fs, n = [], [], 0; ring0 = []; ringN = []
    for L_, N_ in zip(loops, nrm):
        R = [to3(L_ + N_ * ins, w) for w, ins in prof]
        v, f = stack(R, closed=True); Vs.append(v); Fs.append(f + n)
        ring0.append((n, L_ + N_ * prof[0][1])); ringN.append((n + (len(prof) - 1) * len(L_), L_ + N_ * prof[-1][1])); n += len(v)
    for rings, w, flip in ((ring0, prof[0][0], True), (ringN, prof[-1][0], False)):
        cap = Polygon(rings[0][1], [r_[1] for r_ in rings[1:]])
        if not cap.is_valid: cap = cap.buffer(0)
        pts, tri = tri_polygon(cap); allp = np.concatenate([r_[1] for r_ in rings]); base = np.concatenate([np.arange(len(r_[1])) + r_[0] for r_ in rings])
        dd, ii = cKDTree(allp).query(pts); ok = dd < 1e-9
        idx = np.where(ok, base[ii], -1); extra_ = np.nonzero(~ok)[0]
        if len(extra_): idx[extra_] = n + np.arange(len(extra_)); Vs.append(to3(pts[extra_], w)); n += len(extra_)
        Fs.append(idx[tri][:, ::-1] if flip else idx[tri])
    return np.concatenate(Vs), np.concatenate(Fs)

XZ = lambda: (np.zeros(3), (1.0, 0, 0), (0, 0, 1.0), (0, 1.0, 0))           # profile in (x, z), extruded along y
XY = lambda: (np.zeros(3), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0))           # plan profile, extruded along z


# --------------------------------------------------------------------------------------------------------------------------- build
def build(L, poly, zi, here='.'):
    """L: params(...) dict; poly: lid outline (plan); zi: A-surface height z(x, y) (roof before the rolled edges)."""
    p = L; KM, KF = PBag(), PBag(); rep = {}; RES[0] = float(p.get('kit_res', 1.0))
    gap, tl, tc, OPEN = p['gap'], p['t_lid'], p['t_cover'], p['open_deg']
    pg = orient(poly, 1.0); x_rear, _, x_front, hy = poly.bounds
    P2 = resample(pg, 0.0005 * RES[0]); NI = lnormals(P2)                                   # outline samples, inward normals
    rc = max(float(p.get('r', 0.006)), 1e-4); OFF = {}
    def ring(d, dz=None, z=None):
        xy = P2 + NI * d
        if d > 0.9 * rc:                                                                         # offset deeper than the corner radius: no swallowtail,
            if d not in OFF: OFF[d] = pg.buffer(-d, join_style=2).buffer(-0.001, join_style=2).buffer(0.001, quad_segs=8).exterior   # round the corner (R 1)
            xy = shapely.get_coordinates(shapely.line_interpolate_point(OFF[d], shapely.line_locate_point(OFF[d], shapely.points(xy))))
        return np.c_[xy, zi(xy) - dz] if z is None else np.c_[xy, np.full(len(xy), z)]
    # ---- geometry that sets the hinge plane: straight part of the rear edge, pocket width
    w, t, R, c, ca, wgw, wch = p['arc_w'], p['arc_t'], p['arc_r'], p['clr_r'], p['clr_a'], p['guide_wall'], p['guide_cheek']
    gw_half = w / 2 + ca + wch; wp = 2 * gw_half + 0.001; pt = 0.0015
    y_str = float(np.abs(P2[(NI[:, 0] > 0.9998) & (P2[:, 0] < x_rear + 0.002), 1]).max())   # rear edge straight up to |y| = y_str
    yc = p['hinge_y'] if p['hinge_y'] else min(0.058, y_str - wp / 2 - pt - 0.0005, hy - p['dam'][1] - gw_half - 0.001)
    zq = float(zi(np.array([[x_rear, yc]]))[0]); Q = np.array([x_rear + p['axis_dx'], zq + p['axis_dz']])
    rep.update(Q_world_mm=np.round(Q * 1000, 2).tolist(), axis_rel_mm=[p['axis_dx'] * 1000, p['axis_dz'] * 1000], hinge_y_mm=round(yc * 1000, 2), open_deg=OPEN)
    r_in, r_out = R - t / 2, R + t / 2; a0, a1 = p['arc_a']; g0, g1 = p['guide_a']
    co = c + p['band_t'] + 0.0001; rbnd = r_out + c + p['band_t'] / 2                       # outer running clearance holds the spring band
    C3 = hingekit.setup(p, Q, rbnd) if p.get('h3') else None                              # [fix3-hinge] tape / idler / drum layout (sets p['coil'])
    C4 = hingekit4.cfg4(p) if (C3 and p.get('h4')) else None      # [fix4-hinge2] gears instead of tape / idler / drum (hingekit4)
    ORIENT[0] = bool(C4)      # [fix4-hinge2]
    if C4: hingekit4.setup4(p, Q, C4); C4['spiral'] = hingekit4.spiral_of(C4, OPEN)      # [fix4-hinge2]
    ul, uc = undercuts(p)
    if not p['cover_wall']: uc = float(p.get('gutter_setback', 0.0))           # the shell builds the opening wall (vertical): gutter registers on it
    #   [fix1] L4 / E3: gutter_setback > 0 sets the gutter's outer wall that far OUTBOARD of the cover's edge face (a designed shadow step under the
    #   edge face's inner round instead of two coincident walls 0.09-0.2 mm apart); its top flange laps under the cover's inner skin (bonded / staked)
    lr = p['lug_d'] / 2; hl = np.degrees(lr / R); hs = np.degrees((lr + p['lug_clr']) / R)
    as0 = p['lug_a'] - p['over_deg'] - hl; as1 = p['lug_a'] + OPEN + hl + np.degrees(p['bumper'] / R)
    # [v18b] hinge pod (fixed, in front of the dam): annulus about Q from r_p0 to r_p1 up to the guide end g1, filled down under the
    # outer arc to below the coil drum. Its footprint is cut out of the well floor.
    POD = bool(p['pod']); pw, pgp, prr = p['pod_wall'], p['pod_gap'], p['pod_round']
    ri0g, ro1g = r_in - c - wgw, r_out + co + wgw; xw = x_rear + p['dam'][1]
    gw_pod = gw_half + pgp + pw if POD else gw_half; r_p1 = ro1g + pgp + pw
    x_pf = Q[0] + r_p1 * np.cos(np.radians(g1)) if POD else Q[0]; z_pt = Q[1] + r_p1 * np.sin(np.radians(g1))
    PODFP = shapely.union_all([shapely.box(xw, sg * yc - gw_pod, x_pf, sg * yc + gw_pod) for sg in (1, -1)]) if POD else Polygon()
    g1r = g1 - 0.6 if POD else g1                                                             # guide rails end just inside the pod mouth
    if C4: hingekit4.layout(C4, gw_half, gw_pod)      # [fix4-hinge2] cartridge outboard / damper inboard offsets

    # ================================================================================== MOVING (kitm): the lid moulding and the tongues
    ch = p['chamfer']
    ER_ = p.get('edge_ring') if C4 else None      # [fix4-hinge2] closed lid: the chamfer starts on the edge wall's bottom ring (design_gen)
    if ER_ is not None:      # [fix4-hinge2]
        xe0_ = ER_[:, :2]; ne0_ = lnormals(xe0_)      # [fix4-hinge2]
        if not pg.contains(Point(*(xe0_[0] + ne0_[0] * 0.001))): ne0_ = -ne0_      # [fix4-hinge2]
        xe1_ = prismkit.defold(xe0_, xe0_ + ne0_ * ch); Ru = np.c_[xe1_, zi(xe1_) - tl]; KM.add(*stack([ER_, Ru]), K_D, PM['underside']); pgu = Polygon(Ru[:, :2]); bb = pgu.bounds; h = 0.004      # [fix4-hinge2]
    else:      # [fix4-hinge2]
        KM.add(*stack([ring(ul, tl - ch), ring(ul + ch, tl)]), K_D, PM['underside'])                     # bottom edge break of the lid wall
        Ru = ring(ul + ch, tl); pgu = Polygon(Ru[:, :2]); bb = pgu.bounds; h = 0.004
    gx, gy = np.meshgrid(np.arange(bb[0], bb[2], h), np.arange(bb[1], bb[3], h)); ip = np.c_[gx.ravel(), gy.ravel()]
    ip = ip[shapely.contains_xy(pgu.buffer(-0.6 * h), ip[:, 0], ip[:, 1])]
    HOLES_ = p.get('skin_holes') or []      # [fix3-stop] round holes through the lid skin (STOP key stem bore): grid cleared round them, rim points added
    HL_ = [np.asarray(l_, float) for l_ in (p.get('skin_hole_loops') or [])] if C4 else []      # [fix4-hinge2] the skin's real hole loops: rim points of the underside + bore walls
    if HL_: HOLES_ = []      # [fix4-hinge2]
    for l_ in HL_: ip = np.concatenate([ip[~shapely.contains_xy(Polygon(l_[:, :2]).buffer(0.6 * h), ip[:, 0], ip[:, 1])], l_[:, :2]])      # [fix4-hinge2]
    for hx_, hy_, hr_ in HOLES_: ip = np.concatenate([ip[np.hypot(ip[:, 0] - hx_, ip[:, 1] - hy_) > hr_ + 0.6 * h], np.c_[hx_ + hr_ * np.cos(np.linspace(0, 2 * np.pi, 96, endpoint=False)), hy_ + hr_ * np.sin(np.linspace(0, 2 * np.pi, 96, endpoint=False))]])      # [fix3-stop]
    p2 = np.concatenate([Ru[:, :2], ip]); tri = Delaunay(p2).simplices; cen = p2[tri].mean(1)
    kun_ = shapely.contains_xy(pgu, cen[:, 0], cen[:, 1])
    for hx_, hy_, hr_ in HOLES_: kun_ &= np.hypot(cen[:, 0] - hx_, cen[:, 1] - hy_) > hr_      # [fix3-stop]
    for l_ in HL_: kun_ &= ~shapely.contains_xy(Polygon(l_[:, :2]), cen[:, 0], cen[:, 1])      # [fix4-hinge2]
    KM.add(np.c_[p2, np.r_[Ru[:, 2], zi(ip) - tl]], tri[kun_], K_D, PM['underside'])   # skin underside
    for l_ in HL_: KM.add(*stack([l_, np.c_[l_[:, :2], zi(l_[:, :2]) - tl]]), K_D, PM['underside'])      # [fix4-hinge2] bore wall through the skin
    # ribs: section (s across, h below the skin underside), rounded tip, 1 deg draft; they reach 0.3 mm into the skin
    def rib_sec(tr, hr, n=6):
        tb = tr - 2 * hr * np.tan(np.radians(1.0)); q = np.linspace(np.pi, 2 * np.pi, n)
        return [(-tr / 2, 0.0003)] + [(tb / 2 * np.cos(a_), -(hr - tb / 2) + tb / 2 * np.sin(a_)) for a_ in q] + [(tr / 2, 0.0003)]
    a_r, b_r = p['rib_in']; prs = rib_sec(b_r - a_r, p['rib_depth'] - tl)
    # keep-out zones of the hinges (guides, bosses, damper units) for the ribs
    def keepout(xy):
        bad = np.zeros(len(xy), bool)
        for sg in (1, -1):
            bad |= (xy[:, 0] < Q[0] + max(p['boss_r'][1], x_pf - Q[0]) + 0.003) & (np.abs(xy[:, 1] - sg * yc) < gw_pod + 0.0012)
        if p.get('rib_keepout') is not None: bad |= shapely.contains_xy(p['rib_keepout'], xy[:, 0], xy[:, 1])      # [fix3-stop] STOP key bore, arms, springs
        return bad
    sec = rib_sec(p['rib_t'], p['rib_h'])
    def rib_path(path, closed=False, sec=sec):
        path = np.asarray(path, float); seg = LineString(np.r_[path, path[:1]] if closed else path)
        q = resample(seg, 0.0006 * RES[0]) if closed else shapely.get_coordinates(shapely.line_interpolate_point(seg, np.linspace(0, seg.length, max(2, int(seg.length / 0.0006)) + 1)))
        bad = keepout(q)
        if closed and not bad.any(): runs = [np.r_[q, q[:1]]]
        else:
            if closed: k0 = int(np.argmax(bad)); q = np.roll(q, -k0, 0); bad = np.roll(bad, -k0)
            runs = []; cur = []
            for qi, bi in zip(q, bad):
                if bi:
                    if len(cur) > 2: runs.append(np.array(cur))
                    cur = []
                else: cur.append(qi)
            if len(cur) > 2: runs.append(np.array(cur))
        for rn in runs:
            tg = np.gradient(rn, axis=0)
            if p.get('closed_well_kit') and len(runs) == 1 and closed and not bad.any():      # [fix4-int] G5 (lid.rib#7): a whole ring = periodic tangents, so the
                tg = 0.5 * (np.roll(rn[:-1], -1, 0) - np.roll(rn[:-1], 1, 0)); tg = np.r_[tg, tg[:1]]      # last section repeats the first one (was one-sided: open seam)
            tg /= np.linalg.norm(tg, axis=1, keepdims=True); nn = np.c_[-tg[:, 1], tg[:, 0]]; zz = zi(rn) - tl
            Rr = np.array([np.c_[rn + nn * s_, zz + h_] for s_, h_ in sec]).transpose(1, 0, 2)          # (path point, section point, 3)
            cap = not (closed and len(runs) == 1 and not bad.any())
            KM.add(*stack(Rr, closed=True, cap0=cap, cap1=cap), K_D, PM['rib'])
    if POD:                                                                                 # [v18b] perimeter rib (skirt), interrupted over the hinge pods
        pp_ = orient(pg.buffer(-(a_r + b_r) / 2, join_style=1), 1.0); rib_path(np.asarray(pp_.exterior.coords)[:-1], closed=True, sec=prs)
    else: KM.add(*stack([ring((a_r + b_r) / 2 + s_, tl - h_) for s_, h_ in prs]), K_D, PM['rib'])            # perimeter rib (skirt)
    rings_ = []
    for off in p['ring_off']:                                                               # concentric ring ribs: same outline, same corners
        rp_ = pg.buffer(-off, join_style=1)
        if rp_.is_empty or rp_.area < 1e-4: continue
        rr_ = min(p.get('r', 0.006), 0.5 * min(rp_.bounds[2] - rp_.bounds[0], rp_.bounds[3] - rp_.bounds[1]) - 0.001)
        rp_ = orient(rp_.buffer(-rr_, join_style=2).buffer(rr_, quad_segs=16), 1.0); rings_.append(rp_); rib_path(np.asarray(rp_.exterior.coords)[:-1], closed=True)
    # radial ribs: from the perimeter rib through the rings along the corner bisectors, the rear centre line and the side centre lines
    cxm = 0.5 * (x_rear + x_front); dirs = [(-1, 0)] + [(sx, sy) for sx in (1, -1) for sy in (1, -1)] + [(0, 1), (0, -1)]
    V2 = p.get('rib_layout') == 'v2'
    if V2: dirs = [(-1, 0), (1, 0)] + [(sx, sy) for sx in (1, -1) for sy in (1, -1)] + [(0, 1), (0, -1)]      # [fix2] G9: mirrored front / back
    outer = pg.buffer(-b_r); bnds = [outer] + rings_
    for dx_, dy_ in dirs:
        if dx_ and dy_:                                                                     # corner bisector through the corner of the outline
            P_ = P2[np.argmax(P2 @ np.array([dx_, dy_]))]; o_ = P_ - 0.1 * np.array([dx_, dy_])
            ray = LineString([P_ + 0.01 * np.array([dx_, dy_]), o_])
        else:
            o_ = np.array([cxm, 0.0]); ray = LineString([o_, o_ + 0.3 * np.array([dx_, dy_])])
        pts_ = []
        for b_ in bnds:
            hit = shapely.get_coordinates(ray.intersection(b_.exterior))
            if len(hit): pts_.append(hit[np.argmin(np.linalg.norm(hit - np.asarray(ray.coords)[0], axis=1))])
        for k_ in range(len(pts_) - 1):
            if V2 and k_ == 0 and dx_ != 0: continue      # [fix2] G9: diagonals and the front / rear centre ribs only between the two closed rings (pods / striker outside)
            d_ = pts_[k_ + 1] - pts_[k_]; L__ = np.linalg.norm(d_); u_ = d_ / L__
            rib_path([pts_[k_] - u_ * 0.0004, pts_[k_ + 1] + u_ * 0.0004])
    # hinge bosses (moulded wedges whose underside is a plane through the axis Q) with the tongues insert-moulded into them
    zu = lambda x, y: float(zi(np.array([[x, y]]))[0]) - tl
    TONGUE = None
    for sg in (1, -1):
        y0 = sg * yc; ra, rb = p['boss_r']; wa = p['web_a']
        xa = Q[0] + ra * np.cos(np.radians(wa)); xb = Q[0] + rb * np.cos(np.radians(wa))
        xs_ = np.linspace(xb, xa, 12); top = np.c_[xs_, [zu(x_, y0) + 0.0003 for x_ in xs_]]
        prof = Polygon(np.r_[[Q + ra * dirv(wa), Q + rb * dirv(wa)], top]).buffer(-0.0004).buffer(0.0004, quad_segs=6)
        if not C3: KM.add(*extrude(prof, y0 - p['boss_w'] / 2, y0 + p['boss_w'] / 2, XZ(), bev=0.0006), K_D, PM['boss'])      # [fix3-hinge] C3: keel + blade, added after the pods
        # stainless arc tongue (fine-blanked, polished edges): round tail, square root insert-moulded into the boss
        tg_ = sector(Q, r_in, r_out, a0, a1).union(Point(*(Q + R * dirv(a0))).buffer(t / 2, quad_segs=12))
        TONGUE = tg_
        if C4: KM.add(*hingekit4.tongue(p, Q, C4, tg_, y0, XZ()), K_ARM, PM['tongue'])      # [fix4-hinge2] sector gear in the middle band
        if C4: C4['tongue_removed_m2'] = float(tg_.area - C4['tongue_t_area'])      # [fix4-hinge2]
        else: KM.add(*extrude(tg_, y0 - w / 2, y0 + w / 2, XZ(), bev=0.0004, nb=3), K_ARM, PM['tongue'])          # [v18b] dark PVD stainless
        yl0 = y0 + sg * (w / 2 - 0.0002); yl1 = y0 + sg * (w / 2 + ca + wch * 0.85)               # stop lug, pressed into the tongue
        lp_ = Q + R * dirv(p['lug_a'])      # [fix3-hinge]
        if C3: hingekit.lugs(KM, p, Q, R, sg, y0, w, ca, wch, K_ARM, PM['lug'], dirv)            # [fix3-hinge] two integral MIM lugs, both cheeks
        else: KM.add(*tube((lp_[0], yl0, lp_[1]), (lp_[0], yl1, lp_[1]), lr, 20), K_ARM, PM['lug'])      # [fix3-hinge]
    # striker: moulded boss + stainless loop, caught by the rotary catch of the push-push latch
    xs = x_front + p['latch_dx']; zul = zu(xs, 0.0)
    # [v18b] latch housing plan: a soft boss growing out of the front wall (R5 plan corners), just deep enough for the rotor
    xw_f = x_front - p['dam'][1] - 0.00005; hyl = 0.008; wl_ = 0.0015; rR_, rdx_, rdz_ = p['latch_rotor']; hx0 = xs + rdx_ - rR_ - 0.0005 - wl_
    LATCH_PLAN = shapely.box(hx0, -hyl, xw_f + 0.006, hyl).buffer(-0.005, join_style=2).buffer(0.005, quad_segs=12).intersection(shapely.box(hx0, -hyl, xw_f, hyl))
    zt = zul - (p['rib_depth'] - tl) - p['latch_over'] - 0.001; zbar = zt - 0.005            # housing top clears the perimeter rib at full over-travel
    KM.add(*extrude(shapely.box(xs - 0.003, -0.0065, xs + 0.0022, 0.0065), zul - 0.0015, zul + 0.0003, XY(), bev=0.0005, ends=(True, False)), K_D, PM['striker'])
    rb_ = 0.0015; yl = 0.004; q_ = np.linspace(0, np.pi / 2, 7)
    pth = [(-yl, zul - 0.001)] + [(-yl + rb_ - rb_ * np.cos(a_), zbar + rb_ - rb_ * np.sin(a_)) for a_ in q_]
    pth += [(yl - rb_ + rb_ * np.sin(a_), zbar + rb_ - rb_ * np.cos(a_)) for a_ in q_] + [(yl, zul - 0.001)]
    pth = np.array(pth); pth = np.concatenate([np.linspace(pth[i], pth[i + 1], max(2, int(np.linalg.norm(pth[i + 1] - pth[i]) / 0.0004)) + 1)[:-1] for i in range(len(pth) - 1)] + [pth[-1:]])
    KM.add(*sweep(np.c_[np.full(len(pth), xs), pth[:, 0], pth[:, 1]], rrect(0.002, 0.002, 0.00099, 6), up=(1, 0, 0)), K_ARM, PM['striker'])

    # ================================================================================== FIXED (kitf): cover edge, well moulding, mechanism
    if p['cover_wall']:
        KF.add(*stack([ring(-gap - uc, tc), ring(-gap - uc - 0.004, tc)]), K_D, PF['cover'])                   # inside face of the cover wall at the opening
    gf0, gf1 = p['gutter_floor']; gwl = p['gutter_wall']; d0, d1 = p['dam']; dt = p['dam_top']
    # control panel outline (fixed, chassis) -> well floor frame + skirt lip
    C = np.load(os.path.join(here, 'control.npz')); cv_ = C['v']; base = cv_[np.unique(C['f'][C['lab'] == 0])]
    pan = orient(MultiPoint(base[:, :2]).convex_hull, 1.0); PC = resample(pan, 0.0008 * RES[0]); NC = lnormals(PC)
    ktop = cKDTree(base[:, :2]); ztop = np.array([base[ix, 2].max() for ix in ktop.query_ball_point(PC + NC * p['lip'], 0.0008)])
    z_wf = float(base[:, 2].max()) + p['floor_gap']; ft = p['floor_t']
    zpf = Q[1] - (r_out + co + wgw) - 0.001                                                      # pocket floor (top face)
    seg_mid = 0.5 * (P2 + np.roll(P2, -1, 0))
    win = np.zeros(len(P2), bool)
    for sg in (1, -1): win |= (seg_mid[:, 0] < x_rear + 0.010) & (np.abs(seg_mid[:, 1] - sg * yc) < wp / 2)
    # drains at the four corners of the gutter (front: spouts, rear: weeps); the gutter floor falls to the front with the roof
    sarc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(np.r_[P2, P2[:1]], axis=0), axis=1))]; Ltot = sarc[-1]
    drain = np.zeros(len(P2), bool); DR = []
    for dx_ in (1, -1):
        for dy_ in (1, -1):
            j0 = int(np.argmax(P2 @ np.array([dx_, dy_]))); rd = p['drains'][0] if dx_ > 0 else p['drains'][1]
            sm_ = 0.5 * (sarc[:-1] + sarc[1:]); ds_ = np.abs((sm_ - sarc[j0] + Ltot / 2) % Ltot - Ltot / 2); drain |= ds_ < rd / 2 + 0.0004
            dc_ = 0.5 * (d0 - gap - uc); DR.append((P2[j0] + NI[j0] * dc_, rd))
    rw = lambda d, dz=None, z=None: ring(d, dz, z)
    dcv = -gap - uc; p0, p1, p2_, p3 = rw(dcv - gwl, tc + 0.00005), rw(dcv, tc + 0.00005), rw(dcv, gf0), rw(d0, gf0)
    p4, p5, p5b, p5c, p6 = rw(d0, dt), rw(d1, dt), rw(d1, gf1), rw(d1, z=zpf), rw(d1, z=z_wf)
    q0, q0b, q1, q2 = rw(d0, z=z_wf - ft), rw(d0, z=zpf), rw(d0, gf1), rw(dcv - gwl, gf1)
    for Rs, sk in (([p0, p1, p2_], None), ([p2_, p3], drain), ([p3, p4, p5, p5b], None), ([p5b, p5c], win), ([p5c, p6], None),
                   ([q0, q0b], None), ([q0b, q1], win), ([q1, q2], drain), ([q2, p0], None)):
        KF.add(*stack(Rs, skip=sk), K_D, PF['well'])
    hole_ = Polygon(PC - NC * p['lip'])
    if POD:          # [v18b] the pod's front face runs down to the floor's hole edge (no 1 mm floor sliver between pod and panel lip)
        hb_ = np.asarray(hole_.exterior.coords); xpf2 = {}
        for sg in (1, -1):
            m_ = (np.abs(hb_[:, 1] - sg * yc) < gw_pod) & (hb_[:, 0] < Q[0] + 0.05)
            xpf2[sg] = max(x_pf, float(hb_[m_, 0].min()) - 0.0003) if m_.any() else x_pf
        PODFP = shapely.union_all([shapely.box(xw, sg * yc - gw_pod, xpf2[sg] + 0.002, sg * yc + gw_pod) for sg in (1, -1)])
        if C4:      # [fix4-hinge2] housings: round top round the shaft, square below to just under the well floor; front stops 0.3 behind the floor's hole edge
            mh_ = (np.abs(np.abs(hb_[:, 1]) - yc) < 0.02) & (hb_[:, 0] < Q[0] + 0.05); C4['z_bot'] = min(z_wf - p['floor_t'] - 0.0005, C4['Cp'][1] - C4['L']['r_h'] - 0.0002)      # [fix4-hinge2] pod + housing floor: under the housing bore's 1.0 wall
            C4['H'] = hingekit4.housings(C4, xw + 0.00005, (float(hb_[mh_, 0].min()) if mh_.any() else x_pf + 0.002) - 0.0003, C4['z_bot'])      # [fix4-hinge2]
            for sg in (1, -1):      # [fix4-hinge2] one housing INBOARD of each pod (cartridge + damper)
                b_ = C4['H']['cart'].bounds; y_ = sorted((sg * yc + sg * C4['L']['e_k2'], sg * yc - sg * gw_pod)); PODFP = PODFP.union(shapely.box(b_[0], y_[0], b_[2], y_[1]))      # [fix4-hinge2]
    WEEP_, WP_ = Polygon(), []      # [fix4-stop2] S7: TPE duckbill weeps in the well floor frame (front corners) -> body cavity -> deck drains
    if p.get('well_weep'):      # [fix4-stop2] [hole r, inset from the frame edges, y min]: the front-most frame point on each side
        rwp_, ins_, ywp_ = p['well_weep']; fr_ = Polygon(p6[:, :2], [np.asarray(hole_.exterior.coords)[:-1][::-1]]).difference(PODFP.union(LATCH_PLAN)).buffer(-(rwp_ + ins_))
        cw_ = shapely.get_coordinates(shapely.get_rings(shapely.get_parts(fr_))) if not fr_.is_empty else np.zeros((0, 2))
        for sg in (1, -1):
            m_ = sg * cw_[:, 1] > ywp_
            if m_.any(): WP_.append(tuple(cw_[m_][int(np.argmax(cw_[m_, 0]))]))
        WEEP_ = shapely.union_all([Point(c_).buffer(rwp_, quad_segs=8) for c_ in WP_]) if WP_ else Polygon()      # [fix4-stop2]
    for zz_, rr_ in ((z_wf, p6), (z_wf - ft, q0)):                                          # floor frame round the control panel (pods cut out)
        for fp_ in shapely.get_parts(Polygon(rr_[:, :2], [np.asarray(hole_.exterior.coords)[:-1][::-1]]).difference(PODFP.union(LATCH_PLAN).union(WEEP_))):      # [fix4-stop2] WEEP_
            pts, tri = tri_polygon(fp_); KF.add(np.c_[pts, np.full(len(pts), zz_)], tri, K_D, PF['well'])
    KF.add(*stack([np.c_[PC - NC * p['lip'], np.full(len(PC), z_wf)], np.c_[PC - NC * p['lip'], np.full(len(PC), z_wf - ft)]]), K_D, PF['well'])
    lipR = [np.c_[PC - NC * p['lip'], np.full(len(PC), z_wf - ft)], np.c_[PC - NC * 0.0002, ztop + 0.0012], np.c_[PC + NC * p['lip'], ztop + 0.00003]]
    if p.get('closed_well_kit'): KF.add(*wellkit4.lip_solid(PC, NC, [-p['lip'], -0.0002, p['lip']], np.c_[np.full(len(PC), z_wf - ft), ztop + 0.0012, ztop + 0.00003], 0.0006), K_RUB, PF['lip'])      # [fix4-int] G5: 0.6 thick, closed
    else: KF.add(*stack(lipR), K_RUB, PF['lip'])                                                     # TPE skirt lip resting on the panel top
    for c_ in WP_:      # [fix4-stop2] weep hole wall + TPE duckbill (flange on the floor, neck in the d 3 hole, bill closed to a slit 4 mm under the floor)
        cr_ = np.asarray(Point(c_).buffer(rwp_, quad_segs=8).exterior.coords)[:-1]; KF.add(*stack([np.c_[cr_, np.full(len(cr_), z_wf)], np.c_[cr_, np.full(len(cr_), z_wf - ft)]], closed=True), K_D, PF['well'])
        KF.add(*lathe((c_[0], c_[1], z_wf + 0.0006), (0, 0, -1), [(0, 0.0009), (0, 0.0026), (0.0006, 0.0026), (0.0006, rwp_ - 0.00002), (0.0006 + ft, rwp_ - 0.00002),
                                                                  (0.0006 + ft + 0.004, 0.0006), (0.0006 + ft + 0.0036, 0.0002), (0.0006 + ft, 0.0009), (0, 0.0009)], 32), K_RUB, PF['drain'])      # closed solid
    if WP_: rep['well_weeps_mm'] = dict(at=[[round(c_[0] * 1000, 1), round(c_[1] * 1000, 1)] for c_ in WP_], hole_d=round(2 * rwp_ * 1000, 1), floor_z=round(z_wf * 1000, 2),
                                        bill_bottom_z=round((z_wf + 0.0006 - 0.0006 - ft - 0.004) * 1000, 2), note='[fix4-stop2] S7: water past the boot / lip seal leaves the well through 2 TPE duckbills into the body cavity (deck drains); the gutter is ~14 mm higher, so it cannot drain there')      # [fix4-stop2]
    for (cxy, rd) in DR:                                                                      # drain spouts under the gutter corners
        zt_ = float(zi(cxy[None])[0]) - gf0; r0 = rd / 2
        KF.add(*lathe((cxy[0], cxy[1], zt_ - 0.00005), (0, 0, -1), [(0, r0), (0, 0.0028), (0.0006, 0.0028), (0.0006, r0 + 0.0008), (0.010, r0 + 0.0008), (0.010, r0), (0, r0)], 32), K_D, PF['drain'])
    # continuous hollow bulb seal on the dam crest, compressed 1 mm by the lid underside (drawn in its compressed shape)
    hs_ = dt - tl; q = np.linspace(0, 2 * np.pi, 28, endpoint=False); cq, sq = np.cos(q), np.sin(q)
    bul = [rw(p['seal_c'] + 0.5 * p['seal_d'] * np.sign(a_) * abs(a_) ** 0.75, (dt + tl) / 2 - (0.5 * hs_ - 0.00003) * np.sign(b_) * abs(b_) ** 0.55) for a_, b_ in zip(cq, sq)]
    KF.add(*stack(bul + [bul[0]], closed=True), K_RUB, PF['seal'])
    rep.update(seal_compression_mm=round((p['seal_d'] - hs_) * 1000, 2), seal_continuous=True, well_floor_z_mm=round(z_wf * 1000, 2))
    # ---- hinge: guides, pockets, stop bumpers, soft-open units
    for sg in (1, -1):
        y0 = sg * yc; ya, yb = sorted((y0 - w / 2 - ca, y0 + w / 2 + ca))
        yi = sorted((y0 - sg * (w / 2 + ca), y0 - sg * gw_half)); yo = sorted((y0 + sg * (w / 2 + ca), y0 + sg * gw_half))
        ri0, ri1, ro0, ro1 = r_in - c - wgw, r_in - c, r_out + co, r_out + co + wgw; ea = p['exit_a']; bw = p['band_w']
        if C4: KF.add(*prismkit.slab_union([(sector(Q, ri0, ri1, g0, g1r), ya, yb)], XZ(), bev=0.0, nb=2), K_GUIDE, PF['guide'])      # [fix4-hinge2] G5: closed inner rail liner (sharp-edged)
        else: KF.add(*extrude(sector(Q, ri0, ri1, g0, g1r), ya, yb, XZ(), bev=0.0002, nb=2), K_GUIDE, PF['guide'])      # [fix4-hinge2] else-branch = f3
        rs0_, rs1_ = C3['rail_split'] if C3 else (-2.0, 12.0)                                    # [fix3-hinge] tape slot between two narrow outer rails round the idler
        if C4:      # [fix4-hinge2] G5: the outer rail as ONE closed solid (slabs along y: full arc | arc with the pinion slot | full arc)
            an_ = np.unique(np.r_[np.linspace(g0, ea + rs0_, 60), np.linspace(ea + rs0_, ea + rs1_, 30), np.linspace(ea + rs1_, g1r, 60)])      # shared arc vertices
            arc_ = lambda a_: Polygon(np.r_[np.c_[Q[0] + ro1 * np.cos(np.radians(a_)), Q[1] + ro1 * np.sin(np.radians(a_))], np.c_[Q[0] + ro0 * np.cos(np.radians(a_[::-1])), Q[1] + ro0 * np.sin(np.radians(a_[::-1]))]])
            ofull_ = arc_(an_); oslot_ = shapely.MultiPolygon([arc_(an_[an_ <= ea + rs0_ + 1e-9]), arc_(an_[an_ >= ea + rs1_ - 1e-9])])
            hb4_ = C4['gear']['band'] / 2      # the slot = the tongue's toothed band (pinion + 0.3 each side)
            KF.add(*prismkit.slab_union([(ofull_, ya, y0 - hb4_), (oslot_, y0 - hb4_, y0 + hb4_), (ofull_, y0 + hb4_, yb)], XZ(), bev=0.0, nb=2), K_GUIDE, PF['guide'])      # sharp-edged liner (the 0.2 round folded at the arc ends at full resolution)
        for s0_, s1_, yy_ in (() if C4 else ((g0, ea + rs0_, [(ya, yb)]), (ea + rs0_, ea + rs1_, [(ya, y0 - bw / 2 - 0.0003), (y0 + bw / 2 + 0.0003, yb)]), (ea + rs1_, g1r, [(ya, yb)]))):      # [fix3-hinge] [fix4-hinge2] not with C4
            for y_0, y_1 in yy_: KF.add(*extrude(sector(Q, ro0, ro1, s0_, s1_), y_0, y_1, XZ(), bev=0.0002, nb=2), K_GUIDE, PF['guide'])
        chk_i = sector(Q, ri0, ro1, g0, g1r)
        slot = sector(Q, R - lr - p['lug_clr'], R + lr + p['lug_clr'], as0, as1)
        if C3: slot = hingekit.slot3(Q, R, lr + p['lug_clr'], as0, as1, sector)      # [fix3-hinge] semicircular (conformal) slot ends
        chk_o = sector(Q, ri0, ro1, g0, g1r).difference(slot)
        LB3 = bool(C3 and C3['lug_both']); KCH = K_ARM if (C3 and C3['cheek_metal']) else K_GUIDE      # [fix3-hinge] slots in both cheeks; stainless cassette plates
        if C4: chk_o = prismkit.clean(chk_o.union(hingekit4.lobe(C4)))      # [fix4-hinge2] plate lobes carry the pinion shaft journals
        EXP_ = (lambda pg_, yy_: prismkit.slab_union([(prismkit.clean(pg_), yy_[0], yy_[1])], XZ(), bev=0.0003, nb=2)) if C4 else (lambda pg_, yy_: extrude(pg_, *yy_, XZ(), bev=0.0003, nb=2))      # [fix4-hinge2] G5: closed plates
        KF.add(*EXP_(chk_o if LB3 else chk_i, yi), KCH, PF['guide'])      # [fix3-hinge] [fix4-hinge2] EXP_
        KF.add(*EXP_(chk_o, yo), KCH, PF['guide'])      # [fix3-hinge] [fix4-hinge2] EXP_
        bum = sector(Q, R - lr - p['lug_clr'], R + lr + p['lug_clr'], as1 - np.degrees(p['bumper'] / R), as1)
        if C3: bum = hingekit.pad3(Q, R, lr, slot, p['lug_a'] + OPEN, as1, sector)      # [fix3-hinge] TPE crescent between the lug at open_deg and the steel slot end
        if C4: bum = hingekit4.pad4(Q, R, lr, p['lug_clr'], slot, p['lug_a'] + OPEN, as1, sector, C4)      # [fix4-hinge2] + flank liner and detent lips
        for yy_ in ((yo, yi) if LB3 else (yo,)):      # [fix3-hinge]
            KF.add(*extrude(bum, yy_[0] + 0.00005, yy_[1] - 0.00005, XZ()), K_RUB, PF['bumper'])            # TPE open-stop bumper in the slot end      # [fix3-hinge]
        # soft-open: constant-force spring band (pinned to the tongue tail, running in the outer clearance) leaves the guide at exit_a
        # through the slot between the two outer rails and winds onto a coil drum; a one-way rotary damper sits on the drum axle
        rd_, ad_, rh_ = p['coil']; Cd = Q + rd_ * dirv(ad_); Pe = Q + rbnd * dirv(ea)
        if C3: KF3_ = KF; KF = PBag()                                                          # [fix3-hinge] C3: the f2 band / drum below go to a scratch bag
        ba = np.radians(np.linspace(ea - 6, ea, 9)); arcp = np.c_[Q[0] + rbnd * np.cos(ba), Q[1] + rbnd * np.sin(ba)]
        u_ = (Cd - Pe) / np.linalg.norm(Cd - Pe); Pin = Cd - u_ * (rh_ - 0.0002)
        bpath = np.r_[arcp, [Pe + (Pin - Pe) * k_ for k_ in np.linspace(0.15, 1.0, 6)]]
        nrm_ = np.gradient(bpath, axis=0); nrm_ = np.c_[-nrm_[:, 1], nrm_[:, 0]] / np.linalg.norm(nrm_, axis=1, keepdims=True); bt2 = p['band_t'] / 2
        KF.add(*stack(np.stack([np.c_[bpath[:, 0] + nrm_[:, 0] * e1, np.full(len(bpath), y0 + e2), bpath[:, 1] + nrm_[:, 1] * e1] for e1, e2 in ((-bt2, -bw / 2), (bt2, -bw / 2), (bt2, bw / 2), (-bt2, bw / 2))], 1), closed=True, cap0=True, cap1=True), K_STEEL, PF['spring'])
        hw_ = bw / 2 + 0.00075; KF.add(*lathe((Cd[0], y0 - hw_, Cd[1]), (0, 1, 0), [(0, 0), (0, rh_ - 0.0004), (0.0004, rh_), (2 * hw_ - 0.0004, rh_), (2 * hw_, rh_ - 0.0004), (2 * hw_, 0)], 48), K_GLOSS, PF['damper'])
        if C4: KF = KF3_; hingekit4.drive_unit(KF, p, Q, sg, y0, C4, dict(pinion=K_ARM, cart=K_STEEL, spring=K_STEEL, damper=K_GLOSS), PF, XZ(), gw_half, gw_pod)      # [fix4-hinge2]
        elif C3: KF = KF3_; hw_ = hingekit.spring_unit(KF, p, Q, sg, y0, rbnd, C3, dict(tape=K_GREY, idler=K_GREY, steel=K_STEEL, drum=K_GLOSS), PF, extrude, XZ)      # [fix3-hinge] tape + idler + spiral-spring drum
        dr_, dl_ = p['damper']; yd0 = y0 - sg * (hw_ if not C4 else 0.0)      # [fix4-hinge2]
        DMP_ = KF if not C4 else PBag()      # [fix4-hinge2] C4: the f3 one-way damper + axle below go to a scratch bag
        DMP_.add(*lathe((Cd[0], yd0, Cd[1]), (0, -sg, 0), [(0, 0), (0, dr_ - 0.0003), (0.0003, dr_), (0.55 * dl_, dr_), (0.55 * dl_ + 0.0002, dr_ - 0.0004), (0.62 * dl_, dr_ - 0.0004),
                                                     (0.62 * dl_ + 0.0002, dr_), (dl_ - 0.0004, dr_), (dl_, dr_ - 0.0004), (dl_, 0.0018), (dl_ + 0.0005, 0.0014), (dl_ + 0.0005, 0)], 48), K_GLOSS, PF['damper'])
        DMP_.add(*tube((Cd[0], y0 + sg * (hw_ + 0.0004), Cd[1]), (Cd[0], y0 + sg * hw_, Cd[1]), 0.0012, 20), K_STEEL, PF['damper'])     # axle end in the mounting ear      # [fix4-hinge2] DMP_
        # pocket behind the well rear wall, under the gutter: a closed box open only into the well (inside the seal line)
        xb_ = min(sector(Q, ri0, ro1, g0, g1).bounds[0], Q[0] + R * np.cos(np.radians(a0)) - t / 2) - 0.001
        zc = float(zi(np.array([[x_rear, y0]]))[0]) - gf1; y_a, y_b = y0 - wp / 2, y0 + wp / 2
        xg = x_rear - gap - uc - gwl                                                              # outer face of the gutter's outer wall
        if POD:                                                                                   # [v18b] behind the gutter the ceiling steps up to just under the cover's inside face
            GX, GY = np.meshgrid(np.linspace(xb_ - pt, xg, 12), np.linspace(y_a - pt, y_b + pt, 5)); zA_ = float(np.nanmin(zi(np.c_[GX.ravel(), GY.ravel()])))
            zch = max(zA_ - tc - p['ceil_clr'] - pt, zc)                                         # highest plate top: cover inside face less ceil_clr
        else: zch = zc
        BX_ = ((xb_ - pt, xw, y_a - pt, y_b + pt, zpf - pt, zpf), (xb_ - pt, xb_, y_a - pt, y_b + pt, zpf, zch + pt), (xb_, xg, y_a - pt, y_b + pt, zch, zch + pt),
               (xb_, xg, y_a - pt, y_a, zpf, zch), (xb_, xg, y_b, y_b + pt, zpf, zch), (xg, xw, y_a - pt, y_a, zpf, zc), (xg, xw, y_b, y_b + pt, zpf, zc))
        if p.get('closed_well_kit'): KF.add(*wellkit4.boxes_solid(BX_, XZ()), K_D, PF['pocket'])      # [fix4-int] G5: ONE closed pocket (was 7 touching boxes: non-manifold)
        else:
            for bx_ in BX_:
                KF.add(*extrude(shapely.box(bx_[0], bx_[4], bx_[1], bx_[5]), bx_[2], bx_[3], XZ()), K_D, PF['pocket'])      # [v18b] same dark interior colour as the well
        rep.update(pocket_ceiling_mm=[round((Q[1] - zch) * 1000, 2), round((Q[1] - zc) * 1000, 2)])
        if POD:
            # hinge pod: one moulding with the well, in its dark colour. Profile (x, z): annulus r_p0..r_p1 about Q below the guide end g1,
            # plus everything under the outer arc down to zpb; inner arc meets the dam just above the window top; R prr on the free corners.
            zwt = float(zi(np.array([[xw, y0]]))[0]) - gf1
            r_p0 = min(ri0g - pgp - pw, float(np.hypot(xw - Q[0], Q[1] - zwt - 0.0002)))
            zpb = Cd[1] - rh_ - 0.0005 - pw if not C4 else C4['z_bot']      # [fix4-hinge2] C4: pod floor flush with the housings
            Dp1 = Point(*Q).buffer(r_p1, quad_segs=192); Dp0 = Point(*Q).buffer(r_p0, quad_segs=192)
            ann = Dp1.difference(Dp0).intersection(sector(Q, 1e-6, 0.2, -180.0, g1, 0.25))
            xf_ = xpf2[sg]; zf_ = z_pt                                                            # front face reaches the floor hole edge
            low = shapely.box(xw - 0.003, zpb, x_pf, z_pt).difference(Dp1).union(shapely.box(x_pf - 0.003, zpb, xf_, zf_))
            prof = ann.union(low).intersection(shapely.box(xw - 0.003, zpb, xf_, Q[1]))
            prof = prof.buffer(-prr, join_style=1).buffer(prr, quad_segs=8).intersection(shapely.box(xw + 0.00005, zpb, xf_ + 0.001, Q[1]))
            chan = sector(Q, r_in - c, r_out + co, -180.0, g1 + 20.0, 0.25)                    # tongue channel (open at the mouth)
            yc_ = w / 2 + ca
            slabs = [(prof, -gw_pod, -yc_), (prof.difference(chan), -yc_, yc_), (prof.difference(slot), yc_, gw_half), (prof, gw_half, gw_pod)]
            if C3 and C3['lug_both']: slabs = [(prof, -gw_pod, -gw_half), (prof.difference(slot), -gw_half, -yc_)] + slabs[1:]      # [fix3-hinge] inboard lug slot
            if C3:      # [fix3-hinge] the pod is a housing round the guide cassette: no pod material inside the cassette envelope (no coincident faces)
                CAS3 = sector(Q, ri0g, ro1g, g0, g1r)      # [fix3-hinge]
                slabs = [(pg_.difference(CAS3) if max(abs(e0), abs(e1)) <= gw_half + 1e-9 else pg_, e0, e1) for pg_, e0, e1 in slabs]      # [fix3-hinge]
            if C4: C4.setdefault('pod_built', []).append(hingekit4.pod(KF, p, Q, sg, y0, C4, prof, chan, slot, CAS3, gw_half, gw_pod, yc_, K_D, PF['pod'], XZ(), C4['H'])); slabs = []      # [fix4-hinge2] ONE closed pod
            for pg_, e0, e1 in slabs:                                                             # e: offsets along sg (outboard +)
                ya_, yb_ = sorted((y0 + sg * e0, y0 + sg * e1)); outer = (abs(e0) == gw_pod, abs(e1) == gw_pod)
                en_ = (outer[0], outer[1]) if sg > 0 else (outer[1], outer[0])
                for part_ in shapely.get_parts(pg_):
                    if part_.area > 1e-8: KF.add(*extrude(part_, ya_, yb_, XZ(), bev=p['pod_edge'] if any(en_) else 0.0, nb=4, ends=en_), K_D, PF['pod'])
            # under the well floor: closed box for the drum's damper (reaches inboard of the pod)
            dr_, dl_ = p['damper']; hw2_ = hw_ if C3 else bw / 2 + 0.00075; yin_ = y0 - sg * (hw2_ + dl_ + 0.001 + pw)      # [fix3-int] C3: the box follows the real drum half-width (the damper sits on it)
            yA_, yB_ = sorted((yin_, y0 - sg * (gw_pod - 0.001))); rr_m = max(rh_, dr_) + 0.0005 + pw
            if not C4: KF.add(*extrude(shapely.box(max(xw + 0.00005, Cd[0] - rr_m), zpb, min(x_pf, Cd[0] + rr_m), z_wf - ft - 0.00005), yA_, yB_, XZ(), bev=0.0005, nb=2), K_D, PF['pod'])      # [fix4-hinge2] no damper box
            rep.update(pod=dict(r_mm=[round(r_p0 * 1000, 2), round(r_p1 * 1000, 2)], x_front_rel_Q_mm=round((x_pf - Q[0]) * 1000, 2), x_foot_rel_Q_mm=round((xpf2[1] - Q[0]) * 1000, 2), half_width_mm=round(gw_pod * 1000, 2),
                                bottom_below_Q_mm=round((Q[1] - zpb) * 1000, 2), mouth_deg=g1))
            if C3:      # [fix3-hinge] lid side of the hinge: steel root blade (MIM, with the tongue) in a deep plastic keel that ends in ring rib 2
                BL3, KE3, kr3 = hingekit.blade_keel(p, Q, zu, y0, max(xpf2[sg], x_pf), p['boss_r'][0], p['web_a'])      # [fix3-hinge]
                EXQ_ = (lambda pg_, w0_, w1_, bev, nb=3: prismkit.slab_union([(prismkit.clean(pg_), w0_, w1_)], XZ(), bev=bev, nb=nb)) if C4 else (lambda pg_, w0_, w1_, bev, nb=3: extrude(pg_, w0_, w1_, XZ(), bev=bev, nb=nb))      # [fix4-hinge2] G5: closed, oriented keel / blade / spine (the 1e-7 slivers of the unions left them open)
                KM.add(*EXQ_(KE3, y0 - C3['keel']['w'] / 2, y0 + C3['keel']['w'] / 2, 0.0006), K_D, PM['boss'])      # [fix3-hinge] [fix4-hinge2] EXQ_
                KM.add(*EXQ_(BL3, y0 - w / 2, y0 + w / 2, 0.0003, 2), K_ARM, PM['blade'])      # [fix3-hinge] [fix4-hinge2] EXQ_
                SP3 = kr3.pop('spine'); C3['_spine'] = SP3      # [fix3-hinge]
                if SP3 is not None: KM.add(*EXQ_(SP3, y0 - C3['spine']['t'] / 2, y0 + C3['spine']['t'] / 2, 0.0004, 2), K_D, PM['rib'])      # [fix3-hinge] spine rib [fix4-hinge2] EXQ_
                rep.setdefault('h3_root', {})[str(sg)] = {k_: round((v_ - Q[0]) * 1000, 2) for k_, v_ in kr3.items()}      # [fix3-hinge]
    # ---- push-push latch on the front well wall: hollow housing with a mouth on top; inside, a rotary catch (steel rotor on a pin)
    #      drawn in its LATCHED pose: the slot is tilted 27 deg from horizontal, its upper prong lies over the striker bar. Pressing the
    #      lid (over-travel) turns the rotor on its heart-cam track until the slot points up and the bar lifts out (push-push).
    zbt = zt - 0.0129                                                                          # (the floor frame is cut round the housing: it rises out of floor and wall)
    rr2 = lambda b_, r_: b_.buffer(-r_, join_style=2).buffer(r_, quad_segs=10)
    outer_ = LATCH_PLAN; mouth_ = rr2(shapely.box(xs - 0.0025, -0.006, xs + 0.0025, 0.006), 0.0008)
    if p.get('closed_well_kit'):      # [fix4-int] G5: the latch housing as ONE closed solid (floor / hollow walls / top with the mouth as z-slabs; was 3 stacked open prisms)
        KF.add(*prismkit.slab_union([(outer_, zbt, zbt + wl_), (outer_.difference(rr2(shapely.box(hx0 + wl_, -hyl + wl_, xw_f - wl_, hyl - wl_), 0.0035)), zbt + wl_, zt - wl_),
                                     (outer_.difference(mouth_), zt - wl_, zt)], XY(), bev=(0.0005, 0.001), nb=4), K_D, PF['latch'])
    else:
        KF.add(*extrude(outer_.difference(mouth_), zt - wl_, zt, XY(), bev=0.001, nb=5, ends=(False, True)), K_D, PF['latch'])
        KF.add(*extrude(outer_.difference(rr2(shapely.box(hx0 + wl_, -hyl + wl_, xw_f - wl_, hyl - wl_), 0.0035)), zbt + wl_, zt - wl_, XY()), K_D, PF['latch'])
        KF.add(*extrude(outer_, zbt, zbt + wl_, XY(), bev=0.0005, ends=(True, False)), K_D, PF['latch'])
    B_ = np.array([xs, zbar]); P_ = B_ + np.array([rdx_, rdz_]); u_ = (B_ - P_) / np.linalg.norm(B_ - P_)
    rotor = Point(*P_).buffer(rR_, quad_segs=24).difference(LineString([B_, B_ + u_ * 0.008]).buffer(0.0012, quad_segs=12))
    rotor = max(shapely.get_parts(rotor), key=lambda g_: g_.area).difference(Point(*P_).buffer(0.0007, quad_segs=12))
    if p.get('closed_well_kit'): KF.add(*prismkit.slab_union([(prismkit.clean(rotor), -0.001, 0.001)], XZ(), bev=0.0002, nb=2), K_STEEL, PF['claw'])      # [fix4-int] G5: closed rotor (the extrude caps missed ring points)
    else: KF.add(*extrude(rotor, -0.001, 0.001, XZ(), bev=0.0002, nb=2), K_STEEL, PF['claw'])
    KF.add(*tube((P_[0], -hyl + wl_, P_[1]), (P_[0], hyl - wl_, P_[1]), 0.0007, 16), K_STEEL, PF['claw'])
    rep.update(latch_x_mm=round(xs * 1000, 2), striker_bar_z_mm=round(zbar * 1000, 2), latch_over_mm=p['latch_over'] * 1000)
    # ---- mass, centre of mass, gravity torque about Q (moving group, closed pose; skin from the outline, ribs / bosses from their sections)
    rho_pl, rho_st = 1150.0, 7900.0; A_ = poly.area
    xs_c, zs_c = poly.centroid.x, float(zi(np.array([[poly.centroid.x, 0.0]]))[0]) - tl / 2
    m_skin = A_ * tl * rho_pl; per = pg.exterior.length; m_prib = per * (b_r - a_r) * (p['rib_depth'] - tl) * rho_pl
    m_rings = sum(r_.exterior.length for r_ in rings_) * p['rib_t'] * p['rib_h'] * rho_pl
    m_rad = 7 * 0.018 * p['rib_t'] * p['rib_h'] * rho_pl; m_boss = 2 * 0.5 * (p['boss_r'][1] - p['boss_r'][0]) * 0.004 * p['boss_w'] * rho_pl
    m_tong = 2 * TONGUE.area * w * rho_st; m_str = 0.0015
    parts = [(m_skin, xs_c, zs_c), (m_prib + m_rings + m_rad, xs_c, zs_c - 0.004), (m_boss + m_tong, Q[0] + 0.012, Q[1] - 0.012), (m_str, xs, zbar)]
    parts += [tuple(a_) for a_ in (p.get('extra_mass') or [])]      # [fix3-int] parts fixed to the lid built elsewhere (STOP key: stopkit.mass_mesh) -> one lid balance (mass, gravity, h3 spring)
    M = sum(a_[0] for a_ in parts); cx_ = sum(a_[0] * a_[1] for a_ in parts) / M; cz_ = sum(a_[0] * a_[2] for a_ in parts) / M
    th = np.radians(np.array([0, 15, 30, 45, 60, OPEN])); tq = M * 9.81 * ((cx_ - Q[0]) * np.cos(th) - (cz_ - Q[1]) * np.sin(th))
    rep.update(spring=dict(band_N_per_side=p['spring_N'], band_r_mm=round(rbnd * 1000, 2), torque_Nm_total=round(2 * p['spring_N'] * rbnd, 4), exit_a_deg=ea,
                       coil_centre_rel_Q_mm=np.round((Q + p['coil'][0] * dirv(p['coil'][1]) - Q) * 1000, 1).tolist(), band_travel_mm=round(np.radians(OPEN) * rbnd * 1000, 1)), undercut_mm=[round(ul * 1000, 3), round(uc * 1000, 3)])
    rep.update(mass_g=round(M * 1000, 1), mass_parts_g=dict(skin=round(m_skin * 1e3, 1), ribs=round((m_prib + m_rings + m_rad) * 1e3, 1), bosses=round(m_boss * 1e3, 1), tongues=round(m_tong * 1e3, 1)),
               com_rel_Q_mm=[round((cx_ - Q[0]) * 1000, 1), round((cz_ - Q[1]) * 1000, 1)],
               gravity_torque_Nm={str(int(round(a_))): round(float(b_), 4) for a_, b_ in zip(np.degrees(th), tq)},
               lever_latch_mm=round((xs - Q[0]) * 1000, 1),
               stop=dict(lug_a_deg=p['lug_a'], slot_deg=[round(as0, 2), round(as1, 2)], open_stop_deg=OPEN, bottom_stop_deg=-p['over_deg']),
               guide_engagement_deg=dict(closed=round(min(a1 - 0, g1) - max(a0, g0), 1), open=round(min(a1 + OPEN, g1) - max(a0 + OPEN, g0), 1)))
    if C3: rep['h3'] = hingekit.report(p, Q, R, C3, parts, poly, zi, BL3, KE3, rbnd, OPEN, 2 if C3['lug_both'] else 1); rep['spring'] = dict(superseded_by='h3 (tape + spiral-spring drum)')      # [fix3-hinge]
    if C4:      # [fix4-hinge2] ONE lid report (m10): the geared drive balance (hingebal) on the h3 mass model, which then also feeds the top-level keys
        ex_ = [a_ for a_ in (p.get('extra_mass') or [])]; me_ = sum(a_[0] for a_ in ex_)
        rep['_extra_com_rel_Q'] = ((sum(a_[0] * a_[1] for a_ in ex_) / me_ - Q[0]), (sum(a_[0] * a_[2] for a_ in ex_) / me_ - Q[1])) if me_ > 0 else (0.126, -0.0025)
        rep['_L_edge'] = float(x_front - Q[0]); rep['h3']['_lug_eng'] = C3['lug_eng']
        H4_ = hingekit4.report(p, Q, C4, rep['h3'], rep, OPEN, C4.get('tongue_removed_m2', 0.0)); rep.pop('_extra_com_rel_Q'); rep.pop('_L_edge'); rep['h3'].pop('_lug_eng')
        H4_['pods'] = [dict(housing_bounds_mm=np.round(np.array(b_) * 1e3, 2).tolist(), y_mm=np.round(np.array(y_) * 1e3, 2).tolist()) for pb_ in C4.get('pod_built', []) for b_, y_ in pb_]
        H4_['detent_engage_deg_before_stop'] = round(C4.get('detent_engage_deg', 0.0), 2)
        rep['h4'] = H4_
        rep['h3'] = dict(mass_new_g=rep['h3'].get('mass_new_g'), root=rep.get('h3_root'),
                         superseded_by='h4 (geared drive): the tape / drum spring table of fix-hinge no longer applies; mass / COM / inertia / gravity of the ONE lid: '
                                       'the top-level keys = lid.h4 (hingekit.report mass model incl. the STOP parts, less the tongue teeth)')
        rep.pop('mass_g'); rep['mass_g'] = H4_['mass_g']; rep['com_rel_Q_mm'] = H4_['com_rel_Q_mm']; rep['gravity_torque_Nm'] = H4_['gravity_Nm']; rep['inertia_kgm2'] = H4_['inertia_kgm2']
        rep['mass_parts_g'].update(keels_spine=rep['h3']['mass_new_g']['keels'], blades_lugs=rep['h3']['mass_new_g']['blades_lugs'],
                                   stop_key=round(me_ * 1e3, 1), tongue_teeth=H4_['mass_change_g']['tongue_teeth'])
        rep['spring'] = dict(superseded_by='h4 (tongue sector gear -> pinion -> spiral spring cartridge + two-way rotary damper)')
    return dict(Q=Q, open=OPEN, yc=yc, kitm=KM.out(), kitf=KF.out(), report=rep, z_wf=z_wf, **({'pinion_axis': [float(C4['Cp'][0]), float(C4['Cp'][1]), float(C4['gear']['z_w'] / C4['gear']['z_p'])]} if C4 else {}))      # [fix4-hinge2] pinion axis (x, z) + ratio for the QA tools / renderer

