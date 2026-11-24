# [fix3-stop] Raised, parallel-guided STOP key on the control-panel lid (v18 "Orbit", fix round 3, issue "STOP button compliance").
#
# Requirement (EN 50636-2-107:2015/A3:2021 cl. 20.101.4, aligned with IEC 60335-2-107:2017+A1:2020; marking 7.9):
#   single-action, clearly identifiable manual stop in a prominent position on the TOP surface; >= 20 % of the actuator surface raised
#   >= 5 mm above an immediate surrounding area that is >= 15 mm wide; geometric minor dimension of the actuator >= 35 mm; operating
#   force <= 30 N on any part of the raised surface; overrides all controls; restart only after two actions / 4-char code / prompts;
#   marked "STOP", coloured red, no other external control red.  A flush pad on a flush lid can never meet the 5 mm rule, so the
#   STOP becomes a real key ON the lid (the lid itself stays flush and keeps its push-push latch).
#
#  CAP       d 40 red 2K cap, white STOP inlay (Liberation Sans Bold, 28 mm wide, 7.4 mm caps) reading from the front, domed
#            top (0.4), R2.5 top roll. It floats `stroke + clear` over the lid on the HALO boss (d 34 opal light guide = the v18 status
#            halo, lit from the panel LED through the lid): no cup, no trough - rain runs off the cap onto the lid.
#  GUIDE     the cap's carrier (dark PA-GF, stem d 10) hangs on two PARALLEL links (L 70, 5.5 apart, stainless U-channels) on a tongue
#            under the lid: the cap translates without tilting -> the same force and the same stroke on every point of the cap
#            (a centre-guided d 40 cap would jam / need 2x the force at its edge: drawer rule L > 2 mu e).
#  RETURN    two torsion springs on the rear upper pin (2.5 N at rest, 0.4 N/mm at the cap) + silicone boot (0.1 N/mm).
#            up-stop: the upper arm on a TPE bumper under the lid bore boss; down-stop: the carrier's stop ring on the halo boss
#            (stroke 5.0; impacts go straight into the lid, never into the arms or the real key).
#  PLUNGER   in the stem a rod with a TPE tip, preloaded 16 N (1.6 N/mm) on a shoulder: rigid while it presses the real STOP key
#            (fixed hardware, assumed <= 15 N), it yields once that key bottoms -> the cap always reaches its own stop and the real
#            key never sees more than ~25 N.   Tip gap 1.45 nominal: never pressed by the stack (worst case -0.16), always pressed
#            before the cap stops (worst case 4.66 < 5.0).
#  SEAL      opal (translucent) silicone rolling boot between the halo-boss bore and the stem, lit through the light guide: the cap
#            floats on light; the d 15 bore is the only opening in the lid.
# Built in the CLOSED lid pose with the cap at rest; every part moves with the lid (design_render: object StopKey).
import os, json, numpy as np, shapely
from shapely.geometry import Polygon, Point
from scipy.spatial import Delaunay
from meshkit import Bag, stack, tube, sweep

PART = dict(cap=61, legend=62, carrier=63, boot=64, rod=65, cspring=66, tip=67, arm=68, pin=69, rspring=70, halo=71, bore=72, bracket=73, bumper=74, socket=75, bush=76, post=77, magnet=78, pcover=79)      # [fix4-stop2] 75-79: v4 parts (stop4.py)
NAME = {v: k for k, v in PART.items()}
MAT = ['stop', 'legend', 'kit', 'rubber', 'steel', 'arm', 'halo', 'lid']           # stop_lab -> renderer material
M_STOP, M_LEG, M_KIT, M_RUB, M_STEEL, M_ARM, M_HALO, M_LID = range(8)
MOVING = ('cap', 'legend', 'carrier', 'boot', 'rod', 'cspring', 'tip', 'arm', 'pin', 'rspring')    # move with the cap stroke (boot: deforms)

DEFAULTS = dict(
    x=None, y=0.0,                                       # axis; x None = centre of the real STOP key (bbox of its top)
    r=0.020, roll=0.0025, dome=0.0004, side=0.0011, wall=0.0016,     # cap: radius, top roll, dome, visible side, shell wall
    stroke=0.0050, clear=0.0005, gap=0.00145,            # cap stroke (to its stop), skirt clearance at full stroke, tip gap to the scanned key
    boss_r=0.017, bore_r=0.0075, boss_h=None, ring_r=[0.013, 0.016], ring_h=0.001,
    stem_r=0.005, stem_bore=0.0038, stem_above_key=0.007, bush=0.0025, rod_r=0.0015, head_r=0.0035, head_t=0.001, tip_r=0.004, tip_h=0.002,
    arm_L=0.070, arm_d=0.0055, arm_h=0.0028, arm_t=0.0008, arm_y=0.0053, arm_top=-0.0032, pin_r=0.0012, tongue_w=0.0050,
    bore_out=0.0095, bore_bot=-0.0030, bumper_t=0.0002,
    legend_w=0.028, glyphs='stop_glyphs.json', n=160,
    # forces (N, N/mm) and the tolerance model (mm) - see budget()
    F_ret0=2.5, k_ret=0.4, k_boot=0.1, F_c0=16.0, k_c=1.6, F_fric=0.8,
    key_act=[0.3, 1.5], key_full_min=0.8, key_F=[3.0, 15.0], key_load_max=50.0,
    stack=[['real key top vs panel datum (fixed hardware, supplier)', 0.30], ['panel / control tower on the chassis', 0.15],
           ['chassis posts -> body (14 screws)', 0.15], ['body seat pads -> snap-on cover', 0.15], ['cover -> well module (heat-staked)', 0.15],
           ['latch closed height at the striker x 0.79 (lever to the cap)', 0.12], ['hinge axis Q height x 0.21', 0.03],
           ['lid moulding: skin -> arm tongue pivots', 0.10], ['arms + carrier + rod + TPE tip', 0.15]],
    thermal=[-0.31, 0.41],                               # -10 .. +60 C: ~115 mm more plastic (ASA/PP, 9e-5 /K) in the cover path than in the tower path
    seal_k=[0.02, 0.05], latch_F=[3.0, 8.0],             # bulb seal over-compression (N/mm per mm of seal), push-push latch push force at the striker (N)
    colors=dict(stop='#C8102E', legend='#F4F4F2', lid='#CDD0CF'))


def params(SK):
    p = json.loads(json.dumps(DEFAULTS)); p.update({k: v for k, v in (SK or {}).items() if not str(k).startswith('//')}); return p


def key_info(here='.'):
    """The real STOP key (fixed hardware, scanned): bbox centre of its top, and the stand-in cap plane (as design_gen's controls stand-in)."""
    c = np.load(os.path.join(here, 'control.npz')); cv, cf, cl = c['v'], c['f'], c['lab']
    Pk = cv[np.unique(cf[cl == 1])]; P2 = Pk[Pk[:, 2] > 0.254]
    top = Pk[Pk[:, 2] > 0.2585]; A2 = np.c_[np.ones(len(top)), top[:, 0]]; cf2 = np.linalg.lstsq(A2, top[:, 2], rcond=None)[0]
    cf2[0] += float(np.max(Pk[:, 2] - (cf2[0] + cf2[1] * Pk[:, 0]))) + 0.00015
    return dict(P=Pk, cx=0.5 * (P2[:, 0].min() + P2[:, 0].max()), cy=0.5 * (P2[:, 1].min() + P2[:, 1].max()),
                bounds=[float(P2[:, 0].min()), float(P2[:, 1].min()), float(P2[:, 0].max()), float(P2[:, 1].max())], plane=cf2)


def layout(SK, zi, here='.'):
    """Axis, heights and the keep-outs the lid needs (skin hole for the stem bore, no ribs over the arms / springs)."""
    p = params(SK); K = key_info(here)
    xk = float(K['cx'] if p['x'] is None else p['x']); yk = float(p['y'])
    z0 = float(zi(np.array([[xk, yk]]))[0]); e_ = 0.004
    gx = float((zi(np.array([[xk + e_, yk]]))[0] - zi(np.array([[xk - e_, yk]]))[0]) / (2 * e_)); gy = float((zi(np.array([[xk, yk + e_]]))[0] - zi(np.array([[xk, yk - e_]]))[0]) / (2 * e_))
    q_ = np.linspace(0, 2 * np.pi, 72, endpoint=False); cq_ = np.c_[xk + p['r'] * np.cos(q_), yk + p['r'] * np.sin(q_)]
    dev = float(np.max(zi(cq_) - (z0 + gx * (cq_[:, 0] - xk) + gy * (cq_[:, 1] - yk))))      # lid above its tangent plane under the skirt (crown)
    nk = np.hypot(K['P'][:, 0] - xk, K['P'][:, 1] - yk) < p['tip_r']
    zkey = float(K['P'][nk, 2].max())                                       # scanned key under the tip footprint (letters included)
    xP = xk - p['arm_L']
    keep = shapely.union_all([Point(xk, yk).buffer(0.0125, quad_segs=16), shapely.box(xP - 0.007, yk - 0.016, xk, yk + 0.016)])
    return dict(p=p, K=K, xk=xk, yk=yk, z0=z0, grad=(gx, gy), dev=max(dev, 0.0), zkey=zkey, xP=xP, skin_holes=[[xk, yk, p['bore_r']]], rib_keepout=keep)


# ------------------------------------------------------------------------------------------------------------------ mesh helpers
def _ring(c, r, z, n):
    q = 2 * np.pi * np.arange(n) / n; return np.c_[c[0] + r * np.cos(q), c[1] + r * np.sin(q), np.full(n, z)]

def rev(c, z0, prof, n, closed_prof=False):
    """Solid / shell of revolution about the vertical axis through c. prof: [(r, h)] (h relative to z0).
    Open profile: first and last points must have the same h as their neighbour and r > 0 -> both ends are closed by flat fans."""
    R = [_ring(c, max(r_, 1e-6), z0 + h_, n) for r_, h_ in prof]
    if closed_prof: return stack(R + [R[0]], closed=True)
    return stack(R, closed=True, cap0=True, cap1=True)

def helix(c, z_a, z_b, D, d, turns, n_per=28, ang0=0.0):
    t = np.linspace(0, 1, max(8, int(turns * n_per)) + 1); q = ang0 + 2 * np.pi * turns * t
    P = np.c_[c[0] + D / 2 * np.cos(q), c[1] + D / 2 * np.sin(q), z_a + (z_b - z_a) * t]
    sec = np.c_[np.cos(np.linspace(0, 2 * np.pi, 10, endpoint=False)), np.sin(np.linspace(0, 2 * np.pi, 10, endpoint=False))] * d / 2
    return sweep(P, sec, up=(0, 0, 1))

def helix_y(c, y_a, y_b, D, d, turns, n_per=28):
    """Coil about an axis parallel to Y through c = (x, z)."""
    t = np.linspace(0, 1, max(8, int(turns * n_per)) + 1); q = 2 * np.pi * turns * t
    P = np.c_[c[0] + D / 2 * np.cos(q), y_a + (y_b - y_a) * t, c[1] + D / 2 * np.sin(q)]
    sec = np.c_[np.cos(np.linspace(0, 2 * np.pi, 10, endpoint=False)), np.sin(np.linspace(0, 2 * np.pi, 10, endpoint=False))] * d / 2
    return sweep(P, sec, up=(0, 1, 0))

def legend_polys(p, xk, yk, here='.'):
    """STOP glyph outlines on the cap top (world x, y): centred; front=True reads from in front of the mower (letters' up = -x, reading -> +y:
    the rear-hinged lid is opened and the panel operated from the front / side), front=False from behind (up +x, reading -> -y)."""
    G = json.load(open(os.path.join(here, p['glyphs']))); b = G['bounds']; s = p['legend_w'] / (b[2] - b[0]); uc, vc = 0.5 * (b[0] + b[2]), 0.5 * (b[1] + b[3])
    sg = -1.0 if p.get('legend_front', True) else 1.0
    T = lambda a: np.c_[xk + sg * (np.asarray(a)[:, 1] - vc) * s, yk - sg * (np.asarray(a)[:, 0] - uc) * s]
    pg = [Polygon(T(g['ext']), [T(h) for h in g['holes']]).buffer(0) for g in G['glyphs']]
    return shapely.union_all(pg), dict(width_mm=round(p['legend_w'] * 1000, 2), cap_height_mm=round((b[3] - max(b[1], 0.0)) * s * 1000, 2), font=G['font'])


def cap_mesh(p, c, z0, here='.', dev=0.0):
    """Closed cap shell: domed top disk (red, with the white STOP inlay cut in exactly) + roll + side + skirt + ceiling. Returns V, F, lab."""
    n = p['n']; r, ro, wl = p['r'], p['roll'], p['wall']; rf = r - ro
    h_sb = p['stroke'] + p['clear'] + dev; h_st = h_sb + p['side']; h_rim = h_st + ro; h_ceil = h_rim - wl
    prof = [(rf + ro * np.cos(a), h_rim - ro + ro * np.sin(a)) for a in np.radians(np.linspace(90, 0, 9))]
    prof += [(r, h_sb + wl / 2)] + [(r - wl / 2 + wl / 2 * np.cos(a), h_sb + wl / 2 + wl / 2 * np.sin(a)) for a in np.radians(np.linspace(0, -180, 9))[1:]]
    prof += [(r - wl, h_ceil)]
    R = np.array([_ring(c, r_, z0 + h_, n) for r_, h_ in prof]); Vl, Fl = stack(R, closed=True, cap1=True); Fl = Fl[:, ::-1]
    # top disk: boundary = ring 0 (indices 0..n-1); glyph outlines dense, interior grid away from the outlines
    LG, info = legend_polys(p, c[0], c[1], here)
    outl = []
    for g in getattr(LG, 'geoms', [LG]):
        for ring_ in [g.exterior] + list(g.interiors): outl.append(shapely.get_coordinates(shapely.segmentize(ring_, 0.00015))[:-1])
    OL = np.concatenate(outl); gx, gy = np.meshgrid(np.arange(-rf, rf, 0.0007), np.arange(-rf, rf, 0.0007)); G2 = np.c_[gx.ravel() + c[0], gy.ravel() + c[1]]
    rr = np.hypot(G2[:, 0] - c[0], G2[:, 1] - c[1]); G2 = G2[rr < rf - 0.0005]
    G2 = G2[shapely.distance(LG.boundary, shapely.points(G2)) > 0.0003]
    P2 = np.concatenate([Vl[:n, :2], OL, G2]); tri = Delaunay(P2).simplices
    cen = P2[tri].mean(1); tri = tri[np.hypot(cen[:, 0] - c[0], cen[:, 1] - c[1]) < rf]
    e1_ = P2[tri[:, 1]] - P2[tri[:, 0]]; e2_ = P2[tri[:, 2]] - P2[tri[:, 0]]; ar = e1_[:, 0] * e2_[:, 1] - e1_[:, 1] * e2_[:, 0]; tri[ar < 0] = tri[ar < 0][:, ::-1]
    leg = shapely.contains_xy(LG, *P2[tri].mean(1).T)
    rho = np.hypot(P2[n:, 0] - c[0], P2[n:, 1] - c[1]); zt = z0 + h_rim + p['dome'] * (1 - (rho / rf) ** 2)
    V = np.concatenate([Vl, np.c_[P2[n:], zt]]); idx = np.r_[np.arange(n), len(Vl) + np.arange(len(P2) - n)]
    F = np.concatenate([Fl, idx[tri]]); lab = np.r_[np.full(len(Fl), M_STOP), np.where(leg, M_LEG, M_STOP)]
    return V, F, lab, dict(h_skirt=h_sb, h_side_top=h_st, h_rim=h_rim, h_apex=h_rim + p['dome'], h_ceil=h_ceil, legend=info, n_legend_faces=int(leg.sum()))


def build(SK, zi, LP, Q, here='.'):
    """All STOP parts (closed lid, cap at rest). Returns dict(v, f, lab, part, report, L)."""
    import lidmech
    L = layout(SK, zi, here); p = L['p']; xk, yk, z0, zkey, xP = L['xk'], L['yk'], L['z0'], L['zkey'], L['xP']; c = (xk, yk); n = p['n']
    B = []                                                                      # (V, F, lab-array, part)
    def add(vf, mat, part):
        v, f = vf; B.append((np.asarray(v, float), np.asarray(f, np.int64), np.full(len(f), mat, np.int8), PART[part]))
    # ---- cap + legend
    Vc, Fc, Lc, H = cap_mesh(p, c, z0, here, L['dev'])
    B.append((Vc, Fc, Lc.astype(np.int8), PART['cap']))
    h_ring = H['h_ceil'] - p['ring_h']; h_boss = h_ring - p['stroke'] if p['boss_h'] is None else p['boss_h']
    # ---- halo boss (opal light guide, fused 1.6 into the lid skin) and the lid's stem bore (dark lid moulding) + TPE up-stop bumper
    add(rev(c, z0, [(p['bore_r'], -0.0016), (p['boss_r'], -0.0016), (p['boss_r'], h_boss - 0.0004), (p['boss_r'] - 0.0004, h_boss),
                    (p['bore_r'] + 0.0004, h_boss), (p['bore_r'], h_boss - 0.0004)], n, closed_prof=True), M_HALO, 'halo')
    add(rev(c, z0, [(p['bore_r'], -0.0015), (p['bore_out'], -0.0015), (p['bore_out'], p['bore_bot']), (p['bore_r'], p['bore_bot'])], n, closed_prof=True), M_KIT, 'bore')
    add(rev(c, z0, [(p['stem_r'] + 0.0012, p['bore_bot']), (p['bore_out'], p['bore_bot']), (p['bore_out'], p['bore_bot'] - p['bumper_t']),
                    (p['stem_r'] + 0.0012, p['bore_bot'] - p['bumper_t'])], n, closed_prof=True), M_RUB, 'bumper')
    # ---- carrier: stop ring + web under the cap ceiling, stem d 10 down to stem_above_key over the key, spring chamber, bottom bushing
    h_sb2 = zkey + p['stem_above_key'] - z0; h_bt = h_sb2 + p['bush']; r1, r2 = p['ring_r']
    add(rev(c, z0, [(r2, H['h_ceil'] + 0.0002), (r2, h_ring), (r1, h_ring), (r1, H['h_ceil'] - 0.0008), (p['stem_r'], H['h_ceil'] - 0.0008),
                    (p['stem_r'], h_sb2), (p['rod_r'] + 0.0002, h_sb2), (p['rod_r'] + 0.0002, h_bt), (p['stem_bore'], h_bt),
                    (p['stem_bore'], H['h_ceil'] - 0.0002), (0.0, H['h_ceil'] - 0.0002), (0.0, H['h_ceil'] + 0.0002)], n, closed_prof=True), M_KIT, 'carrier')
    # ---- boot (silicone, 0.6 wall): from the halo-boss bore rim up to a bead on the stem
    hb0 = h_boss; hb1 = h_boss + 0.0022
    add(rev(c, z0, [(p['bore_r'] + 0.0011, hb0), (p['bore_r'] + 0.0002, hb0 + 0.0011), (p['stem_r'] + 0.0012, hb1 - 0.0002), (p['stem_r'], hb1),
                    (p['stem_r'], hb1 + 0.0007), (p['stem_r'] + 0.0016, hb1 + 0.0004), (p['bore_r'] + 0.0008, hb0 + 0.0013), (p['bore_r'] + 0.0017, hb0)], n // 2, closed_prof=True), M_HALO, 'boot')      # opal silicone: lit by the halo light guide
    # ---- rod (head on the bushing), compliance spring (d 0.8 x D 6.5, preloaded), TPE tip
    h_tipb = zkey + p['gap'] - z0; h_tipt = h_tipb + p['tip_h']
    add(rev(c, z0, [(p['rod_r'], h_tipt - 0.0005), (p['rod_r'], h_bt), (p['head_r'], h_bt), (p['head_r'], h_bt + p['head_t']), (0.0, h_bt + p['head_t']), (0.0, h_tipt - 0.0005)], 40, closed_prof=True), M_STEEL, 'rod')
    sz0, sz1 = z0 + h_bt + p['head_t'], z0 + H['h_ceil'] - 0.0002
    add(helix(c, sz0 + 0.0004, sz1 - 0.0004, 0.0065, 0.0008, 11.2), M_STEEL, 'cspring')
    tq = np.radians(np.linspace(0, 90, 8))
    add(rev(c, z0, [(0.0, h_tipt), (p['tip_r'], h_tipt), (p['tip_r'], h_tipt - 0.0008)] + [(p['tip_r'] * np.cos(a), h_tipb + (p['tip_h'] - 0.0008) * (1 - np.sin(a))) for a in tq[1:]], 40, closed_prof=True), M_RUB, 'tip')
    # ---- parallel arms (PA66-GF30 plates either side of the stem / tongue), pins, tongue under the lid, torsion springs
    zu = z0 + p['arm_top'] - p['arm_h'] / 2; zl = zu - p['arm_d']; ya, yb = p['arm_y'], p['arm_y'] + p['arm_t']
    for zz, top_ in ((zu, True), (zl, False)):      # stainless U-channel links (0.8, 1.4310): two flanges + a web (top on the upper, bottom on the lower link)
        pg = shapely.box(xP - 0.0025, zz - p['arm_h'] / 2, xk + 0.0025, zz + p['arm_h'] / 2).buffer(-0.0012).buffer(0.0012, quad_segs=6)
        for s_ in (1, -1): add(lidmech.extrude(pg, *sorted((yk + s_ * ya, yk + s_ * yb)), lidmech.XZ(), bev=0.0002, nb=2), M_ARM, 'arm')
        zw_ = (zz + p['arm_h'] / 2 - p['arm_t'], zz + p['arm_h'] / 2) if top_ else (zz - p['arm_h'] / 2, zz - p['arm_h'] / 2 + p['arm_t'])
        add(lidmech.extrude(shapely.box(xP + 0.0055, zw_[0], xk - 0.0065, zw_[1]), yk - yb + 0.0001, yk + yb - 0.0001, lidmech.XZ()), M_ARM, 'arm')
    for zz in (zu, zl):
        add(tube((xP, yk - yb - 0.0006, zz), (xP, yk + yb + 0.0006, zz), p['pin_r'], 20), M_STEEL, 'pin')
        for s_ in (1, -1): add(tube((xk, yk + s_ * 0.0042, zz), (xk, yk + s_ * (yb + 0.0006), zz), p['pin_r'], 20), M_STEEL, 'pin')
    zus = float(zi(np.array([[xP, yk]]))[0]) - LP['t_lid']
    tg = shapely.box(xP - 0.004, zl - 0.0026, xP + 0.004, zus + 0.0003).buffer(-0.0015).buffer(0.0015, quad_segs=6)
    add(lidmech.extrude(tg, yk - p['tongue_w'], yk + p['tongue_w'], lidmech.XZ(), bev=0.0004, nb=2), M_KIT, 'bracket')
    for s_ in (1, -1):
        add(tube((xP, yk + s_ * (yb + 0.0006), zu), (xP, yk + s_ * (yb + 0.0072), zu), p['pin_r'], 20), M_STEEL, 'pin')
        y_a, y_b = yk + s_ * (yb + 0.0008), yk + s_ * (yb + 0.0064)
        add(helix_y((xP, zu), y_a, y_b, 0.0060, 0.0016, 3.0), M_STEEL, 'rspring')
        add(tube((xP + 0.0030, y_b, zu), (xP + 0.0100, yk + s_ * (yb + 0.0008), zu), 0.0008, 10), M_STEEL, 'rspring')       # leg on the arm (bent in to the plate)
        add(tube((xP - 0.0030, y_a, zu), (xP - 0.0030, y_a, zus + 0.0002), 0.0008, 10), M_STEEL, 'rspring')       # leg on the lid
    gx, gy = L['grad']; SHEAR = (PART['cap'], PART['halo'], PART['carrier'], PART['boot'])
    for v, f, l, pt in B:
        if pt in SHEAR: v[:, 2] += gx * (v[:, 0] - xk) + gy * (v[:, 1] - yk)      # z-shear onto the lid's tangent plane at the axis
    V = []; F = []; LB = []; PT = []; o = 0
    for v, f, l, pt in B: V.append(v); F.append(f + o); LB.append(l); PT.append(np.full(len(f), pt, np.int16)); o += len(v)
    V = np.concatenate(V); F = np.concatenate(F); LB = np.concatenate(LB); PT = np.concatenate(PT)
    # ---- report
    kp = L['K']['plane']; z_si = kp[0] + kp[1] * xk
    rep = dict(axis_mm=[round(xk * 1000, 2), round(yk * 1000, 2)], lid_A_surface_z_mm=round(z0 * 1000, 2), cap_d_mm=round(2 * p['r'] * 1000, 1),
               cap_top_above_lid_mm=round(H['h_apex'] * 1000, 2), cap_rim_above_lid_mm=round(H['h_rim'] * 1000, 2), skirt_above_lid_mm=round(H['h_skirt'] * 1000, 2),
               cap_top_z_mm=round((z0 + H['h_apex']) * 1000, 2), stroke_mm=p['stroke'] * 1000, halo_boss_top_mm=round(h_boss * 1000, 2),
               tip_gap_scan_mm=round(p['gap'] * 1000, 2), tip_gap_standin_mm=round((z0 + h_tipb - z_si) * 1000, 2), key_top_under_tip_mm=round(zkey * 1000, 2),
               stem_bottom_above_key_mm=round(p['stem_above_key'] * 1000, 2), arm_pivot_x_mm=round(xP * 1000, 2), arm_L_mm=p['arm_L'] * 1000, arm_spacing_mm=p['arm_d'] * 1000,
               arm_bottom_z_mm=round((zl - p['arm_h'] / 2) * 1000, 2), compliance_spring_installed_mm=round((sz1 - sz0) * 1000, 2), legend=H['legend'],
               link=link_check(p), lid_over_travel_at_cap_mm=None, lid_slope_deg=[round(np.degrees(np.arctan(gx)), 2), round(np.degrees(np.arctan(gy)), 2)], crown_under_skirt_mm=round(L['dev'] * 1000, 3), faces=int(len(F)))
    lever = (xk - Q[0]) / (float(LP.get('latch_x', 0.0)) - Q[0]) if LP.get('latch_x') else None
    return dict(v=V, f=F.astype(np.int32), lab=LB, part=PT, report=rep, L=L, H=H, z=dict(z0=z0, zu=zu, zl=zl, zkey=zkey, h_tipb=h_tipb, h_sb2=h_sb2, h_ring=h_ring, h_boss=h_boss, sz=(sz0, sz1)))


def link_check(p, E=193e3, Sy=500.0):
    """Parallel links (stainless U-channel): axial load from an eccentric press = F e / d. Operating: 30 N at the edge of the raised area;
    impact (cl. 21, 1 J spring hammer): the cap + carrier (~6 g) brought to the hammer speed (2.8 m/s) over ~0.5 mm -> ~63 N on the cap
    before it lands on its ring stop (the stop then takes the blow, not the links)."""
    t, h, w, L = p['arm_t'] * 1000, p['arm_h'] * 1000, 2 * (p['arm_y'] + p['arm_t']) * 1000, p['arm_L'] * 1000
    A = t * w + 2 * t * (h - t); zc = (t * w * (h - t / 2) + 2 * t * (h - t) * (h - t) / 2) / A
    I = w * t ** 3 / 12 + t * w * (h - t / 2 - zc) ** 2 + 2 * (t * (h - t) ** 3 / 12 + t * (h - t) * ((h - t) / 2 - zc) ** 2)
    Pcr = np.pi ** 2 * E * I / L ** 2; e = p['r'] * 1000 - 0.5; d = p['arm_d'] * 1000
    N_op = 30.0 * e / d; N_imp = 63.0 * e / d
    return dict(section='U %.1f x %.1f x %.1f stainless' % (w, h, t), A_mm2=round(A, 2), I_mm4=round(I, 2), euler_N=round(Pcr, 0),
                N_operating_N=round(N_op, 0), N_impact_N=round(N_imp, 0), safety_buckling=round(Pcr / N_imp, 1), stress_MPa=round(N_imp / A, 1))


# ------------------------------------------------------------------------------------------------- force / travel / tolerance budget
def budget(SK, ot_cap, lid=None, n_mc=200000, seed=1, roll=0.0):
    """STOP actuation chain. ot_cap: lid over-travel at the cap axis (mm, latch over-travel x lever).  Units mm, N.
    Gap g between the TPE tip and the scanned key: nominal p.gap; contributors p.stack (+- worst case, RSS), thermal [lo, hi].
    Key (fixed hardware, unknown): actuation travel key_act [min, max], full travel >= key_full_min, actuation force key_F [min, max].
    Cap forces (the same on every point of the cap: parallel guide): F_ret(s) = F_ret0 + (k_ret + k_boot) s, compliance preload F_c0, rate k_c,
    pivot friction F_fric (4 pivots, d 2.4 pins, mu 0.25, at the cap).
    [fix4-stop2] review M4 / S1 / S2: spring tolerances F_c0_tol (N), k_c_tol (rel), F_ret0_tol (N), k_ret_tol (rel), TPE tip compression tip_comp
    [min, max] and the edge-press roll loss at the stop rim (roll, mm) enter the worst case and the Monte Carlo (0 = the f3 model).
    The plunger must not yield before the key switches: F_c0 - F_c0_tol > key_F max, else the yield travel is added."""
    p = params(SK); g0 = p['gap'] * 1000; S = p['stroke'] * 1000; tol = np.array([t for _, t in p['stack']]); th = p['thermal']
    dFc, dkc, dFr, dkr = p.get('F_c0_tol', 0.0), p.get('k_c_tol', 0.0), p.get('F_ret0_tol', 0.0), p.get('k_ret_tol', 0.0); tc = p.get('tip_comp', [0.0, 0.0])      # [fix4-stop2]
    wc = float(tol.sum()); rss = float(np.sqrt((tol ** 2).sum())); gmin, gmax = g0 - wc + th[0], g0 + wc + th[1]
    a0, a1 = p['key_act']; F0, F1 = p['key_F']; kr = p['k_ret'] + p['k_boot']
    Fret = lambda s: p['F_ret0'] + kr * s
    Fret_hi = lambda s: p['F_ret0'] + dFr + (p['k_ret'] * (1 + dkr) + p['k_boot']) * s      # [fix4-stop2]
    Fret_lo = lambda s: p['F_ret0'] - dFr + (p['k_ret'] * (1 - dkr) + p['k_boot']) * s      # [fix4-stop2]
    Fc_lo, Fc_hi, kc_hi = p['F_c0'] - dFc, p['F_c0'] + dFc, p['k_c'] * (1 + dkc)      # [fix4-stop2]
    yld = max(F1 - Fc_lo, 0.0) / (p['k_c'] * (1 - dkc)) if Fc_lo < F1 else 0.0      # [fix4-stop2] plunger yield before the switch point
    s_act_max = gmax + a1 + tc[1] + roll + yld; s_act_min = gmin + a0 + tc[0]           # cap travel to the key's switch point (g < 0: key pre-pressed)
    F_op_max = Fret_hi(s_act_max) + F1 + p['F_fric']; F_op_min = Fret_lo(min(s_act_min, S)) + F0 - p['F_fric']
    d_max = S - gmin - p['key_full_min']                                       # compliance travel at the cap's own stop (no lid over-travel)
    F_bottom_max = Fret_hi(S) + Fc_hi + kc_hi * max(d_max, 0) + p['F_fric']
    F_key_max = Fc_hi + kc_hi * (d_max + ot_cap)                               # load on the real key, cap on its stop + full lid over-travel
    wc_rep = dict(gap_nominal=g0, gap_definition='minimum vertical distance tip surface -> scanned key surface (stop4.true_gap_tip)' if p.get('v4') else 'tip bottom -> highest scan vertex in the tip footprint',
                  stack_wc=round(wc, 2), stack_rss=round(rss, 2), thermal=th, gap_range_wc=[round(gmin, 2), round(gmax, 2)],
                  false_press_at_rest_max=round(max(-gmin, 0), 2), key_act_min=a0, no_false_stop=bool(-gmin < a0),
                  tip_compression=tc, roll_loss_edge_press=round(roll, 3), plunger_preload_min_over_key_F_max=round(Fc_lo - F1, 2), plunger_yield_before_switch_mm=round(yld, 3),
                  cap_travel_to_actuate_max=round(s_act_max, 2), stroke=S, actuation_margin=round(S - s_act_max, 2), always_actuates=bool(s_act_max < S),
                  grit_on_stop_rim_tolerated_wc_mm=round(max(S - s_act_max, 0.0), 2), grit_on_stop_rim_tolerated_nominal_mm=round(S - (g0 + 0.5 * (a0 + a1) + 0.5 * (tc[0] + tc[1])), 2),
                  F_operate_max=round(F_op_max, 1), F_operate_min=round(F_op_min, 1), F_to_cap_stop_max=round(F_bottom_max, 1),
                  compliance_travel_max=round(d_max + ot_cap, 2), F_on_real_key_max=round(F_key_max, 1), F_ret_max=round(Fret_hi(S), 2),
                  springs=dict(F_c0=[round(Fc_lo, 2), round(Fc_hi, 2)], k_c=[round(p['k_c'] * (1 - dkc), 3), round(kc_hi, 3)], F_ret0=[round(p['F_ret0'] - dFr, 2), round(p['F_ret0'] + dFr, 2)],
                               k_ret=[round(p['k_ret'] * (1 - dkr), 3), round(p['k_ret'] * (1 + dkr), 3)], F_fric=[0.0, p['F_fric']]))
    # Monte Carlo: stack normal (3 sigma = tol), thermal uniform, key uniform; [fix4-stop2] springs (force-sorted: uniform in the band), friction, tip, roll uniform
    rg = np.random.default_rng(seed); g = g0 + (rg.normal(0, 1, (n_mc, len(tol))) * tol / 3).sum(1) + rg.uniform(th[0], th[1], n_mc)
    ka = rg.uniform(a0, a1, n_mc); kF = rg.uniform(F0, F1, n_mc); kf = np.maximum(ka + rg.uniform(p['key_full_min'] - a0, 1.5, n_mc), p['key_full_min'])
    Fc = rg.uniform(Fc_lo, Fc_hi, n_mc); kc = p['k_c'] * rg.uniform(1 - dkc, 1 + dkc, n_mc); Fr0 = rg.uniform(p['F_ret0'] - dFr, p['F_ret0'] + dFr, n_mc)      # [fix4-stop2]
    krr = p['k_ret'] * rg.uniform(1 - dkr, 1 + dkr, n_mc) + p['k_boot']; fr = rg.uniform(0, p['F_fric'], n_mc); tcp = rg.uniform(tc[0], tc[1], n_mc); rl = rg.uniform(0, roll, n_mc) if roll > 0 else 0.0      # [fix4-stop2]
    yl = np.where(Fc < kF, (kF - Fc) / kc, 0.0)                                       # [fix4-stop2]
    s_act = g + ka + tcp + rl + yl; Fop = Fr0 + krr * s_act + kF + fr
    dm = S - g - kf; Fb = Fr0 + krr * S + np.where(dm > 0, Fc + kc * dm, kF) + fr
    mc = dict(n=n_mc, gap_p001_p999=[round(float(np.percentile(g, 0.1)), 2), round(float(np.percentile(g, 99.9)), 2)],
              false_stop_fraction=float((-g >= ka).mean()), no_actuation_fraction=float((s_act >= S).mean()), actuation_margin_p001=round(float(np.percentile(S - s_act, 0.1)), 2),
              F_operate_p50_p999=[round(float(np.percentile(Fop, 50)), 1), round(float(np.percentile(Fop, 99.9)), 1)], F_operate_gt_30=float((Fop > 30).mean()),
              F_to_stop_p50_p999=[round(float(np.percentile(Fb, 50)), 1), round(float(np.percentile(Fb, 99.9)), 1)], F_to_stop_gt_30=float((Fb > 30).mean()),
              random=['stack (normal, 3 sigma = tol)', 'thermal', 'key travel / force', 'F_c0', 'k_c', 'F_ret0', 'k_ret', 'friction', 'tip compression', 'edge-press roll'])
    out = dict(worst_case=wc_rep, monte_carlo=mc)
    if lid is not None:      # latch can only release once the cap sits on its stop (then the key is pressed): F_lid_over at the cap > F_ret max
        out['latch_order'] = lid
    return out


def lid_over_force(lid_poly, Q, x_latch, x_cap, latch_over, seal_c, seal_k, latch_F, band_T, grav_T, seal_comp=None):
    """Force at the cap needed to push the lid into the latch over-travel (N): bulb seal (preload + over-compression) + latch push spring
    + net opening torque (spring - gravity): the latch claw holds the lid DOWN against both, so pushing it further down must overcome them.
    [fix4-stop2] review m7: the f3 version subtracted the net opening torque and left out the seal preload (seal_comp = the designed
    seal compression at closed, m; None = f3 behaviour for old reports).  Units: m, N, N/mm per mm (seal_k), N m."""
    pg = Polygon(lid_poly).buffer(-seal_c); s = np.linspace(0, pg.exterior.length, 2000, endpoint=False)
    P = shapely.get_coordinates(shapely.line_interpolate_point(pg.exterior, s)); ds = pg.exterior.length / len(s)
    I2 = float(((P[:, 0] - Q[0]) ** 2).sum() * ds)                                  # int (x - Qx)^2 ds  (m^3)
    I1 = float((P[:, 0] - Q[0]).sum() * ds)                                         # [fix4-stop2] int (x - Qx) ds  (m^2)
    th = latch_over / (x_latch - Q[0]); Lc = x_cap - Q[0]; Ll = x_latch - Q[0]
    res = {}
    for nm, k_, F_ in (('min', seal_k[0], latch_F[0]), ('max', seal_k[1], latch_F[1])):
        if seal_comp is None: T = k_ * 1e6 * I2 * th + F_ * Ll - (band_T - grav_T)                       # f3 (sign error, no preload)
        else: T = k_ * 1e6 * (seal_comp * I1 + I2 * th) + F_ * Ll + (band_T - grav_T)                    # [fix4-stop2]
        res[nm] = round(T / Lc, 1)
    out = dict(F_lid_over_at_cap_N=res, seal_I2_m3=round(I2, 6), theta_over_deg=round(np.degrees(th), 3))
    if seal_comp is not None: out.update(seal_I1_m2=round(I1, 5), seal_compression_mm=round(seal_comp * 1000, 2), formula='[fix4-stop2] k (c I1 + theta I2) + F_latch L_latch + (T_spring - T_gravity), at the cap lever',
                                         note='the lid is opened by pressing the lid at the latch; a press through the STOP cap needs the force to the cap stop + this (a hard STOP press may open the lid; with the lid-closed sensor the open lid is itself a STOP)')
    return out


RHO = dict(cap=1.2, legend=1.2, carrier=1.36, boot=1.15, rod=7.9, cspring=7.9, tip=1.1, arm=7.9, pin=7.9, rspring=7.9, halo=1.2, bore=1.2, bracket=1.36, bumper=1.1)      # [fix3-int] g/cm3


def mass_mesh(SKB, t_lid, rho_lid=1.15):
    """[fix3-int] Mass of the STOP parts from their CLOSED meshes (divergence theorem per connected solid, |volume| x density, RHO), with the
    lid-skin corrections (the lid kit's skin mass is the full plan area x t_lid: the d 2 bore_r hole and the skin volume the halo boss / the bore
    ring are fused into are taken off). Returns the point masses [(kg, x, z)] for lidmech's lid balance (LP['extra_mass']) and a report."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from lightkit import weld, consistent      # winding made consistent per solid (lidmech.extrude's caps can be reversed against its walls)
    V, F, PT = SKB['v'].astype(float), SKB['f'].astype(np.int64), SKB['part']; p = SKB['L']['p']; xk, yk, z0 = SKB['L']['xk'], SKB['L']['yk'], SKB['L']['z0']
    O = np.array([xk, yk, z0]); pts, parts = [], {}
    for code in np.unique(PT):
        nm = NAME[int(code)]; Vw, Fl = weld(V - O, F[PT == code]); u, inv = np.unique(Fl, return_inverse=True); Vw = Vw[u]; Fl = consistent(inv.reshape(-1, 3)); Vp = Vw
        e = np.r_[Fl[:, [0, 1]], Fl[:, [1, 2]], Fl[:, [2, 0]]]; nc, cc = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(len(Vp), len(Vp))), directed=False)
        fc = cc[Fl[:, 0]]; m_ = 0.0; mx_ = 0.0; mz_ = 0.0
        for c in range(nc):
            T = Vp[Fl[fc == c]]; vol6 = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])); vol = vol6.sum() / 6.0
            if abs(vol) < 1e-15: continue
            cen = (vol6[:, None] * T.sum(1)).sum(0) / (24.0 * vol) + O; mk = abs(vol) * RHO.get(nm, 1.2) * 1000.0
            pts.append((float(mk), float(cen[0]), float(cen[2]))); m_ += mk; mx_ += mk * cen[0]; mz_ += mk * cen[2]
        parts[nm] = dict(g=round(m_ * 1000, 2), x_mm=round(mx_ / max(m_, 1e-12) * 1000, 1), z_mm=round(mz_ / max(m_, 1e-12) * 1000, 1))
    hb = SKB['z']['h_boss']; cor = [(np.pi * (p['boss_r'] ** 2 - p['bore_r'] ** 2) * min(0.0016, t_lid), z0 - 0.0008),      # halo boss fused 1.6 into the skin
                                   (np.pi * (p['bore_out'] ** 2 - p['bore_r'] ** 2) * max(t_lid - 0.0015, 0.0), z0 - 0.5 * (t_lid + 0.0015)),      # bore ring fused into the skin
                                   (np.pi * p['bore_r'] ** 2 * t_lid, z0 - t_lid / 2)]                                                   # the stem bore through the skin
    for vol, zc in cor: pts.append((float(-vol * rho_lid * 1000.0), float(xk), float(zc)))
    M = sum(a[0] for a in pts)
    return dict(points=pts, report=dict(g=round(M * 1000, 1), parts_g=parts, skin_correction_g=round(-sum(v for v, _ in cor) * rho_lid * 1e6, 2),
                                        com_mm=[round(sum(a[0] * a[1] for a in pts) / M * 1000, 1), round(sum(a[0] * a[2] for a in pts) / M * 1000, 1)], method='closed meshes x density'))


def mass(SKB, Q):
    """Mass of the STOP parts (analytic volumes from the parameters, mm^3 -> g) and their gravity torque about the lid axis Q, closed (N m)."""
    p = SKB['L']['p']; H = SKB['H']; xk, xP = SKB['L']['xk'], SKB['L']['xP']; mm = lambda a: a * 1000.0; pi = np.pi
    r, wl = mm(p['r']), mm(p['wall']); r1, r2 = mm(p['ring_r'][0]), mm(p['ring_r'][1]); rs, rb = mm(p['stem_r']), mm(p['stem_bore'])
    stem_len = mm(H['h_ceil'] - SKB['z']['h_sb2'])
    V = dict(cap=pi * r ** 2 * wl + 2 * pi * r * wl * mm(p['side'] + p['roll'] + p['clear']),
             carrier=pi * (r2 ** 2 - r1 ** 2) * mm(p['ring_h']) + pi * (r1 ** 2 - rs ** 2) * 1.0 + pi * (rs ** 2 - rb ** 2) * stem_len + pi * (rb ** 2 - 1.7 ** 2) * mm(p['bush']),
             halo=pi * (mm(p['boss_r']) ** 2 - mm(p['bore_r']) ** 2) * (mm(SKB['z']['h_boss']) + 1.6), bore=pi * (mm(p['bore_out']) ** 2 - mm(p['bore_r']) ** 2) * 1.5,
             arm=2 * (2 * mm(p['arm_t']) * mm(p['arm_h']) * (mm(p['arm_L']) + 5) + mm(p['arm_t']) * 2 * mm(p['arm_y'] + p['arm_t']) * (mm(p['arm_L']) - 12)), bracket=8 * 2 * mm(p['tongue_w']) * 11,
             pin=pi * mm(p['pin_r']) ** 2 * (2 * 15.8 + 4 * 3.7 + 2 * 6.6), rspring=2 * 3 * pi * 6.0 * pi * 0.8 ** 2, cspring=11.2 * pi * 6.5 * pi * 0.4 ** 2,
             rod=pi * mm(p['rod_r']) ** 2 * 18 + pi * mm(p['head_r']) ** 2 * mm(p['head_t']), tip=pi * mm(p['tip_r']) ** 2 * mm(p['tip_h']) * 0.7, boot=2 * pi * 6.5 * 3.5 * 0.6, bumper=pi * (9.5 ** 2 - 6.2 ** 2) * 0.2)
    rho = dict(cap=1.2, carrier=1.36, halo=1.2, bore=1.2, arm=7.9, bracket=1.36, pin=7.9, rspring=7.9, cspring=7.9, rod=7.9, tip=1.1, boot=1.15, bumper=1.1)
    xc = dict(arm=0.5 * (xP + xk), bracket=xP, rspring=xP, pin=0.5 * (xP + xk))
    m = {k: V[k] * rho[k] / 1000.0 for k in V}
    T = sum(m[k] / 1000.0 * 9.81 * (xc.get(k, xk) - Q[0]) for k in m)
    return dict(g=round(sum(m.values()), 1), parts_g={k: round(v, 2) for k, v in m.items()}, torque_Nm=round(T, 4))
