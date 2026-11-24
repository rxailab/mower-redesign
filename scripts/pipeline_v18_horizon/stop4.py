# [fix4-stop2] STOP key v4 - one designed control (fix round 4, reviews A1 A5-A8, S1-S7 S10 S11, M4 m7 m8, G2 G3 G5).
#
# What changes against the f3 key (stopkit.build, still used when lid.stop_key.v4 is false):
#  LOOK   red cap d 40 with a crisp R1.2 roll, 2.0 straight side and a 0.6 concave dish (finger seat); white STOP inlay reading from
#         BEHIND, like the display and the real key on the fixed panel (one product reading direction). Under it a dark PA-GF shroud
#         (d 36, part of the carrier) stands in a flush dark socket; round the socket a flush opal status ring r 21.5-24.5 (+0.35) set
#         in 0.4 shadow gaps: red key - dark well - light ring - light lid. The ring is visible from every side (434 mm2 in plan).
#  LIGHT  the ring is a clear-PC light guide with a frosted top (opal look), insert-moulded in the dark socket; ONE straight vertical
#         D3 pipe of the same insert drops from the ring's front point through a window in the lid plate to 2.4 over the panel LED.
#         Below the lid the pipe runs inside a dark post moulded on the underside and tied to the front ring rib (no free
#         cantilever, no light leak); the post also carries the lid-closed MAGNET (NdFeB d4 x 3) over a HALL PAIR on the enlarged
#         panel emitter board: lid open -> STOP + start inhibit in the firmware (EN 50636-2-107 19.11.2 two channels + plausibility).
#  STROKE 5.5 mm to the shroud's castellated rim on the socket (6 drain / clean-out notches); skirt 2.0 over the socket at full stroke
#         (grit + edge-press roll never reach it). TRUE tip-to-key gap 1.6 (minimum vertical distance tip surface -> scanned key
#         surface, set from the meshes, asserted by stopcheck). Stem 9.5 over the key top (no rigid bottoming in the worst stack).
#  GUIDE  parallel links as CLOSED stainless boxes 12.2 x 2.8 x 0.4 between open tines (roll stiffness x8 -> the edge-press roll that
#         shortened the stroke is ~0.5 deg); the stem lug FLOATS +-0.7 in y on the front stub pins, so side loads (21.1 oblique 1 J
#         impact, branch drag) never bend the tines: they go stem -> POM guide bush (0.15 in y, free for the link arc in x) -> lid.
#  SEAL   dark silicone rolling boot (barrier 1) + TPE lip on the stem in the up-stop bumper under the bush (barrier 2).
#  RETURN two torsion springs on the LOWER rear pin (coil 4.6 under the lid plate, review G2), legs under the lower link and under the
#         bracket; snap-on dark pivot cover hides springs, cut leg ends and pins when the lid is open (A8, S11).
# Every part is emitted as closed, outward-oriented solids (G5).  World: metres, +X forward, +Y left, Z up.
import os, json, math, numpy as np, shapely
from shapely.geometry import Polygon, Point
import stopkit as SK
import lidmech
from meshkit import stack, tube
from lightkit import weld, consistent, orient_out

D4 = dict(
    v4=True,
    halo_r=[0.0215, 0.0245], halo_top=0.00035, halo_bot=-0.0012, halo_roll=0.0002, shadow=0.0004,       # opal ring (r, top / bottom over the lid)
    socket_top=0.0003, socket_bot=-0.0025, socket_roll=0.0003, groove=-0.0004, blend_r=[0.019, 0.021],   # dark socket; blend: lid tangent plane -> lid surface
    shroud_r=[0.0168, 0.0180], shroud_roll=0.0003, web_t=0.0012, notch=[6, 0.0020, 0.0008],              # carrier shroud, castellated rim (n, width, depth)
    bush_z=[-0.0012, -0.0030], bush_cy=0.00015, bush_cx=0.0001, bush_r=0.00752,                        # POM guide bush in the socket bore
    bore_ring=[0.0075, 0.0095, -0.00245, -0.0030], lip_r=0.00498,                                         # dark bore ring under the socket, TPE up-stop + lip seal
    boot_bead=0.0046, box_end=0.0058, pin_r2=0.0009, tongue_y=0.0054,                                    # boot bead height, box webs end 5.8 from the pins
    rs_D=0.0060, rs_d=0.0012, rs_turns=2.75, rs_len=0.0042, rs_y0=0.0010,                                  # return springs (lower rear pin)
    pipe_r=0.0015, window_r=0.0020, post_r=0.0025, magnet=[0.0020, 0.0030, 0.0070], magnet_wall=0.0006,  # light post, magnet (r, h, y offset)
    rib_x=None, rib_t=0.0012,                                                                             # front ring rib (centre x at y 0; None: from LP)
    pcover=[0.0054, 0.0098, 0.0138, 0.0012, 0.0050],                                                    # pivot cover: rear, front x from xP; half width; wall; floor under zl
    # forces / tolerances added to stopkit.budget (all at the cap, N, N/mm, mm)
    F_c0_tol=0.8, k_c_tol=0.10, F_ret0_tol=0.2, k_ret_tol=0.10, tip_comp=[0.04, 0.10],
    F_edge=30.0, hall=dict(Br=1.30, BOP_max=12.0, BRP_min=5.0, die_z=0.00045, pkg=[0.002, 0.002, 0.00055], dy=0.0014),
    board=[0.0055, 0.0150, 0.0010], board_x0=-0.0025, board_y0=-0.0035)

RHO4 = dict(SK.RHO, carrier=1.51, socket=1.36, bush=1.41, post=1.36, magnet=7.5, pcover=1.14, halo=1.20, boot=1.15)     # g/cm3 (carrier PPA-GF40; PA-GF30, POM, NdFeB, PA6, PC)


def params4(SK_):
    p = json.loads(json.dumps(SK.DEFAULTS)); p.update(json.loads(json.dumps(D4)))
    p.update({k: v for k, v in (SK_ or {}).items() if not str(k).startswith('//')}); return p


# ------------------------------------------------------------------------------------------------------------- key surface / true gap
def key_tris(here='.'):
    c = np.load(os.path.join(here, 'control.npz')); return c['v'][c['f'][c['lab'] == 1]]


def zkey_at(T, xy):
    """highest key surface z under each (x, y) (vertical ray down; nan where no key face)."""
    xy = np.atleast_2d(np.asarray(xy, float)); out = np.full(len(xy), np.nan)
    lo = T[:, :, :2].min(1); hi = T[:, :, :2].max(1); a, b, c = T[:, 0], T[:, 1], T[:, 2]
    for i, (x, y) in enumerate(xy):
        m = (lo[:, 0] <= x) & (hi[:, 0] >= x) & (lo[:, 1] <= y) & (hi[:, 1] >= y)
        if not m.any(): continue
        A, B, C = a[m], b[m], c[m]; v0 = C[:, :2] - A[:, :2]; v1 = B[:, :2] - A[:, :2]; v2 = np.array([x, y]) - A[:, :2]
        d00 = (v0 * v0).sum(1); d01 = (v0 * v1).sum(1); d11 = (v1 * v1).sum(1); d20 = (v2 * v0).sum(1); d21 = (v2 * v1).sum(1)
        den = d00 * d11 - d01 * d01; den[np.abs(den) < 1e-30] = 1e-30; u = (d11 * d20 - d01 * d21) / den; v = (d00 * d21 - d01 * d20) / den
        ok = (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        if ok.any(): out[i] = float((A[ok, 2] + u[ok] * (C[ok, 2] - A[ok, 2]) + v[ok] * (B[ok, 2] - A[ok, 2])).max())
    return out


def tip_dome(p, r):
    """height of the domed TPE tip surface over its lowest point at radius r (profile of stopkit: elliptic quarter, tip_h - 0.8 high at tip_r)."""
    r = np.minimum(np.asarray(r, float), p['tip_r']); return (p['tip_h'] - 0.0008) * (1 - np.sqrt(np.maximum(1 - (r / p['tip_r']) ** 2, 0.0)))


def true_gap_tip(p, xk, yk, here='.', dz=0.0):
    """[S1] tip bottom z such that the MINIMUM VERTICAL distance tip surface -> scanned key surface equals p.gap (polar grid, 0.1 mm)."""
    T = key_tris(here); rr = np.r_[0.0, np.arange(0.0001, p['tip_r'] + 1e-9, 0.0001)]; out = []
    for r_ in rr:
        q = np.linspace(0, 2 * np.pi, max(1, int(2 * np.pi * r_ / 0.0001)), endpoint=False) if r_ > 0 else np.array([0.0])
        xy = np.c_[xk + r_ * np.cos(q), yk + r_ * np.sin(q)]; zk = zkey_at(T, xy); ok = np.isfinite(zk)
        if ok.any(): out.append(np.c_[np.full(ok.sum(), r_), xy[ok], zk[ok]])
    G = np.concatenate(out); need = G[:, 3] - tip_dome(p, G[:, 0]); i = int(np.argmax(need))
    return float(need[i] + p['gap']), dict(r_mm=round(G[i, 0] * 1000, 2), at_mm=[round(G[i, 1] * 1000, 2), round(G[i, 2] * 1000, 2)], key_z_mm=round(G[i, 3] * 1000, 3),
                                         key_max_in_footprint_mm=round(float(G[:, 3].max()) * 1000, 3), samples=int(len(G)))


# ------------------------------------------------------------------------------------------------------------------------- layout
def layout(SK_, zi, here='.', LP=None):
    """Axis, heights, true-gap tip, skin cut, underside holes (stem bore + light window) and rib keep-outs."""
    p = params4(SK_); K = SK.key_info(here)
    xk = float(K['cx'] if p['x'] is None else p['x']); yk = float(p['y'])
    z0 = float(zi(np.array([[xk, yk]]))[0]); e_ = 0.004
    gx = float((zi(np.array([[xk + e_, yk]]))[0] - zi(np.array([[xk - e_, yk]]))[0]) / (2 * e_)); gy = float((zi(np.array([[xk, yk + e_]]))[0] - zi(np.array([[xk, yk - e_]]))[0]) / (2 * e_))
    plane = lambda x, y: z0 + gx * (np.asarray(x) - xk) + gy * (np.asarray(y) - yk)
    q_ = np.linspace(0, 2 * np.pi, 72, endpoint=False); cq_ = np.c_[xk + p['r'] * np.cos(q_), yk + p['r'] * np.sin(q_)]
    dev = float(np.max(zi(cq_) - plane(cq_[:, 0], cq_[:, 1])))
    nk = np.hypot(K['P'][:, 0] - xk, K['P'][:, 1] - yk) < p['tip_r']; zkey = float(K['P'][nk, 2].max())
    ztip, tg = true_gap_tip(p, xk, yk, here)
    xP = xk - p['arm_L']; rs = p['halo_r'][1] + p['shadow']; xf = xk + 0.5 * (p['halo_r'][0] + p['halo_r'][1]); yf = yk
    keep = shapely.union_all([Point(xk, yk).buffer(0.0125, quad_segs=16), shapely.box(xP - 0.007, yk - 0.016, xk, yk + 0.016)])
    crown = {a: round(float(zi(np.array([[xk + rs * np.cos(np.radians(a)), yk + rs * np.sin(np.radians(a))]]))[0] - plane(xk + rs * np.cos(np.radians(a)), yk + rs * np.sin(np.radians(a)))) * 1000, 3) for a in (0, 90, 180, 270)}
    return dict(p=p, K=K, xk=xk, yk=yk, z0=z0, grad=(gx, gy), dev=max(dev, 0.0), zkey=zkey, ztip=ztip, true_gap=tg, xP=xP, plane=plane,
                skin_cut=[xk, yk, rs], skin_holes=[[xk, yk, p['bore_r']], [xf, yf, p['window_r']]], rib_keepout=keep, feed=(xf, yf), crown_at_ring_mm=crown)


# ---------------------------------------------------------------------------------------------------------------------- mesh helpers
def weld_mask(V, F, tol=1e-9):
    """lightkit.weld that also returns the mask of the faces kept (labels stay aligned)."""
    V = np.asarray(V, float); F = np.asarray(F, np.int64); key = np.round(V / tol).astype(np.int64); _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    F = inv.ravel()[F]; ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2]); return V[idx], F[ok], ok


def solid(V, F, keep=False):
    """weld + consistent winding + outward orientation per connected component (closed solids for the QA tools, review G5)."""
    V, F, ok = weld_mask(V, F)
    u, inv = np.unique(F, return_inverse=True); V = V[u]; F = consistent(inv.reshape(-1, 3))
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    e = np.r_[F[:, [0, 1]], F[:, [1, 2]]]; n, cc = connected_components(coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(len(V), len(V))), directed=False)
    fc = cc[F[:, 0]]
    for c in range(n):
        m = fc == c; F[m] = orient_out(V, F[m])
    return (V, F, ok) if keep else (V, F)


def rev_rings(c, prof, n, dz=None, seg=0.0015):
    """closed revolve of a closed (r, h) loop about the vertical axis through c (h absolute or relative, as given).  dz(i, q) -> extra h of
    profile point i at angle q (castellations).  Poles (r = 0) are welded by solid().  Long profile segments are split (<= seg) so the
    revolve has no long sliver triangles (self-intersection tests)."""
    q = 2 * np.pi * np.arange(n) / n; R = []
    P0 = list(prof); P1 = []; src = []
    for i, (a, b) in enumerate(zip(P0, P0[1:] + P0[:1])):
        k = max(1, int(np.ceil(np.hypot(b[0] - a[0], b[1] - a[1]) / seg)))
        for j in range(k): P1.append((a[0] + (b[0] - a[0]) * j / k, a[1] + (b[1] - a[1]) * j / k)); src.append(i if j == 0 else -1)
    prof = P1
    for i, (r_, h_) in enumerate(list(prof) + [prof[0]]):
        si_ = src[i % len(prof)]; hh = np.full(n, h_) + (dz(si_, q) if (dz is not None and si_ >= 0) else 0.0)
        R.append(np.c_[c[0] + max(r_, 0.0) * np.cos(q), c[1] + max(r_, 0.0) * np.sin(q), hh])
    return stack(np.array(R), closed=True)


def roll(pc, rr, a0, a1, k=5):
    """quarter-ish arc points (r, h) round centre pc, radius rr, from angle a0 to a1 (deg)."""
    return [(pc[0] + rr * np.cos(np.radians(a)), pc[1] + rr * np.sin(np.radians(a))) for a in np.linspace(a0, a1, k)]


def prism_z(pg, z0_, z1_, bev=0.0):
    return lidmech.extrude(pg, z0_, z1_, lidmech.XY(), bev=bev, nb=2)


def helix_y(c, y_a, y_b, D, d, turns, a0=0.0, n_per=36, nsec=12):
    t = np.linspace(0, 1, max(8, int(turns * n_per)) + 1); q = a0 + 2 * np.pi * turns * t
    P = np.c_[c[0] + D / 2 * np.cos(q), y_a + (y_b - y_a) * t, c[1] + D / 2 * np.sin(q)]
    tg = np.gradient(P, axis=0); tg /= np.linalg.norm(tg, axis=1, keepdims=True); e1 = np.cross(tg, (0, 1.0, 0)); e1 /= np.linalg.norm(e1, axis=1, keepdims=True); e2 = np.cross(tg, e1)
    s = np.linspace(0, 2 * np.pi, nsec, endpoint=False); R = P[:, None, :] + d / 2 * (np.cos(s)[None, :, None] * e1[:, None, :] + np.sin(s)[None, :, None] * e2[:, None, :])
    return stack(R, closed=True, cap0=True, cap1=True), P


def wire(points, r, n=12):
    """bent wire (spring leg) through points: one closed tube per straight segment + balls at the bends (welded into one solid by overlap)."""
    out = []
    for a, b in zip(points[:-1], points[1:]): out.append(tube(a, b, r, n))
    return out


# --------------------------------------------------------------------------------------------------------------------------- build
def build(SK_, zi, LP, Q, here='.'):
    """All STOP parts, closed lid, cap at rest. Returns the stopkit.build dict (v, f, lab, part, report, L, H, z) + 'post' info."""
    L = layout(SK_, zi, here, LP); p = L['p']; xk, yk, z0, xP = L['xk'], L['yk'], L['z0'], L['xP']; c = (xk, yk); gx, gy = L['grad']
    S = p['stroke']; n = p['n']; B = []
    plane = L['plane']
    def place(V, mode):
        V = np.asarray(V, float).copy()
        if mode == 'abs': return V
        x, y, h = V[:, 0], V[:, 1], V[:, 2]
        if mode == 'shear': V[:, 2] = plane(x, y) + h
        elif mode == 'drape': V[:, 2] = zi(V[:, :2]) + h
        elif mode == 'socket':
            r_ = np.hypot(x - xk, y - yk); a_, b_ = p['blend_r']; w = np.clip((r_ - a_) / (b_ - a_), 0, 1); w = w * w * (3 - 2 * w)
            zz = zi(V[:, :2]); zz = np.where(np.isfinite(zz), zz, plane(x, y)); V[:, 2] = plane(x, y) + h + w * (zz - plane(x, y))
        return V
    def add(vf, mat, part, mode='abs'):
        v, f = vf; v = place(v, mode); v, f = solid(v, f); B.append((v, f, np.full(len(f), mat, np.int8), SK.PART[part]))
    # ---- cap: crisp roll / straight side / concave dish, legend (rear-reading by default in v4)
    Vc, Fc, Lc, H = SK.cap_mesh(p, c, 0.0, here, L['dev'])
    Vc, Fw, okc = solid(place(Vc, 'shear'), Fc, keep=True)
    B.append((Vc, Fw, np.asarray(Lc)[okc].astype(np.int8), SK.PART['cap']))
    h_ceil = H['h_ceil']; h_apex = H['h_rim'] + max(p['dome'], 0.0)
    # ---- socket (dark, PA-GF, the lid's dark shot): bore rim -> top +0.3 -> R0.3 -> inner shadow groove -> ring pocket -> outer groove -> lip under the skin edge
    r0, r1 = p['halo_r']; sh = p['shadow']; st, sb, sr = p['socket_top'], p['socket_bot'], p['socket_roll']; gv = p['groove']; hb = p['halo_bot']; rs_ = r1 + sh
    prof = [(p['bore_r'], sb), (rs_ + 0.00005, sb), (rs_ + 0.00005, -0.00005), (rs_, -0.00005), (rs_, gv), (r1 + 0.00002, gv), (r1 + 0.00002, hb - 0.00002),
            (r0 - 0.00002, hb - 0.00002), (r0 - 0.00002, gv), (r0 - sh, gv)] + roll((r0 - sh - sr, st - sr), sr, 0, 90)[:] + roll((p['bore_r'] + sr, st - sr), sr, 90, 180)[:]
    add(rev_rings(c, prof, n), SK.M_KIT, 'socket', 'socket')
    # ---- opal halo ring (clear-PC light guide, frosted top): r0..r1, top +0.35 with R0.2 rolls, into the socket pocket
    hr = p['halo_roll']; ht = p['halo_top']
    prof = [(r0, hb), (r1, hb)] + roll((r1 - hr, ht - hr), hr, 0, 90) + roll((r0 + hr, ht - hr), hr, 90, 180)
    add(rev_rings(c, prof, n), SK.M_HALO, 'halo', 'drape')
    # ---- carrier (dark PPA-GF40: low moisture uptake, the stem carries the side loads into the bush): top disc 0.2 into the cap ceiling, shroud with the castellated stop rim, web, stem d 10, spring chamber, bottom bush
    si, so = p['shroud_r']; srr = p['shroud_roll']; hs = st + S; h_web = h_ceil - p['web_t']
    h_sb2 = L['zkey'] + p['stem_above_key'] - z0; h_bt = h_sb2 + p['bush']
    prof = [(0.0, h_ceil + 0.0002), (so, h_ceil + 0.0002)] + roll((so - srr, hs + srr), srr, 0, -90, 4) + roll((si + srr, hs + srr), srr, -90, -180, 4) + [
            (si, h_web), (p['stem_r'], h_web), (p['stem_r'], h_sb2), (p['rod_r'] + 0.0002, h_sb2), (p['rod_r'] + 0.0002, h_bt), (p['stem_bore'], h_bt),
            (p['stem_bore'], h_ceil - 0.0002), (0.0, h_ceil - 0.0002)]
    rim = set(range(2, 10)); nn_, nw_, nd_ = p['notch']
    qc = np.radians(30.0) + 2 * np.pi * np.arange(int(nn_)) / nn_; hw_ = 0.5 * nw_ / (0.5 * (so + si))
    def dzn(i, q):
        if i not in rim: return 0.0
        d_ = np.abs((q[:, None] - qc[None, :] + np.pi) % (2 * np.pi) - np.pi).min(1); return np.where(d_ <= hw_, nd_, 0.0)
    add(rev_rings(c, prof, 360, dzn), SK.M_KIT, 'carrier', 'shear')
    # ---- boot (dark silicone rolling boot): bead on the socket bore rim -> bead on the stem
    hb0, hb1 = st, p['boot_bead']; br = p['bore_r']; sr2 = p['stem_r']
    add(rev_rings(c, [(br + 0.0011, hb0), (br + 0.0002, hb0 + 0.0011), (sr2 + 0.0012, hb1 - 0.0002), (sr2 + 0.00002, hb1), (sr2 + 0.00002, hb1 + 0.0007),
                      (sr2 + 0.0016, hb1 + 0.0004), (br + 0.0008, hb0 + 0.0013), (br + 0.0017, hb0 + 0.00002)], n // 2), SK.M_RUB, 'boot', 'shear')
    # ---- POM guide bush: stadium bore (stem + 0.15 in y, the link arc -0.22 plus 0.1 in x), pressed into the socket bore
    run = p['arm_L'] * (1 - math.cos(math.asin(min(S / p['arm_L'], 1.0))))
    slot = shapely.LineString([(xk - run - p['bush_cx'], yk), (xk + p['bush_cx'], yk)]).buffer(p['stem_r'] + p['bush_cy'], quad_segs=24)
    bz = p['bush_z']; add(prism_z(Point(xk, yk).buffer(p['bush_r'], quad_segs=32).difference(slot), bz[1], bz[0]), SK.M_KIT, 'bush', 'shear')
    # ---- bore ring under the socket (dark, lid moulding) and the TPE up-stop bumper with the stem lip seal
    a_, b_, c_, d_ = p['bore_ring']
    add(rev_rings(c, [(a_, c_), (b_, c_), (b_, d_), (a_ + 0.00002, d_)], n), SK.M_KIT, 'bore', 'shear')
    add(rev_rings(c, [(p['lip_r'], d_), (b_, d_), (b_, d_ - p['bumper_t']), (p['lip_r'], d_ - p['bumper_t'])], n), SK.M_RUB, 'bumper', 'shear')
    # ---- rod, compliance spring, TPE tip (tip bottom from the TRUE gap)
    h_tipb = L['ztip'] - z0; h_tipt = h_tipb + p['tip_h']
    add(rev_rings(c, [(0.0, h_tipt - 0.0005), (p['rod_r'], h_tipt - 0.0005), (p['rod_r'], h_bt), (p['head_r'], h_bt), (p['head_r'], h_bt + p['head_t']), (0.0, h_bt + p['head_t'])], 40), SK.M_STEEL, 'rod', 'shear')
    sz0, sz1 = h_bt + p['head_t'], h_ceil - 0.0002; csp = spring_design(p, (sz1 - sz0))
    hv, hf = SK.helix(c, sz0 + 0.0004, sz1 - 0.0004, csp['D_mm'] / 1000, csp['d_mm'] / 1000, csp['n_total']); add((hv, hf), SK.M_STEEL, 'cspring', 'shear')
    tq = np.radians(np.linspace(0, 90, 8))
    add(rev_rings(c, [(0.0, h_tipt), (p['tip_r'], h_tipt), (p['tip_r'], h_tipt - 0.0008)] + [(p['tip_r'] * np.cos(a), h_tipb + (p['tip_h'] - 0.0008) * (1 - np.sin(a))) for a in tq[1:]], 40), SK.M_RUB, 'tip', 'shear')
    # ---- parallel links: closed stainless boxes between open tines (rear: round the bracket, front: round the stem lug)
    zu = z0 + p['arm_top'] - p['arm_h'] / 2; zl = zu - p['arm_d']; W = p['arm_y'] + p['arm_t']; t = p['arm_t']; ah = p['arm_h']; be = p['box_end']
    for zz, in ((zu,), (zl,)):
        pg = shapely.box(xP - 0.0024, zz - ah / 2, xk + 0.0024, zz + ah / 2).buffer(-0.0012).buffer(0.0012, quad_segs=6)
        for s_ in (1, -1): add(lidmech.extrude(pg, *sorted((yk + s_ * (W - t), yk + s_ * W)), lidmech.XZ(), bev=0.00012, nb=2), SK.M_ARM, 'arm')
        for zw in ((zz + ah / 2 - t, zz + ah / 2), (zz - ah / 2, zz - ah / 2 + t)):
            add(lidmech.extrude(shapely.box(xP + be, zw[0], xk - be, zw[1]), yk - (W - t) - 0.00001, yk + (W - t) + 0.00001, lidmech.XZ()), SK.M_ARM, 'arm')
    pr = p['pin_r2']; ylong = W + p['rs_y0'] + p['rs_len'] + 0.0009
    for zz, ye in ((zu, W + 0.0005), (zl, ylong)):
        add(tube((xP, yk - ye, zz), (xP, yk + ye, zz), pr, 20), SK.M_STEEL, 'pin')
        for s_ in (1, -1): add(tube((xk, yk + s_ * 0.0042, zz), (xk, yk + s_ * (W + 0.0005), zz), pr, 20), SK.M_STEEL, 'pin')
    zus = float(zi(np.array([[xP, yk]]))[0]) - LP['t_lid']; ty = p['tongue_y']
    D, dw = p['rs_D'], p['rs_d']; rd = dw / 2; z_leg = zl - ah / 2 - rd - 0.00002; z_fx = zl - D / 2; x_fx = xP + D / 2 + rd + 0.0008
    # bracket (dark PA-GF tongue on the lid): rear pins, and a front notch whose floor carries the springs' fixed legs
    tg = shapely.box(xP - 0.0035, z_fx - rd - 0.0006, xP + 0.0050, zus + 0.0003).buffer(-0.0010).buffer(0.0010, quad_segs=6)
    tg = tg.difference(shapely.box(x_fx - rd - 0.0002, z_fx - rd - 0.00012, xP + 0.0060, z_fx + rd + 0.00012))      # leg rests on the floor (0.12 modelled play: the extrude bevel lifts the reflex corner)
    add(lidmech.extrude(tg, yk - ty, yk + ty, lidmech.XZ()), SK.M_KIT, 'bracket')
    # ---- return springs on the LOWER rear pin (2.75 turns: inner end at the front, outer end at the bottom). Arm leg forward-down and in under
    #      the lower link's box web (pushes the link up); fixed leg forward from the coil bottom and in, resting on the bracket notch floor.
    for s_ in (1, -1):
        y_a, y_b = yk + s_ * (W + p['rs_y0']), yk + s_ * (W + p['rs_y0'] + p['rs_len'])
        a_in = -np.pi / 2 - 2 * np.pi * p['rs_turns']
        (hv, hf), P = helix_y((xP, zl), y_a, y_b, D, dw, p['rs_turns'], a0=a_in); add((hv, hf), SK.M_STEEL, 'rspring')
        for pts in ([P[0], (xP + 0.0062, yk + s_ * (W + 0.0008), z_leg), (xP + 0.0076, yk + s_ * (W - 0.0021), z_leg)], [P[-1], (x_fx, P[-1][1], z_fx), (x_fx, yk + s_ * 0.0020, z_fx)]):
            pts = [np.asarray(t_, float) for t_ in pts]
            for j, (a, b) in enumerate(zip(pts[:-1], pts[1:])):      # legs: separate solids overlapping the coil end / each other (no shared pole vertices)
                u_ = (b - a) / np.linalg.norm(b - a); a2 = a + u_ * (0.00025 if j == 0 else -0.0003); add(tube(a2, b + (u_ * 0.0003 if j < len(pts) - 2 else 0), rd * (1.0 - 0.02 * j), 12), SK.M_STEEL, 'rspring')
    # ---- pivot cover (dark PA6 snap-on tray, A8 / S11): floor + 2 side walls + rear wall up into the lid plate, low front lip
    cr, cf_, cw, wt, cfl = p['pcover']; zfl = zl - cfl; zt_ = zus + 0.0003
    add(prism_z(shapely.box(xP - cr, yk - cw, xP + cf_, yk + cw).buffer(-0.001).buffer(0.001, quad_segs=4), zfl - wt, zfl), SK.M_KIT, 'pcover')
    for s_ in (1, -1): add(lidmech.extrude(shapely.box(xP - cr, zfl - wt, xP + cf_, zt_), *sorted((yk + s_ * (cw - wt), yk + s_ * cw)), lidmech.XZ()), SK.M_KIT, 'pcover')
    add(prism_z(shapely.box(xP - cr + 0.00002, yk - cw + wt - 0.0001, xP - cr + wt, yk + cw - wt + 0.0001), zfl - 0.00001, zt_ - 0.00002), SK.M_KIT, 'pcover')
    add(prism_z(shapely.box(xP + cf_ - wt, yk - cw + wt - 0.0001, xP + cf_ - 0.00002, yk + cw - wt + 0.0001), zfl - 0.00001, zl - ah / 2 - 0.0013), SK.M_KIT, 'pcover')
    # ---- light post (dark, moulded on the underside, tied to the front ring rib) with the closed magnet pocket; the magnet
    xf, yf = L['feed']; mr, mh, my = p['magnet']; zp_ = float(LP.get('halo_panel_z', 0.25242)); zb = zp_ + 0.0010 + 0.0006 + 0.0024      # LED top + 2.4 (lightkit gap_closed)
    pgp = shapely.LineString([(xf, yf), (xf, yf + my)]).buffer(p['post_r'], quad_segs=16)
    fp = np.asarray(pgp.exterior.coords); zu_fp = zi(fp) - LP['t_lid']; ztop = float(np.nanmax(zu_fp)) + 0.0003
    add(prism_z(pgp, zb, ztop, bev=0.0003), SK.M_KIT, 'post')
    rib_x = p['rib_x'] if p['rib_x'] is not None else LP.get('stop4_rib_x')
    if rib_x is not None:
        zrb = float(np.nanmin(zi(np.array([[rib_x, yf]])))) - LP['t_lid'] - 0.0035
        add(lidmech.extrude(shapely.box(rib_x, zrb, xf - p['post_r'] + 0.0005, ztop), yf - 0.0025, yf + 0.0025, lidmech.XZ()), SK.M_KIT, 'post')
    add(tube((xf, yf + my, zb + p['magnet_wall']), (xf, yf + my, zb + p['magnet_wall'] + mh), mr, 32), SK.M_STEEL, 'magnet')
    # ---- assemble
    V = []; F = []; LB = []; PT = []; o = 0
    for v, f, l, pt in B: V.append(v); F.append(f + o); LB.append(l); PT.append(np.full(len(f), pt, np.int16)); o += len(v)
    V = np.concatenate(V); F = np.concatenate(F); LB = np.concatenate(LB); PT = np.concatenate(PT)
    LK = link_check4(p); K = SK.key_info(here); kp = K['plane']; z_si = kp[0] + kp[1] * xk
    rep = dict(version='v4 [fix4-stop2]', axis_mm=[round(xk * 1000, 2), round(yk * 1000, 2)], lid_A_surface_z_mm=round(z0 * 1000, 2), cap_d_mm=round(2 * p['r'] * 1000, 1),
               cap_rim_above_lid_mm=round(H['h_rim'] * 1000, 2), cap_dish_mm=round(-min(p['dome'], 0.0) * 1000, 2), skirt_above_lid_mm=round(H['h_skirt'] * 1000, 2),
               cap_top_z_mm=round((z0 + h_apex) * 1000, 2), stroke_mm=round(S * 1000, 2), shroud_rim_above_lid_mm=round(hs * 1000, 2),
               skirt_over_socket_full_stroke_mm=round((H['h_skirt'] - S - st) * 1000, 2), halo_ring_mm=dict(r=[r0 * 1000, r1 * 1000], top=ht * 1000, plan_area_mm2=round(np.pi * (r1 ** 2 - r0 ** 2) * 1e6, 1)),
               tip_gap_true_mm=round(p['gap'] * 1000, 3), true_gap_at=L['true_gap'], tip_gap_standin_mm=round((z0 + h_tipb - z_si) * 1000, 2), key_top_under_tip_mm=round(L['zkey'] * 1000, 2),
               stem_bottom_above_key_mm=round(p['stem_above_key'] * 1000, 2), arm_pivot_x_mm=round(xP * 1000, 2), arm_L_mm=p['arm_L'] * 1000, arm_spacing_mm=p['arm_d'] * 1000,
               arm_bottom_z_mm=round((zl - ah / 2) * 1000, 2), legend=H['legend'], legend_reads='front' if p.get('legend_front', True) else 'rear (as the display and the real key)',
               compliance_spring=csp, return_springs=return_spring_design(p), link=LK, lid_over_travel_at_cap_mm=None, lid_slope_deg=[round(np.degrees(np.arctan(gx)), 2), round(np.degrees(np.arctan(gy)), 2)],
               crown_under_skirt_mm=round(L['dev'] * 1000, 3), crown_at_ring_mm=L['crown_at_ring_mm'], light=dict(feed_mm=[round(xf * 1000, 2), round(yf * 1000, 2)], pipe_d_mm=2 * p['pipe_r'] * 1000,
               post_bottom_z_mm=round(zb * 1000, 2), post_top_z_mm=round(ztop * 1000, 2), window_d_mm=2 * p['window_r'] * 1000), sensor=hall_report(p, zb, Q, xf) if Q is not None else None, faces=int(len(F)))
    return dict(v=V, f=F.astype(np.int32), lab=LB, part=PT, report=rep, L=L, H=H,
                z=dict(z0=z0, zu=zu, zl=zl, zkey=L['zkey'], h_tipb=h_tipb, h_sb2=h_sb2, h_ring=hs, h_boss=st, sz=(sz0, sz1), zb_post=zb))


# --------------------------------------------------------------------------------------------------------------- springs / links / sensor
def spring_design(p, L_inst):
    """[M4] compliance spring for the chamber length L_inst (m): music wire EN 10270-1 SH, preload F_c0 at L_inst, rate k_c; travel = compliance
    travel at the cap's stop with the lid over-travel + 10 %; solid height + 0.6 must fit. Returns wire / coil / turns / stresses."""
    G = 79000.0; Li = L_inst * 1000; d = 0.75; D = 6.5; k = p['k_c']; F0 = p['F_c0'] * (1 + 0.0) + p['F_c0_tol']
    k1 = G * d ** 4 / (8 * D ** 3); na = k1 / k; nt = na + 2.0; solid = nt * d; travel = Li - solid - 0.6
    Fmax = F0 + k * (1 + p['k_c_tol']) * travel; C = D / d; Kw = (4 * C - 1) / (4 * C - 4) + 0.615 / C
    tau = 8 * Fmax * D / (np.pi * d ** 3) * Kw; Rm = 2230.0 - 620.0 * np.log10(d)       # EN 10270-1 SH approx (d 0.75: ~2310 MPa)
    return dict(wire='EN 10270-1 SH', d_mm=d, D_mm=D, n_active=round(na, 2), n_total=round(nt, 2), installed_mm=round(Li, 2), solid_mm=round(solid, 2),
                travel_to_block_mm=round(travel, 2), F0_N=[p['F_c0'] - p['F_c0_tol'], p['F_c0'] + p['F_c0_tol']], k_N_mm=[round(k * (1 - p['k_c_tol']), 3), round(k * (1 + p['k_c_tol']), 3)],
                free_length_mm=round(Li + p['F_c0'] / k, 1), tau_at_block_MPa=round(tau, 0), Rm_MPa=round(Rm, 0), tau_over_Rm=round(tau / Rm, 2), sorted='100 % force-sorted at the installed length (grade 1)')


def return_spring_design(p):
    """two torsion springs on the lower rear pin: preload torque / rate from F_ret0 / k_ret at the cap (lever arm_L)."""
    E = 193000.0; d, D, n = p['rs_d'] * 1000, p['rs_D'] * 1000, p['rs_turns']; Lm = p['arm_L'] * 1000
    k_rad = E * d ** 4 / (64 * D * n); T0 = p['F_ret0'] * Lm / 2; kT = p['k_ret'] * Lm * Lm / 2      # per spring (N mm, N mm/rad)
    Tmax = T0 + kT * p['stroke'] * 1000 / Lm; Ki = (4 * D / d - 1) / (4 * D / d - 4); sig = 32 * Tmax * Ki / (np.pi * d ** 3)
    return dict(wire_mm=d, coil_D_mm=D, turns=n, rate_Nmm_rad_each=round(k_rad, 0), rate_needed_Nmm_rad_each=round(kT, 0), preload_Nmm_each=round(T0, 1),
                preload_angle_deg=round(np.degrees(T0 / kT), 1), sigma_max_MPa=round(sig, 0), F_ret0_N=[p['F_ret0'] - p['F_ret0_tol'], p['F_ret0'] + p['F_ret0_tol']],
                note='rate set by the free leg angle at assembly; 100 % return-force check on the cap (EOL)')


def link_check4(p, E=193e3, G=74e3, Sy=1100.0):
    """[S2 S4] closed-box parallel links (1.4310 C1300, Rp0.2 >= 1100 MPa): buckling, ROLL stiffness of the pair (box torsion in series with
    the open tines bending in their plane), the edge-press roll and its stroke loss at the stop rim, and the side-load path
    (y-float of the stem lug -> POM bush -> lid; the tines are never loaded out of plane)."""
    t, h, w, L = p['arm_t'] * 1000, p['arm_h'] * 1000, 2 * (p['arm_y'] + p['arm_t']) * 1000, p['arm_L'] * 1000; be = p['box_end'] * 1000
    Iy = (w * h ** 3 - (w - 2 * t) * (h - 2 * t) ** 3) / 12; A = w * h - (w - 2 * t) * (h - 2 * t); Pcr = np.pi ** 2 * E * Iy / L ** 2
    e = p['r'] * 1000 - 0.5; d = p['arm_d'] * 1000; N_op = 30.0 * e / d; N_imp = 63.0 * e / d
    Am = (w - t) * (h - t); J = 4 * Am ** 2 * t / (2 * ((w - t) + (h - t))); k_box = G * J / (L - 2 * be)
    It = t * h ** 3 / 12; lt = be; k_tine = 3 * E * It / lt ** 3; yc = (w - t) / 2; k_end = 2 * k_tine * yc ** 2
    k_link = 1.0 / (1.0 / k_box + 2.0 / k_end); k_roll = 2 * k_link
    phi = p['F_edge'] * p['r'] * 1000 / k_roll; r_stop = 0.5 * sum(p['shroud_r']) * 1000
    play = np.arctan(0.04 / (2 * yc)); loss = r_stop * (phi + play)      # pins H8/f7: 0.04 diametral play over the tine spacing
    tau_roll = (p['F_edge'] * p['r'] * 1000 / 2) / (2 * Am * t)
    # side loads: stem cantilever from the bush to the rim, PPA-GF40 tube d 10 / 7.6
    rs, rb = p['stem_r'] * 1000, p['stem_bore'] * 1000; Wst = np.pi * (2 * rs) ** 4 * (1 - (rb / rs) ** 4) / (32 * 2 * rs); lev = (p['stroke'] + p['clear'] + p['side'] + p['roll'] - p['bush_z'][0]) * 1000
    cases = {}
    for nm, F in (('branch_drag_80N_rim', 80.0), ('impact_1J_45deg_rim_peak', 1000.0 * np.sqrt(0.5))):
        cases[nm] = dict(F_lateral_N=round(F, 0), path='stem -> POM bush (0.15 in y) -> lid; tines unloaded (stem lug floats +-%.1f on the stub pins)' % ((p['arm_y'] - p['stem_r']) * 1000),
                         stem_bending_MPa=round(F * lev / Wst, 0), bush_pressure_MPa=round(F / (2 * rs * abs(p['bush_z'][1] - p['bush_z'][0]) * 1000), 1),
                         roll_deg=round(np.degrees(F * (lev - 1.0) / k_roll), 2), link_shear_MPa=round(F * (lev - 1.0) / 2 / (2 * Am * t), 0))
    return dict(section='closed box %.1f x %.1f x %.1f (1.4310), open tines %.1f long at both ends' % (w, h, t, be), A_mm2=round(A, 2), Iy_mm4=round(Iy, 2), euler_N=round(Pcr, 0),
                N_operating_N=round(N_op, 0), N_impact_N=round(N_imp, 0), safety_buckling=round(Pcr / N_imp, 1), stress_MPa=round(N_imp / A, 1),
                J_box_mm4=round(J, 1), k_roll_Nmm_rad=round(k_roll, 0), edge_press=dict(F_N=p['F_edge'], at_r_mm=p['r'] * 1000, roll_deg=round(np.degrees(phi), 2),
                rim_drop_mm=round(phi * p['r'] * 1000, 3), pin_play_roll_deg=round(np.degrees(play), 3), stroke_loss_at_stop_rim_mm=round(loss, 3), box_shear_MPa=round(tau_roll, 0)), side_loads=cases,
                stem_PPA_GF40_strength_MPa=[200, 240], stem_SF_impact=[round(200.0 / (707.0 * lev / Wst), 2), round(240.0 / (707.0 * lev / Wst), 2)],
                note='1 J peak is an elastic upper bound; carrier PPA-GF40 (conditioned / dry; PA-GF30 would give SF 0.9-1.5) -> confirm by the 21.1 test')


def roll_loss(p):
    return float(link_check4(p)['edge_press']['stroke_loss_at_stop_rim_mm'])


def hall_field(Br, R, Lm, z):
    """on-axis flux density (mT) of an axially magnetised cylinder (radius R, length Lm, mm) at distance z (mm) from its face."""
    return Br / 2 * ((z + Lm) / np.sqrt((z + Lm) ** 2 + R ** 2) - z / np.sqrt(z ** 2 + R ** 2)) * 1000


def hall_report(p, zb, Q, xf):
    """[S3] lid-closed sensing: NdFeB N42 d4 x 3 in the post, two omnipolar hall switches (BOP <= 12 mT, BRP >= 5 mT) on the emitter board.
    Field at the sensor die vs lid angle (the magnet rises ~ (x - Qx) sin(a) and moves 0.13 mm forward per mm)."""
    hl = p['hall']; R, Lm = p['magnet'][0] * 1000, p['magnet'][1] * 1000; zmag = (zb + p['magnet_wall']) * 1000; zdie = (0.25242 + 0.0010 + hl['die_z']) * 1000
    lev = (xf - Q[0]) * 1000; rows = {}
    for a in (-0.4385, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 10.0):
        g = (zmag - zdie) + lev * np.sin(np.radians(a)); off = hl['dy'] * 1000
        B0 = hall_field(hl['Br'], R, Lm, g); Boff = B0 * (1 - 0.6 * (off / max(g + 0.5 * Lm, 0.1)) ** 2)
        rows[str(a)] = dict(gap_mm=round(g, 2), B_mT=round(float(Boff), 1))
    aa = np.linspace(0, 10, 2001); Bf = np.array([hall_field(hl['Br'], R, Lm, (zmag - zdie) + lev * np.sin(np.radians(a))) for a in aa])
    a_open = float(aa[np.argmax(Bf < hl['BRP_min'])]); a_closed = float(aa[np.argmax(Bf < hl['BOP_max'])])
    return dict(magnet='NdFeB N42 d %.0f x %.0f, axial, Ni-coated, closed pocket (0.6 wall)' % (2 * R, Lm), sensors='2 omnipolar hall switches (dual channel), %.1f mm either side of the magnet axis' % (hl['dy'] * 1000),
                gap_closed_mm=round(zmag - zdie, 2), B_vs_lid_deg=rows, reads_closed_below_deg=round(a_closed, 2), reads_open_above_deg=round(a_open, 2),
                front_edge_rise_at_open_mm=round((0.045 - Q[0]) * 1000 * np.sin(np.radians(a_open)), 1),
                firmware='lid not closed (either channel) = STOP, latched; start (local / scheduled / app) inhibited until both channels read closed + the restart rule; channels disagree > 0.5 s = fault (19.11.2 / Annex R)',
                keep_out='magnet >= 150 mm from any magnetometer (or calibrate with the lid closed)')


# ------------------------------------------------------------------------------------------------------------------- colour (A6)
ISO3864_RED = [(0.735, 0.265), (0.681, 0.239), (0.579, 0.341), (0.655, 0.345)]      # ISO 3864-4 safety red, ordinary materials: CIE 1931 x, y corners; beta >= 0.07


def iso3864_red(hexc):
    """[A6] CIE 1931 x, y (sRGB, D65) and luminance factor of a hex colour; inside the ISO 3864-4 safety-red box and the distance to its edge."""
    c = [int(hexc.lstrip('#')[i:i + 2], 16) / 255 for i in (0, 2, 4)]; c = np.array([x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c])
    X, Y, Z = np.array([[0.4124, 0.3576, 0.1805], [0.2126, 0.7152, 0.0722], [0.0193, 0.1192, 0.9505]]) @ c; x, y = X / (X + Y + Z), Y / (X + Y + Z)
    pg = Polygon(ISO3864_RED); pt = Point(x, y); ins = bool(pg.contains(pt) and Y >= 0.07)
    return dict(hex=hexc, x=round(float(x), 4), y=round(float(y), 4), beta=round(float(Y), 4), inside=ins, edge_distance=round(float(pg.exterior.distance(pt)) * (1 if pg.contains(pt) else -1), 4),
                box='ISO 3864-4 red (ordinary): (0.735, 0.265) (0.681, 0.239) (0.579, 0.341) (0.655, 0.345), beta >= 0.07')


# ----------------------------------------------------------------------------------------------------------------- mass / halo feed
def mass_mesh4(SKB, t_lid, rho_lid=1.15):
    """stopkit.mass_mesh with the v4 densities and skin correction: the lid kit counts the full skin; inside r skin_cut the socket + ring
    replace it (their meshes carry the full plate thickness)."""
    rho0 = dict(SK.RHO); SK.RHO.clear(); SK.RHO.update(RHO4)
    try:
        out = SK.mass_mesh(dict(SKB, z=dict(SKB['z'], h_boss=0.0)), t_lid, rho_lid)
    finally:
        SK.RHO.clear(); SK.RHO.update(rho0)
    L = SKB['L']; xk, z0 = L['xk'], L['z0']; rs = L['skin_cut'][2]; p = L['p']
    pts = [a for a in out['points']][:-3]       # drop stopkit's three f3 skin corrections
    cor = -np.pi * rs ** 2 * t_lid * rho_lid * 1000.0; pts.append((float(cor), float(xk), float(z0 - t_lid / 2)))
    M = sum(a[0] for a in pts); rep = dict(out['report']); rep.update(g=round(M * 1000, 1), skin_correction_g=round(cor * 1000, 2),
                                                                      com_mm=[round(sum(a[0] * a[1] for a in pts) / M * 1000, 1), round(sum(a[0] * a[2] for a in pts) / M * 1000, 1)])
    return dict(points=pts, report=rep)


def mass_summary(SKB, Q, MM=None):
    """gravity torque of the STOP parts (net, mesh masses) about the hinge axis, closed and open (N m)."""
    M = MM if MM is not None else mass_mesh4(SKB, 0.0022)
    T = {}
    for a in (0, 30, 60, 75):
        q = np.radians(a); T[str(a)] = round(sum(m_ * 9.81 * ((x_ - Q[0]) * np.cos(q) - (z_ - Q[1]) * np.sin(q)) for m_, x_, z_ in M['points']), 4)
    return dict(g=M['report']['g'], torque_Nm=T['0'], torque_vs_open_deg_Nm=T, parts_g={k: v['g'] for k, v in M['report']['parts_g'].items()}, method='closed meshes x density')


def halo_feed4(sk3, Hl, KB, KK, add, QB, stack_, weld_, orient_out_, consistent_, MAT, wbox):
    """[fix4-stop2] lightkit section 9 for the v4 key: ONE straight vertical clear pipe from 2.4 over the LED up into the ring's front point
    (through the window in the lid plate, inside the dark post); enlarged emitter board with the LED under the pipe and the hall pair
    under the magnet. Returns the report."""
    xf, yf = sk3['feed']; rb_ = Hl['boss_r']; zp = float(Hl.get('panel_z', 0.25242)); bw, bl, bt = sk3['board']; led_h = Hl['led_h']
    zled = zp + bt + led_h; zb = zled + Hl['gap_closed']; zt = sk3['ring_bot_z'] + 0.0003
    q_ = np.linspace(0, 2 * np.pi, 40, endpoint=False)
    zz_ = np.r_[zb, zb + 0.0003, np.linspace(zb + 0.0006, zt, 24)]; rr_ = np.r_[rb_ - 0.0003, rb_ - 0.00004, np.full(24, rb_)]
    Rg = np.array([np.c_[xf + r_ * np.cos(q_), yf + r_ * np.sin(q_), np.full(len(q_), z_)] for z_, r_ in zip(zz_, rr_)])
    qv, qf = stack_(Rg, closed=True, cap0=True, cap1=True); qv, qf = weld_(qv, qf); QB.add(qv, orient_out_(np.asarray(qv), consistent_(qf)), MAT['clear'])
    x0b = xf + sk3['board_x0']; y0b = yf + sk3['board_y0']
    v_, f_ = wbox((x0b + bw / 2, y0b + bl / 2, zp + bt / 2 + 0.00002), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (bw, bl, bt), r=0.0008); add(KB, KK, 'emitter', v_, f_, MAT['pcb'])
    v_, f_ = wbox((xf, yf, zp + bt + led_h / 2), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (0.0016, 0.0016, led_h)); add(KB, KK, 'emitter', v_, f_, MAT['led'])
    hx, hy, hz = sk3['hall_pkg']
    for dy_ in (-sk3['hall_dy'], sk3['hall_dy']):
        v_, f_ = wbox((xf, yf + sk3['magnet_y'] + dy_, zp + bt + hz / 2), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (hx, hy, hz), r=0.0002); add(KB, KK, 'emitter', v_, f_, MAT['sensor'])
    return dict(design='[fix4-stop2] clear-PC light guide insert: opal (frosted) ring r %.1f-%.1f in the dark socket + ONE vertical D%.0f pipe from the ring front point, inside the dark post' %
                (sk3['halo_r'][0] * 1000, sk3['halo_r'][1] * 1000, rb_ * 2000), feed_top_mm=[round(xf * 1000, 2), round(yf * 1000, 2), round(zt * 1000, 2)],
                feed_bottom_mm=[round(xf * 1000, 2), round(yf * 1000, 2), round(zb * 1000, 2)], pipe_d_mm=rb_ * 2000, pipe_length_mm=round((zt - zb) * 1000, 1), pipe_elevation_deg=90.0,
                led_top_z_mm=round(zled * 1000, 2), gap_closed_mm=round(Hl['gap_closed'] * 1000, 2), board_mm=[round(x0b * 1000, 1), round(y0b * 1000, 1), bw * 1000, bl * 1000, bt * 1000],
                hall_at_mm=[[round(xf * 1000, 2), round((yf + sk3['magnet_y'] + s_ * sk3['hall_dy']) * 1000, 2)] for s_ in (-1, 1)],
                coupling='LED -> 2.4 air -> d 3 polished pipe end -> 21 mm TIR pipe -> 45 deg splitter facets at the ring front point -> ring guide, graded micro-prism extraction under the frosted top; pipe sleeved in the dark post (no stray light)')
