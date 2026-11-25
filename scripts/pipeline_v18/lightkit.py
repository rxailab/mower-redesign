# [fix3-lights] Light bands: lens, sealed lamp modules, mounting, harness, status-halo feed.
#
#   R = lightkit.build(X, log)      X: dict from design_gen.py (lens patches, final body mesh, mount kit, lid data, parameters 'lights')
#   R['arrays'] -> lights_* arrays for the npz, R['report'] -> generator report
#
# Concept (front and rear are the same module family; the rear has a centre dot instead of the pupil):
#  - LENS: a separate smoked PC lens (IR-transparent tint, silicone hard-coat outside, dead-front print inside). Its outer face IS the A-surface of
#    the band (the painted faces cut out of the body by design_gen, 0.4 mm gap to the body opening, both edges rolled R0.2); 2.5 mm thick = wall.
#  - MODULE: the lens is laser-welded (through the IR-transparent tint) onto a black PC/ABS housing: weld land on the lens back face 0.5..2.0 mm inside
#    the outline -> the optics chamber is sealed (IP67) with a pressure-equalising vent membrane and a sealed connector. Behind the gap the housing
#    frame steps back 0.6 mm and runs 0.9..3.0 mm beyond the outline under the body wall: a light trap (nothing to see through the gap) and the land
#    of a closed-cell foam gasket (dust / splash barrier, open over the charger slot = the pocket's drain).
#  - OPTICS: per light segment one PMMA light bar (4.4 x 3.5) behind the 95.5 x 4 window, fed at its outboard end by a 3030 mid-power LED through a
#    45 deg TIR periscope leg; graded prism extraction on the back face, white PC carrier channel (reflector + light shield), diffusing texture on
#    the lens window. Front centre: camera module (barrel through the pupil board), ToF/IR sensor and status LED + pipe on the pupil board.
#    Rear centre: dot LED + PMMA pipe.
#  - MOUNTING: 4 ears per housing on 4 cored screw bosses (OD 4.9 / hole 2.0, 1.45 wall = 0.58 t, two 1.2 gussets = 0.48 t) moulded on the inside
#    of the body wall, all parallel to the module axis (one lifter direction); EJOT PT K25 x 7 (5 mm engagement), the ear seats are the hard stop
#    that sets the gasket compression.
#  - HARNESS: right-angle sealed plug on the module connector, cable up behind the wall, TPE grommet through the deck, P-clip, service connector pair
#    in a cradle on the deck top (reached with the cover off), chassis-side lead over the deck's inner edge (open J-hook) into the chassis.
#  - HALO: the lid's clear 2K halo ring is fed by a clear boss on the lid underside at the ring's front point, over an emitter (LED board) on the
#    front strip of the control panel (control-panel supplier's part; no harness to the lid or the cover).
# Band coordinates: (y, z) = projection on the band (end view), u = depth under the A-surface along the surface normal (m). BandMap maps to world.
import numpy as np, shapely, json
from shapely.geometry import Polygon, Point, LineString
from shapely import affinity
from scipy.spatial import cKDTree
import shell_wall, lidmech
from meshkit import Bag, stack, lathe, rrect

# material index of the kit / harness arrays (design_render.py: lights materials)
MAT = dict(housing=0, pmma=1, white=2, pcb=3, led=4, foam=5, metal=6, sensor=7, boss=8, conn=9, cable=10, tpe=11, clear=12)
KINDS = ['housing', 'gasket', 'bar', 'carrier', 'pcb', 'led', 'camera', 'tof', 'pipe', 'foam', 'boss', 'screw', 'ear', 'shroud', 'vent', 'plug',
         'cable', 'grommet', 'clip', 'svc', 'cradle', 'chassis_lead', 'emitter', 'halo_feed', 'pad']      # [fix4-front2] + 'pad' (crush pads = flush datums)
DEF = dict(
    lens=dict(gap=0.0004, roll_body=0.0002, roll_lens=0.0002, t=0.0025, edge_ri=0.0002),
    housing=dict(weld_in=0.0020, weld_out=0.0005, weld_u=0.00245, step_d=-0.0001, step_u=0.0033, frame_t=0.0015, d_out=0.0025, d_out_top=0.0020,
                 d_out_bot=0.0030, d_trap=0.0009, trap_y=0.064, gasket_in=0.0007, gasket_edge=0.0003,
                 u_back={'front': [[0.0, 0.0108], [0.022, 0.0108], [0.030, 0.0094], [0.074, 0.0094], [0.084, 0.0139], [0.2, 0.0139]],
                         'rear': [[0.0, 0.0098], [0.014, 0.0098], [0.022, 0.0094], [0.074, 0.0094], [0.084, 0.0139], [0.2, 0.0139]]},
                 back_t=0.0015, vent=[0.060, 0.1780, 0.0035]),
    bar=dict(y0=0.0255, y1=0.1245, z0=-0.0022, z1=0.0022, u0=0.00275, u1=0.00625, leg_y=0.1210, leg_u=0.0099, carrier_t=0.0008),
    board=dict(y0=0.1130, y1=0.1265, z_half=0.0070, u0=0.0106, t=0.0010, led=[0.0030, 0.0030, 0.0006]),
    pupil=dict(board=[0.023, 0.0075, 0.0044], cam=[0.0085, 0.0040, 0.0030, 0.00275], tof=[0.0049, 0.0025, 0.00156], tof_y=-0.0165, led_y=0.0165, pipe_r=0.0014),
    dot=dict(board=[0.008, 0.006, 0.0070], pipe_r=0.0023, led=[0.0016, 0.0008, 0.0006]),
    bosses=dict(at=[[0.139, 0.0], [0.086, -0.020]], od=0.0049, hole=0.0020, seat_u=0.0090, ear_t=0.0020, ear_w=0.0090, gusset_t=0.0012, sink=0.0010, fit_hole=0.0026, slot=0.0010, clear_hole=0.0032,
                screw=[0.0025, 0.0070, 0.0045, 0.0016]),
    connector=dict(y=0.112, dz=0.002, w=0.016, h=0.011, wall=0.0012, len=0.0060, plug_len=0.0080),
    harness=dict(cable_d={'front': 0.0050, 'rear': 0.0042}),
    halo=dict(on=True, boss_r=0.0015, gap_closed=0.0024, emitter=[0.005, 0.008, 0.0010], led_h=0.0006))


def _merge(a, b):
    for k, v in (b or {}).items():
        if k.startswith('//'): continue
        if isinstance(v, dict) and isinstance(a.get(k), dict): _merge(a[k], v)
        else: a[k] = v
    return a


# ----------------------------------------------------------------------------------------------------------------- mesh helpers
def weld(V, F, tol=1e-9):
    """Merge coincident vertices (poles of revolved profiles, closing rings) and drop collapsed faces: makes stacked pieces watertight."""
    V = np.asarray(V, float); F = np.asarray(F, np.int64); key = np.round(V / tol).astype(np.int64); _, idx, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.ravel(); F = inv[F]; ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2]); return V[idx], F[ok]


def rev(o, a, prof, n=32):
    """Closed solid of revolution: the profile [(h, r), ...] is a closed loop (last point joins the first); r = 0 points become poles."""
    prof = list(prof); v, f = lathe(o, a, prof + [prof[0]], n); v, f = weld(v, f); return v, orient_out(v, f)


def consistent(F):
    """Consistent winding over every edge-connected component (as Blender's 'recalculate normals'): breadth-first over the face graph, a face is
    flipped when it runs a shared edge in the same direction as its already-placed neighbour."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import breadth_first_order
    F = np.asarray(F, np.int64).copy(); nf = len(F)
    if nf < 2: return F
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]); fid = np.tile(np.arange(nf), 3); nv = int(F.max()) + 1
    key = np.minimum(e[:, 0], e[:, 1]) * nv + np.maximum(e[:, 0], e[:, 1]); fw = e[:, 0] < e[:, 1]; o = np.argsort(key, kind='stable'); ks = key[o]
    pair = np.nonzero(ks[1:] == ks[:-1])[0]; a_, b_ = o[pair], o[pair + 1]; fa, fb = fid[a_], fid[b_]; w = (fw[a_] == fw[b_]).astype(np.int8)      # same direction -> flip
    G = coo_matrix((np.ones(len(fa)), (fa, fb)), shape=(nf, nf)).tocsr(); G = G + G.T
    W = {}
    for x_, y_, w_ in zip(fa.tolist(), fb.tolist(), w.tolist()): W[(x_, y_)] = w_; W[(y_, x_)] = w_
    par = np.full(nf, -1, np.int8)
    for seed in range(nf):
        if par[seed] >= 0: continue
        order, pred = breadth_first_order(G, seed, directed=False, return_predecessors=True); par[seed] = 0
        for n_ in order[1:]: par[n_] = par[pred[n_]] ^ W[(int(n_), int(pred[n_]))]
    F[par == 1] = F[par == 1][:, ::-1]; return F


def orient_out(V, F):
    """Flip a closed mesh so its signed volume is positive (outward normals)."""
    if not len(F): return F
    P = V[F]; vol = np.einsum('ij,ij->i', P[:, 0], np.cross(P[:, 1], P[:, 2])).sum()
    return F[:, ::-1].copy() if vol < 0 else F


def refine(V, F, maxlen, metric=(1, 1, 0)):
    """Midpoint subdivision until no edge is longer than maxlen in the given metric (param space: y, z only). Closed meshes stay closed."""
    w = np.asarray(metric, float)
    for _ in range(12):
        P = V[F]; L = np.linalg.norm((P - np.roll(P, -1, 1)) * w, axis=2); mk = L > maxlen
        if not mk.any(): break
        nv = len(V); b = np.roll(F, -1, 1); key = np.minimum(F, b).astype(np.int64) * nv + np.maximum(F, b); uk = np.unique(key[mk])
        pos = np.clip(np.searchsorted(uk, key), 0, len(uk) - 1); has = uk[pos] == key; ia = uk // nv; ib = uk % nv
        V = np.concatenate([V, 0.5 * (V[ia] + V[ib])]); mid = nv + pos; n_ = has.sum(1); out = [F[n_ == 0]]
        def rot(mask, k0):
            f = F[mask]; md = mid[mask]; r = np.arange(len(f)); return [f[r, (k0 + i) % 3] for i in range(3)] + [md[r, (k0 + i) % 3] for i in range(3)]
        m1 = n_ == 1
        if m1.any(): a_, b2, c_, mab, _, _ = rot(m1, np.argmax(has[m1], 1)); out += [np.c_[a_, mab, c_], np.c_[mab, b2, c_]]
        m2 = n_ == 2
        if m2.any(): a_, b2, c_, _, mbc, mca = rot(m2, np.argmin(has[m2], 1)); out += [np.c_[c_, mca, mbc], np.c_[a_, b2, mbc], np.c_[a_, mbc, mca]]
        m3 = n_ == 3
        if m3.any(): a_, b2, c_, mab, mbc, mca = rot(m3, np.zeros(int(m3.sum()), int)); out += [np.c_[a_, mab, mca], np.c_[mab, b2, mbc], np.c_[mca, mbc, c_], np.c_[mab, mbc, mca]]
        F = np.concatenate(out)
    return V, F


def ext(pg, w0, w1, plane='yz', bev=0.0, step=None):
    """Prism of a shapely polygon in band coordinates. plane 'yz': polygon (y, z), extruded along u; 'yu': polygon (y, u) along z; 'zu': (z, u) along y."""
    v, f = lidmech.extrude(pg, w0, w1, (np.zeros(3), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)), bev=bev, nb=2, step=step)
    if plane == 'yu': v = v[:, [0, 2, 1]]
    elif plane == 'zu': v = v[:, [2, 0, 1]]
    return v, orient_out(v, f)


def cap_cdt(ring2):
    """Constrained Delaunay triangulation of the polygon ring2 using only its vertices (indices into ring2, CCW)."""
    pts, tri = lidmech.tri_polygon(Polygon(ring2)); _, ii = cKDTree(ring2).query(pts); t = ii[tri]
    a = (ring2[t[:, 1], 0] - ring2[t[:, 0], 0]) * (ring2[t[:, 2], 1] - ring2[t[:, 0], 1]) - (ring2[t[:, 2], 0] - ring2[t[:, 0], 0]) * (ring2[t[:, 1], 1] - ring2[t[:, 0], 1])
    return np.where((a < 0)[:, None], t[:, ::-1], t)


def cap_tri(ring2, step):
    """Triangulation of the polygon bounded by ring2 (k, 2, closed implicitly) with interior points (spacing step): ring vertices first, same order."""
    from scipy.spatial import Delaunay
    pg = Polygon(ring2).buffer(0); bx = pg.bounds
    gx, gy = np.meshgrid(np.arange(bx[0] + step / 2, bx[2], step), np.arange(bx[1] + step / 2, bx[3], step), indexing='ij'); gp = np.c_[gx.ravel(), gy.ravel()]
    gp = gp[shapely.contains_xy(pg, gp[:, 0], gp[:, 1])]; gp = gp[shapely.distance(pg.exterior, shapely.points(gp)) > 0.45 * step]
    P = np.r_[ring2, gp]; tri = Delaunay(P).simplices; c = P[tri].mean(1); tri = tri[shapely.contains_xy(pg.buffer(1e-9), c[:, 0], c[:, 1])]
    a = (P[tri[:, 1], 0] - P[tri[:, 0], 0]) * (P[tri[:, 2], 1] - P[tri[:, 0], 1]) - (P[tri[:, 2], 0] - P[tri[:, 0], 0]) * (P[tri[:, 1], 1] - P[tri[:, 0], 1])
    tri = np.where((a < 0)[:, None], tri[:, ::-1], tri); tri = tri[np.abs(a) > 1e-16]
    return P, tri


def tube_path(path, r, n=14, caps=True):
    """Tube of radius r along a 3D polyline (rotation-minimising frames)."""
    path = np.asarray(path, float); tg = np.gradient(path, axis=0); tg /= np.maximum(np.linalg.norm(tg, axis=1, keepdims=True), 1e-15)
    a = np.array([0.0, 0.0, 1.0]) if abs(tg[0, 2]) < 0.9 else np.array([1.0, 0.0, 0.0]); u = np.cross(tg[0], a); u /= np.linalg.norm(u); U = [u]
    for i in range(1, len(path)):
        u = U[-1] - (U[-1] @ tg[i]) * tg[i]; u /= np.linalg.norm(u); U.append(u)
    U = np.array(U); W = np.cross(tg, U); q = np.linspace(0, 2 * np.pi, n, endpoint=False)
    R = path[:, None, :] + r * (np.cos(q)[None, :, None] * U[:, None, :] + np.sin(q)[None, :, None] * W[:, None, :])
    v, f = stack(R, closed=True, cap0=caps, cap1=caps); return v, orient_out(v, f)


def spline_path(P, step=0.001):
    """Smooth path (centripetal Catmull-Rom) through the waypoints P, resampled every ~step."""
    P = np.asarray(P, float); Q = np.r_[2 * P[:1] - P[1:2], P, 2 * P[-1:] - P[-2:-1]]; out = []
    for i in range(1, len(Q) - 2):
        p0, p1, p2, p3 = Q[i - 1], Q[i], Q[i + 1], Q[i + 2]; L = np.linalg.norm(p2 - p1); n = max(2, int(np.ceil(L / step)))
        t0 = 0.0; t1 = t0 + np.linalg.norm(p1 - p0) ** 0.5 + 1e-9; t2 = t1 + L ** 0.5 + 1e-9; t3 = t2 + np.linalg.norm(p3 - p2) ** 0.5 + 1e-9
        for t in np.linspace(t1, t2, n, endpoint=False):
            A1 = (t1 - t) / (t1 - t0) * p0 + (t - t0) / (t1 - t0) * p1; A2 = (t2 - t) / (t2 - t1) * p1 + (t - t1) / (t2 - t1) * p2; A3 = (t3 - t) / (t3 - t2) * p2 + (t - t2) / (t3 - t2) * p3
            B1 = (t2 - t) / (t2 - t0) * A1 + (t - t0) / (t2 - t0) * A2; B2 = (t3 - t) / (t3 - t1) * A2 + (t - t1) / (t3 - t1) * A3
            out.append((t2 - t) / (t2 - t1) * B1 + (t - t1) / (t2 - t1) * B2)
    out.append(P[-1]); return np.array(out)


def bend_r(pth):      # [fix4-front2] radius of curvature at every point of a sampled 3D path (m)
    d1 = np.gradient(pth, axis=0); d2 = np.gradient(d1, axis=0); sp = np.linalg.norm(d1, axis=1)
    return 1.0 / np.maximum(np.linalg.norm(np.cross(d1, d2), axis=1) / np.maximum(sp ** 3, 1e-30), 1e-12)


def relax_path(pth, rmin, iters=400):      # [fix4-front2] G5: a tube self-intersects where the path bends tighter than its radius: Laplacian relaxation of the points
    pth = np.asarray(pth, float).copy()      # bending tighter than rmin (and their neighbours), end points fixed; returns the path and its new minimum bend radius
    for _ in range(iters):
        R = bend_r(pth); bad = np.nonzero(R[1:-1] < rmin)[0] + 1
        if not len(bad): break
        idx = np.unique(np.clip(np.r_[bad - 2, bad - 1, bad, bad + 1, bad + 2], 1, len(pth) - 2))
        pth[idx] = 0.5 * pth[idx] + 0.25 * (pth[idx - 1] + pth[idx + 1])
    return pth, float(bend_r(pth)[1:-1].min())


def wbox(c, e1, e2, e3, s, r=0.0):
    """Box (rounded in the e1-e2 plane by r) centred at c with axes e1, e2, e3 (unit) and size s = (s1, s2, s3)."""
    pg = shapely.box(-s[0] / 2, -s[1] / 2, s[0] / 2, s[1] / 2)
    if r > 0: pg = pg.buffer(-r).buffer(r, quad_segs=6)
    v, f = lidmech.extrude(pg, -s[2] / 2, s[2] / 2, (np.asarray(c, float), e1, e2, e3)); return v, orient_out(v, f)


def frame3(a, up=(0.0, 0.0, 1.0)):
    a = np.asarray(a, float); a = a / np.linalg.norm(a); u = np.asarray(up, float); e1 = np.cross(u, a)
    if np.linalg.norm(e1) < 1e-6: e1 = np.cross((0.0, 1.0, 0.0), a)
    e1 /= np.linalg.norm(e1); e2 = np.cross(a, e1); return e1, e2, a


# ----------------------------------------------------------------------------------------------------------------- band coordinates
class BandMap:
    """Outer A-surface of a band region as a graph x = f(y, z) (least squares, even powers of y, fitted to the lens patch and the body's outer skin
    around it); P(y, z, u) = (f, y, z) - u * n(y, z), n = outward unit normal."""
    def __init__(s, pts, sgn, zc, log=print):
        s.sgn = sgn; s.zc = zc; y, z, x = pts[:, 1], pts[:, 2], pts[:, 0]
        s.ei = [(i, j) for i in range(0, 13, 2) for j in range(0, 6)]
        keep = np.ones(len(x), bool)
        for it in range(3):
            A = s._basis(y[keep], z[keep]); s.c = np.linalg.lstsq(A, x[keep], rcond=None)[0]; res = x - s._basis(y, z) @ s.c
            keep = np.abs(res) < max(3 * np.std(res[keep]), 2e-5)
        s.res_max = float(np.abs(res[keep]).max()); s.res_rms = float(np.sqrt(np.mean(res[keep] ** 2))); s.n_out = int((~keep).sum())
        log('band map %s: %d pts, residual rms %.4f mm, max %.4f mm (%d outliers dropped)' % ('front' if sgn > 0 else 'rear', len(x), s.res_rms * 1000, s.res_max * 1000, s.n_out))
    def _basis(s, y, z, dy=0, dz=0):
        Y = y / 0.15; Z = (z - s.zc) / 0.03; cols = []
        for i, j in s.ei:
            a = (Y ** (i - dy) * (np.prod(range(i - dy + 1, i + 1)) if dy else 1) if i >= dy else 0 * Y) / (0.15 ** dy)
            b = (Z ** (j - dz) * (np.prod(range(j - dz + 1, j + 1)) if dz else 1) if j >= dz else 0 * Z) / (0.03 ** dz)
            cols.append(a * b)
        return np.stack(cols, 1)
    def f(s, y, z): return s._basis(np.atleast_1d(y), np.atleast_1d(z)) @ s.c
    def normal(s, y, z):
        fy = s._basis(y, z, dy=1) @ s.c; fz = s._basis(y, z, dz=1) @ s.c; n = s.sgn * np.c_[np.ones_like(fy), -fy, -fz]
        return n / np.linalg.norm(n, axis=1, keepdims=True)
    def __call__(s, Q):
        Q = np.atleast_2d(np.asarray(Q, float)); y, z, u = Q[:, 0], Q[:, 1], Q[:, 2]
        return np.c_[s.f(y, z), y, z] - u[:, None] * s.normal(y, z)


def pads_(Pd, w, h, zc, H):      # [fix4-front2] m9: crush pads = the module's flush datums on the body's inner skin (band (y, z) polygons)
    """Pd: {'at': [['end', d], ['bottom', |y|, d], ['top', |y|, d]], 'len', 'wid'} -> [(polygon, kind)]: 'end' pads on the band axis beyond the stadium ends,
    'bottom' / 'top' pads on the straight runs; d = distance of the pad centre from the lens outline (on the flange)."""
    if not Pd: return []
    L_, W_ = Pd.get('len', 0.0025), Pd.get('wid', 0.0010); out = []
    for a_ in Pd['at']:
        for s_ in (1, -1):
            if a_[0] == 'end': c_ = (s_ * (w / 2 + a_[1]), zc); pg = shapely.box(c_[0] - W_ / 2, c_[1] - L_ / 2, c_[0] + W_ / 2, c_[1] + L_ / 2)
            else:
                c_ = (s_ * a_[1], zc + (h / 2 + a_[2]) * (-1 if a_[0] == 'bottom' else 1)); pg = shapely.box(c_[0] - L_ / 2, c_[1] - W_ / 2, c_[0] + L_ / 2, c_[1] + W_ / 2)
            out.append((pg.buffer(-0.0002).buffer(0.0002, quad_segs=4), a_[0]))
    return out


def stack_(P, LENS, H):      # [fix4-front2] m9: lens flushness and gasket squeeze tolerance stacks (mm), f3 chain (ear seats) vs f4 chain (crush pads)
    St = P.get('stack') or {}; rss = lambda v: float(np.sqrt(np.sum(np.square(v)))); out = {}
    for nm_ in ('f3_ear_seat', 'f4_crush_pad'):
        ch_ = St.get(nm_)
        if ch_: tl_ = [c_[1] for c_ in ch_]; out[nm_] = dict(chain=[[c_[0], c_[1]] for c_ in ch_], rss=round(rss(tl_), 3), worst=round(float(np.sum(tl_)), 3))
    g_ = St.get('gasket')
    if g_:      # installed height = pad standoff (step_u - t): +- (pad crush, flange flatness, inner-skin waviness); squeeze vs the free height
        hi_ = (H['step_u'] - LENS['t']) * 1000; tl_ = [c_[1] for c_ in g_['chain']]; r_, wc_ = rss(tl_), float(np.sum(tl_)); fr_ = g_['free']
        out['gasket'] = dict(free_mm=fr_, installed_mm=round(hi_, 3), chain=g_['chain'], squeeze_nominal_pct=round((1 - hi_ / fr_) * 100, 1),
                             squeeze_rss_pct=[round((1 - (hi_ + r_) / fr_) * 100, 1), round((1 - (hi_ - r_) / fr_) * 100, 1)],
                             squeeze_worst_pct=[round((1 - (hi_ + wc_) / fr_) * 100, 1), round((1 - (hi_ - wc_) / fr_) * 100, 1)], limit_pct=g_.get('limit_pct'))
    out['weld'] = dict(collapse_mm=St.get('weld_collapse'), modelled_overlap_mm=round((LENS['t'] - H['weld_u']) * 1000, 3), control='displacement-controlled quasi-simultaneous laser weld')
    return out


# ----------------------------------------------------------------------------------------------------------------- build
def build(X, log=print):
    P = _merge(json.loads(json.dumps(DEF)), X['P']); rep = {}; LENS = P['lens']
    V, F, side, part = X['V'], X['F'], X['side'], X['part']
    body_out = F[(part == 0) & (side == 0)] if side is not None else F
    BV = V[np.unique(body_out)]
    mv, mf, mk, mkn = X['mount']; deck_f = mf[mk == mkn.index('deck')]
    LZ = Bag(); LZs = []; LZid = []                                                       # lens solids
    KB = Bag(); KK = []                                                                   # kit (fixed to the body)
    HB = Bag(); HK = []                                                                   # harness
    QB = Bag()                                                                            # halo feed (moves with the lid)
    def add(bag, rec, kind, v, f, mat):
        v, f = weld(v, f); f = orient_out(v, consistent(f)); n0 = len(bag.F); bag.add(v, f, mat)
        if len(bag.F) > n0: rec.append((len(f), KINDS.index(kind)))
    for pt in X['patches']:
        sgn = 1.0 if pt['side'] == 'front' else -1.0; bd = pt['band']; zc = bd['z']; h = bd['h']; w = bd['w']; nm = pt['side']; R = {}
        rb = min(bd.get('r', h / 2), h / 2)      # [v21] corner radius as design_gen's band (default: stadium, R = h / 2)
        outline = shapely.box(-w / 2, zc - h / 2, w / 2, zc + h / 2).buffer(-rb + 1e-6).buffer(rb - 1e-6, quad_segs=48)      # the lens outline
        # ---- 1. lens solid: the cut-out A-surface patch made a closed 2.5 mm solid by shell_wall (edge rolled R roll_lens, inner round edge_ri)
        Wl = shell_wall.build(pt['v'], pt['f'], pt['lab'], np.zeros(len(pt['f']), np.int16), {}, dict(t=LENS['t'], edge_r=LENS['roll_lens'], edge_ri=LENS['edge_ri'],
                              loop_r=[], thin=[], smooth_r=0.004, blend=0.002), log=lambda *a: None)
        ck = Wl.report.get('check', {}).get(0, Wl.report.get('check', {}).get('0', {}))
        R['lens'] = dict(faces=int(len(Wl.F)), boundary_edges=ck.get('boundary_edges'), nonmanifold_edges=ck.get('nonmanifold_edges'), wall_min_mm=ck.get('wall_min_mm'),
                         wall_max_mm=ck.get('wall_max_mm'), outline_mm=[round(w * 1000, 1), round(h * 1000, 1)], gap_mm=LENS['gap'] * 1000)
        LZ.add(Wl.V, Wl.F, 0); LZs.append(Wl.side.astype(np.uint8)); LZid.append(np.full(len(Wl.F), 0 if sgn > 0 else 1, np.uint8)); LZ.L[-1] = Wl.lab.astype(np.int8)
        # ---- 2. band coordinates
        zlo_ = min(0.145, zc - h / 2 - 0.003) if bd.get('map_low') else 0.145      # [v21] map_low: tall visor band, the map also covers the body below the lens (gasket, lower flange)
        sb = np.abs(BV[:, 1]) < 0.158; sb &= (BV[:, 2] > zlo_) & (BV[:, 2] < 0.1875) & (sgn * BV[:, 0] > 0.25)
        dB = shapely.distance(outline.boundary, shapely.points(BV[sb][:, 1:3])); Bs = BV[sb][dB > 0.0015]
        M = BandMap(np.r_[pt['v'][::2], Bs[::2]], sgn, zc, log=log); R['map_residual_mm'] = [round(M.res_rms * 1000, 4), round(M.res_max * 1000, 4)]
        Ym = np.linspace(-w / 2 + h / 2, w / 2 - h / 2, 41); am = -M.normal(Ym, np.full_like(Ym, zc)).mean(0); am /= np.linalg.norm(am)     # module axis (inward)
        R['axis'] = np.round(am, 4).tolist()
        def mapped(v, f, maxlen=0.002):
            v2, f2 = refine(np.asarray(v, float), f, maxlen); return M(v2), f2
        H = dict(P['housing'], **bd.get('housing', {})); ub_k = np.array(bd.get('u_back') or H['u_back']['front' if bd.get('module', 'pupil' if sgn > 0 else 'dot') == 'pupil' else 'rear'])      # [v21] depth by module (the camera needs the deeper centre)
        ub = lambda y: np.interp(np.abs(y), ub_k[:, 0], ub_k[:, 1])
        # ---- 3. housing: one closed swept section round the outline (weld land / step / gasket flange / wall), back plate = two caps
        ring = lidmech.resample(outline, 0.0015); ring = ring[::-1] if Polygon(ring).exterior.is_ccw is False else ring
        Nl = lidmech.lnormals(ring)                                                            # CCW -> inward
        if shapely.contains_xy(outline, *(ring[0] + Nl[0] * 0.001)) is False: Nl = -Nl
        Nout = -Nl; yy = ring[:, 0]; zz = ring[:, 1]
        bottom = zz < zc - 1e-4; top = zz > zc + 1e-4; ends = np.abs(yy) > w / 2 - rb
        dout = np.where(top & ~ends, H['d_out_top'], np.where(bottom & ~ends, H['d_out_bot'], H['d_out']))
        trap = bottom & (np.abs(yy) < H['trap_y']); dout = np.where(trap, H['d_trap'], dout)
        dout = np.convolve(np.r_[dout[-20:], dout, dout[:20]], np.ones(21) / 21, 'same')[20:-20]          # smooth transitions (no steps in the flange)
        ubr = ub(yy); BT = H['back_t']
        uc_ = np.full(len(ring), 0.5 * (H['step_u'] + H['frame_t'] + ubr.min() - BT))      # [fix3-int] chamfer start of the bottom wall over the port frame (collinear elsewhere)
        sdp = H['step_d']
        if 'step_d_low' in H or 'step_d_top' in H:      # [v21] tall lens on a face that curves away at its lower corners and rakes back at its top: the light-trap
            #                                              step moves in on the lower part and along the top edge; it stays at step_d round the upper ends
            sm_ = lambda t_: np.clip(t_, 0.0, 1.0) ** 2 * (3 - 2 * np.clip(t_, 0.0, 1.0))
            wl_ = sm_((zc - zz) / (0.2 * h)); wt_ = sm_((zz - zc) / (0.2 * h)) * sm_((w / 2 - 0.5 * rb - np.abs(yy)) / (0.5 * rb))
            sdp = H['step_d'] + (H.get('step_d_low', H['step_d']) - H['step_d']) * wl_ + (H.get('step_d_top', H['step_d']) - H['step_d']) * wt_
            sdp = np.convolve(np.r_[sdp[-20:], sdp, sdp[:20]], np.ones(21) / 21, 'same')[20:-20]
        secs = [(-H['weld_in'], ubr - BT), (-H['weld_in'], uc_), (-H['weld_in'], H['weld_u']), (sdp, H['weld_u']), (sdp, H['step_u']), (dout, H['step_u']), (dout, H['step_u'] + H['frame_t']),
                (-H['weld_out'], H['step_u'] + H['frame_t']), (-H['weld_out'], uc_), (-H['weld_out'], ubr)]      # [fix3-int] + the two chamfer points
        secs = [(np.broadcast_to(np.asarray(d_, float), len(ring)).copy(), np.broadcast_to(np.asarray(u_, float), len(ring)).copy()) for d_, u_ in secs]      # [fix3-int]
        PF3 = X.get('ports')      # [fix3-int] fix-ports' closed port frame (tunnel top wall 1.5 at z ~162.7, hood back to x ~304.5) runs right under the bottom of the
        if PF3 is not None and sgn > 0 and len(PF3[0]) and float(H.get('port_clear', 0.0005)) > 0:      # [fix3-int] front housing over the slot: the f2-era flange / step / back corner went 0.6 into it. Over the frame the
            # [fix4-front2] port_clear 0 = off: the band now sits 5.1 mm over the slot (A2), so the housing clears the frame without the clamp
            # [fix3-int] bottom section is clamped to a horizontal plane port_clear above the frame top AT THAT y (the slot's R10 ends fall away): the flange shrinks
            # [fix3-int] (light trap = the step behind the gap, then the frame's own top wall), the back-bottom edge gets a chamfer that starts behind the pupil board.
            pv3 = np.asarray(PF3[0], float); pv3 = pv3[pv3[:, 0] > 0.29]; pcl = float(H.get('port_clear', 0.0005)); ytop_ = np.abs(pv3[:, 1]).max()
            ztop_ = np.array([pv3[np.abs(np.abs(pv3[:, 1]) - abs(y_)) < 0.002, 2].max() if (np.abs(np.abs(pv3[:, 1]) - abs(y_)) < 0.002).any() else -1.0 for y_ in yy])
            wcl = np.clip((ytop_ + 0.004 - np.abs(yy)) / 0.003, 0.0, 1.0) * bottom      # 1 over the frame (+1 mm), 0 from 4 mm beyond it
            def dneed(u_, zl_):      # most outward offset d (band z = zc - h/2 - d on the bottom run) whose world z is >= zl_ at depth u_
                zb_ = zz.copy()
                for _ in range(3): zb_ = zb_ + (zl_ - M(np.c_[yy, zb_, u_])[:, 2])
                return (zc - h / 2) - zb_
            zl3 = ztop_ + pcl; Wt = H['weld_in'] - H['weld_out']; dn0_ = dneed(np.zeros(len(ring)), zl3); dn1_ = dneed(np.full(len(ring), 0.01), zl3)      # d_need is linear in u
            ub0_ = P['pupil']['board'][2] + P['board']['t'] + 0.0006      # chamfer starts >= 0.6 behind the pupil board (inner face unchanged in front of that)
            ucs = np.clip((-H['weld_out'] - dn0_) / ((dn1_ - dn0_) / 0.01), max(H['step_u'] + H['frame_t'] + 0.0002, ub0_), ubr - BT - 0.0004)
            ucs = np.where(wcl > 0, ucs, uc_); secs[1] = (secs[1][0], ucs.copy()); secs[8] = (secs[8][0], ucs.copy())
            for k_ in (3, 4, 5, 6, 7, 8, 9): secs[k_] = (np.minimum(secs[k_][0], dneed(secs[k_][1], zl3)), secs[k_][1])           # step, flange, outer wall
            secs[5] = (np.maximum(secs[5][0], secs[4][0]), secs[5][1]); secs[6] = (np.maximum(secs[6][0], secs[7][0]), secs[6][1])    # flange never inside the step / wall
            nof_ = (secs[5][0] - secs[4][0]) < 0.0003      # a flange remnant < 0.3 mm would be a sliver step: no flange there (step face -> straight to the wall)
            secs[5] = (np.where(nof_, secs[4][0], secs[5][0]), secs[5][1]); secs[6] = (np.where(nof_, secs[7][0], secs[6][0]), secs[6][1])
            for k_ in (0, 1): secs[k_] = (np.minimum(secs[k_][0], dneed(secs[k_][1], zl3) - Wt), secs[k_][1])                         # inner wall: the wall stays 1.5
            secs = [(d0_ + wcl * (d1_ - d0_), u_) for (d0_, u_), (d1_, _) in zip([(np.broadcast_to(np.asarray(d_, float), len(ring)), np.broadcast_to(np.asarray(u_, float), len(ring))) for d_, u_ in
                    [(-H['weld_in'], ubr - BT), (-H['weld_in'], ucs), (-H['weld_in'], H['weld_u']), (H['step_d'], H['weld_u']), (H['step_d'], H['step_u']), (dout, H['step_u']),
                     (dout, H['step_u'] + H['frame_t']), (-H['weld_out'], H['step_u'] + H['frame_t']), (-H['weld_out'], ucs), (-H['weld_out'], ubr)]], secs)]
            R['port_frame_trim'] = dict(frame_top_mm=round(float(ztop_[wcl > 0.99].max()) * 1000, 2) if (wcl > 0.99).any() else None, clear_mm=pcl * 1000, frame_half_width_mm=round(ytop_ * 1000, 1),
                                        back_edge_raise_mm=round(float((-H['weld_out'] - secs[9][0]).max()) * 1000, 2), flange_mm_min=round(float(secs[5][0][wcl > 0.99].min()) * 1000, 2) if (wcl > 0.99).any() else None,
                                        chamfer_from_u_mm=round(float(ucs[wcl > 0.99].min()) * 1000, 2) if (wcl > 0.99).any() else None)
        Rg = [np.c_[ring + Nout * d_[:, None], u_] for d_, u_ in secs]      # [fix3-int] per-point d / u
        hv, hf = stack(Rg, closed=True)
        Vc = [hv]; Fc = [hf]; nb = len(hv)
        for k_, flip in ((0, True), (len(Rg) - 1, False)):                                    # back caps (inner face of the back plate, outer back face)
            r2 = Rg[k_][:, :2]; cv_, cf_ = refine(np.c_[r2, np.zeros(len(r2))], cap_cdt(r2), 0.002)      # ring edges (1.5 mm) stay whole: conforming
            cv_[:, 2] = ub(cv_[:, 0]) - (BT if k_ == 0 else 0.0); idx = np.r_[k_ * len(ring) + np.arange(len(ring)), nb + np.arange(len(cv_) - len(ring))]
            Vc.append(cv_[len(ring):]); nb += len(cv_) - len(ring); t_ = idx[cf_]; Fc.append(t_[:, ::-1] if flip else t_)
        hv = np.concatenate(Vc); hf = orient_out(hv, np.concatenate(Fc)); add(KB, KK, 'housing', M(hv), hf, MAT['housing'])
        R['housing'] = dict(u_back_mm=[round(float(ub(0.0)) * 1000, 1), round(float(ub(0.05)) * 1000, 1), round(float(ub(0.12)) * 1000, 1)],
                            flange_mm=dict(top=H['d_out_top'] * 1000, bottom=H['d_out_bot'] * 1000, ends=H['d_out'] * 1000, light_trap_over_slot=H['d_trap'] * 1000))
        # gasket (compressed 1.0 -> 0.6 mm) between the body's inner skin (u = t) and the flange, open over the charger slot (drain)
        Pd_ = dict(P.get('pads') or {}, **bd.get('pads', {})) if (P.get('pads') or bd.get('pads')) else None; PDS = pads_(Pd_, w, h, zc, H)      # [v21] per-band pad layout      # [fix4-front2] crush pads (flush datums) on the flange: (polygon in band (y, z), kind)
        if H.get('gasket_continuous'):      # [fix4-front2] gasket = the flange as built minus gasket_edge, closed round the lens (also over the slot), notched round the pads and
            dfl_ = np.asarray(secs[5][0], float); gpg = Polygon(ring + Nout * (dfl_ - H['gasket_edge'])[:, None]).buffer(0)      # [fix4-front2]   at the drain breaks
            gpg = gpg.difference(outline.buffer(H['gasket_in'], quad_segs=24))      # [fix4-front2]
            for gy_, gw_ in H.get('gasket_drains', []):      # [fix4-front2] drain breaks on the bottom run: the lens pocket drains into the (wet) nose cavity
                for s_ in (1, -1): gpg = gpg.difference(shapely.box(s_ * gy_ - gw_ / 2, -1, s_ * gy_ + gw_ / 2, zc - h / 2 + 1e-4))      # [fix4-front2]
            for pp_, _ in PDS: gpg = gpg.difference(pp_.buffer(0.0004, quad_segs=8))      # [fix4-front2]
        else:      # [fix4-front2] f3: gasket from the nominal flange widths, open over the slot
            gpg = outline.buffer(H['d_out_bot'] - H['gasket_edge'], quad_segs=24).difference(outline.buffer(H['gasket_in'], quad_segs=24))
            top_cut = shapely.box(-1, zc + h / 2 + H['d_out_top'] - H['gasket_edge'], 1, 1); gpg = gpg.difference(top_cut)
            gpg = gpg.difference(shapely.box(-H['trap_y'], -1, H['trap_y'], zc - h / 2 + 1e-4))
        for gp_ in shapely.get_parts(gpg):
            if gp_.area < 1e-6: continue
            v_, f_ = ext(gp_, LENS['t'], H['step_u'], 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'gasket', v3, f3, MAT['foam'])
        for pp_, _ in PDS:      # [fix4-front2] crush pads moulded on the flange face (u = step_u) out to the body's inner skin (u = t): the module is pushed onto them
            v_, f_ = ext(pp_, LENS['t'] - float(Pd_.get('crush', 0.0)), H['step_u'] + 0.0002, 'yz'); v3, f3 = mapped(v_, f_, 0.0005); add(KB, KK, 'pad', v3, f3, MAT['housing'])
        if PDS: R['pads'] = dict(n=len(PDS), at_mm=[np.round(np.array(pp_.centroid.coords[0]) * 1000, 1).tolist() for pp_, _ in PDS], height_mm=round((H['step_u'] - LENS['t']) * 1000, 2))      # [fix4-front2]
        # vent membrane (ePTFE, adhesive) on the back plate
        vy, vz, vr = H['vent']; v_, f_ = rev((vy, vz, ub(vy)), (0, 0, 1.0), [(0, 0), (0, vr), (0.0004, vr), (0.0004, 0)], 24); v3, f3 = mapped(v_, orient_out(v_, f_)); add(KB, KK, 'vent', v3, f3, MAT['tpe'])
        # ---- 4. light bars, carriers, LED end boards (both sides)
        Bp = dict(P['bar'], **bd.get('bar', {})); Bd = dict(P['board'], **bd.get('board', {}));      # [v21] per-band overrides (the visor's bar runs along its top)
        zb0, zb1 = zc + Bp['z0'], zc + Bp['z1']; ct = Bp['carrier_t']
        for s in (1, -1):
            pg = Polygon([(Bp['y0'], Bp['u0']), (Bp['leg_y'], Bp['u0']), (Bp['y1'], Bp['u0'] + (Bp['y1'] - Bp['leg_y'])), (Bp['y1'], Bp['leg_u']),
                          (Bp['leg_y'], Bp['leg_u']), (Bp['leg_y'], Bp['u1']), (Bp['y0'], Bp['u1'])])
            if s < 0: pg = affinity.scale(pg, -1, 1, origin=(0, 0))
            v_, f_ = ext(pg, zb0, zb1, 'yu'); v3, f3 = mapped(v_, f_); add(KB, KK, 'bar', v3, f3, MAT['pmma'])
            cu = Polygon([(zb0 - 0.0001 - ct, Bp['u0'] - 0.00015), (zb0 - 0.0001, Bp['u0'] - 0.00015), (zb0 - 0.0001, Bp['u1'] + 0.0001), (zb1 + 0.0001, Bp['u1'] + 0.0001),
                          (zb1 + 0.0001, Bp['u0'] - 0.00015), (zb1 + 0.0001 + ct, Bp['u0'] - 0.00015), (zb1 + 0.0001 + ct, Bp['u1'] + 0.0001 + ct), (zb0 - 0.0001 - ct, Bp['u1'] + 0.0001 + ct)])
            ya, yb = sorted((s * (Bp['y0'] - 0.00015), s * (Bp['leg_y'] - 0.0002)))      # [fix4-front2] G5: the channel starts 0.05 inside the end plate (was: shared corner vertices -> non-manifold + self-x)
            v_, f_ = ext(cu, ya, yb, 'zu'); v3, f3 = mapped(v_, f_); add(KB, KK, 'carrier', v3, f3, MAT['white'])
            ye = s * (Bp['y0'] - 0.0001 - ct / 2); v_, f_ = ext(shapely.box(ye - ct / 2, zb0 - 0.0001 - ct, ye + ct / 2, zb1 + 0.0001 + ct), Bp['u0'] - 0.00015, Bp['u1'] + 0.0001 + ct, 'yz')
            v3, f3 = mapped(v_, f_); add(KB, KK, 'carrier', v3, f3, MAT['white'])
            zcb = zc + Bd.get('z_off', 0.0)      # [v21] the LED board level with its light bar (a visor's bar runs along the top)
            bpg = shapely.box(Bd['y0'], zcb - Bd['z_half'], Bd['y1'], zcb + Bd['z_half']).intersection(outline.buffer(-H['weld_in'] - 0.0003, quad_segs=24))
            bpg = bpg.buffer(-0.0005).buffer(0.0005, quad_segs=6)
            if s < 0: bpg = affinity.scale(bpg, -1, 1, origin=(0, 0))
            v_, f_ = ext(bpg, Bd['u0'], Bd['u0'] + Bd['t'], 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'pcb', v3, f3, MAT['pcb'])
            ly = s * 0.5 * (Bp['leg_y'] + Bp['y1']); lw, lh, lt = Bd['led']
            v_, f_ = ext(shapely.box(ly - lw / 2, zcb - lh / 2, ly + lw / 2, zcb + lh / 2), Bd['u0'] - lt, Bd['u0'], 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'led', v3, f3, MAT['led'])
        R['light_bar'] = dict(section_mm=[round((Bp['z1'] - Bp['z0']) * 1000, 1), round((Bp['u1'] - Bp['u0']) * 1000, 2)], y_mm=[Bp['y0'] * 1000, Bp['y1'] * 1000],
                              air_to_lens_mm=round((Bp['u0'] - LENS['t']) * 1000, 2), led='3030 mid-power at the outboard end, 45 deg TIR leg')
        # ---- 5. centre: front pupil (camera, ToF, status LED) / rear dot
        if bd.get('module', 'pupil' if sgn > 0 else 'dot') == 'pupil':      # [v21] the camera module goes where band.module says (default: the nose)
            Pu = P['pupil']; bw, bh, bu = Pu['board']
            pb = shapely.box(-bw, zc - bh, bw, zc + bh).buffer(-0.001).buffer(0.001, quad_segs=6).difference(Point(0.0, zc).buffer(0.0032, quad_segs=24))
            v_, f_ = ext(pb, bu, bu + 0.001, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'pcb', v3, f3, MAT['pcb'])
            cs, cd, cr, cu0 = Pu['cam']                                                       # camera: holder cs x cs (depth cd) behind the board, barrel r cr through it
            v_, f_ = ext(shapely.box(-cs / 2, zc - cs / 2, cs / 2, zc + cs / 2), bu + 0.001, bu + 0.001 + cd, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'camera', v3, f3, MAT['sensor'])
            v_, f_ = rev((0.0, zc, cu0), (0, 0, 1.0), [(0, 0), (0, cr - 0.0004), (0.0003, cr), (bu + 0.0012 - cu0, cr), (bu + 0.0012 - cu0, 0)], 40); v3, f3 = mapped(v_, orient_out(v_, f_))
            add(KB, KK, 'camera', v3, f3, MAT['metal'])
            v_, f_ = rev((0.0, zc, cu0 - 0.00005), (0, 0, 1.0), [(0, 0.0021), (0, 0.0024), (0.0001, 0.0024), (0.0001, 0.0021)], 40); v3, f3 = mapped(v_, orient_out(v_, f_))
            add(KB, KK, 'camera', v3, f3, MAT['sensor'])                                    # glass of the camera lens
            v_, f_ = rev((0.0, zc, LENS['t'] + 0.00006), (0, 0, 1.0), [(0, cr + 0.0002), (0, cr + 0.0012), (cu0 - LENS['t'] - 0.00004, cr + 0.0012), (cu0 - LENS['t'] - 0.00004, cr + 0.0002)], 40)      # [v21] 60 um off the lens (band-map residual up to 35 um)
            v3, f3 = mapped(v_, orient_out(v_, f_)); add(KB, KK, 'foam', v3, f3, MAT['foam'])          # anti-flare foam ring between the barrel and the lens
            tw, th_, tt = Pu['tof']; ty = Pu['tof_y']
            v_, f_ = ext(shapely.box(ty - tw / 2, zc - th_ / 2, ty + tw / 2, zc + th_ / 2), bu - tt, bu, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'tof', v3, f3, MAT['sensor'])
            fg = shapely.box(ty - tw / 2 - 0.0008, zc - th_ / 2 - 0.0008, ty + tw / 2 + 0.0008, zc + th_ / 2 + 0.0008).difference(Point(ty, zc).buffer(0.0016, quad_segs=16))
            v_, f_ = ext(fg, LENS['t'] + 0.00006, bu - tt + 0.0001, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'foam', v3, f3, MAT['foam'])   # ToF crosstalk gasket
            ly = Pu['led_y']; v_, f_ = ext(shapely.box(ly - 0.0008, zc - 0.0004, ly + 0.0008, zc + 0.0004), bu - 0.0006, bu, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'led', v3, f3, MAT['led'])
            v_, f_ = rev((ly, zc, LENS['t'] + 0.0001), (0, 0, 1.0), [(0, 0), (0, Pu['pipe_r']), (bu - 0.0006 - LENS['t'] - 0.00015, Pu['pipe_r']), (bu - 0.0006 - LENS['t'] - 0.00015, 0)], 32)
            v3, f3 = mapped(v_, orient_out(v_, f_)); add(KB, KK, 'pipe', v3, f3, MAT['pmma'])
            R['pupil'] = dict(camera_mm=[cs * 1000, round((bu + 0.001 + cd) * 1000, 1)], barrel_gap_to_lens_mm=round((cu0 - LENS['t']) * 1000, 2), tof_air_mm=round((bu - tt - LENS['t']) * 1000, 2))
        else:
            Dt = P['dot']; dw, dh, du = Dt['board']
            v_, f_ = ext(shapely.box(-dw, zc - dh, dw, zc + dh).buffer(-0.001).buffer(0.001, quad_segs=6), du, du + 0.001, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'pcb', v3, f3, MAT['pcb'])
            lw, lh, lt = Dt['led']; v_, f_ = ext(shapely.box(-lw / 2, zc - lh / 2, lw / 2, zc + lh / 2), du - lt, du, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'led', v3, f3, MAT['led'])
            v_, f_ = rev((0.0, zc, LENS['t'] + 0.00025), (0, 0, 1.0), [(0, 0), (0, Dt['pipe_r']), (du - lt - LENS['t'] - 0.0004, Dt['pipe_r']), (du - lt - LENS['t'] - 0.0004, 0)], 40)
            v3, f3 = mapped(v_, orient_out(v_, f_)); add(KB, KK, 'pipe', v3, f3, MAT['pmma'])
            v_, f_ = rev((0.0, zc, LENS['t'] + 0.0002), (0, 0, 1.0), [(0, Dt['pipe_r'] + 0.0001), (0, Dt['pipe_r'] + 0.0009), (du - LENS['t'] - 0.0003, Dt['pipe_r'] + 0.0009),
                                                                     (du - LENS['t'] - 0.0003, Dt['pipe_r'] + 0.0001)], 40)
            v3, f3 = mapped(v_, orient_out(v_, f_)); add(KB, KK, 'carrier', v3, f3, MAT['white'])      # white sleeve round the dot pipe (no spill into the mask)
        # ---- 6. module connector (left end, on the back plate) + right-angle sealed plug
        Cn = P['connector']; cyy = Cn['y']; czz = zc + Cn.get('dz', 0.0); cpg = shapely.box(cyy - Cn['w'] / 2, czz - Cn['h'] / 2, cyy + Cn['w'] / 2, czz + Cn['h'] / 2).buffer(-0.0015).buffer(0.0015, quad_segs=6)
        ubc = float(ub(cyy)); sh = cpg.difference(cpg.buffer(-Cn['wall']))
        v_, f_ = ext(cpg, ubc - 0.0005, ubc + 0.0012, 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'shroud', v3, f3, MAT['housing'])
        v_, f_ = ext(sh, ubc + 0.0011, ubc + Cn['len'], 'yz'); v3, f3 = mapped(v_, f_); add(KB, KK, 'shroud', v3, f3, MAT['housing'])
        c0 = M(np.array([[cyy, czz, ubc + Cn['len'] - 0.004]]))[0]; c1 = M(np.array([[cyy, czz, ubc + Cn['len'] - 0.004 + Cn['plug_len']]]))[0]; ax = (c1 - c0) / np.linalg.norm(c1 - c0)
        e1, e2, e3 = frame3(ax); pc = 0.5 * (c0 + c1); ins = cpg.buffer(-Cn['wall'] - 0.0002)
        v_, f_ = wbox(pc, e1, e2, e3, (ins.bounds[2] - ins.bounds[0], ins.bounds[3] - ins.bounds[1], np.linalg.norm(c1 - c0)), r=0.001); add(HB, HK, 'plug', v_, f_, MAT['conn'])
        hood_top = c1 - ax * 0.0035 + np.array([0, 0, 1.0]) * (Cn['h'] / 2)                   # right-angle cable exit (upwards) at the plug's back end
        R['connector'] = dict(at_mm=(pc * 1000).round(1).tolist(), axis=np.round(ax, 3).tolist(), cable_exit_mm=(hood_top * 1000).round(1).tolist())
        # ---- 7. bosses on the body wall + ears + screws (parallel to the module axis am)
        Bo = dict(P['bosses'], **bd.get('bosses', {})); axb = np.array([-sgn, 0.0, 0.0]) if Bo.get('axis', 'module') == 'x' else am      # [fix4-front2] M6: 'x' = horizontal boss axes (one internal slide per end)
        e1b, e2b, _ = frame3(axb); cth = float(axb @ am); flt = float(Bo.get('float', 0.0)); Rb = []      # [fix4-front2] float: ear-to-boss gap (the ear slides on the screw's sleeve)
        for (by, bz_off) in Bo['at']:
            for s in (1, -1):
                y_ = s * by; z_ = zc + bz_off if bz_off else zc
                if bz_off: z_ = zc - h / 2 - 0.010                                              # lower bosses: 10 mm under the lens
                p0 = M(np.array([[y_, z_, LENS['t']]]))[0]                                     # on the body's inner skin
                Ls_ = (Bo['seat_u'] - LENS['t']) / cth; topE = p0 + axb * Ls_; top = p0 + axb * (Ls_ - flt / cth); base = p0 - axb * Bo['sink']      # [fix4-front2] axb, ear seat topE, boss top (float)
                ro, rh = Bo['od'] / 2, Bo['hole'] / 2; Lb = np.linalg.norm(top - base)
                v_, f_ = rev(base, axb, [(0, rh * 0.6), (0, ro), (Lb, ro), (Lb, rh), (Bo['sink'] + 0.0001, rh), (Bo['sink'] + 0.0001, rh * 0.6)], 32)      # [fix4-front2] axb
                v_ = np.asarray(v_); add(KB, KK, 'boss', v_, orient_out(v_, f_), MAT['boss'])
                for gs in (1, -1):                                                              # two gussets along +-e2 (vertical-ish), 1.2 thick
                    d_ = e2b * gs; hg = 0.6 * (Lb - Bo['sink']) + Bo['sink']      # [fix4-front2] along axb (planes containing the slide direction)
                    q = [base + d_ * (ro - 0.0003), base + d_ * (ro + 0.0025), base + d_ * (ro + 0.0025) + axb * Bo['sink'], base + d_ * (ro - 0.0003) + axb * hg]
                    T_ = np.array(q); nn = e1b * Bo['gusset_t'] / 2; gv, gf = stack([T_ - nn, T_ + nn], cap0=True, cap1=True); add(KB, KK, 'boss', gv, orient_out(gv, gf), MAT['boss'])
                # ear: plate perpendicular to am at the seat, reaching into the housing wall (d = -1.0 mm .. from the outline)
                tgt2 = np.array([y_ - s * 0.0105, zc]) if not bz_off else np.array([y_, zc - h / 2 + 0.0012])
                tgt = M(np.array([[tgt2[0], tgt2[1], Bo['seat_u'] + Bo['ear_t'] / 2]]))[0]; dv = tgt - (topE + axb * Bo['ear_t'] / 2); dv -= (dv @ axb) * axb      # [fix4-front2] topE, axb
                Le = np.linalg.norm(dv); de = dv / Le; dn = np.cross(axb, de)
                # holes: the two END ears locate the module (fitted 2.6 hole for the 2.5 screw: left round, right a 2.6 x 3.6 slot along y for the span tolerance /
                # expansion); the two lower ears have 3.2 clearance holes (no over-constraint)
                if not bz_off: hole = Point(0, 0).buffer(Bo['fit_hole'] / 2, quad_segs=16) if s > 0 else LineString([(-Bo['slot'] / 2, 0), (Bo['slot'] / 2, 0)]).buffer(Bo['fit_hole'] / 2, quad_segs=12)
                else: hole = Point(0, 0).buffer(Bo['clear_hole'] / 2, quad_segs=16)
                ear = shapely.LineString([(0, 0), (Le, 0)]).buffer(Bo['ear_w'] / 2, cap_style=2).union(Point(0, 0).buffer(Bo['ear_w'] / 2, quad_segs=12)).difference(hole)
                ev, ef = lidmech.extrude(ear, 0.0, Bo['ear_t'], (topE, de, dn, axb)); add(KB, KK, 'ear', ev, orient_out(ev, ef), MAT['housing'])      # [fix4-front2] topE, axb
                sd_, sl_, hd_, hh_ = Bo['screw']; st = topE + axb * Bo['ear_t']
                if Bo.get('sleeve'):      # [fix4-front2] m9: floating ear = screw + steel sleeve (compression limiter, seated on the boss top) + wave spring under the head (one solid of revolution)
                    so_, wd_, wh_ = Bo['sleeve']; sl2_ = flt + Bo['ear_t'] + wh_; st = st + axb * wh_      # sleeve OD, spring OD, spring height (compressed)
                    sv, sf = rev(st + axb * hh_, -axb, [(0, 0), (0, hd_ / 2 - 0.0003), (0.0003, hd_ / 2), (hh_, hd_ / 2), (hh_, wd_ / 2), (hh_ + wh_, wd_ / 2), (hh_ + wh_, so_ / 2),
                                                        (hh_ + sl2_, so_ / 2), (hh_ + sl2_, sd_ / 2), (hh_ + sl2_ + sl_ - Bo['ear_t'] - 0.0008, sd_ / 2), (hh_ + sl2_ + sl_ - Bo['ear_t'], 0.0004), (hh_ + sl2_ + sl_ - Bo['ear_t'], 0)], 24)
                else: sv, sf = rev(st + am * hh_, -am, [(0, 0), (0, hd_ / 2 - 0.0003), (0.0003, hd_ / 2), (hh_, hd_ / 2), (hh_, sd_ / 2), (hh_ + sl_ - 0.0008, sd_ / 2), (hh_ + sl_, 0.0004), (hh_ + sl_, 0)], 24)
                add(KB, KK, 'screw', sv, orient_out(np.asarray(sv), sf), MAT['metal']); Rb.append(np.round(p0 * 1000, 1).tolist())
        R['bosses'] = dict(at_mm=Rb, od_mm=Bo['od'] * 1000, hole_mm=Bo['hole'] * 1000, wall_mm=round((Bo['od'] - Bo['hole']) / 2 * 1000, 2), wall_over_t=round((Bo['od'] - Bo['hole']) / 2 / LENS['t'], 2),
                           gusset_mm=Bo['gusset_t'] * 1000, height_mm=round((Bo['seat_u'] - LENS['t']) * 1000, 1), screw='PT K25 x %.0f' % (Bo['screw'][1] * 1000),
                           engagement_mm=round((Bo['screw'][1] - Bo['ear_t']) * 1000, 1), tip_above_inner_skin_mm=round((Bo['seat_u'] - (Bo['screw'][1] - Bo['ear_t']) - LENS['t']) * 1000, 1),
                           axis=np.round(axb, 4).tolist(), axis_to_module_axis_deg=round(float(np.degrees(np.arccos(np.clip(cth, -1, 1)))), 2), float_mm=flt * 1000,      # [fix4-front2]
                           sleeve_mm=[x_ * 1000 for x_ in Bo['sleeve']] if Bo.get('sleeve') else None)      # [fix4-front2]
        R['stack'] = stack_(P, LENS, H)      # [fix4-front2] m9
        # ---- 8. harness (body side) from the plug: waypoints in world (metres) from the parameters, then grommet, clip, service connector, chassis lead
        Hn = P['harness'].get(nm, {}); cd_ = P['harness']['cable_d'][nm]
        if Hn.get('path'):
            wp = np.r_[hood_top[None], np.asarray(Hn['path'], float)]; pth = spline_path(wp, 0.0008); r0_ = float(bend_r(pth)[1:-1].min())
            pth, r1_ = relax_path(pth, 1.6 * cd_ / 2); tv, tf = tube_path(pth, cd_ / 2, 16); add(HB, HK, 'cable', tv, tf, MAT['cable'])      # [fix4-front2] G5 bend >= 1.6 r
            boot_v, boot_f = rev(hood_top - np.array([0, 0, 0.0015]), (0, 0, 1.0), [(0, 0), (0, cd_ / 2 + 0.0012), (0.004, cd_ / 2 + 0.0005), (0.007, cd_ / 2 + 0.0002), (0.007, 0)], 20)
            add(HB, HK, 'plug', boot_v, orient_out(np.asarray(boot_v), boot_f), MAT['tpe'])
            R['harness'] = dict(cable_d_mm=cd_ * 1000, length_mm=round(float(np.linalg.norm(np.diff(pth, axis=0), axis=1).sum()) * 1000, 0), min_bend_r_mm=[round(r0_ * 1000, 2), round(r1_ * 1000, 2)])      # [fix4-front2]
            gm = Hn.get('grommet')
            if gm:                                                                              # TPE grommet in the deck hole [x, y, z of the deck top, hole r, deck t]
                g0 = np.asarray(gm[:3], float); gr = gm[3]; td = gm[4] if len(gm) > 4 else 0.0025; ez = np.array([0, 0, 1.0])
                gv, gf = rev(g0 - ez * (td + 0.001), ez, [(0, cd_ / 2 - 0.0002), (0, gr + 0.0012), (0.001, gr + 0.0012), (0.001, gr), (td + 0.001, gr), (td + 0.001, gr + 0.0012),
                                                         (td + 0.002, gr + 0.0012), (td + 0.0032, cd_ / 2 + 0.0004), (td + 0.0032, cd_ / 2 - 0.0002)], 32)
                add(HB, HK, 'grommet', gv, gf, MAT['tpe']); R['grommet_mm'] = dict(at=(g0 * 1000).round(1).tolist(), hole_d=gr * 2000, bore_d=round((cd_ - 0.0004) * 1000, 2))
            for cl in Hn.get('clips', []):                                                      # P-clips moulded on the deck / wall: ring round the cable + foot
                c_ = np.asarray(cl[:3], float); k_ = np.argmin(np.linalg.norm(pth - c_, axis=1)); tg = pth[min(k_ + 1, len(pth) - 1)] - pth[max(k_ - 1, 0)]; tg /= np.linalg.norm(tg)
                pcen = pth[k_]; cv, cf = rev(pcen - tg * 0.002, tg, [(0, cd_ / 2 + 0.0001), (0, cd_ / 2 + 0.0013), (0.004, cd_ / 2 + 0.0013), (0.004, cd_ / 2 + 0.0001)], 24)
                add(HB, HK, 'clip', cv, orient_out(np.asarray(cv), cf), MAT['boss'])
                dn_ = np.asarray(cl[3:6], float); dn_ /= np.linalg.norm(dn_); e1c = np.cross(tg, dn_); e1c /= np.linalg.norm(e1c)
                fh = cl[6]; fv, ff = wbox(pcen + dn_ * (cd_ / 2 + fh / 2 + 0.0008), e1c, tg, dn_, (0.0024, 0.004, fh + 0.0012)); add(HB, HK, 'clip', fv, ff, MAT['boss'])
            sv = Hn.get('service'); sv_seal_ = P['harness'].get('service_sealed', False)      # [fix4-front2] S12
            if sv:                                                                              # service connector pair (body plug + chassis receptacle) in a cradle
                s0 = np.asarray(sv[:3], float); sa = np.asarray(sv[3:6], float); sa /= np.linalg.norm(sa); se1, se2, se3 = frame3(sa)
                L1, L2, sw, shh = 0.012, 0.014, 0.012, 0.008
                v_, f_ = wbox(s0 + sa * L1 / 2, se1, se2, se3, (sw, shh, L1), r=0.0012); add(HB, HK, 'svc', v_, f_, MAT['conn'])
                v_, f_ = wbox(s0 + sa * (L1 + L2 / 2 - 0.0005), se1, se2, se3, (sw - 0.001, shh - 0.001, L2), r=0.0012); add(HB, HK, 'svc', v_, f_, MAT['conn'])
                v_, f_ = wbox(s0 + sa * (L1 - 0.0025) + se2 * (shh / 2 + 0.0004), se1, se2, se3, (0.004, 0.0008, 0.006)); add(HB, HK, 'svc', v_, f_, MAT['tpe'])     # latch
                if sv_seal_:      # [fix4-front2] S12: IP67 sealed pair: mating seal collar (TPE) over the joint + a wire-seal boot where the body cable enters the plug
                    rr_ = shapely.box(-sw / 2 - 0.0005, -shh / 2 - 0.0005, sw / 2 + 0.0005, shh / 2 + 0.0005).buffer(-0.0017).buffer(0.0017, quad_segs=6)      # 13 x 9 R1.7
                    rr_ = rr_.difference(shapely.box(-sw / 2 + 0.00055, -shh / 2 + 0.00055, sw / 2 - 0.00055, shh / 2 - 0.00055).buffer(-0.00115).buffer(0.00115, quad_segs=6))      # on the 11 x 7 R1.2 receptacle (0.05 fused)
                    cv_, cf_ = lidmech.extrude(rr_, 0.0, 0.0024, (s0 + sa * L1, se1, se2, sa)); add(HB, HK, 'svc', cv_, orient_out(cv_, cf_), MAT['tpe'])      # seated on the receptacle, the plug face closes on it; clear of the cradle saddles
                    bv_, bf_ = rev(s0 + sa * 0.0008, -sa, [(0, 0), (0, cd_ / 2 + 0.0012), (0.004, cd_ / 2 + 0.0004), (0.004, 0)], 20); add(HB, HK, 'svc', bv_, orient_out(np.asarray(bv_), bf_), MAT['tpe'])
                for cr_ in sv[6:8]:                                                              # cradle: two U-saddles fused into the deck top
                    cc = s0 + sa * cr_; cz_ = float(sv[8])
                    for sx in (1, -1):
                        v_, f_ = wbox(np.array([cc[0], cc[1], 0.5 * (cz_ + cc[2])]) + se1 * sx * (sw / 2 + 0.0007), se1, sa, np.array([0, 0, 1.0]), (0.0012, 0.003, cc[2] - cz_ + shh * 0.7))
                        add(HB, HK, 'cradle', v_, f_, MAT['boss'])
                cl_ = Hn.get('chassis_lead')
                if cl_:
                    wp2 = np.r_[(s0 + sa * (L1 + L2 + 0.002))[None], np.asarray(cl_, float)]; p2 = spline_path(wp2, 0.0008); r2_ = float(bend_r(p2)[1:-1].min())
                    p2, r3_ = relax_path(p2, 1.6 * cd_ / 2); tv, tf = tube_path(p2, cd_ / 2, 16); add(HB, HK, 'chassis_lead', tv, tf, MAT['cable'])      # [fix4-front2] G5
                    R['chassis_lead_bend_r_mm'] = [round(r2_ * 1000, 2), round(r3_ * 1000, 2)]      # [fix4-front2]
                    bv, bf = rev(s0 + sa * (L1 + L2 - 0.0008), sa, [(0, 0), (0, cd_ / 2 + 0.001), (0.004, cd_ / 2 + 0.0003), (0.004, 0)], 20); add(HB, HK, 'chassis_lead', bv, orient_out(np.asarray(bv), bf), MAT['tpe'])
                    R['chassis_lead_mm'] = round(float(np.linalg.norm(np.diff(p2, axis=0), axis=1).sum()) * 1000, 0)
            for jh in Hn.get('j_hooks', []):                                                    # open J-hooks for the chassis lead on the deck top (lead lifts out)
                jc = np.asarray(jh[:3], float); ja = np.asarray(jh[3:6], float); ja /= np.linalg.norm(ja); je1, je2, je3 = frame3(ja)
                jv, jf = rev(jc, ja, [(-0.002, cd_ / 2 + 0.0001), (-0.002, cd_ / 2 + 0.0013), (0.002, cd_ / 2 + 0.0013), (0.002, cd_ / 2 + 0.0001)], 24)
                jv = np.asarray(jv); jf = np.asarray(jf); cc_ = jv[jf].mean(1); keep = ((cc_ - jc) @ np.array([0, 0, 1.0])) < 0.0006       # lower half + sides only (open on top)
                add(HB, HK, 'clip', jv, jf[keep], MAT['boss'])
                fv, ff = wbox(jc - np.array([0, 0, cd_ / 2 + 0.0013 + jh[6] / 2]), je1, ja, np.array([0, 0, 1.0]), (0.003, 0.004, jh[6] + 0.0006)); add(HB, HK, 'clip', fv, ff, MAT['boss'])
        rep[nm] = R
    # ---- 9. status halo: clear feed boss on the lid underside at the ring's front point; LED emitter on the control panel's front strip
    Hl = P['halo']; sp = X['lid'].get('stoppad'); sk3 = X['lid'].get('stopkey')      # [fix3-int]
    if Hl.get('on', True) and sk3 is not None and sk3.get('v4'):      # [fix4-stop2] v4 STOP key: ring light guide + vertical pipe in the dark post, hall pair on the board (stop4.py)
        import stop4; rep['halo'] = stop4.halo_feed4(sk3, Hl, KB, KK, add, QB, stack, weld, orient_out, consistent, MAT, wbox)      # [fix4-stop2]
    elif Hl.get('on', True) and sk3 is not None:      # [fix3-int] ONE halo = the opal halo boss of the raised STOP key (stopkit), fed from the same panel emitter:
        #   a clear PC light pipe (d 2 boss_r, in the halo's 2K shot) runs STRAIGHT from a window in the lid skin under the boss's front sector (its top fused
        #   0.1 into the boss bottom) down-forward to gap_closed over the emitter LED on the panel's front strip at feed_x. It passes under the front ring rib
        #   (the orphaned centre stub rib between the STOP keep-out and that ring is kept out, design_gen [fix3-int]), in front of the real key, behind the lip.
        xk_, yk_, z0_, gx_ = sk3['xk'], sk3['yk'], sk3['z0'], sk3['gx']; rb_ = Hl['boss_r']; ex, ey, et = Hl['emitter']
        xf = float(Hl.get('feed_x', 0.0291)); xt = xk_ + float(Hl.get('boss_rho', 0.0136))
        zt = z0_ - sk3['fuse'] + gx_ * (xt - xk_) + 0.0001                                    # top: 0.1 into the halo boss bottom (the skin window is filled by the 2K shot)
        zp = float(Hl.get('panel_z', 0.25242)); zled = zp + et + Hl['led_h']; zb = zled + Hl['gap_closed']
        ax_ = np.array([xt - xf, 0.0, zt - zb]); sa_ = ax_[2] / np.linalg.norm(ax_); ux_ = ax_[0] / max(ax_[2], 1e-9)      # horizontal ellipse rings along the axis
        zz_ = np.r_[zb, zb + 0.0003, np.linspace(zb + 0.0006, zt, 24)]; rr_ = np.r_[rb_ - 0.0003, rb_ - 0.00004, np.full(24, rb_)]; q_ = np.linspace(0, 2 * np.pi, 40, endpoint=False)
        Rg = np.array([np.c_[xf + ux_ * (z_ - zb) + r_ / sa_ * np.cos(q_), yk_ + r_ * np.sin(q_), np.full(len(q_), z_)] for z_, r_ in zip(zz_, rr_)])
        qv, qf = stack(Rg, closed=True, cap0=True, cap1=True); qv, qf = weld(qv, qf); QB.add(qv, orient_out(np.asarray(qv), consistent(qf)), MAT['clear'])
        v_, f_ = wbox((xf, yk_, zp + et / 2 + 0.00002), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (ex, ey, et), r=0.0008); add(KB, KK, 'emitter', v_, f_, MAT['pcb'])
        v_, f_ = wbox((xf, yk_, zp + et + Hl['led_h'] / 2), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (0.0016, 0.0016, Hl['led_h'])); add(KB, KK, 'emitter', v_, f_, MAT['led'])
        rep['halo'] = dict(design='[fix3-int] opal halo boss of the STOP key (stopkit) fed by an inclined clear pipe through a skin window',
                           feed_top_mm=[round(xt * 1000, 2), round(yk_ * 1000, 2), round(zt * 1000, 2)], feed_bottom_mm=[round(xf * 1000, 2), round(yk_ * 1000, 2), round(zb * 1000, 2)],
                           pipe_d_mm=rb_ * 2000, pipe_length_mm=round(float(np.linalg.norm(ax_)) * 1000, 1), pipe_elevation_deg=round(float(np.degrees(np.arcsin(sa_))), 1),
                           boss_annulus_mm=[sk3['bore_r'] * 1000, sk3['boss_r'] * 1000], top_at_radius_mm=round((xt - xk_) * 1000, 2),
                           led_top_z_mm=round(zled * 1000, 2), gap_closed_mm=round(Hl['gap_closed'] * 1000, 2), emitter_mm=[round(xf * 1000, 1), ex * 1000, ey * 1000, et * 1000])
    elif Hl.get('on', True) and sp is not None:      # [fix3-int] f2 flush pad (no STOP key): the fix-lights boss at the ring's front point
        zi = X['lid']['zi']; rr = sp['r'] + sp['hg'] + sp['hw'] / 2; xf = sp['x'] + rr
        zt = float(zi(np.array([[xf, 0.0]]))[0]) - X['lid']['t_lid'] + 0.0004; ex, ey, et = Hl['emitter']
        zp = float(Hl.get('panel_z', 0.25242)); zled = zp + et + Hl['led_h']; zb = zled + Hl['gap_closed']
        qv, qf = rev((xf, 0.0, zt), (0, 0, -1.0), [(0, 0), (0, Hl['boss_r'] + 0.0004), (0.0008, Hl['boss_r']), (zt - zb - 0.0003, Hl['boss_r']), (zt - zb, Hl['boss_r'] - 0.0003), (zt - zb, 0)], 28)
        QB.add(qv, orient_out(np.asarray(qv), qf), MAT['clear'])
        v_, f_ = wbox((xf, 0.0, zp + et / 2 + 0.00002), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (ex, ey, et), r=0.0008); add(KB, KK, 'emitter', v_, f_, MAT['pcb'])
        v_, f_ = wbox((xf, 0.0, zp + et + Hl['led_h'] / 2), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0), (0.0016, 0.0016, Hl['led_h'])); add(KB, KK, 'emitter', v_, f_, MAT['led'])
        rep['halo'] = dict(feed_at_mm=[round(xf * 1000, 1), 0.0], boss_d_mm=Hl['boss_r'] * 2000, boss_bottom_z_mm=round(zb * 1000, 2), led_top_z_mm=round(zled * 1000, 2),
                           gap_closed_mm=round(Hl['gap_closed'] * 1000, 2), boss_length_mm=round((zt - zb) * 1000, 1))
    lv, lf, ll = LZ.out(); kv, kf, kl = KB.out(); hv_, hf_, hl_ = HB.out(); qv_, qf_, ql_ = QB.out()
    # lens shading normals: area x angle weighted vertex normals (no weight for the slivers of the cut (the lens solid is G1 everywhere: roll R0.2 into the
    # edge face, inner round R0.2 into the inner face), so the glossy edge roll reads as one continuous highlight
    P_ = lv[lf]; fn_ = np.cross(P_[:, 1] - P_[:, 0], P_[:, 2] - P_[:, 0]); fa_ = np.linalg.norm(fn_, axis=1); fn_ = fn_ / np.maximum(fa_, 1e-30)[:, None]
    ev_ = np.roll(P_, -1, 1) - P_; ev_ /= np.maximum(np.linalg.norm(ev_, axis=2, keepdims=True), 1e-30); ang_ = np.arccos(np.clip(-(ev_ * np.roll(ev_, 1, 1)).sum(2), -1, 1)) * (fa_ > 1e-12)[:, None]
    lvn = np.zeros_like(lv); wf_ = fa_[:, None] * np.minimum(ang_, 2.0)                   # area x angle (capped): slivers of the cut lines get no say
    for c_ in range(3): np.add.at(lvn, lf[:, c_], fn_ * wf_[:, c_:c_ + 1])
    lvn /= np.maximum(np.linalg.norm(lvn, axis=1, keepdims=True), 1e-30)
    arrays = dict(lights_lens_v=lv, lights_lens_f=lf, lights_lens_lab=ll, lights_lens_side=np.concatenate(LZs) if LZs else np.zeros(0, np.uint8),
                  lights_lens_id=np.concatenate(LZid) if LZid else np.zeros(0, np.uint8), lights_lens_vn=lvn.astype(np.float32),
                  lights_kit_v=kv, lights_kit_f=kf, lights_kit_lab=kl, lights_kit_kind=np.concatenate([np.full(n, k, np.int8) for n, k in KK]) if KK else np.zeros(0, np.int8),
                  lights_harn_v=hv_, lights_harn_f=hf_, lights_harn_lab=hl_, lights_harn_kind=np.concatenate([np.full(n, k, np.int8) for n, k in HK]) if HK else np.zeros(0, np.int8),
                  lights_halo_v=qv_, lights_halo_f=qf_, lights_halo_lab=ql_, lights_kinds=np.array(KINDS))
    log('lights: lens %d, kit %d, harness %d, halo %d faces' % (len(lf), len(kf), len(hf_), len(qf_)))
    return dict(arrays=arrays, report=rep)


if __name__ == '__main__':      # standalone iteration: python lightkit.py DUMP.pkl IN.npz OUT.npz [fix3_lights.json]
    import sys, pickle
    X = pickle.load(open(sys.argv[1], 'rb'))
    if len(sys.argv) > 4: X['P'] = json.load(open(sys.argv[4]))['lights']
    R = build(X); d = dict(np.load(sys.argv[2])); d = {k: v for k, v in d.items() if not k.startswith('lights_')}; d.update(R['arrays'])
    np.savez(sys.argv[3], **d); print(json.dumps(R['report'], indent=1, default=float))
