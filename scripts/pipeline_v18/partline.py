# [fix2] G4 / G2 / R2E2: the side-view part line between the light cover and the dark core as an EXPLICIT 2-D curve made only of
# level lines, circle arcs and true-circle R fillets (tangent construction), and its exact signed distance (> 0 on the cover side).
#   nose brow (level, z_nose) -R- wheel orbit (circle about the drive-wheel hub, R_o) -R- base line (level, z_base) -R- tail arc -R- tail brow
# Tail arc modes:
#   'orbit' : a second circle of the orbit radius about a virtual hub (x_v, z_hub): it meets the base line and the brow at the same angles
#             as the wheel orbit does; both corners get the same fillet R (the tail block's front edge leans BACK = positive draft for the
#             cover's vertical removal).
#   'sweep' : one convex arc of radius R_t tangent to the tail brow at x_top (no corner there), R fillet into the base line.
# Every corner is an exact circle, so the joint is the same at both ends and on both sides (G4); x_line(z) never increases with z
# along the tail rise, so the cover lifts straight off (R2E2).
import numpy as np, shapely
from shapely.geometry import Polygon


def _arc_pts(c, r, a0, a1, n=160):
    a = np.linspace(a0, a1, n); return np.c_[c[0] + r * np.cos(a), c[1] + r * np.sin(a)]


def _fillet_line_circle(zl, c, R, r, side, below_region=True, circle_inside=True):
    """Fillet of radius r rounding the CONCAVE corner of the union {z < zl} U disk(c, R) (fillet circle above the line, outside the disk).
    side = +1: the corner on the +x side of the disk, -1: on the -x side. Returns fillet centre, tangent point on the line, on the circle."""
    zf = zl + r; dz = zf - c[1]; dx = np.sqrt(max((R + r) ** 2 - dz ** 2, 0.0)); cf = np.array([c[0] + side * dx, zf])
    tl = np.array([cf[0], zl]); u = (cf - c) / np.linalg.norm(cf - c); tc = c + u * R
    return cf, tl, tc


def build(OB, WX=0.119, WZ=0.1145):
    Ro = OB.get('r', 0.128); zb = OB.get('z_base', 0.123); zn = OB.get('z_nose', 0.19); fr = OB.get('fillet', 0.02)
    TL = OB.get('tail', {}); mode = TL.get('mode', 'orbit'); frt = TL.get('fillet', fr)
    H = np.array([WX, WZ]); prims = []; pts = []
    # ---- front: brow / orbit fillet (concave corner of core = nose block U orbit disk): fillet above the brow, outside the disk, +x side
    c1, t1l, t1c = _fillet_line_circle(zn, H, Ro, fr, +1)
    # ---- rear of the wheel: orbit / base fillet, -x side
    c2, t2l, t2c = _fillet_line_circle(zb, H, Ro, fr, -1)
    ang = lambda c, p: np.arctan2(p[1] - c[1], p[0] - c[0])
    pts.append(np.array([[1.0, zn], [t1l[0], zn]])); prims.append(('seg', np.array([t1l[0], zn]), np.array([1.0, zn])))
    a0, a1 = ang(c1, t1l), ang(c1, t1c)
    if a1 > a0: a1 -= 2 * np.pi
    pts.append(_arc_pts(c1, fr, a0, a1, 40)); prims.append(('arc', c1, fr, a1, a0))
    b0, b1 = ang(H, t1c), ang(H, t2c)
    pts.append(_arc_pts(H, Ro, b0, b1, 400)); prims.append(('arc', H, Ro, b0, b1))
    a0, a1 = ang(c2, t2c), ang(c2, t2l)
    if a1 > a0: a1 -= 2 * np.pi
    pts.append(_arc_pts(c2, fr, a0, a1, 40)); prims.append(('arc', c2, fr, a1, a0))
    rep = dict(front_fillet_centre_mm=(c1 * 1000).round(2).tolist(), front_brow_tangent_x_mm=round(t1l[0] * 1000, 2), front_orbit_tangent_mm=(t1c * 1000).round(2).tolist(),
               rear_wheel_fillet_centre_mm=(c2 * 1000).round(2).tolist(), base_tangent_x_mm=round(t2l[0] * 1000, 2))
    if mode == 'orbit':      # virtual hub behind the tail block: the same circle as the wheel orbit
        Rt = TL.get('R', Ro); zv = TL.get('z_hub', WZ); xv = TL['x_hub']; Ct = np.array([xv, zv])
        c3, t3l, t3c = _fillet_line_circle(zb, Ct, Rt, frt, +1)                              # base / tail circle: concave core corner (union)
        # brow / tail circle: CONVEX corner of the core = intersection {z < zn} n disk -> rounding circle inside both, tangent to the brow from below
        rr = TL.get('fillet_top', frt); zf = zn - rr; dz = zf - zv; dx = np.sqrt(max((Rt - rr) ** 2 - dz ** 2, 0.0)); c4 = np.array([xv + dx, zf])
        t4l = np.array([c4[0], zn]); u = (c4 - Ct) / np.linalg.norm(c4 - Ct); t4c = Ct + u * Rt
        pts.append(np.array([t2l, t3l])); prims.append(('seg', t3l, t2l))
        a0, a1 = ang(c3, t3l), ang(c3, t3c)
        if a1 > a0: a1 -= 2 * np.pi
        pts.append(_arc_pts(c3, frt, a0, a1, 40)); prims.append(('arc', c3, frt, a1, a0))
        b0, b1 = ang(Ct, t3c), ang(Ct, t4c)
        pts.append(_arc_pts(Ct, Rt, b0, b1, 200)); prims.append(('arc', Ct, Rt, b0, b1))
        a0, a1 = ang(c4, t4c), ang(c4, t4l)
        if a1 < a0: a1 += 2 * np.pi
        pts.append(_arc_pts(c4, rr, a0, a1, 40)); prims.append(('arc', c4, rr, a0, a1))
        rep.update(tail_mode='orbit', tail_hub_mm=(Ct * 1000).round(2).tolist(), tail_R_mm=Rt * 1000, tail_base_tangent_x_mm=round(t3l[0] * 1000, 2),
                   tail_brow_tangent_x_mm=round(t4l[0] * 1000, 2), tail_circle_span_mm=[(t3c * 1000).round(2).tolist(), (t4c * 1000).round(2).tolist()])
        xbrow = t4l[0]
    else:                    # 'sweep': convex arc tangent to the brow at x_top
        Rt = TL.get('R', 0.09); xt = TL['x_top']; Ct = np.array([xt, zn - Rt])
        c3, t3l, t3c = _fillet_line_circle(zb, Ct, Rt, frt, +1)
        pts.append(np.array([t2l, t3l])); prims.append(('seg', t3l, t2l))
        a0, a1 = ang(c3, t3l), ang(c3, t3c)
        if a1 > a0: a1 -= 2 * np.pi
        pts.append(_arc_pts(c3, frt, a0, a1, 40)); prims.append(('arc', c3, frt, a1, a0))
        b0, b1 = ang(Ct, t3c), np.pi / 2
        pts.append(_arc_pts(Ct, Rt, b0, b1, 300)); prims.append(('arc', Ct, Rt, b0, b1))
        rep.update(tail_mode='sweep', tail_centre_mm=(Ct * 1000).round(2).tolist(), tail_R_mm=Rt * 1000, tail_base_tangent_x_mm=round(t3l[0] * 1000, 2),
                   tail_brow_tangent_x_mm=round(xt * 1000, 2))
        xbrow = xt
    pts.append(np.array([[xbrow, zn], [-1.0, zn]])); prims.append(('seg', np.array([-1.0, zn]), np.array([xbrow, zn])))
    line = np.concatenate(pts)
    core = Polygon(np.r_[line, [[-1.0, -1.0], [1.0, -1.0]]]).buffer(0); shapely.prepare(core)
    return dict(prims=prims, line=line, core=core, report=rep)


def sdist(PL, x, z):
    """Exact distance to the part line (min over the primitives), signed: > 0 on the cover side (outside the core region)."""
    x = np.asarray(x, float); z = np.asarray(z, float); d = np.full(x.shape, np.inf)
    for pr in PL['prims']:
        if pr[0] == 'seg':
            a, b = pr[1], pr[2]; ab = b - a; t = np.clip(((x - a[0]) * ab[0] + (z - a[1]) * ab[1]) / (ab @ ab), 0, 1)
            d = np.minimum(d, np.hypot(x - a[0] - t * ab[0], z - a[1] - t * ab[1]))
        else:
            c, r, a0, a1 = pr[1], pr[2], pr[3], pr[4]; th = np.arctan2(z - c[1], x - c[0])
            inr = (np.mod(th - a0, 2 * np.pi) <= np.mod(a1 - a0, 2 * np.pi) + 1e-12)
            dr = np.abs(np.hypot(x - c[0], z - c[1]) - r)
            pa = c + r * np.array([np.cos(a0), np.sin(a0)]); pb = c + r * np.array([np.cos(a1), np.sin(a1)])
            de = np.minimum(np.hypot(x - pa[0], z - pa[1]), np.hypot(x - pb[0], z - pb[1]))
            d = np.minimum(d, np.where(inr, dr, de))
    inside = shapely.contains_xy(PL['core'], x, z)
    return np.where(inside, -d, d)


def x_line(PL, z):
    """Tail rise in the side view: x of the part line at height z (rear half, between base and brow); for the draft check."""
    L = PL['line']; m = (L[:, 0] < -0.1) & (L[:, 0] > -0.5) & (L[:, 1] > 0.1231) & (L[:, 1] < 0.1899)
    seg = L[m]; o = np.argsort(seg[:, 1]); return np.interp(z, seg[o, 1], seg[o, 0])
