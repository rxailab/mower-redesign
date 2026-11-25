# [fix3-ports] Mirror-symmetric charger windows inside the front service slot (v18 f3).
#
# Measured inputs (see CHANGES.txt section 6):
#  * dock pins (work/asm_dock.npz, lab 1, docked pose of design_render.py: dock lowered by DOCK_LIFT 17.3): two round brass pins,
#    d 7.9-7.94 (scan) = d 8 nominal, 57 mm free length + a 15 x 18.6 collar at the dock, domed tips at x 290.5, axes horizontal within
#    0.6-0.8 deg; axis fits (mm, valid x 296..346): pin +y  y = 22.74 - 0.0043 (x - 298.5), z = 152.49 + 0.0139 (x - 298.5)
#                                                   pin -y  y = -30.47 - 0.0107 (x - 298.5), z = 152.83 + 0.0099 (x - 298.5).
#    Approach: the mower drives +x onto the plate; relative to the mower the pins travel -x (all four wheels are already on the plate
#    for the last 100+ mm, so z is set by the plate, not by the lawn).
#  * chassis contact pockets (dev/base_parts.npz Chassis, the fixed part): two V funnels in plan (flanks 50-56 deg to x), apex x 284,
#    pocket floor z 143.5, ceiling z 167.6, mouth at the chassis face (x 302): y 8.7..52.0 and -45.4..-4.6 (z 152.6); pocket centres
#    y +28.8 / -24.9 -> the contacts are 53.65 apart (pins 53.2-53.5) and centred on y = +1.95, NOT on -4.3 (that was the old shell scan).
#    asm_dock places the pins 5.8 mm to -y of the pockets (it was aligned to the old shell); the funnels pull the mower so that the pins
#    seat in the pockets: seated pose = asm_dock + seat_dy (+5.8 mm).
#  * lateral capture of the funnels (pin d 8 must enter the mouth): delta in [-15.8, +16.1] mm about the seated pose (both pins move
#    together; limited by the chassis rib between the pockets on both sides).
# Design: each window must pass every position of BOTH pins (one directly, the other mirrored) over the misalignment box
# (capture x dz) + margin, so the two windows are exact mirror images although the pins are not; where a window comes within `snap`
# of the slot tunnel it runs out to it (no hair-thin web), and every convex window corner is filleted. Behind each window a duct
# (prismatic along x, 1 deg draft = the slider pull) runs back to 1 mm in front of the chassis face, so the slot is closed against the
# body cavity (deck drain water stays outside) and only dark duct walls are seen through the windows.
import numpy as np, shapely
from shapely.geometry import Polygon, MultiPolygon, box
from shapely import affinity
from shapely.geometry.polygon import orient
import lidmech
from meshkit import Bag, stack

DEFAULTS = dict(pins=[dict(x=0.2985, y=0.02274, z=0.15249, dy=-0.0043, dz=0.0139), dict(x=0.2985, y=-0.03047, z=0.15283, dy=-0.0107, dz=0.0099)],
                pin_r=0.004, tip_x=0.2905, seat_dy=0.0058, capture=[-0.0158, 0.0161], dz=[-0.0035, 0.0035], margin=0.0005, snap=0.0012,
                fillet=0.0015, web_t=0.0015, wall_t=0.0015, duct_clear=0.001, draft_deg=1.0, bev=0.0004, step=0.0005, hood_lift=0.0025, x_end_est=0.3045, dock_dy=0.0)

def params(Q):
    q = dict(DEFAULTS); q.update(Q or {}); return q

def mirror(g): return affinity.scale(g, -1.0, 1.0, origin=(0.0, 0.0))

def pin_axis(p, x, dy=0.0):
    """(y, z) of a pin axis at mower x (seated / asm frame + dy)."""
    x = np.asarray(x, float); return p['y'] + p['dy'] * (x - p['x']) + dy, p['z'] + p['dz'] * (x - p['x'])

def envelope(Q, x_plane, extra=0.0):
    """Region (y, z) on the +y side that the pins can occupy in the plane x = x_plane during the docking stroke (pin section from the
    seated tip up to that plane), over lateral capture x vertical misalignment, grown by pin_r + margin + extra; the -y pin mirrored."""
    r = Q['pin_r'] + Q['margin'] + extra; c0, c1 = Q['capture']; z0, z1 = Q['dz']; xs = np.linspace(Q['tip_x'] + Q['pin_r'], max(x_plane, Q['tip_x'] + Q['pin_r']) + 1e-6, 9); E = []
    yaw = np.radians(float(Q.get('yaw_deg', 0.0)))      # [fix4-front2] S9: heading error +-yaw: both pins rotated about the seated tip mid-point (vertical axis), then
    if yaw:                                               # [fix4-front2]   driven along x: the plane x_plane sees every rotated axis point that reached it
        xs = np.linspace(Q['tip_x'] + Q['pin_r'], max(x_plane, Q['tip_x'] + Q['pin_r']) / np.cos(yaw) + 0.01, 401)      # [fix4-front2]
        piv = 0.5 * sum(pin_axis(p_, Q['tip_x'], Q['seat_dy'])[0] for p_ in Q['pins'])      # [fix4-front2]
    for p in Q['pins']:
        y_, z_ = pin_axis(p, xs, Q['seat_dy'])
        if yaw:      # [fix4-front2]
            Y_, Z_ = [], []
            for th in (-yaw, 0.0, yaw):      # [fix4-front2] 'heading': rotated pins driven along x -> every rotated point that reached the plane;
                xr = Q['tip_x'] + (xs - Q['tip_x']) * np.cos(th) - (y_ - piv) * np.sin(th); yr = piv + (xs - Q['tip_x']) * np.sin(th) + (y_ - piv) * np.cos(th); k_ = xr <= x_plane + 1e-9
                yc = piv + (y_ - piv) / np.cos(th) + (x_plane - Q['tip_x']) * np.tan(th)      # [fix4-front2] 'crab' (review S9): the pins travel along the rotated x
                Y_ += [yr[k_], yc[k_], np.interp([x_plane], xr, yr)]; Z_ += [z_[k_], z_[k_], np.interp([x_plane], xr, z_)]
            y_, z_ = np.concatenate(Y_), np.concatenate(Z_)
        e_ = box(y_.min() + c0, z_.min() + z0, y_.max() + c1, z_.max() + z1).buffer(r, quad_segs=32)
        E.append(e_ if p['y'] > 0 else mirror(e_))
    return shapely.union_all(E).convex_hull

def windows(Q, T, x_plane, extra=0.0):
    """T: tunnel inner outline at the web plane (y, z). Returns the +y window, the -y window (its exact mirror), the requirement."""
    E = envelope(Q, x_plane, extra); Ts = T.intersection(mirror(T))                               # exactly symmetric tunnel for the construction
    if Q.get('ts_simplify'): Ts = Ts.simplify(Q['ts_simplify'])      # [fix4-front2] G5: T and its mirror cross many times along the top (asymmetry <= 0.045): the window top inherited
                                                                    # [fix4-front2] 0.000..0.05 mm zigzag segments and the web's R0.4 bevel round them self-intersected
    R_ = Q['pin_r'] + Q['margin'] + extra; b_ = E.bounds                                           # the hull of the two pins' rounded boxes has a
    E = box(b_[0] + R_, b_[1] + R_, b_[2] - R_, b_[3] - R_).buffer(R_, quad_segs=32).union(E)       # slanted edge (their z differ 0.3): squared off
    Es = box(*E.bounds) if Q.get('square') else E      # [fix4-front2] A4: squared envelope, so the fillet below gives all four window corners the same radius
    y0, z0, y1, z1 = E.bounds; t0, t1 = Ts.bounds[1], Ts.bounds[3]; ext = [Es]                  # a window side closer than snap to the tunnel  [fix4-front2] Es
    if t1 - z1 < Q['snap']: ext.append(box(y0, 0.5 * (z0 + z1), y1, t1 + 0.01))                # runs straight out to it (no hair-thin web)
    if z0 - t0 < Q['snap']: ext.append(box(y0, t0 - 0.01, y1, 0.5 * (z0 + z1)))
    W = shapely.union_all(ext).intersection(Ts)
    W = W.buffer(-Q['fillet'], quad_segs=16).buffer(Q['fillet'], quad_segs=16).intersection(Ts).union(E)      # convex corners filleted (window -> tunnel); clipped, E re-added (arc discretisation)
    if isinstance(W, MultiPolygon): W = max(W.geoms, key=lambda g: g.area)
    W = orient(Polygon(W.exterior), 1.0)
    if not (W.buffer(1e-7).contains(E) and Ts.buffer(1e-7).contains(W) and Ts.buffer(1e-7).contains(E)):
        np.savez('portskit_fail.npz', T=np.asarray(T.exterior.coords), E=np.asarray(E.exterior.coords), W=np.asarray(W.exterior.coords))
        raise AssertionError('pin envelope %s does not fit the slot tunnel %s at the web (mm; outside: %.4f mm2)' % (np.round(np.array(E.bounds) * 1e3, 2).tolist(),
                             np.round(np.array(Ts.bounds) * 1e3, 2).tolist(), E.difference(Ts).area * 1e6))
    return W, orient(mirror(W), 1.0), E

def _outward(P):
    """Outward unit normals (mitred) of a closed CCW polyline (y, z)."""
    return -lidmech.lnormals(P)

def _ring(a, b, c, d):
    """Closed tube between four rings (n, 3) each: inner front -> inner back -> outer back -> outer front -> inner front."""
    R = np.array([a, b, c, d, a]); V, F = stack(R, closed=True); n = len(a)
    V = V[:4 * n]; F = F % (4 * n); return V, F

def orient_closed(V, F):
    """Make the winding of a closed mesh consistent (breadth-first over shared edges; lidmech.extrude's caps can come out reversed
    against its walls) and flip it so that its signed volume is positive (normals outward)."""
    F = np.asarray(F, np.int64).copy(); n = len(V); E = {}
    for fi, f in enumerate(F):
        for k in range(3): E.setdefault((min(f[k], f[(k + 1) % 3]), max(f[k], f[(k + 1) % 3])), []).append(fi)
    seen = np.zeros(len(F), bool)
    for f0 in range(len(F)):
        if seen[f0]: continue
        seen[f0] = True; st = [f0]
        while st:
            fi = st.pop(); f = F[fi]
            for k in range(3):
                a, b = f[k], f[(k + 1) % 3]
                for fj in E[(min(a, b), max(a, b))]:
                    if seen[fj]: continue
                    g = F[fj]; same = any(g[m] == a and g[(m + 1) % 3] == b for m in range(3))      # same direction -> reversed neighbour
                    if same: F[fj] = g[::-1]
                    seen[fj] = True; st.append(fj)
    P = V[F]; vol = np.einsum('ij,ij->i', P[:, 0], np.cross(P[:, 1], P[:, 2])).sum() / 6.0
    return (F[:, ::-1].copy() if vol < 0 else F), abs(vol)

def mirrored(V, F): return V * np.array([1.0, -1.0, 1.0]), F[:, ::-1].copy()

def chassis_face_x(Xc, foot, x_lo=0.285, x_hi=0.33):
    """Largest x of the chassis sample points Xc whose (y, z) lie in the footprint polygon."""
    m = (Xc[:, 0] > x_lo) & (Xc[:, 0] < x_hi); X = Xc[m]; inn = shapely.contains_xy(foot, X[:, 1], X[:, 2])
    return float(X[inn, 0].max()) if inn.any() else x_lo

def chassis_samples(Cv, Cf, lo=(0.27, -0.08, 0.12), hi=(0.33, 0.08, 0.18), step=0.0003, seed=3):
    c = Cv[Cf].mean(1); m = np.all((c > np.array(lo) - 0.01) & (c < np.array(hi) + 0.01), axis=1); P3 = Cv[Cf[m]]
    ar = 0.5 * np.linalg.norm(np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]), axis=1); ns = np.clip((ar / step ** 2).astype(int) + 1, 1, 4000)
    fi = np.repeat(np.arange(len(P3)), ns); u = np.random.default_rng(seed).random((len(fi), 2)); f_ = u.sum(1) > 1; u[f_] = 1 - u[f_]
    return np.r_[Cv, P3[fi, 0] + (P3[fi, 1] - P3[fi, 0]) * u[:, :1] + (P3[fi, 2] - P3[fi, 0]) * u[:, 1:]]

def lip_crest(p0, p_out, r):      # [fix4-front2] M6 / A4
    """Crest of the slot's rolled lip per rim point. The hole runs through the raked wall along its normal ne = (p_out - p0) / |..|, so on the
    lower half the perimeter falls 0.36 mm from the A-surface edge to p0: a trough behind the lip = an undercut for the front slider (+x) and a
    water trap. The crest is the highest point (along m, the outline's inward normal = into the hole) of the R r roll between the face and the
    perimeter; returns (crest points, lift along m >= 0)."""
    yz = p0[:, 1:]; m = lidmech.lnormals(yz)      # CCW polyline -> inward normals (into the slot)
    m3 = np.c_[np.zeros(len(m)), m]; ne = p_out - p0; ne /= np.linalg.norm(ne, axis=1, keepdims=True)
    f = m3 - (m3 * ne).sum(1)[:, None] * ne; f /= np.maximum(np.linalg.norm(f, axis=1, keepdims=True), 1e-12)
    C = p_out - r * ne - r * f; cr = C + r * m3; up = (m3 * ne).sum(1) > 0      # the arc (from ne to f) contains m only where the perimeter falls inward
    lift = np.where(up, ((cr - p0) * m3).sum(1), 0.0); cr = np.where(up[:, None], cr, p0); return cr, np.maximum(lift, 0.0), m3


def build(Qin, p0, x_w, Xc=None, log=print, p_out=None):
    """Port frame behind the slot: tunnel (from the slot rim back through the web), web plate with the two windows, two ducts to the
    chassis. p0: rim edge points of the slot loop (inside the body wall, as W_.edge_point gives them). Returns V, F, lab, report."""
    Q = params(Qin); tw, ts, td = Q['web_t'], Q['wall_t'], Q['wall_t']; tg = np.tan(np.radians(Q['draft_deg'])); B = Bag(); rep = {}
    p0 = np.asarray(p0, float); yz = p0[:, 1:]; np.savez('portskit_last.npz', p0=p0, x_w=x_w)       # inputs kept for portcheck.py / standalone runs
    if Polygon(yz).exterior.is_ccw is False:
        p0 = p0[::-1]; yz = p0[:, 1:]; p_out = p_out[::-1] if p_out is not None else None      # [fix4-front2] p_out
    k_ = np.linalg.norm(p0 - np.roll(p0, 1, 0), axis=1) > 2e-5      # [fix4-front2] G5 / M6: rim points closer than 0.02 to their predecessor dropped (the full-resolution
    if not k_.all():      # [fix4-front2] loop has coincident pairs: zero-width sliver quads in the tunnel ring with random normals = 'trapped' faces in toolcheck)
        p0 = p0[k_]; yz = p0[:, 1:]; p_out = p_out[k_] if p_out is not None else None; rep['rim_points_dropped'] = int((~k_).sum())      # [fix4-front2]
    xb = x_w - tw                                                                               # back face of the web
    if p_out is not None and Q.get('lip_fill'):      # [fix4-front2] M6 / A4: the bore starts AT the crest of the bottom lip (floor level with it, rising inward
        cr_, lift_, m3_ = lip_crest(p0, np.asarray(p_out, float), Q['lip_fill'])      # [fix4-front2] by the draft): no trough, no slider undercut; the tunnel
        w_ = np.clip(lift_ / 0.0003, 0.0, 1.0); sel_ = lift_ > 0; dsp_ = m3_ * lift_[:, None]; dsp_[:, 0] = w_ * (cr_[:, 0] - 0.00005 - p0[:, 0])      # [fix4-front2] wall fills the old trough.
        for _ in range(3): dsp_ = 0.25 * np.roll(dsp_, 1, 0) + 0.5 * dsp_ + 0.25 * np.roll(dsp_, -1, 0)      # [fix4-front2] smoothed along the loop (rim-loop jitter)
        p0 = p0 + dsp_; yz = p0[:, 1:]      # [fix4-front2] bore lifted along m by the lift; its front end moves out to the crest gradually (no fold where the lift fades)
        rep['lip_fill_mm'] = dict(max_lift=round(float(lift_.max()) * 1e3, 3), points=int(sel_.sum()), floor_front_z=round(float(p0[sel_, 2].min()) * 1e3, 2) if sel_.any() else None)      # [fix4-front2]
    if Q.get('rim_min_step'):      # [v21] f9: on the more raked face the (lifted) rim loop bunches into 0.1-0.4 mm zigzags at the lower end rounds -> crossing
        #                            tunnel faces; keep only rim points at least rim_min_step (end view) from the last kept one
        k_ = [0]
        for i_ in range(1, len(p0)):
            if np.linalg.norm(yz[i_] - yz[k_[-1]]) >= Q['rim_min_step'] and np.linalg.norm(yz[i_] - yz[0]) >= 0.5 * Q['rim_min_step']: k_.append(i_)
        rep['rim_min_step_dropped'] = len(p0) - len(k_); p0 = p0[k_]; yz = p0[:, 1:]
    # ---- tunnel: rim -> back face of the web, 1 deg draft (opening shrinks going in), wall ts outward, front end buried in the body wall
    nT = _outward(yz); dT = (p0[:, 0] - xb) * tg
    inF = p0.copy(); inB = np.c_[np.full(len(p0), xb), yz - nT * dT[:, None]]
    outF = np.c_[p0[:, 0] + 0.00025, yz + nT * ts]; outB = np.c_[np.full(len(p0), xb), yz + nT * (ts - dT[:, None])]
    v_, f_ = _ring(inF, inB, outB, outF); f_, _ = orient_closed(v_, f_); B.add(v_, f_, 0)
    Tw = Polygon(yz - nT * ((p0[:, 0] - x_w) * tg)[:, None]).buffer(0)                         # tunnel inner outline at the web's front face
    # ---- windows (mirror pair) and the web plate
    Wp, Wm, E = windows(Q, Tw, x_w, (x_w - Q['x_end_est']) * tg)                                # grown by the draft shrink down to the duct end
    web = Tw.buffer(ts - 0.0002, quad_segs=16).difference(Wp.union(Wm))
    for g_ in (web.geoms if isinstance(web, MultiPolygon) else [web]):
        v_, f_ = lidmech.extrude(g_, 0.0, tw, (np.array([x_w, 0.0, 0.0]), (0, 1.0, 0), (0, 0, 1.0), (-1.0, 0, 0)), bev=Q['bev'], nb=4, step=Q['step'], ends=(True, False))
        f_, _ = orient_closed(v_, f_); B.add(v_, f_, 0)
    # ---- ducts: window outline straight back (draft) to duct_clear in front of the chassis. The chassis carries a ledge under the
    #      pocket mouths (x up to 307.4, top z 140.5..142.4, tilted 1.3 deg): the full ring stops duct_clear in front of it, the top and
    #      the sides (a hood, above zb + hood_lift) run on to duct_clear in front of the chassis face. One end plane for both ducts (the
    #      deeper of the two), so the pair stays mirror-symmetric.
    zb = float(Wp.bounds[1]); top = box(-1, zb + Q['hood_lift'], 1, 1)
    if Xc is not None:
        xc = max(chassis_face_x(Xc, W.buffer(td + Q['duct_clear'])) for W in (Wp, Wm)) + Q['duct_clear'] + 0.00005
        xe = max(chassis_face_x(Xc, W.buffer(td).difference(W).intersection(top).buffer(Q['duct_clear'])) for W in (Wp, Wm)) + Q['duct_clear'] + 0.00005
    else: xc = xe = Q.get('x_end', 0.3085)
    xe = min(xe, xc); xe_ = [xc, xe]
    SC = Q.get('scarf') if xc - xe > 0.0005 else None      # [fix4-front2] A4: [z0, z1] above the window bottom: ONE ring per duct whose end is scarfed - x_end = xc
    for W in (Wp,):                                                                             # the -y duct is the exact mirror image
        P = lidmech.resample(W, Q['step']); n_ = _outward(P); xs_ = xb + 0.0002; d_ = (xs_ - xc) * tg
        if SC:      # [fix4-front2] (over the chassis ledge) up to zb + z0, xe (1 mm before the chassis face) from zb + z1 up, a plane between: the floor and the R3 lower
            xend_ = lambda z_: np.interp(z_, [zb + SC[0], zb + SC[1]], [xc, xe])      # [fix4-front2] corners end along one slanted line (no step where the hood began)
            Po_ = P + n_ * td; xi_, xo_ = xend_(P[:, 1]), xend_(Po_[:, 1]); di_, do_ = (xs_ - xi_) * tg, (xs_ - xo_) * tg      # [fix4-front2] per-vertex end x + draft
            iF = np.c_[np.full(len(P), xs_), P]; iB = np.c_[xi_, P - n_ * di_[:, None]]; oB = np.c_[xo_, P + n_ * (td - do_)[:, None]]; oF = np.c_[np.full(len(P), xs_), Po_]
            v_, f_ = _ring(iF, iB, oB, oF); f_, _ = orient_closed(v_, f_); B.add(v_, f_, 0); B.add(*mirrored(v_, f_), 0)      # [fix4-front2]
            if Xc is not None:      # [fix4-front2] clearance of the scarfed end (end face samples) to the chassis samples
                from scipy.spatial import cKDTree
                Es_ = np.concatenate([iB + (oB - iB) * t_ for t_ in np.linspace(0, 1, 7)]); Es_ = np.r_[Es_, Es_ * np.array([1.0, -1.0, 1.0])]
                rep['scarf_mm'] = dict(z=[round((zb + s_) * 1e3, 2) for s_ in SC], x=[round(xc * 1e3, 2), round(xe * 1e3, 2)],
                                       end_to_chassis_min=round(float(cKDTree(Xc).query(Es_)[0].min()) * 1e3, 3))      # [fix4-front2]
            continue      # [fix4-front2] no separate hood
        iF = np.c_[np.full(len(P), xs_), P]; iB = np.c_[np.full(len(P), xc), P - n_ * d_]; oB = np.c_[np.full(len(P), xc), P + n_ * (td - d_)]; oF = np.c_[np.full(len(P), xs_), P + n_ * td]
        v_, f_ = _ring(iF, iB, oB, oF); f_, _ = orient_closed(v_, f_); B.add(v_, f_, 0); B.add(*mirrored(v_, f_), 0)
        if xc - xe > 0.0005:                                                                    # hood: top + sides on to the chassis face
            Wi = W.buffer(-d_, quad_segs=16); H = Wi.buffer(td, quad_segs=16).difference(Wi).intersection(top)
            for g_ in (H.geoms if hasattr(H, 'geoms') else [H]):
                v_, f_ = lidmech.extrude(g_, 0.0, xc + 0.0002 - xe, (np.array([xc + 0.0002, 0.0, 0.0]), (0, 1.0, 0), (0, 0, 1.0), (-1.0, 0, 0)), step=Q['step'])
                f_, _ = orient_closed(v_, f_); B.add(v_, f_, 0); B.add(*mirrored(v_, f_), 0)
    V, F, L_ = B.out()
    # ---- report
    zb = float(np.min(np.asarray(Wp.exterior.coords)[:, 1])); yi = float(np.min(np.asarray(Wp.exterior.coords)[:, 0])); yo = float(np.max(np.asarray(Wp.exterior.coords)[:, 0]))
    zt = float(np.max(np.asarray(Wp.exterior.coords)[:, 1])); tz0 = float(Tw.bounds[1]); tz1 = float(Tw.bounds[3])
    sym = float(Wp.symmetric_difference(mirror(Wm)).area)
    rep.update(window_y_mm=[round(yi * 1e3, 2), round(yo * 1e3, 2)], window_z_mm=[round(zb * 1e3, 2), round(zt * 1e3, 2)], window_area_mm2=round(Wp.area * 1e6, 1),
               mullion_mm=round(2 * yi * 1e3, 2), sill_mm=round((zb - tz0) * 1e3, 2), tunnel_z_at_web_mm=[round(tz0 * 1e3, 2), round(tz1 * 1e3, 2)],
               envelope_y_mm=[round(E.bounds[0] * 1e3, 2), round(E.bounds[2] * 1e3, 2)], envelope_z_mm=[round(E.bounds[1] * 1e3, 2), round(E.bounds[3] * 1e3, 2)],
               mirror_xor_mm2=round(sym * 1e6, 4), web_x_mm=[round(xb * 1e3, 2), round(x_w * 1e3, 2)], duct_end_x_mm=[round(x * 1e3, 2) for x in xe_], duct_len_mm=[round((xb + 0.0002 - x) * 1e3, 2) for x in xe_], capture_mm=[round(c * 1e3, 2) for c in Q['capture']], dz_mm=[round(c * 1e3, 2) for c in Q['dz']],
               seat_dy_mm=round(Q['seat_dy'] * 1e3, 2), faces=int(len(F)))
    log('ports3', rep)
    return V, F, L_, rep, dict(Wp=np.asarray(Wp.exterior.coords), Wm=np.asarray(Wm.exterior.coords), Tw=np.asarray(Tw.exterior.coords), x_w=x_w)
