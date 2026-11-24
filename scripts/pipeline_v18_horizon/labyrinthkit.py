# [fix3-labyrinth] Labyrinth at the cover / body part line, built INTO the two closed solids by shell_wall.py (hooks tagged [fix3-labyrinth]).
#
# Section through the joint (n = depth under the cover's outer skin, the cover stands 0.5 proud of the body; u = along the skin away
# from the body edge, i.e. towards the cover across the gap). Default numbers (wall.labyrinth in fix3_labyrinth.json):
#
#        cover outer skin  n 0 ---------------------------+
#        cover LIP 1.55 (rebate on the inside)            |  45 deg taper back to 2.5 at u 4.1 .. 5.1 (3.3 .. 4.3 above the cover edge)
#        rebate face       n 1.55 ......................  |
#        CHANNEL 0.80 (= the visible gap: no capillary suction from the gap into it)
#        upstand outer face n 2.35 (1 deg draft, leans away from the cover going up) ...  top R0.6 at u = 4.0 above the body cut line
#        UPSTAND (tongue) 1.28 at the root, 1.23 (>= 1.2) at the top; its inner face is the body's inner skin, thickened 2.5 -> 3.13 at the edge
#        body ledge (gap floor): R0.5 roll -> flat ledge raised by delta at its inner end (outward drain slope) -> R0.3 root fillet
#
# Water path: the 0.8 gap (straight in) -> a 90 deg turn UP into the 0.8 channel -> a climb of 3.2 mm above the cover's lower edge (4.0 above
# the ledge) -> over the upstand top -> down its inner face. Anything that does not make the climb falls back onto the ledge, which slopes
# OUT (>= ledge_slope_deg where the line is level and the wall allows it) and runs along the channel to the level runs, where it leaves
# through the gap: the channel drains to the outside along its whole length, the low points (brows, base line) included.
#
# wall.labyrinth = { "<loop_r rule name>": rule }       e.g. "cover_seam" (part 1) and "core_seam" (part 0)
#   rule.t_edge    wall thickness at the free edge (body: derived from the upstand when absent)
#   rule.t_len     ... held up to this distance from the ORIGINAL cut line (the rolled edge is trimmed back by its radius from it)
#   rule.t_taper   then a C1 taper back to the nominal wall over this length
#   rule.splits    iso-lines of the edge distance cut into the outer skin across the taper (crisp step whatever the mesh size)
#   rule.r_frac    roll radius cap as a fraction of the edge wall (shell_wall's default 0.42 would shrink R0.8 to 0.65 on a 1.55 lip)
#   rule.upstand   {w_o, h, t_top, draft_deg, fillet, ledge_slope_deg, ledge_dmax, n_fillet, n_top}: the rim profile of that loop becomes
#                  roll -> ledge -> root fillet -> upstand outer face -> round top -> upstand inner face (= the thickened inner skin);
#                  its frame (tangent, normal) is smoothed along the loop by ARC LENGTH (frame_sigma 1.5 mm, smooth_along): the trimmed loop has
#                  vertices microns apart and 1 mrad of frame noise folds a 4.5 mm long profile
# wall.labyrinth.kit_cv_min   design_gen: datum-pin gusset feet stay this far (side-view distance) above the part line (clear of lip and upstand)
# wall.labyrinth.deck_inset   design_gen: {extra, below, above}: the deck edge stops `extra` further in where it meets the thickened edge /
#                             the upstand (side-view distance of the deck edge to the part line), so it stays 0.5 inside the body there too
import numpy as np

SS = lambda x: np.clip(x, 0, 1) ** 2 * (3 - 2 * np.clip(x, 0, 1))
UP_DEF = dict(w_o=0.00185, h=0.004, t_top=0.0012, draft_deg=1.0, fillet=0.0003, ledge_slope_deg=5.0, ledge_dmax=0.0005, n_fillet=3, n_top=8)


def rules(p):
    """Rules by loop name, with the derived body edge thickness (upstand: outer face depth + top thickness + the draft over its height)."""
    out = {}
    for nm, r in (p.get('labyrinth') or {}).items():
        if nm.startswith('//') or nm in ('deck_inset',) or not isinstance(r, dict): continue      # deck_inset: design_gen's deck hook
        r = dict(r)
        if r.get('upstand') is not None:
            u = dict(UP_DEF, **r['upstand']); r['upstand'] = u
            if 't_edge' not in r: r['t_edge'] = u['w_o'] + u['t_top'] + u['h'] * np.tan(np.radians(u['draft_deg'])) + 0.00001
        out[nm] = r
    return out


def relief(X, r):
    """[fix4-cover2] M5: thermal relief of the channel towards the ends of the part line. The light cover is x-located at mid-length (its
    x-locator legs at x0); the dark body grows up to 1.47e-3 more than the cover (sun: body 75 C / cover 50 C), i.e. 0.45 mm at the nose /
    tail, where the joint faces x. upstand.relief = {x0, r: [r0, r1], dw}: the upstand outer face (and with it the body edge, the upstand
    keeps its thickness) moves dw further in, smoothstep over |x - x0| = r0 .. r1. -> extra depth per point (m)"""
    rf = (r.get('upstand') or {}).get('relief') if isinstance(r, dict) else None
    if not rf: return np.zeros(len(X))
    ax = np.abs(np.asarray(X, float)[:, 0] - rf.get('x0', 0.0)); r0, r1 = rf['r']
    return float(rf['dw']) * SS((ax - r0) / max(r1 - r0, 1e-6))


def at_points(r, X):
    """[fix4-cover2] the rule with per-point w_o / t_edge (relief at the loop vertices X), or the rule itself"""
    d = relief(X, r)
    if not d.any(): return r
    r2 = dict(r); u = dict(r2['upstand']); u['w_o_base'] = u['w_o']; u['w_o'] = u['w_o'] + d; r2['upstand'] = u; r2['t_edge'] = r['t_edge'] + d
    return r2


def weight(db, r):
    """1 within t_len of the original cut line, C1 taper to 0 at t_len + t_taper."""
    return 1.0 - SS((np.asarray(db, float) - r.get('t_len', 0.0)) / max(r.get('t_taper', 0.001), 1e-6))


def levels(r):
    k = int(r.get('splits', 4)); return list(r.get('t_len', 0.0) + r.get('t_taper', 0.001) * np.linspace(0, 1, k + 1)) if k > 0 else []


def smooth_along(s, Ltot, X, sigma, step=0.0001):
    """Periodic Gaussian smoothing of per-vertex vectors X (n, 3) along a closed loop as a function of ARC LENGTH s (n,): resampled every
    `step`, convolved (FFT), interpolated back. Vertices microns apart (the trimmed loop has them) get frames microns apart; a window counted
    in vertices (shell_wall._ls) jumps there by ~1 mrad, which a 4.5 mm long rim profile turns into folded strips."""
    m = max(int(np.ceil(Ltot / step)), 8); su = np.arange(m) * (Ltot / m); s2 = np.r_[s, s[0] + Ltot]
    Y = np.stack([np.interp(su, s2, np.r_[X[:, c], X[0, c]]) for c in range(X.shape[1])], 1)
    du = np.minimum(np.arange(m), m - np.arange(m)) * (Ltot / m); g = np.exp(-0.5 * (du / sigma) ** 2); g /= g.sum()
    Ys = np.real(np.fft.ifft(np.fft.fft(Y, axis=0) * np.fft.fft(g)[:, None], axis=0))
    su2 = np.r_[su, Ltot]; return np.stack([np.interp(np.mod(s, Ltot), su2, np.r_[Ys[:, c], Ys[0, c]]) for c in range(X.shape[1])], 1)


def ledge_delta(ne, b, r, roll):
    """Raise (along b) of the ledge's inner end so the ledge runs DOWN to the outside by ledge_slope_deg in 3D where the part line is level
    (b points up). On a tumblehome wall the plain edge face (perpendicular to the skin) would slope INTO the joint by the lean angle."""
    u = r['upstand']; Lw = np.maximum(np.asarray(u['w_o'], float) - u['fillet'] - roll, 1e-5); nz = ne[:, 2]; bz = b[:, 2]      # [fix4-cover2] w_o may be per point
    d = Lw * (nz + np.tan(np.radians(u['ledge_slope_deg']))) / np.maximum(bz, 0.3)
    dmx = u['ledge_dmax'] * Lw / max(u.get('w_o_base', np.min(u['w_o'])) - u['fillet'] - roll, 1e-5)      # [fix4-cover2] M5: a ledge lengthened by the relief keeps its slope
    d = np.clip(d, 0.0, dmx) * SS((bz - 0.15) / 0.25)                  # steep line (b ~ horizontal): the water runs along the line  [fix4-cover2] dmx
    return d, Lw


def profile(n, RLo, thl, r_, rule, NR, d_, Lw):
    """(u, w) rings of the rim with an upstand (each entry: arrays of n, or scalars). u along b from the trimmed outer edge (the original cut
    line is at u = RLo), w along the outward normal (0 = outer skin, -thl = inner skin). The last ring lies in the inner-skin plane; the rim's
    closing strip runs from it to the inner-skin edge vertex at (0, -thl)."""
    u = rule['upstand']; f = u['fillet']; wo = u['w_o']; td = np.tan(np.radians(u['draft_deg'])); thl = np.broadcast_to(np.asarray(thl, float), (n,))
    prof = []
    for k in range(1, NR + 1):                                                         # outer roll (unchanged: the visible edge)
        ph = 0.5 * np.pi * k / NR; prof.append((RLo * np.sin(ph), -r_ * (1 - np.cos(ph))))
    prof.append((RLo + d_, np.broadcast_to(-(wo - f), (n,))))                        # ledge, its inner end raised by delta (drain slope)  [fix4-cover2] wo per point
    cu = RLo + d_ + f; cw = -(wo - f)                                                  # root fillet (concave, R f), ledge -> outer face
    for k in range(1, int(u['n_fillet']) + 1):
        ps = 0.5 * np.pi * k / int(u['n_fillet']); prof.append((cu - f * np.cos(ps), np.broadcast_to(cw - f * np.sin(ps), (n,))))      # [fix4-cover2]
    u0 = cu                                                                            # outer face from u0 (depth wo) to ua (depth w1)
    w2 = thl; ua = RLo + u['h'] - 0.5 * (w2 - wo)
    for _ in range(2):                                                                 # top half-round of radius rt = (w2 - w1) / 2 ends at u = RLo + h
        w1 = wo + (ua - u0) * td; rt = 0.5 * (w2 - w1); ua = RLo + u['h'] - rt
    w1 = wo + (ua - u0) * td; rt = 0.5 * (w2 - w1); wc = -0.5 * (w1 + w2)
    prof.append((0.5 * (u0 + ua), -(wo + 0.5 * (ua - u0) * td)))
    prof.append((ua, -w1))
    for k in range(1, int(u['n_top']) + 1):
        ph = np.pi * k / int(u['n_top']); prof.append((ua + rt * np.sin(ph), wc + rt * np.cos(ph)))
    prof.append((0.5 * (ua + RLo), -thl))                                              # inner face = the thickened inner skin, back to the edge;
    prof.append((np.full(n, max(RLo - 0.0003, 0.5 * RLo)), -thl))                     # the closing strip in the inner-skin plane stays 0.2 long (as shell_wall's)
    rep = dict(t_root_mm=round(float(np.median(w2 - wo)) * 1000, 3), t_top_mm=round(float(np.min(w2 - w1)) * 1000, 3),
                        top_above_cut_mm=round(u['h'] * 1000, 2), outer_face_depth_mm=round(float(np.min(wo)) * 1000, 2), edge_wall_mm=round(float(np.median(w2)) * 1000, 3))      # [fix4-cover2] min(wo)
    if np.ndim(wo): rep.update(outer_face_depth_max_mm=round(float(np.max(wo)) * 1000, 3), edge_wall_max_mm=round(float(np.max(w2)) * 1000, 3))      # [fix4-cover2] M5 relief
    return prof, rep


def ledge_report(ne, b, d_, Lw):
    """3-D slope of the ledge going OUT (deg, > 0 = drains outward) where the part line is level-ish (bz > 0.7)."""
    s = np.degrees(np.arctan2(d_ * b[:, 2] - Lw * ne[:, 2], Lw)); m = b[:, 2] > 0.7
    return dict(level_runs_n=int(m.sum()), drain_slope_deg_min=round(float(s[m].min()), 2) if m.any() else None,
                drain_slope_deg_p5=round(float(np.percentile(s[m], 5)), 2) if m.any() else None, drain_slope_deg_median=round(float(np.median(s[m])), 2) if m.any() else None,
                delta_max_mm=round(float(d_.max()) * 1000, 3), ledge_len_mm=round(float(np.min(Lw)) * 1000, 3))      # [fix4-cover2] Lw per point


class RobustSurfDist:
    """[fix3-labyrinth] Signed distance to a CLOSED mesh for the checks (labcheck.py): the distance is exact among the k faces with the nearest
    centroids (unsigned), the sign comes from the parity of a vertical ray (+z) against all faces of the solid (plan buckets of 1 mm), evaluated
    only for points closer than `sign_within` (every point inside a <= 3.2 mm wall is). shell_wall.SurfDist signs by the nearest face's plane,
    which is wrong near the edges of large faces (deck cells, coarse skin away from the joint): it reports deep false 'inside' points there."""
    def __init__(s, V, F, k=16, cell=0.001, sign_within=0.0018):
        from shell_wall import SurfDist
        b = SurfDist(V, F, k); s.P, s.n, s.tr, s.k = b.P, b.n, b.tr, k; s.cell = cell; s.sw = sign_within; P = s.P
        a2 = (P[:, 1, 0] - P[:, 0, 0]) * (P[:, 2, 1] - P[:, 0, 1]) - (P[:, 2, 0] - P[:, 0, 0]) * (P[:, 1, 1] - P[:, 0, 1]); ok = np.abs(a2) > 1e-16
        s.fi = np.nonzero(ok)[0]; lo = np.floor(P[ok][:, :, :2].min(1) / cell).astype(np.int64); hi = np.floor(P[ok][:, :, :2].max(1) / cell).astype(np.int64)
        nx = hi[:, 0] - lo[:, 0] + 1; ny = hi[:, 1] - lo[:, 1] + 1; n = nx * ny; fr = np.repeat(np.arange(len(lo)), n); o = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        cx = lo[fr, 0] + o % nx[fr]; cy = lo[fr, 1] + o // nx[fr]; key = cx * 1000003 + cy; srt = np.argsort(key, kind='stable'); s.key = key[srt]; s.fcell = s.fi[fr[srt]]
    def inside(s, X):
        X = X + np.array([3.1e-9, 1.7e-9, 0.0]); c = np.floor(X[:, :2] / s.cell).astype(np.int64); key = c[:, 0] * 1000003 + c[:, 1]
        a = np.searchsorted(s.key, key, 'left'); b = np.searchsorted(s.key, key, 'right'); n = b - a; qi = np.repeat(np.arange(len(X)), n)
        fj = s.fcell[np.repeat(a, n) + (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n))]; P = s.P[fj]; x = X[qi]
        e1 = P[:, 1, :2] - P[:, 0, :2]; e2 = P[:, 2, :2] - P[:, 0, :2]; den = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]; dp = x[:, :2] - P[:, 0, :2]
        u = (dp[:, 0] * e2[:, 1] - dp[:, 1] * e2[:, 0]) / den; v = (e1[:, 0] * dp[:, 1] - e1[:, 1] * dp[:, 0]) / den
        z = P[:, 0, 2] + u * (P[:, 1, 2] - P[:, 0, 2]) + v * (P[:, 2, 2] - P[:, 0, 2]); hit = (u >= 0) & (v >= 0) & (u + v <= 1) & (z > x[:, 2])
        return np.bincount(qi[hit], minlength=len(X)) % 2 == 1
    def __call__(s, X, upper=np.inf):
        from shell_wall import _pt_tri
        out = np.full(len(X), np.inf); fid = np.full(len(X), -1); k = s.k
        for s0 in range(0, len(X), 50000):
            x = X[s0:s0 + 50000]; d_, i_ = s.tr.query(x, k=k, distance_upper_bound=upper, workers=-1); ok = np.isfinite(d_); i_ = np.where(ok, i_, 0)
            P = s.P[i_.ravel()]; xx = np.repeat(x, k, 0); q = _pt_tri(xx, P[:, 0], P[:, 1], P[:, 2]); dd = np.where(ok, np.linalg.norm(xx - q, axis=1).reshape(-1, k), np.inf)
            dm = dd.min(1); j = np.argmin(dd, 1); sg = np.ones(len(x)); near = np.isfinite(dm) & (dm < s.sw)
            if near.any(): sg[near] = np.where(s.inside(x[near]), -1.0, 1.0)
            out[s0:s0 + 50000] = np.where(np.isfinite(dm), sg * dm, np.inf); fid[s0:s0 + 50000] = np.where(np.isfinite(dm), i_[np.arange(len(x)), j], -1)
        return out, fid
