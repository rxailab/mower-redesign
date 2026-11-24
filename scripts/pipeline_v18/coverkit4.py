# [fix4-cover2] Cover kit / deck fixes of fix round 4 (called from design_gen.py's kit2 code, hooks tagged [fix4-cover2]).
#
# G1  snap legs over the kept chassis: fix-struts keeps the rim flange and leaves the old mudguard pin as a stub capped at z 218.5; the two
#     legs found between posts 8 / 10 and 9 / 7 (x ~118) stood on it (leg tip 216.6, up to 1.9 mm into the chassis). cover.legs.move moves a
#     leg whose default site lies in a plan box (mirrored in y) along the cover edge to `to_x`, where the rim flange has fallen away
#     (it drops 2.7 mm per 10 mm going forward, the deck only 1.6). The deck there slopes along the edge (~9 deg), so the window edge the hook
#     catches is not level: cover.legs.follow_deck shears the flexing part of the leg and the hook (z += slope * u, u along the edge) so the
#     45 deg retention face runs parallel to the deck underside over the whole 8 mm width (f3: level hook, 0.17-0.19 mm into the deck at the
#     downhill end of the x ~118 windows, a gap at the uphill end).
# G4  the deck solid self-intersected (f3: 4 053 pairs full / 943 web) for three reasons: (1) its underside was the top moved 2.5 mm along
#     the vertex normals, which folds (swallowtail) where the top bends tighter than 2.5 mm (plate -> skirt down the cover's inner flank);
#     (2) the 1.0 mm push from the cover solid (design_gen, after the wall) moved single top / underside vertices by up to 1.7 mm (the
#     skirt outline lay ON the cover's inner skin beside the wheels); (3) Delaunay triangles of three collinear outline points (dropped:
#     T-junction, or kept: a vertical sliver) that the edge wall crossed. cover.deck_envelope: build_deck() = the same outline / grid,
#     tri_plan() (no collinear outline triple), underside = deck_envelope(): the exact lower offset (lower envelope of the balls of radius t
#     on the top, swallowtails trimmed) straight under each top vertex, edge / hole walls vertical; top and underside share one plan
#     triangulation, so the plate cannot fold. cover.deck_clear: the clearance to the cover solid is made by cutting the plan OUTLINE back
#     where a deck point is closer than c (deck_clear(), rebuilt + re-measured), and the push skips the deck (cover.deck_push false).
# G5  orient(): every closed component of the cover kit / body mount kit is wound consistently and outward (positive volume).
import numpy as np

REP = {'leg_moves': [], 'legs_follow_deck': [], 'deck_offset': {}, 'oriented': {}}


# ---------------------------------------------------------------------------------------------------------------- G1 legs
def leg_moves(LG):
    """cover.legs.move: [{x: [x0, x1], ay: [|y|0, |y|1], to_x}] (metres)"""
    return [m for m in ((LG or {}).get('move') or []) if isinstance(m, dict) and 'to_x' in m]


def moved_site(rules, O, i, ninb):
    """(index of the edge point the leg moves to (same side, nearest x to to_x, within the rule's |y| band), rule) or (None, None)"""
    p = O[i]
    for m in rules:
        if m['x'][0] <= p[0] <= m['x'][1] and m['ay'][0] <= abs(p[1]) <= m['ay'][1]:
            band = m.get('ay_to', m['ay'])
            c = np.nonzero((np.sign(O[:, 1]) == np.sign(p[1])) & (np.abs(O[:, 1]) >= band[0]) & (np.abs(O[:, 1]) <= band[1]))[0]
            if len(c): return int(c[np.argmin(np.abs(O[c, 0] - m['to_x']))]), m
    return None, None


def locator(rule, c, tg, w, wh):
    """M5: a leg whose rule gives window_half is an x-locator (its window fits it along the edge); recorded for the thermal datum"""
    if rule and rule.get('window_half'):
        REP.setdefault('x_locators', []).append(dict(at_mm=np.round(np.asarray(c[:2]) * 1000, 2).tolist(), window_mm=round(2 * wh * 1000, 2),
                                                     play_per_side_mm=round((wh - w / 2) * 1000, 3), tangent=np.round(np.asarray(tg[:2]), 3).tolist()))


def follow_deck(LG, c, tg, zraw, w, z_hi, z_lo):
    """z-shear of the leg / hook vertices: full below z_lo (flexing part + hook), fading to 0 at z_hi (stiff stem, root unchanged).
    slope = deck height change per metre along the edge tangent tg at the leg centre c (measured over the leg width w)."""
    if not (LG or {}).get('follow_deck'): return lambda R: R
    t2 = np.asarray(tg[:2], float); h = 0.5 * w
    za = zraw(np.array([c[0] - t2[0] * h, c[0] + t2[0] * h]), np.array([c[1] - t2[1] * h, c[1] + t2[1] * h]))
    s = float(za[1] - za[0]) / (2 * h)
    REP['legs_follow_deck'].append(dict(at_mm=np.round(np.asarray(c[:2]) * 1000, 1).tolist(), deck_slope_along_deg=round(float(np.degrees(np.arctan(s))), 2),
                                        hook_end_shift_mm=round(abs(s) * h * 1000, 3)))

    def sh(R):
        R = np.array(R, float); u = (R[:, :2] - np.asarray(c[:2])) @ t2
        wz = np.clip((z_hi - R[:, 2]) / max(z_hi - z_lo, 1e-6), 0.0, 1.0)
        R[:, 2] = R[:, 2] + s * u * wz
        return R
    return sh


# ---------------------------------------------------------------------------------------------------------------- m12 seat stop
def seat_pad(Pk, nh, tk, SF, zseat):
    """m12: 2K TPE bump pad on the underside of the seat flange / wall edge over the drive-housing crest. SF['tpe'] = {x: [x0, x1],
    t (total, 1.0), proud (below the flange underside), out (inboard of the outer skin), r (plan corner radius), bev}. The rest of t sits
    in the flange (2K pocket, drawn as an overlap with the seat flange / wall: one moulding). Plan: between `out` from the outer skin and
    0.5 mm inside the flange's inner edge, over x0..x1. -> (V, F)"""
    import shapely, sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); import lidmech
    tp = SF['tpe']; m = (Pk[:, 0] >= tp['x'][0]) & (Pk[:, 0] <= tp['x'][1])
    if m.sum() < 2: return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    a0 = tp.get('out', 0.001); a1 = (tk[m] + SF.get('w', 0.004) - 0.0005)
    outer = Pk[m, :2] - nh[m, :2] * a0; inner = Pk[m, :2] - nh[m, :2] * a1[:, None]      # nh = outward edge normal (the flange runs along -nh)
    pg = shapely.Polygon(np.r_[outer, inner[::-1]]).buffer(0); r = tp.get('r', 0.0015); pg = pg.buffer(-r, join_style=1).buffer(r, join_style=1)
    z1 = zseat + tp.get('t', 0.001) - tp.get('proud', 0.0005); z0 = zseat - tp.get('proud', 0.0005)
    V, F = lidmech.extrude(pg, z0, z1, (np.zeros(3), (1.0, 0, 0), (0, 1.0, 0), (0, 0, 1.0)), bev=tp.get('bev', 0.0003), nb=2, ends=(True, False))
    REP.setdefault('seat_tpe', []).append(dict(x_mm=[round(tp['x'][0] * 1000, 1), round(tp['x'][1] * 1000, 1)], area_mm2=round(pg.area * 1e6, 1),
                                               bottom_z_mm=round(z0 * 1000, 2), proud_mm=round(tp.get('proud', 0.0005) * 1000, 2), thick_mm=round(tp.get('t', 0.001) * 1000, 2)))
    return V, F


_CH = {}
def post_top(x, y, r, z0, dz=0.004):
    """highest scanned chassis point (dev/base_parts) within r of (x, y) and z0 +- dz, or z0 when that is lower"""
    import os
    if 'v' not in _CH: _CH['v'] = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dev', 'base_parts.npz'))['Chassis_v'].astype(float)
    X = _CH['v']; m = (np.hypot(X[:, 0] - x, X[:, 1] - y) < r) & (np.abs(X[:, 2] - z0) < dz)
    zt = float(X[m, 2].max()) if m.any() else z0
    REP.setdefault('pad_on_scan', []).append(dict(at_mm=[round(x * 1000, 1), round(y * 1000, 1)], json_mm=round(z0 * 1000, 2), scan_mm=round(zt * 1000, 2)))
    return max(z0, zt)


# ---------------------------------------------------------------------------------------------------------------- G4 deck underside
def deck_envelope(T3, tri, t, step=0.0003, tag='deck'):
    """underside of the deck plate as the exact lower offset of its top: zb(p) = min over top-surface points q of h(q) - sqrt(t^2 - |q - p|^2)
    (the lower envelope of the balls of radius t on the top surface, i.e. the offset surface with the swallowtails trimmed), evaluated
    straight under every top vertex. Top and underside share their plan triangulation, so the plate cannot fold; side walls (edge, holes) are
    vertical. q = top vertices + barycentric samples of every top triangle (<= `step` apart). Returns B3."""
    from scipy.spatial import cKDTree
    P = T3[tri]; L = np.max(np.linalg.norm(P - np.roll(P, 1, 1), axis=2), 1); kk = np.clip(np.ceil(L / step).astype(int), 1, 12); S = [T3]
    for k in np.unique(kk):
        m = kk == k; a, b = np.meshgrid(np.arange(k + 1), np.arange(k + 1), indexing='ij'); ok = (a + b <= k) & ~(((a == 0) & (b == 0)) | ((a == k) & (b == 0)) | ((a == 0) & (b == k)))
        w = np.c_[a[ok], b[ok], k - a[ok] - b[ok]] / k
        if len(w): S.append(np.einsum('kj,fjc->fkc', w, P[m]).reshape(-1, 3))
    S = np.concatenate(S); tr = cKDTree(S[:, :2]); zb = T3[:, 2] - t
    # memory bound: neighbour pairs are expanded per chunk of <= PAIRS (counted first with return_length), never for all vertices
    PAIRS = 2_000_000; cnt = tr.query_ball_point(T3[:, :2], t * 0.999, workers=-1, return_length=True).astype(np.int64)
    s0 = 0; peak = 0
    while s0 < len(T3):
        cs = np.cumsum(cnt[s0:]); s1 = s0 + max(1, int(np.searchsorted(cs, PAIRS, side='right'))); X = T3[s0:s1]
        nb = tr.query_ball_point(X[:, :2], t * 0.999, workers=-1)
        ln = cnt[s0:s1]; qi = np.repeat(np.arange(len(X)), ln); fi = np.concatenate([np.asarray(q, np.int64) for q in nb]); del nb
        peak = max(peak, len(fi))
        zc = S[fi, 2] - np.sqrt(np.maximum(t * t - ((S[fi, :2] - X[qi, :2]) ** 2).sum(1), 0.0)); del fi
        z = np.full(len(X), np.inf); np.minimum.at(z, qi, zc); zb[s0:s1] = np.minimum(zb[s0:s1], z); del qi, zc; s0 = s1
    dz = (T3[:, 2] - t) - zb
    REP['deck_offset'].setdefault(tag, []).append(dict(mode='envelope', samples=int(len(S)), verts=int(len(T3)), peak_pairs=int(peak), lowered_gt_0p01mm=int((dz > 1e-5).sum()),
                                                       max_extra_depth_mm=round(float(dz.max()) * 1000, 3)))
    return np.c_[T3[:, :2], zb]


def tri_plan(P2, poly, hmin=1e-5, it=10):
    """plan triangulation of one deck part: Delaunay of the outline + grid points, triangles with the centroid inside, all CCW. A triangle
    of three (nearly) collinear outline points (height < hmin over its longest edge) was dropped (f3: area < 1e-14) or kept as a vertical
    sliver; either leaves a T-junction / a vertical facet on the outline that the edge wall crosses. Its middle point (opposite the longest
    edge, < hmin off the line) is removed and the set re-triangulated."""
    from scipy.spatial import Delaunay
    import shapely
    nd = 0
    for _ in range(it):
        tri = Delaunay(P2).simplices; cc = P2[tri].mean(1); tri = tri[shapely.contains_xy(poly, cc[:, 0], cc[:, 1])]
        T = P2[tri]; ar2 = (T[:, 1, 0] - T[:, 0, 0]) * (T[:, 2, 1] - T[:, 0, 1]) - (T[:, 2, 0] - T[:, 0, 0]) * (T[:, 1, 1] - T[:, 0, 1])
        tri = np.where((ar2 < 0)[:, None], tri[:, ::-1], tri); T = P2[tri]
        L = np.linalg.norm(np.roll(T, -1, 1) - T, axis=2)                                # edge k: vertex k -> k + 1
        bad = np.abs(ar2) / np.maximum(L.max(1), 1e-12) < hmin
        if not bad.any(): break
        k = np.argmax(L[bad], 1); mid = np.unique(tri[bad][np.arange(int(bad.sum())), (k + 2) % 3])
        keep = np.ones(len(P2), bool); keep[mid] = False; P2 = P2[keep]; nd += int(len(mid))
    REP['deck_offset'].setdefault('outline_points_dropped_collinear', []).append(nd)
    us = np.unique(tri); mp = -np.ones(len(P2), np.int64); mp[us] = np.arange(len(us))
    return P2[us], mp[tri]


def build_deck(dpg, zraw, CV, t, tag='deck', refine=None):
    """[fix4-cover2] G4: the deck plate (design_gen kit2 loop, same outline / grid / bow-tie rule) with tri_plan + deck_envelope:
    top = zraw on the plan triangulation, underside = exact lower offset straight below, side walls vertical. -> (V, F)"""
    import shapely
    DKV, DKF = [], []; o = 0
    for pp in shapely.get_parts(dpg):
        if pp.area < 2e-5: continue
        pp = shapely.segmentize(pp.simplify(0.0001), CV.get('deck_edge', 0.0007))
        bp = np.concatenate([np.asarray(r.coords)[:-1] for r in [pp.exterior] + list(pp.interiors)])
        hs = CV.get('deck_cell', 0.0012); bx = pp.bounds; gx, gy = np.meshgrid(np.arange(bx[0], bx[2], hs), np.arange(bx[1], bx[3], hs), indexing='ij')
        gp = np.c_[gx.ravel(), gy.ravel()]; gp = gp[shapely.contains_xy(pp, gp[:, 0], gp[:, 1])]
        gp = gp[shapely.distance(pp.boundary, shapely.points(gp)) > 0.7 * CV.get('deck_edge', 0.0007) + 0.0004]
        P2 = np.unique(np.round(np.r_[bp, gp], 9), axis=0); nref = []
        NR = int(CV.get('deck_refine', 0)) if refine is None else int(refine)      # refine 0 while deck_clear iterates (outline only), once at the end
        for rf in range(NR + 1):
            P2, tri = _tri_clean(*tri_plan(P2, pp))
            T3 = np.c_[P2, zraw(P2[:, 0], P2[:, 1])]; B3 = deck_envelope(T3, tri, t, tag=tag + '_r%d' % rf)
            if rf == NR: break
            Q = _steep_points(P2, B3, tri, pp, CV.get('deck_refine_slope', 2.0), CV.get('deck_refine_h', 0.00025))
            if CV.get('deck_refine_near') and len(Q):      # only round the through-holes (windows / pins / drains: where kit parts sit under the plate)
                hb = shapely.union_all([shapely.LinearRing(r.coords) for r in pp.interiors]) if len(pp.interiors) else None
                Q = Q[shapely.distance(hb, shapely.points(Q)) < CV['deck_refine_near']] if hb is not None else Q[:0]
            nmx = int(CV.get('deck_refine_max', 20000)); Q = Q[np.random.default_rng(rf).permutation(len(Q))[:nmx]] if len(Q) > nmx else Q; nref.append(int(len(Q)))      # memory: bounded refinement
            if not len(Q): break
            P2 = np.unique(np.round(np.r_[P2, Q], 9), axis=0)
        REP['deck_offset'].setdefault('refine_points_added', []).append(nref); thickness_report(T3, tri, B3, tag=tag); nt = len(T3)
        ed = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]); kk = ed[:, 0] * nt + ed[:, 1]; rk = ed[:, 1] * nt + ed[:, 0]
        bd = ed[~np.isin(kk, rk)]; a2, b2 = bd[:, 0], bd[:, 1]
        DKV.append(np.r_[T3, B3]); DKF.append(np.concatenate([tri, tri[:, ::-1] + nt, np.c_[a2, a2 + nt, b2 + nt], np.c_[a2, b2 + nt, b2]]) + o); o += 2 * nt
    return (np.concatenate(DKV), np.concatenate(DKF)) if DKV else (np.zeros((0, 3)), np.zeros((0, 3), np.int64))


def _tri_clean(P2, tri):
    """no bow-tie vertices on the boundary (a vertex with two outgoing boundary edges: drop its smallest triangle, as design_gen); unused
    points removed"""
    for _ in range(10):
        ed = np.concatenate([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]); kk = ed[:, 0] * len(P2) + ed[:, 1]; rk = ed[:, 1] * len(P2) + ed[:, 0]
        bd = ed[~np.isin(kk, rk)]; cnt = np.bincount(bd[:, 0], minlength=len(P2)); bv = np.nonzero(cnt > 1)[0]
        if not len(bv): break
        T = P2[tri]; ar = np.abs((T[:, 1, 0] - T[:, 0, 0]) * (T[:, 2, 1] - T[:, 0, 1]) - (T[:, 2, 0] - T[:, 0, 0]) * (T[:, 1, 1] - T[:, 0, 1]))
        drop = {int(np.nonzero((tri == v).any(1))[0][np.argmin(ar[(tri == v).any(1)])]) for v in bv}; tri = np.delete(tri, sorted(drop), 0)
    us = np.unique(tri); mp = -np.ones(len(P2), np.int64); mp[us] = np.arange(len(us)); return P2[us], mp[tri]


def _steep_points(P2, B3, tri, pp, smax, hmin):
    """plan points to add where the underside is steeper than smax (the cliff of a skirt's inside face): centroid + edge midpoints of
    every such triangle larger than hmin, inside the outline (a chord across a cliff otherwise cuts into the room beside the skirt)"""
    import shapely
    T = B3[tri]; d1 = T[:, 1] - T[:, 0]; d2 = T[:, 2] - T[:, 0]; det = d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]; ok = np.abs(det) > 1e-14
    gx = np.where(ok, (d1[:, 2] * d2[:, 1] - d2[:, 2] * d1[:, 1]) / np.where(ok, det, 1), 0); gy = np.where(ok, (d1[:, 0] * d2[:, 2] - d2[:, 0] * d1[:, 2]) / np.where(ok, det, 1), 0)
    L = np.linalg.norm(np.roll(T[:, :, :2], -1, 1) - T[:, :, :2], axis=2).max(1); st = ok & (np.hypot(gx, gy) > smax) & (L > 2 * hmin)
    if not st.any(): return np.zeros((0, 2))
    Pt = P2[tri[st]]; Q = np.r_[Pt.mean(1), 0.5 * (Pt[:, 0] + Pt[:, 1]), 0.5 * (Pt[:, 1] + Pt[:, 2]), 0.5 * (Pt[:, 2] + Pt[:, 0])]
    Q = Q[shapely.contains_xy(pp, Q[:, 0], Q[:, 1])]
    return Q[shapely.distance(pp.boundary, shapely.points(Q)) > 0.5 * hmin] if len(Q) else Q


def bag_replace(bag, k, V, F):
    """replace block k of a meshkit.Bag (later blocks' face indices shift)"""
    dn = len(V) - len(bag.V[k]); lab = bag.L[k][0] if len(bag.L[k]) else 0
    o = sum(len(v) for v in bag.V[:k])
    bag.V[k] = np.asarray(V, float); bag.F[k] = np.asarray(F, np.int64) + o; bag.L[k] = np.full(len(F), lab, np.int8)
    for j in range(k + 1, len(bag.F)): bag.F[j] = bag.F[j] + dn
    bag.n += dn


def deck_clear(dpg, zraw, CV, t, Xdeck, sd_cover, c, band=0.004, iters=4):
    """[fix4-cover2] G4: the deck keeps >= c from the cover solid by its OUTLINE (f3: the push moved single vertices up to 1.7 mm, which
    folded the plate). Deck points closer than c (signed distance sd_cover, < 0 inside the cover) within `band` (plan) of the outline cut
    a disc of radius 1.5 (c - d) + 0.25 mm out of the plan outline; rebuilt and re-measured until clear (only points within `band` of the
    outline are measured: the deck top keeps inset_inner 1.5 under the cover elsewhere; check_wall / deckclear measure the whole deck). -> (V, F, dpg, report)"""
    import shapely
    import time
    V, F = Xdeck; rep = []
    for it in range(iters + 1):
        t0 = time.time(); X = V[np.unique(F)]; X = np.r_[X, V[F].mean(1)]
        from scipy.spatial import cKDTree
        bq = shapely.get_coordinates(shapely.segmentize(dpg.boundary, 0.0003)); near = np.isfinite(cKDTree(bq).query(X[:, :2], distance_upper_bound=band)[0]); d = np.full(len(X), np.inf); d[near] = sd_cover(X[near]); t1 = time.time()
        cx = c + _thermal_allow(X, CV)      # [M5] the target grows towards the ends: the body + deck grow more than the cover about its x-datum
        bad = np.isfinite(d) & (d < cx) & near
        rep.append(dict(iter=it, min_mm=round(float(np.nanmin(np.where(np.isfinite(d), d, np.nan))) * 1000, 3) if np.isfinite(d).any() else None, n_lt_c=int(bad.sum()), measure_s=round(t1 - t0, 1)))
        if not bad.any() or it == iters: break
        cut = shapely.union_all(shapely.buffer(shapely.points(X[bad, :2]), 1.5 * (cx[bad] - d[bad]) + 0.00025, quad_segs=6))
        dpg = dpg.difference(cut).buffer(-0.0003, join_style=1).buffer(0.0003, join_style=1)
        t2 = time.time(); V, F = build_deck(dpg, zraw, CV, t, tag='deck_clear%d' % it, refine=0); rep[-1]['rebuild_s'] = round(time.time() - t2, 1)
    if int(CV.get('deck_refine', 0)) > 0:      # final build with the underside refined round the through-holes (same outline: the cover clearance is unchanged)
        t2 = time.time(); V, F = build_deck(dpg, zraw, CV, t, tag='deck_final'); rep.append(dict(final_refined_build_s=round(time.time() - t2, 1)))
    REP['deck_clear'] = rep
    return V, F, dpg, rep


def _thermal_allow(X, CV):
    """M5: extra deck clearance to the cover = k * max(|x - x0| - r0, 0) (cover.deck_clear_thermal {x0, r0, k}); the deck belongs to the dark
    body, which grows up to 1.47e-3 more than the light cover about the cover's x-datum x0"""
    th = (CV or {}).get('deck_clear_thermal')
    if not th: return np.zeros(len(X))
    return float(th['k']) * np.maximum(np.abs(X[:, 0] - th.get('x0', 0.0)) - th.get('r0', 0.0), 0.0)


def thickness_report(T3, tri, B3, tag='deck'):
    """distance of every underside vertex to the top surface (closest point on the top triangles, local search)"""
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from shell_wall import SurfDist
    Q = np.r_[B3, B3[tri].mean(1)]                                                     # underside vertices + face centroids
    d, _ = SurfDist(T3, tri, 12)(Q, 0.006); ok = np.isfinite(d); X = Q[ok]
    d = np.abs(d[ok])
    if len(d): REP['deck_offset'].setdefault(tag + '_thickness', []).append(dict(min_mm=round(float(d.min()) * 1000, 3), min_at_mm=np.round(X[np.argmin(d)] * 1000, 1).tolist(),
                                                                              p1_mm=round(float(np.percentile(d, 1)) * 1000, 3), median_mm=round(float(np.median(d)) * 1000, 3), n_lt_2=int((d < 0.002).sum())))


# ---------------------------------------------------------------------------------------------------------------- G5 orientation
def _components(nv, F):
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]]]); p = np.arange(nv)
    while len(E):
        a = p[E[:, 0]]; b = p[E[:, 1]]; m = a != b; E, a, b = E[m], a[m], b[m]
        if not len(E): break
        lo = np.minimum(a, b); hi = np.maximum(a, b); o = np.lexsort((lo, hi)); hs, ls = hi[o], lo[o]
        fst = np.r_[True, hs[1:] != hs[:-1]]; p[hs[fst]] = np.minimum(p[hs[fst]], ls[fst])
        while True:
            pp = p[p]
            if np.array_equal(pp, p): break
            p = pp
    return np.unique(p[F[:, 0]], return_inverse=True)[1].ravel()


def _weld(V, tol=1e-7):
    key = np.round(V / tol).astype(np.int64); _, inv = np.unique(key, axis=0, return_inverse=True); return inv.ravel()


def orient(V, F, tag, groups=None):
    """[groups: per-face kind; each kind is oriented on its own (a pilot inlay shares its ring with the boss: welded together they
    would form one non-manifold component)]
    consistent + outward winding per connected component (over welded vertices). Faces of a component are made consistent by a
    breadth-first walk over manifold edges (a neighbour that runs a shared edge the same way is flipped); then a component with a
    negative signed volume is flipped as a whole. Open components are made consistent only (their volume sign is meaningless)."""
    F = np.asarray(F, np.int64).copy()
    if groups is not None:
        groups = np.asarray(groups); tot = {}
        for gk in np.unique(groups):
            m = groups == gk; F[m] = orient(V, F[m], tag + '.%d' % int(gk)); r = REP['oriented'].pop(tag + '.%d' % int(gk))
            for k, v in r.items(): tot[k] = tot.get(k, 0) + v
        REP['oriented'][tag] = tot; return F
    w = _weld(np.asarray(V, float)); G = w[F]; nv = int(w.max()) + 1 if len(w) else 0
    if not len(F): return F
    comp = _components(nv, G); nflip = 0; ninv = 0; nopen = 0
    order = np.argsort(comp, kind='stable'); bounds = np.r_[0, np.cumsum(np.bincount(comp))]
    for k in range(len(bounds) - 1):
        fi = order[bounds[k]:bounds[k + 1]]; g = G[fi]
        deg = (g[:, 0] == g[:, 1]) | (g[:, 1] == g[:, 2]) | (g[:, 0] == g[:, 2])
        e = np.concatenate([g[:, [0, 1]], g[:, [1, 2]], g[:, [2, 0]]]); ef = np.tile(np.arange(len(g)), 3)
        ok = ~np.tile(deg, 3); e, ef = e[ok], ef[ok]
        ku = np.minimum(e[:, 0], e[:, 1]) * nv + np.maximum(e[:, 0], e[:, 1]); su, cnt = np.unique(ku, return_counts=True)
        closed = bool((cnt == 2).all())
        if not closed: nopen += 1
        # adjacency over manifold edges
        o = np.argsort(ku, kind='stable'); ks = ku[o]; pair = np.nonzero(ks[1:] == ks[:-1])[0]
        man = np.isin(ks[pair], su[cnt == 2]); pa = o[pair[man]]; pb = o[pair[man] + 1]
        same = e[pa, 0] == e[pb, 0]                                                    # both faces run the shared edge the same way
        flip = np.zeros(len(g), bool); seen = np.zeros(len(g), bool)
        if not same.any(): seen[:] = True                                              # already consistent: no walk
        A = [[] for _ in range(len(g))] if same.any() else None
        if A is not None:
            for x, y, sm in zip(ef[pa], ef[pb], same): A[x].append((y, sm)); A[y].append((x, sm))
        for s0 in range(len(g)):
            if seen[s0]: continue
            seen[s0] = True; st = [s0]
            while st:
                x = st.pop()
                for y, sm in A[x]:
                    if seen[y]: continue
                    seen[y] = True; flip[y] = flip[x] ^ bool(sm); st.append(y)
        g2 = np.where(flip[:, None], g[:, ::-1], g); nflip += int(flip.sum())
        if closed:
            T = np.asarray(V, float)[np.where(flip[:, None], F[fi][:, ::-1], F[fi])]
            vol = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum()
            if vol < 0: flip = ~flip; ninv += 1
        F[fi] = np.where(flip[:, None], F[fi][:, ::-1], F[fi])
    REP['oriented'][tag] = dict(components=int(len(bounds) - 1), faces_flipped=int(nflip), components_inverted=int(ninv), open_components=int(nopen))
    return F

# ---------------------------------------------------------------------------------------------------------------- body under-mounts vs the real chassis
# (interfere.py, pre-existing since f1: rear socket boss foot 1.1 into the scanned socket floor, bracket plate 1.5 into the apron wall,
#  front boss rib 0.13-0.2 over the chassis wing corner)
def _chassis():
    import os
    if 'v' not in _CH: _CH['v'] = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dev', 'base_parts.npz'))['Chassis_v'].astype(float)
    return _CH['v']


def _chassis_pts(lo, hi, h=0.0003):
    """chassis surface samples (vertices + area samples <= h apart) of the faces whose bounding box meets the box lo..hi (the scan's
    triangles are large: vertices alone miss faces that pass through a socket)"""
    import os
    if 'f' not in _CH: _CH['f'] = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dev', 'base_parts.npz'))['Chassis_f'].astype(np.int64)
    X = _chassis(); T = X[_CH['f']]; m = (T.min(1) <= hi).all(1) & (T.max(1) >= lo).all(1); T = T[m]
    if not len(T): return np.zeros((0, 3))
    ar = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1); n = np.clip((ar / (h * h)).astype(int), 1, 20000)
    rng = np.random.default_rng(5); fi = np.repeat(np.arange(len(T)), n); u = rng.random((len(fi), 2)); f = u.sum(1) > 1; u[f] = 1 - u[f]
    P = np.r_[T.reshape(-1, 3), T[fi, 0] + (T[fi, 1] - T[fi, 0]) * u[:, :1] + (T[fi, 2] - T[fi, 0]) * u[:, 1:]]
    return P[((P >= lo) & (P <= hi)).all(1)]


def socket_boss(x, y, rb, z0, zt, clr):
    """rear socket boss in the scanned apron socket: foot on the highest scanned floor point within 0.6 rb of the axis (z0 +- 4 mm), radius
    reduced so the socket wall (scan points between the foot + 0.5 and zt within rb + 0.5 of the axis) stays >= clr away (>= 2.5 mm).
    -> (z_foot, rb)"""
    X = _chassis_pts(np.array([x - rb - 0.002, y - rb - 0.002, z0 - 0.005]), np.array([x + rb + 0.002, y + rb + 0.002, zt + 0.002])); r = np.hypot(X[:, 0] - x, X[:, 1] - y)
    m = (r < 0.6 * rb) & (np.abs(X[:, 2] - z0) < 0.004); zf = max(z0, float(X[m, 2].max())) if m.any() else z0
    w = (r < rb + clr) & (X[:, 2] > zf + 0.0005) & (X[:, 2] < zt)
    rb2 = max(0.0025, min(rb, float(r[w].min()) - clr)) if w.any() else rb
    REP.setdefault('under_mounts', []).append(dict(kind='rear socket boss', at_mm=[round(x * 1000, 1), round(y * 1000, 1)], foot_z_mm=[round(z0 * 1000, 2), round(zf * 1000, 2)],
                                                   r_mm=[round(rb * 1000, 2), round(rb2 * 1000, 2)]))
    return zf, rb2


def plate_inner(x0, x1, y_, yi, z0, z1, clr):
    """rear bracket plate (plan x0..x1, |y| from |yi| outwards, z0..z1): its inner edge moves out until the chassis (apron) points in that
    box are >= clr inboard of it; never further in than yi. -> yi"""
    sg = np.sign(y_); X = _chassis_pts(np.array([x0 - clr, -0.3, z0 - clr]), np.array([x1 + clr, 0.3, z1 + clr]))
    m = (X[:, 0] > x0 - clr) & (X[:, 0] < x1 + clr) & (X[:, 2] > z0 - clr) & (X[:, 2] < z1 + clr) & (X[:, 1] * sg > abs(yi) - 0.004) & (X[:, 1] * sg < abs(y_))
    yn = max(abs(yi), float((X[m, 1] * sg).max()) + clr) if m.any() else abs(yi)
    REP.setdefault('under_mounts', []).append(dict(kind='rear plate', at_mm=[round(x0 * 500 + x1 * 500, 1), round(y_ * 1000, 1)], inner_y_mm=[round(abs(yi) * 1000, 2), round(yn * 1000, 2)]))
    return sg * yn


def rib_clear(p0, pts, th, clr, zmax):
    """front boss rib (_fin outline p0 = bottom point at the boss, pts = top points along the skin): p0 rises (<= zmax) until no chassis
    point lies within clr of the fin (plan strip th / 2 + clr about its plane, inside the outline grown by clr). -> new p0 or None"""
    import shapely
    X = _chassis(); P = np.array([p0] + list(pts), float); d = P[-1, :2] - P[0, :2]; L = np.linalg.norm(d)
    if L < 1e-6: return p0
    t = d / L; nrm = np.array([-t[1], t[0]])
    m = (np.abs((X[:, :2] - P[0, :2]) @ nrm) < th / 2 + clr) & ((X[:, :2] - P[0, :2]) @ t > -clr) & ((X[:, :2] - P[0, :2]) @ t < L + clr)
    m &= (X[:, 2] > P[:, 2].min() - clr) & (X[:, 2] < P[:, 2].max() + clr)
    if not m.any(): return p0
    Q = np.c_[(X[m, :2] - P[0, :2]) @ t, X[m, 2]]; z0 = p0[2]
    for zb in np.arange(z0, zmax, 0.0002):
        poly = shapely.Polygon(np.c_[np.r_[0.0, (P[1:, :2] - P[0, :2]) @ t], np.r_[zb, P[1:, 2]]]).buffer(clr)
        if not shapely.contains_xy(poly, Q[:, 0], Q[:, 1]).any():
            if zb > z0: REP.setdefault('under_mounts', []).append(dict(kind='front rib', at_mm=np.round(np.asarray(p0[:2]) * 1000, 1).tolist(), raised_mm=round((zb - z0) * 1000, 2)))
            return (p0[0], p0[1], zb)
    REP.setdefault('under_mounts', []).append(dict(kind='front rib dropped', at_mm=np.round(np.asarray(p0[:2]) * 1000, 1).tolist()))
    return None
