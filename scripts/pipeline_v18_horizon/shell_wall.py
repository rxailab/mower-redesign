# Real plastic wall thickness for zero-thickness skins (numpy / scipy only, no Blender).
#
#   W = shell_wall.build(V, F, lab, part, attrs, params)
#
# Input : the generator's final skin (all cuts done): V (n, 3) metres, F (m, 3) triangles with OUTWARD winding, lab (m,) paint code,
#         part (m,) moulding id (0 body, 1 cover, ...), attrs = dict of per-vertex float arrays (the generator's A; may be {}).
# Output: a closed solid per part:
#   - the outer skin, unchanged except that every free edge is trimmed back by edge_r and replaced by a rolled edge of radius edge_r,
#   - an inner skin offset by the wall thickness t along a heavily smoothed normal field (so small fillets and grooves are translated,
#     never folded), turning into the true surface normal near the edges so every edge face is perpendicular to the skin,
#   - rim faces closing every open boundary loop: outer roll R edge_r -> flat edge face -> inner round R edge_ri -> inner skin.
#   Faces of `exclude` (default lab 4 = the lid, a separate moving part) are passed through untouched.
# Per-face flag side: 0 outer (A-surface), 1 inner (B-surface), 2 rim. Rim faces of the roll keep the paint (lab) of the outer face they
# continue; the edge face and inner round get lab = bare_lab (7 = unpainted resin). Inner faces keep their outer lab (side says inner).
# W.frame_at(P, part) gives, for points on an ORIGINAL cut line, the local edge frame (n_e outward skin normal, b in-surface direction away
# from the material, wall thickness, distance); W.edge_point(P, part, depth) is the point on the edge face at that depth under the skin
# (default: the bottom of the flat edge face, t - edge_ri) - parts that used to start on a free edge (deck, well flange, sleeves) start there.
# thickness_at(X, params, part) evaluates the nominal wall incl. the local `thin` zones (also used by the generator's screw-room maths).
# Steps: clean -> free edges per part -> distance to them -> smoothed normals -> trim by edge_r -> edge frames -> offset field (smoothed
# normal, turning into the edge normal within `blend` of an edge) -> inner skin -> untangle (relaxes the few folds an exact offset makes
# behind ridges narrower than the wall) -> rims -> check (closure, wall thickness, poke-through, folds).
import numpy as np, time
from scipy.spatial import cKDTree
import labyrinthkit as _lk      # [fix3-labyrinth] cover lip rebate + body upstand at the part line (wall.labyrinth)

DEFAULTS = dict(t=0.0025, edge_r=0.0008, edge_ri=0.0003, roll_seg=6, ri_seg=3, smooth_r=0.0035, blend=0.004, bare_lab=10,
                paint_edge=False, thin=[], t_part={}, min_loop_len=0.0005, edge_follow_deg=20.0, untangle=6, check=True, loop_r=[])
# [v18c] loop_r: per-loop rolled-edge radius. Rules {name, attr, iso, part?, r, tol?}: an ORIGINAL free-edge loop whose vertices have
#        attrs[attr] == iso (median |attr - iso| < tol) gets the radius r (else edge_r). Every free edge is rolled exactly once, here:
#        the generator does not roll these edges itself when the wall is on (no double roll).
def _loop_radius(V, lp, pa, attrs, p):
    for ru in p.get('loop_r') or []:
        if 'part' in ru and int(ru['part']) != int(pa): continue
        a = attrs.get(ru['attr'])
        if a is None: continue
        if np.median(np.abs(a[lp] - ru['iso'])) < ru.get('tol', 1e-6): return float(ru['r']), ru.get('name', ru['attr'])
    return float(p['edge_r']), 'default'
SS = lambda x: np.clip(x, 0, 1) ** 2 * (3 - 2 * np.clip(x, 0, 1))


# ------------------------------------------------------------------------------------------------------------------ mesh helpers
def clean_faces(V, F):
    """Mask of faces to keep: drops float32-degenerate faces, exact duplicates (keeps one) and mirrored duplicate pairs (same three
    vertices, opposite winding: a zero-volume fin, e.g. the centre-line weld of the two halves) completely."""
    V32 = V.astype(np.float32); ok = ~((V32[F[:, 0]] == V32[F[:, 1]]).all(1) | (V32[F[:, 1]] == V32[F[:, 2]]).all(1) | (V32[F[:, 0]] == V32[F[:, 2]]).all(1))
    fs = np.sort(F, 1); _, inv, cnt = np.unique(fs, axis=0, return_inverse=True, return_counts=True); inv = inv.ravel()
    for g_ in np.nonzero(cnt > 1)[0]:
        ids = np.nonzero(inv == g_)[0]; r_ = [tuple(np.roll(F[i], -int(np.argmin(F[i])))) for i in ids]
        if len(set(r_)) > 1: ok[ids] = False                                   # opposite windings present: the pair is a fin, remove all
        else: ok[ids[1:]] = False
    return ok


def boundary_loops(F, nv):
    """Directed boundary loops (vertex index arrays) of a face set, plus for each loop the face owning each boundary edge."""
    e = np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]).astype(np.int64); fid = np.tile(np.arange(len(F)), 3)
    key = e[:, 0] * nv + e[:, 1]; rk = e[:, 1] * nv + e[:, 0]; isb = ~np.isin(key, rk)
    b = e[isb]; bf = fid[isb]; out = {}
    for i, (a_, c_) in enumerate(b.tolist()): out.setdefault(a_, []).append((c_, i))
    loops = []
    while out:
        a0 = next(iter(out)); lp = [a0]; fl = []; cur = a0
        while True:
            lst = out.get(cur)
            if not lst: break
            nx, ei = lst.pop()
            if not lst: del out[cur]
            fl.append(bf[ei])
            if nx == a0: break
            lp.append(nx); cur = nx
        loops.append((np.array(lp), np.array(fl)))
    return loops


def vertex_normals(V, F, area=True):
    P = V[F]; fn = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    if not area: fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-30)
    vn = np.zeros((len(V), 3))
    for c in range(3): np.add.at(vn, F[:, c], fn)
    return vn / np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-30)


def smooth_normals(V, F, query, R, h=None, opposite=-0.95):
    """Normal field of a heavily smoothed copy of the surface: area-weighted face normals are pooled in cells of size h, blurred with a
    Gaussian of radius R in space, then sampled at the vertices `query`. Cells facing almost exactly away from each other (dot < opposite:
    the two sides of a thin sheet) do not mix; anything less strict makes the field jump between facing pocket walls."""
    h = h or R / 3.0; P = V[F]; fn = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); a = np.linalg.norm(fn, axis=1); c = P.mean(1)
    key = np.floor(c / h).astype(np.int64); _, ci = np.unique(key, axis=0, return_inverse=True); ci = ci.ravel(); nc = ci.max() + 1
    N = np.zeros((nc, 3)); C = np.zeros((nc, 3)); W = np.zeros(nc)
    for k in range(3): np.add.at(N[:, k], ci, fn[:, k]); np.add.at(C[:, k], ci, c[:, k] * a)
    np.add.at(W, ci, a); C /= np.maximum(W, 1e-30)[:, None]; Nu = N / np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-30)
    tr = cKDTree(C); k_ = 96; d, i = tr.query(C, k=k_, distance_upper_bound=2.5 * R, workers=-1); ok = np.isfinite(d); i = np.where(ok, i, 0)
    w = np.where(ok, np.exp(-0.5 * (np.where(ok, d, 0) / R) ** 2), 0.0) * ((Nu[i] * Nu[:, None, :]).sum(2) > opposite)
    Ns = (w[..., None] * N[i]).sum(1); Ns /= np.maximum(np.linalg.norm(Ns, axis=1, keepdims=True), 1e-30)
    vn = vertex_normals(V, F)[query]; Q = V[query]; out = np.zeros((len(Q), 3))
    for s0 in range(0, len(Q), 200000):
        q = Q[s0:s0 + 200000]; d, i = tr.query(q, k=8, workers=-1); w = np.exp(-0.5 * (d / h) ** 2) * ((Ns[i] * vn[s0:s0 + 200000, None, :]).sum(2) > opposite) + 1e-12
        o_ = (w[..., None] * Ns[i]).sum(1); out[s0:s0 + 200000] = o_ / np.maximum(np.linalg.norm(o_, axis=1, keepdims=True), 1e-30)
    return out


def split(V, F, h, facs, vatt, iso=0.0, eps=2e-9, snap=0.015):
    """Cut every triangle crossed by the iso-line h == iso (h per vertex). facs: list of per-face arrays, vatt: dict of per-vertex arrays
    (interpolated on new vertices; arrays of shape (n, 3) allowed). Same scheme as design_gen.split (corner snapping against needles)."""
    h = h - iso; sg = np.where(h > eps, 1, np.where(h < -eps, -1, 0)).astype(np.int8)
    fs = sg[F]; cross = ((fs < 0).sum(1) > 0) & ((fs > 0).sum(1) > 0)
    if not cross.any(): return V, F, facs, vatt, h + iso
    ea = F[cross].ravel(); eb = np.roll(F[cross], -1, 1).ravel(); ce = sg[ea] * sg[eb] < 0; ea = ea[ce]; eb = eb[ce]; tq = h[ea] / (h[ea] - h[eb])
    sn = np.unique(np.r_[ea[tq < snap], eb[tq > 1 - snap]])
    if len(sn):
        h = h.copy(); h[sn] = 0.0; sg = np.where(h > eps, 1, np.where(h < -eps, -1, 0)).astype(np.int8); fs = sg[F]; cross = ((fs < 0).sum(1) > 0) & ((fs > 0).sum(1) > 0)
    Fc = F[cross]; sc = fs[cross]; nv = len(V); e1 = np.roll(Fc, -1, 1); cut = (sc * np.roll(sc, -1, 1)) < 0
    lo = np.minimum(Fc, e1)[cut]; hi = np.maximum(Fc, e1)[cut]; uk, inv = np.unique(lo.astype(np.int64) * nv + hi, return_inverse=True); a = uk // nv; b = uk % nv
    tp = h[a] / (h[a] - h[b]); V = np.concatenate([V, V[a] + (V[b] - V[a]) * tp[:, None]])
    for k in vatt:
        x = vatt[k]; t_ = tp.reshape((-1,) + (1,) * (x.ndim - 1)); vatt[k] = np.concatenate([x, x[a] + (x[b] - x[a]) * t_])
    h = np.concatenate([h, np.zeros(len(a))]); E = np.full(Fc.shape, -1, np.int64); E[cut] = nv + inv.ravel()
    n0 = (sc == 0).sum(1); out = [F[~cross]]; idx = [np.nonzero(~cross)[0]]; ci = np.nonzero(cross)[0]
    m2 = n0 == 0
    if m2.any():
        f = Fc[m2]; s = sc[m2]; e = E[m2]; r = np.arange(len(f)); k0 = np.where(s[:, 0] != s[:, 1], np.where(s[:, 0] != s[:, 2], 0, 1), 2)
        a_ = f[r, k0]; b_ = f[r, (k0 + 1) % 3]; c_ = f[r, (k0 + 2) % 3]; pab = e[r, k0]; pca = e[r, (k0 + 2) % 3]
        out += [np.c_[a_, pab, pca], np.c_[pab, b_, c_], np.c_[pab, c_, pca]]; idx += [ci[m2]] * 3
    m1 = n0 == 1
    if m1.any():
        f = Fc[m1]; s = sc[m1]; e = E[m1]; r = np.arange(len(f)); kz = np.argmax(s == 0, 1)
        a_ = f[r, kz]; b_ = f[r, (kz + 1) % 3]; c_ = f[r, (kz + 2) % 3]; p = e[r, (kz + 1) % 3]
        out += [np.c_[a_, b_, p], np.c_[a_, p, c_]]; idx += [ci[m1]] * 2
    idx = np.concatenate(idx); return V, np.concatenate(out), [x[idx] for x in facs], vatt, h + iso


def _pt_tri(p, A, B, C):
    """Closest points on triangles (A, B, C) to points p (all (k, 3)); Ericson's region test, vectorised."""
    ab = B - A; ac = C - A; ap = p - A; d1 = (ab * ap).sum(1); d2 = (ac * ap).sum(1)
    bp = p - B; d3 = (ab * bp).sum(1); d4 = (ac * bp).sum(1); cp = p - C; d5 = (ab * cp).sum(1); d6 = (ac * cp).sum(1)
    va = d3 * d6 - d5 * d4; vb = d5 * d2 - d1 * d6; vc = d1 * d4 - d3 * d2; den = np.where(np.abs(va + vb + vc) > 1e-300, va + vb + vc, 1e-300)
    v = vb / den; w = vc / den; res = A + ab * v[:, None] + ac * w[:, None]
    def put(m, q):
        nonlocal res; res = np.where(m[:, None], q, res)
    tbc = np.where(np.abs((d4 - d3) + (d5 - d6)) > 1e-300, (d4 - d3) / ((d4 - d3) + (d5 - d6) + 1e-300), 0)
    put((va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0), B + (C - B) * tbc[:, None])
    tac = np.where(np.abs(d2 - d6) > 1e-300, d2 / (d2 - d6 + 1e-300), 0); put((vb <= 0) & (d2 >= 0) & (d6 <= 0), A + ac * tac[:, None])
    tab = np.where(np.abs(d1 - d3) > 1e-300, d1 / (d1 - d3 + 1e-300), 0); put((vc <= 0) & (d1 >= 0) & (d3 <= 0), A + ab * tab[:, None])
    put((d6 >= 0) & (d5 <= d6), C); put((d3 >= 0) & (d4 <= d3), B); put((d1 <= 0) & (d2 <= 0), A)
    return res


class SurfDist:
    """Signed distance of points to a triangle set (exact, among the k faces with the nearest centroids). Sign: + on the side the face
    normals point to."""
    def __init__(s, V, F, k=10):
        s.P = V[F]; s.n = np.cross(s.P[:, 1] - s.P[:, 0], s.P[:, 2] - s.P[:, 0]); s.n /= np.maximum(np.linalg.norm(s.n, axis=1, keepdims=True), 1e-30)
        s.tr = cKDTree(s.P.mean(1)); s.k = k
    def __call__(s, X, upper=np.inf):
        out = np.full(len(X), np.inf); fid = np.full(len(X), -1)
        for s0 in range(0, len(X), 100000):
            x = X[s0:s0 + 100000]; d, i = s.tr.query(x, k=s.k, distance_upper_bound=upper, workers=-1); ok = np.isfinite(d); i = np.where(ok, i, 0)
            P = s.P[i.ravel()]; xx = np.repeat(x, s.k, 0); q = _pt_tri(xx, P[:, 0], P[:, 1], P[:, 2]); dd = np.linalg.norm(xx - q, axis=1).reshape(-1, s.k)
            dd = np.where(ok, dd, np.inf); j = np.argmin(dd, 1); r = np.arange(len(x)); fi = i[r, j]; dm = dd[r, j]
            sg = np.sign(((x - s.P[fi].mean(1)) * s.n[fi]).sum(1)); sg = np.where(sg == 0, 1, sg)
            out[s0:s0 + 100000] = np.where(np.isfinite(dm), dm * sg, np.inf); fid[s0:s0 + 100000] = np.where(np.isfinite(dm), fi, -1)
        return out, fid


def _ls(X, s, sig, win):
    n = len(X); acc = np.zeros_like(X); ws = np.zeros(n); Ltot = s[-1]
    for o in range(-win, win + 1):
        j = (np.arange(n) + o) % n; ds = s[j] - s
        ds = np.where(ds > Ltot / 2, ds - Ltot, np.where(ds < -Ltot / 2, ds + Ltot, ds)); w = np.exp(-0.5 * (ds / sig) ** 2)
        acc += w[:, None] * X[j]; ws += w
    return acc / ws[:, None]


def untangle(X, Fi, Pout, pin, rounds=6, iters=12, lam=0.5):
    """Local folds of the inner skin (an inner face turned over against the outer face it was copied from: the swallowtail an exact
    offset makes behind a ridge narrower than the wall, or behind a fillet tighter than the wall) are relaxed with umbrella smoothing of the
    inner vertices around them (2-ring), pinned vertices excepted. Returns the new positions and the face counts per round."""
    import scipy.sparse as sp
    if rounds <= 0: return X, []
    n = len(X); e = np.concatenate([Fi[:, [0, 1]], Fi[:, [1, 2]], Fi[:, [2, 0]]]); Adj = sp.coo_matrix((np.ones(len(e)), (e[:, 0], e[:, 1])), shape=(n, n)).tocsr()
    Adj = ((Adj + Adj.T) > 0).astype(float); deg = np.asarray(Adj.sum(1)).ravel(); no = np.cross(Pout[:, 1] - Pout[:, 0], Pout[:, 2] - Pout[:, 0]); hist = []
    best = (np.inf, X)
    for r_ in range(rounds + 1):
        Q = X[Fi]; ni = np.cross(Q[:, 1] - Q[:, 0], Q[:, 2] - Q[:, 0]); bad = (ni * no).sum(1) > 0       # inner winding is reversed: same sign = turned over
        hist.append(int(bad.sum()))
        if bad.sum() < best[0]: best = (int(bad.sum()), X.copy())                                        # [v18c] keep the best state (smoothing can add folds)
        if not bad.any() or r_ == rounds: break
        m = np.zeros(n, bool); m[Fi[bad].ravel()] = True
        for _ in range(2 + r_): m = m | (Adj @ m.astype(float) > 0)
        m &= ~pin
        for _ in range(iters):
            Lx = (Adj @ X) / np.maximum(deg, 1)[:, None]; X = np.where(m[:, None], X + lam * (Lx - X), X)
    return best[1], hist


# ------------------------------------------------------------------------------------------------------------------ the solid
class Wall:
    pass


def build(V, F, lab, part, attrs=None, params=None, exclude=None, log=print):
    T0 = time.time(); p = dict(DEFAULTS); p.update(params or {}); attrs = {k: np.asarray(v, float).copy() for k, v in (attrs or {}).items()}
    t0 = p['t']; R = p['edge_r']; RI = p['edge_ri']; NR = int(p['roll_seg']); NI = int(p['ri_seg']); BL = p['blend']
    RMAX = max([R] + [float(ru['r']) for ru in (p.get('loop_r') or [])])
    V = np.asarray(V, float).copy(); F = np.asarray(F, np.int64).copy(); lab = np.asarray(lab, np.int8).copy(); part = np.asarray(part, np.int16).copy()
    exc = (lab == 4) if exclude is None else np.asarray(exclude, bool)
    keep = clean_faces(V, F); ncl = int((~keep).sum()); F = F[keep]; lab = lab[keep]; part = part[keep]; exc = exc[keep]
    tm = {}; tm['clean'] = time.time() - T0
    # orientation: outward = +z on the highest faces of the skin
    P_ = V[F]; fn_ = np.cross(P_[:, 1] - P_[:, 0], P_[:, 2] - P_[:, 0]); top = P_[:, :, 2].mean(1) > np.percentile(P_[:, :, 2].mean(1), 97)
    flipped = bool(fn_[top, 2].sum() < 0)
    if flipped: F = F[:, ::-1].copy()
    wf = ~exc; parts = sorted(set(part[wf].tolist()))
    # ---- 1. original free edges of every part
    used = np.zeros(len(V), bool); used[F[wf].ravel()] = True
    loops0 = {}; dense = {}; RL0 = {}
    for pa in parts:
        lp = [l_ for l_, _ in boundary_loops(F[wf & (part == pa)], len(V))]
        lp = [l_ for l_ in lp if len(l_) >= 3 and np.linalg.norm(np.diff(V[np.r_[l_, l_[:1]]], axis=0), axis=1).sum() > p['min_loop_len']]
        loops0[pa] = lp; pts = []; lid_ = []; RL0[pa] = [_loop_radius(V, l_, pa, attrs, p) for l_ in lp]
        for k, l_ in enumerate(lp):
            Q = V[np.r_[l_, l_[:1]]]; sl = np.linalg.norm(np.diff(Q, axis=0), axis=1); s_ = np.r_[0, np.cumsum(sl)]; ss_ = np.arange(0, s_[-1], 0.0001)
            pts.append(np.c_[[np.interp(ss_, s_, Q[:, c]) for c in range(3)]].T); lid_.append(np.full(len(ss_), k))
        dense[pa] = (np.concatenate(pts), np.concatenate(lid_)) if pts else (np.zeros((0, 3)), np.zeros(0, int))
    tm['loops'] = time.time() - T0
    # ---- 2. distance of every wall vertex to the free edges of its own part
    vpart = np.full(len(V), -1)
    for pa in parts: vpart[np.unique(F[wf & (part == pa)])] = pa
    db = np.full(len(V), np.inf); dbl = np.full(len(V), -1); rv = np.full(len(V), R)
    for pa in parts:
        D, Lk = dense[pa]
        if not len(D): continue
        vi = np.nonzero(vpart == pa)[0]; d_, i_ = cKDTree(D).query(V[vi], distance_upper_bound=RMAX + BL + 0.002, workers=-1); ok = np.isfinite(d_)
        db[vi[ok]] = d_[ok]; dbl[vi[ok]] = Lk[i_[ok]] + 1000 * pa; rv[vi[ok]] = np.array([r_ for r_, _ in RL0[pa]])[Lk[i_[ok]]]
    # ---- 3. smoothed normal field (pooled over the whole skin, the lid included: it is flush with the cover)
    ns = np.zeros((len(V), 3)); qi = np.nonzero(used)[0]; ns[qi] = smooth_normals(V, F, qi, p['smooth_r'])
    tm['smooth'] = time.time() - T0
    # ---- 4. trim the outer skin back by R along every free edge (exact iso-line cut, then the band is removed)
    vatt = dict(attrs); vatt['_ns'] = ns; vatt['_db'] = np.where(np.isfinite(db), db, 1.0); vatt['_rv'] = rv
    LBR = _lk.rules(p); LBF = []                                                       # [fix3-labyrinth] per-rule edge-distance field of its own loop(s)
    for nm_, ru_ in LBR.items():      # [fix3-labyrinth]
        ks_ = [k_ + 1000 * pa for pa in parts for k_, (_, n2_) in enumerate(RL0[pa]) if n2_ == nm_]      # [fix3-labyrinth]
        if ks_ and _lk.levels(ru_): vatt['_lbf_' + nm_] = np.where(np.isin(dbl, ks_) & np.isfinite(db), db, 1.0); LBF.append(nm_)      # [fix3-labyrinth]
    hfield = np.where(np.isfinite(db), db - rv, 1.0)                                   # [v18c] trimmed back by each loop's own radius
    facs = [lab, part.astype(np.int64), exc.astype(np.int8)]
    V, F, facs, vatt, hfield = split(V, F, hfield, facs, vatt, iso=0.0)
    lab, part, exc = facs[0], facs[1].astype(np.int16), facs[2].astype(bool); wf = ~exc
    band = wf & (hfield[F].mean(1) < 0.0); F = F[~band]; lab = lab[~band]; part = part[~band]; exc = exc[~band]; wf = ~exc
    for nm_ in LBF:                                                                    # [fix3-labyrinth] exact iso-lines across the thickness taper
        for lv_ in _lk.levels(LBR[nm_]):      # [fix3-labyrinth]
            facs = [lab, part.astype(np.int64), exc.astype(np.int8)]; V, F, facs, vatt, _ = split(V, F, vatt['_lbf_' + nm_], facs, vatt, iso=lv_)      # [fix3-labyrinth]
            lab, part, exc = facs[0], facs[1].astype(np.int16), facs[2].astype(bool); wf = ~exc      # [fix3-labyrinth]
        vatt.pop('_lbf_' + nm_)      # [fix3-labyrinth]
    for nm_ in LBR:      # [fix3-labyrinth]
        if nm_ not in LBF: vatt.pop('_lbf_' + nm_, None)      # [fix3-labyrinth]
    ns = vatt.pop('_ns'); ns /= np.maximum(np.linalg.norm(ns, axis=1, keepdims=True), 1e-30); db = vatt.pop('_db'); rv = vatt.pop('_rv'); attrs = vatt
    tm['trim'] = time.time() - T0
    # ---- 5. new free edges (at distance R inside the cut lines): frames
    vn = vertex_normals(V, F[wf]); loops = []
    for pa in parts:
        D, Lk = dense[pa]; trD = cKDTree(D) if len(D) else None
        for lp, lf in boundary_loops(F[wf & (part == pa)], len(V)):
            if len(lp) < 3: continue
            Q = V[lp]; sl = np.linalg.norm(np.roll(Q, -1, 0) - Q, axis=1); s_ = np.r_[0, np.cumsum(sl)[:-1]]; Ltot = sl.sum()
            if Ltot < p['min_loop_len']: continue
            dq, iq = trD.query(Q, workers=-1); src = int(np.bincount(Lk[iq]).argmax())
            s2 = np.r_[s_, Ltot]; Q2 = np.r_[Q, Q[:1]]
            def at(sv): sv = sv % Ltot; return np.c_[[np.interp(sv, s2, Q2[:, c]) for c in range(3)]].T
            tg = at(s_ + 0.0003) - at(s_ - 0.0003); tg /= np.maximum(np.linalg.norm(tg, axis=1, keepdims=True), 1e-30)
            ne = _ls(vn[lp], s_, 0.0004, 12); ne /= np.maximum(np.linalg.norm(ne, axis=1, keepdims=True), 1e-30)
            # where the skin at the edge turns away from the smoothed normal (a narrow strip in a sharp corner), the edge face follows the
            # smoothed normal instead, otherwise the offset direction would swing by that angle within the blend band and fold the wall
            ag = np.degrees(np.arccos(np.clip((ne * ns[lp]).sum(1), -1, 1))); q_ = SS((ag - p['edge_follow_deg']) / 15.0)[:, None]
            ne = (1 - q_) * ne + q_ * ns[lp]; ne -= (ne * tg).sum(1, keepdims=True) * tg; ne /= np.maximum(np.linalg.norm(ne, axis=1, keepdims=True), 1e-30)
            bb = np.cross(tg, ne); sgn = np.sign(((D[iq] - Q) * bb).sum(1).sum()); bb *= sgn if sgn != 0 else 1.0
            up_ = (LBR.get(RL0[pa][src][1]) or {}).get('upstand')                          # [fix3-labyrinth] a 4.5 mm long rim profile: a smoother frame
            if up_ is not None:                                                            # (1 deg of noise in b = 0.08 mm at the upstand top: folds) [fix3-labyrinth]
                sgf_ = up_.get('frame_sigma', 0.0015)      # [fix3-labyrinth]
                tg = _lk.smooth_along(s_, Ltot, tg, sgf_); tg /= np.maximum(np.linalg.norm(tg, axis=1, keepdims=True), 1e-30); ne = _lk.smooth_along(s_, Ltot, ne, sgf_)      # [fix3-labyrinth]
                ne -= (ne * tg).sum(1, keepdims=True) * tg; ne /= np.maximum(np.linalg.norm(ne, axis=1, keepdims=True), 1e-30); bb = np.cross(tg, ne) * (sgn if sgn != 0 else 1.0)      # [fix3-labyrinth]
            rl_, nm_ = RL0[pa][src]; loops.append(dict(part=pa, v=lp, ne=ne, b=bb, tg=tg, src=src, length=Ltot, lf=lf, r=rl_, name=nm_))
    # labels of the outer faces along each loop (for the painted roll)
    Fw_idx = {pa: np.nonzero(wf & (part == pa))[0] for pa in parts}
    for L in loops: L['lab'] = lab[Fw_idx[L['part']][L['lf']]]
    tm['frames'] = time.time() - T0
    # ---- 6. thickness and offset direction of every wall vertex
    used = np.zeros(len(V), bool); used[F[wf].ravel()] = True; wv = np.nonzero(used)[0]
    vp_ = np.full(len(V), -1); vp_[F[wf].ravel()] = np.repeat(part[wf], 3)
    th = thickness_at(V, p, vp_)
    LBREP = {}      # [fix3-labyrinth]
    for nm_, ru_ in LBR.items():                                                       # [fix3-labyrinth] wall at the seam edge: cover lip / body upstand root
        if 't_edge' not in ru_: continue      # [fix3-labyrinth]
        for pa in parts:      # [fix3-labyrinth]
            D, Lk = dense[pa]; ks_ = [k_ for k_, (_, n2_) in enumerate(RL0[pa]) if n2_ == nm_]      # [fix3-labyrinth]
            if not ks_ or not len(D): continue      # [fix3-labyrinth]
            vi = wv[vp_[wv] == pa]; d_, i_ = cKDTree(D).query(V[vi], distance_upper_bound=ru_.get('t_len', 0) + ru_.get('t_taper', 0.001) + 0.001, workers=-1)      # [fix3-labyrinth]
            ok = np.isfinite(d_); ok[ok] = np.isin(Lk[i_[ok]], ks_); w_ = _lk.weight(d_[ok], ru_)      # [fix3-labyrinth]
            th[vi[ok]] = th[vi[ok]] + (float(ru_['t_edge']) + _lk.relief(V[vi[ok]], ru_) - th[vi[ok]]) * w_; LBREP[nm_] = dict(part=int(pa), t_edge_mm=round(float(ru_['t_edge']) * 1000, 3), verts=int(ok.sum()))      # [fix3-labyrinth]  [fix4-cover2] + M5 relief
    LV = np.concatenate([L['v'] for L in loops]); LN = np.concatenate([L['ne'] for L in loops]); LP = np.concatenate([np.full(len(L['v']), L['part']) for L in loops])
    noff = ns.copy()
    for pa in parts:
        m_ = LP == pa
        if not m_.any(): continue
        vi = wv[vpart_of(F, wf, part, pa, len(V))[wv]]; tr = cKDTree(V[LV[m_]]); d_, i_ = tr.query(V[vi], distance_upper_bound=BL + RMAX + 0.003, workers=-1)
        ok = np.isfinite(d_); w = SS((db[vi] - rv[vi]) / BL); w = np.where(ok, w, 1.0); ne_ = LN[m_][np.where(ok, i_, 0)]
        n2 = w[:, None] * ns[vi] + (1 - w[:, None]) * ne_; noff[vi] = n2 / np.maximum(np.linalg.norm(n2, axis=1, keepdims=True), 1e-30)
    noff[LV] = LN                                                                     # edge faces exactly perpendicular to the skin
    tm['offset'] = time.time() - T0
    # ---- 7. inner skin: 1:1 copy, reversed winding
    nV = len(V); imap = -np.ones(nV, np.int64); imap[wv] = nV + np.arange(len(wv))
    Vin = V[wv] - th[wv, None] * noff[wv]
    Fw = F[wf]; Fin = imap[Fw][:, ::-1]
    pin = np.zeros(len(wv), bool); pin[imap[LV] - nV] = True                         # inner edge rings stay where the rims expect them
    Vin, W_unt = untangle(Vin, Fin - nV, V[Fw], pin, rounds=p['untangle'])
    # ---- 8. rims
    RV = []; RF = []; RL = []; RP = []; nxt = nV + len(wv); rim_info = []
    DRAFT_ = {ru.get('name', ru.get('attr')): float(ru.get('draft_deg', 0.0)) for ru in (p.get('loop_r') or [])}      # [hz] per-loop edge-face draft
    for L in loops:
        lp = L['v']; n = len(lp); ne = L['ne']; bb = L['b']; thl = th[lp]; o = V[lp]; RLo = L['r']
        ru_ = LBR.get(L['name'], {})                                                    # [fix3-labyrinth] r_frac: keep the roll radius on a thin lip
        r_ = np.minimum(RLo, ru_.get('r_frac', 0.42) * thl); ri_ = np.minimum(RI, 0.2 * thl)      # [fix3-labyrinth]
        prof = []                                                                      # (u, w) per ring, each (n,)
        if ru_.get('upstand') is not None and L['part'] == 0:                          # [fix3-labyrinth] roll -> drained ledge -> upstand -> inner skin
            ru_ = _lk.at_points(ru_, o)      # [fix4-cover2] M5: per-vertex upstand depth (thermal relief at the ends)
            dl_, Lw_ = _lk.ledge_delta(ne, bb, ru_, float(np.median(r_)))      # [fix3-labyrinth]
            prof, rp_ = _lk.profile(n, RLo, thl, r_, ru_, NR, dl_, Lw_); rp_.update(_lk.ledge_report(ne, bb, dl_, Lw_)); LBREP.setdefault(L['name'], {}).update(upstand=rp_)      # [fix3-labyrinth]
        else:      # [fix3-labyrinth]
          dft_ = np.tan(np.radians(DRAFT_.get(L['name'], 0.0)))      # [hz] drafted edge face (loop_r rule draft_deg): the face leans into the opening going in,
          for k in range(1, NR + 1):      # [fix3-labyrinth]
            ph = 0.5 * np.pi * k / NR; prof.append((RLo * np.sin(ph), -r_ * (1 - np.cos(ph))))
          if dft_ > 0:      # [hz] i.e. the opening is widest at the outer skin (a pin / slide pulled outwards along the normal releases)
            sh_ = (thl - ri_ - r_) * dft_; prof.append((RLo + sh_, -(thl - ri_)))      # [hz]
            for k in range(1, NI + 1):      # [hz]
                ps = 0.5 * np.pi * k / NI; prof.append((RLo + sh_ - ri_ + ri_ * np.cos(ps), -(thl - ri_) - ri_ * np.sin(ps)))      # [hz]
          else:      # [hz]
            prof.append((np.full(n, RLo), -(thl - ri_)))      # [fix3-labyrinth]
            for k in range(1, NI + 1):      # [fix3-labyrinth]
              ps = 0.5 * np.pi * k / NI; prof.append((RLo - ri_ + ri_ * np.cos(ps), -(thl - ri_) - ri_ * np.sin(ps)))
        L['nprof'] = len(prof)                                                         # [fix3-labyrinth] rings per loop (the upstand profile has more)
        rings = [np.asarray(lp)]
        for u, w in prof:
            u = np.broadcast_to(u, (n,)); w = np.broadcast_to(w, (n,))
            RV.append(o + u[:, None] * bb + w[:, None] * ne); rings.append(nxt + np.arange(n)); nxt += n
        rings.append(imap[lp])
        j = np.arange(n); j2 = (j + 1) % n; nring = len(rings)
        for k in range(nring - 1):
            a = rings[k][j]; b_ = rings[k][j2]; c = rings[k + 1][j2]; d = rings[k + 1][j]
            RF.append(np.c_[a, d, c]); RF.append(np.c_[a, c, b_])
            lk = L['lab'] if k < NR or p['paint_edge'] and k < nring - 1 - NI - 1 else np.full(n, p['bare_lab'], np.int8)
            RL += [lk, lk]; RP.append(np.full(2 * n, L['part']))
        rim_info.append(dict(part=L['part'], src=L['src'], n=n, length=L['length']))
    RV = np.concatenate(RV); RF = np.concatenate(RF); RL = np.concatenate(RL).astype(np.int8); RP = np.concatenate(RP)
    # orientation of the rims: the first roll face of every loop must face outwards (along ne + b)
    Vall = np.concatenate([V, Vin, RV]); tm['rims'] = time.time() - T0
    Fr = RF; pr = Vall[Fr]; fr = np.cross(pr[:, 1] - pr[:, 0], pr[:, 2] - pr[:, 0])
    o_ = 0
    for L in loops:
        n = len(L['v']); nring = L['nprof'] + 2; seg = fr[o_:o_ + 2 * n]; ex = np.r_[L['ne'] + L['b'], L['ne'] + L['b']]      # [fix3-labyrinth] per-loop ring count
        if (seg * ex).sum() < 0: Fr[o_:o_ + 2 * n * (nring - 1)] = Fr[o_:o_ + 2 * n * (nring - 1)][:, ::-1]
        o_ += 2 * n * (nring - 1)
    # ---- 9. assemble
    Fout = np.concatenate([F, Fin, Fr]); labo = np.concatenate([lab, lab[wf], RL]); parto = np.concatenate([part, part[wf], RP]).astype(np.uint8)
    side = np.concatenate([np.zeros(len(F), np.uint8), np.ones(len(Fin), np.uint8), np.full(len(Fr), 2, np.uint8)])
    src_v = np.concatenate([np.arange(nV), wv, np.concatenate([np.repeat(L['v'][None], L['nprof'], 0).ravel() for L in loops])])      # [fix3-labyrinth] per-loop ring count
    attrs = {k: x[src_v] for k, x in attrs.items()}
    W = Wall(); W.V = Vall; W.F = Fout; W.lab = labo; W.part = parto; W.side = side; W.attrs = attrs; W.flipped = flipped
    W.loops = loops; W.th = np.concatenate([th, th[wv], np.zeros(len(RV))]); W.params = p; W.R = p['edge_r']; W.RI = RI
    W.untangle = W_unt; W.noff = noff; W.src_v = src_v; W.n_cleaned = ncl; W.exclude = np.concatenate([exc, np.zeros(len(Fin) + len(Fr), bool)])
    # edge frames for points on the original cut lines
    W._fr = {}
    for pa in parts:
        m_ = [L for L in loops if L['part'] == pa]
        if not m_: continue
        Pq = np.concatenate([V[L['v']] + L['r'] * L['b'] for L in m_]); W._fr[pa] = (cKDTree(Pq), np.concatenate([L['ne'] for L in m_]), np.concatenate([L['b'] for L in m_]),
                                                                              np.concatenate([th[L['v']] for L in m_]))
    def frame_at(P, pa):
        tr, ne, b, t_ = W._fr[pa]; d, i = tr.query(np.asarray(P, float), workers=-1); return ne[i], b[i], t_[i], d
    def edge_point(P, pa, depth=None):
        ne, b, t_, _ = frame_at(P, pa); dp = (t_ - np.minimum(RI, 0.2 * t_)) if depth is None else depth; return np.asarray(P, float) - ne * np.reshape(dp, (-1, 1))
    W.frame_at = frame_at; W.edge_point = edge_point
    tm['assemble'] = time.time() - T0
    W.report = dict(t_mm=t0 * 1000, edge_r_mm=p['edge_r'] * 1000, edge_ri_mm=RI * 1000, faces_in=int(keep.sum() + ncl), removed_bad_faces=ncl, winding_flipped=flipped,
                    loops=[dict(part=int(L['part']), name=L['name'], r_mm=round(L['r'] * 1000, 2), verts=int(len(L['v'])), length_mm=round(L['length'] * 1000, 1),
                                z_mm=[round(float(V[L['v'], 2].min()) * 1000, 1), round(float(V[L['v'], 2].max()) * 1000, 1)],
                                x_mm=[round(float(V[L['v'], 0].min()) * 1000, 1), round(float(V[L['v'], 0].max()) * 1000, 1)]) for L in loops],
                    faces_out=dict(outer=int((side == 0).sum()), inner=int((side == 1).sum()), rim=int((side == 2).sum())), untangle_rounds=W_unt)
    if LBREP: W.report['labyrinth'] = LBREP                                           # [fix3-labyrinth]
    if p['check']: W.report['check'] = check(W, log=log)
    tm['total'] = time.time() - T0; W.report['time_s'] = {k: round(v, 2) for k, v in tm.items()}
    return W


def thickness_at(X, params, part=None):
    """Nominal wall at points X: t, or t_part[part], reduced by the `thin` zones ({c, c2?, r, t, taper, part?}: t inside radius r around the
    point c (or the segment c-c2), back to nominal over `taper`, default 3x the thickness change + 2 mm). part: scalar or per-point array."""
    p = dict(DEFAULTS); p.update(params or {}); X = np.atleast_2d(np.asarray(X, float)); t0 = p['t']
    pa = np.broadcast_to(np.asarray(-1 if part is None else part), (len(X),)); th = np.full(len(X), t0)
    for k, v in (p.get('t_part') or {}).items(): th = np.where(pa == int(k), v, th)
    for z in p['thin']:
        c_ = np.asarray(z['c'], float)
        if 'c2' in z:                                                                  # capsule: distance to the segment c - c2
            ab = np.asarray(z['c2'], float) - c_; u_ = np.clip(((X - c_) @ ab) / max(ab @ ab, 1e-30), 0, 1); dd = np.linalg.norm(X - c_ - u_[:, None] * ab, axis=1)
        else: dd = np.linalg.norm(X - c_, axis=1)
        tz = z['t']; rz = z.get('r', 0.008); taper = z.get('taper', 3 * max(t0 - tz, 0.0005) + 0.002)
        m = (pa == z['part']) if 'part' in z else np.ones(len(X), bool)
        th = np.where(m, np.minimum(th, tz + (th - tz) * SS((dd - rz) / taper)), th)
    return th


def vpart_of(F, wf, part, pa, nv):
    m = np.zeros(nv, bool); m[np.unique(F[wf & (part == pa)])] = True; return m


def check(W, log=print, sample=4):
    """Closure (boundary / non-manifold edges per part), wall thickness measured from the inner vertices to the outer skin, inner vertices
    outside the outer skin (poke-through), folded inner faces."""
    V, F, side, part, exc = W.V, W.F, W.side, W.part, W.exclude; rep = {}
    for pa in sorted(set(part[~exc].tolist())):
        m = (part == pa) & ~exc; Fs = F[m]; e = np.concatenate([Fs[:, [0, 1]], Fs[:, [1, 2]], Fs[:, [2, 0]]]).astype(np.int64)
        key = np.minimum(e[:, 0], e[:, 1]) * len(V) + np.maximum(e[:, 0], e[:, 1]); _, c = np.unique(key, return_counts=True)
        mo = m & (side == 0); sd = SurfDist(V, F[mo]); iv = np.unique(F[m & (side == 1)]); iv = iv[::sample]
        d, _ = sd(V[iv], upper=0.02); th_nom = W.th[iv]; inside = d < 0; dd = -d
        fo = F[m & (side == 1)]; Pi = V[fo]; ni = np.cross(Pi[:, 1] - Pi[:, 0], Pi[:, 2] - Pi[:, 0])
        Po = V[W.src_v[fo]]; no = np.cross(Po[:, 1] - Po[:, 0], Po[:, 2] - Po[:, 0])           # the outer face it was copied from (same corner order)
        fold = int(((ni * no).sum(1) < 0).sum())                                               # reversed copy turned over = folded
        rep[int(pa)] = dict(boundary_edges=int((c == 1).sum()), nonmanifold_edges=int((c > 2).sum()), inner_outside_outer=int((~inside).sum()),
                            wall_min_mm=round(float(dd[inside].min()) * 1000, 2), wall_p1_mm=round(float(np.percentile(dd[inside], 1)) * 1000, 2),
                            wall_median_mm=round(float(np.median(dd[inside])) * 1000, 2), wall_max_mm=round(float(dd[inside].max()) * 1000, 2),
                            thin_below_60pct=int((dd[inside] < 0.6 * th_nom[inside]).sum()), folded_inner_faces=fold, sampled=int(len(iv)))
        log('wall part %d: %s' % (pa, rep[int(pa)]))
    return rep
