# [fix4-hinge2] Closed, consistently oriented solids for the lid-hinge kit (numpy / scipy / shapely only).
#  - clean(pg)              : drop near-duplicate / collinear vertices (the buffer(1e-7) unions of the blade / keel left 1e-7 slivers ->
#                             degenerate faces and invalid end caps in lidmech.extrude)
#  - slab_union(slabs, ...) : ONE closed 2-manifold from piecewise-constant profiles along w (y-slabs of a moulding: the hinge pod with its
#                             channel / slots / housings). Adjacent slabs share their interface: only the symmetric difference of the two
#                             profiles becomes a face there (the old pod was separate touching prisms: non-manifold, internal faces).
#  - orient(V, F)           : make every connected component consistently wound (BFS over manifold edges) and outward (signed volume > 0
#                             for closed components; open ones keep the winding of the majority of their faces).
import sys, numpy as np, shapely
from shapely.geometry import Polygon, MultiPolygon
from shapely.geometry.polygon import orient as _orient
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components, breadth_first_order
from scipy.spatial import cKDTree

TOL = 1e-9


def clean(pg, tol=2e-6):
    """valid polygon without vertices closer than tol and without collinear runs (keeps the shape within tol)"""
    pg = shapely.make_valid(pg) if not pg.is_valid else pg
    parts = [g for g in getattr(pg, 'geoms', [pg]) if g.geom_type == 'Polygon' and g.area > tol * tol * 100]
    out = []
    for g in parts:
        g = g.simplify(tol, preserve_topology=True)
        if g.is_valid and not g.is_empty: out.append(g)
    if not out: return Polygon()
    return max(out, key=lambda g: g.area) if len(out) == 1 else MultiPolygon(out)


def _polys(g):
    if g is None or g.is_empty: return []
    return [p for p in getattr(g, 'geoms', [g]) if p.geom_type == 'Polygon' and p.area > 1e-12]


def _rings(p):
    p = _orient(p, 1.0)
    return [np.asarray(p.exterior.coords)[:-1]] + [np.asarray(r.coords)[:-1] for r in p.interiors]


def _insert(ring, pts):
    """insert the points that lie on the ring's edges (within TOL*1e3) in order along each edge"""
    if not len(pts): return ring
    A = ring; B = np.roll(ring, -1, 0); out = []
    d = B - A; L2 = (d * d).sum(1)
    for i in range(len(A)):
        if L2[i] < 1e-24: out.append(A[i]); continue
        t = ((pts - A[i]) @ d[i]) / L2[i]; q = A[i] + t[:, None] * d[i]; dist = np.linalg.norm(pts - q, axis=1)
        m = (t > 1e-9) & (t < 1 - 1e-9) & (dist < 1e-8)
        out.append(A[i])
        if m.any():
            o = np.argsort(t[m]); out.extend(pts[m][o])
    R = np.array(out)
    keep = np.r_[True, np.linalg.norm(np.diff(R, axis=0), axis=1) > 1e-12]; R = R[keep]
    if np.linalg.norm(R[0] - R[-1]) < 1e-12: R = R[:-1]
    return R


def _lnormals(P):
    t = np.roll(P, -1, 0) - P; t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-15); n1 = np.c_[-t[:, 1], t[:, 0]]; n0 = np.roll(n1, 1, 0)
    m = n0 + n1; m /= np.maximum(np.linalg.norm(m, axis=1, keepdims=True), 1e-15); c = np.clip((m * n1).sum(1), 0.5, None); return m / c[:, None]


def _tri(pg):
    """constrained Delaunay of a polygon (holes allowed), triangles CCW in (u, v): (points, tris)"""
    T = shapely.get_parts(shapely.constrained_delaunay_triangles(pg))
    if not len(T): return np.zeros((0, 2)), np.zeros((0, 3), np.int64)
    C = shapely.get_coordinates(shapely.get_exterior_ring(T)).reshape(-1, 4, 2)[:, :3]
    a = (C[:, 1, 0] - C[:, 0, 0]) * (C[:, 2, 1] - C[:, 0, 1]) - (C[:, 1, 1] - C[:, 0, 1]) * (C[:, 2, 0] - C[:, 0, 0])
    C = np.where((a < 0)[:, None, None], C[:, ::-1], C); C = C[np.abs(a) > 1e-22]
    pts, inv = np.unique(C.reshape(-1, 2), axis=0, return_inverse=True); return pts, inv.reshape(-1, 3)


def _tri_robust(rings):
    """triangulate a polygon given as rings that may carry extra collinear vertices (they must all be used, so the neighbouring walls
    meet it without T-junctions). GEOS' triangulator can fail on such rings: then triangulate without the collinear points and split
    the boundary triangles at them (fan from the opposite corner)."""
    try: return _tri(Polygon(rings[0], rings[1:]))
    except Exception: pass
    red, extra = [], []
    for R in rings:
        a = R - np.roll(R, 1, 0); b = np.roll(R, -1, 0) - R
        cr = np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]); col = (cr < 1e-9 * np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1)) & ((a * b).sum(1) > 0)
        red.append(R[~col]); extra.append(R[col])
    try: pts, tri = _tri(Polygon(red[0], red[1:]))
    except Exception:
        import os, pickle
        if os.environ.get('PK_DUMP'): pickle.dump(rings, open(os.environ['PK_DUMP'], 'wb'))
        raise
    X = np.concatenate(extra)
    if not len(X): return pts, tri
    out = []; stack_ = [pts[t] for t in tri]
    while stack_:
        T = stack_.pop(); done = True
        for i in range(3):
            A, B, Cc = T[i], T[(i + 1) % 3], T[(i + 2) % 3]; d = B - A; L2 = d @ d
            t = ((X - A) @ d) / L2; q = A + t[:, None] * d; m = (t > 1e-9) & (t < 1 - 1e-9) & (np.linalg.norm(X - q, axis=1) < 1e-8)
            if m.any():
                P = np.r_[[A], X[m][np.argsort(t[m])], [B]]
                stack_.extend([np.array([P[j], P[j + 1], Cc]) for j in range(len(P) - 1)]); done = False; break
        if done: out.append(T)
    C = np.array(out); pts, inv = np.unique(C.reshape(-1, 2), axis=0, return_inverse=True); return pts, inv.reshape(-1, 3)


def slab_union(slabs, frame, bev=0.0, nb=3, ends=(True, True)):
    """slabs: [(polygon|multipolygon, w0, w1), ...] along w (sorted, touching slabs form one solid). frame = (origin, eu, ev, ew).
    bev: rounded edge radius at the two outer ends of every chain of touching slabs (ends: which ends). Returns V, F (closed, oriented)."""
    o, eu, ev, ew = [np.asarray(a, float) for a in frame]
    to3 = lambda uv, w: o + uv[:, :1] * eu + uv[:, 1:2] * ev + w * ew
    S = [(shapely.union_all(_polys(g)) if _polys(g) else Polygon(), float(w0), float(w1)) for g, w0, w1 in sorted(slabs, key=lambda s: s[1])]
    S = [s for s in S if not s[0].is_empty and s[2] - s[1] > 1e-9]
    chains, cur = [], []
    for s in S:
        if cur and abs(cur[-1][2] - s[1]) < 1e-9: cur.append(s)
        else:
            if cur: chains.append(cur)
            cur = [s]
    if cur: chains.append(cur)
    VV, FF = [], []
    def add(v, f): VV.append(v); FF.append(f + sum(len(x) for x in VV[:-1]))
    for ch in chains:
        n = len(ch)
        # interface regions and their vertices
        iface = []
        for k in range(1, n):
            A, B = ch[k - 1][0], ch[k][0]
            iface.append((A.difference(B), B.difference(A)))
        P = []                                                                     # every vertex of every profile / interface region of the chain
        for g in [c_[0] for c_ in ch] + [g_ for pr_ in iface for g_ in pr_]:      # (a point inserted in a slab's ring for one interface must
            for p in _polys(g):                                                    #  also be in its neighbour's ring at the other interface)
                for r in _rings(p): P.append(r)
        allp = np.unique(np.round(np.concatenate(P), 15), axis=0) if P else np.zeros((0, 2)); evp = [allp] * n
        for k, (pg, w0, w1) in enumerate(ch):
            bv0, bv1 = (bev if isinstance(bev, (tuple, list)) else (bev, bev))      # bev: one radius or (start, end) of the chain along w
            b0 = bv0 if (k == 0 and ends[0]) else 0.0; b1 = bv1 if (k == n - 1 and ends[1]) else 0.0
            if b0 + b1 > 0.9 * (w1 - w0): f_ = 0.9 * (w1 - w0) / (b0 + b1); b0, b1 = b0 * f_, b1 * f_      # the round stays inside its slab
            ph = np.radians(np.linspace(0, 90, nb + 1))
            prof = [(w0 + b0 * (1 - np.cos(q)), b0 * (1 - np.sin(q))) for q in (ph if b0 > 0 else [np.pi / 2])]
            prof += [(w1 - b1 * (1 - np.cos(q)), b1 * (1 - np.sin(q))) for q in (ph[::-1] if b1 > 0 else [np.pi / 2])]
            for p in _polys(pg):
                rings = [_insert(r, evp[k]) for r in _rings(p)]
                caps0, caps1 = [], []
                NN = [_lnormals(R) for R in rings]; bm = max(b0, b1); sc = 1.0      # edge-round depth: scaled down until the inset rings stay a
                if bm > 0:                                                         # valid polygon (no fold at concave corners tighter than it)
                    for sc in (1.0, 0.7, 0.5, 0.35, 0.25, 0.15, 0.08, 0.0):
                        if sc == 0.0: break
                        try:
                            okd_ = all(((np.roll(R, -1, 0) - R) * (np.roll(R + N * bm * sc, -1, 0) - (R + N * bm * sc))).sum(1).min() > 0 for R, N in zip(rings, NN))      # no edge reversed
                            if okd_ and Polygon(rings[0] + NN[0] * bm * sc, [R + N * bm * sc for R, N in zip(rings[1:], NN[1:])]).is_valid: break
                        except Exception: pass
                for R, N in zip(rings, NN):
                    N = N * sc; Rs = [to3(R + N * ins, w) for w, ins in prof]
                    m = len(R); idx = np.arange(m); j2 = (idx + 1) % m; F = []
                    for i in range(len(Rs) - 1):
                        a0, a1, c1, c0 = i * m + idx, i * m + j2, (i + 1) * m + j2, (i + 1) * m + idx
                        F.append(np.c_[a0, a1, c1]); F.append(np.c_[a0, c1, c0])
                    add(np.concatenate(Rs), np.concatenate(F))
                    caps0.append(R + N * prof[0][1]); caps1.append(R + N * prof[-1][1])
                for capr, w, up in ((caps0, prof[0][0], False), (caps1, prof[-1][0], True)):
                    if (not up and k > 0) or (up and k < n - 1): continue          # interior interface: handled below
                    try: pts, tri = _tri_robust(capr)
                    except Exception:                                              # the inset ring folds at a concave corner tighter than the
                        P0_, P1_ = np.concatenate(rings), np.concatenate(capr)     # bevel: triangulate the un-inset ring and move its vertices
                        pts0_, tri = _tri_robust(rings); pts = P1_[cKDTree(P0_).query(pts0_)[1]]      # (same topology: the solid stays closed)
                    if len(tri): add(to3(pts, w), tri if up else tri[:, ::-1])
        for k in range(1, n):
            w = ch[k][1]; AmB, BmA = iface[k - 1]
            for g, up in ((AmB, True), (BmA, False)):
                for q in _polys(g):
                    pts, tri = _tri_robust([_insert(r, allp) for r in _rings(q)])      # the walls' extra vertices on its edges too
                    if len(tri): add(to3(pts, w), tri if up else tri[:, ::-1])
    if not VV: return np.zeros((0, 3)), np.zeros((0, 3), np.int64)
    V = np.concatenate(VV); F = np.concatenate(FF)
    key = np.round(V / 1e-10).astype(np.int64); _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    V = V[first]; F = inv.ravel()[F]; F = F[(F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])]
    if np.linalg.det(np.c_[eu, ev, ew]) < 0: F = F[:, ::-1]                    # left-handed frame (XZ profile along +y): flip
    return V, F


def orient(V, F, weld_tol=1e-10):
    """consistent winding per connected component (BFS over edges shared by exactly two faces), outward by signed volume when closed"""
    F = np.asarray(F, np.int64)
    if not len(F): return V, F
    key = np.round(np.asarray(V) / weld_tol).astype(np.int64); _, inv = np.unique(key, axis=0, return_inverse=True); W = inv.ravel()[F]
    nf = len(F); e = np.concatenate([W[:, [0, 1]], W[:, [1, 2]], W[:, [2, 0]]]); fid = np.tile(np.arange(nf), 3)
    es = np.sort(e, 1); nv = int(W.max()) + 1; k = es[:, 0] * nv + es[:, 1]; o = np.argsort(k, kind='stable'); ks = k[o]
    st = np.r_[True, ks[1:] != ks[:-1]]; grp = np.cumsum(st) - 1; cnt = np.bincount(grp)
    two = cnt[grp] == 2; first = np.where(st)[0]
    a = o[first[grp[two]]]; b = o[two]; pr = a != b; a, b = a[pr], b[pr]
    fa, fb = fid[a], fid[b]; same = (e[a, 0] == e[b, 0])                          # same direction -> one of them must flip
    G = coo_matrix((np.ones(len(fa)), (fa, fb)), shape=(nf, nf)).tocsr(); G = G + G.T
    pk = np.minimum(fa, fb) * nf + np.maximum(fa, fb); po = np.argsort(pk); pk = pk[po]; ps = same[po]
    ncomp, comp = connected_components(G, directed=False)
    roots = np.unique(comp, return_index=True)[1]
    PRED = -np.ones(nf, np.int64)
    for r in roots:                                                               # BFS tree per component (scipy), predecessors only
        _, pred = breadth_first_order(G, int(r), directed=False, return_predecessors=True)
        m = pred >= 0; PRED[m] = pred[m]
    s = np.zeros(nf, bool); m = PRED >= 0; q = np.minimum(np.where(m)[0], PRED[m]) * nf + np.maximum(np.where(m)[0], PRED[m])
    s[m] = ps[np.clip(np.searchsorted(pk, q), 0, len(pk) - 1)]
    val = s.copy(); ptr = PRED.copy()
    while (ptr >= 0).any():                                                       # XOR of the parities along the tree path (pointer doubling)
        h = ptr >= 0; val = np.where(h, val ^ val[np.maximum(ptr, 0)], val); ptr = np.where(h, ptr[np.maximum(ptr, 0)], -1)
    flip = val
    F2 = np.where(flip[:, None], F[:, ::-1], F)
    Vv = np.asarray(V, float); T = Vv[F2]; fv = np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2]))
    vol = np.bincount(comp, fv, ncomp)
    ecnt = np.zeros(len(e), np.int64); ecnt[o] = cnt[grp]                       # faces on each (undirected) edge, per half-edge
    bad = np.bincount(comp[fid], (ecnt != 2).astype(float), ncomp) > 0
    fl = np.bincount(comp, flip.astype(float), ncomp) / np.maximum(np.bincount(comp, minlength=ncomp), 1)
    neg = np.where(bad, fl > 0.5, vol < 0)
    F2 = np.where(neg[comp][:, None], F2[:, ::-1], F2)
    return V, F2


def weld_clean(V, F, tol=1e-10):
    """weld coincident vertices and drop the faces that collapse (lathe poles -> proper fans): closed lathes / sweeps stay closed"""
    V = np.asarray(V, float); F = np.asarray(F, np.int64)
    _, first, inv = np.unique(np.round(V / tol).astype(np.int64), axis=0, return_index=True, return_inverse=True); W = inv.ravel()[F]
    W = W[(W[:, 0] != W[:, 1]) & (W[:, 1] != W[:, 2]) & (W[:, 0] != W[:, 2])]
    return V[first], W


def defold(P, X, iters=50):
    """offset X (n, 2) of the ring P (n, 2), same vertex count: where an offset segment runs against its source segment (a fold at a bend
    tighter than the offset), its end points move to the mean of their neighbours until no segment is reversed (the offset stays simple)"""
    X = np.array(X, float)
    for _ in range(iters):
        dP = np.roll(P, -1, 0) - P; dX = np.roll(X, -1, 0) - X
        bad = np.where((dP * dX).sum(1) <= 0)[0]
        if not len(bad): break
        idx = np.unique(np.r_[bad, (bad + 1) % len(X)]); X[idx] = 0.5 * (np.roll(X, 1, 0)[idx] + np.roll(X, -1, 0)[idx])
    return X
