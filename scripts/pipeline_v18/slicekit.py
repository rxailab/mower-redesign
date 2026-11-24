# Exact 2D kinematic clearance check by y-slices: the lid hinge axis is parallel to Y, so every plane y = const
# is mapped onto itself by the opening motion. Each mesh is cut into line segments in that plane; moving segments
# are rotated about the pivot; distances / crossings are measured with shapely STRtrees.
import numpy as np, shapely
from shapely import STRtree

def slice_y(V, F, y0):
    """Segments (k, 2, 2) in (x, z) where the triangle mesh (V, F) crosses the plane y = y0."""
    T = V[F]; d = T[:, :, 1] - y0
    s = np.sign(d); s[s == 0] = 1e-9
    hit = (d.min(1) < 0) & (d.max(1) > 0)
    T = T[hit]; d = d[hit]
    out = []
    for (i, j) in ((0, 1), (1, 2), (2, 0)):
        a, b = d[:, i], d[:, j]; m = (a * b) < 0
        t = np.where(m, a / np.where(m, a - b, 1), 0)
        P = T[:, i] + (T[:, j] - T[:, i]) * t[:, None]
        out.append((m, P[:, [0, 2]]))
    seg = []
    m0, p0 = out[0]; m1, p1 = out[1]; m2, p2 = out[2]
    for ma, pa, mb, pb in ((m0, p0, m1, p1), (m1, p1, m2, p2), (m2, p2, m0, p0)):
        k = ma & mb
        seg.append(np.stack([pa[k], pb[k]], 1))
    return np.concatenate(seg) if seg else np.zeros((0, 2, 2))

def rot(seg, piv, deg):
    q = np.radians(deg); c, s = np.cos(q), np.sin(q); p = seg - piv
    return np.stack([piv[0] + c * p[..., 0] - s * p[..., 1], piv[1] + s * p[..., 0] + c * p[..., 1]], -1)

def lines(seg):
    return shapely.linestrings(seg) if len(seg) else np.array([], object)

def clearance(mov, fix):
    """min distance (m) between two segment sets and whether they cross. mov, fix: (k,2,2)."""
    if len(mov) == 0 or len(fix) == 0: return np.inf, False
    gm = lines(mov); gf = lines(fix); tr = STRtree(gf)
    idx = tr.query(gm, predicate='intersects')
    hit = idx.shape[1] > 0
    _, dist = tr.query_nearest(gm, return_distance=True, all_matches=False)
    return float(dist.min()), hit
