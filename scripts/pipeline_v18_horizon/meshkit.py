# Small numpy-only mesh primitives shared by the generator (venv python) and the renderer (Blender python).
import numpy as np

class Bag:
    """Collects triangle meshes with one material label per part."""
    def __init__(s): s.V = []; s.F = []; s.L = []; s.n = 0
    def add(s, v, f, lab):
        v = np.asarray(v, float).reshape(-1, 3); f = np.asarray(f, np.int64).reshape(-1, 3)
        if len(f) == 0: return
        s.V.append(v); s.F.append(f + s.n); s.L.append(np.full(len(f), lab, np.int8)); s.n += len(v)
    def out(s):
        if not s.V: return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros(0, np.int8)
        return np.concatenate(s.V), np.concatenate(s.F), np.concatenate(s.L)

def quads(a, b, c, d): return np.concatenate([np.c_[a, b, c], np.c_[a, c, d]])

def stack(R, closed=True, cap0=False, cap1=False, skip=None):
    """R: (m, n, 3) rings with the same point count -> strips between successive rings. skip: bool (n,) segments left open."""
    R = np.asarray(R, float); m, n, _ = R.shape; V = R.reshape(-1, 3); F = []
    j = np.arange(n if closed else n - 1); j2 = (j + 1) % n
    if skip is not None: keep = ~np.asarray(skip)[j]; j = j[keep]; j2 = j2[keep]
    for i in range(m - 1): F.append(quads(i * n + j, i * n + j2, (i + 1) * n + j2, (i + 1) * n + j))
    F = np.concatenate(F) if F else np.zeros((0, 3), np.int64)
    for cap, i in ((cap0, 0), (cap1, m - 1)):
        if cap:
            c = len(V); V = np.concatenate([V, R[i].mean(0)[None]]); k = np.arange(n); k2 = (k + 1) % n
            F = np.concatenate([F, np.c_[i * n + k, i * n + k2, np.full(n, c)]])
    return V, F

def frame(a):
    a = np.asarray(a, float); a = a / np.linalg.norm(a); h = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(a, h); u /= np.linalg.norm(u); return a, u, np.cross(a, u)

def lathe(o, a, prof, n=64):
    """Surface of revolution about the axis through o along a. prof: [(h along the axis, radius), ...]"""
    a, u, w = frame(a); q = np.linspace(0, 2 * np.pi, n, endpoint=False); cs = np.cos(q)[:, None] * u + np.sin(q)[:, None] * w
    R = np.array([np.asarray(o, float) + a * h + cs * max(r, 0.0) for h, r in prof])
    return stack(R, closed=True)

def tube(p0, p1, r, n=24, caps=True):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float); L = np.linalg.norm(p1 - p0)
    V, F = lathe(p0, p1 - p0, [(0, 0.0)] * caps + [(0, r), (L, r)] + [(L, 0.0)] * caps, n); return V, F

def ball(c, r, n=20):
    q = np.linspace(-np.pi / 2, np.pi / 2, n // 2 + 1); return lathe(c, (0, 0, 1), [(r * np.sin(t), r * np.cos(t)) for t in q], n)

def box(c, s, rot=None):
    """Axis-aligned box (centre c, size s); rot: optional 3x3 applied about the centre."""
    s = np.asarray(s, float) / 2; k = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float) * s
    if rot is not None: k = k @ np.asarray(rot).T
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return k + np.asarray(c, float), np.array(f)

def sweep(path, sec, up=(0, 1, 0), caps=True):
    """Sweep a closed 2D section (k, 2) along a 3D path (m, 3). Section axes: e1 = up (fixed), e2 = tangent x up."""
    path = np.asarray(path, float); up = np.asarray(up, float); up = up / np.linalg.norm(up)
    tg = np.gradient(path, axis=0); tg /= np.linalg.norm(tg, axis=1, keepdims=True)
    e2 = np.cross(tg, up); e2 /= np.linalg.norm(e2, axis=1, keepdims=True); sec = np.asarray(sec, float)
    R = path[:, None, :] + sec[None, :, 0, None] * up[None, None, :] + sec[None, :, 1, None] * e2[:, None, :]
    return stack(R, closed=True, cap0=caps, cap1=caps)

def rrect(w, h, r, n=5):
    """Rounded rectangle section (centred), counter-clockwise."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for q in np.radians(np.linspace(a0, a0 + 90, n)): pts.append((cx + r * np.cos(q), cy + r * np.sin(q)))
    return np.array(pts)

def roty(deg):
    """Rotation about +Y that lifts +X (front) upwards for positive angles."""
    q = np.radians(deg); c, s = np.cos(q), np.sin(q); return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])

def strut(bag, N, M, labs=(5, 2, 3), body=0.062, rb=0.0032, rr=0.0017):
    """Gas strut from the fixed ball stud N to the lid ball stud M: body at the N end, rod to M, ball sockets at both ends."""
    N = np.asarray(N, float); M = np.asarray(M, float); d = M - N; L = np.linalg.norm(d); d = d / L
    e0 = 0.006                                                   # socket length at each end
    bag.add(*tube(N + d * e0, N + d * (e0 + body), rb, 28), labs[0])
    bag.add(*lathe(N + d * (e0 + body), d, [(0, rb), (0.0012, rb * 0.8), (0.0012, rr)], 28), labs[2])      # end seal
    bag.add(*tube(N + d * (e0 + body), M - d * e0, rr, 20), labs[1])
    for P_, s_ in ((N, 1), (M, -1)):
        bag.add(*lathe(P_, d * s_, [(-0.0036, 0.0), (-0.0036, 0.0026), (-0.002, 0.0038), (0.002, 0.0038), (0.004, 0.0028), (e0 + 0.0005, 0.0024), (e0 + 0.0005, 0.0)], 20), labs[2])
        bag.add(*ball(P_, 0.0027, 14), labs[1])
