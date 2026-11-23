# Parametric upper body for the mower redesign.
# usage: python design_gen.py PARAMS.json OUT.npz      (run inside the design/ folder)
# The lower skirt, bumper and front face with the charger openings (keep.npz) stay as scanned; this script
# generates everything above the belt line as ONE continuous surface: outline at the belt -> walls -> roof.
# Wheels are fully exposed: between x=-0.03 and x=0.262 the body is pinched in to |y| <= y_pinch.
# The lid is a region of that same roof surface (perfectly flush); only a thin seam ribbon marks it.
import numpy as np, json, sys
from scipy.interpolate import PchipInterpolator, LinearNDInterpolator
from shapely.geometry import Polygon, Point
import shapely
from shapely import affinity
P = json.load(open(sys.argv[1])); OUT = sys.argv[2]
K = json.load(open('constraints.json'))
g = lambda k, d: P.get(k, d)
ss = lambda a, b, x: np.clip((x - a) / (b - a), 0, 1) ** 2 * (3 - 2 * np.clip((x - a) / (b - a), 0, 1))
zcut = lambda x: 0.135 - 0.012 * ss(0.20, 0.25, x)

# ---- belt outline (left half, nose -> tail), symmetric
xs = np.array(K['xs']); yl = np.array([np.nan if v is None else v for v in K['yl']]); yr = np.array([np.nan if v is None else -v for v in K['yr']])
yo = np.nanmean(np.c_[yl, yr], 1)
import tail as _tail; yo = np.where(np.isfinite(yo), _tail.make(P)(xs, np.nan_to_num(yo)), yo)      # slim tail (optional)
front = [(x, y) for x, y in zip(xs, yo) if x >= 0.268 and np.isfinite(y)]
rear = [(x, y) for x, y in zip(xs, yo) if x <= -0.034 and np.isfinite(y)]
yp = g('y_pinch', 0.166); zw = g('z_wheel_edge', None); LIP = 0.0012
def outline(pinched):
    yf = front[0][1] + LIP; yr_ = rear[-1][1] + LIP
    zw_f = zw if zw is not None else zcut(0.262) - 0.003; zw_r = zw if zw is not None else zcut(-0.03) - 0.003
    mid = [(0.262, yp, zw_f), (-0.030, yp, zw_r)] if pinched else [(0.262, yf, zcut(0.262) - 0.003), (-0.030, yr_, zcut(-0.03) - 0.003)]
    pts = [(K['x_nose'] + LIP, 0.0, zcut(0.31) - 0.003)] + [(x + LIP * 0.5, y + LIP, zcut(x) - 0.003) for x, y in front[::-1]] + mid
    pts += [(x, y + LIP, zcut(x) - 0.003) for x, y in rear[::-1]] + [(K['x_tail'] - LIP, 0.0, zcut(-0.31) - 0.003)]
    return np.array(pts)
ptsW = outline(False); ptsP = outline(True)
ptsP[:, 1] = np.minimum(ptsP[:, 1], yp)          # the pinched body is narrow along its whole length; the blend decides where it shows
seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ptsW[:, :2], axis=0), axis=1))]      # one parameterisation for both outlines
NH = int(g('n_outline', 640)); sres = np.linspace(0, seg[-1], NH)
def resample(pts):
    B = np.c_[np.interp(sres, seg, pts[:, 0]), np.interp(sres, seg, pts[:, 1]), np.interp(sres, seg, pts[:, 2])]
    for _ in range(int(g('outline_smooth', 6))):
        Bs = B.copy(); Bs[1:-1] = 0.25 * B[:-2] + 0.5 * B[1:-1] + 0.25 * B[2:]
        lock = (np.arange(NH) < 2) | (np.arange(NH) > NH - 3); B = np.where(lock[:, None], B, Bs)
    return B
BP = resample(ptsP); BW = resample(ptsW); B = BW

# ---- roof height along x, spine, wall/roof profile
rk = np.array(sorted(g('roof', [[-0.32, 0.262], [-0.18, 0.272], [0.0, 0.274], [0.2, 0.258], [0.31, 0.225]])))
Zr = PchipInterpolator(rk[:, 0], rk[:, 1], extrapolate=True)
kx = g('spine_scale', 0.55); xc = g('spine_center', -0.02)
prof = np.array(sorted(g('profile', [[0, 0, 0], [0.22, 0.05, 0.55], [0.4, 0.3, 0.93], [1, 1, 1]])))   # [t, q(plan inward), p(height)]
mode = g('profile_mode', 'smooth')
if mode == 'smooth':
    qf = PchipInterpolator(prof[:, 0], prof[:, 1]); pf = PchipInterpolator(prof[:, 0], prof[:, 2])
else:
    qf = lambda t: np.interp(t, prof[:, 0], prof[:, 1]); pf = lambda t: np.interp(t, prof[:, 0], prof[:, 2])
MR = int(g('n_rows', 170))
tt = np.unique(np.r_[np.linspace(0, 1, MR), prof[:, 0]]); MR = len(tt)
T = np.c_[xc + (B[:, 0] - xc) * kx, np.zeros(NH)]
crown = g('crown', 0.006)
def surf(B, sign):
    V = np.zeros((MR, NH, 3))
    for i, t in enumerate(tt):
        q = float(qf(t)); p = float(pf(t))
        x = B[:, 0] + (T[:, 0] - B[:, 0]) * q; y = (B[:, 1] + (T[:, 1] - B[:, 1]) * q) * sign
        zr = Zr(x) - crown * (np.abs(y) / 0.2) ** 2
        V[i, :, 0] = x; V[i, :, 1] = y; V[i, :, 2] = B[:, 2] + (zr - B[:, 2]) * p
    return V
# Waist: the full-width body and the body pinched in beside the wheels are two complete surfaces; they are blended
# along raked S-curves (no square notch). Behind the wheel the blend starts further back the higher up it is.
zb0 = 0.135
sm5 = lambda a, b, x: (lambda u: u * u * u * (u * (u * 6 - 15) + 10))(np.clip((x - a) / (b - a), 0, 1))
def half(sign):
    Vw = surf(BW, sign); Vp = surf(BP, sign); x = Vw[..., 0]; z = Vw[..., 2]; h = np.clip(z - zb0, 0, 1)
    xr1 = -0.030 - g('rear_rake', 0.45) * h; Lr = g('rear_blend', 0.03) + g('rear_sweep', 0.9) * h
    xf0 = 0.262 + g('front_rake', 0.25) * h; Lf = g('front_blend', 0.02) + g('front_sweep', 0.5) * h
    w = np.maximum(1 - sm5(xr1 - Lr, xr1, x), sm5(xf0, xf0 + Lf, x))
    return Vp + w[..., None] * (Vw - Vp)
sm5 = lambda a, b, x: (lambda u: u * u * u * (u * (u * 6 - 15) + 10))(np.clip((x - a) / (b - a), 0, 1))
TT = np.repeat(tt[:, None], NH, 1)
def sculpt(V):
    """Soft sculpting: smooth bulges (amp > 0, e.g. a haunch over the rear) or hollows (amp < 0, e.g. a light-catching
    scoop) on the side surface. Gaussian in x and in t; 'rake' slants the feature (x centre moves with t)."""
    for sc in g('sculpt', []):
        xcn = sc['x'] + sc.get('rake', 0.0) * (TT - sc.get('t', 0.2))
        d = sc['amp'] * np.exp(-((V[..., 0] - xcn) / sc.get('sx', 0.08)) ** 2) * np.exp(-((TT - sc.get('t', 0.2)) / sc.get('st', 0.12)) ** 2)
        V[..., 1] += d; V[..., 2] += d * sc.get('lift', 0.0)
    return V
def fair(V, n):
    """Taubin fairing of the whole surface (belt edge fixed, mirror symmetry at the centre line and at nose/tail):
    removes every crease while keeping the volume, like the continuous surfacing of a car body."""
    M_ = np.array([1.0, -1.0, 1.0])
    def lap(V):
        L = np.concatenate([V[:, 1:2] * M_, V[:, :-1]], 1); R = np.concatenate([V[:, 1:], V[:, -2:-1] * M_], 1)
        U = np.concatenate([V[:1], V[:-1]], 0); D = np.concatenate([V[1:], V[-2:-1] * M_], 0)
        return 0.25 * (L + R + U + D) - V
    for i in range(n):
        for lam in (0.5, -0.52):
            d = lap(V); d[0] = 0; V = V + lam * d
    return V
VL = fair(sculpt(half(1)), int(g('fair', 60)))
def ridges(V):
    """Character lines running front to rear over the top (applied after fairing so they survive).
    Each: pts [[x, y], ...] = the line in plan (left half, mirrored); kind "step" = everything inboard of the line is raised
    by amp (a sculpted centre dome bounded by a soft line), "crease" = a raised soft ridge along the line; width = softness."""
    for rd in g('ridges', []):
        pl = np.array(sorted(rd['pts'])); yl_ = PchipInterpolator(pl[:, 0], pl[:, 1], extrapolate=True)(V[..., 0])
        d = V[..., 1] - yl_; w = rd.get('width', 0.012); fade = rd.get('fade', 0.05)
        env = sm5(pl[0, 0], pl[0, 0] + fade, V[..., 0]) * (1 - sm5(pl[-1, 0] - fade, pl[-1, 0], V[..., 0])) * sm5(0.2, 0.4, TT)
        prof_ = 1 / (1 + np.exp(d / (w / 4))) if rd.get('kind', 'step') in ('step', 'deck') else np.exp(-np.abs(d) / w)
        if rd.get('kind') == 'deck':
            # flatten the top: inboard of the line the roof is lifted towards the centre-line height, so the top becomes a
            # calm deck that turns down into the sides along a defined shoulder line (instead of one continuous dome)
            c = np.abs(V[..., 1]) < 0.004; xs = V[..., 0][c]; zs = V[..., 2][c]
            bins = np.linspace(xs.min(), xs.max(), 90); ib = np.clip(np.digitize(xs, bins) - 1, 0, 88)
            zc = np.array([zs[ib == k].max() if (ib == k).any() else np.nan for k in range(89)]); xb = 0.5 * (bins[1:] + bins[:-1])
            ok_ = ~np.isnan(zc); zcx = np.interp(V[..., 0], xb[ok_], zc[ok_])
            V[..., 2] += rd.get('flat', 0.6) * np.clip(zcx - V[..., 2], 0, 0.03) * prof_ * env
            continue
        V[..., 2] += rd.get('amp', 0.004) * prof_ * env
    return V
if g('ridges', []): VL = fair(ridges(VL), int(g('ridge_fair', 3)))
VL[-1, :, 1] = 0.0; VL[:, 0, 1] = np.maximum(VL[:, 0, 1], 0) * 0; VL[:, -1, 1] = 0.0
VR = VL * np.array([1.0, -1.0, 1.0])
def grid_faces(off, flip):
    F = []
    for i in range(MR - 1):
        a = off + i * NH + np.arange(NH - 1); b = a + 1; c = a + NH + 1; d = a + NH
        F += [np.c_[a, b, c], np.c_[a, c, d]] if not flip else [np.c_[a, c, b], np.c_[a, d, c]]
    return np.concatenate(F)
V = np.concatenate([VL.reshape(-1, 3), VR.reshape(-1, 3)])
F = np.concatenate([grid_faces(0, True), grid_faces(MR * NH, False)])
capx = (BW[:, 0] < -0.031) | (BW[:, 0] > 0.263)
gap = np.linalg.norm(VL[0] - BW, axis=1) > 0.0008
ci = np.nonzero(capx[:-1] & capx[1:] & (gap[:-1] | gap[1:]))[0]
nV0 = len(V); capV = np.concatenate([BW, BW * [1, -1, 1]]); V = np.concatenate([V, capV])
cl = np.c_[ci, ci + 1, nV0 + ci + 1]; cl2 = np.c_[ci, nV0 + ci + 1, nV0 + ci]
cr = np.c_[MR * NH + ci, nV0 + NH + ci + 1, MR * NH + ci + 1]; cr2 = np.c_[MR * NH + ci, nV0 + NH + ci, nV0 + NH + ci + 1]
Fcap = np.concatenate([cl, cl2, cr, cr2]) if len(ci) else np.zeros((0, 3), int)
tv = np.tile(np.repeat(tt, NH), 2); uv = np.tile(np.tile(sres / seg[-1], MR), 2)     # per-vertex t and u (0 nose .. 1 tail)
tv = np.r_[tv, np.zeros(2 * NH)]; uv = np.r_[uv, np.zeros(2 * NH)]
F = np.concatenate([F, Fcap]); ft = tv[F].mean(1); fu = uv[F].mean(1); C = V[F].mean(1)
lab = np.zeros(len(F), np.int8)                                   # 0 body
lab[ft > g('roof_panel_t', 2.0)] = 1                               # 1 roof panel (second tone), off by default
for band in g('bands', []):                                        # wall bands: visor / light bar / accent
    m = (ft >= band['t0']) & (ft <= band['t1']) & (fu >= band.get('u0', 0)) & (fu <= band.get('u1', 1))
    if 'x_min' in band: m &= C[:, 0] >= band['x_min']
    if 'x_max' in band: m &= C[:, 0] <= band['x_max']
    lab[m] = {'visor': 2, 'light': 3, 'accent': 5, 'roof': 1, 'body': 0}[band['kind']]

# ---- front face graphics, drawn in the front view (y, z): lamps, sensor visor, accents; charger ports are real openings
def fshape(sh):
    if 'pts' in sh: pg = Polygon(sh['pts'])
    elif sh.get('shape', 'rrect') == 'ellipse': pg = affinity.scale(Point(sh['cy'], sh['cz']).buffer(1, 48), sh['w'] / 2, sh['h'] / 2)
    else: pg = shapely.box(sh['cy'] - sh['w'] / 2, sh['cz'] - sh['h'] / 2, sh['cy'] + sh['w'] / 2, sh['cz'] + sh['h'] / 2)
    r = min(sh.get('r', 0.004), 0.49 * min(pg.bounds[2] - pg.bounds[0], pg.bounds[3] - pg.bounds[1]))
    pg = pg.buffer(-r).buffer(r, quad_segs=12)
    if sh.get('tilt'): pg = affinity.rotate(pg, sh['tilt'], origin='centroid')
    return pg
frontf = (C[:, 0] > g('front_x_min', 0.17)) & (fu < 0.2)
CRISP = g('crisp_front', True); front_polys = []
for sh in g('front_shapes', []):
    pg = fshape(sh)
    for sgn in ([1, -1] if sh.get('mirror', True) else [1]):
        if CRISP: front_polys.append((affinity.scale(pg, sgn, 1, origin=(0, 0)) if sgn < 0 else pg, sh['kind'])); continue
        m = frontf & shapely.contains_xy(pg, C[:, 1] * sgn, C[:, 2])
        lab[m] = {'visor': 2, 'light': 3, 'accent': 5, 'roof': 1, 'body': 0}[sh['kind']]
ports = shapely.union_all([shapely.box(-0.0521, 0.1341, -0.0087, 0.1581), shapely.box(0.0005, 0.1341, 0.0435, 0.1581)]).buffer(-0.003).buffer(0.003 + g('port_margin', 0.0015), quad_segs=10)
hole = (C[:, 0] > 0.2) & shapely.contains_xy(ports, C[:, 1], C[:, 2])
F = F[~hole]; lab = lab[~hole]; ft = ft[~hole]; fu = fu[~hole]; C = C[~hole]
# crisp front graphics: each shape is its own finely triangulated decal laid on the nose (smooth outlines at any mesh size)
from scipy.spatial import Delaunay
from scipy.ndimage import maximum_filter as _mx, gaussian_filter as _gf
fv = V[(V[:, 0] > 0.15)]; pxm = 0.001; y0m, z0m = -0.26, 0.10
Xm = np.full((int(0.52 / pxm), int(0.20 / pxm)), -1.0)
iy = ((fv[:, 1] - y0m) / pxm).astype(int); iz = ((fv[:, 2] - z0m) / pxm).astype(int); okm = (iy >= 0) & (iy < Xm.shape[0]) & (iz >= 0) & (iz < Xm.shape[1])
np.maximum.at(Xm, (iy[okm], iz[okm]), fv[okm, 0]); Xm = _mx(Xm, 5); Xs = _gf(np.where(Xm > 0, Xm, 0.0), 1.2) / np.maximum(_gf((Xm > 0).astype(float), 1.2), 1e-6)
def xfront(y, z):
    a = np.clip((y - y0m) / pxm - 0.5, 0, Xs.shape[0] - 1.001); b = np.clip((z - z0m) / pxm - 0.5, 0, Xs.shape[1] - 1.001)
    i = a.astype(int); j = b.astype(int); fa = a - i; fb = b - j
    return Xs[i, j] * (1 - fa) * (1 - fb) + Xs[i + 1, j] * fa * (1 - fb) + Xs[i, j + 1] * (1 - fa) * fb + Xs[i + 1, j + 1] * fa * fb
DV, DF, DL = [], [], []; doff = 0; KL = {'body': 0, 'roof': 1, 'visor': 2, 'light': 3, 'accent': 5}
for k_, (pg, kind) in enumerate(front_polys):
    pg = pg.difference(ports.buffer(0.0006))
    for part in (pg.geoms if hasattr(pg, 'geoms') else [pg]):
        if part.is_empty or part.area < 2e-7: continue
        rings = [part.exterior] + list(part.interiors)
        bp = np.concatenate([shapely.get_coordinates(shapely.line_interpolate_point(r_, np.arange(0, r_.length, 0.0006))) for r_ in rings])
        b = part.bounds; gy, gz = np.meshgrid(np.arange(b[0], b[2], 0.0015), np.arange(b[1], b[3], 0.0015)); ip = np.c_[gy.ravel(), gz.ravel()]
        inner = part.buffer(-0.0007); ip = ip[shapely.contains_xy(inner, ip[:, 0], ip[:, 1])] if not inner.is_empty else ip[:0]
        pts2 = np.concatenate([bp, ip]); tri = Delaunay(pts2).simplices; cen = pts2[tri].mean(1)
        tri = tri[shapely.contains_xy(part, cen[:, 0], cen[:, 1])]
        xv = xfront(pts2[:, 0], pts2[:, 1]) + 0.0003 + 0.00008 * k_
        DV.append(np.c_[xv, pts2]); DF.append(tri + doff); DL.append(np.full(len(tri), KL[kind])); doff += len(pts2)
# charger ports: dark sleeves running back from the two openings, so no internals show
SV, SF = [], []; soff = 0
for part in ports.geoms:
    rr = shapely.get_coordinates(shapely.line_interpolate_point(part.exterior, np.arange(0, part.exterior.length, 0.0008))); n_ = len(rr)
    xf_ = xfront(rr[:, 0], rr[:, 1]) + 0.0002; xb_ = np.full(n_, xf_.min() - 0.03); cy_, cz_ = rr.mean(0)
    SV.append(np.concatenate([np.c_[xf_, rr], np.c_[xb_, rr], [[xb_[0], cy_, cz_]]])); i_ = np.arange(n_); j_ = (i_ + 1) % n_
    SF.append(np.concatenate([np.c_[i_, j_, j_ + n_], np.c_[i_, j_ + n_, i_ + n_], np.c_[i_ + n_, j_ + n_, np.full(n_, 2 * n_)]]) + soff); soff += 2 * n_ + 1
extra = {'sleeve_v': np.concatenate(SV), 'sleeve_f': np.concatenate(SF)}
if DV: extra.update(decal_v=np.concatenate(DV), decal_f=np.concatenate(DF), decal_lab=np.concatenate(DL))

# ---- lid: a region of the roof surface
L_ = g('lid', {'shape': 'rrect', 'cx': -0.047, 'length': 0.2, 'width': 0.15, 'r': 0.025})
cx, ln, wd = L_['cx'], L_['length'], L_['width']; hx, hy = ln / 2, wd / 2
if L_['shape'] == 'rrect':
    poly = shapely.box(cx - hx, -hy, cx + hx, hy)
elif L_['shape'] == 'hex':
    k = L_.get('cut', 0.035); poly = Polygon([(cx - hx, -hy + k), (cx - hx + k, -hy), (cx + hx - k, -hy), (cx + hx, -hy + k), (cx + hx, hy - k), (cx + hx - k, hy), (cx - hx + k, hy), (cx - hx, hy - k)])
elif L_['shape'] == 'trapezoid':
    wf = L_.get('width_front', wd * 0.7) / 2; poly = Polygon([(cx - hx, -hy), (cx + hx, -wf), (cx + hx, wf), (cx - hx, hy)])
elif L_['shape'] == 'shield':
    k = L_.get('cut', 0.05); poly = Polygon([(cx - hx, -hy), (cx + hx - k, -hy), (cx + hx, 0), (cx + hx - k, hy), (cx - hx, hy)])
else:
    poly = affinity.scale(Point(cx, 0).buffer(1, 48), hx, hy)
r = L_.get('r', 0.02); poly = poly.buffer(-r).buffer(r, quad_segs=16)
inlid = shapely.contains_xy(poly, C[:, 0], C[:, 1]) & (C[:, 2] > 0.2) & (ft > 0.3)
lab[inlid] = 4
roofv = tv > 0.25
zi = LinearNDInterpolator(V[roofv][:, :2], V[roofv][:, 2])
def ribbon(xy, w, lift=0.0004, closed=True):
    xy = np.asarray(xy); nx = np.roll(xy, -1, 0) - np.roll(xy, 1, 0) if closed else np.gradient(xy, axis=0)
    n = np.c_[-nx[:, 1], nx[:, 0]]; n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    a = xy + n * w / 2; b = xy - n * w / 2
    va = np.c_[a, zi(a) + lift]; vb = np.c_[b, zi(b) + lift]; n_ = len(xy)
    v = np.concatenate([va, vb]); i = np.arange(n_ if closed else n_ - 1); j = (i + 1) % n_
    f = np.concatenate([np.c_[i, j, j + n_], np.c_[i, j + n_, i + n_]])
    ok = np.isfinite(v[:, 2]); v[~ok, 2] = np.nanmax(v[:, 2])
    return v, f
ring = np.array(poly.exterior.coords)[:-1]
dens = shapely.line_interpolate_point(poly.exterior, np.linspace(0, 1, 600, endpoint=False), normalized=True)
seam_v, seam_f = ribbon(shapely.get_coordinates(dens), g('seam_width', 0.0014))
rib = {'seam_v': seam_v, 'seam_f': seam_f}
RV, RF, RL = [], [], []; off = 0
for ln_ in g('lines', []):                                         # extra graphic lines on the roof: light / accent
    pl = np.array(ln_['pts']); d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pl, axis=0), axis=1))]
    q_ = np.linspace(0, d[-1], 200); xy = np.c_[np.interp(q_, d, pl[:, 0]), np.interp(q_, d, pl[:, 1])]
    for sgn in ([1, -1] if ln_.get('mirror', True) else [1]):
        v, f = ribbon(xy * [1, sgn], ln_.get('width', 0.004), closed=False)
        RV.append(v); RF.append(f + off); off += len(v); RL.append(np.full(len(f), 3 if ln_['kind'] == 'light' else 5))
if RV: rib.update(line_v=np.concatenate(RV), line_f=np.concatenate(RF), line_lab=np.concatenate(RL))

# ---- checks
H = np.load('chassis_top.npy')
gx = ((V[:, 0] + 0.34) / 0.005).astype(int); gy = ((V[:, 1] + 0.25) / 0.005).astype(int)
ok = (gx >= 0) & (gx < H.shape[0]) & (gy >= 0) & (gy < H.shape[1]) & (tv > 0.3) & (np.abs(V[:, 1]) < 0.15)
clear = V[ok, 2] - H[gx[ok], gy[ok]]; hit = ok.nonzero()[0][H[gx[ok], gy[ok]] > 0.05]
cl = V[hit, 2] - H[gx[hit], gy[hit]]
WX, WZ, WR = 0.119, 0.1145, 0.116                                  # drive wheel centre / radius (side view)
dx = V[:, 0] - WX; inwheel = (np.abs(dx) < WR + 0.006) & ((V[:, 2] >= WZ) | (dx * dx + (V[:, 2] - WZ) ** 2 < (WR + 0.008) ** 2)) & (V[:, 2] < 0.30)
wz = inwheel & (np.arange(len(V)) < nV0)                         # body surface only (not the unused cap copies)
# surface continuity: curvature between neighbouring quads (belt rows excluded) -> smallest fillet radius on the body
G = VL; qn = np.cross(G[1:, 1:] - G[:-1, :-1], G[:-1, 1:] - G[1:, :-1]); qa = np.linalg.norm(qn, axis=2); qn = qn / np.maximum(qa[..., None], 1e-20)
qc = 0.25 * (G[1:, 1:] + G[:-1, :-1] + G[:-1, 1:] + G[1:, :-1]); okq = qa > 2e-7
def curv(n1, n2, c1, c2, m):
    ang = np.arccos(np.clip(np.sum(n1 * n2, -1), -1, 1)); dist = np.linalg.norm(c1 - c2, axis=-1)
    return np.where(m, ang / np.maximum(dist, 1e-6), 0.0), np.where(m, ang, 0.0)
k1, a1 = curv(qn[3:, 1:], qn[3:, :-1], qc[3:, 1:], qc[3:, :-1], okq[3:, 1:] & okq[3:, :-1])
k2, a2 = curv(qn[4:], qn[3:-1], qc[4:], qc[3:-1], okq[4:] & okq[3:-1])
kk = np.r_[k1.ravel(), k2.ravel()]; rmin = 1.0 / max(np.percentile(kk, 99.9), 1e-6)
crease = float(np.degrees(max(np.percentile(a1, 99.95), np.percentile(a2, 99.95))))
# chassis parts left uncovered above the belt (would show beside / above the wheels)
from scipy.ndimage import maximum_filter
Hb = np.zeros_like(H); gxa = np.clip(((V[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1); gya = np.clip(((V[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)
np.maximum.at(Hb, (gxa, gya), V[:, 2]); Hb = maximum_filter(Hb, 3)
cxs = -0.34 + (np.arange(H.shape[0]) + 0.5) * 0.005; cys = -0.25 + (np.arange(H.shape[1]) + 0.5) * 0.005; CX, CY = np.meshgrid(cxs, cys, indexing='ij')
strut = (np.abs(CY) > 0.158) & (CX > -0.06) & (CX < 0.29)           # mudguard struts: removed with the mudguards
expo = (H > 0.150) & ~strut & (Hb < H + 0.002)
lidv = np.unique(F[lab == 4])
b = poly.bounds
rep = {'faces': int(len(F)), 'roof_min_clear_over_chassis_mm': round(float(cl.min()) * 1000, 1) if len(cl) else None,
       'max_half_width_beside_wheels_mm': round(float(np.abs(V[wz, 1]).max()) * 1000, 1), 'wheel_inner_face_mm': 175.0,
       'lid_min_height_mm': round(float(V[lidv, 2].min()) * 1000, 1), 'controls_top_mm': round(K['ctl_top'] * 1000, 1),
       'lid_covers_controls': bool(b[0] <= -0.128 and b[2] >= 0.034 and b[1] <= -0.062 and b[3] >= 0.062),
       'body_max_height_mm': round(float(V[:, 2].max()) * 1000, 1),
       'min_surface_radius_mm': round(rmin * 1000, 1), 'max_crease_deg': round(crease, 1),
       'chassis_exposed_cm2': round(float(expo.sum()) * 0.25, 1)}
rep['ok'] = bool((rep['roof_min_clear_over_chassis_mm'] or 9) >= 2.0 and rep['max_half_width_beside_wheels_mm'] <= 171.0
                 and rep['lid_min_height_mm'] >= rep['controls_top_mm'] + 4 and rep['lid_covers_controls']
                 and rep['min_surface_radius_mm'] >= (3.0 if g('ridges', []) else 8.0) and rep['chassis_exposed_cm2'] <= 2.0)
print(json.dumps(rep))
np.savez(OUT, v=V, f=F.astype(np.int32), lab=lab, lid_poly=ring, **rib, **extra)
