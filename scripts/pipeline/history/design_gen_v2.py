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
zcut = lambda x: 0.135 + 0.025 * ss(0.20, 0.25, x)

# ---- belt outline (left half, nose -> tail), symmetric
xs = np.array(K['xs']); yl = np.array([np.nan if v is None else v for v in K['yl']]); yr = np.array([np.nan if v is None else -v for v in K['yr']])
yo = np.nanmean(np.c_[yl, yr], 1)
front = [(x, y) for x, y in zip(xs, yo) if x >= 0.268 and np.isfinite(y)]
rear = [(x, y) for x, y in zip(xs, yo) if x <= -0.034 and np.isfinite(y)]
yp = g('y_pinch', 0.166); zw = g('z_wheel_edge', 0.105); LIP = 0.0012
def outline(pinched):
    yf = front[0][1] + LIP; yr_ = rear[-1][1] + LIP
    mid = [(0.262, yp, zw), (-0.030, yp, zw)] if pinched else [(0.262, yf, zcut(0.262) - 0.003), (-0.030, yr_, zcut(-0.03) - 0.003)]
    pts = [(K['x_nose'] + LIP, 0.0, zcut(0.31) - 0.003)] + [(x + LIP * 0.5, y + LIP, zcut(x) - 0.003) for x, y in front[::-1]] + mid
    pts += [(x, y + LIP, zcut(x) - 0.003) for x, y in rear[::-1]] + [(K['x_tail'] - LIP, 0.0, zcut(-0.31) - 0.003)]
    return np.array(pts)
ptsW = outline(False); ptsP = outline(True)
ptsP[:, 1] = np.minimum(ptsP[:, 1], yp)          # the pinched body is narrow along its whole length; the blend decides where it shows
seg = np.r_[0, np.cumsum(np.linalg.norm(np.diff(ptsW[:, :2], axis=0), axis=1))]      # one parameterisation for both outlines
NH = int(g('n_outline', 420)); sres = np.linspace(0, seg[-1], NH)
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
MR = int(g('n_rows', 110))
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
def half(sign):
    Vw = surf(BW, sign); Vp = surf(BP, sign); x = Vw[..., 0]; z = Vw[..., 2]; h = np.clip(z - zb0, 0, 1)
    xr1 = -0.030 - g('rear_rake', 0.45) * h; Lr = g('rear_blend', 0.014) + g('rear_sweep', 0.75) * h
    xf0 = 0.262 + g('front_rake', 0.25) * h; Lf = g('front_blend', 0.012) + g('front_sweep', 0.45) * h
    w = np.maximum(1 - ss(xr1 - Lr, xr1, x), ss(xf0, xf0 + Lf, x))
    w = np.where(z < zb0 - 0.004, np.where((x > -0.031) & (x < 0.263), 0.0, 1.0), w)       # below the belt: plain wheel opening
    return Vp + w[..., None] * (Vw - Vp)
VL = half(1); VR = half(-1)
def grid_faces(off, flip):
    F = []
    for i in range(MR - 1):
        a = off + i * NH + np.arange(NH - 1); b = a + 1; c = a + NH + 1; d = a + NH
        F += [np.c_[a, b, c], np.c_[a, c, d]] if not flip else [np.c_[a, c, b], np.c_[a, d, c]]
    return np.concatenate(F)
V = np.concatenate([VL.reshape(-1, 3), VR.reshape(-1, 3)])
F = np.concatenate([grid_faces(0, True), grid_faces(MR * NH, False)])
tv = np.tile(np.repeat(tt, NH), 2); uv = np.tile(np.tile(sres / seg[-1], MR), 2)     # per-vertex t and u (0 nose .. 1 tail)
ft = tv[F].mean(1); fu = uv[F].mean(1); C = V[F].mean(1)
lab = np.zeros(len(F), np.int8)                                   # 0 body
lab[ft > g('roof_panel_t', 2.0)] = 1                               # 1 roof panel (second tone), off by default
for band in g('bands', []):                                        # wall bands: visor / light bar / accent
    m = (ft >= band['t0']) & (ft <= band['t1']) & (fu >= band.get('u0', 0)) & (fu <= band.get('u1', 1))
    if 'x_min' in band: m &= C[:, 0] >= band['x_min']
    if 'x_max' in band: m &= C[:, 0] <= band['x_max']
    lab[m] = {'visor': 2, 'light': 3, 'accent': 5, 'roof': 1, 'body': 0}[band['kind']]

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
wz = (V[:, 0] > 0.0) & (V[:, 0] < 0.238) & (V[:, 2] < 0.235)
lidv = np.unique(F[lab == 4])
b = poly.bounds
rep = {'faces': int(len(F)), 'roof_min_clear_over_chassis_mm': round(float(cl.min()) * 1000, 1) if len(cl) else None,
       'max_half_width_beside_wheels_mm': round(float(np.abs(V[wz, 1]).max()) * 1000, 1), 'wheel_inner_face_mm': 175.0,
       'lid_min_height_mm': round(float(V[lidv, 2].min()) * 1000, 1), 'controls_top_mm': round(K['ctl_top'] * 1000, 1),
       'lid_covers_controls': bool(b[0] <= -0.128 and b[2] >= 0.034 and b[1] <= -0.062 and b[3] >= 0.062),
       'body_max_height_mm': round(float(V[:, 2].max()) * 1000, 1)}
rep['ok'] = bool((rep['roof_min_clear_over_chassis_mm'] or 9) >= 2.0 and rep['max_half_width_beside_wheels_mm'] <= 171.0
                 and rep['lid_min_height_mm'] >= rep['controls_top_mm'] + 4 and rep['lid_covers_controls'])
print(json.dumps(rep))
np.savez(OUT, v=V, f=F.astype(np.int32), lab=lab, lid_poly=ring, **rib)
