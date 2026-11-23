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
zcut = lambda x: 0.135 - (0.012 + g('belt_drop_front', 0.0)) * ss(0.20, 0.25, x)      # belt_drop_front: body reaches further down at the nose

# ---- belt outline (left half, nose -> tail), symmetric
xs = np.array(K['xs']); yl = np.array([np.nan if v is None else v for v in K['yl']]); yr = np.array([np.nan if v is None else -v for v in K['yr']])
yo = np.nanmean(np.c_[yl, yr], 1)
import tail as _tail; yo = np.where(np.isfinite(yo), _tail.make(P)(xs, np.nan_to_num(yo)), yo)      # slim tail (optional)
yo_orig = yo.copy()
if g('nose', None):          # narrower nose: the body no longer spans the whole bumper; a dark plinth covers what it leaves open
    nw = np.array(sorted(g('nose', {})['width'])); yo = np.where(np.isfinite(yo) & (xs >= 0.262), np.minimum(np.nan_to_num(yo), np.interp(xs, nw[:, 0], nw[:, 1])), yo)
if g('waist', None):         # narrower waist / tail: only as wide as the parts inside need; a dark cap closes the skirt top
    ww = np.array(sorted(g('waist', {})['width'])); yo = np.where(np.isfinite(yo) & (xs <= -0.03), np.minimum(np.nan_to_num(yo), np.interp(xs, ww[:, 0], ww[:, 1])), yo)
front = [(x, y) for x, y in zip(xs, yo) if x >= 0.268 and np.isfinite(y)]
rear = [(x, y) for x, y in zip(xs, yo) if x <= -0.034 and np.isfinite(y)]
XT = K['x_tail']; tp_ = g('tail_plan', None)
if tp_:      # designed tail (plan view): a superellipse end running into a width curve; replaces the scanned outline behind the wheels
    XT = tp_['x_tip']; xc_ = tp_['x_c']; w_ = tp_['w']; n_ = tp_.get('n', 3.0); th_ = np.linspace(0, np.pi / 2, 90)[1:]
    wk_ = np.array(sorted(tp_.get('width', [[xc_, w_], [-0.034, w_]]))); xr_ = np.arange(xc_ + 0.004, -0.0339, 0.004)
    rear = list(zip(xc_ - (xc_ - XT) * np.cos(th_) ** (2 / n_), w_ * np.sin(th_) ** (2 / n_))) + list(zip(xr_, PchipInterpolator(wk_[:, 0], wk_[:, 1])(xr_)))
yp = g('y_pinch', 0.166); zw = g('z_wheel_edge', None); LIP = 0.0012
def outline(pinched):
    yf = front[0][1] + LIP; yr_ = rear[-1][1] + LIP
    zw_f = zw if zw is not None else zcut(0.262) - 0.003; zw_r = zw if zw is not None else zcut(-0.03) - 0.003
    mid = [(0.262, yp, zw_f), (-0.030, yp, zw_r)] if pinched else [(0.262, yf, zcut(0.262) - 0.003), (-0.030, yr_, zcut(-0.03) - 0.003)]
    pts = [(K['x_nose'] + LIP, 0.0, zcut(0.31) - 0.003)] + [(x + LIP * 0.5, y + LIP, zcut(x) - 0.003) for x, y in front[::-1]] + mid
    pts += [(x, y + LIP, zcut(x) - 0.003) for x, y in rear[::-1]] + [(XT - LIP, 0.0, zcut(-0.31) - 0.003)]
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
tt = np.unique(np.r_[np.linspace(0, 1, MR), prof[:, 0]]); TB = tt
SK = g('skirt', None); NS = int(SK.get('n', 40)) if SK else 0; NB = NS            # NB = row of the belt
tt = np.r_[-np.linspace(1, 0, NS + 1)[:-1], TB]; MR = len(tt)                      # skirt rows have t < 0 (t = -1 at the lowest row)
T = np.c_[xc + (B[:, 0] - xc) * kx, np.zeros(NH)]
crown = g('crown', 0.006)
def surf(B, sign):
    V = np.zeros((len(TB), NH, 3))
    for i, t in enumerate(TB):
        q = float(qf(t)); p = float(pf(t))
        x = B[:, 0] + (T[:, 0] - B[:, 0]) * q; y = (B[:, 1] + (T[:, 1] - B[:, 1]) * q) * sign
        zr = Zr(x) - crown * (np.abs(y) / 0.2) ** 2
        V[i, :, 0] = x; V[i, :, 1] = y; V[i, :, 2] = B[:, 2] + (zr - B[:, 2]) * p
    return V
def skirt_rows(B, sign):
    """Rows below the belt (lowest first): the body carries on downwards as one surface (one-piece shell with the skirt).
    flare [[x, m], ...] = how far the lowest row stands out from the belt outline in plan, bulge = extra belly half way down."""
    tg = np.gradient(B[:, :2], axis=0); tg /= np.maximum(np.linalg.norm(tg, axis=1, keepdims=True), 1e-12); nrm = np.c_[tg[:, 1], -tg[:, 0]]
    fk = np.array(sorted(SK.get('flare', [[-0.4, 0.0], [0.4, 0.0]]))); rear_ = 1 - ss(-0.15, -0.02, B[:, 0])
    fl = np.interp(B[:, 0], fk[:, 0], fk[:, 1]) * rear_; bg = SK.get('bulge', 0.0) * rear_; D = SK.get('depth', 0.09); V = np.zeros((NS, NH, 3))
    for i, u in enumerate(np.linspace(1, 0, NS + 1)[:-1]):
        off = fl * u + bg * np.sin(np.pi * u)
        V[i, :, 0] = B[:, 0] + nrm[:, 0] * off; V[i, :, 1] = (B[:, 1] + nrm[:, 1] * off) * sign; V[i, :, 2] = B[:, 2] - D * u
    return V
# Waist: the full-width body and the body pinched in beside the wheels are two complete surfaces; they are blended
# along raked S-curves (no square notch). Behind the wheel the blend starts further back the higher up it is.
zb0 = 0.135
sm5 = lambda a, b, x: (lambda u: u * u * u * (u * (u * 6 - 15) + 10))(np.clip((x - a) / (b - a), 0, 1))
def half(sign):
    Vw = surf(BW, sign); Vp = surf(BP, sign)
    if NS: Vw = np.concatenate([skirt_rows(BW, sign), Vw]); Vp = np.concatenate([skirt_rows(BP, sign), Vp])
    x = Vw[..., 0]; z = Vw[..., 2]; h = np.clip(z - zb0, 0, 1)
    xr1 = -0.030 - g('rear_rake', 0.45) * h; Lr = g('rear_blend', 0.03) + g('rear_sweep', 0.9) * h
    xf0 = 0.262 + g('front_rake', 0.25) * h; Lf = g('front_blend', 0.02) + g('front_sweep', 0.5) * h
    w = np.maximum(1 - sm5(xr1 - Lr, xr1, x), sm5(xf0, xf0 + Lf, x))
    Vb = Vp + w[..., None] * (Vw - Vp); tr = g('tail_rake', None)
    if tr:      # raked tail: above z0 the rear face leans forwards (by dx at z1 and above); fades out along the sides between x_a and x_b
        Vb[..., 0] += tr['dx'] * sm5(tr.get('z0', 0.10), tr.get('z1', 0.20), Vb[..., 2]) * (1 - sm5(tr.get('x_a', -0.31), tr.get('x_b', -0.19), Vb[..., 0]))
    return Vb
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
    free = np.ones(V.shape[:2])
    if NS: free[:NB + 1] = 1 - ss(-0.03, 0.0, V[NB, :, 0])[None, :]          # nose and beside the wheels: the belt stays exactly where it was
    for i in range(n):
        for lam in (0.5, -0.52):
            d = lap(V); d[0] = 0; V = V + lam * d * free[..., None]
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
VLb = VL.copy()                                                    # the faired body form (used by the continuity check)

# ======================================================================================================================
# Details. From here on the body is an unstructured triangle mesh, and every edge of a feature (chamfer start / end, paint
# line, opening) is cut into the mesh exactly along the iso-line of a distance field. Outlines are therefore smooth
# curves at any zoom, independent of the grid the body was built on.
if int(g('stride', 1)) > 1:        # lighter mesh (web): keep every n-th row / column of the faired body; feature edges are still cut exactly
    k_ = int(g('stride', 1)); ri_ = np.unique(np.r_[np.arange(0, MR, k_), NB, MR - 1]); cj_ = np.unique(np.r_[np.arange(0, NH, k_), NH - 1])
    VL = VL[ri_][:, cj_]; tt = tt[ri_]; sres = sres[cj_]; BW = BW[cj_]; BP = BP[cj_]; MR, NH = len(ri_), len(cj_); NB = int(np.searchsorted(ri_, NB))
VL[-1, :, 1] = 0.0; VL[:, 0, 1] = 0.0; VL[:, -1, 1] = 0.0
VR = VL * np.array([1.0, -1.0, 1.0])
iL = np.arange(MR * NH).reshape(MR, NH); iR = iL + MR * NH; iR[:, 0] = iL[:, 0]; iR[:, -1] = iL[:, -1]; iR[-1, :] = iL[-1, :]      # halves welded on the centre line
def grid_faces(I, flip):
    a = I[:-1, :-1].ravel(); b = I[:-1, 1:].ravel(); c = I[1:, 1:].ravel(); d = I[1:, :-1].ravel()
    return np.concatenate([np.c_[a, b, c], np.c_[a, c, d]] if not flip else [np.c_[a, c, b], np.c_[a, d, c]])
V = np.concatenate([VL.reshape(-1, 3), VR.reshape(-1, 3)])
F = np.concatenate([grid_faces(iL, True), grid_faces(iR, False)])
capx = (BW[:, 0] > 0.263) if NS else ((BW[:, 0] < -0.031) | (BW[:, 0] > 0.263)); gap_ = np.linalg.norm(VL[NB] - BW, axis=1) > 0.0008
ci = np.nonzero(capx[:-1] & capx[1:] & (gap_[:-1] | gap_[1:]))[0]
nV0 = len(V); V = np.concatenate([V, BW, BW * [1, -1, 1]])
if len(ci):
    F = np.concatenate([F, np.c_[iL[NB, ci], iL[NB, ci + 1], nV0 + ci + 1], np.c_[iL[NB, ci], nV0 + ci + 1, nV0 + ci],
                        np.c_[iR[NB, ci], nV0 + NH + ci + 1, iR[NB, ci + 1]], np.c_[iR[NB, ci], nV0 + NH + ci, nV0 + NH + ci + 1]])
tgrid = np.repeat(tt[:, None], NH, 1).ravel(); sgrid = np.tile(sres, MR); hgrid = (VL[..., 2] - VL[NB][None, :, 2]).ravel()
# per-vertex attributes, interpolated whenever an edge is cut: t (0 belt .. 1 spine), s (arc length along the belt from the nose),
# hb (height above the belt), cap (1 on the closing strip under the body)
A = {'t': np.r_[tgrid, tgrid, np.zeros(2 * NH)], 's': np.r_[sgrid, sgrid, sres, sres], 'hb': np.r_[hgrid, hgrid, np.zeros(2 * NH)],
     'cap': np.r_[np.zeros(2 * MR * NH), np.ones(2 * NH)]}
lab = np.zeros(len(F), np.int8)
KL = {'body': 0, 'roof': 1, 'visor': 2, 'light': 3, 'lid': 4, 'accent': 5, 'glass': 6}
WX, WZ, WR = 0.119, 0.1145, 0.116                                  # drive wheel centre / radius (side view)
_dx = V[:nV0, 0] - WX; _inw = (np.abs(_dx) < WR + 0.006) & ((V[:nV0, 2] >= WZ) | (_dx * _dx + (V[:nV0, 2] - WZ) ** 2 < (WR + 0.008) ** 2)) & (V[:nV0, 2] < 0.30)
half_w_wheels = float(np.abs(V[:nV0][_inw, 1]).max())

def split(name, iso=0.0, eps=2e-7):
    """Cut every triangle along the iso-line A[name] == iso. New vertices get every attribute interpolated."""
    global V, F, lab
    h = A[name] - iso; sg = np.where(h > eps, 1, np.where(h < -eps, -1, 0)).astype(np.int8)
    fs = sg[F]; cross = ((fs < 0).sum(1) > 0) & ((fs > 0).sum(1) > 0)
    if not cross.any(): return
    # where the line passes very close to an existing corner, the corner is put on the line instead of adding a needle triangle
    ea = F[cross].ravel(); eb = np.roll(F[cross], -1, 1).ravel(); ce = sg[ea] * sg[eb] < 0; ea = ea[ce]; eb = eb[ce]; tq = h[ea] / (h[ea] - h[eb])
    snap = np.unique(np.r_[ea[tq < SNAP], eb[tq > 1 - SNAP]])
    if len(snap):
        A[name][snap] = iso; h = A[name] - iso; sg = np.where(h > eps, 1, np.where(h < -eps, -1, 0)).astype(np.int8)
        fs = sg[F]; cross = ((fs < 0).sum(1) > 0) & ((fs > 0).sum(1) > 0)
        if not cross.any(): return
    Fc = F[cross]; sc = fs[cross]; lc = lab[cross]; nv = len(V)
    e1 = np.roll(Fc, -1, 1); cut = (sc * np.roll(sc, -1, 1)) < 0                     # edge k runs from corner k to corner k + 1
    lo = np.minimum(Fc, e1)[cut]; hi = np.maximum(Fc, e1)[cut]
    uk, inv = np.unique(lo.astype(np.int64) * nv + hi, return_inverse=True); a = uk // nv; b = uk % nv
    tp = h[a] / (h[a] - h[b])
    V = np.concatenate([V, V[a] + (V[b] - V[a]) * tp[:, None]])
    for k in A: A[k] = np.concatenate([A[k], A[k][a] + (A[k][b] - A[k][a]) * tp])
    A[name][nv:] = iso
    E = np.full(Fc.shape, -1, np.int64); E[cut] = nv + inv
    n0 = (sc == 0).sum(1); out = [F[~cross]]; outl = [lab[~cross]]
    m2 = n0 == 0                                                                      # two edges cut: one corner on its own side
    if m2.any():
        f = Fc[m2]; s = sc[m2]; e = E[m2]; r = np.arange(len(f))
        k0 = np.where(s[:, 0] != s[:, 1], np.where(s[:, 0] != s[:, 2], 0, 1), 2)
        a_ = f[r, k0]; b_ = f[r, (k0 + 1) % 3]; c_ = f[r, (k0 + 2) % 3]; pab = e[r, k0]; pca = e[r, (k0 + 2) % 3]
        out += [np.c_[a_, pab, pca], np.c_[pab, b_, c_], np.c_[pab, c_, pca]]; outl += [lc[m2]] * 3
    m1 = n0 == 1                                                                      # the line passes through one corner
    if m1.any():
        f = Fc[m1]; s = sc[m1]; e = E[m1]; r = np.arange(len(f)); kz = np.argmax(s == 0, 1)
        a_ = f[r, kz]; b_ = f[r, (kz + 1) % 3]; c_ = f[r, (kz + 2) % 3]; p = e[r, (kz + 1) % 3]
        out += [np.c_[a_, b_, p], np.c_[a_, p, c_]]; outl += [lc[m1]] * 2
    F = np.concatenate(out); lab = np.concatenate(outl)
SNAP = 0.015
def refine(sel, maxedge):
    """Halve every edge longer than maxedge in the faces picked by sel() until none is left (neighbours are split to match)."""
    global V, F, lab
    for it in range(8):
        m_ = sel(); P_ = V[F]; mk = (np.linalg.norm(P_ - np.roll(P_, -1, 1), axis=2) > maxedge) & m_[:, None]
        if not mk.any(): break
        nv = len(V); b_ = np.roll(F, -1, 1); key = np.minimum(F, b_).astype(np.int64) * nv + np.maximum(F, b_); uk = np.unique(key[mk])
        pos = np.clip(np.searchsorted(uk, key), 0, len(uk) - 1); has = uk[pos] == key; ia = uk // nv; ib = uk % nv
        V = np.concatenate([V, 0.5 * (V[ia] + V[ib])])
        for k in A: A[k] = np.concatenate([A[k], 0.5 * (A[k][ia] + A[k][ib])])
        mid = nv + pos; n_ = has.sum(1); out = [F[n_ == 0]]; outl = [lab[n_ == 0]]
        def rot(mask, k0):
            f = F[mask]; md = mid[mask]; r = np.arange(len(f)); return [f[r, (k0 + i) % 3] for i in range(3)] + [md[r, (k0 + i) % 3] for i in range(3)]
        m1 = n_ == 1
        if m1.any(): a_, b2, c_, mab, _, _ = rot(m1, np.argmax(has[m1], 1)); out += [np.c_[a_, mab, c_], np.c_[mab, b2, c_]]; outl += [lab[m1]] * 2
        m2 = n_ == 2
        if m2.any(): a_, b2, c_, _, mbc, mca = rot(m2, np.argmin(has[m2], 1)); out += [np.c_[c_, mca, mbc], np.c_[a_, b2, mbc], np.c_[a_, mbc, mca]]; outl += [lab[m2]] * 3
        m3 = n_ == 3
        if m3.any(): a_, b2, c_, mab, mbc, mca = rot(m3, np.zeros(int(m3.sum()), int)); out += [np.c_[a_, mab, mca], np.c_[mab, b2, mbc], np.c_[mca, mbc, c_], np.c_[mab, mbc, mca]]; outl += [lab[m3]] * 4
        F = np.concatenate(out); lab = np.concatenate(outl)
def fine(name, fn, lo, hi, maxedge, extra=None):
    """Local refinement where lo < A[name] < hi (small radii need more points than the body grid has); the field is then evaluated again."""
    if not maxedge: return
    maxedge = maxedge * g('fine_scale', 1.0)
    def sel():
        m_ = fmean(name); k_ = (m_ > lo) & (m_ < hi)
        return k_ if extra is None else k_ & extra()
    refine(sel, maxedge); A[name] = fn()
def fmean(name): return A[name][F].mean(1)
def paint(name, kind, iso=0.0, lo=None):
    """Paint everything with lo < A[name] < iso; the border is cut exactly."""
    split(name, iso)
    if lo is not None: split(name, lo)
    m = fmean(name); lab[(m < iso) if lo is None else ((m < iso) & (m > lo))] = KL[kind]
def sd(pg, a, b):
    d = shapely.distance(pg.boundary, shapely.points(a, b)); return np.where(shapely.contains_xy(pg, a, b), -d, d)
HOLD = 0.0004
def cham(c, f, n=2):
    """Chamfer of width c centred on distance 0 (1 inside .. 0 outside), each end rounded by a fillet of half-width f.
    Returns the distances where the mesh is cut (fillet segments + one holding line each side) and the profile there."""
    f = min(f, 0.2 * c); h = f / c
    S = lambda v: np.where(v <= -h, 0.0, np.where(v >= h, v, (v + h) ** 2 / (4 * h)))
    o = np.linspace(-f, f, 2 * n + 1); hi_ = [-c / 2 + f + HOLD, c / 2 - f - HOLD] if c - 2 * f > 3 * HOLD else []
    d = np.r_[-c / 2 - f - HOLD, -c / 2 + o, hi_, c / 2 + o, c / 2 + f + HOLD]; u = 0.5 - d / c
    return d, S(u) - S(u - 1)
def cuts(name, d):
    for v in d: split(name, float(v))

if SK:
    # Lower edge of the one-piece body: the belt at the nose and beside the wheels, an arc round each wheel, then the hem of the
    # tail skirt (hem [[|y|, z], ...]: low along the sides and round the corners, cut out in the middle of the tail where the
    # finned chassis apron shows as a diffuser).
    hk_ = np.array(sorted(SK['hem'])); Ra_ = SK.get('arch_r', 0.128)
    hq_ = np.abs(V[:, 1]) if SK.get('hem_by', 'y') == 'y' else V[:, 0]            # hem height by |y| (corners low, centre of the tail cut out) or by x
    zh_ = PchipInterpolator(hk_[:, 0], hk_[:, 1])(np.clip(hq_, hk_[0, 0], hk_[-1, 0])); arch_ = np.hypot(V[:, 0] - WX, V[:, 2] - WZ) - Ra_
    A['hem'] = np.where(V[:, 0] > 0.1, A['hb'], np.minimum(V[:, 2] - zh_, np.maximum(arch_, A['hb']))); A['hem'][A['cap'] > 0.5] = 1.0
    split('hem'); keep_ = fmean('hem') > 0; F = F[keep_]; lab = lab[keep_]
def panel_poly(pn):
    if 'poly' in pn:
        pg = Polygon(pn['poly']); pg = shapely.union(pg, affinity.scale(pg, 1, -1, origin=(0, 0))); r_ = pn.get('r', 0.004); return pg.buffer(-r_).buffer(r_, quad_segs=16)
    pl = [tuple(p_) for p_ in pn['pts']]; pg = Polygon(pl + [(x_, -y_) for x_, y_ in pl[::-1]]).buffer(0)
    r_ = pn.get('r', 0.01); return pg.buffer(-r_).buffer(r_, quad_segs=24)
PANELS = [(pn, panel_poly(pn)) for pn in g('panels', [])]
for k, (pn, pg) in enumerate(PANELS):
    # top panel: a plan polygon raised (or sunk) with a chamfer all round; chamfer edges are real mesh edges with a small fillet
    nm = 'pn%d' % k; tm = pn.get('t_min', 0.25); c = max(pn.get('chamfer', 0.008), 1e-4)
    fn_ = lambda pg=pg, tm=tm: np.maximum(sd(pg, V[:, 0], V[:, 1]), 0.5 * (tm - 0.012 - A['t']))      # cut lines close just below t_min, where the panel has no height left
    A[nm] = fn_(); d, r = cham(c, pn.get('fillet', 0.0005)); fine(nm, fn_, d[0] - 0.001, d[-1] + 0.002, pn.get('fine', 0)); cuts(nm, d); pn['_d'] = d
    V[:, 2] += pn.get('raise', 0.0) * np.interp(A[nm], d, r) * sm5(tm, tm + 0.08, A['t'])
if g('nose_profile', None):     # side profile of the nose: [[z, dx], ...] moves the front face forwards (+) / back (-) by height -> raked, wedge-like nose
    npf = np.array(sorted(g('nose_profile', []))); V[:, 0] += np.interp(V[:, 2], npf[:, 0], npf[:, 1]) * sm5(0.255, 0.30, V[:, 0]) * (1 - A['cap'])

# ---- front face: real pockets, charger openings with raised bezels
_pb = [shapely.box(-0.0521, 0.1341, -0.0087, 0.1581), shapely.box(0.0005, 0.1341, 0.0435, 0.1581)]
ports = shapely.union_all(_pb).buffer(-0.003).buffer(0.003 + g('port_margin', 0.0015), quad_segs=12)
PORT_CY = 0.5 * (ports.bounds[0] + ports.bounds[2])               # the two charger openings are not centred on the body: -4.3 mm
def _cy(v): return PORT_CY if v == 'ports' else float(v)
def _fpoly(sh):
    pg_ = Polygon(sh['pts']); r_ = sh.get('r', 0.004); return affinity.translate(pg_.buffer(-r_).buffer(r_, quad_segs=16), _cy(sh.get('cy', 0.0)), 0)
RECESS = [(rc, _fpoly(rc), 1.0) for rc in g('front_recess', [])] + [(rc, _fpoly(rc), -1.0) for rc in g('rear_recess', [])]; CAMX = None
for k, (rc, pg, sg_) in enumerate(RECESS):      # a real pocket in the front (or rear) face, drawn in the end view: pushed in by depth, bevelled walls
    nm = 'rc%d' % k; fn_ = lambda pg=pg, sg_=sg_: np.maximum(sd(pg, V[:, 1], V[:, 2]), 0.22 - sg_ * V[:, 0]); A[nm] = fn_()
    d, r = cham(max(rc.get('chamfer', 0.005), 1e-4), rc.get('fillet', 0.0004)); fine(nm, fn_, d[0] - 0.001, d[-1] + 0.001, rc.get('fine', 0.0007))
    cuts(nm, d); rc['_d'] = d; rmp = np.interp(A[nm], d, r)
    if rc.get('flat'):        # upright window: the pocket floor is a vertical plane (faces straight ahead even on a raked nose)
        xpl = float(V[rmp > 0.999, 0].min()) - rc.get('depth', 0.0); V[:, 0] += (xpl - V[:, 0]) * rmp; rc['_x'] = xpl
    else: V[:, 0] -= sg_ * rc.get('depth', 0.007) * rmp
pbz = g('port_bezel', None)
fn_ = lambda: np.maximum(sd(ports, V[:, 1], V[:, 2]), 0.22 - V[:, 0]); A['pt'] = fn_(); fine('pt', fn_, -0.001, 0.004, g('port_fine', 0.0006)); split('pt', 0.0)
if pbz:                     # raised frame round the two openings
    bw = pbz.get('width', 0.002); cuts('pt', [bw - 0.0007, bw - 0.0003, bw, bw + 0.0003, bw + 0.0007])
    V[:, 0] += pbz.get('raise', 0.0008) * np.interp(A['pt'], [bw - 0.0003, bw + 0.0003], [1.0, 0.0]) * (A['pt'] < 0.02)

# ---- paint. Order matters: later coats cover earlier ones.
bib = g('bib', None)
for k, (pn, pg) in enumerate(PANELS):
    nm = 'pn%d' % k; d = pn['_d']; c = max(pn.get('chamfer', 0.008), 1e-4); kind = pn.get('kind', 'roof')
    edge = {'outer': c / 2, 'inner': -c / 2}[pn.get('paint', 'outer' if pn.get('raise', 0.0) >= 0 else 'inner')]
    fld = nm
    if k == 0 and bib:
        # the top panel runs down the nose as a "bib": on the front face its outline is drawn in the front view (y, z)
        hw = pg.intersection(shapely.LineString([(0.26, 0), (0.26, 1)])).bounds[3] + edge
        bp = shapely.box(-hw, bib['z_low'], hw, 0.6).buffer(-bib.get('r', 0.01)).buffer(bib.get('r', 0.01), quad_segs=24)
        if bib.get('tab') is not None:      # the camera window hangs below the bib as a tab; both are one black outline
            rc, rp, _ = RECESS[bib['tab']]; bp = bp.union(rp.buffer(rc['_d'][-1] + bib.get('tab_margin', 0.0012), quad_segs=24))
            bp = bp.buffer(bib.get('tab_fillet', 0.003), quad_segs=24).buffer(-bib.get('tab_fillet', 0.003), quad_segs=24)
        e0 = edge; fn_ = lambda: (lambda w: (A[nm] - e0) * (1 - w) + sd(bp, V[:, 1], V[:, 2]) * w)(ss(0.24, 0.28, V[:, 0]))
        A['deck'] = fn_(); fine('deck', fn_, -0.0012, 0.0012, bib.get('fine', 0.0007), extra=lambda: V[F][:, :, 0].mean(1) > 0.27); fld = 'deck'; edge = 0.0
    if kind != 'body' or k > 0: paint(fld, kind, iso=edge)
    Tp = lambda u: np.clip(1.25 * u - 0.25, -0.25, 1.0)
    for e in pn.get('edges', []):           # pin stripes following the panel outline; both ends taper to a point
        o = e.get('off', 0.0075) - (c / 2 if fld == 'deck' else edge); w2 = e.get('width', 0.0022) / 2
        tf = np.minimum(Tp((V[:, 0] - e.get('x0', -1.0)) / e.get('taper0', 0.03)), Tp((V[:, 2] - e.get('z1', 0.0)) / e.get('taper1', 0.012)))
        A['so'] = A[fld] - o - w2 * tf; A['si'] = A[fld] - o + w2 * tf; split('so'); split('si')
        lab[(fmean('so') < 0) & (fmean('si') > 0)] = KL[e['kind']]
    if pn.get('groove'):                    # thin dark shadow line round the foot of a raised part
        paint(nm, pn['groove'].get('kind', 'roof'), iso=d[-1] + pn['groove'].get('width', 0.0008), lo=d[-1])
for k, (rc, pg, sg_) in enumerate(RECESS):
    if rc.get('kind', 'visor') != 'none': paint('rc%d' % k, rc.get('kind', 'visor'), iso=float(rc['_d'][-1]))
if pbz: paint('pt', pbz.get('kind', 'body'), iso=pbz.get('width', 0.002))
def fshape(sh):
    if 'pts' in sh: pg = Polygon(sh['pts'])
    elif sh.get('shape', 'rrect') == 'ellipse': pg = affinity.scale(Point(sh['cy'], sh['cz']).buffer(1, 48), sh['w'] / 2, sh['h'] / 2)
    else: pg = shapely.box(sh['cy'] - sh['w'] / 2, sh['cz'] - sh['h'] / 2, sh['cy'] + sh['w'] / 2, sh['cz'] + sh['h'] / 2)
    r = min(sh.get('r', 0.004), 0.49 * min(pg.bounds[2] - pg.bounds[0], pg.bounds[3] - pg.bounds[1]))
    pg = pg.buffer(-r).buffer(r, quad_segs=16)
    if sh.get('tilt'): pg = affinity.rotate(pg, sh['tilt'], origin='centroid')
    if sh.get('mirror', True): pg = shapely.union(pg, affinity.scale(pg, -1, 1, origin=(0, 0)))
    return affinity.translate(pg, _cy(sh.get('dy', 0.0)), 0)
for sh in g('front_shapes', []):            # lamps, status light ... drawn in the front view (y, z)
    fn_ = lambda sh=sh: np.maximum(sd(fshape(sh), V[:, 1], V[:, 2]), 0.22 - V[:, 0]); A['fs'] = fn_(); fine('fs', fn_, -0.0012, 0.0012, sh.get('fine', 0.0006)); paint('fs', sh['kind'])
for sh in g('rear_shapes', []):             # tail lamp ... drawn in the rear view (y, z)
    fn_ = lambda sh=sh: np.maximum(sd(fshape(sh), V[:, 1], V[:, 2]), V[:, 0] + 0.22); A['fs'] = fn_(); fine('fs', fn_, -0.0012, 0.0012, sh.get('fine', 0.0006)); paint('fs', sh['kind'])
rk_ = g('rocker', None)
if rk_:
    # dark rocker band round the tail with the accent line along its top edge. Drawn in (s, height above the belt), so it wraps
    # round the tail; behind each wheel the band ends on a raked edge and the accent line sweeps down it to the belt.
    jr = np.nonzero(BW[:, 0] < 0.1)[0]; sx = lambda x: float(np.interp(x, BW[jr, 0][::-1], sres[jr][::-1]))
    r_ = rk_.get('r', 0.015)
    if 'pts' in rk_:      # upper edge given as [[x, height above the belt], ...] from the wheel to the tail (free sweep)
        tp2 = [(sx(x_), h_) for x_, h_ in rk_['pts']]; pg = Polygon(tp2 + [(seg[-1] + 0.3, tp2[-1][1]), (seg[-1] + 0.3, -0.4), (tp2[0][0], -0.4)])
        if rk_.get('spline', True):
            from scipy.interpolate import PchipInterpolator as _P
            ss_ = np.linspace(tp2[0][0], tp2[-1][0], 240); hh_ = _P([p_[0] for p_ in tp2], [p_[1] for p_ in tp2])(ss_)
            pg = Polygon(list(zip(ss_, hh_)) + [(seg[-1] + 0.3, tp2[-1][1]), (seg[-1] + 0.3, -0.4), (tp2[0][0], -0.4)])
        pg = pg.buffer(-r_).buffer(r_, quad_segs=24)
    else:
        sA = sx(rk_['x_low']); sB = sx(rk_['x_top']); h_ = rk_['h']; kk_ = (sB - sA) / h_
        pg = Polygon([(sA - 0.1 * kk_, -0.1), (sB, h_), (seg[-1] + 0.3, h_), (seg[-1] + 0.3, -0.1)]).buffer(-r_).buffer(r_, quad_segs=24)
    A['rk'] = sd(pg, A['s'], A['hb']); paint('rk', 'accent', iso=rk_.get('line', 0.003)); paint('rk', rk_.get('kind', 'roof'))

# ---- charger openings: cut out, dark sleeves run back from the exact edge
def loops_of(faces):
    """Ordered boundary loops (vertex indices) of a set of faces."""
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]); n_ = int(e.max()) + 1
    b = e[~np.isin(e[:, 0].astype(np.int64) * n_ + e[:, 1], e[:, 1].astype(np.int64) * n_ + e[:, 0])]
    nxt = dict(zip(b[:, 0].tolist(), b[:, 1].tolist())); out = []
    while nxt:
        a0 = next(iter(nxt)); lp = [a0]; cur = nxt.pop(a0)
        while cur != a0 and cur in nxt: lp.append(cur); cur = nxt.pop(cur)
        out.append(np.array(lp))
    return out
from meshkit import Bag, stack, lathe, tube, ball, box, sweep, rrect, roty
hole = fmean('pt') < 0; SL = Bag()
for lp in loops_of(F[hole]):
    p_ = V[lp]; q_ = p_.copy(); q_[:, 0] = g('port_back_x', 0.3082); SL.add(*stack([p_, q_], cap1=True), 0)      # flat back wall just ahead of the chassis contacts
F = F[~hole]; lab = lab[~hole]
sv_, sf_, _ = SL.out(); extra = {'sleeve_v': sv_, 'sleeve_f': sf_}

# ---- camera module: a real lens (bezel ring, barrel, glass dome) standing in the upright window, plus a small IR sensor
cam = g('camera', None)
if cam:
    rc, rp, _ = RECESS[cam.get('recess', 1)]; cyc = 0.5 * (rp.bounds[0] + rp.bounds[2]); czc = cam.get('cz', 0.5 * (rp.bounds[1] + rp.bounds[3])); CB = Bag()
    def lens(o, R, proud):
        k_ = R / 0.0061
        CB.add(*lathe(o, (1, 0, 0), [(-0.0002, R), (proud * 0.6, R * 0.96), (proud, R * 0.87), (proud, R * 0.76), (proud * 0.7, R * 0.70)], 72), 0)      # bezel ring
        CB.add(*lathe(o, (1, 0, 0), [(proud * 0.7, R * 0.70), (-0.0012 * k_, R * 0.63), (-0.0012 * k_, R * 0.57)], 72), 1)                              # barrel
        rl = R * 0.57; sag = 0.0013 * k_; Rc = (rl * rl + sag * sag) / (2 * sag); am = np.arcsin(rl / Rc)
        CB.add(*lathe(o, (1, 0, 0), [(-0.0012 * k_ + sag - Rc * (1 - np.cos(q_)), Rc * np.sin(q_)) for q_ in np.linspace(am, 0, 12)], 72), 2)           # glass dome
    lens(np.array([rc['_x'], cyc, czc]), cam.get('r', 0.0061), cam.get('proud', 0.0016))
    if cam.get('ir', True): lens(np.array([rc['_x'], cyc - cam.get('ir_dy', 0.0165), czc]), 0.0021, 0.0006)
    if cam.get('led', True):        # status light: a small flush lens that glows
        o_ = np.array([rc['_x'], cyc + cam.get('ir_dy', 0.0165), czc]); CB.add(*lathe(o_, (1, 0, 0), [(-0.0002, 0.0021), (0.0004, 0.0019), (0.0004, 0.00155)], 48), 0)
        CB.add(*lathe(o_, (1, 0, 0), [(0.0004, 0.00155), (0.0007, 0.0011), (0.0008, 0.0)], 48), 3)
    cv_, cf_, cl_ = CB.out(); extra.update(cam_v=cv_, cam_f=cf_, cam_lab=cl_); print('camera at', np.round([rc['_x'], cyc, czc], 4), 'ports centre y', round(PORT_CY, 4), file=sys.stderr)

# ---- dark plinth caps where the body is narrower than the skirt
PLV = np.zeros((0, 3)); PV, PF = [], []; po = 0
def plinth(pgp, zb_, tiers, depth=0.004):
    global po
    for inset, zt in [(0.0, zb_)] + [(t_['inset'], zb_ + t_['height']) for t_ in tiers]:
        pp = pgp.buffer(-inset, join_style=1) if inset else pgp
        rr = shapely.get_coordinates(shapely.line_interpolate_point(pp.exterior, np.arange(0, pp.exterior.length, 0.002))); n_ = len(rr); cc = rr.mean(0)
        PV.append(np.concatenate([np.c_[rr, np.full(n_, zb_ - depth)], np.c_[rr, np.full(n_, zt)], [[cc[0], cc[1], zt]]])); i_ = np.arange(n_); j_ = (i_ + 1) % n_
        PF.append(np.concatenate([np.c_[i_, j_, j_ + n_], np.c_[i_, j_ + n_, i_ + n_], np.c_[i_ + n_, j_ + n_, np.full(n_, 2 * n_)]]) + po); po += 2 * n_ + 1
if g('nose', None):
    fo = [(x, y) for x, y in zip(xs, yo_orig) if x >= 0.262 and np.isfinite(y)]
    plinth(Polygon([(0.262, -fo[0][1])] + [(x, -y) for x, y in fo] + [(K['x_nose'], 0.0)] + [(x, y) for x, y in fo[::-1]] + [(0.262, fo[0][1])]).buffer(0).buffer(-0.003), g('nose', {}).get('cap_z', 0.1228), g('nose', {}).get('tiers', []), 0.0015)
if g('waist', None):
    ro = [(x, y) for x, y in zip(xs, yo_orig) if x <= -0.03 and np.isfinite(y)]          # tail -> wheel
    plinth(Polygon([(K['x_tail'], 0.0)] + ro + [(-0.03, ro[-1][1]), (-0.03, -ro[-1][1])] + [(x, -y) for x, y in ro[::-1]]).buffer(0).buffer(-0.007, join_style=1).buffer(0.006, join_style=1), 0.1347, g('waist', {}).get('tiers', []))      # corners rounded, 1 mm inside the skirt edge: nothing pokes through the body
if PV: PLV = np.concatenate(PV); extra.update(plinth_v=PLV, plinth_f=np.concatenate(PF))

# ---- lid: its own part, cut out of the roof exactly with a real gap. Underneath: well with seal, two gooseneck hinges,
#      two gas struts and a push latch (all generated in the closed position; the renderer swings the moving parts open).
from scipy.interpolate import CubicSpline
from scipy.spatial import Delaunay
L_ = g('lid', {'shape': 'rrect', 'cx': -0.047, 'length': 0.2, 'width': 0.15, 'r': 0.025})
cx, ln, wd = L_['cx'], L_['length'], L_['width']; hx, hy = ln / 2, wd / 2
if L_['shape'] == 'rrect': poly = shapely.box(cx - hx, -hy, cx + hx, hy)
elif L_['shape'] == 'hex':
    k = L_.get('cut', 0.035); poly = Polygon([(cx - hx, -hy + k), (cx - hx + k, -hy), (cx + hx - k, -hy), (cx + hx, -hy + k), (cx + hx, hy - k), (cx + hx - k, hy), (cx - hx + k, hy), (cx - hx, hy - k)])
else: poly = affinity.scale(Point(cx, 0).buffer(1, 48), hx, hy)
r = L_.get('r', 0.02); poly = poly.buffer(-r).buffer(r, quad_segs=24); ring_xy = np.array(poly.exterior.coords)[:-1]
gapw = L_.get('gap', 0.0006); OPEN = float(L_.get('open_deg', 65.0))
A['lid'] = np.maximum(sd(poly, V[:, 0], V[:, 1]), 0.5 * (0.35 - A['t'])); split('lid', 0.0); split('lid', gapw)
fl = fmean('lid'); lab[fl < 0] = 4
used = np.zeros(len(V), bool); used[F.ravel()] = True; tm_ = used & (A['t'] > 0.25) & (A['cap'] < 0.5)
zi = LinearNDInterpolator(V[tm_][:, :2], V[tm_][:, 2])
lid_loop = max(loops_of(F[fl < 0]), key=len); hole_loop = max(loops_of(F[fl < gapw]), key=len)
gapf = (fl > 0) & (fl < gapw); F = F[~gapf]; lab = lab[~gapf]
def inward(xy):
    ext = poly.exterior; s_ = shapely.line_locate_point(ext, shapely.points(xy)); L0 = ext.length
    a = shapely.get_coordinates(shapely.line_interpolate_point(ext, (s_ - 3e-4) % L0)); b = shapely.get_coordinates(shapely.line_interpolate_point(ext, (s_ + 3e-4) % L0))
    tg = b - a; tg /= np.linalg.norm(tg, axis=1, keepdims=True); n = np.c_[-tg[:, 1], tg[:, 0]]
    return n if ext.is_ccw else -n
def ringz(p, n, d, dz=0.0, z=None):
    xy = p[:, :2] + n * d; return np.c_[xy, (zi(xy) + dz) if z is None else np.full(len(xy), z)]
def fill(R, dz, h=0.006):
    """Surface closing the ring R (n, 3): follows the roof at the offset dz."""
    pg = Polygon(R[:, :2]); b = pg.bounds; gx_, gy_ = np.meshgrid(np.arange(b[0], b[2], h), np.arange(b[1], b[3], h)); ip = np.c_[gx_.ravel(), gy_.ravel()]
    ip = ip[shapely.contains_xy(pg.buffer(-0.6 * h), ip[:, 0], ip[:, 1])]; p2 = np.concatenate([R[:, :2], ip]); tri = Delaunay(p2).simplices; cen = p2[tri].mean(1)
    return np.c_[p2, np.r_[R[:, 2], zi(ip) + dz]], tri[shapely.contains_xy(pg, cen[:, 0], cen[:, 1])]
KM = Bag(); KF = Bag()                                             # moving with the lid / fixed to the body
K_D, K_RUB, K_STEEL, K_ARM, K_LINER, K_STRUT, K_EDGE = 0, 1, 2, 3, 4, 5, 6
pl = V[lid_loop]; nl = inward(pl[:, :2]); ph = V[hole_loop]; nh = inward(ph[:, :2])
x_rear, _, x_front, hwid = poly.bounds; zf = L_.get('floor_z', 0.2452); yh = L_.get('hinge_y', 0.048)
# lid body: 2 mm rim that presses on the seal, 6 mm inner liner that drops into the well
KM.add(*stack([pl, ringz(pl, nl, 0, -0.002)]), K_EDGE)
R6 = ringz(pl, nl, 0.0058, -0.006)
KM.add(*stack([ringz(pl, nl, 0, -0.002), ringz(pl, nl, 0.0054, -0.002), R6]), K_LINER); KM.add(*fill(R6, -0.006), K_LINER)
rp_ = poly.buffer(-0.024, join_style=2).buffer(-0.006).buffer(0.006, quad_segs=12)       # stiffening rib frame under the liner
rq = shapely.get_coordinates(shapely.line_interpolate_point(rp_.exterior, np.arange(0, rp_.exterior.length, 0.002))); rq3 = np.c_[rq, np.zeros(len(rq))]
rn = (lambda c: (c - rq) / np.linalg.norm(c - rq, axis=1, keepdims=True))(np.array(rp_.centroid.coords[0])[None]) * 0 + inward(rq)
KM.add(*stack([ringz(rq3, rn, 0, -0.0052), ringz(rq3, rn, 0.0002, -0.0074), ringz(rq3, rn, 0.0018, -0.0074), ringz(rq3, rn, 0.002, -0.0052)]), K_LINER)
# well: flange under the roof edge, ledge with the seal, walls down to the control panel floor; open slots where the hinge arms pass
slot = (np.abs(np.abs(ph[:, 1]) - yh) < 0.0058) & (ph[:, 0] < x_rear + 0.02)
KF.add(*stack([ph, ringz(ph, nh, 0, -0.0036)]), K_D)
Rw = ringz(ph, nh, gapw + 0.0051, z=zf)
KF.add(*stack([ringz(ph, nh, 0, -0.0036), ringz(ph, nh, gapw + 0.0045, -0.0036), Rw], skip=slot), K_D); KF.add(*stack(Rw[None], cap1=True), K_D)
KF.add(*stack([ringz(ph, nh, gapw + d_, z_) for d_, z_ in ((0.0008, -0.0036), (0.0010, -0.0027), (0.0016, -0.0022), (0.0025, -0.0022), (0.0031, -0.0027), (0.0033, -0.0036))], skip=slot), K_RUB)
SN, SM = [], []
for sg_ in (1, -1):
    yc = sg_ * yh; zr_ = float(zi(x_rear + 0.004, yc)); xp = x_rear - 0.026; zp = zr_ - 0.0085          # hinge axis: 26 mm behind the opening, 8.5 mm under the roof
    pw_ = 0.0116 if L_.get('struts', False) else 0.0240
    bv, bf = box(((xp - 0.008 + x_rear + 0.006) / 2, yc, (zf + zr_ - 0.0037) / 2), (x_rear + 0.006 - (xp - 0.008), pw_, zr_ - 0.0037 - zf))
    KF.add(bv, np.delete(bf, [2, 3], 0), K_D)                                                           # hinge pocket under the roof (open towards the well)
    prof = shapely.box(-0.006, zf - zp, 0.006, 0.0).union(Point(0, 0).buffer(0.0055, 24)); cc = np.array(prof.exterior.coords)[:-1]
    for ys_ in (yc - 0.0036, yc + 0.0036):                                                              # two bracket cheeks carrying the pin
        R0 = np.c_[xp + cc[:, 0], np.full(len(cc), ys_ - 0.0008), zp + cc[:, 1]]; KF.add(*stack([R0, R0 + [0, 0.0016, 0]], cap0=True, cap1=True), K_ARM)
    KF.add(*tube((xp, yc - 0.0052, zp), (xp, yc + 0.0052, zp), 0.0016, 20), K_STEEL)
    for e_ in (-1, 1): KF.add(*tube((xp, yc + e_ * 0.0044, zp), (xp, yc + e_ * 0.0055, zp), 0.0028, 20), K_STEEL)
    zl = float(zi(x_rear + 0.023, yc)) - 0.006                                                          # liner underside at the hinge foot
    cp = np.array([(0, 0), (0.004, -0.011), (0.011, -0.023), (0.020, -0.0295), (0.030, -0.027), (0.036, -0.016), (0.0385, -0.004), (0.0385, zl - zp - 0.0008)])
    u = np.r_[0, np.cumsum(np.linalg.norm(np.diff(cp, axis=0), axis=1))]; pp = CubicSpline(u, cp)(np.linspace(0, u[-1], 80))
    KM.add(*sweep(np.c_[xp + pp[:, 0], np.full(len(pp), yc), zp + pp[:, 1]], rrect(0.005, 0.004, 0.001)), K_ARM)      # gooseneck arm
    KM.add(*tube((xp, yc - 0.0025, zp), (xp, yc + 0.0025, zp), 0.0034, 28), K_ARM)
    KM.add(*box((xp + 0.049, yc, zl - 0.0008), (0.030, 0.012, 0.0016)), K_ARM)                           # foot plate screwed to the liner
    for xs_ in (0.046, 0.058): KM.add(*lathe((xp + xs_, yc, zl - 0.0016), (0, 0, -1), [(0, 0.002), (0.0007, 0.002), (0.001, 0.0014), (0.001, 0.0)], 20), K_STEEL)
    if L_.get('struts', False):
        ys_ = sg_ * (hwid - 0.0125); Np = np.array([xp + 0.190, ys_, zp - 0.017]); Mp = np.array([xp + 0.070, ys_, float(zi(xp + 0.070, ys_)) - 0.006 - 0.0045])
        KF.add(*box((Np[0], sg_ * (hwid - 0.0062), Np[2]), (0.012, 0.0028, 0.010)), K_ARM)               # strut bracket on the well wall + ball stud
        KF.add(*tube((Np[0], sg_ * (hwid - 0.0076), Np[2]), Np, 0.0013, 16), K_STEEL)
        KM.add(*box((Mp[0], sg_ * (hwid - 0.0077), Mp[2] + 0.0016), (0.009, 0.002, 0.0074)), K_ARM)      # strut bracket on the lid + ball stud
        KM.add(*tube((Mp[0], sg_ * (hwid - 0.0087), Mp[2]), Mp, 0.0013, 16), K_STEEL)
        SN.append(Np); SM.append(Mp)
    else:                                                                                               # friction (torque) cartridge on the pin: holds the lid at any angle, no struts
        e_ = sg_; y0_ = yc + e_ * 0.0058
        KF.add(*lathe((xp, y0_, zp), (0, e_, 0), [(0, 0.0), (0, 0.0040), (0.0003, 0.0043), (0.0052, 0.0043), (0.0056, 0.0039), (0.0056, 0.0022), (0.0050, 0.0022), (0.0050, 0.0)], 32), K_D)
        KF.add(*box((xp - 0.0056, y0_ + e_ * 0.0028, zp - 0.0005), (0.0050, 0.0036, 0.0030)), K_D)       # anti-rotation tab of the cartridge, keyed into the bracket
zfr = float(zi(x_front - 0.012, 0.0)); xs_ = x_front - 0.0115                                           # push latch at the front: housing on the wall, striker loop on the lid
KF.add(*box((x_front - 0.0101, 0, zfr - 0.0135), (0.010, 0.019, 0.011)), K_ARM); KF.add(*box((xs_, 0, zfr - 0.00795), (0.0034, 0.015, 0.0001)), K_RUB)
KM.add(*box((xs_, 0, zfr - 0.0066), (0.007, 0.02, 0.0012)), K_ARM)
for e_ in (-1, 1): KM.add(*tube((xs_, e_ * 0.006, zfr - 0.006), (xs_, e_ * 0.006, zfr - 0.0125), 0.0011, 14), K_STEEL)
KM.add(*tube((xs_, -0.006, zfr - 0.0125), (xs_, 0.006, zfr - 0.0125), 0.0011, 14), K_STEEL)
kmv, kmf, kml = KM.out(); kfv, kff, kfl = KF.out()
extra.update(kitm_v=kmv, kitm_f=kmf, kitm_lab=kml, kitf_v=kfv, kitf_f=kff, kitf_lab=kfl, strut_n=np.array(SN).reshape(-1, 3), strut_m=np.array(SM).reshape(-1, 3),
             lid_pivot=np.array([xp, zp]), lid_open=np.array([OPEN]))

# ---- fastening to the chassis. Positions come from mounts.json (measured on the chassis mesh by mounts_from_chassis.py).
#      posts -> screws from above: a well through the body down to a seat on each post (chamfered rim, screw tower inside).
#      under -> screws from below: bosses of the body standing on the chassis (front wings; rear corner sockets via a bracket).
#      clips -> snap hooks on the inside of the tail skirt that reach into the windows of the chassis apron.
MT = g('mounts', None); MB = Bag(); M_IN, M_DARK, M_STEEL, M_SCREW = 0, 1, 2, 3; mrep = {}
def _proj(sub, i, j, k, a, b):
    """Intersections of the axis-parallel line (coordinates i, j fixed at a, b) with the faces 'sub': values of coordinate k."""
    P0 = V[F[sub, 0]]; e1 = V[F[sub, 1]] - P0; e2 = V[F[sub, 2]] - P0; den = e1[:, i] * e2[:, j] - e1[:, j] * e2[:, i]; ok_ = np.abs(den) > 1e-18
    da = a - P0[:, i]; db = b - P0[:, j]; dn = np.where(ok_, den, 1.0); u_ = (da * e2[:, j] - db * e2[:, i]) / dn; v_ = (e1[:, i] * db - e1[:, j] * da) / dn
    hit = ok_ & (u_ >= -1e-9) & (v_ >= -1e-9) & (u_ + v_ <= 1 + 1e-9); return (P0[:, k] + u_ * e1[:, k] + v_ * e2[:, k])[hit]
def _fin(*pts, th=0.0012):
    """Thin flat rib with the outline pts (3D points in one plane), thickness th."""
    T_ = np.array([np.asarray(t_, float) for t_ in pts]); n_ = np.cross(T_[1] - T_[0], T_[-1] - T_[0]); n_ /= max(np.linalg.norm(n_), 1e-12)
    MB.add(*stack([T_ - n_ * th / 2, T_ + n_ * th / 2], cap0=True, cap1=True), M_IN)
if MT:
    MJ = json.load(open('mounts.json')); posts = np.array(MJ['posts'], float)
    rw = MT.get('well_r', 0.00375); cw = MT.get('well_chamfer', 0.0006); ft = MT.get('seat', 0.003); rt = MT.get('tower_r', 0.0056); rh = MT.get('hole_r', 0.0018)
    def wfield():
        d = np.full(len(V), 1.0)
        for x_, y_, z_, _ in posts:
            dk = np.hypot(V[:, 0] - x_, V[:, 1] - y_) - rw; d = np.where((V[:, 2] > z_ + ft + 0.0005) & (dk < d), dk, d)
        return d
    A['wl'] = wfield(); fine('wl', wfield, -0.0015, cw + 0.002, MT.get('fine', 0.0005)); cuts('wl', [0.0, cw, cw + HOLD])
    dd = A['wl']; V[:, 2] -= np.where(dd < cw, cw - np.maximum(dd, 0.0), 0.0)                      # 45 degree chamfer round the rim
    hole_ = fmean('wl') < 0; wloops = loops_of(F[hole_]); F = F[~hole_]; lab = lab[~hole_]; Cf = V[F].mean(1); depth_ = {}
    def zup(x_, y_, z0, sub):
        z = _proj(sub, 0, 1, 2, x_, y_); z = z[z > z0]; return float(z.min()) if len(z) else np.nan
    def skin_ring(x_, y_, dirs, r_, z0, sub):
        zs = np.array([zup(x_ + d_[0] * r_, y_ + d_[1] * r_, z0, sub) for d_ in dirs]); return np.where(np.isfinite(zs), zs, np.nanmin(zs) if np.isfinite(zs).any() else z0 + 0.02) - 0.0008
    def rib(x_, y_, q_, r0, ln_, z0, drop, sub):
        """Rib from a boss (radius r0) out along the angle q_: its top edge follows the inside of the skin (1 mm below the outer surface)."""
        d_ = np.array([np.cos(q_), np.sin(q_)]); rs = r0 + np.linspace(0, ln_, 6); zt = np.array([zup(x_ + d_[0] * r_, y_ + d_[1] * r_, z0, sub) for r_ in rs]) - 0.0018
        if not np.isfinite(zt).all() or np.ptp(zt) > 0.02: return
        zb = zt[0] - (drop if drop > 0 else -drop * (zt[0] - z0))
        if zb < z0 + 0.001: return
        _fin((x_ + d_[0] * r0, y_ + d_[1] * r0, zb), *[(x_ + d_[0] * r_, y_ + d_[1] * r_, z_i) for r_, z_i in zip(rs, zt)])
    def screw(x_, y_, zf, down=0.011):
        q = np.linspace(0, 2 * np.pi, 48, endpoint=False); cs = np.c_[np.cos(q), np.sin(q)]; lobe = 0.00105 + 0.00022 * np.cos(6 * q)
        R = [np.c_[x_ + cs[:, 0] * r_, y_ + cs[:, 1] * r_, np.full(48, zf + h_)] for r_, h_ in ((0.0028, 0.0), (0.0028, 0.0013), (0.00235, 0.0019), (lobe, 0.0019), (lobe, 0.0008))]
        MB.add(*stack(R, cap1=True), M_SCREW); MB.add(*tube((x_, y_, zf), (x_, y_, zf - down), 0.0015, 20), M_SCREW)
    for lp in wloops:
        p = V[lp]; c2 = p[:, :2].mean(0); k = int(np.argmin(np.hypot(posts[:, 0] - c2[0], posts[:, 1] - c2[1]))); x_, y_, z_ = posts[k, :3]
        if np.hypot(c2[0] - x_, c2[1] - y_) > 0.002: continue
        dirs = p[:, :2] - [x_, y_]; dirs /= np.linalg.norm(dirs, axis=1, keepdims=True); n_ = len(p)
        zf = z_ + (ft if p[:, 2].min() - z_ - ft >= 0.0025 else 0.002)                 # thinner seat where the skin is close above the post
        ring = lambda r_, z: np.c_[x_ + dirs[:, 0] * r_, y_ + dirs[:, 1] * r_, np.full(n_, z) if np.isscalar(z) else z]
        sub = np.hypot(Cf[:, 0] - x_, Cf[:, 1] - y_) < 0.022; zs = skin_ring(x_, y_, dirs, rt, z_ + 0.001, sub)
        MB.add(*stack([p, ring(rw, zf), ring(rh, zf)]), M_IN); MB.add(*stack([ring(rh, zf), ring(rh, z_ + 0.0002)]), M_DARK)      # well wall, seat, screw hole
        MB.add(*stack([ring(rh, z_ + 0.0002), ring(rt, z_ + 0.0002), ring(rt, zs)]), M_IN)                                          # foot on the post, tower up to the skin
        for q_ in np.radians([45, 135, 225, 315]): rib(x_, y_, q_, rt, 0.009, z_, 0.011, sub)                                      # ribs tying the tower to the skin
        screw(x_, y_, zf); depth_[k] = float(p[:, 2].min() - zf)
    nund = 0
    for ub in MJ['under']:
        x_, y_, z_ = ub['x'], ub['y'], ub['z'] + 0.0002; q = np.linspace(0, 2 * np.pi, 40, endpoint=False); dirs = np.c_[np.cos(q), np.sin(q)]
        ring = lambda r_, z: np.c_[x_ + dirs[:, 0] * r_, y_ + dirs[:, 1] * r_, np.full(40, z) if np.isscalar(z) else z]
        sub = np.hypot(Cf[:, 0] - x_, Cf[:, 1] - y_) < 0.03
        if ub['kind'] == 'front':                      # long boss hanging from the skin above, standing on the wing of the chassis
            rb = 0.0042; zs = skin_ring(x_, y_, dirs, rb, z_ + 0.001, sub); MB.add(*stack([ring(0.0013, z_), ring(rb, z_), ring(rb, zs)]), M_IN)
            for q_ in np.radians([0, 90, 180, 270]): rib(x_, y_, q_, rb, 0.010, z_, -0.6, sub)
        else:                                          # rear corner: boss sitting in the socket of the apron, carried by a bracket from the skirt wall
            rb = ub['r_socket'] * 0.001 - 0.0007; zt = ub['z_socket_top'] + 0.0015; sg_ = np.sign(y_); MB.add(*stack([ring(0.0013, z_), ring(rb, z_), ring(rb, zt)]), M_IN)
            sb2 = (np.abs(Cf[:, 0] - x_) < 0.03) & (np.abs(Cf[:, 2] - zt) < 0.04) & (Cf[:, 1] * sg_ > 0.15)
            def ywall(xx, zz):
                yy = np.concatenate([_proj(sb2, 0, 2, 1, xx + dx_, zz + dz_) for dx_ in (-0.001, 0.001) for dz_ in (-0.0016, 0.0016)]) * sg_; yy = yy[yy > abs(y_)]
                return sg_ * (float(yy.min()) - 0.0014) if len(yy) else np.nan
            xa, xb = x_ - 0.0075, x_ + 0.0075; yi = y_ - sg_ * (rb + 0.001); Q = [(xa, yi), (xb, yi), (xb, ywall(xb, zt + 0.0015)), (xa, ywall(xa, zt + 0.0015))]
            MB.add(*stack([np.array([(a_, b_, zt) for a_, b_ in Q]), np.array([(a_, b_, zt + 0.003) for a_, b_ in Q])], cap0=True, cap1=True), M_IN)
            for xx in (x_ - 0.005, x_ + 0.005): _fin((xx, ywall(xx, zt + 0.003), zt + 0.003), (xx, ywall(xx, zt + 0.003) - sg_ * 0.012, zt + 0.003), (xx, ywall(xx, zt + 0.016), zt + 0.016), th=0.0015)
        MB.add(*stack([ring(0.0013, z_ + 0.012), ring(0.0013, z_)], cap0=True), M_DARK); nund += 1                                  # pilot hole for the screw from below
    nclip = 0
    for cl in MJ['clips']:                             # snap hooks: tongue from the inside of the tail wall into the window of the apron, barb behind the wall
        yc = (cl['y0'] + cl['y1']) / 2; zc = (cl['z0'] + cl['z1']) / 2; wy = min(0.006, cl['y1'] - cl['y0'] - 0.002)
        sb3 = (np.abs(Cf[:, 1] - yc) < 0.02) & (np.abs(Cf[:, 2] - zc) < 0.02) & (Cf[:, 0] < -0.25)
        xin = lambda zz: max(float(_proj(sb3, 1, 2, 0, yc + dy_, zz + dz_).min()) for dy_ in (-wy / 2, 0, wy / 2) for dz_ in (-0.0012, 0.0012)) + 0.0012      # just inside the skin
        x0 = xin(zc); x1 = cl['x_below'] + 0.0085
        MB.add(*box(((x0 + x1) / 2, yc, zc), (x1 - x0, wy, 0.0022)), M_IN); MB.add(*box((x1 - 0.0013, yc, zc - 0.0019), (0.0026, wy, 0.0016)), M_IN)
        xr3 = xin(zc + 0.009)
        _fin((x0, yc, zc + 0.0011), (x0 + 0.008, yc, zc + 0.0011), (xr3, yc, zc + 0.009), th=0.0015); nclip += 1
    mrep = {'screws_from_above': len(depth_), 'well_min_depth_mm': round(min(depth_.values()) * 1000, 1) if depth_ else None, 'screws_from_below': nund, 'snap_hooks': nclip,
            'posts': len(posts)}

# ---- checks
H = np.load('chassis_top.npy'); tv = A['t']; used = np.zeros(len(V), bool); used[F.ravel()] = True
gx = ((V[:, 0] + 0.34) / 0.005).astype(int); gy = ((V[:, 1] + 0.25) / 0.005).astype(int)
ok = used & (gx >= 0) & (gx < H.shape[0]) & (gy >= 0) & (gy < H.shape[1]) & (tv > 0.3) & (np.abs(V[:, 1]) < 0.15)
hit = ok.nonzero()[0][H[gx[ok], gy[ok]] > 0.05]; cl = V[hit, 2] - H[gx[hit], gy[hit]]
# surface continuity of the body form: curvature between neighbouring quads (belt rows excluded) -> smallest fillet radius
G = VLb; qn = np.cross(G[1:, 1:] - G[:-1, :-1], G[:-1, 1:] - G[1:, :-1]); qa = np.linalg.norm(qn, axis=2); qn = qn / np.maximum(qa[..., None], 1e-20)
qc = 0.25 * (G[1:, 1:] + G[:-1, :-1] + G[:-1, 1:] + G[1:, :-1]); okq = qa > 2e-7
def curv(n1, n2, c1, c2, m):
    ang = np.arccos(np.clip(np.sum(n1 * n2, -1), -1, 1)); dist = np.linalg.norm(c1 - c2, axis=-1)
    return np.where(m, ang / np.maximum(dist, 1e-6), 0.0), np.where(m, ang, 0.0)
k1, a1 = curv(qn[3:, 1:], qn[3:, :-1], qc[3:, 1:], qc[3:, :-1], okq[3:, 1:] & okq[3:, :-1])
k2, a2 = curv(qn[4:], qn[3:-1], qc[4:], qc[3:-1], okq[4:] & okq[3:-1])
kk = np.r_[k1.ravel(), k2.ravel()]; rmin = 1.0 / max(np.percentile(kk, 99.9), 1e-6)
crease = float(np.degrees(max(np.percentile(a1, 99.95), np.percentile(a2, 99.95))))
from scipy.ndimage import maximum_filter
Vu = V[used]; Hb = np.zeros_like(H); gxa = np.clip(((Vu[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1); gya = np.clip(((Vu[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)
np.maximum.at(Hb, (gxa, gya), Vu[:, 2])
if len(PLV): np.maximum.at(Hb, (np.clip(((PLV[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1), np.clip(((PLV[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)), PLV[:, 2])
Hb = maximum_filter(Hb, 3)
cxs = -0.34 + (np.arange(H.shape[0]) + 0.5) * 0.005; cys = -0.25 + (np.arange(H.shape[1]) + 0.5) * 0.005; CX, CY = np.meshgrid(cxs, cys, indexing='ij')
strut_ = (np.abs(CY) > 0.158) & (CX > -0.06) & (CX < 0.29)           # mudguard struts: removed with the mudguards
expo = (H > 0.150) & ~strut_ & (Hb < H + 0.002)
lidv = np.unique(F[lab == 4]); b = poly.bounds
ar = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
rep = {'faces': int(len(F)), 'roof_min_clear_over_chassis_mm': round(float(cl.min()) * 1000, 1) if len(cl) else None,
       'max_half_width_beside_wheels_mm': round(half_w_wheels * 1000, 1), 'wheel_inner_face_mm': 175.0,
       'lid_min_height_mm': round(float(V[lidv, 2].min()) * 1000, 1), 'controls_top_mm': round(K['ctl_top'] * 1000, 1),
       'lid_covers_controls': bool(b[0] <= -0.128 and b[2] >= 0.034 and b[1] <= -0.062 and b[3] >= 0.062),
       'body_max_height_mm': round(float(Vu[:, 2].max()) * 1000, 1),
       'min_surface_radius_mm': round(rmin * 1000, 1), 'max_crease_deg': round(crease, 1),
       'chassis_exposed_cm2': round(float(expo.sum()) * 0.25, 1), 'ports_centre_y_mm': round(PORT_CY * 1000, 2),
       'smallest_triangle_mm2': float(np.round(ar.min() * 5e5, 6)), 'lid_edge_points': int(len(lid_loop)), 'mounts': mrep}
rep['ok'] = bool((rep['roof_min_clear_over_chassis_mm'] or 9) >= 2.0 and rep['max_half_width_beside_wheels_mm'] <= 171.0
                 and rep['lid_min_height_mm'] >= rep['controls_top_mm'] + 4 and rep['lid_covers_controls']
                 and rep['min_surface_radius_mm'] >= g('min_radius', 3.0 if g('ridges', []) else 8.0) and rep['chassis_exposed_cm2'] <= 2.0
                 and (not MT or (mrep['screws_from_above'] == mrep['posts'] and mrep['well_min_depth_mm'] >= 2.0 and mrep['screws_from_below'] == len(MJ['under']) and mrep['snap_hooks'] == len(MJ['clips']))))
print(json.dumps(rep))
# ---- shading normals, computed here in double precision (angle weighted, creases sharper than smooth_deg stay sharp)
fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); fa = np.linalg.norm(fn, axis=1); okf = fa > 1e-13
V32 = V.astype(np.float32); okf &= ~((V32[F[:, 0]] == V32[F[:, 1]]).all(1) | (V32[F[:, 1]] == V32[F[:, 2]]).all(1) | (V32[F[:, 0]] == V32[F[:, 2]]).all(1))
fsrt = np.sort(F, 1); _, fi = np.unique(fsrt, axis=0, return_index=True); dup = np.ones(len(F), bool); dup[fi] = False; okf &= ~dup
F = F[okf]; lab = lab[okf]; fn = fn[okf] / fa[okf][:, None]
if fn[A['t'][F].mean(1) > 0.9][:, 2].mean() < 0: F = F[:, ::-1].copy(); fn = -fn              # outward winding
P3 = V[F]; ev = np.roll(P3, -1, 1) - P3; ev /= np.maximum(np.linalg.norm(ev, axis=2, keepdims=True), 1e-20)
cang = np.arccos(np.clip(-(ev * np.roll(ev, 1, 1)).sum(2), -1, 1))                           # corner angles (nf, 3)
vn = np.zeros((len(V), 3)); [np.add.at(vn[:, c_], F.ravel(), (fn[:, None, c_] * cang).ravel()) for c_ in range(3)]
vn /= np.maximum(np.linalg.norm(vn, axis=1, keepdims=True), 1e-20)
cth = np.cos(np.radians(g('smooth_deg', 38))); LN = vn[F].copy()                              # (nf, 3, 3): start fully smooth
dev = (fn[:, None, :] * vn[F]).sum(2); vmin = np.ones(len(V)); np.minimum.at(vmin, F.ravel(), dev.ravel())
hard = vmin < np.cos(np.radians(g('smooth_deg', 38)) / 2)                                     # vertices that touch a crease: smooth only within each side
cf, cc_ = np.nonzero(hard[F]); cv = F[cf, cc_]; o_ = np.argsort(cv, kind='stable'); cf = cf[o_]; cc_ = cc_[o_]; cv = cv[o_]
st = np.r_[0, np.nonzero(np.diff(cv))[0] + 1]; cnt = np.diff(np.r_[st, len(cv)]); gs = np.repeat(st, cnt); gk = np.repeat(cnt, cnt)
pi = np.repeat(np.arange(len(cv)), gk); pj = np.repeat(gs, gk) + (np.arange(len(pi)) - np.repeat(np.r_[0, np.cumsum(gk)[:-1]], gk))
wgt = cang[cf[pj], cc_[pj]] * ((fn[cf[pi]] * fn[cf[pj]]).sum(1) > cth)
acc = np.stack([np.bincount(pi, wgt * fn[cf[pj], c_], minlength=len(cv)) for c_ in range(3)], 1)
acc /= np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-20); LN[cf, cc_] = acc
if SK:      # the lower edge gets a return flange (wall thickness seen from below / behind)
    hl = max((lp for lp in loops_of(F[lab != 4])), key=lambda lp: np.ptp(V[lp, 0])); pH = V[hl]; nH = vn[hl].copy(); nH[:, 2] = 0; ln_ = np.linalg.norm(nH, axis=1)
    okh = (ln_ > 0.35) & (pH[:, 0] < 0.02); nH /= np.maximum(ln_[:, None], 1e-9); qH = pH - nH * SK.get('lip', 0.0028); i0 = np.arange(len(hl)); i1 = (i0 + 1) % len(hl); kq = okh[i0] & okh[i1]
    MB.add(np.concatenate([pH, qH]), np.concatenate([np.c_[i0[kq], i1[kq], i1[kq] + len(hl)], np.c_[i0[kq], i1[kq] + len(hl), i0[kq] + len(hl)]]), M_IN)
mv_, mf_, ml_ = MB.out(); extra.update(mount_v=mv_, mount_f=mf_, mount_lab=ml_)
used = np.zeros(len(V), bool); used[F.ravel()] = True
rmap = -np.ones(len(V), np.int64); rmap[used] = np.arange(int(used.sum()))
np.savez(OUT, v=V[used], f=rmap[F].astype(np.int32), lab=lab, ln=LN.astype(np.float32), lid_poly=ring_xy, **extra)
