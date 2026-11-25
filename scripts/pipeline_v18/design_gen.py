# Parametric upper body for the mower redesign.
# usage: python design_gen.py PARAMS.json OUT.npz      (run inside the design/ folder)
# The lower skirt, bumper and front face with the charger openings (keep.npz) stay as scanned; this script
# generates everything above the belt line as ONE continuous surface: outline at the belt -> walls -> roof.
# Wheels are fully exposed: between x=-0.03 and x=0.262 the body is pinched in to |y| <= y_pinch.
# The lid is a region of that same roof surface (perfectly flush), cut out with the v18 joint; its hinge, latch and well: lidmech.py.
import numpy as np, json, sys
from scipy.interpolate import PchipInterpolator, LinearNDInterpolator
from shapely.geometry import Polygon, Point
import shapely
from shapely import affinity
P = json.load(open(sys.argv[1])); OUT = sys.argv[2]
if P.get('wall'):      # [v18c] real plastic wall thickness (shell_wall.py); without the key the skins stay zero-thickness as before
    import shell_wall; WALL_ = dict(shell_wall.DEFAULTS, **P['wall'])
else: WALL_ = None
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
np_ = g('nose_plan', None)
FX1 = bool(g('fix1_outline', False))      # [fix1] outline: superellipses sampled densely INCLUDING the tip, offset by LIP along the plan normal (no centre-line nub)
def _se_dense(xc_, xt_, w_, n_):
    """[fix1] Points (x, y) on the plan superellipse from the tip (y = 0) to the side (x = xc_), dense in both x and y (uniform in angle AND in y),
    so the centre line is sampled as finely as the rest (the angle parameter alone leaves a 12-18 mm gap next to the tip)."""
    a_ = xt_ - xc_; th = np.linspace(0, np.pi / 2, 3001); u = np.linspace(0, 1, 3001)
    x1 = xc_ + a_ * np.abs(np.cos(th)) ** (2 / n_); y1 = w_ * np.sin(th) ** (2 / n_)
    x2 = xc_ + a_ * (1 - u ** n_) ** (1 / n_); y2 = w_ * u
    X = np.r_[x1, x2]; Y = np.r_[y1, y2]; o = np.argsort(Y, kind='stable'); X, Y = X[o], Y[o]
    keep = np.r_[True, np.hypot(np.diff(X), np.diff(Y)) > 2e-5]; return X[keep], Y[keep]
def _plan_normals(X, Y, xref):
    tx = np.gradient(X); ty = np.gradient(Y); L_ = np.maximum(np.hypot(tx, ty), 1e-15); nx_, ny_ = ty / L_, -tx / L_
    s_ = np.sign(nx_ * (X - xref) + ny_ * Y); s_[s_ == 0] = 1; return nx_ * s_, ny_ * s_
if np_:      # [v18] designed nose (plan view): the same superellipse family as the tail, so front and rear faces have the same plan curvature
    xcn_ = np_['x_c']; xtn_ = K['x_nose']; thn_ = np.linspace(0, np.pi / 2, 120)[1:-1]
    xn_s = xcn_ + (xtn_ - xcn_) * np.cos(thn_) ** (2 / np_.get('n', 3.3)); yn_s = np_['w'] * np.sin(thn_) ** (2 / np_.get('n', 3.3))
    kk_ = xn_s >= 0.268; front = sorted(zip(xn_s[kk_], yn_s[kk_]))
    if FX1:
        Xf_, Yf_ = _se_dense(xcn_, xtn_, np_['w'], np_.get('n', 3.3)); NXf_, NYf_ = _plan_normals(Xf_, Yf_, xcn_ - 0.05)
        kf_ = Xf_ >= 0.2625; FRONT_N = (Xf_[kf_], Yf_[kf_], NXf_[kf_], NYf_[kf_]); front = sorted(zip(Xf_[kf_], Yf_[kf_]))
rear = [(x, y) for x, y in zip(xs, yo) if x <= -0.034 and np.isfinite(y)]
def rake_ramp(z, z0, z1, b):
    """[v18] Integral of a smoothed box slope: 0 below z0, rises with slope 1 between z0 and z1, constant above; the slope is
    blended over +-b at both ends (C1, no kink). A face moved by k * rake_ramp(z) is a plane raked at atan(k) between z0 and z1."""
    zz = np.linspace(min(z0 - 3 * b, 0.0), max(z1 + 3 * b, 0.4), 8001); s_ = ss(z0 - b, z0 + b, zz) * (1 - ss(z1 - b, z1 + b, zz))
    I_ = np.r_[0, np.cumsum(0.5 * (s_[1:] + s_[:-1]) * np.diff(zz))]; return np.interp(z, zz, I_)
XT = K['x_tail']; tp_ = g('tail_plan', None)
if tp_:      # designed tail (plan view): a superellipse end running into a width curve; replaces the scanned outline behind the wheels
    XT = tp_['x_tip']; xc_ = tp_['x_c']; w_ = tp_['w']; n_ = tp_.get('n', 3.0); th_ = np.linspace(0, np.pi / 2, 90)[1:]
    wk_ = np.array(sorted(tp_.get('width', [[xc_, w_], [-0.034, w_]]))); xr_ = np.arange(xc_ + 0.004, -0.0339, 0.004)
    rear = list(zip(xc_ - (xc_ - XT) * np.cos(th_) ** (2 / n_), w_ * np.sin(th_) ** (2 / n_))) + list(zip(xr_, PchipInterpolator(wk_[:, 0], wk_[:, 1])(xr_)))
    if FX1:      # [fix1] F1: dense tail superellipse incl. the tip; F4: the width curve leaves the superellipse with zero slope (C2 spline, clamped)
        from scipy.interpolate import CubicSpline as _CS
        Xr_, Yr_ = _se_dense(xc_, XT, w_, n_); NXr_, NYr_ = _plan_normals(Xr_, Yr_, xc_ + 0.05)
        wfun_ = _CS(wk_[:, 0], wk_[:, 1], bc_type=((1, 0.0), 'natural')); xr2_ = np.arange(xc_ + 0.002, -0.0339, 0.002); yr2_ = wfun_(xr2_)
        REAR_N = (np.r_[Xr_, xr2_], np.r_[Yr_, yr2_], np.r_[NXr_, np.zeros(len(xr2_))], np.r_[NYr_, np.ones(len(xr2_))])
        rear = list(zip(REAR_N[0], REAR_N[1]))                 # tip (y = 0) first ... wheel last, as the original list
yp = g('y_pinch', 0.166); zw = g('z_wheel_edge', None); LIP = 0.0012
def outline(pinched):
    if FX1:      # [fix1] F1: every outline point is offset by LIP along its plan normal (front: x component halved, as before); no separate tip points
        Xf, Yf, Nf, Mf = FRONT_N; Xr, Yr, Nr, Mr = REAR_N
        yf = Yf[np.argmin(Xf)] + LIP; yr_ = Yr[np.argmax(Xr)] + LIP
        mid = [(0.262, yp, zw if zw is not None else zcut(0.262) - 0.003), (-0.030, yp, zw if zw is not None else zcut(-0.03) - 0.003)] if pinched else \
              [(0.262, yf, zcut(0.262) - 0.003), (-0.030, yr_, zcut(-0.03) - 0.003)]
        of = np.argsort(Yf); orr = np.argsort(-Xr)          # nose: tip -> side; tail: wheel -> tip
        pts = [(x + 0.5 * LIP * a, y + LIP * b, zcut(x) - 0.003) for x, y, a, b in zip(Xf[of], Yf[of], Nf[of], Mf[of])] + mid
        pts += [(x + LIP * a, y + LIP * b, zcut(x) - 0.003) for x, y, a, b in zip(Xr[orr], Yr[orr], Nr[orr], Mr[orr])]
        P_ = np.array(pts); P_[0, 1] = 0.0; P_[-1, 1] = 0.0; return P_
    yf = front[0][1] + LIP; yr_ = rear[-1][1] + LIP
    zw_f = zw if zw is not None else zcut(0.262) - 0.003; zw_r = zw if zw is not None else zcut(-0.03) - 0.003
    mid = [(0.262, yp, zw_f), (-0.030, yp, zw_r)] if pinched else [(0.262, yf, zcut(0.262) - 0.003), (-0.030, yr_, zcut(-0.03) - 0.003)]
    pts = [(K['x_nose'] + LIP, 0.0, zcut(0.31) - 0.003)] + [(x + LIP * 0.5, y + LIP, zcut(x) - 0.003) for x, y in front[::-1]] + mid
    pts += [(x, y + LIP, zcut(x) - 0.003) for x, y in rear[::-1]] + [(XT - LIP, 0.0, zcut(-0.31) - 0.003)]
    return np.array(pts)
ptsW = outline(False); ptsP = outline(True)
ptsP[:, 1] = np.minimum(ptsP[:, 1], yp)          # the pinched body is narrow along its whole length; the blend decides where it shows
if g('pinch_soft', 0.0):    # [v18] behind the wheels the clip is a smooth saturation, not a corner: the tail corner (where the waist blend
    #                         and the haunch tuck still mix in some of the pinched surface) then has no plan kink -> no ridge on the tail shoulder
    a_ps = g('pinch_soft', 0.03); y_raw = outline(True)[:, 1]; u_ps = np.maximum(y_raw - (yp - a_ps), 0.0)
    y_sf = np.where(u_ps > 0, yp - a_ps * np.exp(-u_ps / a_ps), y_raw)
    y_sf = y_sf + (np.minimum(y_raw, yp) - y_sf) * ss(-0.20, -0.06, ptsP[:, 0])          # hard clip again towards the wheel (no dent at the waist)
    ptsP[:, 1] = np.where(ptsP[:, 0] < -0.031, y_sf, ptsP[:, 1])
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
if g('roof_spline', 'pchip') == 'cubic':      # [fix1] F3: C2 roof (PCHIP is only C1: curvature spikes / a concave trough at the knots)
    from scipy.interpolate import CubicSpline as _CS2; Zr = _CS2(rk[:, 0], rk[:, 1], bc_type='not-a-knot')
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
if g('profile_wide', None):     # [orbit] the full-width body (haunch, tail) may have its own section, e.g. with tumblehome
    _pw = np.array(sorted(g('profile_wide', []))); qfw = PchipInterpolator(_pw[:, 0], _pw[:, 1]); pfw = PchipInterpolator(_pw[:, 0], _pw[:, 2])
else: qfw, pfw = qf, pf
SEC_ = g('section', None)       # [fix2] A1 / A6 / A7: G2 superellipse shoulder in ABSOLUTE size (the same corner along the whole body and round the nose / tail)
def _sec_table(sc):
    a_, b_, n_, al_ = sc['a'], sc['b'], sc.get('n', 3.3), np.tan(np.radians(sc.get('draft_deg', 0.0)))
    th_ = np.linspace(0, np.pi / 2, 20001); uc_ = a_ * (1 - np.cos(th_) ** (2 / n_)); vc_ = b_ * np.sin(th_) ** (2 / n_); uc_ = uc_ + vc_ * al_      # sheared: G2 with the drafted wall
    sl_ = np.r_[0, np.cumsum(np.hypot(np.diff(uc_), np.diff(vc_)))]; return dict(a=a_, b=b_, al=al_, uc=uc_, vc=vc_, sl=sl_, L=sl_[-1], t1=sc.get('t1', 0.28), t2=sc.get('t2', 0.52))
SECT_ = _sec_table(SEC_) if SEC_ else None; SECTW_ = _sec_table(dict(SEC_, **g('section_wide', {}))) if SEC_ else None
def _pchip_cols(tk, Y, t):
    """Monotone cubic (Fritsch-Carlson) through the knots (tk[k], Y[k, j]) for every column j at the scalar t."""
    h_ = np.diff(tk); dl_ = np.diff(Y, axis=0) / h_[:, None]; m_ = np.zeros_like(Y)
    m_[1:-1] = np.where(dl_[:-1] * dl_[1:] > 0, 2 * dl_[:-1] * dl_[1:] / np.maximum(dl_[:-1] + dl_[1:], 1e-12), 0.0); m_[0] = dl_[0]; m_[-1] = dl_[-1]
    k_ = int(np.clip(np.searchsorted(tk, t, side='right') - 1, 0, len(tk) - 2)); u_ = (t - tk[k_]) / h_[k_]
    h00, h10, h01, h11 = 2 * u_ ** 3 - 3 * u_ ** 2 + 1, u_ ** 3 - 2 * u_ ** 2 + u_, -2 * u_ ** 3 + 3 * u_ ** 2, u_ ** 3 - u_ ** 2
    return h00 * Y[k_] + h10 * h_[k_] * m_[k_] + h01 * Y[k_ + 1] + h11 * h_[k_] * m_[k_ + 1]
def surf_se(B, sign, wide=False):
    """[fix2] Section = drafted wall (draft_deg, plan-inward) -> superellipse corner a x b (exponent n, zero curvature at both ends) -> roof.
    Rows are spaced by arc length along each column's own section (monotone cubic through the three junctions at t1 / t2), so the corner
    is the same absolute shape everywhere and only the wall and roof lengths change with the local height and half-width."""
    S_ = SECTW_ if wide else SECT_; V = np.zeros((len(TB), NH, 3)); D_ = T[:, :2] - B[:, :2]; W_ = np.linalg.norm(D_, axis=1); dn_ = D_ / W_[:, None]
    H_ = Zr(B[:, 0]) - crown * (np.abs(B[:, 1]) / 0.2) ** 2 - B[:, 2]
    for _ in range(3):      # height at the corner end (it moves inward by u_e)
        ue_ = (H_ - S_['b']) * S_['al'] + S_['a'] + S_['b'] * S_['al']; xe_ = B[:, 0] + dn_[:, 0] * ue_; ye_ = B[:, 1] + dn_[:, 1] * ue_
        H_ = Zr(xe_) - crown * (np.abs(ye_) / 0.2) ** 2 - B[:, 2]
    s1_ = (H_ - S_['b']) * np.sqrt(1 + S_['al'] ** 2); s2_ = s1_ + S_['L']; S3_ = s2_ + np.maximum(W_ - ue_, 1e-4)
    tk_ = np.array([0.0, S_['t1'], S_['t2'], 1.0]); Yk_ = np.stack([np.zeros(NH), s1_, s2_, S3_])
    for i, t in enumerate(TB):
        s_ = _pchip_cols(tk_, Yk_, float(t))
        cw_ = s_ <= s1_; cr_ = s_ >= s2_; cc_ = ~cw_ & ~cr_
        sv_ = np.clip(s_ - s1_, 0, S_['L']); uc_ = np.interp(sv_, S_['sl'], S_['uc']); vc_ = np.interp(sv_, S_['sl'], S_['vc'])
        hw_ = (H_ - S_['b'])
        u_ = np.where(cw_, s_ / np.sqrt(1 + S_['al'] ** 2) * S_['al'], np.where(cc_, hw_ * S_['al'] + uc_, ue_ + (s_ - s2_)))
        v_ = np.where(cw_, s_ / np.sqrt(1 + S_['al'] ** 2), np.where(cc_, hw_ + vc_, H_))
        q = np.clip(u_ / W_, 0, 1); p = np.clip(v_ / H_, 0, 1)
        x = B[:, 0] + (T[:, 0] - B[:, 0]) * q; y = (B[:, 1] + (T[:, 1] - B[:, 1]) * q) * sign
        zr = Zr(x) - crown * (np.abs(y) / 0.2) ** 2
        V[i, :, 0] = x; V[i, :, 1] = y; V[i, :, 2] = B[:, 2] + (zr - B[:, 2]) * p
    return V
def surf(B, sign, wide=False):
    if SECT_ is not None: return surf_se(B, sign, wide)
    V = np.zeros((len(TB), NH, 3))
    for i, t in enumerate(TB):
        q = float((qfw if wide else qf)(t)); p = float((pfw if wide else pf)(t))
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
    Vw = surf(BW, sign, True); Vp = surf(BP, sign)
    if NS: Vw = np.concatenate([skirt_rows(BW, sign), Vw]); Vp = np.concatenate([skirt_rows(BP, sign), Vp])
    x = Vw[..., 0]; z = Vw[..., 2]; h = np.clip(z - zb0, 0, 1)
    xr1 = -0.030 - g('rear_rake', 0.45) * h; Lr = g('rear_blend', 0.03) + g('rear_sweep', 0.9) * h
    xf0 = 0.262 + g('front_rake', 0.25) * h; Lf = g('front_blend', 0.02) + g('front_sweep', 0.5) * h
    wr_ = 1 - sm5(xr1 - Lr, xr1, x); tk_ = g('waist_tuck', None)
    if tk_: wr_ = wr_ * (1 - tk_.get('amount', 0.5) * sm5(tk_.get('z0', 0.15), tk_.get('z1', 0.26), z))      # [orbit] the haunch widens the lower body only
    w = np.maximum(wr_, sm5(xf0, xf0 + Lf, x))
    Vb = Vp + w[..., None] * (Vw - Vp); tr = g('tail_rake', None)
    if tr and tr.get('k') is not None:      # [v18] planar rake: the tail face leans forwards at atan(k) between z0 and z1 (same k as the nose face and the tail rise)
        Vb[..., 0] += tr['k'] * rake_ramp(Vb[..., 2], tr['z0'], tr['z1'], tr.get('b', 0.012)) * (1 - sm5(tr.get('x_a', -0.31), tr.get('x_b', -0.19), Vb[..., 0]))
    elif tr:      # raked tail: above z0 the rear face leans forwards (by dx at z1 and above); fades out along the sides between x_a and x_b
        Vb[..., 0] += tr['dx'] * sm5(tr.get('z0', 0.10), tr.get('z1', 0.20), Vb[..., 2]) * (1 - sm5(tr.get('x_a', -0.31), tr.get('x_b', -0.19), Vb[..., 0]))
    trd_ = g('tail_round', None)
    if trd_ and tp_:      # [tail5] a rounder tail: above z_lo the plan corners blend from the tail superellipse (n) to a rounder one (n2), pulled radially
        xc5_ = tp_['x_c']; A5_ = tp_['x_c'] - tp_['x_tip']; w5_ = tp_['w']; n1_ = tp_.get('n', 3.0); n2_ = trd_['n2']      # towards the plan centre (x_c, 0); the chassis
        u5_ = np.clip((xc5_ - Vb[..., 0]) / A5_, 0, None); v5_ = np.abs(Vb[..., 1]) / w5_                                 # below z_lo (wide rear tub) keeps the old outline
        r1_ = (u5_ ** n1_ + v5_ ** n1_) ** (1 / n1_); r2_ = (u5_ ** n2_ + v5_ ** n2_) ** (1 / n2_)
        s5_ = np.where(r2_ > 1e-9, r1_ / np.maximum(r2_, 1e-9), 1.0)                                                       # <= 1: a point on the old outline lands on the new one
        wb5_ = trd_.get('w_base', 0.0); wz_ = (wb5_ + (1 - wb5_) * sm5(trd_.get('z_lo', 0.10), trd_.get('z_hi', 0.17), Vb[..., 2])) * (1 - trd_.get('top_keep', 0.0) * sm5(trd_.get('zt0', 0.24), trd_.get('zt1', 0.28), Vb[..., 2]))
        wv_ = sm5(trd_.get('v0', 0.0), trd_.get('v1', 0.0001), v5_) if trd_.get('v0', 0.0) > 0 else 1.0                      # optional: keep the middle of the tail face (lens band) flat
        sc5_ = 1 + (s5_ - 1) * wz_ * wv_ * (u5_ > 0)
        Vb[..., 0] = xc5_ - (xc5_ - Vb[..., 0]) * sc5_; Vb[..., 1] = Vb[..., 1] * sc5_
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
import os as _os
if _os.environ.get('FORM_ONLY'):      # [fix1] fast form probe: save the faired body grid and stop
    np.savez(_os.environ['FORM_ONLY'], VL=VL, BW=BW, BP=BP, tt=tt, NB=NB, ptsW=ptsW, rk=rk); sys.exit(0)

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
if g('section', None):      # [fix2] the grid's spine ends: several columns converge on one point there; weld float32-coincident grid vertices into one index
    #                         (otherwise the wall's clean-up drops the zero-area faces and leaves a 5 mm hole with a rolled rim at the pole)
    from scipy.sparse import coo_matrix as _coo0
    from scipy.sparse.csgraph import connected_components as _cc0
    _V32 = V.astype(np.float32); _pr = []
    for _i, _j in ((0, 1), (1, 2), (0, 2)):
        _mm = (_V32[F[:, _i]] == _V32[F[:, _j]]).all(1) & (F[:, _i] != F[:, _j]); _pr.append(np.c_[F[_mm, _i], F[_mm, _j]])
    _pr = np.concatenate(_pr)
    if len(_pr):
        _, _cl = _cc0(_coo0((np.ones(len(_pr)), (_pr[:, 0], _pr[:, 1])), shape=(len(V), len(V))), directed=False)
        _rp = np.full(_cl.max() + 1, len(V)); np.minimum.at(_rp, _cl, np.arange(len(V))); F = _rp[_cl][F]
        _ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2]); print('grid pole weld: %d vertices, %d faces dropped' % (int((_rp[_cl] != np.arange(len(V))).sum()), int((~_ok).sum())), file=sys.stderr); F = F[_ok]
tgrid = np.repeat(tt[:, None], NH, 1).ravel(); sgrid = np.tile(sres, MR); hgrid = (VL[..., 2] - VL[NB][None, :, 2]).ravel()
# per-vertex attributes, interpolated whenever an edge is cut: t (0 belt .. 1 spine), s (arc length along the belt from the nose),
# hb (height above the belt), cap (1 on the closing strip under the body)
A = {'t': np.r_[tgrid, tgrid, np.zeros(2 * NH)], 's': np.r_[sgrid, sgrid, sres, sres], 'hb': np.r_[hgrid, hgrid, np.zeros(2 * NH)],
     'cap': np.r_[np.zeros(2 * MR * NH), np.ones(2 * NH)]}
# [orbit] smooth outward normal of the faired body form, carried through every cut: offsets (step at the part line, rolled
# edges) are made along it, so they cannot fold where a detail is sharper than the offset
_NL = np.cross(np.gradient(VL, axis=1), np.gradient(VL, axis=0)); _NL /= np.maximum(np.linalg.norm(_NL, axis=2, keepdims=True), 1e-12)
if _NL[NB + 5:NB + 40, :, 1].mean() < 0: _NL = -_NL
for _r, _c, _c2 in ((-1, 0, 1), (-1, -1, -2)):      # [v18c] the two grid corners where the spine meets the nose / tail centre line: both gradients lie in
    _n = _NL[-2, _c2] * np.array([1.0, 0.0, 1.0]); _NL[_r, _c] = _n / np.linalg.norm(_n)      # y = 0, so the cross product was +-y (the 0.5 mm step went sideways: a pit at x -216)
if g('section', None):      # [fix2] the spine row is a symmetry line (n_y = 0); near its two ends several columns converge on one point and the grid
    #                         gradient there is degenerate: take the normal of the row below with n_y zeroed for every spine vertex whose own normal is off
    _sp = _NL[-2] * np.array([1.0, 0.0, 1.0]); _sp /= np.maximum(np.linalg.norm(_sp, axis=1, keepdims=True), 1e-12)
    _bad = (np.abs(_NL[-1, :, 1]) > 0.02) | (_NL[-1, :, 2] < 0.9) | (np.linalg.norm(np.gradient(VL[-1], axis=0), axis=1) < 1e-5)
    _NL[-1, _bad] = _sp[_bad]; _NL[-1, :, 1] = 0.0; _NL[-1] /= np.maximum(np.linalg.norm(_NL[-1], axis=1, keepdims=True), 1e-12)
_NR = _NL * np.array([1.0, -1.0, 1.0]); _NC = np.tile([0.0, 0.0, -1.0], (2 * NH, 1))
for _k, _c in (('nx', 0), ('ny', 1), ('nz', 2)): A[_k] = np.r_[_NL[..., _c].ravel(), _NR[..., _c].ravel(), _NC[:, _c]]
def nrm_():
    N_ = np.c_[A['nx'], A['ny'], A['nz']]; return N_ / np.maximum(np.linalg.norm(N_, axis=1, keepdims=True), 1e-12)
lab = np.zeros(len(F), np.int8)
KL = {'body': 0, 'roof': 1, 'visor': 2, 'light': 3, 'lid': 4, 'accent': 5, 'glass': 6, 'core': 7, 'glow': 8, 'ring': 9}      # [orbit] core, glow; [v18] ring (sensor rims behind the lens)
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
    if SK.get('pocket_r'): arch_ = np.hypot(V[:, 0] - WX, V[:, 2] - WZ) - SK['pocket_r']      # [v18] wheel pocket: behind the tyre the core wall carries on down inside the orbit (hides the chassis), its edge an arc about the hub
    if SK.get('hem_round', 0.0) > 0:      # [fix1] D8: the hem loop's corners rounded (smooth min / max of the three fields), so the R1 roll cannot fold
        rh_ = SK['hem_round']
        _u = lambda a, b, r: np.maximum(r, np.minimum(a, b)) - np.hypot(np.maximum(r - a, 0), np.maximum(r - b, 0))      # smooth min (intersection)
        _i = lambda a, b, r: np.minimum(-r, np.maximum(a, b)) + np.hypot(np.maximum(r + a, 0), np.maximum(r + b, 0))     # smooth max (union)
        A['hem'] = np.where(V[:, 0] > 0.1, A['hb'], _u(V[:, 2] - zh_, _i(arch_, A['hb'], rh_), rh_))
    else: A['hem'] = np.where(V[:, 0] > 0.1, A['hb'], np.minimum(V[:, 2] - zh_, np.maximum(arch_, A['hb'])))
    A['hem'][A['cap'] > 0.5] = 1.0
    SEAT = SK.get('seat')
    if SEAT:      # [v18c] seated edge beside the wheels: over the cut mudguard-strut stubs of the chassis (tops z 133-135, 1.3 mm inside the skin)
        #           the lower edge rises to z_seat, where its 2.5 mm edge face + an inward seat flange sit on the stubs (all behind the tyre)
        # [fix3-struts] those 'stubs' are the tops of the drive-unit housings (4-bolt flange, top face z 152..155.0), not mudguard struts: they stay;
        #               seat.z 0.1565 (fix3_struts.json) puts the edge + flange 1.5 mm over them, the housing top is the flange's vertical stop
        zl_ = np.interp(V[:, 0], [-0.030, 0.262], [zcut(-0.03) - 0.003, zcut(0.262) - 0.003]); (xa_, xb_), (ra_, rb_) = SEAT['x'], SEAT['ramp']
        w_ = sm5(xa_ - ra_, xa_, V[:, 0]) * (1 - sm5(xb_, xb_ + rb_, V[:, 0])); m_ = (w_ > 1e-6) & (np.abs(V[:, 1]) > SEAT.get('y_min', 0.14)) & (A['cap'] < 0.5)
        A['hem'] = np.where(m_, np.minimum(A['hem'], V[:, 2] - (zl_ + (SEAT['z'] - zl_) * w_)), A['hem'])
    split('hem'); keep_ = fmean('hem') > 0; F = F[keep_]; lab = lab[keep_]

def roll_edge(name, iso, sign, r, steps=(0.1, 0.25, 0.45, 0.7, 1.0)):
    """[orbit] Quarter-round of radius r on a free edge along A[name] == iso. The surface on the side where sign * (A - iso) > 0
    turns inwards (along -n) towards the edge, so the edge reads as the rounded end of a moulded wall, not a knife cut."""
    global V
    for s_ in steps: split(name, iso + sign * r * s_)
    d = sign * (A[name] - iso); m = (d > -2.5e-7) & (d < r)          # [v18b] same tolerance as split(): vertices snapped onto the line are rolled too (no 0.8 mm spikes)
    off = np.where(m, r - np.sqrt(np.maximum(r * r - (r - np.clip(d, 0, r)) ** 2, 0.0)), 0.0); V = V - nrm_() * off[:, None]
CV = g('cover', None); gapc = CV.get('gap', 0.0006) if CV else 0.0; OB = CV.get('orbit') if CV else None
if OB:
    # [orbit] The part line between the light shell (snap-on cover) and the dark core (one-piece body) is drawn in the side view
    # from three primitives: the base line (level, = the top of the fixed bumper), the orbit (a circle concentric with the drive
    # wheel hub, the same radius as the hem arch behind the wheel) and the brow over the face (level). Concave corners between
    # them get one fillet radius. The field is a true distance in the side view, so the gap is the same width everywhere.
    Ro_, zb_, zn_, xn_, fr_ = OB.get('r', 0.128), OB.get('z_base', 0.123), OB.get('z_nose', 0.19), OB.get('x_nose', 0.15), OB.get('fillet', 0.02)
    def _uround(a, b, r):       # union of two regions given by distance fields, concave corner rounded with radius r
        return np.maximum(r, np.minimum(a, b)) - np.hypot(np.maximum(r - a, 0), np.maximum(r - b, 0))
    def _iround(a, b, r):       # [v18] intersection of two regions, convex corner rounded with radius r
        return np.minimum(-r, np.maximum(a, b)) + np.hypot(np.maximum(r + a, 0), np.maximum(r + b, 0))
    TL_ = OB.get('tail', None); PLINE = None
    if OB.get('explicit', False):      # [fix2] G4 / G2 / R2E2: the part line as an explicit 2-D curve (level lines, circle arcs, true R fillets)
        import partline as _pl; PLINE = _pl.build(OB, WX, WZ); print('part line', json.dumps(PLINE['report']), file=sys.stderr)
    def cvf():
        x_, z_ = V[:, 0], V[:, 2]
        if PLINE is not None: return np.where(A['cap'] > 0.5, -1.0, _pl.sdist(PLINE, x_, z_))
        a_ = z_ - zb_; b_ = np.hypot(x_ - WX, z_ - WZ) - Ro_; c_ = np.maximum(xn_ - x_, z_ - zn_)
        f_ = _uround(_uround(a_, b_, fr_), c_, fr_)
        if TL_:     # [v18] tail = nose: the part line rises behind the wheel along a line raked like the tail face, then runs level round the tail
            k_ = TL_['k']; d_ = ((x_ - TL_['x']) - k_ * (z_ - zb_)) / np.hypot(1.0, k_)       # signed distance to the rise (> 0 ahead of it)
            e_ = _iround(d_, z_ - TL_.get('z', zn_), TL_.get('fillet_top', fr_))               # rear block: behind the rise and below the tail brow
            f_ = _uround(f_, e_, TL_.get('fillet', fr_))
        return np.where(A['cap'] > 0.5, -1.0, f_)
    A['cv'] = cvf(); fine('cv', cvf, -0.002, gapc + 0.003, CV.get('fine', 0.0005)); split('cv', 0.0); split('cv', gapc)
    fc_ = fmean('cv'); kp_ = ~((fc_ > 0) & (fc_ < gapc)); F = F[kp_]; lab = lab[kp_]
    rc_, rb_, st_ = OB.get('roll', 0.0012), OB.get('roll_core', 0.0005), OB.get('step', 0.001)
    if rb_ and not WALL_: roll_edge('cv', 0.0, -1, rb_)        # [v18c] with a real wall the edge is rolled once, by shell_wall (loop_r 'core_seam')
    if OB.get('zone_sink', False):
        # [v18] beside the wheels the shell may not stand proud of the 171 mm envelope: there the shell stays on the body form
        # and the core is sunk by the step instead, only in a 6 mm land along the part line (chassis post 10, 15 mm below the
        # line, keeps its room). Outside the wheel zone the shell is proud as before. The visible joint is the same everywhere.
        zxr_ = OB.get('zone_x_ramp', 0.035)      # [fix2] F-S1: ramp length in x (fix1 0.035 = a 0.5 mm S-wave of +-2.5 1/m on the flanks; 0.08 -> +-0.45 1/m)
        zw_ = sm5(WX - 0.127 - zxr_, WX - 0.127, V[:, 0]) * (1 - sm5(WX + 0.127, WX + 0.127 + zxr_, V[:, 0]))
        if OB.get('zone_by_normal', False):      # [fix1] D4: only the side-facing surfaces beside the wheels (|n_y| high) trade the proud shell for a sunk core;
            #                                    the roof stays proud everywhere (an x-only zone put a 0.5 mm ramp across the whole roof and lid at x -43..-8 and 246..281)
            zw_ = zw_ * sm5(OB.get('zone_ny', [0.55, 0.9])[0], OB.get('zone_ny', [0.55, 0.9])[1], np.abs(nrm_()[:, 1]))
        ld_ = sm5(-0.008, -0.002, A['cv'])
        off_ = np.where(A['cv'] > gapc * 0.5, st_ * (1 - zw_), -st_ * zw_ * ld_)
    else: off_ = st_ * (A['cv'] > gapc * 0.5)
    V = V + nrm_() * off_[:, None]          # the shell stands one step proud of the core all round
    # (proud shell rather than a sunk core: beside the wheels the core wall must keep its full width over chassis post 10)
    if rc_ and not WALL_: roll_edge('cv', gapc, 1, rc_)        # [v18c] with a real wall: shell_wall (loop_r 'cover_seam')
    lab[(fmean('cv') < gapc * 0.5) & (lab == 0)] = KL['core']
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
if g('nose_profile', None) or g('nose_rake', None):     # side profile of the nose: [[z, dx], ...] moves the front face forwards (+) / back (-) by height -> raked, wedge-like nose
    _nz = np.zeros(len(V))
    if g('nose_profile', None):
        npf = np.array(sorted(g('nose_profile', [])))
        if g('nose_smooth', False):     # [orbit] smooth (PCHIP) rake: a piecewise-linear rake puts a slope kink in the face at every knot (seen as a broken reflection)
            _nz = PchipInterpolator(npf[:, 0], npf[:, 1])(np.clip(V[:, 2], npf[0, 0], npf[-1, 0]))
        else: _nz = np.interp(V[:, 2], npf[:, 0], npf[:, 1])
    nr_ = g('nose_rake', None)
    if nr_:     # [v18] planar rake of the front face: d0 at the foot, leaning back at atan(k) between z0 and z1 (the same k as the tail face and the tail rise)
        _nz = _nz + nr_.get('d0', 0.0) - nr_['k'] * rake_ramp(V[:, 2], nr_['z0'], nr_['z1'], nr_.get('b', 0.012))
    V[:, 0] += _nz * sm5(g('nose_x0', 0.255), g('nose_x1', 0.30), V[:, 0]) * (1 - A['cap'])

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
pbz = g('port_bezel', None); PSL_ = g('port_slot', None); PORTCUT = ports
if PSL_:      # [fix2] A5 / L7: ONE stadium service slot centred on y = 0 under the band (both fixed ports inside it, the 4.3 mm port offset goes inside)
    PORTCUT = shapely.LineString([(-PSL_['y'] + (PSL_['z1'] - PSL_['z0']) / 2, 0.5 * (PSL_['z0'] + PSL_['z1'])), (PSL_['y'] - (PSL_['z1'] - PSL_['z0']) / 2, 0.5 * (PSL_['z0'] + PSL_['z1']))]).buffer((PSL_['z1'] - PSL_['z0']) / 2, quad_segs=24)
    P3_ = g('ports3', None); PORTS3_REP = None      # [fix3-ports] mirror-symmetric windows (portskit.py); the openings are then sized for the dock pins, not the old-shell boxes
    assert P3_ or PORTCUT.contains(ports.buffer(0.0004)), 'port slot does not contain the ports'      # [fix3-ports] (portskit asserts its own pin envelope)
fn_ = lambda: np.maximum(sd(PORTCUT, V[:, 1], V[:, 2]), 0.22 - V[:, 0]); A['pt'] = fn_(); fine('pt', fn_, -0.001, 0.004, g('port_fine', 0.0006)); split('pt', 0.0)
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
LT_ = g('lights', None); LENS_ = LT_.get('lens') if (LT_ and LT_.get('on', True)) else None      # [fix3-lights] separate PC lens per band (lightkit.py)
if LENS_: A['lns'] = np.ones(len(V))      # [fix3-lights] min of the band outline fields: < 0 inside a lens outline (cut exactly at +- gap / 2)
for bd_ in g('bands', []):
    # [v18] one light/sensor motif, the same at the nose and the tail: a flush smoked lens (one continuous surface, stadium ends
    # R = h/2) in the dark core face just under the brow, held by a fine groove (its part line). Behind it: straight light segments
    # (stadiums, level, concentric ends = one constant margin) and, at the centre, the sensor 'pupil' (front) or a dot (rear).
    sg_b = 1.0 if bd_.get('side', 'front') == 'front' else -1.0; zc_b = bd_['z']; h_b = bd_['h']; w_b = bd_['w']
    bp_ = shapely.box(-w_b / 2, zc_b - h_b / 2, w_b / 2, zc_b + h_b / 2).buffer(-h_b / 2 + 1e-6).buffer(h_b / 2 - 1e-6, quad_segs=48)
    fb_ = lambda bp_=bp_, sg_b=sg_b: np.maximum(sd(bp_, V[:, 1], V[:, 2]), 0.22 - sg_b * V[:, 0])
    gw_ = bd_.get('groove_w', 0.0005); gd_ = bd_.get('groove_d', 0.0003)
    A['bd'] = fb_(); fine('bd', fb_, -gw_ - 0.0015, gw_ + 0.0015, bd_.get('fine', 0.00025))
    if LENS_: A['lns'] = np.minimum(A['lns'], A['bd']); gd_ = 0.0      # [fix3-lights] no groove: the lens is its own part, its joint is a real gap
    if gd_ > 0 and bd_.get('groove_profile') == 'smooth':
        # [fix1] L3 / D2: the parabola met the face with a 67 deg crease (right at smooth_deg 70: some vertices hard, some soft = a saw-tooth glint).
        #        Now a quintic-smoothstep section (zero slope and curvature at the lips, rounded lips ~R0.04) cut on 15 iso-lines, so every row of the
        #        groove is an exact iso-line of the lens outline and the normals change only across the groove, never along it.
        w2o_ = gw_ / 2; us_ = np.array(bd_.get('groove_rows', [0.0, 0.2, 0.4, 0.55, 0.7, 0.8, 0.9, 1.0])); cuts('bd', sorted(set(np.r_[-us_, us_] * w2o_)))
        u_ = np.clip(1 - np.abs(A['bd']) / w2o_, 0, 1); dep_ = gd_ * u_ ** 3 * (u_ * (u_ * 6 - 15) + 10)
        V = V - nrm_() * (dep_ * (sg_b * V[:, 0] > 0.22))[:, None]
    elif gd_ > 0:        # the groove: a U section centred on the lens outline, pushed in along the body normal
        cuts('bd', [-gw_ / 2, -gw_ / 4, 0.0, gw_ / 4, gw_ / 2]); u_ = np.clip(A['bd'] / (gw_ / 2), -1, 1)
        V = V - nrm_() * (gd_ * (1 - u_ * u_) * (np.abs(A['bd']) < gw_ / 2) * (sg_b * V[:, 0] > 0.22))[:, None]
    paint('bd', bd_.get('kind', 'visor'), iso=0.0)
    for ls_ in bd_.get('lights', []):        # light segments [y0, y1] (mirrored), height hl, stadium ends
        hl_ = ls_.get('h', 0.004); lp_ = shapely.box(ls_['y0'], zc_b - hl_ / 2, ls_['y1'], zc_b + hl_ / 2).buffer(-hl_ / 2 + 1e-7).buffer(hl_ / 2 - 1e-7, quad_segs=24)
        if ls_.get('mirror', True): lp_ = shapely.union(lp_, affinity.scale(lp_, -1, 1, origin=(0, 0)))
        fl_ = lambda lp_=lp_, sg_b=sg_b: np.maximum(sd(lp_, V[:, 1], V[:, 2]), 0.22 - sg_b * V[:, 0]); A['bl'] = fl_()
        fine('bl', fl_, -0.0008, 0.0008, ls_.get('fine', 0.0003)); paint('bl', ls_.get('kind', 'light'))
    for el_ in bd_.get('elements', []):      # round elements on the band axis: disc (r) or ring (r, w)
        cy_ = el_.get('y', 0.0); cz_ = el_.get('z', zc_b)
        fe_ = lambda cy_=cy_, cz_=cz_, el_=el_, sg_b=sg_b: np.maximum(np.hypot(V[:, 1] - cy_, V[:, 2] - cz_) - el_['r'], 0.22 - sg_b * V[:, 0]); A['be'] = fe_()
        wr_ = el_.get('w', 0.0)
        fine('be', fe_, -max(wr_, 0.0005) - 0.0006, max(wr_, 0.0005) + 0.0006, el_.get('fine', 0.00015))
        if wr_: paint('be', el_['kind'], iso=wr_ / 2, lo=-wr_ / 2)
        else: paint('be', el_['kind'])
LENS_PATCH = []      # [fix3-lights]
if LENS_ and g('bands', []):      # [fix3-lights]
    # [fix3-lights] the smoked lens is a separate PC part fitted from inside: the body gets an opening along the lens outline + gap / 2 (its edge is
    #               rolled by shell_wall, loop 'lens_opening'), the lens keeps the painted outer faces inside the outline - gap / 2 (lightkit.py makes it a      # [fix3-lights]
    #               closed 2.5 mm solid). The old 0.6 x 0.3 groove becomes this real 0.4 mm gap.      # [fix3-lights]
    glh_ = 0.5 * LENS_.get('gap', 0.0004); split('lns', -glh_); split('lns', glh_); fl_ = fmean('lns'); cx_ = V[F][:, :, 0].mean(1)      # [fix3-lights]
    for bd_ in g('bands', []):      # [fix3-lights]
        sg_b = 1.0 if bd_.get('side', 'front') == 'front' else -1.0; m_ = (fl_ < -glh_) & (sg_b * cx_ > 0.22); ui_ = np.unique(F[m_]); rm_ = -np.ones(len(V), np.int64); rm_[ui_] = np.arange(len(ui_))      # [fix3-lights]
        LENS_PATCH.append(dict(side=bd_.get('side', 'front'), v=V[ui_].copy(), f=rm_[F[m_]], lab=lab[m_].copy(), n=nrm_()[ui_].copy(), lns=A['lns'][ui_].copy(), band=bd_))      # [fix3-lights]
    cut_ = (fl_ < glh_) & (np.abs(cx_) > 0.22) & (fl_ < 0.5); F = F[~cut_]; lab = lab[~cut_]      # [fix3-lights]
    if WALL_: WALL_['loop_r'] = list(WALL_.get('loop_r') or []) + [dict(name='lens_opening', attr='lns', iso=glh_, part=0, r=LENS_.get('roll_body', 0.0002))]      # [fix3-lights]
    print('lens cut: %d lens faces, %d removed' % (sum(len(p_['f']) for p_ in LENS_PATCH), int(cut_.sum())), file=sys.stderr)      # [fix3-lights]
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
hole = fmean('pt') < 0; SL = Bag(); hole_pts = []
for lp in loops_of(F[hole]):
    hole_pts.append(V[lp].copy())                                                                  # [v18c] sleeves are rebuilt from the edge face of the wall
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
if g('nose', None) and g('nose', {}).get('plinth', True):      # [fix1] L2 / D1: nose.plinth false = no separate cap plate (the core's own sole sits over the bumper)
    fo = [(x, y) for x, y in zip(xs, yo_orig) if x >= 0.262 and np.isfinite(y)]
    plinth(Polygon([(0.262, -fo[0][1])] + [(x, -y) for x, y in fo] + [(K['x_nose'], 0.0)] + [(x, y) for x, y in fo[::-1]] + [(0.262, fo[0][1])]).buffer(0).buffer(-0.003), g('nose', {}).get('cap_z', 0.1228), g('nose', {}).get('tiers', []), 0.0015)
if g('waist', None):
    ro = [(x, y) for x, y in zip(xs, yo_orig) if x <= -0.03 and np.isfinite(y)]          # tail -> wheel
    plinth(Polygon([(K['x_tail'], 0.0)] + ro + [(-0.03, ro[-1][1]), (-0.03, -ro[-1][1])] + [(x, -y) for x, y in ro[::-1]]).buffer(0).buffer(-0.007, join_style=1).buffer(0.006, join_style=1), 0.1347, g('waist', {}).get('tiers', []))      # corners rounded, 1 mm inside the skirt edge: nothing pokes through the body
CF_ = g('nose', {}).get('corner_floor') if g('nose', None) else None
if CF_:
    # [fix1] L2 / D1: instead of the 1.5 mm plate ON the bumper (pointed tips, 48 mm beyond the nose), a dark floor INSIDE the opening between the
    #        narrowed nose, the bumper's inner edge and the tyre, 2-3 mm below the bumper crest: it closes the see-through without standing on or
    #        beyond the bumper (region = behind the bumper arc, minus the bumper's footprint + 0.5 mm; chassis >= 7 mm away, tyre >= 13 mm).
    from shapely import affinity as _aff; import lidmech
    k_ = np.load('keep.npz'); Pb_ = k_['v'][k_['f'][k_['lab'] == 7]]; cb_ = Pb_.mean(1); FLV_, FLF_, fo_ = [], [], 0
    if CF_.get('v2'):
        # [fix2] F-M1 / R2E7 / G8 / A9: the floor is ONE closed 2.5 mm solid per side, mirror-symmetric (built from the intersection of the left
        #        region and the mirrored right one), never past the bumper's rear end or outer edge (clipped to the bumper's own plan hull - 1 mm),
        #        >= 1.5 mm from the bumper (travel), every free edge rolled R0.5. Its inner edge runs 2 mm in under the nose wall and three ribs tie it
        #        to the wall's foot: an integral flange of the one-piece body, not a loose plate.
        fpg_ = Polygon(np.r_[BW[:, :2], (BW[:, :2] * [1, -1])[::-1]]).buffer(0)          # body footprint at the belt
        regs_ = []
        for sg_ in (1, -1):
            mb_ = (Pb_[:, :, 2].max(1) > CF_.get('z_bump', 0.1105)) & (cb_[:, 0] > 0.235) & (cb_[:, 1] * sg_ > 0.10)
            bp0_ = shapely.union_all(shapely.make_valid(shapely.polygons(Pb_[mb_][:, :, :2]))); bp2_ = bp0_.buffer(CF_.get('clear', 0.0015))
            shd_ = shapely.union_all([_aff.translate(bp0_, -dx_, 0) for dx_ in np.arange(0, 0.07, 0.0015)])
            y0_, y1_ = sorted((sg_ * CF_.get('y_min', 0.12), sg_ * 0.23))
            R_ = shapely.box(0.2, y0_, 0.31, y1_).intersection(shd_).intersection(bp0_.convex_hull.buffer(-CF_.get('hull_in', 0.001)))
            R_ = R_.union(R_.buffer(0.006).intersection(fpg_.difference(fpg_.buffer(-CF_.get('under', 0.002))))).difference(bp2_)
            R_ = R_.buffer(-0.0012).buffer(0.0012)
            if sg_ < 0: R_ = _aff.scale(R_, 1, -1, origin=(0, 0))
            regs_.append(R_)
        RS_ = regs_[0].intersection(regs_[1]); RS_ = max(shapely.get_parts(RS_), key=lambda q_: q_.area) if not RS_.is_empty else RS_
        RS_ = RS_.simplify(0.0002).buffer(-0.001).buffer(0.001, quad_segs=12)          # R >= 1 at every corner
        zt_ = CF_.get('z_top', 0.1135); tf_ = CF_.get('t', 0.0025); FLR_ = Bag(); flrep_ = []
        for sg_ in (1, -1):
            pg_ = RS_ if sg_ > 0 else _aff.scale(RS_, 1, -1, origin=(0, 0))
            v_, f_ = lidmech.extrude(pg_, zt_ - tf_, zt_, lidmech.XY(), bev=CF_.get('roll', 0.0005), nb=3, step=0.001)
            FLR_.add(v_, f_, 0)
            # ribs under the nose wall: from the floor top to 0.5 mm into the wall's foot, at three stations along the floor's inner edge
            inn_ = pg_.intersection(fpg_)
            if not inn_.is_empty:
                bx_ = inn_.bounds
                for fx_ in (0.2, 0.5, 0.8):
                    xr_ = bx_[0] + (bx_[2] - bx_[0]) * fx_; ln_ = shapely.LineString([(xr_, -0.3), (xr_, 0.3)]).intersection(inn_)
                    if ln_.is_empty: continue
                    yy_ = np.array(ln_.coords)[:, 1] if ln_.geom_type == 'LineString' else np.concatenate([np.array(q_.coords)[:, 1] for q_ in shapely.get_parts(ln_)])
                    ya_, yb_ = yy_.min(), yy_.max()
                    us0_ = np.zeros(len(V), bool); us0_[F.ravel()] = True
                    nb_ = np.nonzero(us0_ & (np.abs(V[:, 0] - xr_) < 0.003) & (V[:, 1] * sg_ > min(abs(ya_), abs(yb_)) - 0.004) & (V[:, 1] * sg_ < max(abs(ya_), abs(yb_)) + 0.004) & (V[:, 2] > zt_))[0]
                    if not len(nb_): continue
                    zw_ = float(V[nb_, 2].min()) + 0.0005
                    if zw_ - zt_ > 0.008: continue      # no wall foot right above this station
                    FLR_.add(*box((xr_, 0.5 * (ya_ + yb_), 0.5 * (zt_ - 0.0002 + zw_)), (0.0012, max(yb_ - ya_, 0.001), zw_ - zt_ + 0.0002)), 0); flrep_.append(round((zw_ - zt_) * 1000, 2))
        fv_, ff_, _ = FLR_.out(); FLV_ = [fv_]; FLF_ = [ff_]
        print('nose floor v2: %d faces, region %.0f mm2 per side, x %.1f..%.1f, |y| %.1f..%.1f, ribs %s' % (len(ff_), RS_.area * 1e6, RS_.bounds[0] * 1000, RS_.bounds[2] * 1000, RS_.bounds[1] * 1000, RS_.bounds[3] * 1000, flrep_), file=sys.stderr)
    for sg_ in ((1, -1) if not CF_.get('v2') else ()):
        mb_ = (Pb_[:, :, 2].max(1) > CF_.get('z_bump', 0.1105)) & (cb_[:, 0] > 0.235) & (cb_[:, 1] * sg_ > 0.10)
        bp2_ = shapely.union_all(shapely.make_valid(shapely.polygons(Pb_[mb_][:, :, :2]))).buffer(CF_.get('clear', 0.0005))
        shd_ = shapely.union_all([_aff.translate(bp2_, -dx_, 0) for dx_ in np.arange(0, 0.07, 0.0015)])
        y0_, y1_ = sorted((sg_ * CF_.get('y_min', 0.12), sg_ * 0.23))
        R_ = shapely.box(CF_.get('x_min', 0.245), y0_, 0.31, y1_).intersection(shd_).difference(bp2_).buffer(-0.0008).buffer(0.0008)
        for pg_ in [q_ for q_ in shapely.get_parts(R_) if q_.area > 2e-5]:
            pg_ = pg_.simplify(0.0002); ring_ = lidmech.resample(pg_, 0.001); pts_, tri_ = lidmech.tri_polygon(Polygon(ring_))
            zt_, zb2_ = CF_.get('z_top', 0.113), CF_.get('z_top', 0.113) - CF_.get('t', 0.001); n_ = len(pts_)
            Vt_ = np.c_[pts_, np.full(n_, zt_)]; Vb_ = np.c_[pts_, np.full(n_, zb2_)]
            sv2_, sf2_ = stack([np.c_[ring_, np.full(len(ring_), zt_)], np.c_[ring_, np.full(len(ring_), zb2_)]], closed=True)
            FLV_ += [Vt_, Vb_, sv2_]; FLF_ += [tri_ + fo_, tri_[:, ::-1] + fo_ + n_, sf2_ + fo_ + 2 * n_]; fo_ += 2 * n_ + len(sv2_)
    if FLV_: base_ = sum(len(v_) for v_ in PV); PV.append(np.concatenate(FLV_)); PF.append(np.concatenate(FLF_) + base_); print('nose corner floor: %d faces' % len(np.concatenate(FLF_)), file=sys.stderr)
if PV: PLV = np.concatenate(PV); extra.update(plinth_v=PLV, plinth_f=np.concatenate(PF))
if PV and (CV or {}).get('orient_kits'): import coverkit4 as _ck4; extra['plinth_f'] = _ck4.orient(PLV, extra['plinth_f'], 'nosefloor').astype(extra['plinth_f'].dtype)      # [fix4-cover2] G5: nose corner floor wound consistently / outward

# ---- lid (v18b): its own moulded part, cut out of the roof exactly with a real gap, with the v18 joint (the same as the shell /
#      core part line): 0.8 gap, both edges rolled R 0.8 along the faired body normal (roll_edge), and under each roll a dark wall
#      (lid: t_lid deep, opening: t_cover deep), slightly drafted (undercut_deg), in the one dark interior colour.
#      Under it (lidmech.py): virtual-pivot arc-track hinge with soft-open and a lug stop, push-push latch, continuous bulb seal,
#      drained gutter, well round the control panel. Built closed; the renderer / viewer open the lid group by a pure rotation
#      about lid_pivot (axis parallel to Y).
#      lid.cover_wall: true = the opening's dark wall and the cover's inside face round it are built here (kitf part 'edge' / 'cover');
#      false = the shell wall module (stage C) builds the opening wall from the rolled hole loop; then only the well moulding is added
#      here and the gutter registers on a vertical wall at d = -gap.
import lidmech
L_ = g('lid', {'shape': 'rrect', 'cx': -0.047, 'length': 0.2, 'width': 0.15, 'r': 0.025})
LP = lidmech.params(dict({'kit_res': float(g('stride', 1))}, **{k_: v_ for k_, v_ in L_.items() if k_ not in ('shape', 'cx', 'length', 'width', 'struts', 'roll', 'fine')}))
cx, ln, wd = L_['cx'], L_['length'], L_['width']; hx, hy = ln / 2, wd / 2
if L_['shape'] == 'rrect': poly = shapely.box(cx - hx, -hy, cx + hx, hy)
elif L_['shape'] == 'hex':
    k = L_.get('cut', 0.035); poly = Polygon([(cx - hx, -hy + k), (cx - hx + k, -hy), (cx + hx - k, -hy), (cx + hx, -hy + k), (cx + hx, hy - k), (cx + hx - k, hy), (cx - hx + k, hy), (cx - hx, hy - k)])
else: poly = affinity.scale(Point(cx, 0).buffer(1, 48), hx, hy)
r = L_.get('r', 0.02); poly = poly.buffer(-r).buffer(r, quad_segs=24); ring_xy = np.array(poly.exterior.coords)[:-1]; LP['r'] = r
gapw = LP['gap']; OPEN = LP['open_deg']; rl_ = float(L_.get('roll', 0.0))
if rl_ > 0 and 'edge_r' not in L_: LP['edge_r'] = rl_                       # the roll radius is the joint's edge radius (lid and opening)
if rl_ > 0 and 'hole_r' not in L_: LP['hole_r'] = rl_
lidf_ = lambda: np.maximum(sd(poly, V[:, 0], V[:, 1]), 0.5 * (0.35 - A['t']))
A['lid'] = lidf_(); fine('lid', lidf_, -0.0025, gapw + 0.0025, L_.get('fine', 0.0005))
split('lid', 0.0); split('lid', gapw)
used = np.zeros(len(V), bool); used[F.ravel()] = True; tm_ = used & (A['t'] > 0.25) & (A['cap'] < 0.5)
zi = LinearNDInterpolator(V[tm_][:, :2], V[tm_][:, 2])                                            # A-surface before the edges are rolled
if rl_ > 0: roll_edge('lid', 0.0, -1, rl_)                                                         # [orbit] the same rolled edge as the shell's part line
if rl_ > 0 and not WALL_: roll_edge('lid', gapw, 1, rl_)                                          # [v18c] opening edge: rolled by shell_wall when the wall is on
fl = fmean('lid'); lab[fl < 0] = 4
lid_loop = max(loops_of(F[fl < 0]), key=len); hole_loop = max(loops_of(F[fl < gapw]), key=len)
gapf = (fl > 0) & (fl < gapw); F = F[~gapf]; lab = lab[~gapf]
STOPMARK = None; STOPPAD = None
if L_.get('stop_pad'):
    # [fix2] A2 / G5 / R2E4: ONE graphic on the roof instead of two parallel strokes. A flush STOP pad on the lid (2K, accent colour, a white
    #        'stop' square, d = 2 r) over the STOP key, ringed by the status halo (clear 2K ring: lit through the lid by a light-guide boss on the
    #        lid underside from an LED on the control panel while the lid is closed - no harness to the lid or the cover). Pressing the pad
    #        pushes the lid into the latch over-travel: a plunger under the pad presses the real STOP key (fixed hardware), then the push-push
    #        latch releases and the lid opens onto the controls.
    sp_ = L_['stop_pad']; xp_ = float(sp_.get('x', 0.018)); rp_ = sp_.get('r', 0.009); hg_ = sp_.get('halo_gap', 0.0015); hw_ = sp_.get('halo_w', 0.0012); ic_ = sp_.get('icon', 0.0055)
    STOPPAD = dict(x=xp_, r=rp_, hg=hg_, hw=hw_, icon=shapely.box(xp_ - ic_ / 2, -ic_ / 2, xp_ + ic_ / 2, ic_ / 2).buffer(-sp_.get('icon_r', 0.001)).buffer(sp_.get('icon_r', 0.001), quad_segs=12))
    fsp_ = lambda: np.hypot(V[:, 0] - xp_, V[:, 1]) - rp_; A['sm'] = fsp_(); fine('sm', fsp_, -rp_ - 0.0005, hg_ + hw_ + 0.0008, sp_.get('fine', 0.0003), extra=lambda: lab == 4)
    for v_ in (0.0, hg_, hg_ + hw_): split('sm', v_)
    fic_ = lambda: sd(STOPPAD['icon'], V[:, 0], V[:, 1]); A['si2'] = fic_(); fine('si2', fic_, -0.0008, 0.0008, 0.0002, extra=lambda: lab == 4); split('si2', 0.0)
elif L_.get('stop_mark'):
    # [fix1] E4: the flush same-colour lid gets a flush orange 2K mark over the push-latch point - it shows that STOP is under the lid and where to
    #        press (the first press of the latch is also the stop switch: blade and drive stop before the lid rises). One stroke of the light family:
    #        a stadium w wide (R w/2), len long, across the car at the striker.
    sm_ = L_['stop_mark']; xm_ = float(sm_.get('x', cx + hx + lidmech.DEFAULTS['latch_dx'])); wm_ = sm_.get('w', 0.004); lm_ = sm_.get('len', 0.024)
    STOPMARK = shapely.box(xm_ - 1e-6, -lm_ / 2 + wm_ / 2, xm_ + 1e-6, lm_ / 2 - wm_ / 2).buffer(wm_ / 2, quad_segs=24)
    fsm_ = lambda: sd(STOPMARK, V[:, 0], V[:, 1]); A['sm'] = fsm_(); fine('sm', fsm_, -0.0008, 0.0008, sm_.get('fine', 0.00025), extra=lambda: lab == 4); split('sm', 0.0)
STOPKEY = None      # [fix3-stop]
if L_.get('stop_key'):      # [fix3-stop] raised, parallel-guided STOP key on the lid (stopkit.py, EN 50636-2-107/A3 20.101.4): the lid skin only gets the
    import stopkit           # [fix3-stop] stem bore (under the halo boss); underside hole + rib keep-outs go to lidmech through LP; the parts are built after it
    import stop4; SKV4_ = bool(L_['stop_key'].get('v4'))      # [fix4-stop2] v4 key (stop4.py): skin cut = the socket outline (r 24.9), underside holes = stem bore + light window
    STOPKEY = (stop4 if SKV4_ else stopkit).layout(L_['stop_key'], zi); xs3_, ys3_, rb3_ = STOPKEY.get('skin_cut', STOPKEY['skin_holes'][0])      # [fix3-stop] [fix4-stop2]
    fsk_ = lambda: np.hypot(V[:, 0] - xs3_, V[:, 1] - ys3_) - rb3_; A['sk3'] = fsk_(); fine('sk3', fsk_, -0.002, 0.002, 0.0004, extra=lambda: lab == 4); split('sk3', 0.0)      # [fix3-stop]
    k3_ = ~((lab == 4) & (fmean('sk3') < 0)); F = F[k3_]; lab = lab[k3_]; LP['skin_holes'] = STOPKEY['skin_holes']; LP['rib_keepout'] = STOPKEY['rib_keepout']      # [fix3-stop]
def edge_face(lp, depth, shift):
    """Wall face from the boundary loop lp (bottom of the rolled edge) down to depth under the A-surface, its bottom moved by shift towards
    the lid centre (draft / undercut: the face leans back under the roll and faces down). Unpainted dark plastic (kit), so the shut line
    reads as two rolled highlights with a dark wall between them, like the shell / core part line. Returns V, F (top ring = the skin loop)."""
    p_ = V[lp]; nn_ = lidmech.lnormals(p_[:, :2])
    if not poly.contains(Point(*(p_[0, :2] + nn_[0] * (gapw + 0.0003)))): nn_ = -nn_                    # nn_: horizontal normal into the lid
    xy_ = p_[:, :2] + nn_ * shift; mid_ = 0.5 * (p_[:, :2] + xy_)
    if LP.get('h4'): import prismkit as _pk; xy_ = _pk.defold(p_[:, :2], xy_); mid_ = 0.5 * (p_[:, :2] + xy_)      # [fix4-hinge2] no fold of the offset ring (closed lid at full resolution)
    R_ = [p_, np.c_[mid_, 0.5 * (p_[:, 2] + zi(xy_) - depth)], np.c_[xy_, zi(xy_) - depth]]
    return stack(R_, closed=True)
ul_, uc_ = lidmech.undercuts(LP)
EWs = [(lid_loop, LP['t_lid'] - LP['chamfer'], ul_, 'm')] + ([(hole_loop, LP['t_cover'], -uc_, 'f')] if LP['cover_wall'] else [])
k_ = int(g('stride', 1)); EW = [(edge_face(lp_[::k_] if (k_ > 1 and not (m_ == 'm' and LP.get('h4'))) else lp_, d_, s_), m_) for lp_, d_, s_, m_ in EWs]     # light web model: every 2nd point      # [fix4-hinge2] h4: the lid edge keeps every skin point (closed lid)
if LP.get('h4'):      # [fix4-hinge2] ONE closed lid moulding (G5): the underside chamfer starts on the edge wall's bottom ring, every hole in the skin gets a bore wall
    import prismkit as _pk0      # [fix4-hinge2]
    pe_ = V[lid_loop]; ne_ = lidmech.lnormals(pe_[:, :2])      # [fix4-hinge2]
    if not poly.contains(Point(*(pe_[0, :2] + ne_[0] * (gapw + 0.0003)))): ne_ = -ne_      # [fix4-hinge2] (as edge_face)
    xe_ = _pk0.defold(pe_[:, :2], pe_[:, :2] + ne_ * ul_); LP['edge_ring'] = np.c_[xe_, zi(xe_) - (LP['t_lid'] - LP['chamfer'])]      # [fix4-hinge2] = the edge face's bottom ring
    LS_ = sorted(loops_of(F[lab == 4]), key=len); LP['skin_hole_loops'] = [V[l_] for l_ in LS_[:-1]]      # [fix4-hinge2]
SKB_ = None; SKMM_ = None      # [fix3-int]
if STOPKEY is not None:      # [fix3-int] the STOP parts are built BEFORE the lid kit (stopkit.build needs only LP and zi): their mass from the closed meshes enters
    if SKV4_: LP['stop4_rib_x'] = poly.bounds[2] - LP['ring_off'][0]; LP['halo_panel_z'] = float(((LT_ or {}).get('halo') or {}).get('panel_z', 0.25242))      # [fix4-stop2] front ring rib (the light post ties into it), panel strip z
    SKB_ = (stop4 if SKV4_ else stopkit).build(L_['stop_key'], zi, LP, None); SKMM_ = (stop4.mass_mesh4 if SKV4_ else stopkit.mass_mesh)(SKB_, LP['t_lid']); LP['extra_mass'] = SKMM_['points']      # [fix3-int] the ONE lid balance (lidmech mass / gravity, hinge h3 spring)  [fix4-stop2]
    HLO_ = (LT_ or {}).get('halo', {}) if LENS_ else {}      # [fix3-int] status halo = the STOP key's opal boss, fed by lightkit's inclined pipe from the panel emitter:
    if HLO_ and HLO_.get('on', True):      # [fix3-int] the centre stub rib (front centre rib, cut by the STOP keep-out to x 17..23 with a free inner end) is in its path -> kept out
        xr1_ = poly.bounds[2] - LP['ring_off'][0] - LP['rib_t'] / 2 - 0.0003      # [fix3-int] up to the inner face of the front ring rib (offset ring_off[0]), which stays whole
        LP['rib_keepout'] = shapely.union_all([LP['rib_keepout'], shapely.box(STOPKEY['xk'] + 0.010, STOPKEY['yk'] - 0.0026, xr1_, STOPKEY['yk'] + 0.0026)])      # [fix3-int]
LK = lidmech.build(LP, poly, zi, '.')
kmv, kmf, kml, kmp = LK['kitm']; kfv, kff, kfl, kfp = LK['kitf']; LIDREP = LK['report']
for (ev_, ef_), side_ in EW:                                                                         # edge walls of the shut line join the kits
    if side_ == 'm': kmf = np.concatenate([kmf, ef_ + len(kmv)]); kmv = np.concatenate([kmv, ev_]); kml = np.r_[kml, np.full(len(ef_), lidmech.K_D, np.int8)]; kmp = np.r_[kmp, np.full(len(ef_), lidmech.PM['edge'], np.int16)]
    else: kff = np.concatenate([kff, ef_ + len(kfv)]); kfv = np.concatenate([kfv, ev_]); kfl = np.r_[kfl, np.full(len(ef_), lidmech.K_D, np.int8)]; kfp = np.r_[kfp, np.full(len(ef_), lidmech.PF['edge'], np.int16)]
if LP.get('h4'):      # [fix4-hinge2] the lid composite (skin + kitm underside / edge) wound consistently and outward; only kitm faces are flipped
    import prismkit as _pk; lm_ = np.isin(kmp, [lidmech.PM['underside'], lidmech.PM['edge']]); fs_ = F[lab == 4]      # [fix4-hinge2]
    _, fo_ = _pk.orient(np.r_[V, kmv], np.r_[fs_, kmf[lm_] + len(V)])      # [fix4-hinge2]
    LIDREP['lid_solid_skin_faces_flipped'] = int((fo_[:len(fs_)] != fs_).any(1).sum()); kmf = kmf.copy(); kmf[lm_] = fo_[len(fs_):] - len(V)      # [fix4-hinge2]
extra.update(kitm_v=kmv, kitm_f=kmf, kitm_lab=kml, kitm_part=kmp, kitf_v=kfv, kitf_f=kff, kitf_lab=kfl, kitf_part=kfp,
             strut_n=np.zeros((0, 3)), strut_m=np.zeros((0, 3)), lid_pivot=np.array(LK['Q']), lid_open=np.array([OPEN]), lid_hinge_y=np.array([LK['yc']]),
             lid_latch_x=np.array([LIDREP['latch_x_mm'] / 1000.0]))
if 'pinion_axis' in LK: extra.update(lid_pinion=np.array(LK['pinion_axis']))      # [fix4-hinge2] pinion axis x, z (m) + gear ratio: it turns -ratio x the lid angle (QA tools, renderer)
print('lid: Q', np.round(LK['Q'], 5), 'hinge_y', round(LK['yc'], 4), 'kitm', kmf.shape, 'kitf', kff.shape, file=sys.stderr)
if STOPKEY is not None:      # [fix3-stop] STOP key parts (closed lid, cap at rest) -> stop_v / stop_f / stop_lab / stop_part; report + budget in rep['lid']['stop_key']
    xl3_ = LIDREP['latch_x_mm'] / 1000.0      # [fix3-stop] (SKB_ built before the lid kit) [fix3-int]
    ot3_ = lidmech.DEFAULTS['latch_over'] * (SKB_['L']['xk'] - LK['Q'][0]) / (xl3_ - LK['Q'][0]); SKB_['report']['lid_over_travel_at_cap_mm'] = round(ot3_ * 1000, 2)      # [fix3-stop]
    SKM_ = stop4.mass_summary(SKB_, LK['Q'], SKMM_) if SKV4_ else stopkit.mass(SKB_, LK['Q']); H3R_ = LIDREP.get('h3')      # [fix3-stop] analytic cross-check; [fix3-int] the balance uses the mesh masses (already in LIDREP)  [fix4-stop2] v4: mesh masses + torque
    H4R_ = LIDREP.get('h4')      # [fix4-hinge2]
    bT3_ = (H3R_['spring_Nm']['0'] if H3R_ else LIDREP['spring']['torque_Nm_total']) if not H4R_ else 0.0      # [fix3-int] [fix4-hinge2] guarded h3: tape + spiral-spring torque at the closed pose (the f2 band is superseded)
    gT3_ = (H3R_['gravity_Nm']['0'] if H3R_ else LIDREP['gravity_torque_Nm']['0']) if not H4R_ else 0.0      # [fix3-int] [fix4-hinge2] guarded lid gravity torque closed, STOP parts included (LP extra_mass: no double count)
    if H4R_: bT3_ = H4R_['balance']['spring_at_lid_Nm']['0']; gT3_ = H4R_['gravity_Nm']['0']      # [fix4-hinge2]
    LO3_ = stopkit.lid_over_force(ring_xy, LK['Q'], xl3_, SKB_['L']['xk'], lidmech.DEFAULTS['latch_over'], LP['seal_c'], SKB_['L']['p']['seal_k'], SKB_['L']['p']['latch_F'],
                                  bT3_, gT3_, seal_comp=LIDREP.get('seal_compression_mm', 1.0) / 1000.0 if SKV4_ else None)      # [fix3-stop] [fix3-int]  [fix4-stop2] m7: sign + seal preload
    LO3_.update(spring_T_closed_Nm=bT3_, gravity_T_closed_Nm=gT3_, net_open_closed_Nm=round(bT3_ - gT3_, 4), spring_source='lid.h4' if H4R_ else ('lid.h3' if H3R_ else 'lid.spring'))      # [fix3-int] [fix4-hinge2] h4
    LIDREP['stop_key'] = dict(SKB_['report'], mass=SKM_, mass_mesh=SKMM_['report'], budget=stopkit.budget(L_['stop_key'], ot3_ * 1000, lid=LO3_, roll=stop4.roll_loss(SKB_['L']['p']) if SKV4_ else 0.0))      # [fix3-stop] [fix3-int] mass_mesh  [fix4-stop2] roll
    if SKV4_: LIDREP['stop_key']['sensor'] = stop4.hall_report(SKB_['L']['p'], SKB_['z']['zb_post'], LK['Q'], SKB_['L']['feed'][0])      # [fix4-stop2] S3 lid-closed magnet / hall pair (needs the hinge axis)
    extra.update(stop_v=SKB_['v'], stop_f=SKB_['f'], stop_lab=SKB_['lab'], stop_part=SKB_['part'],
                 stop_axis=np.array([SKB_['L']['xk'], SKB_['L']['yk'], SKB_['L']['z0'], SKB_['L']['p']['stroke']]),
                 stop_kin=np.array([SKB_['L']['xP'], SKB_['z']['zu'], SKB_['z']['zl'], SKB_['L']['p']['arm_L'], SKB_['L']['p']['gap'], SKB_['report']['tip_gap_standin_mm'] / 1000.0]))      # [fix3-stop] arm pivots, tip gap to the scan / to the stand-in key (renderer STOPSTROKE)
    print('stop key:', json.dumps({k_: v_ for k_, v_ in LIDREP['stop_key'].items() if k_ != 'budget'}), file=sys.stderr)      # [fix3-stop]
STK_ = L_.get('stakes')
STAKE_CK = []
if STK_ and not LP['cover_wall']:
    # [fix1] E3: the well module is FASTENED to the cover: ears on the gutter flange (it laps 1.5 mm under the cover's inner skin) are heat-staked
    #        onto pins moulded on the cover's inside. 6 ears (front 2, sides 2 + 2), clear of the hinge pockets behind the rear edge.
    pgl_ = shapely.Polygon(ring_xy); bnd_ = pgl_.exterior; ucs_ = float(L_.get('gutter_setback', 0.0)); gwl_ = lidmech.DEFAULTS['gutter_wall']; tc_ = LP['t_cover']
    EB_ = Bag(); ears_ = []
    for sx_, sy_ in STK_.get('at', [[1, 0.5], [1, -0.5], [0, 1], [0, -1], [-0.5, 1], [-0.5, -1]]):
        # seed: front edge (sx 1) at y = sy * half width, or a side (sy +-1) at x = cx + sx * half length
        sp_ = Point(cx + hx, sy_ * hy) if sx_ == 1 else Point(cx + sx_ * hx, sy_ * hy)
        s0_ = bnd_.project(sp_); p0_ = np.array(bnd_.interpolate(s0_).coords[0]); p1_ = np.array(bnd_.interpolate(s0_ + 0.001).coords[0])
        tg_ = (p1_ - p0_) / np.linalg.norm(p1_ - p0_); no_ = np.array([tg_[1], -tg_[0]])
        if pgl_.contains(Point(*(p0_ + no_ * 0.002))): no_ = -no_                                               # outward (away from the lid)
        d0_ = gapw + ucs_ + gwl_ - 0.0003; d1_ = d0_ + STK_.get('ear', 0.006); we_ = STK_.get('ear_w', 0.008) / 2
        Q4_ = np.array([p0_ + no_ * d0_ - tg_ * we_, p0_ + no_ * d1_ - tg_ * we_, p0_ + no_ * d1_ + tg_ * we_, p0_ + no_ * d0_ + tg_ * we_])
        zt_ = zi(Q4_) - tc_ - 0.00005; Rt_ = np.c_[Q4_, zt_]; Rb_ = np.c_[Q4_, zt_ - 0.0015]
        cE_ = p0_ + no_ * (0.5 * (d0_ + d1_)); zE_ = float(zi(cE_[None])[0]) - tc_
        if STK_.get('pin_r'):      # [fix2] F-N1: the ear has a clearance hole for the stake pin (0.2 mm over its diameter), seat face 0.05 mm off the skin
            pe_ = shapely.Polygon(Q4_).difference(Point(*cE_).buffer(STK_['pin_r'] + 0.0001, quad_segs=10)); zt2_ = float(zt_.min()) - 0.00005
            EB_.add(*(lidmech.prismkit.orient if L_.get('closed_well_kit') else (lambda v_, f_: (v_, f_)))(*lidmech.extrude(pe_, zt2_ - 0.0015, zt2_, lidmech.XY(), bev=0.0)), lidmech.K_D)      # [fix4-int] G5: consistent winding
        else: EB_.add(*stack([Rt_, Rb_], closed=True, cap0=True, cap1=True), lidmech.K_D)
        STAKE_CK.append((cE_, zE_)); ears_.append(np.round(np.r_[cE_, zE_] * 1000, 1).tolist())
    ev2_, ef2_, el2_ = EB_.out()
    kff = np.concatenate([kff, ef2_ + len(kfv)]); kfv = np.concatenate([kfv, ev2_]); kfl = np.r_[kfl, el2_.astype(np.int8)]; kfp = np.r_[kfp, np.full(len(ef2_), lidmech.PF['ear'], np.int16)]
    extra.update(kitf_v=kfv, kitf_f=kff, kitf_lab=kfl, kitf_part=kfp); LIDREP['stake_ears_mm'] = ears_

# ---- top cover: everything above a seam that runs round the body just under the shoulder is a separate snap-on part.
#      It hides the screws: they sit on a deck of the body underneath (built in the fastening section below).
if CV and not OB:
    ts_ = CV.get('t', 0.18); kc_ = CV.get('k', 0.5); zn_ = CV.get('z_nose', 0.191); eb_ = CV.get('blend', 0.004); gapc = CV.get('gap', 0.0006)
    def cvf():      # > 0 on the cover. Sides and tail: the line t = ts_ (parallel to the shoulder); round the nose: the height zn_ (above the camera window)
        a_ = kc_ * (A['t'] - ts_); b_ = V[:, 2] - zn_ + 10.0 * (1 - ss(CV.get('x_n0', 0.20), CV.get('x_n1', 0.245), V[:, 0]))
        c_ = V[:, 2] - CV.get('z_tail', 0.0) + 10.0 * ss(CV.get('x_t0', -0.27), CV.get('x_t1', -0.24), V[:, 0])      # across the tail the seam is level (the t line dips in the middle)
        sm_ = lambda p_, q_: 0.5 * (p_ + q_ - np.sqrt((p_ - q_) ** 2 + eb_ ** 2))
        return np.where(A['cap'] > 0.5, -1.0, sm_(sm_(a_, b_), c_))
    A['cv'] = cvf(); fine('cv', cvf, -0.0015, gapc + 0.0015, CV.get('fine', 0)); split('cv', 0.0); split('cv', gapc)
    fc_ = fmean('cv'); kp_ = ~((fc_ > 0) & (fc_ < gapc)); F = F[kp_]; lab = lab[kp_]

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
CK = Bag(); mok = True
MKN = ['deck', 'pad', 'screw', 'slot', 'boss', 'rib', 'bracket', 'pilot', 'clip', 'well', 'hem_lip', 'other', 'seat', 'drain']; CKN = ['flange', 'leg', 'barb', 'pin', 'seatpad', 'stake']      # [fix1] pin / seatpad (E1), drain (E2)      # [v18c] part kinds (check_wall.py)
MK_ = {'m': 'other', 'c': 'flange'}; MKF, CKF = [], []
def _kinds(bag, rec, key):
    add0 = bag.add
    def add(v, f, lab_):
        n0 = len(bag.F); add0(v, f, lab_)
        if len(bag.F) > n0: rec.append((len(bag.F[-1]), MK_[key]))
    bag.add = add
_kinds(MB, MKF, 'm'); _kinds(CK, CKF, 'c'); deck_ref = None
if MT:
    MJ = json.load(open('mounts.json')); posts = np.array(MJ['posts'], float)
    if CV and CV.get('skip_posts'): posts = np.delete(posts, CV['skip_posts'], 0)      # [v18c] posts left unscrewed (no room under a real wall)
    rw = MT.get('well_r', 0.00375); cw = MT.get('well_chamfer', 0.0006); ft = MT.get('seat', 0.003); rt = MT.get('tower_r', 0.0056); rh = MT.get('hole_r', 0.0018)
    depth_ = {}; zdeck = None; wloops = []; SUBM = np.ones(len(F), bool); crep = {}
    def zup(x_, y_, z0, sub, m=0.0):
        """Height of the body above (x_, y_, z0): the deck where there is one, else the skin less the margin m."""
        if zdeck is not None:
            zd = float(zdeck(np.array([x_]), np.array([y_]))[0])
            if np.isfinite(zd) and zd > z0: return zd
        z = _proj(sub, 0, 1, 2, x_, y_); z = z[z > z0]; return float(z.min()) - m if len(z) else np.nan
    def skin_ring(x_, y_, dirs, r_, z0, sub):
        zs = np.array([zup(x_ + d_[0] * r_, y_ + d_[1] * r_, z0, sub, 0.0008) for d_ in dirs]); return np.where(np.isfinite(zs), zs, np.nanmin(zs) if np.isfinite(zs).any() else z0 + 0.02)
    def rib(x_, y_, q_, r0, ln_, z0, drop, sub):
        """Rib from a boss (radius r0) out along the angle q_: its top edge follows the inside of the skin (1.8 mm below the outer surface)."""
        d_ = np.array([np.cos(q_), np.sin(q_)]); rs = r0 + np.linspace(0, ln_, 6); zt = np.array([zup(x_ + d_[0] * r_, y_ + d_[1] * r_, z0, sub, 0.0018) for r_ in rs])
        if not np.isfinite(zt).all() or np.ptp(zt) > 0.02: return
        zb = zt[0] - (drop if drop > 0 else -drop * (zt[0] - z0))
        if zb < z0 + 0.001: return
        p0_ = (x_ + d_[0] * r0, y_ + d_[1] * r0, zb); pt_ = [(x_ + d_[0] * r_, y_ + d_[1] * r_, z_i) for r_, z_i in zip(rs, zt)]      # [fix4-cover2]
        if CV and CV.get('mount_chassis_clear'):      # [fix4-cover2] the rib's lower edge clears the real chassis (front boss rib over the wing corner)
            import coverkit4 as _ck4; p0_ = _ck4.rib_clear(p0_, pt_, 0.0012, CV['mount_chassis_clear'], zt[0] - 0.002)      # [fix4-cover2]
            if p0_ is None: return      # [fix4-cover2]
        _fin(p0_, *pt_)      # [fix4-cover2]
    def screw(x_, y_, zf, down=0.011, flat=False):
        q = np.linspace(0, 2 * np.pi, 48, endpoint=False); cs = np.c_[np.cos(q), np.sin(q)]; lobe = 0.00105 + 0.00022 * np.cos(6 * q); h1, h2, hr = (0.0002, 0.0006, 0.0001) if flat else (0.0013, 0.0019, 0.0008)
        R = [np.c_[x_ + cs[:, 0] * r_, y_ + cs[:, 1] * r_, np.full(48, zf + h_)] for r_, h_ in ((0.0028, 0.0), (0.0028, h1), (0.00235, h2), (lobe, h2), (lobe, hr))]
        k0_ = MK_['m']; MK_['m'] = 'screw'; MB.add(*stack(R, cap0=bool(CV and CV.get('orient_kits')), cap1=True), M_SCREW);      # [fix4-cover2] G5: head closed underneath MB.add(*tube((x_, y_, zf), (x_, y_, zf - down), 0.0015, 20), M_SCREW); MK_['m'] = k0_
    if not CV:
        def wfield():
            d = np.full(len(V), 1.0)
            for x_, y_, z_, _ in posts:
                dk = np.hypot(V[:, 0] - x_, V[:, 1] - y_) - rw; d = np.where((V[:, 2] > z_ + ft + 0.0005) & (dk < d), dk, d)
            return d
        A['wl'] = wfield(); fine('wl', wfield, -0.0015, cw + 0.002, MT.get('fine', 0.0005)); cuts('wl', [0.0, cw, cw + HOLD])
        dd = A['wl']; V[:, 2] -= np.where(dd < cw, cw - np.maximum(dd, 0.0), 0.0)                  # 45 degree chamfer round the rim
        hole_ = fmean('wl') < 0; wloops = loops_of(F[hole_]); F = F[~hole_]; lab = lab[~hole_]; Cf = V[F].mean(1); SUBM = np.ones(len(F), bool)
    else:
        # Under the cover the body carries a deck: a ledge at the height of the screw seats, lowered where the cover comes closer.
        # The 14 screws sit on it; the middle stays open (controls, chassis). The cover snaps on with legs through slots in the deck.
        from shapely.geometry.polygon import orient as _orient
        import coverkit4 as _ck4      # [fix4-cover2]
        isb = fmean('cv') < gapc * 0.5; SUBM = isb; Cf = V[F].mean(1); hc = CV.get('cell', 0.0015)
        w_ = np.array([[a_, b_, 1 - a_ - b_] for a_ in np.linspace(0, 1, 6) for b_ in np.linspace(0, 1 - a_, max(int(round((1 - a_) * 5)) + 1, 1))])
        Sm = np.einsum('kj,fjc->fkc', w_, V[F[~isb]]).reshape(-1, 3); gx0, gy0 = -0.36, -0.24; nx_, ny_ = int(0.72 / hc), int(0.48 / hc)
        if WALL_:      # [v18c] real wall: the deck keeps its gap under the cover's INNER skin (samples moved in by the wall along the face normal)
            Fc_ = F[~isb]; Pc_ = V[Fc_]; nc_ = np.cross(Pc_[:, 1] - Pc_[:, 0], Pc_[:, 2] - Pc_[:, 0]); nc_ /= np.maximum(np.linalg.norm(nc_, axis=1, keepdims=True), 1e-30)
            if nc_[A['t'][Fc_].mean(1) > 0.9][:, 2].mean() < 0: nc_ = -nc_
            Sm = Sm - float(WALL_.get('t_part', {}).get('1', WALL_['t'])) * np.repeat(nc_, len(w_), 0)
        ZC = np.full((nx_, ny_), np.inf); np.minimum.at(ZC, (((Sm[:, 0] - gx0) / hc).astype(int), ((Sm[:, 1] - gy0) / hc).astype(int)), Sm[:, 2])      # underside of the cover (lowest point per cell)
        from scipy.ndimage import distance_transform_edt as _edt, map_coordinates as _mapc, gaussian_filter as _gf
        ZC = ZC[tuple(_edt(~np.isfinite(ZC), return_distances=False, return_indices=True))]
        if WALL_: from scipy.ndimage import minimum_filter as _minf; ZC = _minf(ZC, 3)      # [v18c] steep walls: the Gaussian below must not lift the deck into them
        ZC = _gf(ZC, CV.get('smooth', 2.0))      # cells outside the cover take the nearest value; smoothed (the deck keeps `inset` below it)
        zcov = lambda x, y: _mapc(ZC, [(np.asarray(x, float) - gx0) / hc - 0.5, (np.asarray(y, float) - gy0) / hc - 0.5], order=1, mode='nearest')      # smooth, and never above the real surface
        cvr = ~isb; hd = np.array([[0, 0], [0.0029, 0], [-0.0029, 0], [0, 0.0029], [0, -0.0029], [0.002, 0.002], [-0.002, 0.002], [0.002, -0.002], [-0.002, -0.002]])
        def room(x_, y_, z_, wall=0.0):      # lowest point of the cover's outer surface over a screw head at (x_, y_), above z_ ([v18c] wall > 0: of its inner surface)
            sub_ = cvr & (np.hypot(Cf[:, 0] - x_, Cf[:, 1] - y_) < 0.015); best = np.inf; sf_ = np.nonzero(sub_)[0]
            Pn_ = V[F[sf_]]; cn_ = np.cross(Pn_[:, 1] - Pn_[:, 0], Pn_[:, 2] - Pn_[:, 0]); nz_ = np.abs(cn_[:, 2]) / np.maximum(np.linalg.norm(cn_, axis=1), 1e-30)
            for a_, b_ in hd:
                for zz_, s_ in zip(*_proj2(sub_, x_ + a_, y_ + b_)):
                    if zz_ > z_: best = min(best, zz_ - wall / max(nz_[s_], 0.2))      # a sloping wall is t / cos(slope) thick in z
            return best - z_
        def _proj2(sub, a, b):
            P0 = V[F[sub, 0]]; e1 = V[F[sub, 1]] - P0; e2 = V[F[sub, 2]] - P0; den = e1[:, 0] * e2[:, 1] - e1[:, 1] * e2[:, 0]; ok_ = np.abs(den) > 1e-18
            da = a - P0[:, 0]; db = b - P0[:, 1]; dn = np.where(ok_, den, 1.0); u_ = (da * e2[:, 1] - db * e2[:, 0]) / dn; v_ = (e1[:, 0] * db - e1[:, 1] * da) / dn
            hit = ok_ & (u_ >= -1e-9) & (v_ >= -1e-9) & (u_ + v_ <= 1 + 1e-9); return (P0[:, 2] + u_ * e1[:, 2] + v_ * e2[:, 2])[hit], np.nonzero(hit)[0]
        rm_ = np.array([room(x_, y_, z_) for x_, y_, z_, _ in posts]); wl_ = CV.get('wall', 0.002)
        if WALL_:      # [v18c] real wall: room measured to the cover's inner surface (local wall incl. thin zones, slope-corrected); nothing more to subtract
            tl_ = shell_wall.thickness_at(posts[:, :3], WALL_, 1); rm_ = np.array([room(x_, y_, z_, t_) for (x_, y_, z_, _), t_ in zip(posts, tl_)]); wl_ = 0.0
        ftk = np.clip(rm_ - wl_ - 0.0006 - 0.0004, 0.0015, ft); seatz = posts[:, 2] + ftk               # seat thinner where the cover is close above the post (flat-head screw there)
        def _side(sg):
            m_ = posts[:, 1] * sg > -0.02; q = np.c_[posts[m_, 0], seatz[m_]]; q = q[np.argsort(q[:, 0])]; f_ = PchipInterpolator(q[:, 0], q[:, 1]); return lambda x: f_(np.clip(x, q[0, 0], q[-1, 0]))
        zL, zR = _side(1), _side(-1)
        def zraw(x, y):
            w2 = ss(-0.04, 0.04, y); a_ = zL(x) * w2 + zR(x) * (1 - w2); b_ = zcov(x, y) - (CV.get('inset_inner', 0.0015) if WALL_ else CV.get('inset', 0.004))
            return 0.5 * (a_ + b_ - np.sqrt((a_ - b_) ** 2 + CV.get('round', 0.004) ** 2))                  # the lower of the two, with a rounded transition
        bl = [l2 for l2 in loops_of(F[isb]) if len(l2) > 200 and np.abs(A['cv'][l2]).max() < 1e-6]; lpb = max(bl, key=len)
        def ccw(lp_):
            P_ = V[lp_]; return lp_ if np.sum(P_[:, 0] * np.roll(P_[:, 1], -1) - np.roll(P_[:, 0], -1) * P_[:, 1]) > 0 else lp_[::-1]
        def inward(P_):
            tg_ = np.roll(P_[:, :2], -4, 0) - np.roll(P_[:, :2], 4, 0); tg_ /= np.maximum(np.linalg.norm(tg_, axis=1, keepdims=True), 1e-12); return tg_, np.c_[-tg_[:, 1], tg_[:, 0]]
        lpb = ccw(lpb); Pb = V[lpb]; nb_ = len(Pb); tgb, ninb = inward(Pb)
        if OB:      # [v18] where the part line runs (nearly) vertically (tail rise, orbit flanks) the plan tangent is degenerate:
            #         take the inward direction from the body normal instead (it is horizontal on every wall the part line crosses)
            nn_ = -nrm_()[lpb][:, :2]; ln_ = np.linalg.norm(nn_, axis=1, keepdims=True); nn_ = np.where(ln_ > 0.3, nn_ / np.maximum(ln_, 1e-12), ninb)
            for _ in range(3): nn_ = 0.25 * np.roll(nn_, 1, 0) + 0.5 * nn_ + 0.25 * np.roll(nn_, -1, 0)
            ninb = nn_ / np.maximum(np.linalg.norm(nn_, axis=1, keepdims=True), 1e-12); tgb = np.c_[ninb[:, 1], -ninb[:, 0]]
        O = Pb[:, :2] + ninb * CV.get('lip', 0.0028)
        cen = posts[:, :2].mean(0); pr = posts[np.argsort(np.arctan2(posts[:, 1] - cen[1], posts[:, 0] - cen[0]))]; om = CV.get('opening_margin', 0.010)
        op = _orient(Polygon(pr[:, :2]).buffer(-(om + 0.012), join_style=1).buffer(0.012, join_style=1), 1.0); ex = op.exterior; Lx = ex.length
        sp = np.unwrap(shapely.line_locate_point(ex, shapely.points(O)) * 2 * np.pi / Lx) * Lx / (2 * np.pi); sp = np.maximum.accumulate(sp)
        tr_ = sp[0] + (sp[-1] - sp[0]) * np.arange(nb_) / max(nb_ - 1, 1); dv = sp - tr_; kw = 41; dv = np.convolve(np.r_[dv[-kw:], dv, dv[:kw]], np.ones(kw) / kw, 'same')[kw:-kw]
        I_ = shapely.get_coordinates(shapely.line_interpolate_point(ex, (tr_ + dv) % Lx)); wd = ((I_ - O) * ninb).sum(1); I_ = np.where((wd < 0.003)[:, None], O + ninb * 0.003, I_)
        Kd = int(CV.get('rings', 14)); R_ = [Pb, np.c_[O, Pb[:, 2] - 0.0002]]
        for k_ in range(Kd + 1):
            X_ = O + (I_ - O) * (k_ / Kd); R_.append(np.c_[X_, zraw(X_[:, 0], X_[:, 1])])
        if not CV.get('kit2', False): MK_['m'] = 'deck'; deck_ref = (len(MB.V), nb_); MB.add(*stack(R_), M_IN)
        reg = Polygon(O).buffer(0).difference(op)
        def zdeck(x, y):
            return np.where(shapely.contains_xy(reg, x, y), zraw(x, y), np.nan)
        seat_ = []; edge_ = []; pads_ = []; PADR_ = []
        for k, (x_, y_, z_, _) in enumerate(posts):
            zf = float(seatz[k]); seat_.append(zf - z_); edge_.append(reg.boundary.distance(Point(x_, y_)))
            rp_ = 0.0056
            if WALL_:      # [v18c] the pad stays 0.8 mm off the inside of the body wall (a pad merged into the skin = a thick spot = sink mark)
                vb_ = np.unique(F[isb]); vb_ = vb_[(V[vb_, 2] > z_ - 0.002) & (V[vb_, 2] < zf + 0.002) & (np.hypot(V[vb_, 0] - x_, V[vb_, 1] - y_) < 0.02)]
                if len(vb_): rp_ = float(min(rp_, np.hypot(V[vb_, 0] - x_, V[vb_, 1] - y_).min() - shell_wall.thickness_at([[x_, y_, zf]], WALL_, 0)[0] - 0.0008))
            pads_.append(round(rp_ * 1000, 2)); PADR_.append(rp_)
            zp0_ = _ck4.post_top(x_, y_, rp_, z_ + 0.0002) if (CV or {}).get('pad_on_scan') else z_ + 0.0002      # [fix4-cover2] pad bears on the SCANNED post top (post 3: 1.28 above mounts.json)
            MK_['m'] = 'pad'; MB.add(*lathe((x_, y_, zp0_), (0, 0, 1), [(0, 0.0), (0, rp_), (max(zf - 0.0002 - zp0_, 0.0003), rp_)] + ([(max(zf - 0.0002 - zp0_, 0.0003), 0.0)] if CV.get('orient_kits') else []), 40), M_IN)            # pad between the post and the deck ([fix4-cover2] G5: top closed)
            cl_ = rm_[k] - (zf - z_) - wl_; flat_ = cl_ < 0.0026; screw(x_, y_, zf, flat=flat_); depth_[k] = cl_ - (0.0006 if flat_ else 0.0019)      # room between the screw head and the inside of the cover
        # the cover: return flange along its edge, snap legs between the screws
        lc = [l2 for l2 in loops_of(F[(~isb) & (lab != 4)]) if len(l2) > 200 and np.abs(A['cv'][l2] - gapc).max() < 1e-6]; lpc = ccw(max(lc, key=len)); Pc = V[lpc]; tgc, ninc = inward(Pc)
        if WALL_: pass                  # [v18c] the real wall has its own edge face: no fake flange / ribbon
        elif OB:      # [orbit] the shell edge shows its wall: a ribbon from the rolled edge straight in along the body normal (visual only)
            Nc_ = nrm_()[lpc]; CK.add(*stack([Pc, Pc - Nc_ * CV.get('wall', 0.0025)]), 1)
        else: CK.add(*stack([Pc, np.c_[Pc[:, :2] + ninc * 0.0026, Pc[:, 2]]]), 0)
        nleg = 0; LG = CV.get('legs', None); legrep = []; GAPS_ = []; LEGF_ = []; KIT2 = bool(CV.get('kit2', False)); HOLES_ = []; LEGAT_ = []
        def _slope(q_, h_=0.0015):      # [fix2] deck slope (|grad z|) at q_
            zx_ = zraw(np.array([q_[0] - h_, q_[0] + h_]), np.array([q_[1], q_[1]])); zy_ = zraw(np.array([q_[0], q_[0]]), np.array([q_[1] - h_, q_[1] + h_]))
            return float(np.hypot(zx_[1] - zx_[0], zy_[1] - zy_[0]) / (2 * h_))
        def _inner_z(px_, py_, zmin_):      # [fix2] the cover's inner skin above (px_, py_): outer skin hit minus the wall measured along z (t / n_z)
            sc3_ = cvr & (np.hypot(Cf[:, 0] - px_, Cf[:, 1] - py_) < 0.02); best_ = np.inf; tcw_ = float(WALL_['t']) if WALL_ else 0.0
            for zz_, s_ in zip(*_proj2(sc3_, px_, py_)):
                if zz_ > zmin_:
                    Pn_ = V[F[np.nonzero(sc3_)[0][s_]]]; nn_ = np.cross(Pn_[1] - Pn_[0], Pn_[2] - Pn_[0]); nz_ = abs(nn_[2]) / max(np.linalg.norm(nn_), 1e-30)
                    best_ = min(best_, zz_ - tcw_ / max(nz_, 0.2))
            return best_
        LMOVE_ = _ck4.leg_moves(LG)      # [fix4-cover2] G1: legs over the kept chassis rim / pin stub move along the edge
        for a_, b_ in zip(pr, np.roll(pr, -1, 0)):
            GAPS_.append((a_, b_, False))
            i_ = int(np.argmin(np.hypot(O[:, 0] - 0.5 * (a_[0] + b_[0]), O[:, 1] - 0.5 * (a_[1] + b_[1])))); c_ = O[i_] + ninb[i_] * 0.0045; zd = float(zraw(c_[:1], c_[1:2])[0])
            i2_, LRULE_ = _ck4.moved_site(LMOVE_, O, i_, ninb)      # [fix4-cover2]
            if i2_ is not None: _ck4.REP['leg_moves'].append(dict(gap=len(GAPS_) - 1, from_mm=np.round(O[i_] * 1000, 1).tolist(), to_mm=np.round(O[i2_] * 1000, 1).tolist())); i_ = i2_; c_ = O[i_] + ninb[i_] * 0.0045; zd = float(zraw(c_[:1], c_[1:2])[0])      # [fix4-cover2]
            if KIT2 and LG:      # [fix2] R2E1: the leg goes where the deck is level (|slope| <= kit2_slope), so its barb catches a horizontal deck underside
                for din_ in np.arange(0.0045, 0.035, 0.0005):
                    q_ = O[i_] + ninb[i_] * din_
                    if max(_slope(q_), _slope(q_ - ninb[i_] * 0.0035), _slope(q_ + ninb[i_] * 0.004)) <= CV.get('kit2_slope', 0.27) and reg.buffer(-0.006).contains(Point(*q_)) and np.hypot(posts[:, 0] - q_[0], posts[:, 1] - q_[1]).min() > 0.012: c_ = q_; break
                else: continue
                zd = float(zraw(c_[:1], c_[1:2])[0])
            sc_ = cvr & (np.hypot(Cf[:, 0] - c_[0], Cf[:, 1] - c_[1]) < 0.02); zz = []
            if LG and WALL_:      # [v18c] cantilever snap leg to the engineering spec (critique 7.3): w 8, t 2.0 -> 1.4, hook 1.2 with a 30 deg lead-in and a
                #                   45 deg retention face catching the deck underside, free length >= L_min; deck window 9 x 3.1 (drawn as a dark inlay)
                w_l, t0_l, t1_l, hk_, td_ = LG.get('w', 0.008), LG.get('t_root', 0.002), LG.get('t_tip', 0.0014), LG.get('hook', 0.0012), LG.get('deck_t', 0.0025)
                tg_ = np.array([tgb[i_, 0], tgb[i_, 1], 0.0]); o3_ = np.array([-ninb[i_, 0], -ninb[i_, 1], 0.0]); zr_ = []
                tcv_ = float(shell_wall.thickness_at([[c_[0], c_[1], zd]], WALL_, 1)[0])
                for e1_ in (-w_l / 2, 0.0, w_l / 2):
                    for e2_ in (0.0, -t0_l):
                        q_ = c_ + tg_[:2] * e1_ + o3_[:2] * e2_; best_ = np.inf
                        for zz_, s_ in zip(*_proj2(sc_, q_[0], q_[1])):
                            if zz_ > zd:
                                Pn_ = V[F[np.nonzero(sc_)[0][s_]]]; nn_ = np.cross(Pn_[1] - Pn_[0], Pn_[2] - Pn_[0]); nz_ = abs(nn_[2]) / max(np.linalg.norm(nn_), 1e-30)
                                best_ = min(best_, zz_ - tcv_ / max(nz_, 0.2))
                        zr_.append(best_)
                z_root = min(zr_); z_ret = zd - td_; Lf = z_root - z_ret; legrep.append(round(Lf * 1000, 1))
                if not np.isfinite(Lf) or Lf < LG.get('L_min', 0.016): continue
                z_top = z_root + 0.0008; li_ = LG.get('lead_in_deg', 30.0); rt_ = LG.get('ret_deg', 45.0)
                zh0 = z_ret; zh1 = z_ret - hk_ / np.tan(np.radians(rt_)); zh2 = zh1 - 0.0003; zh3 = zh2 - hk_ / np.tan(np.radians(li_)); z_tip = zh3 - 0.0005
                ring_ = lambda z, t_: np.array([np.array([c_[0], c_[1], z]) + tg_ * u_ + o3_ * v_ for u_, v_ in ((-w_l / 2, -t_), (w_l / 2, -t_), (w_l / 2, 0.0), (-w_l / 2, 0.0))])
                zs_l = np.linspace(z_top, z_tip, 8); tt_l = t0_l + (t1_l - t0_l) * np.clip((z_top - zs_l) / max(z_top - zh0, 1e-6), 0, 1)
                L_fx = LG.get('L_flex', 0.0); Lfe_ = Lf
                if L_fx and Lf > L_fx + 0.003:      # [fix1] E1: a stiff stem (t_stem) carries the leg down; only the last L_flex flexes (shorter = stiffer, ~10x retention)
                    z_fx = z_ret + L_fx; ts_l = LG.get('t_stem', 0.0032); Lfe_ = L_fx
                    zs_l = np.r_[np.linspace(z_top, z_fx + 0.002, 4), np.linspace(z_fx, z_tip, 8)]
                    tt_l = np.where(zs_l > z_fx + 0.001, ts_l, t0_l + (t1_l - t0_l) * np.clip((z_fx - zs_l) / max(z_fx - zh0, 1e-6), 0, 1))
                # retention (tapered cantilever as a uniform beam of the mean section, E 2.3 GPa, mu 0.3): deflection force at the hook, x (mu + tan a)/(1 - mu tan a)
                tm_ = 0.5 * (t0_l + t1_l); Pd_ = 3 * 2.3e9 * (w_l * tm_ ** 3 / 12) * hk_ / Lfe_ ** 3; ta_ = np.tan(np.radians(rt_))
                LEGF_.append(dict(L_flex_mm=round(Lfe_ * 1000, 1), deflect_N=round(Pd_, 2), retain_N=round(Pd_ * (0.3 + ta_) / max(1 - 0.3 * ta_, 1e-3), 1), strain_pct=round(150 * t0_l * hk_ / Lfe_ ** 2, 2)))
                GAPS_[-1] = (a_, b_, True)
                if KIT2:      # [fix2] R2E1 / R2E8: the root follows the inner skin (fused 0.6 mm into it over its whole section) and is necked to 1.5 mm
                    #             (0.6 t: no sink on the light cover); the 3.2 stem starts 4 mm below it, two 1.2 mm gussets carry the neck
                    tn_ = LG.get('t_neck', 0.0015); R0_ = ring_(z_top, tn_); zc0_ = np.array([_inner_z(p_[0], p_[1], zd) for p_ in R0_])
                    if not np.isfinite(zc0_).all(): continue
                    R0_[:, 2] = zc0_ + 0.0006; zn0_ = float(zc0_.min()) - 0.001
                    keep_ = zs_l < zn0_ - 0.004; zs2_ = np.r_[zn0_, zn0_ - 0.004, zs_l[keep_]]; tt2_ = np.r_[tn_, tt_l[keep_][0] if keep_.any() else LG.get('t_stem', 0.0032), tt_l[keep_]]
                    SH_ = _ck4.follow_deck(LG, c_, tg_, lambda X_, Y_: zraw(X_, Y_), w_l, zn0_ - 0.004, (z_ret + L_fx) if L_fx else z_ret)      # [fix4-cover2] G1: lower leg + hook parallel to the deck underside
                    MK_['c'] = 'leg'; CK.add(*stack([R0_] + [SH_(ring_(z, t_)) for z, t_ in zip(zs2_, tt2_)], closed=True, cap0=True, cap1=True), 0)      # [fix4-cover2] SH_
                    for u_ in (-w_l / 2 + 0.0006, w_l / 2 - 0.0006):      # gussets on the inward face
                        p0_ = np.array([c_[0], c_[1], 0.0]) + tg_ * u_ + o3_ * (-tn_); p1_ = np.array([c_[0], c_[1], 0.0]) + tg_ * u_ + o3_ * (-tn_ - 0.005)
                        z1_ = _inner_z(p1_[0], p1_[1], zd)
                        if not np.isfinite(z1_): continue
                        T_ = np.array([[p0_[0], p0_[1], zn0_ - 0.006], [p0_[0], p0_[1], float(zc0_.mean()) + 0.0004], [p1_[0], p1_[1], z1_ + 0.0004]]); n_ = tg_ * 0.0006
                        MK_['c'] = 'leg'; CK.add(*stack([T_ - n_, T_ + n_], cap0=True, cap1=True), 0)
                    LEGAT_.append(np.round(np.r_[c_, zd] * 1000, 1).tolist())
                else:
                    MK_['c'] = 'leg'; CK.add(*stack([ring_(z, t_) for z, t_ in zip(zs_l, tt_l)], closed=True, cap0=True, cap1=True), 0)
                hp_ = [(0.0, zh0), (hk_, zh1), (hk_, zh2), (0.0, zh3)]                                                              # hook profile (v outward, z)
                hr_ = [np.array([[c_[0], c_[1], 0.0] + tg_ * u_ + o3_ * v_ + np.array([0, 0, z]) for v_, z in hp_]) for u_ in (-w_l / 2, w_l / 2)]
                if KIT2: hr_ = [SH_(h2_) for h2_ in hr_]      # [fix4-cover2] G1: the hook follows the deck underside along the window
                MK_['c'] = 'barb'; CK.add(*stack(hr_ + [], closed=True, cap0=True, cap1=True), 0)
                rot = np.array([[tgb[i_, 0], -tgb[i_, 1], 0], [tgb[i_, 1], tgb[i_, 0], 0], [0, 0, 1.0]]); wc_ = c_ + o3_[:2] * (-0.00155)
                wh_ = (LRULE_ or {}).get('window_half', 0.0048); wr_ = (LRULE_ or {}).get('window_r', 0.0004); _ck4.locator(LRULE_, c_, tg_, w_l, wh_)      # [fix4-cover2] M5: x-locating legs (thermal datum at mid-length) fit their window +-0.1
                if KIT2: HOLES_.append(('window', shapely.Polygon([c_ + tg_[:2] * u_ + o3_[:2] * v_ for u_, v_ in ((-wh_, 0.0003), (wh_, 0.0003), (wh_, -0.0035), (-wh_, -0.0035))]).buffer(-wr_).buffer(wr_, quad_segs=6))); nleg += 1      # [fix2] real 9.6 x 3.8 window (leg + hook + 0.3)  [fix4-cover2] wh_
                else: MK_['m'] = 'slot'; MB.add(*box((wc_[0], wc_[1], zd + 0.00015), (0.009, 0.0031, 0.0003), rot), M_DARK); nleg += 1                    # deck window 9 x 3.1
                continue
            for e1_, e2_ in ((0, 0), (0.0038, 0.0013), (-0.0038, 0.0013), (0.0038, -0.0013), (-0.0038, -0.0013)):          # the leg stays under the cover at all four corners
                h_ = _proj(sc_, 0, 1, 2, c_[0] + tgb[i_, 0] * e1_ + ninb[i_, 0] * e2_, c_[1] + tgb[i_, 1] * e1_ + ninb[i_, 1] * e2_); h_ = h_[h_ > zd]; zz.append(float(h_.min()) if len(h_) else -1.0)
            if min(zz) - zd < 0.0045: continue
            zt_ = min(zz) - 0.0016; zb_ = zd - 0.0045; rot = np.array([[tgb[i_, 0], -tgb[i_, 1], 0], [tgb[i_, 1], tgb[i_, 0], 0], [0, 0, 1.0]]); o_ = -ninb[i_]
            MK_['c'] = 'leg'; CK.add(*box((c_[0], c_[1], (zt_ + zb_) / 2), (0.007, 0.002, zt_ - zb_), rot), 0); MK_['c'] = 'barb'; CK.add(*box((c_[0] + o_[0] * 0.0017, c_[1] + o_[1] * 0.0017, zd - 0.0022), (0.007, 0.0016, 0.0022), rot), 0)
            MK_['m'] = 'slot'; MB.add(*box((c_[0], c_[1], zd + 0.00015), (0.0095, 0.0038, 0.0003), rot), M_DARK); nleg += 1                                                  # slot in the deck
        DAT_ = CV.get('datum', None); datrep = {}
        if DAT_ and WALL_ and LG:
            # [fix1] E1: location and seating of the cover. (a) Two datum pins of the cover (nose and tail centre line) drop through round holes in the
            #        deck: fore-aft and lateral position, so the 0.8 joint is set by the tool, not by the legs' window play. (b) Seat pads (crush ribs)
            #        of the cover land on the deck top: the vertical down-stop that sets the 0.5 step and takes the push on the lid / latch.
            tcv0_ = float(WALL_['t'])
            def _cov_inner(q_, r_):
                sc2_ = cvr & (np.hypot(Cf[:, 0] - q_[0], Cf[:, 1] - q_[1]) < 0.02); best_ = np.inf
                for e1_, e2_ in ((0, 0), (r_, 0), (-r_, 0), (0, r_), (0, -r_)):
                    for zz_, s_ in zip(*_proj2(sc2_, q_[0] + e1_, q_[1] + e2_)):
                        Pn_ = V[F[np.nonzero(sc2_)[0][s_]]]; nn_ = np.cross(Pn_[1] - Pn_[0], Pn_[2] - Pn_[0]); nz_ = abs(nn_[2]) / max(np.linalg.norm(nn_), 1e-30)
                        if zz_ > float(zraw(q_[:1], q_[1:2])[0]): best_ = min(best_, zz_ - tcv0_ / max(nz_, 0.2))
                return best_
            pins_ = []
            for nm_, k_ in (('nose', int(np.argmax(O[:, 0]))), ('tail', int(np.argmin(O[:, 0])))):
                for din_ in np.arange(DAT_.get('inset', 0.006), 0.04, 0.001):      # first spot inboard: clear of the posts, on the deck, cover >= 3 mm above it
                    q_ = O[k_] + ninb[k_] * din_; q_[1] = 0.0
                    if np.hypot(posts[:, 0] - q_[0], posts[:, 1] - q_[1]).min() >= DAT_.get('post_clear', 0.010) and reg.contains(Point(*q_)) and \
                       _cov_inner(q_, 0.004) - float(zraw(q_[:1], q_[1:2])[0]) >= DAT_.get('pin_gap', 0.003): break
                zd_ = float(zraw(q_[:1], q_[1:2])[0]); zc_ = _cov_inner(q_, 0.003); rp2_ = DAT_.get('r', 0.002); zb_ = zd_ - LG.get('deck_t', 0.0025) - DAT_.get('below', 0.003)
                if not np.isfinite(zc_): continue
                if KIT2:
                    # [fix2] R2E1 / R2E8 / R2E6: a CORED pin (OD 2 r, wall 1.0 = 0.4 t) whose top ring follows the inner skin (fused 0.6 mm into it all
                    #        round), 0.5 deg draft, 0.8 mm lead-in, two 1.2 mm gussets along x to the skin. Nose hole round (primary datum), tail hole a
                    #        4.3 x 9 slot along x (no over-constraint over the 579 mm span); both are real holes in the 2.5 mm deck.
                    nq_ = 48; aq_ = np.linspace(0, 2 * np.pi, nq_, endpoint=False); cq_ = np.c_[np.cos(aq_), np.sin(aq_)]; ri_ = rp2_ - DAT_.get('wall', 0.001)
                    top_o_ = np.array([_inner_z(q_[0] + c2_[0] * rp2_, q_[1] + c2_[1] * rp2_, zd_) for c2_ in cq_]); top_i_ = np.array([_inner_z(q_[0] + c2_[0] * ri_, q_[1] + c2_[1] * ri_, zd_) for c2_ in cq_])
                    if not (np.isfinite(top_o_).all() and np.isfinite(top_i_).all()): continue
                    zt0_ = float(min(top_o_.min(), top_i_.min())) - 0.001; Lp_ = zt0_ - zb_; dr_ = Lp_ * np.tan(np.radians(0.5))
                    RO_ = lambda r__, z__: np.c_[q_[0] + cq_[:, 0] * r__, q_[1] + cq_[:, 1] * r__, np.broadcast_to(z__, nq_)]
                    rings_ = [RO_(rp2_, top_o_ + 0.0006), RO_(rp2_, zt0_), RO_(rp2_ - dr_, zb_ + 0.0008), RO_(rp2_ - dr_ - 0.0006, zb_), RO_(ri_ - dr_, zb_), RO_(ri_ - dr_, zb_ + 0.001), RO_(ri_, zt0_), RO_(ri_, top_i_ + 0.0006)]
                    MK_['c'] = 'pin'; CK.add(*stack(rings_ + [rings_[0]], closed=True), 0)
                    for sx_ in (-1, 1):
                        p1_ = q_ + np.array([sx_ * (rp2_ + 0.006), 0.0]); z1_ = _inner_z(p1_[0], p1_[1], zd_)
                        KCV_ = (WALL_.get('labyrinth') or {}).get('kit_cv_min') if WALL_ else None      # [fix3-labyrinth] the gusset foot stays out of the
                        if KCV_ and PLINE is not None:                                                    # cover lip / upstand zone at the part line [fix3-labyrinth]
                            for gl_ in np.arange(0.006, 0.0015, -0.0005):      # [fix3-labyrinth]
                                p1_ = q_ + np.array([sx_ * (rp2_ + gl_), 0.0]); z1_ = _inner_z(p1_[0], p1_[1], zd_)      # [fix3-labyrinth]
                                if np.isfinite(z1_) and float(_pl.sdist(PLINE, np.array([p1_[0]]), np.array([z1_]))[0]) >= KCV_: break      # [fix3-labyrinth]
                        if not np.isfinite(z1_): continue
                        T_ = np.array([[q_[0] + sx_ * (rp2_ - 0.0003), q_[1], zt0_ - 0.006], [q_[0] + sx_ * (rp2_ - 0.0003), q_[1], float(top_o_.mean()) + 0.0004], [p1_[0], p1_[1], z1_ + 0.0004]])
                        nn2_ = np.array([0, 0.0006, 0]); MK_['c'] = 'pin'; CK.add(*stack([T_ - nn2_, T_ + nn2_], cap0=True, cap1=True), 0)
                    sl_ = DAT_.get('slot', 0.009) if nm_ == 'tail' else DAT_.get('slot_nose', 0.0)      # [fix4-cover2] M5: nose hole a slot along x too (x located at mid-length)
                    HOLES_.append(('pin_' + nm_, shapely.LineString([(q_[0] - sl_ / 2, q_[1]), (q_[0] + sl_ / 2, q_[1])]).buffer(rp2_ + 0.00015, quad_segs=12) if sl_ else Point(*q_).buffer(rp2_ + 0.00015, quad_segs=16)))
                    zc_ = float(top_o_.min())
                else:
                    MK_['c'] = 'pin'; CK.add(*lathe((q_[0], q_[1], zc_ + 0.0008), (0, 0, -1), [(0, 0.0), (0, rp2_ + 0.0012), (0.0012, rp2_), (zc_ + 0.0008 - zb_ - 0.0008, rp2_), (zc_ + 0.0008 - zb_, rp2_ - 0.0008), (zc_ + 0.0008 - zb_, 0.0)], 32), 0)
                    MK_['m'] = 'slot'; MB.add(*lathe((q_[0], q_[1], zd_ + 0.00015), (0, 0, 1), [(0, rp2_ + 0.00015), (0, rp2_ + 0.0012), (0.0003, rp2_ + 0.0012), (0.0003, rp2_ + 0.00015)], 32), M_DARK)      # hole in the deck (dark inlay, like the leg windows)
                pins_.append(dict(at_mm=np.round(np.r_[q_, zd_] * 1000, 1).tolist(), length_mm=round((zc_ - zb_) * 1000, 1), engage_below_deck_mm=round((zd_ - zb_) * 1000, 1)))
            pads2_ = []
            ex_ = {int(g_): f_ for g_, f_ in DAT_.get('pad_extra', [])}
            for gi_, (a_, b_, hasleg_) in enumerate(GAPS_):
                fr_ = [0.5] if not hasleg_ else ([ex_[gi_]] if gi_ in ex_ else [])
                if KIT2: fr_ = [0.5] if not hasleg_ else [0.22, 0.78]      # [fix2] R2E1: down-stops all round (beside the legs too), not only at the ends
                for f_ in fr_:
                    mpt_ = a_[:2] + (b_[:2] - a_[:2]) * f_; i_ = int(np.argmin(np.hypot(O[:, 0] - mpt_[0], O[:, 1] - mpt_[1])))
                    for din_ in np.arange(DAT_.get('pad_inset', 0.004), 0.022, 0.001):      # first spot inboard where the cover is 1.5 .. pad_max above the deck
                        q_ = O[i_] + ninb[i_] * din_; zd_ = float(zraw(q_[:1], q_[1:2])[0]); zc_ = _cov_inner(q_, DAT_.get('pad_r', 0.003))
                        if np.isfinite(zc_) and 0.0015 <= zc_ - zd_ <= DAT_.get('pad_max', 0.03) * (1.7 if KIT2 else 1.0) and reg.contains(Point(*q_)) and np.hypot(posts[:, 0] - q_[0], posts[:, 1] - q_[1]).min() > 0.009 \
                           and (not KIT2 or (max(_slope(q_ + np.array(o_)) for o_ in ((0, 0), (0.004, 0), (-0.004, 0), (0, 0.004), (0, -0.004))) <= CV.get('pad_slope', 0.09) and reg.buffer(-0.005).contains(Point(*q_)))): break
                    else: continue
                    rq_ = DAT_.get('pad_r', 0.003)
                    if KIT2:      # [fix2] R2E1 / R2E8: a cored pad (OD 6, wall 1.2 = 0.5 t), top fused into the inner skin all round, flat foot ON a level deck land
                        nq_ = 40; aq_ = np.linspace(0, 2 * np.pi, nq_, endpoint=False); cq_ = np.c_[np.cos(aq_), np.sin(aq_)]; ri_ = rq_ - 0.0012
                        to_ = np.array([_inner_z(q_[0] + c2_[0] * rq_, q_[1] + c2_[1] * rq_, zd_) for c2_ in cq_]); ti_ = np.array([_inner_z(q_[0] + c2_[0] * ri_, q_[1] + c2_[1] * ri_, zd_) for c2_ in cq_])
                        if not (np.isfinite(to_).all() and np.isfinite(ti_).all()) or min(to_.min(), ti_.min()) - zd_ < 0.0015: continue
                        RO_ = lambda r__, z__: np.c_[q_[0] + cq_[:, 0] * r__, q_[1] + cq_[:, 1] * r__, np.broadcast_to(z__, nq_)]
                        zfo_ = zraw(q_[0] + cq_[:, 0] * (rq_ - 0.0003), q_[1] + cq_[:, 1] * (rq_ - 0.0003)); zfi_ = zraw(q_[0] + cq_[:, 0] * (ri_ + 0.0003), q_[1] + cq_[:, 1] * (ri_ + 0.0003))
                        rings_ = [RO_(rq_, to_ + 0.0006), RO_(rq_, zfo_ + 0.0003), RO_(rq_ - 0.0003, zfo_ + 0.00002), RO_(ri_ + 0.0003, zfi_ + 0.00002), RO_(ri_, zfi_ + 0.0003), RO_(ri_, ti_ + 0.0006)]
                        MK_['c'] = 'seatpad'; CK.add(*stack(rings_ + [rings_[0]], closed=True), 0)
                        pads2_.append(dict(gap=gi_, at_mm=np.round(np.r_[q_, zd_] * 1000, 1).tolist(), length_mm=round((float(to_.min()) - zd_) * 1000, 1), deck_slope_deg=round(float(np.degrees(np.arctan(_slope(q_)))), 1)))
                        continue
                    MK_['c'] = 'seatpad'; CK.add(*lathe((q_[0], q_[1], zc_ + 0.0008), (0, 0, -1), [(0, 0.0), (0, rq_ + 0.001), (0.001, rq_), (zc_ + 0.0008 - zd_ - 0.0004, rq_), (zc_ + 0.0008 - zd_, rq_ - 0.0004), (zc_ + 0.0008 - zd_, 0.0)], 32), 0)
                    pads2_.append(dict(gap=gi_, at_mm=np.round(np.r_[q_, zd_] * 1000, 1).tolist(), length_mm=round((zc_ - zd_) * 1000, 1)))
            drn_ = []
            if DAT_.get('drains'):
                # [fix1] E2: the level runs of the joint (nose / tail brow z 190, flank base line z 123) collect what gets through the 0.8 gap on the deck
                #        land behind the body's top edge. Drain holes (slots 6 x 2.5) at the deck's lowest ring let it fall onto the sealed chassis top and
                #        out under the hem (rain-shield concept: the chassis electronics carry their own IPX5 sealing).
                zO_ = Pb[:, 2]; cand_ = []
                for nm_, sel_ in (('nose', (O[:, 0] > 0.2) & (np.abs(zO_ - 0.19) < 0.003)), ('tail', (O[:, 0] < -0.27) & (np.abs(zO_ - 0.19) < 0.003)),
                                  ('flank', (np.abs(zO_ - OB.get('z_base', 0.123)) < 0.003) & (O[:, 0] < -0.02) & (O[:, 0] > -0.25))):
                    ix_ = np.nonzero(sel_)[0]
                    if not len(ix_): continue
                    if nm_ == 'flank':
                        for sg_ in (1, -1):
                            jj_ = ix_[O[ix_, 1] * sg_ > 0]
                            for xq_ in DAT_['drains'].get('flank_x', [-0.06, -0.20]):
                                if len(jj_): cand_.append(jj_[np.argmin(np.abs(O[jj_, 0] - xq_))])
                    else:
                        for yq_ in DAT_['drains'].get('brow_y', [-0.09, 0.0, 0.09]): cand_.append(ix_[np.argmin(np.abs(O[ix_, 1] - yq_))])
                for i_ in cand_:
                    q_ = O[i_] + ninb[i_] * 0.0015; zq_ = Pb[i_, 2] - 0.0002
                    if KIT2:      # [fix2] R2E3: a real 6 x 3 slot through the deck at its lowest line (1 mm in from the deck / wall junction, where the water lies)
                        q_ = O[i_] + ninb[i_] * 0.0025; tq_ = tgb[i_]
                        HOLES_.append(('drain', shapely.LineString([q_ - tq_ * 0.0015, q_ + tq_ * 0.0015]).buffer(0.0015, quad_segs=10))); drn_.append(np.round(np.r_[q_, float(zraw(q_[:1], q_[1:2])[0])] * 1000, 1).tolist()); continue
                    rot = np.array([[tgb[i_, 0], -tgb[i_, 1], 0], [tgb[i_, 1], tgb[i_, 0], 0], [0, 0, 1.0]])
                    MK_['m'] = 'drain'; MB.add(*box((q_[0], q_[1], zq_ - 0.0002), (0.006, 0.0025, 0.0003), rot), M_DARK); drn_.append(np.round(np.r_[q_, zq_] * 1000, 1).tolist())
            datrep = dict(datum_pins=pins_, seat_pads=pads2_, deck_drains=drn_)
            if KIT2:
                # [fix2] R2E5 / F-S5: the deck is a real 2.5 mm moulded plate (closed solid): plan region = lip line + 0.8 mm (its edge runs 0.5 mm into
                #        the body wall: the joint) minus the control opening, minus REAL holes (leg windows, round nose datum hole, tail datum slot,
                #        drain slots, a notch round the chassis strut stub at x 120); top = the deck height field, bottom = top - 2.5 mm along the normal.
                for nx_ in CV.get('deck_notch', [[0.120, 0.1575, 0.120, 0.0055]]):      # [x0, |y|, x1, r]: a stadium clearance slot over a chassis rib
                    for sg_ in (1, -1): HOLES_.append(('notch', shapely.LineString([(nx_[0], sg_ * nx_[1]), (nx_[2] + 1e-6, sg_ * nx_[1])]).buffer(nx_[3], quad_segs=12)))
                for gh_ in ((LT_ or {}).get('harness', {}).get('grommet_holes', []) if LENS_ else []): HOLES_.append(('grommet', affinity.scale(Point(gh_[0], gh_[1]).buffer(1.0, quad_segs=24), gh_[2], gh_[3])))      # [fix3-lights] light harness grommets [x, y, rx, ry]
                for (xp2_, yp2_, _, _), rp2__ in zip(posts, PADR_): HOLES_.append(('post', Point(xp2_, yp2_).buffer(rp2__ - 0.0001, quad_segs=16)))      # the screw boss (pad) fills it
                LBDI_ = (WALL_.get('labyrinth') or {}).get('deck_inset') if WALL_ else None      # [fix3-labyrinth] where the deck edge meets the thickened body edge /
                if LBDI_ and PLINE is not None:      # [fix3-labyrinth] the upstand (side-view distance of the deck edge to the part line in [-t_len - taper, h + clear]),
                    zr_ = zraw(O[:, 0], O[:, 1]); cve_ = _pl.sdist(PLINE, O[:, 0], zr_)      # [fix3-labyrinth] it stops `extra` further in: 0.5 into the 3.13 edge
                    we_ = (1 - ss(LBDI_['below'][0], LBDI_['below'][1], -cve_)) * (1 - ss(LBDI_['above'][0], LBDI_['above'][1], cve_))      # [fix3-labyrinth]
                    ie_ = CV.get('deck_into_wall', 0.0008) - LBDI_['extra'] * we_      # [fix3-labyrinth] instead of 2.5; elsewhere unchanged (0.5 into the 2.5 wall)
                    dpg_ = Polygon(Pb[:, :2] + ninb * (CV.get('lip', 0.0028) - ie_)[:, None]).buffer(0).difference(op).difference(shapely.union_all([h_[1] for h_ in HOLES_]))      # [fix3-labyrinth]
                    LBDECK_ = dict(edge_points_moved_in=int((we_ > 0.01).sum()), of=int(len(we_)), extra_mm=LBDI_['extra'] * 1000)      # [fix3-labyrinth]
                else:      # [fix3-labyrinth]
                  dpg_ = Polygon(O).buffer(0).buffer(CV.get('deck_into_wall', 0.0008), join_style=1).difference(op).difference(shapely.union_all([h_[1] for h_ in HOLES_]))      # [fix3-labyrinth]
                dpg_ = dpg_.buffer(-0.0006, join_style=1).buffer(0.0006, join_style=1).buffer(0.0002).buffer(-0.0002)
                tdk_ = LG.get('deck_t', 0.0025); DKV_, DKF_ = [], []; dko_ = 0
                from scipy.spatial import Delaunay as _Del
                DECK4_ = dict(dpg=dpg_) if CV.get('deck_envelope') else None      # [fix4-cover2] G4: coverkit4.build_deck (no fold, vertical edge walls)
                for pp_ in ([] if DECK4_ else shapely.get_parts(dpg_)):      # [fix4-cover2]
                    if pp_.area < 2e-5: continue
                    pp_ = shapely.segmentize(pp_.simplify(0.0001), CV.get('deck_edge', 0.0007))
                    bpts_ = np.concatenate([np.asarray(r_.coords)[:-1] for r_ in [pp_.exterior] + list(pp_.interiors)])
                    hs_ = CV.get('deck_cell', 0.0012); bx_ = pp_.bounds; gx_, gy_ = np.meshgrid(np.arange(bx_[0], bx_[2], hs_), np.arange(bx_[1], bx_[3], hs_), indexing='ij')
                    gp_ = np.c_[gx_.ravel(), gy_.ravel()]; gp_ = gp_[shapely.contains_xy(pp_, gp_[:, 0], gp_[:, 1])]
                    gp_ = gp_[shapely.distance(pp_.boundary, shapely.points(gp_)) > 0.7 * CV.get('deck_edge', 0.0007) + 0.0004]
                    P2_ = np.r_[bpts_, gp_]; P2_ = np.unique(np.round(P2_, 9), axis=0); tri_ = _Del(P2_).simplices; cc_ = P2_[tri_].mean(1)
                    tri_ = tri_[shapely.contains_xy(pp_, cc_[:, 0], cc_[:, 1])]
                    ar2_ = (P2_[tri_[:, 1], 0] - P2_[tri_[:, 0], 0]) * (P2_[tri_[:, 2], 1] - P2_[tri_[:, 0], 1]) - (P2_[tri_[:, 2], 0] - P2_[tri_[:, 0], 0]) * (P2_[tri_[:, 1], 1] - P2_[tri_[:, 0], 1])
                    tri_ = np.where((ar2_ < 0)[:, None], tri_[:, ::-1], tri_); tri_ = tri_[np.abs(ar2_) > 1e-14]
                    for _it in range(10):      # no bow-tie vertices on the boundary (a vertex with two outgoing boundary edges): drop the smallest triangle there
                        ed_ = np.concatenate([tri_[:, [0, 1]], tri_[:, [1, 2]], tri_[:, [2, 0]]]); kk_ = ed_[:, 0] * len(P2_) + ed_[:, 1]; rk_ = ed_[:, 1] * len(P2_) + ed_[:, 0]
                        bd_ = ed_[~np.isin(kk_, rk_)]; cnt_ = np.bincount(bd_[:, 0], minlength=len(P2_)); bv_ = np.nonzero(cnt_ > 1)[0]
                        if not len(bv_): break
                        ar3_ = np.abs((P2_[tri_[:, 1], 0] - P2_[tri_[:, 0], 0]) * (P2_[tri_[:, 2], 1] - P2_[tri_[:, 0], 1]) - (P2_[tri_[:, 2], 0] - P2_[tri_[:, 0], 0]) * (P2_[tri_[:, 1], 1] - P2_[tri_[:, 0], 1]))
                        drop_ = set()
                        for v_ in bv_:
                            inc_ = np.nonzero((tri_ == v_).any(1))[0]; drop_.add(int(inc_[np.argmin(ar3_[inc_])]))
                        tri_ = np.delete(tri_, sorted(drop_), 0)
                    us_ = np.unique(tri_); mp2_ = -np.ones(len(P2_), np.int64); mp2_[us_] = np.arange(len(us_)); P2_ = P2_[us_]; tri_ = mp2_[tri_]
                    Zt_ = zraw(P2_[:, 0], P2_[:, 1]); T3_ = np.c_[P2_, Zt_]
                    fn2_ = np.cross(T3_[tri_[:, 1]] - T3_[tri_[:, 0]], T3_[tri_[:, 2]] - T3_[tri_[:, 0]]); vn2_ = np.zeros_like(T3_)
                    for c3_ in range(3): np.add.at(vn2_, tri_[:, c3_], fn2_)
                    vn2_ /= np.maximum(np.linalg.norm(vn2_, axis=1, keepdims=True), 1e-20); vn2_[vn2_[:, 2] < 0] *= -1
                    B3_ = T3_ - vn2_ * tdk_; nt_ = len(T3_)
                    ed_ = np.concatenate([tri_[:, [0, 1]], tri_[:, [1, 2]], tri_[:, [2, 0]]]); kk_ = ed_[:, 0] * nt_ + ed_[:, 1]; rk_ = ed_[:, 1] * nt_ + ed_[:, 0]
                    bd_ = ed_[~np.isin(kk_, rk_)]; a2_, b2_ = bd_[:, 0], bd_[:, 1]
                    sides_ = [np.c_[a2_, a2_ + nt_, b2_ + nt_], np.c_[a2_, b2_ + nt_, b2_]]
                    DKV_.append(np.r_[T3_, B3_]); DKF_.append(np.concatenate([tri_, tri_[:, ::-1] + nt_] + sides_) + dko_); dko_ += 2 * nt_
                if DECK4_: _dv4_, _df4_ = _ck4.build_deck(dpg_, zraw, CV, tdk_, refine=0 if CV.get('deck_clear') else None); DKV_, DKF_ = ([_dv4_], [_df4_]) if len(_df4_) else ([], [])      # [fix4-cover2]
                if DKV_:
                    MK_['m'] = 'deck'; MB.add(np.concatenate(DKV_), np.concatenate(DKF_), M_IN)
                    datrep.update(deck=dict(thickness_mm=tdk_ * 1000, faces=int(sum(len(f_) for f_ in DKF_)), holes={k2_: sum(1 for h_ in HOLES_ if h_[0].startswith(k2_)) for k2_ in ('window', 'pin', 'drain', 'notch')}, legs_at_mm=LEGAT_))
                    if 'LBDECK_' in globals(): datrep['deck']['labyrinth_inset'] = LBDECK_      # [fix3-labyrinth]
                print('deck v2:', datrep.get('deck'), file=sys.stderr)
            print('gaps:', [(gi_, np.round(0.5 * (a_[:2] + b_[:2]) * 1000).tolist(), h_) for gi_, (a_, b_, h_) in enumerate(GAPS_)], file=sys.stderr)
            print('datum pins:', pins_, ' seat pads:', len(pads2_), file=sys.stderr)
        print('room over screw head per post (mm, after a %.1f mm cover wall):' % (wl_ * 1000), [round(v * 1000, 1) for v in depth_.values()], file=sys.stderr)
        print('post to deck edge per post (mm):', [round(v * 1000, 1) for v in edge_], file=sys.stderr)      # [v18]
        crep = {'hidden_screws': len(posts), 'snap_legs': nleg, 'min_seat_mm': round(min(seat_) * 1000, 1), 'min_room_over_screw_head_mm': round(min(depth_.values()) * 1000, 1),
                'min_post_to_deck_edge_mm': round(min(edge_) * 1000, 1)}
        if LG and WALL_: crep.update(leg_free_length_mm_per_gap=legrep, leg_L_min_mm=LG.get('L_min', 0.016) * 1000, pad_r_mm=pads_)
        if LEGF_: crep.update(legs_fix1=dict(per_leg=LEGF_, retention_total_N=round(sum(l_['retain_N'] for l_ in LEGF_), 1), max_strain_pct=max(l_['strain_pct'] for l_ in LEGF_)))
        if datrep: crep.update(location=datrep)
        if any(_ck4.REP.values()): crep['fix4_cover2'] = _ck4.REP      # [fix4-cover2] leg moves / deck-following hooks / deck untangle
        print('snap legs: free length per gap between posts (mm):', legrep, ' pads r:', pads_, file=sys.stderr)
        mok = crep['min_seat_mm'] >= 1.0 and crep['min_room_over_screw_head_mm'] >= 0.0 and nleg >= CV.get('legs_min', 8) and crep['min_post_to_deck_edge_mm'] >= 3.0
    for lp in wloops:
        p = V[lp]; c2 = p[:, :2].mean(0); k = int(np.argmin(np.hypot(posts[:, 0] - c2[0], posts[:, 1] - c2[1]))); x_, y_, z_ = posts[k, :3]
        if np.hypot(c2[0] - x_, c2[1] - y_) > 0.002: continue
        dirs = p[:, :2] - [x_, y_]; dirs /= np.linalg.norm(dirs, axis=1, keepdims=True); n_ = len(p)
        zf = z_ + (ft if p[:, 2].min() - z_ - ft >= 0.0025 else 0.002)                 # thinner seat where the skin is close above the post
        ring = lambda r_, z: np.c_[x_ + dirs[:, 0] * r_, y_ + dirs[:, 1] * r_, np.full(n_, z) if np.isscalar(z) else z]
        sub = np.hypot(Cf[:, 0] - x_, Cf[:, 1] - y_) < 0.022; zs = skin_ring(x_, y_, dirs, rt, z_ + 0.001, sub)
        MK_['m'] = 'well'; MB.add(*stack([p, ring(rw, zf), ring(rh, zf)]), M_IN); MB.add(*stack([ring(rh, zf), ring(rh, z_ + 0.0002)]), M_DARK)      # well wall, seat, screw hole
        MB.add(*stack([ring(rh, z_ + 0.0002), ring(rt, z_ + 0.0002), ring(rt, zs)]), M_IN)                                          # foot on the post, tower up to the skin
        MK_['m'] = 'rib'
        for q_ in np.radians([45, 135, 225, 315]): rib(x_, y_, q_, rt, 0.009, z_, 0.011, sub)                                      # ribs tying the tower to the skin
        screw(x_, y_, zf); depth_[k] = float(p[:, 2].min() - zf)
    nund = 0
    def _ck4c(ring, z0_, zt_):      # [fix4-cover2] G5: a boss ring stack closed: top annulus in to the 1.3 hole, hole wall down to the foot
        if not (CV and CV.get('orient_kits')): return []      # [fix4-cover2]
        zh_ = float(min(z0_ + 0.012, np.min(zt_) - 0.0005)); return [ring(0.0013, zh_), ring(0.0013, z0_)]      # [fix4-cover2]
    for ub in MJ['under']:
        x_, y_, z_ = ub['x'], ub['y'], ub['z'] + 0.0002; q = np.linspace(0, 2 * np.pi, 40, endpoint=False); dirs = np.c_[np.cos(q), np.sin(q)]
        RB4_ = None      # [fix4-cover2] rear socket boss: foot on the SCANNED socket floor, radius clear of the scanned socket wall
        if ub['kind'] != 'front' and CV and CV.get('mount_chassis_clear'): import coverkit4 as _ck4; z_, RB4_ = _ck4.socket_boss(x_, y_, ub['r_socket'] * 0.001 - 0.0007, z_ - 0.0002, ub['z_socket_top'], CV['mount_chassis_clear']); z_ += 0.0002      # [fix4-cover2]
        ring = lambda r_, z: np.c_[x_ + dirs[:, 0] * r_, y_ + dirs[:, 1] * r_, np.full(40, z) if np.isscalar(z) else z]
        sub = (np.hypot(Cf[:, 0] - x_, Cf[:, 1] - y_) < 0.03) & SUBM
        if ub['kind'] == 'front':                      # long boss hanging from the skin above, standing on the wing of the chassis
            MK_['m'] = 'boss'; rb = 0.0042; zs = skin_ring(x_, y_, dirs, rb, z_ + 0.001, sub); MB.add(*stack([ring(0.0013, z_), ring(rb, z_), ring(rb, zs)] + _ck4c(ring, z_, zs)), M_IN)      # [fix4-cover2] _ck4c: G5 closed
            MK_['m'] = 'rib'
            for q_ in np.radians([0, 90, 180, 270]): rib(x_, y_, q_, rb, 0.010, z_, -0.6, sub)
        else:                                          # rear corner: boss sitting in the socket of the apron, carried by a bracket from the skirt wall
            MK_['m'] = 'bracket'; rb = RB4_ or ub['r_socket'] * 0.001 - 0.0007; zt = ub['z_socket_top'] + 0.0015;      # [fix4-cover2] RB4_
            sg_ = np.sign(y_); MB.add(*stack([ring(0.0013, z_), ring(rb, z_), ring(rb, zt)] + _ck4c(ring, z_, zt)), M_IN)      # [fix4-cover2] _ck4c
            sb2 = (np.abs(Cf[:, 0] - x_) < 0.03) & (np.abs(Cf[:, 2] - zt) < 0.04) & (Cf[:, 1] * sg_ > 0.15) & SUBM
            def ywall(xx, zz):
                yy = np.concatenate([_proj(sb2, 0, 2, 1, xx + dx_, zz + dz_) for dx_ in (-0.001, 0.001) for dz_ in (-0.0016, 0.0016)]) * sg_; yy = yy[yy > abs(y_)]
                return sg_ * (float(yy.min()) - 0.0014) if len(yy) else np.nan
            xa, xb = x_ - 0.0075, x_ + 0.0075; yi = y_ - sg_ * (rb + 0.001)      # [fix4-cover2] (Q on its own line)
            if CV and CV.get('mount_chassis_clear'): yi = _ck4.plate_inner(xa, xb, y_, yi, zt, zt + 0.003, CV['mount_chassis_clear'])      # [fix4-cover2] plate clear of the chassis apron
            Q = [(xa, yi), (xb, yi), (xb, ywall(xb, zt + 0.0015)), (xa, ywall(xa, zt + 0.0015))]      # [fix4-cover2]
            MB.add(*stack([np.array([(a_, b_, zt) for a_, b_ in Q]), np.array([(a_, b_, zt + 0.003) for a_, b_ in Q])], cap0=True, cap1=True), M_IN)
            for xx in (x_ - 0.005, x_ + 0.005):
                zft_ = zt + 0.016
                if g('fix1_outline', False):      # [fix1] the fin ends on the BODY wall: lowered until the body skin is there (a low part line puts the cover above it)
                    while zft_ > zt + 0.006 and not np.isfinite(ywall(xx, zft_ + 0.002)): zft_ -= 0.001
                yt_ = ywall(xx, zft_); yt_ = ywall(xx, zt + 0.003) if not np.isfinite(yt_) else yt_      # [orbit] the body wall may end below the fin top (low part line)
                _fin((xx, ywall(xx, zt + 0.003), zt + 0.003), (xx, ywall(xx, zt + 0.003) - sg_ * 0.012, zt + 0.003), (xx, yt_, zft_), th=0.0015)
        MK_['m'] = 'pilot'; MB.add(*stack([ring(0.0013, z_ + 0.012), ring(0.0013, z_)], cap0=True, cap1=bool(CV and CV.get('orient_kits'))), M_DARK); nund += 1      # [fix4-cover2] G5: cap1                                  # pilot hole for the screw from below
    nclip = 0; MK_['m'] = 'clip'
    for cl in MJ['clips']:                             # snap hooks: tongue from the inside of the tail wall into the window of the apron, barb behind the wall
        yc = (cl['y0'] + cl['y1']) / 2; zc = (cl['z0'] + cl['z1']) / 2; wy = min(0.006, cl['y1'] - cl['y0'] - 0.002)
        sb3 = (np.abs(Cf[:, 1] - yc) < 0.02) & (np.abs(Cf[:, 2] - zc) < 0.02) & (Cf[:, 0] < -0.25) & SUBM
        xin = lambda zz: max(float(_proj(sb3, 1, 2, 0, yc + dy_, zz + dz_).min()) for dy_ in (-wy / 2, 0, wy / 2) for dz_ in (-0.0012, 0.0012)) + 0.0012      # just inside the skin
        x0 = xin(zc); x1 = cl['x_below'] + 0.0085
        ov_ = 0.0001 if CV and CV.get('orient_kits') else 0.0      # [fix4-cover2] G5: the barb box overlaps the tongue (a shared edge made it non-manifold)
        MB.add(*box(((x0 + x1) / 2, yc, zc), (x1 - x0, wy, 0.0022)), M_IN); MB.add(*box((x1 - 0.0013, yc, zc - 0.0019 + ov_ / 2), (0.0026, wy, 0.0016 + ov_)), M_IN)      # [fix4-cover2]
        xr3 = xin(zc + 0.009)
        _fin((x0, yc, zc + 0.0011), (x0 + 0.008, yc, zc + 0.0011), (xr3, yc, zc + 0.009), th=0.0015); nclip += 1
    mrep = {'screws_from_above': len(depth_), 'well_min_depth_mm': (round(min(depth_.values()) * 1000, 1) if depth_ else None) if not CV else None, 'screws_from_below': nund, 'snap_hooks': nclip,
            'posts': len(posts)}
    if CV: mrep['cover'] = crep
    if CV and CV.get('skip_posts'): mrep['posts_skipped'] = list(CV['skip_posts'])
    mok = mok and mrep['screws_from_above'] == mrep['posts'] and (CV or mrep['well_min_depth_mm'] >= 2.0) and nund == len(MJ['under']) and nclip == len(MJ['clips'])

# ---- [v18c] wall thickness (shell_wall.py): the body and the cover become closed solids - outer skin, every free edge rolled ONCE
#      (radius per loop: wall.loop_r; the generator's own roll_edge is off for these edges), flat edge face, inner round, inner skin
#      at the wall t along a smoothed normal. Placed after the mounts (they ray-cast the OUTER skin and subtract the wall themselves).
SIDE = None
def collapse_needles(amax=1e-8, maxit=6):
    """[v18c] Needle triangles (area < amax m^2) that are turned over against the faired normal (a few at the tail end of the spine, where the
    outline columns converge) are removed by collapsing their shortest edge; their inner-skin copies would otherwise be folded too."""
    global F, lab
    nt_ = 0
    for it_ in range(maxit):
        P_ = V[F]; fn_ = np.cross(P_[:, 1] - P_[:, 0], P_[:, 2] - P_[:, 0]); a_ = 0.5 * np.linalg.norm(fn_, axis=1); dt_ = (fn_ * nrm_()[F].mean(1)).sum(1)
        sg_ = np.sign(np.median(dt_[a_ > 1e-7])); bad_ = np.nonzero((dt_ * sg_ < 0) & (a_ < amax))[0]
        if not len(bad_): break
        par_ = {}
        def fnd(i):
            while par_.get(i, i) != i: i = par_[i]
            return i
        for f_ in bad_:
            e_ = [(F[f_, k], F[f_, (k + 1) % 3]) for k in range(3)]; i_, j_ = min(e_, key=lambda e: np.linalg.norm(V[e[0]] - V[e[1]]))
            ri_, rj_ = fnd(i_), fnd(j_)
            if ri_ != rj_: par_[rj_] = ri_; V[ri_] = 0.5 * (V[ri_] + V[rj_]); nt_ += 1
        mp_ = np.arange(len(V)); ks_ = np.array(list(par_.keys()), np.int64); mp_[ks_] = [fnd(k) for k in ks_]; F = mp_[F]
        ok_ = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2]); F = F[ok_]; lab = lab[ok_]
    return nt_
if WALL_:
    print('needle edges collapsed before the wall:', collapse_needles(), file=sys.stderr)
    PART0 = ((A['cv'][F].mean(1) > gapc * 0.5) if CV else np.zeros(len(F), bool)).astype(np.uint8)
    W_ = shell_wall.build(V, F, lab, PART0, A, WALL_, exclude=(lab == KL['lid']), log=lambda *a: print(*a, file=sys.stderr))
    V, F, lab, SIDE, PART = W_.V, W_.F, W_.lab, W_.side, W_.part; A = W_.attrs; KL['inner'] = int(WALL_['bare_lab'])
    print('wall', json.dumps({k: v for k, v in W_.report.items() if k != 'loops'}), file=sys.stderr)
    for L_w in W_.report['loops']: print('  wall loop', L_w, file=sys.stderr)
    if CV and deck_ref is not None:            # the deck's first strip lay on the seam edge: it now starts inside the body wall under the land
        k_, n_ = deck_ref; Dv = MB.V[k_]; Pb0 = Dv[:n_].copy(); ne_, b_, t_, _ = W_.frame_at(Pb0, 0); t_ = t_[:, None]
        Dv[:n_] = Pb0 - b_ * 0.0006 - ne_ * (t_ - 0.0004); Dv[n_:2 * n_] = Pb0 - b_ * 0.0008 - ne_ * (t_ + 0.0003)      # ring 0 0.6 mm inside the wall's top edge (the joint gap stays the only gap to the cover)
    if SK and SK.get('seat') and SK['seat'].get('flange'):      # seat flange: inward ledge along the raised edge, its underside flush with the wall's edge face
        SF = SK['seat']['flange']
        for L_w in W_.loops:
            if L_w['part'] != 0 or L_w['name'] != 'hem': continue
            P0_ = V[L_w['v']] + L_w['r'] * L_w['b']; ne_ = L_w['ne']; t_l = W_.th[L_w['v']]
            for sg_ in (1, -1):
                k_ = (np.abs(P0_[:, 2] - SK['seat']['z']) < 0.0004) & (P0_[:, 1] * sg_ > 0.14) & (P0_[:, 0] > SK['seat']['x'][0] + SF.get('end', 0.003)) & (P0_[:, 0] < SK['seat']['x'][1] - SF.get('end', 0.003))
                if k_.sum() < 4: continue
                o_ = np.argsort(P0_[k_, 0]); Pk = P0_[k_][o_]; nk = ne_[k_][o_]; tk = t_l[k_][o_]; nh = nk.copy(); nh[:, 2] = 0; nh /= np.maximum(np.linalg.norm(nh, axis=1, keepdims=True), 1e-9)
                prof_ = [(tk - 0.0006, 0.0), (tk + SF.get('w', 0.004), 0.0), (tk + SF.get('w', 0.004), SF.get('t', 0.0025)), (tk - 0.0006, SF.get('t', 0.0025))]
                Rg_ = np.array([[p_ - n_ * a_ + np.array([0, 0, b_]) for p_, n_, a_, b_ in zip(Pk, nh, np.broadcast_to(a0_, len(Pk)), np.broadcast_to(b0_, len(Pk)))] for a0_, b0_ in prof_])
                MK_['m'] = 'seat'; MB.add(*stack(np.transpose(Rg_, (1, 0, 2)), closed=True, cap0=True, cap1=True), M_IN)
                if SF.get('tpe'): import coverkit4 as _ck4; MB.add(*_ck4.seat_pad(Pk, nh, tk, SF, SK['seat']['z']), M_DARK)      # [fix4-cover2] m12: 2K TPE bump pad over the drive-housing crest (kind 'seat')
    if CV and CV.get('deck_clear') and globals().get('DECK4_'):      # [fix4-cover2] G4: deck clearance to the cover solid by its outline, not by a vertex push
        from labyrinthkit import RobustSurfDist as _RSD4; _SD4 = _RSD4(V, F[(PART == 1) & (lab != KL['lid'])], sign_within=0.002)      # [fix4-cover2]
        _kd4 = [k_ for k_, (_, kd_) in enumerate(MKF) if kd_ == 'deck'][0]; _o4 = sum(len(v_) for v_ in MB.V[:_kd4])      # [fix4-cover2]
        _dv4_, _df4_, DECK4_['dpg'], _ = _ck4.deck_clear(DECK4_['dpg'], zraw, CV, LG.get('deck_t', 0.0025), (MB.V[_kd4], MB.F[_kd4] - _o4), lambda X_: _SD4(X_, 0.004)[0], CV['deck_clear'])      # [fix4-cover2]
        _ck4.bag_replace(MB, _kd4, _dv4_, _df4_); MKF[_kd4] = (len(_df4_), 'deck'); print('deck clear:', _ck4.REP.get('deck_clear'), file=sys.stderr)      # [fix4-cover2]
    PUSH = WALL_.get('push_clear', 0.0)
    if CV and PUSH > 0:                        # every hidden part of the body keeps PUSH from the cover solid: vertices closer are moved straight away from it
        SDc = shell_wall.SurfDist(V, F[(PART == 1) & (lab != KL['lid'])]); Pc3 = V[F[(PART == 1) & (lab != KL['lid'])]]; pushed = 0
        for k_, (nf_, kd_) in enumerate(MKF):
            if kd_ in ('screw', 'slot', 'pilot', 'drain'): continue
            if kd_ == 'deck' and CV.get('deck_push') is False: continue      # [fix4-cover2] G4: the deck keeps its clearance by construction (coverkit4.deck_clear); a push folds it
            X_ = MB.V[k_]
            for it_ in range(10):
                s_, fi_ = SDc(X_, 0.01); bad_ = np.isfinite(s_) & (s_ < PUSH)
                if deck_ref is not None and k_ == deck_ref[0]: bad_[:2 * deck_ref[1]] = False             # rings 0 / 1 lie inside the body wall
                if not bad_.any(): break
                q_ = shell_wall._pt_tri(X_[bad_], Pc3[fi_[bad_], 0], Pc3[fi_[bad_], 1], Pc3[fi_[bad_], 2]); dv_ = X_[bad_] - q_
                dv_ *= np.sign(s_[bad_])[:, None] / np.maximum(np.linalg.norm(dv_, axis=1, keepdims=True), 1e-12); X_[bad_] = q_ + dv_ * (PUSH + 0.00005); pushed += int(bad_.sum())
        print('push: %d hidden-part vertices moved to %.1f mm from the cover' % (pushed, PUSH * 1000), file=sys.stderr)
    if hole_pts:                               # charger sleeves continue the openings' edge faces back to the chassis contacts
        SL = Bag()
        for p_ in hole_pts:
            p0_ = W_.edge_point(p_, 0); q_ = p0_.copy()
            if PSL_ and P3_:      # [fix3-ports] closed tunnel + web with two mirror-symmetric windows + ducts to the chassis (portskit.py)
                import portskit; _cb = np.load('dev/base_parts.npz'); _Xc = portskit.chassis_samples(_cb['Chassis_v'], _cb['Chassis_f'])      # [fix3-ports]
                pv3_, pf3_, pl3_, PORTS3_REP, pw3_ = portskit.build(P3_, p0_, float(p0_[:, 0].min()) - PSL_.get('web', 0.003), _Xc, log=lambda *a: print(*a, file=sys.stderr), p_out=p_)      # [fix3-ports] [fix4-front2] p_out: lip crest
                SL.add(pv3_, pf3_, 0); extra.update(ports_wp=pw3_['Wp'], ports_wm=pw3_['Wm'], ports_tw=pw3_['Tw'], ports_xw=np.array([pw3_['x_w']]))      # [fix3-ports]
                continue      # [fix3-ports]
            if PSL_:      # [fix2] slot sleeve back to a web plate set web mm behind the face; the two port sleeves continue from its back face
                xw_ = float(p0_[:, 0].min()) - PSL_.get('web', 0.003); q_[:, 0] = xw_; SL.add(*stack([p0_, q_]), 0)
                pl_ = PORTCUT.buffer(0.0003).difference(ports)
                SL.add(*lidmech.extrude(pl_, 0.0, 0.0012, (np.array([xw_ + 0.0001, 0.0, 0.0]), (0, 1.0, 0), (0, 0, 1.0), (-1.0, 0, 0)), bev=0.0003, nb=2), 0)
                for prt_ in shapely.get_parts(ports):
                    rg_ = lidmech.resample(prt_, 0.0006); r0_ = np.c_[np.full(len(rg_), xw_ - 0.001), rg_]; r1_ = r0_.copy(); r1_[:, 0] = g('port_back_x', 0.3082)
                    SL.add(*stack([r0_, r1_], cap1=True), 0)
                continue
            q_[:, 0] = g('port_back_x', 0.3082); SL.add(*stack([p0_, q_], cap1=True), 0)
        sv_, sf_, _ = SL.out(); extra.update(sleeve_v=sv_, sleeve_f=sf_)

# ---- checks
H = np.load('chassis_top.npy'); tv = A['t']; used = np.zeros(len(V), bool); used[F.ravel()] = True
gx = ((V[:, 0] + 0.34) / 0.005).astype(int); gy = ((V[:, 1] + 0.25) / 0.005).astype(int)
ok = used & (gx >= 0) & (gx < H.shape[0]) & (gy >= 0) & (gy < H.shape[1]) & (tv > 0.3) & (np.abs(V[:, 1]) < 0.15)
cl_in = np.zeros(0)
if SIDE is not None:      # [v18c] legacy rule on the outer skin, new rule on the inner skin
    vo_ = np.zeros(len(V), bool); vo_[F[SIDE == 0].ravel()] = True; vi_ = np.zeros(len(V), bool); vi_[F[SIDE == 1].ravel()] = True
    hin = (ok & vi_).nonzero()[0]; hin = hin[H[gx[hin], gy[hin]] > 0.05]; cl_in = V[hin, 2] - H[gx[hin], gy[hin]]; ok = ok & vo_
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
if SKB_ is not None and np.isin(SKB_['part'], [stopkit.PART['socket'], stopkit.PART['halo']]).any():      # [fix4-stop2] the v4 socket + halo ring close the lid skin's d 49.8 cut
    SV4_ = SKB_['v'][np.unique(SKB_['f'][np.isin(SKB_['part'], [stopkit.PART['socket'], stopkit.PART['halo']])])]; np.maximum.at(Hb, (np.clip(((SV4_[:, 0] + 0.34) / 0.005).astype(int), 0, H.shape[0] - 1), np.clip(((SV4_[:, 1] + 0.25) / 0.005).astype(int), 0, H.shape[1] - 1)), SV4_[:, 2])      # [fix4-stop2]
Hb = maximum_filter(Hb, 3)
cxs = -0.34 + (np.arange(H.shape[0]) + 0.5) * 0.005; cys = -0.25 + (np.arange(H.shape[1]) + 0.5) * 0.005; CX, CY = np.meshgrid(cxs, cys, indexing='ij')
strut_ = (np.abs(CY) > 0.158) & (CX > -0.06) & (CX < 0.29)           # mudguard struts: removed with the mudguards
expo = (H > 0.150) & ~strut_ & (Hb < H + 0.002)
lidv = np.unique(F[lab == 4]); b = poly.bounds
ar = np.linalg.norm(np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]), axis=1)
if SIDE is not None:      # [fix2] F-S1: half-width beside the wheels measured on the FINAL outer skins (after the step / sink offsets and the wall)
    _uo = np.zeros(len(V), bool); _uo[F[SIDE == 0].ravel()] = True; _dxf = V[:, 0] - WX
    _inwf = _uo & (np.abs(_dxf) < WR + 0.006) & ((V[:, 2] >= WZ) | (_dxf * _dxf + (V[:, 2] - WZ) ** 2 < (WR + 0.008) ** 2)) & (V[:, 2] < 0.30)
    half_w_final = float(np.abs(V[_inwf, 1]).max()); half_w_final_at = (V[_inwf][np.argmax(np.abs(V[_inwf, 1]))] * 1000).round(1).tolist()
else: half_w_final = half_w_wheels; half_w_final_at = None
rep = {'faces': int(len(F)), 'roof_min_clear_over_chassis_mm': round(float(cl.min()) * 1000, 1) if len(cl) else None,
       'max_half_width_beside_wheels_mm': round(max(half_w_wheels, half_w_final) * 1000, 2), 'half_width_form_mm': round(half_w_wheels * 1000, 2),
       'half_width_final_outer_skin_mm': round(half_w_final * 1000, 2), 'half_width_final_at_mm': half_w_final_at, 'wheel_inner_face_mm': 175.0,
       'lid_min_height_mm': round(float(V[lidv, 2].min()) * 1000, 1), 'controls_top_mm': round(K['ctl_top'] * 1000, 1),
       'lid_covers_controls': bool(b[0] <= -0.128 and b[2] >= 0.034 and b[1] <= -0.062 and b[3] >= 0.062),
       'body_max_height_mm': round(float(Vu[:, 2].max()) * 1000, 1),
       'min_surface_radius_mm': round(rmin * 1000, 1), 'max_crease_deg': round(crease, 1),
       'chassis_exposed_cm2': round(float(expo.sum()) * 0.25, 1), 'ports_centre_y_mm': round(PORT_CY * 1000, 2),
       'smallest_triangle_mm2': float(np.round(ar.min() * 5e5, 6)), 'lid_edge_points': int(len(lid_loop)), 'mounts': mrep, 'lid': LIDREP}
if PSL_ and PORTS3_REP: rep['ports3'] = PORTS3_REP      # [fix3-ports]
wok_ = True
if SIDE is not None:
    rep['roof_min_clear_inner_mm'] = round(float(cl_in.min()) * 1000, 1) if len(cl_in) else None
    if len(cl_in): rep['roof_min_clear_inner_at_mm'] = (V[hin[np.argmin(cl_in)]] * 1000).round(1).tolist()
    rep['wall'] = {k: v for k, v in W_.report.items()}
    wok_ = all(c_['boundary_edges'] == 0 and c_['nonmanifold_edges'] == 0 and c_['inner_outside_outer'] == 0 for c_ in W_.report.get('check', {}).values())
rep['ok'] = bool(((rep['roof_min_clear_over_chassis_mm'] or 9) >= 2.0 if SIDE is None else ((rep['roof_min_clear_inner_mm'] or 9) >= WALL_.get('min_clear', 0.0) * 1000 and wok_)) and rep['max_half_width_beside_wheels_mm'] <= 171.0
                 and rep['lid_min_height_mm'] >= rep['controls_top_mm'] + 4 and rep['lid_covers_controls']
                 and rep['min_surface_radius_mm'] >= g('min_radius', 3.0 if g('ridges', []) else 8.0) and rep['chassis_exposed_cm2'] <= 2.0
                 and mok)
# ---- shading normals, computed here in double precision (angle weighted, creases sharper than smooth_deg stay sharp)
SLIVER = None
if g('weld_final', False):
    # [fix1] D9: float32-coincident corners are WELDED (union of the coincident vertices, collapsed faces dropped) instead of deleting the degenerate
    #        faces, which left 90-94 open micro edges in the cover at the status light; zero-area slivers stay in the topology (no hole) but get no
    #        weight in the shading normals.
    from scipy.sparse import coo_matrix as _coo
    from scipy.sparse.csgraph import connected_components as _cc
    V32 = V.astype(np.float32); pr_ = []
    for i_, j_ in ((0, 1), (1, 2), (0, 2)):
        mm_ = (V32[F[:, i_]] == V32[F[:, j_]]).all(1) & (F[:, i_] != F[:, j_]); pr_.append(np.c_[F[mm_, i_], F[mm_, j_]])
    pr_ = np.concatenate(pr_); nw_ = 0
    if len(pr_):
        G_ = _coo((np.ones(len(pr_)), (pr_[:, 0], pr_[:, 1])), shape=(len(V), len(V))); _, cl_ = _cc(G_, directed=False)
        rep_ = np.full(cl_.max() + 1, len(V)); np.minimum.at(rep_, cl_, np.arange(len(V))); mp_ = rep_[cl_]; nw_ = int((mp_ != np.arange(len(V))).sum()); F = mp_[F]
    kp_ = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    fsrt = np.sort(F, 1); _, inv_, cnt_ = np.unique(fsrt, axis=0, return_inverse=True, return_counts=True); inv_ = inv_.ravel()
    for gq_ in np.nonzero(cnt_ > 1)[0]:          # duplicates: same winding -> keep one; opposite windings (a fin) -> drop all
        ids_ = np.nonzero(inv_ == gq_)[0]; rr_ = {tuple(np.roll(F[i], -int(np.argmin(F[i])))) for i in ids_}
        kp_[ids_ if len(rr_) > 1 else ids_[1:]] = False
    print('final weld: %d vertices welded, %d faces dropped' % (nw_, int((~kp_).sum())), file=sys.stderr)
    F = F[kp_]; lab = lab[kp_]
    if SIDE is not None: SIDE = SIDE[kp_]; PART = PART[kp_]
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); fa = np.linalg.norm(fn, axis=1); okf = np.ones(len(F), bool)
    SLIVER = fa <= 1e-13; fa = np.where(SLIVER, 1.0, fa); fn = np.where(SLIVER[:, None], 0.0, fn)
else:
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]]); fa = np.linalg.norm(fn, axis=1); okf = fa > 1e-13
    V32 = V.astype(np.float32); okf &= ~((V32[F[:, 0]] == V32[F[:, 1]]).all(1) | (V32[F[:, 1]] == V32[F[:, 2]]).all(1) | (V32[F[:, 0]] == V32[F[:, 2]]).all(1))
    fsrt = np.sort(F, 1); _, fi = np.unique(fsrt, axis=0, return_index=True); dup = np.ones(len(F), bool); dup[fi] = False; okf &= ~dup
F = F[okf]; lab = lab[okf]; fn = fn[okf] / fa[okf][:, None]
if SIDE is not None: SIDE = SIDE[okf]; PART = PART[okf]
if fn[(A['t'][F].mean(1) > 0.9) & ((SIDE == 0) if SIDE is not None else True)][:, 2].mean() < 0: F = F[:, ::-1].copy(); fn = -fn              # outward winding (outer skin decides)
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
if SLIVER is not None and SLIVER.any(): LN[SLIVER] = vn[F[SLIVER]]          # [fix1] zero-area slivers: plain vertex normals
if SK and SIDE is None:      # the lower edge gets a return flange (wall thickness seen from below / behind); [v18c] a real wall closes it itself
    hl = min((lp for lp in loops_of(F[lab != 4]) if np.ptp(V[lp, 0]) > 0.5), key=lambda lp: V[lp, 2].mean()); pH = V[hl]; nH = vn[hl].copy(); nH[:, 2] = 0; ln_ = np.linalg.norm(nH, axis=1)
    okh = (ln_ > 0.35) & (pH[:, 0] < 0.02); nH /= np.maximum(ln_[:, None], 1e-9); qH = pH - nH * SK.get('lip', 0.0028); i0 = np.arange(len(hl)); i1 = (i0 + 1) % len(hl); kq = okh[i0] & okh[i1]
    MK_['m'] = 'hem_lip'; MB.add(np.concatenate([pH, qH]), np.concatenate([np.c_[i0[kq], i1[kq], i1[kq] + len(hl)], np.c_[i0[kq], i1[kq] + len(hl), i0[kq] + len(hl)]]), M_IN)
if STAKE_CK:      # [fix1] E3: heat-stake pins of the cover (fused 0.8 into the wall) through the well ears, staked heads under them
    for cE_, zE_ in STAKE_CK:
        rs_ = STK_.get('pin_r', 0.001); hd_ = 0.0008 + 0.00005 + 0.0015 + (0.0002 if not STK_.get('pin_r') else 0.0003)      # [fix2] R2E8: pin r 0.75 (0.6 t)
        MK_['c'] = 'stake'; CK.add(*lathe((cE_[0], cE_[1], zE_ + 0.0008), (0, 0, -1), [(0, 0.0), (0, rs_ + 0.0006), (0.0008, rs_), (hd_, rs_),
                                                                         (hd_, rs_ + 0.0008), (hd_ + 0.0008, rs_ + 0.0004), (hd_ + 0.001, 0.0)], 24), 0)
mv_, mf_, ml_ = MB.out(); extra.update(mount_v=mv_, mount_f=mf_, mount_lab=ml_)
if CV and CV.get('orient_kits'): import coverkit4 as _ck4; extra['mount_f'] = _ck4.orient(mv_, mf_, 'mount', np.concatenate([np.full(n_, MKN.index(k_)) for n_, k_ in MKF]) if MKF else None).astype(mf_.dtype)      # [fix4-cover2] G5: consistent, outward winding per component (per kind)
extra.update(mount_kind=np.concatenate([np.full(n_, MKN.index(k_), np.int8) for n_, k_ in MKF]) if MKF else np.zeros(0, np.int8), mount_kinds=np.array(MKN),
             ckit_kind=np.concatenate([np.full(n_, CKN.index(k_), np.int8) for n_, k_ in CKF]) if CKF else np.zeros(0, np.int8), ckit_kinds=np.array(CKN))
cv_, cf2_, cl2_ = CK.out(); extra.update(ckit_v=cv_, ckit_f=cf2_, ckit_lab=cl2_, part=((A['cv'][F].mean(1) > gapc * 0.5) if CV else np.zeros(len(F), bool)).astype(np.uint8))
if CV and CV.get('orient_kits'): extra['ckit_f'] = _ck4.orient(cv_, cf2_, 'ckit').astype(cf2_.dtype)      # [fix4-cover2] G5
if SIDE is not None: extra.update(part=PART.astype(np.uint8), side=SIDE.astype(np.uint8), wall_t=np.array([WALL_['t']]))      # [v18c] side: 0 outer, 1 inner, 2 rim
used = np.zeros(len(V), bool); used[F.ravel()] = True
rmap = -np.ones(len(V), np.int64); rmap[used] = np.arange(int(used.sum()))
CSI_ = g('controls_standin', None)
if CSI_:
    # [fix1] D6 / F10: visualisation stand-in over the scanned controls (fixed hardware; footprint and heights kept): a flat 0.6 mm smoked window over
    #        the display (the scan's glass is lumpy) and a STOP key cap with a fair planar top just above the scanned relief (legend added by the renderer).
    c_ = np.load('control.npz'); cv0_, cf0_, cl0_ = c_['v'], c_['f'], c_['lab']; SB_ = Bag(); csrep_ = {}
    Pg_ = cv0_[np.unique(cf0_[cl0_ == 2])]; m_ = CSI_.get('margin', 0.0004)
    gx0_, gx1_, gy0_, gy1_ = Pg_[:, 0].min() - m_, Pg_[:, 0].max() + m_, Pg_[:, 1].min() - m_, Pg_[:, 1].max() + m_; zg_ = float(Pg_[:, 2].max()) + 0.00015
    if CSI_.get('symmetric'):      # [fix2] G7: the window is centred on the well axis (y = 0): it still covers the display + margin, 4.4 mm to the box both sides
        hy_ = max(-gy0_, gy1_); gy0_, gy1_ = -hy_, hy_
    rg_ = rrect(gx1_ - gx0_, gy1_ - gy0_, CSI_.get('glass_r', 0.004), 10) + [0.5 * (gx0_ + gx1_), 0.5 * (gy0_ + gy1_)]
    SB_.add(*stack([np.c_[rg_, np.full(len(rg_), zg_ - 0.0004)], np.c_[rg_, np.full(len(rg_), zg_ + 0.0006)]], closed=True, cap0=True, cap1=True), 0)
    Pk_ = cv0_[np.unique(cf0_[cl0_ == 1])]; Pk2_ = Pk_[Pk_[:, 2] > 0.254]
    kx0_, kx1_, ky0_, ky1_ = Pk2_[:, 0].min() - 0.0003, Pk2_[:, 0].max() + 0.0003, Pk2_[:, 1].min() - 0.0003, Pk2_[:, 1].max() + 0.0003
    top_ = Pk_[Pk_[:, 2] > 0.2585]; A2_ = np.c_[np.ones(len(top_)), top_[:, 0]]; cfk_ = np.linalg.lstsq(A2_, top_[:, 2], rcond=None)[0]
    cfk_[0] += float(np.max(Pk_[:, 2] - (cfk_[0] + cfk_[1] * Pk_[:, 0]))) + 0.00015      # plane just above every scanned point (letters included)
    zk_ = lambda X: cfk_[0] + cfk_[1] * X[:, 0]; rk_ = CSI_.get('key_r', 0.006); ck_ = np.array([0.5 * (kx0_ + kx1_), 0.5 * (ky0_ + ky1_)])
    Rk_ = []; eR_ = CSI_.get('key_edge', 0.0012)
    base_ = rrect(kx1_ - kx0_, ky1_ - ky0_, rk_, 12) + ck_
    Rk_.append(np.c_[base_, np.full(len(base_), float(np.percentile(cv0_[np.unique(cf0_[cl0_ == 0])][:, 2], 99)) - 0.0002)])
    for a_ in np.linspace(0, np.pi / 2, 6):      # quarter-round top edge of radius eR_
        rr_ = rrect(kx1_ - kx0_ - 2 * eR_ * (1 - np.cos(a_)), ky1_ - ky0_ - 2 * eR_ * (1 - np.cos(a_)), rk_ - eR_ * (1 - np.cos(a_)), 12) + ck_
        Rk_.append(np.c_[rr_, zk_(rr_) - eR_ + eR_ * np.sin(a_)])
    SB_.add(*stack(Rk_, closed=True, cap1=True), 1)
    sv_, sf_, sl_ = SB_.out(); extra.update(ctlsi_v=sv_, ctlsi_f=sf_, ctlsi_lab=sl_, ctlsi_key=np.array([ck_[0], ck_[1], cfk_[0], cfk_[1], kx1_ - kx0_, ky1_ - ky0_]))
    if STOPPAD is not None and CSI_.get('plunger', True):
        # [fix2] R2E4: STOP plunger on the lid underside, under the STOP pad, over the scanned STOP key: a cored boss (OD 7, wall 1.2) fused into the lid
        #        skin with a TPE tip; closed it stands gap_pl above the key cap; at the latch over-travel the lid turns down and the tip presses the key.
        xq_ = STOPPAD['x']; zlu_ = float(zi(np.array([[xq_, 0.0]]))[0]) - LP['t_lid'] + 0.0006
        nk_ = np.hypot(Pk_[:, 0] - xq_, Pk_[:, 1]) < 0.0045; zkt_ = float(Pk_[nk_, 2].max()) if nk_.any() else float(cfk_[0] + cfk_[1] * xq_)      # the scanned key under the tip
        gpl_ = CSI_.get('plunger_gap', 0.0005); ztp_ = zkt_ + gpl_; ro_, ri2_ = 0.0035, 0.0023
        PB_ = Bag()
        Lb_ = zlu_ - ztp_ - 0.0015
        PB_.add(*lathe((xq_, 0.0, zlu_), (0, 0, -1), [(0, 0.0), (0, ro_), (Lb_, ro_), (Lb_, ri2_), (0.001, ri2_), (0.001, 0.0)], 40), lidmech.K_D)
        PB_.add(*lathe((xq_, 0.0, ztp_ + 0.0015), (0, 0, -1), [(0, 0.0), (0, ro_ - 0.0004), (0.0011, ro_ - 0.0004), (0.0015, ro_ - 0.0012), (0.0015, 0.0)], 40), lidmech.K_RUB)
        pv_, pf_, pl_ = PB_.out(); k0_ = len(extra['kitm_v'])
        extra.update(kitm_v=np.concatenate([extra['kitm_v'], pv_]), kitm_f=np.concatenate([extra['kitm_f'], pf_ + k0_]), kitm_lab=np.r_[extra['kitm_lab'], pl_.astype(np.int8)],
                     kitm_part=np.r_[extra['kitm_part'], np.full(len(pf_), lidmech.PM['plunger'], np.int16)])
        over_ = float(lidmech.DEFAULTS['latch_over'] * (xq_ - LK['Q'][0]) / (LIDREP['latch_x_mm'] / 1000.0 - LK['Q'][0]))
        LIDREP['stop_plunger'] = dict(at_mm=[round(xq_ * 1000, 1), 0.0], tip_gap_closed_mm=round(gpl_ * 1000, 2), travel_at_over_mm=round(over_ * 1000, 2), key_press_mm=round((over_ - gpl_) * 1000, 2),
                                      key_top_mm=round(zkt_ * 1000, 2), length_mm=round((zlu_ - ztp_) * 1000, 1))
        rep['lid'] = LIDREP; print('stop plunger:', LIDREP['stop_plunger'], file=sys.stderr)
    print('controls stand-in: glass top z %.2f, key plane z = %.4f %+.4f x, key %.1f x %.1f mm' % (zg_ * 1000 + 0.6, cfk_[0], cfk_[1], (kx1_ - kx0_) * 1000, (ky1_ - ky0_) * 1000), file=sys.stderr)
if STOPMARK is not None:      # [fix1] E4: per-face paint of the lid (0 lid colour, 1 STOP orange); faces were cut exactly on the mark outline
    Cl_ = V[F].mean(1); extra['lid_paint'] = ((lab == 4) & shapely.contains_xy(STOPMARK, Cl_[:, 0], Cl_[:, 1])).astype(np.uint8)
if STOPPAD is not None:       # [fix2] lid paint: 0 lid colour, 1 STOP pad (accent), 2 stop square (white), 3 status halo (light)
    Cl_ = V[F].mean(1); rr_ = np.hypot(Cl_[:, 0] - STOPPAD['x'], Cl_[:, 1]); lp_ = np.zeros(len(F), np.uint8)
    lp_[rr_ < STOPPAD['r']] = 1; lp_[(rr_ < STOPPAD['r']) & shapely.contains_xy(STOPPAD['icon'], Cl_[:, 0], Cl_[:, 1])] = 2
    lp_[(rr_ > STOPPAD['r'] + STOPPAD['hg']) & (rr_ < STOPPAD['r'] + STOPPAD['hg'] + STOPPAD['hw'])] = 3; lp_[lab != 4] = 0; extra['lid_paint'] = lp_
if P.get('struts'):           # [fix3-struts] the chassis cut (strutskit.py) travels with the design: renderer / checks read it from the npz
    import strutskit as _sk; _SKS = _sk.spec(P); extra['struts_spec'] = np.array(json.dumps(_SKS))      # [fix3-struts]
    _BP = np.load('dev/base_parts.npz'); _cv, _cf, _ci = _sk.cap_mesh(_BP['Chassis_v'].astype(float), _BP['Chassis_f'].astype(np.int64), _SKS)      # [fix3-struts] the cut faces (flat caps), world frame
    extra.update(struts_cap_v=_cv, struts_cap_f=_cf, struts_cap_cut=_ci)      # [fix3-struts] drawn by design_render.py (welded into the cut chassis)
    rep['struts'] = {'mode': _SKS['mode'], 'cuts': [c_['id'] for c_ in _SKS['cuts']], 'seat_z_mm': round(SK['seat']['z'] * 1000, 2) if SK and SK.get('seat') else None}      # [fix3-struts]
if LENS_PATCH:      # [fix3-lights] lens solids, lamp modules, mounting bosses, harness, halo feed (lightkit.py) -> lights_* arrays
    import lightkit      # [fix3-lights]
    LKX_ = dict(patches=LENS_PATCH, P=LT_, wall=WALL_, V=V, F=F, side=SIDE, part=PART if SIDE is not None else None, lab=lab,      # [fix3-lights]
                mount=(extra['mount_v'], extra['mount_f'], extra['mount_kind'], [str(k_) for k_ in extra['mount_kinds']]), lid=dict(Q=LK['Q'], stoppad=STOPPAD, zi=zi, t_lid=LP['t_lid']))      # [fix3-lights]
    LKX_['ports'] = (extra.get('sleeve_v', np.zeros((0, 3))), extra.get('sleeve_f', np.zeros((0, 3), np.int64)))      # [fix3-int] fix-ports' port frame (the lamp housing clears it)
    if SKB_ is not None: LKX_['lid']['stopkey'] = dict(xk=SKB_['L']['xk'], yk=SKB_['L']['yk'], z0=SKB_['L']['z0'], gx=SKB_['L']['grad'][0], fuse=0.0016, bore_r=SKB_['L']['p']['bore_r'], boss_r=SKB_['L']['p']['boss_r'])      # [fix3-int] the halo boss to feed
    if SKB_ is not None and SKV4_:      # [fix4-stop2] v4: vertical pipe into the halo ring's front point, emitter board with the hall pair under the magnet
        p4_ = SKB_['L']['p']; xf4_, yf4_ = SKB_['L']['feed']
        LKX_['lid']['stopkey'].update(v4=True, feed=(xf4_, yf4_), halo_r=p4_['halo_r'], ring_bot_z=float(zi(np.array([[xf4_, yf4_]]))[0]) + p4_['halo_bot'], board=p4_['board'], board_x0=p4_['board_x0'],
                                      board_y0=p4_['board_y0'], hall_pkg=p4_['hall']['pkg'], hall_dy=p4_['hall']['dy'], magnet_y=p4_['magnet'][2])      # [fix4-stop2]
    if _os.environ.get('LIGHTS_DUMP'): import pickle; pickle.dump(LKX_, open(_os.environ['LIGHTS_DUMP'], 'wb'))      # standalone iteration of lightkit.py      # [fix3-lights]
    LKR_ = lightkit.build(LKX_, log=lambda *a: print(*a, file=sys.stderr)); extra.update(LKR_['arrays']); rep['lights'] = LKR_['report']      # [fix3-lights]
np.savez(OUT, v=V[used], f=rmap[F].astype(np.int32), lab=lab, ln=LN.astype(np.float32), lid_poly=ring_xy, **extra)
print(json.dumps(rep))      # [fix2] printed last (the STOP plunger report is added after the shading normals)
