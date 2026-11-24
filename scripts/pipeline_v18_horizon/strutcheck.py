# [fix3-struts] The chassis beside the drive wheels against the body, feature by feature (strutskit.py).
# usage: python strutcheck.py DESIGN.npz dev/base_parts.npz [OUT.json] [--uncut]
#   the chassis as cut by the npz's struts_spec (--uncut: the scan as it is, nothing removed = what the body would have to take)
# per feature and side: chassis samples (vertices + 0.4 mm face samples + cap samples) ->
#   min gap to the body solid / the cover solid / every closed body-kit solid (deck, seat flange, ribs, clips), points inside each,
#   how far it reaches past the body wall's inner / outer skin (horizontal rays along y at the same x, z),
#   drive housing: gap to the raised seat edge; wheels: gap body -> tyre; cuts: cap area, removal depth, removed vertices.
import numpy as np, sys, json
sys.path.insert(0, '.')
from shell_wall import SurfDist
import strutskit as sk
from scipy.spatial import cKDTree

class YRay:
    """horizontal rays along +-y: every crossing |y| of a triangle set at given (x, z)"""
    def __init__(s, V, F, cell=0.001):
        s.T = V[F]; xz = s.T[:, :, [0, 2]]; lo = xz.min(1); hi = xz.max(1); s.c = cell
        i0 = np.floor(lo / cell).astype(np.int64); i1 = np.floor(hi / cell).astype(np.int64); n = (i1 - i0 + 1).prod(1)
        s.o = i0.min(0); s.sh = i1.max(0) - s.o + 1
        fi = np.repeat(np.arange(len(F)), n); k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n); w = np.repeat(i1[:, 1] - i0[:, 1] + 1, n)
        key = (np.repeat(i0[:, 0], n) + k // w - s.o[0]) * s.sh[1] + (np.repeat(i0[:, 1], n) + k % w - s.o[1]); o = np.argsort(key, kind='stable'); s.key = key[o]; s.fi = fi[o]
    def ys(s, p):
        ix = np.floor(p / s.c).astype(np.int64) - s.o
        if (ix < 0).any() or (ix >= s.sh).any(): return np.zeros(0)
        kk = ix[0] * s.sh[1] + ix[1]; a = np.searchsorted(s.key, kk); b = np.searchsorted(s.key, kk, 'right')
        T = s.T[s.fi[a:b]]; A = T[:, 0, [0, 2]]; Bv = T[:, 1, [0, 2]] - A; Cv = T[:, 2, [0, 2]] - A; q = p - A
        den = Bv[:, 0] * Cv[:, 1] - Bv[:, 1] * Cv[:, 0]; nz = np.abs(den) > 1e-16; den = np.where(nz, den, 1)
        u = (q[:, 0] * Cv[:, 1] - q[:, 1] * Cv[:, 0]) / den; v = (Bv[:, 0] * q[:, 1] - Bv[:, 1] * q[:, 0]) / den; m = nz & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
        return T[m, 0, 1] + u[m] * (T[m, 1, 1] - T[m, 0, 1]) + v[m] * (T[m, 2, 1] - T[m, 0, 1])

FEAT = {'rim_flange': [(0.040, 0.112), (0.150, 0.185), (0.195, 0.226)], 'rim_flange_front': [(0.128, 0.215), (0.150, 0.185), (0.185, 0.226)],
        'pin': [(0.106, 0.131), (0.150, 0.185), (0.205, 0.245)], 'drive_housing': [(0.060, 0.180), (0.150, 0.185), (0.060, 0.165)],
        'hook': [(0.240, 0.290), (0.150, 0.185), (0.140, 0.185)]}

def main():
    a = [t for t in sys.argv[1:] if not t.startswith('--')]; UNCUT = '--uncut' in sys.argv
    d = np.load(a[0]); B = np.load(a[1]); OUT = a[2] if len(a) > 2 else None
    SKS = None if UNCUT else sk.from_npz(d)
    X = B['Chassis_v'].astype(float); CF = B['Chassis_f'].astype(np.int64)
    V = d['v'].astype(float); F = d['f'].astype(np.int64); part = d['part']; side = d['side']; lab = d['lab']; lid = lab == 4
    # dense chassis samples in the wheel-side band
    T = X[CF]; c = T.mean(1); band = (np.abs(c[:, 1]) > 0.145) & (c[:, 0] > 0.03) & (c[:, 0] < 0.30) & (c[:, 2] > 0.05) & (c[:, 2] < 0.25)
    P3 = T[band]; ar = 0.5 * np.linalg.norm(np.cross(P3[:, 1] - P3[:, 0], P3[:, 2] - P3[:, 0]), axis=1); ns = np.clip((ar / 0.16e-6).astype(int), 1, 4000)
    rng = np.random.default_rng(3); fi = np.repeat(np.arange(len(P3)), ns); u = rng.random((len(fi), 2)); fl = u.sum(1) > 1; u[fl] = 1 - u[fl]
    S = np.r_[X[np.unique(CF[band])], P3[fi, 0] + (P3[fi, 1] - P3[fi, 0]) * u[:, :1] + (P3[fi, 2] - P3[fi, 0]) * u[:, 1:]]
    out = dict(npz=a[0], chassis='uncut scan' if UNCUT else ('cut: ' + ', '.join(c_['id'] for c_ in SKS['cuts']) if SKS and SKS['mode'] == 'cut' else 'legacy kill (f2 rule)'))
    if SKS and SKS['mode'] == 'cut': S = np.r_[S[~sk.inside(S, SKS)], sk.cap_samples(X, CF, SKS)]
    elif not UNCUT:
        PO = json.load(open('mounts.json'))['posts']; kl = sk.legacy_kill(X, PO); S = S[~(cKDTree(X[kl]).query(S, distance_upper_bound=0.0012)[0] < 0.0012)] if kl.any() else S
    # solids
    SOL = {'body': SurfDist(V, F[(part == 0) & ~lid]), 'cover': SurfDist(V, F[(part == 1) & ~lid])}
    mv = d['mount_v'].astype(float); mf = d['mount_f'].astype(np.int64); mk = d['mount_kind']; MKN = [str(x) for x in d['mount_kinds']]
    for kn in MKN:
        Fm = mf[mk == MKN.index(kn)]
        if not len(Fm): continue
        e = np.concatenate([Fm[:, [0, 1]], Fm[:, [1, 2]], Fm[:, [2, 0]]]); key = np.minimum(e[:, 0], e[:, 1]) * len(mv) + np.maximum(e[:, 0], e[:, 1]); _, cnt = np.unique(key, return_counts=True)
        if (cnt == 1).sum() == 0: SOL['kit_' + kn] = (SurfDist(mv, Fm), YRay(mv, Fm))      # closed solids only; inside by ray parity (their winding is not reliable)
    Tb = V[F]; mb = (np.abs(Tb[:, :, 1]).max(1) > 0.14) & (Tb[:, :, 0].max(1) > 0.0) & (Tb[:, :, 0].min(1) < 0.32) & (Tb[:, :, 2].max(1) > 0.05) & (Tb[:, :, 2].min(1) < 0.26) & (part == 0) & ~lid
    YB = YRay(V, F[mb])
    feats = {}
    for nm, (bx, bay, bz) in FEAT.items():
        for sg, sn in ((1, 'L'), (-1, 'R')):
            m = (S[:, 0] > bx[0]) & (S[:, 0] < bx[1]) & (S[:, 1] * sg > bay[0]) & (S[:, 1] * sg < bay[1]) & (S[:, 2] > bz[0]) & (S[:, 2] < bz[1])
            Q = S[m]; r = dict(samples=int(m.sum()))
            if not len(Q): feats[nm + '_' + sn] = r; continue
            r['reach_abs_y_mm'] = round(float(np.abs(Q[:, 1]).max()) * 1000, 2); r['top_z_mm'] = round(float(Q[:, 2].max()) * 1000, 2)
            for k_, SD in SOL.items():
                if isinstance(SD, tuple):
                    s, _ = SD[0](Q, 0.02); fin = np.isfinite(s); s = np.abs(s)
                    ins = np.array([len(yy) % 2 == 1 for yy in (lambda Q_: [(lambda y_: y_[y_ > q_[1]])(SD[1].ys(q_[[0, 2]])) for q_ in Q_])(Q[fin & (s < 0.003)])], bool)
                    if fin.any(): r[k_] = dict(min_gap_mm=round(float(s[fin].min()) * 1000, 2) if not ins.any() else -1.0, inside=int(ins.sum()))
                    continue
                s, _ = SD(Q, 0.02); fin = np.isfinite(s)
                if fin.any(): r[k_] = dict(min_gap_mm=round(float(s[fin].min()) * 1000, 2), inside=int((s[fin] < 0).sum()))
            # past the wall skins (only the points the body wall covers at that x, z)
            bi = []; bo = []
            for q in Q[np.argsort(-np.abs(Q[:, 1]))[:4000]]:
                y = YB.ys(q[[0, 2]]); y = np.abs(y[y * sg > 0])
                if len(y) >= 2: y.sort(); bi.append(abs(q[1]) - y[-2]); bo.append(abs(q[1]) - y[-1])
            if bi: r['past_inner_skin_mm'] = round(max(bi) * 1000, 2); r['past_outer_skin_mm'] = round(max(bo) * 1000, 2)
            feats[nm + '_' + sn] = r
    out['features'] = feats
    # drive housing top vs the seat edge: per side, the housing crest and the lowest body / seat point above it
    hs = {}
    for sg, sn in ((1, 'L'), (-1, 'R')):
        h = (S[:, 0] > 0.06) & (S[:, 0] < 0.18) & (S[:, 1] * sg > 0.155) & (S[:, 2] > 0.12) & (S[:, 2] < 0.1555)      # housing top face (the tub wall rises above 0.1555 inboard of |y| 155)
        bw = (V[:, 1] * sg > 0.15) & (V[:, 2] < 0.20) & (V[:, 0] > 0.04) & (V[:, 0] < 0.20); bw[np.unique(F[(part != 0) | lid])] = False
        sm = mk == MKN.index('seat') if 'seat' in MKN else np.zeros(len(mf), bool); sv_ = mv[np.unique(mf[sm])]; sv_ = sv_[sv_[:, 1] * sg > 0] if len(sv_) else sv_
        U_ = np.r_[V[bw], sv_]; tr_ = cKDTree(U_[:, :2]); Hq = S[h]; gap_ = np.full(len(Hq), np.inf)
        for j_, nb_ in enumerate(tr_.query_ball_point(Hq[:, :2], 0.0008)):      # the lowest body / seat-flange point straight above each housing point (0.8 mm column)
            if nb_: zz_ = U_[nb_, 2]; zz_ = zz_[zz_ > Hq[j_, 2] - 0.0005]; gap_[j_] = (zz_.min() - Hq[j_, 2]) if len(zz_) else np.inf
        jm_ = int(np.argmin(gap_))
        hs[sn] = dict(housing_crest_z_mm=round(float(Hq[:, 2].max()) * 1000, 2), body_edge_z_over_housing_mm=round(float(V[bw & (V[:, 0] > 0.078) & (V[:, 0] < 0.160) & (V[:, 1] * sg > 0.1634)][:, 2].min()) * 1000, 2),
                      seat_flange_min_z_mm=round(float(sv_[:, 2].min()) * 1000, 2) if len(sv_) else None,
                      vertical_gap_min_mm=round(float(gap_[jm_]) * 1000, 2), at_mm=(Hq[jm_] * 1000).round(1).tolist())
    out['drive_housing_vs_seat'] = hs
    # wheels: body outer skin -> tyre
    Vo = V[np.unique(F[(part == 0) & (side == 0) & ~lid])]; wr = {}
    for w, sg in (('Wheel_L', 1), ('Wheel_R', -1)):
        W = B[w + '_v']; q = Vo[(Vo[:, 1] * sg > 0.15) & (Vo[:, 0] > -0.02) & (Vo[:, 0] < 0.26)]; dd, _ = cKDTree(W).query(q); wr[w] = round(float(dd.min()) * 1000, 2)
    out['body_to_wheel_min_mm'] = wr
    if SKS and SKS['mode'] == 'cut':
        cr = {}
        for c_, G, rp in sk.cap_polys(X, CF, SKS):
            rm = sk.inside(X, dict(cuts=[c_])); dep = (X[rm] - np.array(c_['p'])) @ np.array(c_['n'])
            cr[c_['id']] = dict(cap_area_mm2=rp['area_mm2'], removed_vertices=int(rm.sum()), removal_depth_mm=round(float(dep.max()) * 1000, 2) if rm.any() else 0.0, cap_bounds_mm=rp['bounds_mm'])
        out['cuts'] = cr; out['air_check'] = sk.air_check(X, CF, SKS)
        out['legacy_kill_vertices'] = int(sk.legacy_kill(X, json.load(open('mounts.json'))['posts']).sum()); out['removed_vertices_total'] = int(sk.inside(X, SKS).sum())
    print(json.dumps(out, indent=1))
    if OUT: json.dump(out, open(OUT, 'w'), indent=1)

if __name__ == '__main__': main()
