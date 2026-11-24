# [fix3-hinge] FEA of the lid hinge open stop (linear elastic, quadratic tetrahedra; gmsh OCC geometry + scikit-fem).
# usage: PYTHONPATH=../pylib python hingefea.py PARAMS.json DESIGN.npz CASE [F_edge_N] [--out JSON]
#   CASE tongue : the moving member (tongue + stop lug(s) [+ root blade]) at the open stop. Units mm / N / MPa, frame = tongue frame
#                 (closed pose, origin Q, X = x - Qx, Y = z - Qz, Z = y - y_hinge). Supports: stop face on the lug(s) (rigid in the
#                 tangential direction), POM rail liners as unilateral Winkler springs with the 0.2 mm running clearance (active-set
#                 iteration) over the guide engagement at the open angle. Load: the edge force F (perpendicular, opening sense) of a
#                 corner load carried by ONE hinge: f3 = bearing couple of skin / keel on the blade (resultant F, moment F * L_edge about Q);
#                 f2 = tongue root clamped in the boss (the f2 root has no lever - see hingecalc) and the stop force M / R on the pin.
#   CASE cheek  : the cheek with the stop slot (f2: POM, pin bears 1.1 of 1.3 mm; f3: 1.4310 plate, lug bears 1.2 mm) under the lug force;
#                 held at the mouth end (tangential), on its outer arc (radial) and on its outboard face (axial) by the pod.
import sys, json, numpy as np, shapely, gmsh
from shapely.geometry import Polygon, Point
sys.path.insert(0, '.')
import lidmech as LM
from skfem import MeshTet, Basis, FacetBasis, ElementVector, ElementTetP2, ElementTetP1, BilinearForm, LinearForm, asm, solve, condense
from skfem.helpers import dot, sym_grad, ddot, trace, eye
from skfem.models.elasticity import linear_elasticity, lame_parameters
import time, pyamg, scipy.sparse.linalg as spla
T0 = time.time()
RBM = [None]
def rbm(ib):
    """rigid-body modes of the P2 vector basis (near-null space for smoothed-aggregation AMG)"""
    N = ib.N; comp = np.zeros(N, int); X = ib.doflocs
    for k in range(3):
        comp[ib.nodal_dofs[k]] = k
        if hasattr(ib, 'edge_dofs') and ib.edge_dofs.size: comp[ib.edge_dofs[k]] = k
    B = np.zeros((N, 6)); x, y, z = X
    for k in range(3): B[comp == k, k] = 1.0
    for j, (a_, b_) in enumerate(((0, 1), (1, 2), (2, 0))):          # rotations: u_a = -x_b, u_b = x_a
        B[comp == a_, 3 + j] = -X[b_][comp == a_]; B[comp == b_, 3 + j] = X[a_][comp == b_]
    RBM[0] = B; return B
def solve(A, b, x0=None, I=None):
    """AMG (smoothed aggregation with rigid-body modes) preconditioned CG; replaces the direct solver."""
    A = A.tocsr(); B = RBM[0] if I is None else RBM[0][I]
    ml = pyamg.smoothed_aggregation_solver(A, B=B, max_coarse=3000, symmetry='symmetric')
    it = [0]; x, info = spla.cg(A, b, x0=x0, rtol=1e-9, maxiter=4000, M=ml.aspreconditioner(), callback=lambda xk: it.__setitem__(0, it[0] + 1))
    print('cg', it[0], 'it', 'info', info, round(time.time() - T0, 1), 's', file=sys.stderr)
    return x

a = sys.argv[1:]; OUT = None
if '--out' in a: i = a.index('--out'); OUT = a[i + 1]; del a[i:i + 2]
PJ, NPZ, CASE = a[0], a[1], a[2]; FE = float(a[3]) if len(a) > 3 else 20.0
P = json.load(open(PJ)); L_ = P['lid']; p = LM.params({k: v for k, v in L_.items() if k not in ('shape', 'cx', 'length', 'width', 'struts', 'roll', 'fine')})
H3 = bool(p.get('h3')); C3 = LM.hingekit.cfg(p) if H3 else None
R, t, w = p['arc_r'] * 1e3, p['arc_t'] * 1e3, p['arc_w'] * 1e3; a0, a1 = p['arc_a']; g0, g1 = p['guide_a']; OPEN = p['open_deg']
r_in, r_out = R - t / 2, R + t / 2; ca, wch = p['clr_a'] * 1e3, p['guide_cheek'] * 1e3; c = p['clr_r'] * 1e3
lr = p['lug_d'] * 1e3 / 2; lug_a = p['lug_a']
d = np.load(NPZ); Q = np.array(d['lid_pivot']) * 1e3; YC = float(d['lid_hinge_y'][0]) * 1e3
lp = d['lid_poly'] * 1e3; L_EDGE = float(lp[:, 0].max() - Q[0])                      # lever of the free edge about Q (mm)
E_ST, NU_ST = (196e3, 0.27) if H3 else (190e3, 0.28)
dirv = lambda q: np.array([np.cos(np.radians(q)), np.sin(np.radians(q))])
sector = lambda r0, r1, q0, q1, n=None: Polygon(np.r_[[[r1 * np.cos(x), r1 * np.sin(x)] for x in np.radians(np.linspace(q0, q1, n or max(3, int(abs(q1 - q0) / 1.0) + 1)))],
                                                    [[r0 * np.cos(x), r0 * np.sin(x)] for x in np.radians(np.linspace(q1, q0, n or max(3, int(abs(q1 - q0) / 1.0) + 1)))]])
gmsh.initialize(); gmsh.option.setNumber('General.Terminal', 0); occ = gmsh.model.occ

def surf(pg):
    pg = shapely.geometry.polygon.orient(pg.simplify(0.002), 1.0); loops = []
    for ring in [pg.exterior] + list(pg.interiors):
        C = np.asarray(ring.coords)[:-1]; pts = [occ.addPoint(x, y, 0) for x, y in C]
        loops.append(occ.addCurveLoop([occ.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]))
    return occ.addPlaneSurface(loops)

def prism(pg, z0, z1):
    s = surf(pg); e = occ.extrude([(2, s)], 0, 0, z1 - z0); v = [x for x in e if x[0] == 3][0]
    occ.translate([v], 0, 0, z0); return v

def mesh(h, refine):
    occ.synchronize(); gmsh.option.setNumber('Mesh.MeshSizeMax', h); gmsh.option.setNumber('Mesh.MeshSizeMin', h / 8)
    if refine:
        fs = []
        for (cx, cy, cz, r_, hr) in refine:
            f_ = gmsh.model.mesh.field.add('Ball'); gmsh.model.mesh.field.setNumber(f_, 'XCenter', cx); gmsh.model.mesh.field.setNumber(f_, 'YCenter', cy)
            gmsh.model.mesh.field.setNumber(f_, 'ZCenter', cz); gmsh.model.mesh.field.setNumber(f_, 'Radius', r_); gmsh.model.mesh.field.setNumber(f_, 'VIn', hr)
            gmsh.model.mesh.field.setNumber(f_, 'VOut', h); gmsh.model.mesh.field.setNumber(f_, 'Thickness', 2 * r_); fs.append(f_)
        mf = gmsh.model.mesh.field.add('Min'); gmsh.model.mesh.field.setNumbers(mf, 'FieldsList', fs); gmsh.model.mesh.field.setAsBackgroundMesh(mf)
        gmsh.option.setNumber('Mesh.MeshSizeExtendFromBoundary', 0); gmsh.option.setNumber('Mesh.MeshSizeFromPoints', 0)
    gmsh.option.setNumber('Mesh.Algorithm3D', 1); gmsh.model.mesh.generate(3); print('mesh', gmsh.model.mesh.getNodes()[0].size, 'nodes', round(time.time() - T0, 1), 's', file=sys.stderr)
    tags, xyz, _ = gmsh.model.mesh.getNodes(); xyz = xyz.reshape(-1, 3); et, etags, en = gmsh.model.mesh.getElements(3)
    T = np.asarray(en[list(et).index(4)]).reshape(-1, 4); mp = np.zeros(int(tags.max()) + 1, np.int64); mp[tags] = np.arange(len(tags))
    T = mp[T]; used = np.unique(T); rm = -np.ones(len(xyz), np.int64); rm[used] = np.arange(len(used))
    return MeshTet(np.ascontiguousarray(xyz[used].T), np.ascontiguousarray(rm[T].T))

def bfacets(m):
    """boundary facets: index, centroid (k,3), outward unit normal (k,3), area (k,)"""
    bf = m.boundary_facets(); F = m.facets[:, bf]; X = m.p[:, F]; n = np.cross(X[:, 1] - X[:, 0], X[:, 2] - X[:, 0], axis=0)
    t_ = m.f2t[0, bf]; dv = m.p[:, m.t[:, t_]].mean(1) - X.mean(1); s = np.sign((n * dv).sum(0)); n = -n * s
    A = np.linalg.norm(n, axis=0) / 2; n = n / np.maximum(2 * A, 1e-30); return bf, X.mean(1).T, n.T, A

def vonmises(m, ib, u, E, nu):
    lam, mu = lame_parameters(E, nu); du = ib.interpolate(u).grad; eps = 0.5 * (du + du.transpose(1, 0, 2, 3))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]; S = 2 * mu * eps + lam * tr * np.eye(3)[:, :, None, None]
    vm = np.sqrt(0.5 * ((S[0, 0] - S[1, 1]) ** 2 + (S[1, 1] - S[2, 2]) ** 2 + (S[2, 2] - S[0, 0]) ** 2) + 3 * (S[0, 1] ** 2 + S[1, 2] ** 2 + S[0, 2] ** 2))
    s1 = np.linalg.eigvalsh(np.moveaxis(S, (0, 1), (-2, -1)))                         # principal stresses at quadrature points
    sb = Basis(m, ElementTetP1(), intorder=4)
    vmn = sb.project(vm); return vm, vmn, s1

def solve_contact(m, ib, K0, f0, sets, stop_sets, iters=25):
    """sets: list of (facet idx array, k [N/mm^3], gap [mm]) unilateral Winkler springs on u.n (n = outward facet normal);
    stop_sets: list of (facet idx, k, direction (3,)) bilateral penalty on u.d. Active-set iteration on the unilateral springs."""
    def pen_n(k):
        @BilinearForm
        def f_(u, v, w): return k * dot(u, w.n) * dot(v, w.n)
        return f_
    def gapload(k, g):
        @LinearForm
        def f_(v, w): return k * g * dot(v, w.n)
        return f_
    def pen_d(k, dd):
        dd = np.asarray(dd, float)[:, None, None]
        @BilinearForm
        def f_(u, v, w): return k * dot(u, dd) * dot(v, dd)
        return f_
    Ks = K0.copy()
    for idx, k, dd in stop_sets: Ks = Ks + asm(pen_d(k, dd), FacetBasis(m, ib.elem, facets=idx))
    act = [np.ones(len(s[0]), bool) for s in sets]; u = None
    for it in range(iters):
        Kt = Ks.copy(); ft = f0.copy()
        for (idx, k, g), ac in zip(sets, act):
            if ac.any():
                fb = FacetBasis(m, ib.elem, facets=idx[ac]); Kt = Kt + asm(pen_n(k), fb); ft = ft + asm(gapload(k, g), fb)
        u = solve(Kt, ft); changed = 0; new = []; print('contact it', it, round(time.time() - T0, 1), file=sys.stderr)
        for (idx, k, g), ac in zip(sets, act):
            X = m.p[:, m.facets[:, idx]]; un = np.zeros(len(idx))
            for j in range(3): un += np.einsum('ik,ik->k', u[ib.nodal_dofs[:, m.facets[j, idx]]], FN[idx].T) / 3
            a_ = un - g > 0; changed += int((a_ != ac).sum()); new.append(a_)
        act = new
        if changed == 0: break
    pens = []
    for (idx, k, g), ac in zip(sets, act):
        un = np.zeros(len(idx))
        for j in range(3): un += np.einsum('ik,ik->k', u[ib.nodal_dofs[:, m.facets[j, idx]]], FN[idx].T) / 3
        pens.append((un - g) * ac)
    return u, act, pens, it + 1

res = {'case': CASE, 'params': PJ, 'F_edge_N': FE, 'L_edge_mm': round(L_EDGE, 1), 'design': ('f4' if p.get('h4') else 'f3') if H3 else 'f2'}      # [fix4-hinge2] f4
M_edge = FE * L_EDGE                                                                  # N.mm about Q, one hinge (corner load)

if CASE == 'tongue':
    pg = sector(r_in, r_out, a0, a1).union(Point(*(R * dirv(a0))).buffer(t / 2, 24))
    if H3:                                                                            # root blade from the generated mesh (slice at the hinge plane)
        from slicekit import slice_y
        km = d['kitm_part'] == LM.PM['blade']; sg_ = d['kitm_v'][np.unique(d['kitm_f'][km])][:, 1].mean() > 0
        seg = slice_y(d['kitm_v'], d['kitm_f'][km], (YC if sg_ else -YC) / 1e3) * 1e3 - Q
        BL = max(shapely.get_parts(shapely.polygonize(shapely.linestrings(seg))), key=lambda g_: g_.area).buffer(0)
        pg = pg.union(BL.buffer(0.01)).buffer(0)
    else: BL = None
    V = prism(pg, -w / 2, w / 2)[1]
    H4F = bool(p.get('h4'))      # [fix4-hinge2] geared tongue: the sector gear in the middle band - material above the tooth ROOT circle removed over the
    if H4F:                      # toothed zone (the teeth themselves carry no bending: conservative; tooth-root notch factor in hingecalc)
        G4_ = LM.hingekit4.cfg4(p)['gear']; rf4_ = (G4_['z_w'] * G4_['m'] / 2 - 1.25 * G4_['m']) * 1e3
        zc4_ = prism(LM.hingekit4.zone_cut((0.0, 0.0), rf4_, r_out, p['exit_a'] + G4_['zone'][0], p['exit_a'] + G4_['zone'][1], G4_.get('runout', 5.5), 1.0, 0.5), -G4_['band'] * 1e3 / 2, G4_['band'] * 1e3 / 2)      # with the run-outs
        cut4_, _ = occ.cut([(3, V)], [zc4_]); V = cut4_[0][1]
    lc = R * dirv(lug_a); z_tip = w / 2 + ca + wch * (C3['lug_eng'] if H3 else 0.85)
    sides = (1, -1) if (H3 and C3['lug_both']) else (1,)
    fr = C3['lug_fillet'] * 1e3 if H3 else 0.0                                         # f3: R0.25 root fillet (lug = solid of revolution)
    cyl = []
    for s in sides:
        q_ = np.linspace(0, np.pi / 2, 7); prof = [(0.0, w / 2 - 0.2), (lr + fr, w / 2 - 0.2)] + ([(lr + fr * (1 - np.sin(x)), w / 2 + fr * (1 - np.cos(x))) for x in q_] if fr > 0 else []) + [(lr, z_tip), (0.0, z_tip)]
        pts = [occ.addPoint(lc[0] + rr_, lc[1], s * zz_) for rr_, zz_ in prof if not (fr == 0 and False)]
        ln_ = [occ.addLine(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]
        sf_ = occ.addPlaneSurface([occ.addCurveLoop(ln_)]); rv = occ.revolve([(2, sf_)], lc[0], lc[1], 0, 0, 0, 1, 2 * np.pi)
        cyl += [x for x in rv if x[0] == 3]
    out, _ = occ.fuse([(3, V)], cyl); occ.synchronize(); vol = out[0][1]
    ref = [(lc[0], lc[1], s * (w / 2 + 0.5), 1.5 * lr, 0.16) for s in sides]
    mouth = R * dirv(g1 - OPEN); ref.append((mouth[0], mouth[1], 0.0, 4.0, 0.45))
    if H3: ref.append((18.5, -5.0, 0.0, 5.0, 0.45))
    m = mesh(0.9, ref)
    e = ElementVector(ElementTetP2()); ib = Basis(m, e, intorder=4); rbm(ib); K0 = asm(linear_elasticity(*lame_parameters(E_ST, NU_ST)), ib)
    bf, XC, FNb, AR = bfacets(m); FN = np.zeros((m.facets.shape[1], 3)); FN[bf] = FNb
    rr = np.hypot(XC[:, 0], XC[:, 1]); ph = np.degrees(np.arctan2(XC[:, 1], XC[:, 0])); rh = np.c_[np.cos(np.radians(ph)), np.sin(np.radians(ph)), 0 * ph]
    nr = (FNb * rh).sum(1)
    q_m = g1 - OPEN                                                                   # tongue angle at the pod mouth at the open stop (-97)
    eng = (ph < q_m) & (ph > a0 - 12)
    tape_slot = (ph > p['exit_a'] + (C3['rail_split'][0] if H3 else -2) - OPEN) & (ph < p['exit_a'] + (C3['rail_split'][1] if H3 else 12) - OPEN) & (np.abs(XC[:, 2]) < (G4_['band'] if H4F else p['band_w']) * 1e3 / 2 + 0.3)      # [fix4-hinge2] h4: the pinion slot
    # rail contact patches from the rigid-body statics of the tongue at the stop (the stop force -theta near the tail is balanced by an
    # outward push of the inner rail at the pod mouth and an inward push of the outer rail at the tail): bilateral Winkler springs there
    inner = bf[eng & (np.abs(rr - r_in) < 0.08) & (nr < -0.9) & (ph > q_m - 15.0)]
    outer = bf[(ph < a0 + 22.0) & (ph > a0 - 12) & (np.abs(rr - r_out) < 0.08) & (nr > 0.9) & ~tape_slot]
    k_r = 1500.0                                                                        # POM liner 1.3 mm, E(60 C) ~ 2.0 GPa, backed: k = E / t
    th_l = np.array([-np.sin(np.radians(lug_a + 0)), np.cos(np.radians(lug_a)), 0.0])  # +theta at the lug (tongue frame)
    dl = XC[:, :2] - lc; lugf = (np.hypot(dl[:, 0], dl[:, 1]) < lr + 0.05) & (np.abs(XC[:, 2]) > w / 2 + ca - 0.01) & ((FNb[:, :2] @ th_l[:2]) > 0.35)
    axial = bf[(np.abs(np.abs(XC[:, 2]) - w / 2) < 0.01) & eng]                         # side faces in the guide: weak axial support (running fit)
    F0 = np.zeros(ib.N)
    if H3:
        # lid -> blade: skin bearing (down, -Y) on the root top, keel floor (up, +Y) under the deep part; resultant +Y F, moment F * L_edge
        xs_ = XC[:, 0]; top = bf[(FNb[:, 1] > 0.6) & (xs_ > 13.0) & (xs_ < 19.5) & (XC[:, 1] > -5.5)]
        bot = bf[(FNb[:, 1] < -0.6) & (XC[:, 1] < -7.0) & (xs_ > 22.0)]
        A1, A2 = AR[np.isin(bf, top)].sum(), AR[np.isin(bf, bot)].sum()
        x1, x2 = (XC[np.isin(bf, top), 0] * AR[np.isin(bf, top)]).sum() / A1, (XC[np.isin(bf, bot), 0] * AR[np.isin(bf, bot)]).sum() / A2
        P1 = (M_edge - x2 * FE) / (x2 - x1); P2 = P1 + FE                               # P2 - P1 = F, x2 P2 - x1 P1 = M
        @LinearForm
        def l_top(v, w_): return -P1 / A1 * v[1]
        @LinearForm
        def l_bot(v, w_): return P2 / A2 * v[1]
        F0 = asm(l_top, FacetBasis(m, e, facets=top)) + asm(l_bot, FacetBasis(m, e, facets=bot))
        stop = [(bf[lugf], 2.0e5, th_l), (axial, 0.5, [0.0, 0.0, 1.0])]
        res['load'] = dict(M_Nmm=round(M_edge), skin_bearing_N=round(P1, 1), keel_bearing_N=round(P2, 1), x_bearing_mm=[round(x1, 1), round(x2, 1)],
                           p_skin_MPa=round(P1 / A1, 2), p_keel_MPa=round(P2 / A2, 2))
        u, act, pens, nit = solve_contact(m, ib, K0, F0, [(inner, k_r, 0.0), (outer, k_r, 0.0)], stop, iters=1)
    else:
        # f2: root clamped in the boss (tongue angles web_a .. a1), stop force M / R on the pin's contact half
        Fs = M_edge / R; area = AR[lugf].sum()
        @LinearForm
        def l_lug(v, w_): return -Fs / area * dot(v, th_l[:, None, None])
        F0 = asm(l_lug, FacetBasis(m, e, facets=bf[lugf]))
        rootf = bf[(ph > p['web_a']) & (ph < a1 + 1)]
        dofs = ib.get_dofs(facets=rootf).all()
        res['load'] = dict(M_Nmm=round(M_edge), stop_force_N=round(Fs, 1), pin_contact_mm2=round(area, 2))
        _solve = solve                                                                  # contact iteration with the clamped root dofs condensed out
        def solve(Kt, ft):
            A_, b_, x_, I_ = condense(Kt, ft, D=dofs); x_ = x_.copy(); x_[I_] = _solve(A_, b_, I=I_); return x_
        u, act, pens, nit = solve_contact(m, ib, K0, F0, [(inner, k_r, 0.0), (outer, k_r, 0.0)], [(axial, 0.5, [0.0, 0.0, 1.0])], iters=1)
    np.savez_compressed('fea/sol_%s_tongue.npz' % res['design'], u=u, p=m.p, t=m.t)
    vm, vmn, s1 = vonmises(m, ib, u, E_ST, NU_ST)
    # regions (element centroids)
    TC = m.p[:, m.t].mean(1).T; phc = np.degrees(np.arctan2(TC[:, 1], TC[:, 0])); rc = np.hypot(TC[:, 0], TC[:, 1])
    vme = vm.max(1)
    reg = {'lug_root': (np.hypot(TC[:, 0] - lc[0], TC[:, 1] - lc[1]) < lr + 1.0) & (np.abs(TC[:, 2]) > w / 2 - 1.0),
           'tongue_free_span (mouth..root)': (phc > q_m - 3) & (phc < (p['web_a'] if True else a1)) & (rc < r_out + 0.3) & (np.abs(TC[:, 2]) <= w / 2 + 0.01),
           'tongue_in_guide': (phc <= q_m - 3) & (np.abs(TC[:, 2]) <= w / 2 + 0.01) & ~((np.hypot(TC[:, 0] - lc[0], TC[:, 1] - lc[1]) < lr + 1.0))}
    if H3: reg['blade_root (x 14..22)'] = (TC[:, 0] > 14) & (TC[:, 0] < 22.5) & ((phc > p['web_a'] - 1) | (rc > r_out + 0.3))
    if H3: reg['blade_deep (x > 22.5)'] = (TC[:, 0] >= 22.5)
    if H4F:      # [fix4-hinge2] the band over the full-depth teeth (root circle, teeth not modelled) and over the two run-outs (shallow partial tooth spaces)
        zb_ = (np.abs(TC[:, 2]) < G4_['band'] * 1e3 / 2 + 0.3) & (rc > rf4_ - 1.2); zl_, zh_ = p['exit_a'] + G4_['zone'][0], p['exit_a'] + G4_['zone'][1]; ru_ = G4_.get('runout', 5.5)
        reg['toothed_band_full_depth (root circle, teeth not modelled)'] = zb_ & (phc > zl_ + ru_) & (phc < zh_ - ru_)
        reg['toothed_band_runouts'] = zb_ & (((phc >= zl_ - 0.5) & (phc <= zl_ + ru_)) | ((phc >= zh_ - ru_) & (phc <= zh_ + 0.5)))
    # node-projected peaks too
    vmn_max = {}
    PN = m.p.T
    for k_, msk in reg.items():
        if msk.any():
            nodes = np.unique(m.t[:, msk]); j = nodes[np.argmax(vmn[nodes])]
            vmn_max[k_] = dict(vm_qp_max_MPa=round(float(vme[msk].max()), 1), vm_node_max_MPa=round(float(vmn[j]), 1), at_mm=np.round(PN[j], 2).tolist())
    # reactions
    rail = []
    for nm_, idx in (('inner (mouth)', inner), ('outer (tail)', outer)):
        un = np.zeros(len(idx))
        for j in range(3): un += np.einsum('ik,ik->k', u[ib.nodal_dofs[:, m.facets[j, idx]]], FN[idx].T) / 3
        a_ = AR[np.isin(bf, idx)]; pr = k_r * un; Fr = (pr * a_).sum(); ac = un > 0
        jj = np.argmax(pr); rail.append(dict(rail=nm_, force_N=round(float(Fr), 1), p_max_MPa=round(float(pr.max()), 2), at_deg=round(float(ph[np.isin(bf, idx)][jj]), 1) if len(pr) else None,
                                             active_facets=int(ac.sum())))
    res.update(mesh=dict(nodes=int(m.p.shape[1]), tets=int(m.t.shape[1]), dofs=int(ib.N)), contact_iterations=nit, rails=rail, regions=vmn_max,
               vm_max_MPa=round(float(vmn.max()), 1), vm_max_at_mm=np.round(PN[int(np.argmax(vmn))], 2).tolist(),
               max_disp_mm=round(float(np.abs(u).max()), 4))
    if H3:
        # stop reaction from the penalty
        un = np.zeros(int(lugf.sum()))
        for j in range(3): un += np.einsum('ik,i->k', u[ib.nodal_dofs[:, m.facets[j, bf[lugf]]]], th_l) / 3
        res['stop_force_N'] = round(float(-(2.0e5 * un * AR[lugf]).sum()), 1); res['stop_force_expected_N'] = round(M_edge / R, 1)

elif CASE == 'cheek':
    ri0, ro1 = r_in - c - p['guide_wall'] * 1e3, r_out + (c + p['band_t'] * 1e3 + 0.1) + p['guide_wall'] * 1e3
    lclr = p['lug_clr'] * 1e3; hl = np.degrees(lr / R); as0 = lug_a - p['over_deg'] - hl; as1 = lug_a + OPEN + hl + np.degrees(p['bumper'] * 1e3 / R)
    g1r = g1 - 0.6
    slot = sector(R - lr - lclr, R + lr + lclr, as0, as1)
    if H3: slot = LM.hingekit.slot3(np.zeros(2), R, lr + lclr, as0, as1, lambda Q_, a_, b_, c_, d_: sector(a_, b_, c_, d_))      # conformal ends
    pg = sector(ri0, ro1, g0, g1r).difference(slot)
    tch = wch; V = prism(pg, 0.0, tch)
    eng_t = wch * (C3['lug_eng'] if H3 else 0.85) - (0.0 if H3 else 0.0)                 # lug engagement depth into the cheek (from the tongue side, z = 0)
    if not H3: eng_t = wch * 0.85                                                       # f2: pin reaches 1.105 of 1.3 mm
    se = R * dirv(as1); m = mesh(0.5, [(se[0], se[1], tch / 2, 1.6, 0.14)])
    E_, NU_ = (193e3, 0.29) if H3 else (2000.0, 0.35)                                   # 1.4310 / POM-C at 60 C (E 2.0 GPa)
    e = ElementVector(ElementTetP2()); ib = Basis(m, e, intorder=4); rbm(ib); K0 = asm(linear_elasticity(*lame_parameters(E_, NU_)), ib)
    bf, XC, FNb, AR = bfacets(m); FN = np.zeros((m.facets.shape[1], 3)); FN[bf] = FNb
    rr = np.hypot(XC[:, 0], XC[:, 1]); ph = np.degrees(np.arctan2(XC[:, 1], XC[:, 0]))
    tg = np.tile([-np.sin(np.radians(as1)), np.cos(np.radians(as1)), 0.0], (len(ph), 1))
    nsl = (FNb * tg).sum(1)                                                             # slot end face: outward normal = -theta (into the slot)
    nlug = 2 if (H3 and C3['lug_both']) else 1; Fl = M_edge / R / nlug                   # one cheek takes one lug
    # contact strip on the slot end: line contact of the lug through the crushed TPE pad, width 2 b (Hertz half width for steel-steel /
    # spread by the pad) ~ 0.6 * lug radius, over the engaged depth
    bw_ = 0.6 * lr
    load = bf[(np.abs(ph - as1) < 0.6) & (nsl < -0.9) & (np.abs(rr - R) < bw_) & (XC[:, 2] < eng_t)]
    if H3:                                                                              # conformal semicircular end: the lug bears on its front +-40 deg
        rs_ = lr + lclr; Ce = R * dirv(as1 - np.degrees(rs_ / R)); dX = XC[:, :2] - Ce; dn = np.hypot(dX[:, 0], dX[:, 1])
        inward = ((FNb[:, :2] * (-dX)).sum(1) / np.maximum(dn, 1e-9)) > 0.9
        load = bf[(np.abs(dn - rs_) < 0.06) & inward & ((dX @ tg[0, :2]) > rs_ * np.cos(np.radians(40))) & (XC[:, 2] < eng_t)]
    A = AR[np.isin(bf, load)].sum()
    @LinearForm
    def l_end(v, w_): return Fl / A * (v[0] * tg[0, 0] + v[1] * tg[0, 1])
    F0 = asm(l_end, FacetBasis(m, e, facets=load))
    tgm = np.array([-np.sin(np.radians(g1r)), np.cos(np.radians(g1r)), 0.0]); mouth = bf[(ph > g1r - 0.3) & (np.abs(FNb @ tgm) > 0.9)]
    outer = bf[(np.abs(rr - ro1) < 0.05) & (ph > as1 - 40)]
    side = bf[(np.abs(XC[:, 2] - tch) < 0.01) & (ph > as1 - 40)]
    @BilinearForm
    def pen_t(u, v, w_): return 1e5 * dot(u, w_.n) * dot(v, w_.n)
    Kt = K0 + asm(pen_t, FacetBasis(m, e, facets=mouth)) + asm(pen_t, FacetBasis(m, e, facets=outer)) + asm(pen_t, FacetBasis(m, e, facets=side))
    u = solve(Kt, F0)
    vm, vmn, s1 = vonmises(m, ib, u, E_, NU_)
    PN = m.p.T; j = int(np.argmax(vmn)); smin = float(s1[..., 0].min())
    res.update(material='1.4310 plate' if H3 else 'POM-C (60 C)', lug_force_N=round(Fl, 1), contact_area_mm2=round(A, 2), p_nominal_MPa=round(Fl / A, 1),
               bearing_nominal_MPa=round(Fl / (2 * lr * eng_t), 1), engaged_mm=round(eng_t, 3),
               vm_max_MPa=round(float(vmn[j]), 1), vm_max_at_mm=np.round(PN[j], 2).tolist(), min_principal_MPa=round(smin, 1),
               mesh=dict(nodes=int(m.p.shape[1]), tets=int(m.t.shape[1])))
print(json.dumps(res, indent=1))
if OUT: json.dump(res, open(OUT, 'w'), indent=1)
gmsh.finalize()
