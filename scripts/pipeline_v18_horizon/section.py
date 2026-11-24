# Section of a walled design: cut every generated mesh exactly by a plane, keep one side, and close the cut faces of the shell and the cover
# with flat caps (the plastic cross-section), so the renderer can show the real wall thickness.
# usage: python section.py SHELL.npz OUT.npz AXIS VALUE [+|-]      e.g.  y 0.02 +   keeps y > 0.02 (look from -y)
# Render the result with the same plane on the imported product meshes:  CLIP=y:0.02:+ blender ... design_render.py -- P.json OUT.npz ...
# Adds cap_v / cap_f / cap_part (0 body, 1 cover) to the npz.
import numpy as np, sys, shapely
from shapely.geometry import Polygon
sys.path.insert(0, '.'); import shell_wall as sw
d = dict(np.load(sys.argv[1])); OUT = sys.argv[2]; ax = 'xyz'.index(sys.argv[3]); v0 = float(sys.argv[4]); sg = -1.0 if len(sys.argv) > 5 and sys.argv[5] == '-' else 1.0
uv = [i for i in range(3) if i != ax]


def cut(V, F, facs):
    h = (V[:, ax] - v0) * sg
    V2, F2, f2, _, h2 = sw.split(V, F, h, facs + [np.arange(len(F))], {}, iso=0.0, eps=1e-10)
    k = h2[F2].mean(1) > 0; return V2, F2[k], [x[k] for x in f2], h2


V = d['v'].astype(float); F = d['f'].astype(np.int64); side = d.get('side', np.zeros(len(F), np.uint8))
LP_ = 'lid_paint' in d and len(d['lid_paint']) == len(F)      # [fix2] the lid paint (per face) follows the cut too
V2, F2, fk_, h2 = cut(V, F, [d['lab'], d['part'], side] + ([d['lid_paint']] if LP_ else []))
lab, part, side2, orig = fk_[0], fk_[1], fk_[2], fk_[-1]
if LP_: d['lid_paint'] = fk_[3].astype(np.uint8)
same = (F2 == F[orig]).all(1); P = V2[F2]; fn = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); fn /= np.maximum(np.linalg.norm(fn, axis=1, keepdims=True), 1e-30)
ln = np.where(same[:, None, None], d['ln'][orig], np.repeat(fn[:, None, :], 3, 1)).astype(np.float32)
used = np.zeros(len(V2), bool); used[F2.ravel()] = True; rm = -np.ones(len(V2), np.int64); rm[used] = np.arange(used.sum())
d.update(v=V2[used], f=rm[F2].astype(np.int32), lab=lab.astype(np.int8), part=part.astype(np.uint8), side=side2.astype(np.uint8), ln=ln)
# caps: the cut loops of each closed moulding lie in the plane; the plastic is the even-odd interior of those loops
CV, CF, CP = [], [], []; off = 0
for pa in (0, 1):
    m = (part == pa) & (lab != 4)
    if not m.any(): continue
    region = None
    for lp, _ in sw.boundary_loops(F2[m], len(V2)):
        if len(lp) < 3 or np.abs(h2[lp]).max() > 1e-7: continue
        pg = Polygon(V2[lp][:, uv]).buffer(0)
        if pg.is_empty: continue
        region = pg if region is None else region.symmetric_difference(pg)
    if region is None: continue
    for poly in getattr(region, 'geoms', [region]):
        if poly.geom_type != 'Polygon' or poly.area < 1e-9: continue
        tri = shapely.constrained_delaunay_triangles(poly)
        for t_ in tri.geoms:
            c = np.array(t_.exterior.coords)[:3]; X = np.zeros((3, 3)); X[:, uv] = c; X[:, ax] = v0 + sg * 2e-6
            n = np.cross(X[1] - X[0], X[2] - X[0])
            if n[ax] * sg > 0: X = X[::-1]                                     # cap faces the removed side (the camera)
            CV.append(X); CF.append(np.arange(3) + off); CP.append(pa); off += 3
d.update(cap_v=np.concatenate(CV) if CV else np.zeros((0, 3)), cap_f=np.array(CF).reshape(-1, 3).astype(np.int32), cap_part=np.array(CP, np.int8))
print('caps', len(CF), 'triangles', {pa: int((np.array(CP) == pa).sum()) for pa in (0, 1)})
# the other generated meshes: cut, no caps
for key, labk in (('mount', 'mount_lab'), ('ckit', 'ckit_lab'), ('kitf', 'kitf_lab'), ('kitm', 'kitm_lab'), ('cam', 'cam_lab'), ('sleeve', None), ('plinth', None), ('stop', 'stop_lab')):      # [fix3-stop] stop: STOP key parts
    if key + '_v' not in d or not len(d[key + '_f']): continue
    Vk = d[key + '_v'].astype(float); Fk = d[key + '_f'].astype(np.int64)
    fkeys = [k_ for k_ in ([labk] if labk else []) + [key + '_kind', key + '_part'] if k_ in d and len(d[k_]) == len(Fk)]      # [v18c] every per-face array follows the cut
    Vk2, Fk2, fk2, _ = cut(Vk, Fk, [d[k_] for k_ in fkeys])
    d[key + '_v'] = Vk2; d[key + '_f'] = Fk2
    for k_, a_ in zip(fkeys, fk2): d[k_] = a_
    if key == 'mount' and 'mount_kind' in d and '--cap-deck' in sys.argv:      # [fix3-labyrinth] the deck is a closed solid of the body: cap its cut too
        dk_ = [str(x) for x in d['mount_kinds']].index('deck'); Fdk = Fk2[d['mount_kind'] == dk_]; hk_ = (Vk2[:, ax] - v0) * sg; region = None      # [fix3-labyrinth]
        for lp, _ in sw.boundary_loops(Fdk, len(Vk2)):      # [fix3-labyrinth]
            if len(lp) < 3 or np.abs(hk_[lp]).max() > 3e-5: continue      # [fix3-labyrinth] (split() snaps corners within 1.5 % of an edge: up to ~15 um off)
            pg = Polygon(Vk2[lp][:, uv]).buffer(0)      # [fix3-labyrinth]
            if not pg.is_empty: region = pg if region is None else region.symmetric_difference(pg)      # [fix3-labyrinth]
        n0_ = len(CF)      # [fix3-labyrinth]
        for poly in (getattr(region, 'geoms', [region]) if region is not None else []):      # [fix3-labyrinth]
            if poly.geom_type != 'Polygon' or poly.area < 1e-9: continue      # [fix3-labyrinth]
            for t_ in shapely.constrained_delaunay_triangles(poly).geoms:      # [fix3-labyrinth]
                c = np.array(t_.exterior.coords)[:3]; X = np.zeros((3, 3)); X[:, uv] = c; X[:, ax] = v0 + sg * 2e-6      # [fix3-labyrinth]
                if np.cross(X[1] - X[0], X[2] - X[0])[ax] * sg > 0: X = X[::-1]      # [fix3-labyrinth]
                CV.append(X); CF.append(np.arange(3) + off); CP.append(0); off += 3      # [fix3-labyrinth]
        if len(CF) > n0_: d.update(cap_v=np.concatenate(CV), cap_f=np.array(CF).reshape(-1, 3).astype(np.int32), cap_part=np.array(CP, np.int8)); print('deck caps', len(CF) - n0_)      # [fix3-labyrinth]
if 'lid_pinion' in d:      # [fix4-hinge2] n14 / G5: cap the closed kit solids (hinge pod, pinion + arbor, cartridge, damper, tongue, ...) and the lid moulding
    from scipy.sparse import coo_matrix as _coo4; from scipy.sparse.csgraph import connected_components as _cc4      # [fix4-hinge2]
    def _caps4(Vc, Fc, hc, off=2.5e-6, weld=False):      # [fix4-hinge2] even-odd region of the boundary loops that lie in the plane -> cap triangles (facing the removed side)
        reg_ = None
        if weld:      # separate vertex arrays (lid skin + kitm): weld on a 0.1 um grid first
            _, fi_, wi_ = np.unique(np.round(Vc / 1e-7).astype(np.int64), axis=0, return_index=True, return_inverse=True); Fc = wi_.ravel()[Fc]; Vc = Vc[fi_]; hc = hc[fi_]
        for lp, _ in sw.boundary_loops(Fc, len(Vc)):
            if len(lp) < 3 or np.abs(hc[lp]).max() > 6e-5: continue
            pg = Polygon(Vc[lp][:, uv]).buffer(0)
            if not pg.is_empty: reg_ = pg if reg_ is None else reg_.symmetric_difference(pg)
        out = []
        for poly in (getattr(reg_, 'geoms', [reg_]) if reg_ is not None else []):
            if poly.geom_type != 'Polygon' or poly.area < 1e-10: continue
            for t_ in shapely.constrained_delaunay_triangles(poly).geoms:
                c = np.array(t_.exterior.coords)[:3]; X = np.zeros((3, 3)); X[:, uv] = c; X[:, ax] = v0 + sg * off
                if np.cross(X[1] - X[0], X[2] - X[0])[ax] * sg > 0: X = X[::-1]
                out.append(X)
        return out
    CAPL4 = {2: 7, 3: 7, 6: 9}      # cap material (design_render KITM): metal (steel / arm) 7 'SectionMetal', the lid 9 'SectionLid', every other (dark plastic, TPE) 8 'SectionDark'
    import lidmech as _lm4      # [fix4-hinge2]
    D0_ = np.load(sys.argv[1])      # [fix4-hinge2] the uncut kit meshes (component ids of the closed solids)
    for key in ('kitf', 'kitm'):      # [fix4-hinge2] per connected component; caps join the kit arrays with the component's material / part (they move with LidKit)
        Vk = d[key + '_v'].astype(float); Fk = d[key + '_f'].astype(np.int64); hk = (Vk[:, ax] - v0) * sg; n_ = len(Vk); P_ = d[key + '_part']; L_ = d[key + '_lab']
        lidc_ = np.isin(P_, [_lm4.PM['underside'], _lm4.PM['edge']]) if key == 'kitm' else np.zeros(len(Fk), bool)
        Vo_ = D0_[key + '_v'].astype(float); Fo_ = D0_[key + '_f'].astype(np.int64); no_ = len(Vo_)      # the solids BEFORE the cut (a cut cup falls apart into
        eo_ = np.r_[Fo_[:, [0, 1]], Fo_[:, [1, 2]]]; _, clo_ = _cc4(_coo4((np.ones(len(eo_)), (eo_[:, 0], eo_[:, 1])), shape=(no_, no_)), directed=False)      # outer and inner skins:
        _, Fr_, (cf_, _), _ = cut(Vo_, Fo_, [clo_[Fo_[:, 0]]]); assert len(Fr_) == len(Fk)      # their loops make ONE even-odd region)
        AV, AF, AL, AP = [], [], [], []; o_ = n_
        for c_ in np.unique(cf_[~lidc_]):
            m_ = (cf_ == c_) & ~lidc_; ml_ = CAPL4.get(int(np.bincount(L_[m_].astype(np.int64) % 64).argmax()), 8)
            if key == 'kitf' and int(np.bincount(P_[m_].astype(np.int64)).argmax()) == _lm4.PF['spring']: ml_ = 10      # the spiral spring: darker steel (reads against the cup / arbor)
            X_ = _caps4(Vk, Fk[m_], hk, 4.5e-6 if ml_ == 7 else 2.5e-6)      # metal inserts (blade in the keel, arbor in the spring) drawn in front of the plastic around them
            for X in X_: AV.append(X); AF.append(np.arange(3) + o_); AL.append(ml_); AP.append(int(np.bincount(P_[m_].astype(np.int64)).argmax())); o_ += 3
        if key == 'kitm' and lidc_.any():      # the lid moulding = skin (lab 4 of the main mesh, already cut) + kitm underside + edge
            ls_ = d['lab'] == 4; Vl_ = np.r_[d['v'].astype(float), Vk]; Fl_ = np.r_[d['f'][ls_].astype(np.int64), Fk[lidc_] + len(d['v'])]; hl_ = (Vl_[:, ax] - v0) * sg
            for X in _caps4(Vl_, Fl_, hl_, 1.5e-6, weld=True): AV.append(X); AF.append(np.arange(3) + o_); AL.append(9); AP.append(_lm4.PM['underside']); o_ += 3      # the lid moulding
        if AV: d[key + '_v'] = np.r_[Vk, np.concatenate(AV)]; d[key + '_f'] = np.r_[Fk, np.array(AF).reshape(-1, 3)].astype(Fk.dtype); d[key + '_lab'] = np.r_[L_, np.array(AL, L_.dtype)]; d[key + '_part'] = np.r_[P_, np.array(AP, P_.dtype)]
        print('[fix4-hinge2] %s caps' % key, len(AF))
if 'ports_wp' in d and len(d['sleeve_f']):      # [fix3-ports] the port frame is closed solids too: cap each component (even-odd), union of the components
    from scipy.sparse import coo_matrix as _coo; from scipy.sparse.csgraph import connected_components as _ccs      # [fix3-ports]
    Vs_ = d['sleeve_v'].astype(float); Fs_ = d['sleeve_f'].astype(np.int64); hs_ = (Vs_[:, ax] - v0) * sg; ns_ = len(Vs_); es_ = np.r_[Fs_[:, [0, 1]], Fs_[:, [1, 2]], Fs_[:, [2, 0]]]      # [fix3-ports]
    _, cs_ = _ccs(_coo((np.ones(len(es_)), (es_[:, 0], es_[:, 1])), shape=(ns_, ns_))); reg_ = None      # [fix3-ports]
    for c_ in np.unique(cs_[Fs_[:, 0]]):      # [fix3-ports]
        r_ = None
        for lp, _ in sw.boundary_loops(Fs_[cs_[Fs_[:, 0]] == c_], ns_):      # [fix3-ports] the cut sleeve mesh: its open loops lie in the plane
            if len(lp) < 3 or np.abs(hs_[lp]).max() > 1e-7: continue
            pg = Polygon(Vs_[lp][:, uv]).buffer(0); r_ = pg if r_ is None else r_.symmetric_difference(pg)      # [fix3-ports]
        if r_ is not None: reg_ = r_ if reg_ is None else reg_.union(r_)      # [fix3-ports]
    if reg_ is not None:      # [fix3-ports]
        CV_, CF_ = [], []; o_ = len(d['cap_v'])      # [fix3-ports]
        for poly in getattr(reg_, 'geoms', [reg_]):      # [fix3-ports]
            if poly.geom_type != 'Polygon' or poly.area < 1e-10: continue
            for t_ in shapely.constrained_delaunay_triangles(poly).geoms:      # [fix3-ports]
                c = np.array(t_.exterior.coords)[:3]; X = np.zeros((3, 3)); X[:, uv] = c; X[:, ax] = v0 + sg * 2e-6
                if np.cross(X[1] - X[0], X[2] - X[0])[ax] * sg > 0: X = X[::-1]
                CV_.append(X); CF_.append(np.arange(3) + o_); o_ += 3      # [fix3-ports]
        if CV_: d.update(cap_v=np.concatenate([d['cap_v']] + CV_), cap_f=np.r_[d['cap_f'], np.array(CF_, np.int32)], cap_part=np.r_[d['cap_part'], np.zeros(len(CF_), np.int8)]); print('port frame caps', len(CF_))      # [fix3-ports]
if 'lights_kit_v' in d:      # [fix3-lights] the light kit (lens, lamp module, bosses, harness): cut and capped per closed component (cap colour = the part's material)
    from scipy.sparse import coo_matrix as _coo; from scipy.sparse.csgraph import connected_components as _cc      # [fix3-lights]
    CV2, CF2, CL2 = [], [], []; o2 = 0      # [fix3-lights]
    for key, labk, extra in (('lights_lens', 'lights_lens_lab', ['lights_lens_side', 'lights_lens_id']), ('lights_kit', 'lights_kit_lab', ['lights_kit_kind']), ('lights_harn', 'lights_harn_lab', ['lights_harn_kind']), ('lights_halo', 'lights_halo_lab', [])):      # [fix3-lights] [fix3-int] + the halo feed pipe
        if not len(d[key + '_f']): continue      # [fix3-lights]
        Vk = d[key + '_v'].astype(float); Fk = d[key + '_f'].astype(np.int64); fkeys = [labk] + extra      # [fix3-lights]
        Vk2, Fk2, fk2, hk = cut(Vk, Fk, [d[k_] for k_ in fkeys]); d[key + '_v'] = Vk2; d[key + '_f'] = Fk2      # [fix3-lights]
        for k_, a_ in zip(fkeys, fk2): d[k_] = a_      # [fix3-lights]
        if key == 'lights_lens' and 'lights_lens_vn' in d:      # the lens shading normals follow the cut (new vertices: nearest old normal)      # [fix3-lights]
            from scipy.spatial import cKDTree as _KD; d['lights_lens_vn'] = d['lights_lens_vn'][_KD(Vk).query(Vk2)[1]]      # [fix3-lights]
        if not len(Fk2): continue      # [fix3-lights]
        n_ = len(Vk2); e_ = np.r_[Fk2[:, [0, 1]], Fk2[:, [1, 2]]]; _, cl_ = _cc(_coo((np.ones(len(e_)), (e_[:, 0], e_[:, 1])), shape=(n_, n_)), directed=False); cf_ = cl_[Fk2[:, 0]]      # [fix3-lights]
        for c_ in np.unique(cf_):      # [fix3-lights]
            m_ = cf_ == c_; region = None; lb_ = int(np.bincount(fk2[0][m_].astype(np.int64) % 64).argmax()) if key != 'lights_lens' else 99      # [fix3-lights]
            if key == 'lights_halo': lb_ = 98      # [fix3-int] halo feed caps move with the lid (renderer: LightHaloCap)
            for lp, _ in sw.boundary_loops(Fk2[m_], n_):      # [fix3-lights]
                if len(lp) < 3 or np.abs(hk[lp]).max() > 1e-7: continue      # [fix3-lights]
                pg = Polygon(Vk2[lp][:, uv]).buffer(0)      # [fix3-lights]
                if pg.is_empty: continue      # [fix3-lights]
                region = pg if region is None else region.symmetric_difference(pg)      # [fix3-lights]
            if region is None: continue      # [fix3-lights]
            for poly in getattr(region, 'geoms', [region]):      # [fix3-lights]
                if poly.geom_type != 'Polygon' or poly.area < 1e-10: continue      # [fix3-lights]
                for t_ in shapely.constrained_delaunay_triangles(poly).geoms:      # [fix3-lights]
                    c = np.array(t_.exterior.coords)[:3]; X = np.zeros((3, 3)); X[:, uv] = c; X[:, ax] = v0 + sg * 3e-6      # [fix3-lights]
                    if np.cross(X[1] - X[0], X[2] - X[0])[ax] * sg > 0: X = X[::-1]      # [fix3-lights]
                    CV2.append(X); CF2.append(np.arange(3) + o2); CL2.append(lb_); o2 += 3      # [fix3-lights]
    d.update(lights_cap_v=np.concatenate(CV2) if CV2 else np.zeros((0, 3)), lights_cap_f=np.array(CF2).reshape(-1, 3).astype(np.int32), lights_cap_lab=np.array(CL2, np.int8))      # [fix3-lights]
    print('light kit caps', len(CF2))      # [fix3-lights]
np.savez(OUT, **d); print('saved', OUT)
