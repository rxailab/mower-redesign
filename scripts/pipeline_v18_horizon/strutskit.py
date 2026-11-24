# [fix3-struts] Chassis strut / stub handling for the v18 body (fix round 3).
#
# Until f2 the renderer, the web exporter / viewer and the checks deleted EVERY chassis vertex at |y| > 160, z > 135, -60 < x < 290 mm
# (except within 6.8 mm of a screw post), calling it "the struts of the old mudguards". Measured against the real chassis
# (dev/base_parts.npz = base.glb) that region holds four different things per side:
#   rim flange   x 40..215, z 200..225, outer edge |y| 165.9..167.0   -> clear of the body (>= 1.05 mm to the inner skin): KEPT
#   drive unit   housing face |y| 171.0..172.5 over x 78..160, z 72..155, bearing ring to |y| 178; top face z 152..155
#                                                                  -> drive train, NOT a strut: KEPT; the body's lower edge (skirt.seat)
#                                                                     rises to z 156.5 over it (1.5 mm gap) and uses its top face as a stop
#   pin          old mudguard locating pin, ~12 mm, on the rim flange above the wheel hub, z 214.5..236.4, outboard face |y| 168.6 / 169.9
#                                                                  -> 0.3 / 1.5 mm into the 2.5 mm body wall and through the deck edge: CUT
#   hook         old mudguard front hook (L-bracket) ahead of the wheel, x 258..273, z 149.3..163.4, out to |y| 168.0 / 171.6
#                                                                  -> 1.4 mm into / 2.3 mm through the nose-corner wall: CUT
#
# A CUT = remove everything inside an axis-aligned box. Exactly one face of the box is the cut plane (p, n; n points into the removed
# side); every other face of the box lies in air (checked on the scan when the spec is written), so the result is a plane cut + a flat
# cap, nothing else. The cuts are given once (|y| ranges) in params['struts'] and mirrored to both sides.
# Used by: design_gen.py (writes the spec into the npz as 'struts_spec'), design_render.py (cut_bmesh: bisect + fill in Blender),
# wall_clear.py / deckcheck.py / strutcheck.py (inside / cap_samples on point samples), and `python strutskit.py PARAMS.json OUT.json`
# writes struts_cut.json for the web exporter / viewer (world AND glTF frames, cap triangles included).
import numpy as np, json

AX = {'x': 0, 'y': 1, 'ay': 1, 'z': 2}


def spec(P):
    """params -> {'mode': 'cut' | 'legacy', 'cuts': [{id, feature, side, box [[x0, x1], [y0, y1], [z0, z1]], p, n, note}], ...}"""
    S = P.get('struts') or {}
    cuts = []
    for c in S.get('cuts', []):
        for sg, sn in (((1, 'L'), (-1, 'R')) if c.get('mirror', True) else ((1, ''),)):
            b = c['box']; ys = sorted([sg * b['ay'][0], sg * b['ay'][1]]) if 'ay' in b else [float(t) for t in b['y']]
            box = [[float(t) for t in b['x']], ys, [float(t) for t in b['z']]]
            ax = c['plane']['axis']; i = AX[ax]; at = float(c['plane']['at']); p = [0.0, 0.0, 0.0]; n = [0.0, 0.0, 0.0]
            if ax == 'ay': p[1] = sg * at; n[1] = float(sg)
            else: p[i] = at; n[i] = 1.0 if c['plane'].get('remove', '+') == '+' else -1.0
            face = box[i][0] if n[i] > 0 else box[i][1]
            if abs(face - p[i]) > 1e-9: raise ValueError('struts cut %s: the plane must be a face of its box' % c['id'])
            cuts.append(dict(id=c['id'] + ('_' + sn if sn else ''), feature=c.get('feature', c['id']), side=sn, box=box, p=p, n=n, note=c.get('//', '')))
    return dict(mode=S.get('mode', 'cut') if cuts else 'legacy', cuts=cuts)


def from_npz(d):
    """the spec the generator wrote into a design npz (None for older npz files)"""
    return json.loads(str(d['struts_spec'])) if 'struts_spec' in getattr(d, 'files', d) else None


def legacy_kill(X, posts):
    """the f2 rule (vertex mask): |y| > 160, z > 135, -60 < x < 290 mm, not within 6.8 mm of a screw post"""
    k = (np.abs(X[:, 1]) > 0.160) & (X[:, 2] > 0.135) & (X[:, 0] > -0.06) & (X[:, 0] < 0.29)
    for p_ in posts: k &= (X[:, 0] - p_[0]) ** 2 + (X[:, 1] - p_[1]) ** 2 > 0.0068 ** 2
    return k


def inside(P, sp, eps=1e-7):
    """points removed by the cuts (strictly inside a box)"""
    m = np.zeros(len(P), bool)
    for c in (sp or {}).get('cuts', []):
        b = np.array(c['box']); m |= ((P > b[:, 0] + eps) & (P < b[:, 1] - eps)).all(1)
    return m


def _frame(c):
    i = int(np.argmax(np.abs(c['n']))); u, v = [j for j in range(3) if j != i]; return i, u, v


def section(V, F, c):
    """segments (k, 2, 3) where the mesh crosses the cut plane, inside the box face rectangle"""
    i, u, v = _frame(c); b = np.array(c['box']); p0 = c['p'][i]
    T = V[F]; lo = T.min(1); hi = T.max(1)
    pre = (lo[:, i] < p0) & (hi[:, i] > p0) & (hi[:, u] > b[u, 0]) & (lo[:, u] < b[u, 1]) & (hi[:, v] > b[v, 0]) & (lo[:, v] < b[v, 1])
    T = T[pre]; d = T[:, :, i] - p0; out = []
    for (a_, b_) in ((0, 1), (1, 2), (2, 0)):
        x, y = d[:, a_], d[:, b_]; m = x * y < 0; t = np.where(m, x / np.where(m, x - y, 1), 0)
        out.append((m, T[:, a_] + (T[:, b_] - T[:, a_]) * t[:, None]))
    seg = [np.stack([pa[ma & mb], pb[ma & mb]], 1) for (ma, pa), (mb, pb) in ((out[0], out[1]), (out[1], out[2]), (out[2], out[0]))]
    S = np.concatenate(seg); S[:, :, i] = p0
    k = (S[:, :, u].min(1) >= b[u, 0]) & (S[:, :, u].max(1) <= b[u, 1]) & (S[:, :, v].min(1) >= b[v, 0]) & (S[:, :, v].max(1) <= b[v, 1])
    return S[k]


def cap_polys(V, F, sp, close_gap=0.004):
    """per cut: the flat cap (shapely geometry in the plane's (u, v) coordinates) = solid section on the plane minus the other boxes;
    a section line left open by a hole in the scan is closed when its two ends are < close_gap apart. Returns [(cut, geom, report)]."""
    import shapely
    from shapely.ops import linemerge, unary_union, polygonize
    from shapely.geometry import Polygon, box as sbox
    out = []
    for c in sp['cuts']:
        i, u, v = _frame(c); S = section(V, F, c); rep = dict(segments=int(len(S)), closed_gaps=[])
        if not len(S): out.append((c, Polygon(), rep)); continue
        L = linemerge(unary_union(shapely.multilinestrings([s[:, [u, v]].tolist() for s in S])))
        polys = []
        for g in (L.geoms if hasattr(L, 'geoms') else [L]):
            cs = np.array(g.coords)
            if not g.is_closed:
                gap = float(np.linalg.norm(cs[0] - cs[-1]))
                if gap > close_gap or len(cs) < 4: continue
                rep['closed_gaps'].append(round(gap * 1000, 2))
            pg = Polygon(cs).buffer(0)
            if pg.area > 0: polys.append(pg)
        G = Polygon()
        for pg in sorted(polys, key=lambda q: -q.area): G = G.symmetric_difference(pg)      # even-odd: holes in a solid section
        for c2 in sp['cuts']:
            if c2 is c: continue
            b2 = np.array(c2['box'])
            if b2[i, 0] < c['p'][i] < b2[i, 1]: G = G.difference(sbox(b2[u, 0], b2[v, 0], b2[u, 1], b2[v, 1]))
        rep['area_mm2'] = round(G.area * 1e6, 1); rep['bounds_mm'] = [round(t * 1000, 1) for t in G.bounds] if not G.is_empty else None
        out.append((c, G, rep))
    return out


def _to3(c, uv):
    i, u, v = _frame(c); P = np.zeros((len(uv), 3)); P[:, u] = uv[:, 0]; P[:, v] = uv[:, 1]; P[:, i] = c['p'][i]; return P


def cap_samples(V, F, sp, step=0.0004):
    """points on every cap (grid inside + outline), for the clearance checks"""
    import shapely
    pts = [np.zeros((0, 3))]
    for c, G, _ in cap_polys(V, F, sp):
        if G.is_empty: continue
        x0, y0, x1, y1 = G.bounds; gx, gy = np.meshgrid(np.arange(x0, x1 + step, step), np.arange(y0, y1 + step, step)); q = np.c_[gx.ravel(), gy.ravel()]
        q = q[shapely.contains_xy(G, q[:, 0], q[:, 1])]
        b = shapely.segmentize(G.boundary, step); bc = shapely.get_coordinates(b)
        pts.append(_to3(c, np.r_[q, bc]))
    return np.concatenate(pts)


def cap_mesh(V, F, sp):
    """cap triangles (V, F, cut index), wound so the normal points along n (out of the kept material)"""
    import shapely
    CV, CF, CI = [], [], []; off = 0
    for k, (c, G, _) in enumerate(cap_polys(V, F, sp)):
        if G.is_empty: continue
        T = shapely.get_parts(shapely.constrained_delaunay_triangles(G))
        for t in T:
            q = np.array(t.exterior.coords)[:3]; P = _to3(c, q); nn = np.cross(P[1] - P[0], P[2] - P[0])
            if np.dot(nn, c['n']) < 0: P = P[[0, 2, 1]]
            CV.append(P); CF.append([off, off + 1, off + 2]); CI.append(k); off += 3
    if not CV: return np.zeros((0, 3)), np.zeros((0, 3), np.int64), np.zeros(0, np.int64)
    return np.concatenate(CV), np.array(CF, np.int64), np.array(CI, np.int64)


def air_check(V, F, sp, eps=2e-5):
    """the box faces other than the cut plane must lie in air: mesh faces crossing them (count per cut, should be 0)"""
    out = {}
    T = V[F]; lo = T.min(1); hi = T.max(1)
    for c in sp['cuts']:
        b = np.array(c['box']); i = _frame(c)[0]; n = 0
        near = ((hi > b[:, 0] - eps) & (lo < b[:, 1] + eps)).all(1)
        for a in range(3):
            for s in (0, 1):
                if a == i and ((c['n'][i] > 0) == (s == 0)): continue          # the cut plane itself
                w = b[a, s]; o = [j for j in range(3) if j != a]
                cr = near & (lo[:, a] < w - eps) & (hi[:, a] > w + eps) & (lo[:, o[0]] < b[o[0], 1]) & (hi[:, o[0]] > b[o[0], 0]) & (lo[:, o[1]] < b[o[1], 1]) & (hi[:, o[1]] > b[o[1], 0])
                n += int(cr.sum())
        out[c['id']] = n
    return out


def cut_bmesh(ob, sp, caps=None, cap_mat=None):
    """Blender: cut a mesh object exactly by the spec - bisect the faces near each box with its plane and delete what lies inside the
    box; then add the flat caps and weld them to the cut outline. caps = (V world, F, cut index) from cap_mesh() - the section outline of
    the ORIGINAL mesh on each plane, trimmed by the other boxes, constrained Delaunay; the generator stores them in the npz (struts_cap_*,
    Blender's Python has no shapely) and struts_cut.json gives the viewer the same triangles. Returns a small report."""
    import bmesh
    from mathutils import Vector
    me = ob.data; mw = ob.matrix_world.copy(); mwi = mw.inverted(); R3 = mw.to_3x3()
    if caps is None:
        me.calc_loop_triangles(); Vl = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get('co', Vl); Vl = Vl.reshape(-1, 3)
        Fl = np.zeros(len(me.loop_triangles) * 3, np.int64); me.loop_triangles.foreach_get('vertices', Fl); Fl = Fl.reshape(-1, 3)
        M = np.array(mw); caps = cap_mesh(Vl @ M[:3, :3].T + M[:3, 3], Fl, sp)      # needs shapely
    CVw, CFc, CIc = [np.asarray(t) for t in caps]
    bm = bmesh.new(); bm.from_mesh(me); rep = {}
    U = np.array([[c['box'][a][0] for a in range(3)] for c in sp['cuts']]).min(0) - 0.002; W_ = np.array([[c['box'][a][1] for a in range(3)] for c in sp['cuts']]).max(0) + 0.002
    pool = {f for f in bm.faces if any(all(U[a] <= (mw @ v.co)[a] <= W_[a] for a in range(3)) for v in f.verts)}      # every face any cut can touch
    for c in sp['cuts']:
        b = c['box']; pw = Vector(c['p']); nw = Vector(c['n'])
        def inb(w, e):
            return all(b[a][0] - e <= w[a] <= b[a][1] + e for a in range(3))
        pool = {f for f in pool if f.is_valid}
        fs = [f for f in pool if any(inb(mw @ v.co, 5e-4) for v in f.verts)]
        if not fs: rep[c['id']] = dict(faces=0); continue
        vs = list({v for f in fs for v in f.verts}); es = list({e for f in fs for e in f.edges})
        nl = (R3.transposed() @ nw).normalized()          # plane normal in object space (rotation / uniform scale)
        res = bmesh.ops.bisect_plane(bm, geom=vs + es + fs, dist=1e-7, plane_co=mwi @ pw, plane_no=nl, clear_inner=False, clear_outer=False)
        loc = {g for g in res['geom'] if isinstance(g, bmesh.types.BMFace)} | set(fs); pool |= loc
        dead = [f for f in loc if f.is_valid and all(inb(mw @ v.co, 1e-6) for v in f.verts) and inb(mw @ f.calc_center_median(), -1e-7)]
        bmesh.ops.delete(bm, geom=dead, context='FACES')
        pool = {f for f in pool if f.is_valid}
        rest = sum(1 for f in pool if inb(mw @ f.calc_center_median(), -2e-5))
        rep[c['id']] = dict(faces_near=len(fs), removed=len(dead), faces_left_in_box=rest)
    nv = [bm.verts.new(mwi @ Vector(p)) for p in CVw]; cf = []
    for (i, j, k), ci in zip(CFc, CIc):
        try: f = bm.faces.new((nv[i], nv[j], nv[k]))
        except ValueError: continue
        f.smooth = False; cf.append(f)
        if cap_mat is not None: f.material_index = cap_mat
    def onplane(v):
        w = mw @ v.co
        return any(abs((w - Vector(c['p'])).dot(Vector(c['n']))) < 3e-6 and all(c['box'][a][0] - 1e-5 <= w[a] <= c['box'][a][1] + 1e-5 for a in range(3)) for c in sp['cuts'])
    wv = list({v for f in pool if f.is_valid for v in f.verts if onplane(v)} | set(nv))
    bmesh.ops.remove_doubles(bm, verts=wv, dist=2e-6)                       # weld the caps to the cut outline
    cf = [f for f in cf if f.is_valid]
    for k_, c in enumerate(sp['cuts']):
        m_ = [f for f, ci in zip(cf, CIc) if ci == k_]
        if c['id'] in rep: rep[c['id']]['cap_area_mm2'] = round(float(sum(f.calc_area() for f in m_)) * 1e6, 1)
    pool = {f for f in pool if f.is_valid} | set(cf)
    rep['open_edges_on_cut_planes'] = sum(1 for e in {e for f in pool for e in f.edges} if e.is_boundary and onplane(e.verts[0]) and onplane(e.verts[1]))
    bm.to_mesh(me); bm.free(); me.update()
    return rep


def viewer_json(P, V, F, out_path):
    """struts_cut.json for the web exporter / viewer: the same cuts in the world frame and in glTF axes (x, z, -y), with the cap triangles"""
    sp = spec(P); g = lambda a: [float(a[0]), float(a[2]), float(-a[1])]
    CVv, CFf, CI = cap_mesh(V, F, sp); polys = cap_polys(V, F, sp); cuts = []
    for k, (c, G, rep) in enumerate(polys):
        b = np.array(c['box']); bg = np.array([b[0], b[2], sorted([-b[1, 1], -b[1, 0]])])
        m = CI == k; idx = np.unique(CFf[m]); rm = -np.ones(len(CVv), np.int64); rm[idx] = np.arange(len(idx))
        cuts.append(dict(id=c['id'], feature=c['feature'], side=c['side'], note=c['note'],
                         world=dict(box_min=b[:, 0].round(6).tolist(), box_max=b[:, 1].round(6).tolist(), plane_point=c['p'], plane_normal=c['n']),
                         gltf=dict(box_min=bg[:, 0].round(6).tolist(), box_max=bg[:, 1].round(6).tolist(), plane_point=g(c['p']), plane_normal=g(c['n'])),
                         cap=dict(area_mm2=rep['area_mm2'], gltf_positions=np.array([g(p) for p in CVv[idx]]).round(6).ravel().tolist(), indices=rm[CFf[m]].ravel().tolist())))
    J = {'//': ['[fix3-struts] chassis cut for the v18 body (replaces the f2 vertex deletion |y| > 160, z > 135, -60 < x < 290 mm).',
                'Rule: per cut, remove every chassis triangle part that lies inside the box [box_min, box_max]; exactly one box face is the cut plane',
                '(plane_point, plane_normal: the normal points INTO the removed side) - clip the triangles that cross it there, keep the other side;',
                'all other box faces lie in air. Then add the cap triangles (flat, chassis material) - they close the cut. Cuts are applied in order;',
                'caps are already trimmed by the other boxes. world: metres, +X forward, +Y left, Z up. gltf: (x, z, -y) as in base.glb / the viewer.',
                'Kept (NOT cut any more): the rim flange beside the wheels, the drive-unit housings (the body edge now clears them by 1.5 mm).'],
         'source': 'base.glb object Chassis (= dev/base_parts.npz Chassis_v / Chassis_f)', 'cuts': cuts}
    json.dump(J, open(out_path, 'w'), indent=1)
    return J


if __name__ == '__main__':          # python strutskit.py PARAMS.json OUT.json   (the viewer / exporter data; also prints the air check)
    import sys
    P = json.load(open(sys.argv[1])); B = np.load('dev/base_parts.npz'); V = B['Chassis_v'].astype(float); F = B['Chassis_f'].astype(np.int64)
    sp = spec(P); print('air check (faces crossing a non-cut box face, want 0):', air_check(V, F, sp))
    for c, G, rep in cap_polys(V, F, sp): print(c['id'], rep)
    viewer_json(P, V, F, sys.argv[2]); print('wrote', sys.argv[2])
