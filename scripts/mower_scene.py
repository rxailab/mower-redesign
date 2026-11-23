"""Shared Blender scene builder for the assembled mower.

Used by render_asm2.py (full-resolution Cycles renders) and export_glb.py
(decimated glTF models for the web viewer).  All geometry is read from the
work directory (world space, Z up, metres, front of the mower = +X):

  asm_shell.npz        top shell (outer skin, solidified here)
  asm_middle2_lab.npz  middle layer with per-face material labels
  asm_chassis.npz      chassis
  asm_wheels.npz       both drive wheels
  asm_casters.npz      both caster wheels
  mid_tex_geom.json    UV frames for the display and sensor window
  wheel_geom.json      drive-wheel axles / centres
  tex/display.png, tex/window.png
"""
import bpy, bmesh, numpy as np, json, os, math, mathutils

MID_NAMES = ['MidBody', 'StopButton', 'DisplayGlass', 'PodLight', 'SensorGlass', 'LEDLens', 'SlotInner', 'Bumper']
PAINT = {"black": (0.015, 0.015, 0.017), "yellow": (0.88, 0.58, 0.0), "blue": (0.0, 0.035, 0.33),
         "red": (0.42, 0.0, 0.004), "stripes": (0.78, 0.78, 0.76)}


# two-tone colour schemes (viewer/models/palettes.json): sRGB hex + a finish preset per part
FINISH = {
    'shell': {'gloss': (0.30, 0.0, 0.8, 0.06), 'metallic': (0.36, 0.55, 1.0, 0.05),
              'satin': (0.50, 0.0, 0.15, 0.2), 'matte': (0.75, 0.0, 0.0, 0.3)},
    'middle': {'gloss': (0.28, 0.0, 0.45, 0.08), 'metallic': (0.40, 0.5, 0.6, 0.08),
               'satin': (0.52, 0.0, 0.0, 0.3), 'matte': (0.72, 0.0, 0.0, 0.3)},
}


def hex_lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)


def load_palettes(path=None):
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'viewer', 'models', 'palettes.json')
    return json.load(open(path))


def _set_bsdf(mt, col, rough, metal, coat, coat_rough):
    p = mt.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*col, 1)
    p.inputs["Roughness"].default_value = rough; p.inputs["Metallic"].default_value = metal
    p.inputs["Coat Weight"].default_value = coat; p.inputs["Coat Roughness"].default_value = coat_rough


def apply_palette(M, pal):
    """Recolour ShellPaint / MidBody / PodLight in place (fast: no scene rebuild)."""
    sc = hex_lin(pal['shell']['hex']); _set_bsdf(M['ShellPaint'], sc, *FINISH['shell'][pal['shell']['finish']])
    _set_bsdf(M['MidBody'], hex_lin(pal['middle']['hex']), *FINISH['middle'][pal['middle']['finish']])
    if pal.get('pod'):
        M['PodLight'].node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (*hex_lin(pal['pod']), 1)
    nt = M['ShellPaint'].node_tree; N = nt.nodes; Lk = nt.links; b = N["Principled BSDF"]
    if "StripeMix" not in N:                       # two stripes over the centre line, |y| in [11, 56] mm
        tc = N.new("ShaderNodeTexCoord"); sp = N.new("ShaderNodeSeparateXYZ"); Lk.new(tc.outputs["Object"], sp.inputs[0])
        ab = N.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"; Lk.new(sp.outputs["Y"], ab.inputs[0])
        g = N.new("ShaderNodeMath"); g.operation = "GREATER_THAN"; g.inputs[1].default_value = 0.011; Lk.new(ab.outputs[0], g.inputs[0])
        l = N.new("ShaderNodeMath"); l.operation = "LESS_THAN"; l.inputs[1].default_value = 0.056; Lk.new(ab.outputs[0], l.inputs[0])
        mm = N.new("ShaderNodeMath"); mm.operation = "MULTIPLY"; Lk.new(g.outputs[0], mm.inputs[0]); Lk.new(l.outputs[0], mm.inputs[1])
        on = N.new("ShaderNodeMath"); on.operation = "MULTIPLY"; on.name = "StripeOn"; Lk.new(mm.outputs[0], on.inputs[0])
        mx = N.new("ShaderNodeMix"); mx.data_type = "RGBA"; mx.name = "StripeMix"; Lk.new(on.outputs[0], mx.inputs["Factor"])
        Lk.new(mx.outputs["Result"], b.inputs["Base Color"])
    N["StripeOn"].inputs[1].default_value = 1.0 if pal.get('stripe') else 0.0
    if pal.get('ring'):                            # drive-wheel light ring follows the scheme
        rl = hex_lin(pal['ring']); mx = max(rl) or 1.0; e = tuple(c / mx for c in rl)
        rp = M['CyanRing'].node_tree.nodes["Principled BSDF"]
        rp.inputs["Base Color"].default_value = (*[0.78 * c for c in e], 1); rp.inputs["Emission Color"].default_value = (*e, 1)
    N["StripeMix"].inputs["A"].default_value = (*sc, 1)
    N["StripeMix"].inputs["B"].default_value = (*hex_lin(pal.get('stripe') or '#000000'), 1)


# --------------------------------------------------------------------------- helpers
def mesh_obj(name, V, F, mat_idx=None, smooth=True, origin=None):
    V = np.asarray(V, np.float64)
    if origin is not None:
        V = V - origin
    m = bpy.data.meshes.new(name)
    m.vertices.add(len(V)); m.vertices.foreach_set("co", V.astype(np.float32).ravel())
    F = np.asarray(F, np.int32)
    m.loops.add(F.size); m.loops.foreach_set("vertex_index", F.ravel())
    m.polygons.add(len(F))
    m.polygons.foreach_set("loop_start", np.arange(0, F.size, 3, dtype=np.int32))
    m.polygons.foreach_set("loop_total", np.full(len(F), 3, np.int32))
    if mat_idx is not None:
        m.polygons.foreach_set("material_index", np.asarray(mat_idx, np.int32))
    m.polygons.foreach_set("use_smooth", np.full(len(F), smooth, bool))
    m.update(); m.validate()
    o = bpy.data.objects.new(name, m)
    bpy.context.scene.collection.objects.link(o)
    if origin is not None:
        o.location = mathutils.Vector(origin.tolist())
    return o


def principled(name, col, rough, metal=0.0, coat=0.0, coat_rough=0.05, spec=0.5,
               emit=None, emit_strength=0.0, image=None, emit_image=None):
    mt = bpy.data.materials.new(name); mt.use_nodes = True
    nt = mt.node_tree; p = nt.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*col, 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    p.inputs["Specular IOR Level"].default_value = spec
    p.inputs["Coat Weight"].default_value = coat
    p.inputs["Coat Roughness"].default_value = coat_rough
    if emit is not None:
        p.inputs["Emission Color"].default_value = (*emit, 1)
        p.inputs["Emission Strength"].default_value = emit_strength
    if image is not None:
        tx = nt.nodes.new("ShaderNodeTexImage"); tx.image = bpy.data.images.load(image)
        tx.image.colorspace_settings.name = 'sRGB'
        nt.links.new(tx.outputs["Color"], p.inputs["Base Color"])
    if emit_image is not None:
        te = nt.nodes.new("ShaderNodeTexImage"); te.image = bpy.data.images.load(emit_image)
        te.image.colorspace_settings.name = 'sRGB'
        nt.links.new(te.outputs["Color"], p.inputs["Emission Color"])
    return mt


def make_materials(WD, color="black"):
    M = {}
    M['ShellPaint'] = principled('ShellPaint', PAINT[color], 0.4 if color == "black" else 0.3, coat=0.8, coat_rough=0.07)
    M['MidBody'] = principled('MidBody', (0.035, 0.037, 0.041), 0.55)
    M['StopButton'] = principled('StopButton', (0.87, 0.105, 0.045), 0.45)
    M['DisplayGlass'] = principled('DisplayGlass', (0.02, 0.02, 0.025), 0.18, coat=0.55, coat_rough=0.04, spec=0.35,
                                   image=os.path.join(WD, 'tex', 'display.png'))
    M['PodLight'] = principled('PodLight', (0.42, 0.46, 0.52), 0.5)
    M['SensorGlass'] = principled('SensorGlass', (0.01, 0.01, 0.012), 0.04, coat=1.0, coat_rough=0.01,
                                  image=os.path.join(WD, 'tex', 'window.png'))
    M['LEDLens'] = principled('LEDLens', (0.85, 0.87, 0.9), 0.15, coat=0.6, coat_rough=0.03, emit=(1.0, 0.96, 0.88),
                              emit_strength=1.3, image=os.path.join(WD, 'tex', 'led.png'),
                              emit_image=os.path.join(WD, 'tex', 'led_emit.png'))
    M['LedBezel'] = principled('LedBezel', (0.78, 0.79, 0.8), 0.18, metal=1.0)
    M['LedGap'] = principled('LedGap', (0.008, 0.008, 0.009), 0.7)
    M['SlotInner'] = principled('SlotInner', (0.012, 0.012, 0.013), 0.6)
    M['Bumper'] = principled('Bumper', (0.02, 0.02, 0.022), 0.6)
    M['DockBody'] = principled('DockBody', (0.028, 0.029, 0.032), 0.55)
    M['DockPin'] = principled('DockPin', (0.80, 0.60, 0.36), 0.3, metal=1.0)
    M['ChassisBlack'] = principled('ChassisBlack', (0.02, 0.02, 0.022), 0.6)
    M['Rubber'] = principled('Rubber', (0.022, 0.024, 0.03), 0.72, spec=0.35)
    M['WheelHub'] = principled('WheelHub', (0.018, 0.022, 0.034), 0.45)
    M['CyanRing'] = principled('CyanRing', (0.08, 0.62, 0.78), 0.3, emit=(0.1, 0.85, 1.0), emit_strength=1.4)
    return M


def caster_pivots(Vc):
    """Split casters by side; return per side: wheel centre, axle, swivel pivot (x,y) (symmetric fit)."""
    out = {}
    for s, nm in [(1, 'L'), (-1, 'R')]:
        W = Vc[np.sign(Vc[:, 1]) == s]
        c = (W.max(0) + W.min(0)) / 2
        _, _, vt = np.linalg.svd(W[::5] - W.mean(0), full_matrices=False); ax = vt[2]
        if ax[1] * s < 0: ax = -ax
        roll = np.cross(ax, [0, 0, 1.0]); roll[2] = 0; roll /= np.linalg.norm(roll)
        out[nm] = dict(c=c, ax=ax, roll=roll, r=(W[:, 2].max() - W[:, 2].min()) / 2)
    best = None
    for sA in (1, -1):
        for sB in (1, -1):
            # unknowns xp, yp, rt : cA = (xp, yp) + sA rt dA ; cB = (xp,-yp) + sB rt dB
            A = []; b = []
            for nm, sgn, sv in [('L', 1, sA), ('R', -1, sB)]:
                d = out[nm]['roll']; c = out[nm]['c']
                A.append([1, 0, sv * d[0]]); b.append(c[0])
                A.append([0, sgn, sv * d[1]]); b.append(c[1])
            A = np.array(A); b = np.array(b)
            x, res, *_ = np.linalg.lstsq(A, b, rcond=None)
            r = np.linalg.norm(A @ x - b)
            if x[2] > 0 and (best is None or r < best[0]):
                best = (r, x, sA, sB)
    _, (xp, yp, rt), sA, sB = best
    out['L']['pivot'] = np.array([xp, yp]); out['R']['pivot'] = np.array([xp, -yp])
    out['trail'] = rt
    return out


def wheel_labels(V, F, center, axle):
    C = V[F].mean(1) - center
    t = C @ axle
    r = np.linalg.norm(C - t[:, None] * axle, axis=1)
    lab = np.zeros(len(F), np.int32)              # 0 rubber
    outer = t > 0.012
    lab[outer & (r <= 0.0815)] = 1                 # hub dish
    lab[outer & (r > 0.0815) & (r < 0.0912)] = 2   # light ring on the hub lip, next to the tyre
    lab[(t < -0.012) & (r < 0.0912)] = 1
    return lab


def compact(V, F):
    u = np.unique(F); remap = -np.ones(len(V), np.int64); remap[u] = np.arange(len(u))
    return V[u], remap[F]


def set_middle_uvs(o, V, F, lab, geo):
    m = o.data
    uv = np.zeros((len(F) * 3, 2), np.float32)
    P = V[F.ravel()]
    L3 = np.repeat(lab, 3)
    d = geo['display']
    sel = L3 == 2
    uv[sel, 0] = (d['y1'] - P[sel, 1]) / (d['y1'] - d['y0'])
    uv[sel, 1] = (P[sel, 0] - d['x0']) / (d['x1'] - d['x0'])
    w = geo['window']; pf = geo['pod_frame']
    o_ = np.array(pf['o']); R = np.array(pf['right']); U = np.array(pf['up'])
    sel = L3 == 4
    Q = P[sel] - o_
    uv[sel, 0] = (Q @ R - w['u0']) / (w['u1'] - w['u0'])
    uv[sel, 1] = (Q @ U - w['v0']) / (w['v1'] - w['v0'])
    ul = m.uv_layers.new(name="UVMap"); ul.data.foreach_set("uv", uv.ravel())


def smooth_by_angle(o, deg=40):
    bpy.context.view_layer.objects.active = o
    for ob in bpy.context.selected_objects: ob.select_set(False)
    o.select_set(True)
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(deg))
    except Exception as e:
        print("smooth_by_angle failed:", e)
    o.select_set(False)


def relax_flips(o, rounds=6):
    """After decimation: find faces whose normal opposes the locally smoothed surface normal (tiny pleats that
    render as see-through specks with back-face culling) and relax the vertices around them (numpy only)."""
    me = o.data; nv = len(me.vertices)
    V = np.empty(nv * 3); me.vertices.foreach_get('co', V); V = V.reshape(-1, 3)
    F = np.array([p.vertices[:] for p in me.polygons if len(p.vertices) == 3], np.int64)
    I = np.r_[F[:, 0], F[:, 1], F[:, 2], F[:, 1], F[:, 2], F[:, 0]]; J = np.r_[F[:, 1], F[:, 2], F[:, 0], F[:, 0], F[:, 1], F[:, 2]]
    deg = np.maximum(np.bincount(I, minlength=nv), 1).astype(float)
    def nsum(X):
        S = np.zeros_like(X); np.add.at(S, I, X[J]); return S
    total = 0; bad = np.zeros(len(F), bool)
    for r in range(rounds):
        fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        vn = np.zeros_like(V)
        for k in range(3): np.add.at(vn, F[:, k], fn)
        for _ in range(3): vn = vn + nsum(vn)
        bad = np.sum(fn * vn[F].sum(1), 1) < 0
        if r == 0: total = int(bad.sum())
        if not bad.any(): break
        m = np.zeros(nv); m[np.unique(F[bad])] = 1
        for _ in range(2): m = np.minimum(1, m + nsum(m))
        idx = np.nonzero(m)[0]
        for _ in range(20): V[idx] += 0.5 * (nsum(V)[idx] / deg[idx, None] - V[idx])
    me.vertices.foreach_set('co', V.ravel()); me.update()
    print(f"relax_flips {o.name}: {total} flipped faces before, {int(bad.sum())} after")


def surface_bvh(o):
    """BVH of o's current mesh in world space (used for the decimated shell's outer skin)."""
    from mathutils.bvhtree import BVHTree
    mw = o.matrix_world
    return BVHTree.FromPolygons([mw @ v.co for v in o.data.vertices], [p.vertices[:] for p in o.data.polygons])


def tuck_under(o, bvh, clr):
    """After decimation, parts under the shell can poke through its paint (single dark pixels on light colours).
    Push every vertex that sits under the shell's skin and closer than clr to it back to clr below the skin."""
    mw = o.matrix_world; mwi = mw.inverted(); n = 0
    for v in o.data.vertices:
        w = mw @ v.co
        loc, nrm, idx, dist = bvh.find_nearest(w, 0.02)
        if loc is None: continue
        d = w - loc; s = d.dot(nrm)
        if (d - s * nrm).length < 0.0015 and s > -clr:
            v.co = mwi @ (w - (s + clr) * nrm); n += 1
    o.data.update(); print(f"tuck_under {o.name}: {n} vertices pushed under the shell")


def decimate(o, ratio, vgroup=None):
    """Collapse-decimate o; runs before any other modifier on the stack (e.g. the shell's Solidify)."""
    if ratio >= 1.0: return
    d = o.modifiers.new("Dec", 'DECIMATE'); d.ratio = ratio; d.use_collapse_triangulate = True
    if vgroup:
        d.vertex_group = vgroup; d.vertex_group_factor = 4.0; d.invert_vertex_group = True   # protect weighted verts
    bpy.context.view_layer.objects.active = o
    for ob in bpy.context.selected_objects: ob.select_set(False)
    o.select_set(True)
    if len(o.modifiers) > 1: bpy.ops.object.modifier_move_to_index(modifier="Dec", index=0)
    bpy.ops.object.modifier_apply(modifier="Dec"); o.select_set(False)


# --------------------------------------------------------------------------- build
DOCK_LIFT = 0.0173          # mower height on the charging base (plate top under the wheels, m)


def build(WD, color="black", lod=None, shell_thick=0.0025, apply_solidify=False, auto_smooth=None, dock=False):
    """lod: None for full resolution, or dict of decimation ratios per part."""
    M = make_materials(WD, color)
    objs = {}
    # shell
    d = np.load(os.path.join(WD, 'asm_shell.npz')); V = d['v']; F = d['f'].astype(np.int64)
    sh = mesh_obj('Shell', V, F)
    n = np.empty(len(sh.data.polygons) * 3); sh.data.polygons.foreach_get("normal", n); n = n.reshape(-1, 3)
    cz = np.empty(len(sh.data.polygons) * 3); sh.data.polygons.foreach_get("center", cz); cz = cz.reshape(-1, 3)
    if n[cz[:, 2] > np.percentile(cz[:, 2], 80), 2].mean() < 0: sh.data.flip_normals()
    sol = sh.modifiers.new("Solidify", 'SOLIDIFY'); sol.thickness = shell_thick; sol.offset = -1
    sol.use_even_offset = False; sol.thickness_clamp = 1.0; sol.use_rim = True; sol.use_quality_normals = True
    sh.data.materials.append(M['ShellPaint'])
    if apply_solidify and not lod:
        bpy.context.view_layer.objects.active = sh; sh.select_set(True)
        bpy.ops.object.modifier_apply(modifier="Solidify"); sh.select_set(False)
    objs['Shell'] = sh
    # middle (labelled)
    d = np.load(os.path.join(WD, 'asm_middle2_lab.npz')); V = d['v']; F = d['f'].astype(np.int64); lab = d['lab'].astype(np.int32)
    geo = json.load(open(os.path.join(WD, 'mid_tex_geom.json')))
    matidx = lab.copy(); matidx[lab == 2] = 0; matidx[lab == 4] = 3   # display/window drawn by decals
    mid = mesh_obj('Middle', V, F, mat_idx=matidx)
    for nm in MID_NAMES: mid.data.materials.append(M[nm])
    if lod:
        # protect material boundaries + small detailed regions from decimation
        E = np.sort(np.concatenate([F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]]), 1)
        key = E[:, 0] * len(V) + E[:, 1]; fl = np.tile(lab, 3)
        o_ = np.argsort(key); ks = key[o_]; fs = fl[o_]
        same = ks[1:] == ks[:-1]; diff = same & (fs[1:] != fs[:-1])
        bverts = np.unique(np.concatenate([E[o_][:-1][diff].ravel(), E[o_][1:][diff].ravel()]))
        w = np.zeros(len(V)); w[bverts] = 1.0
        fl = np.unique(F[(lab > 0) & (lab != 7)]); w[fl] = np.maximum(w[fl], 0.6)   # bumper decimates like the body
        if 'keep' in d.files: w[d['keep']] = 1.0             # smoothed band around the modelled LED lamps
        vg = mid.vertex_groups.new(name="keep")
        for val in (1.0, 0.6):
            idx = np.nonzero(np.isclose(w, val))[0].tolist()
            if idx: vg.add(idx, val, 'REPLACE')
    objs['Middle'] = mid
    # chassis
    d = np.load(os.path.join(WD, 'asm_chassis.npz')); ch = mesh_obj('Chassis', d['v'], d['f'])
    ch.data.materials.append(M['ChassisBlack']); objs['Chassis'] = ch
    # drive wheels (origin at axle centre)
    WG = json.load(open(os.path.join(WD, 'wheel_geom.json')))
    d = np.load(os.path.join(WD, 'asm_wheels.npz')); V = d['v']; F = d['f'].astype(np.int64)
    side = np.sign(V[F[:, 0], 1])
    for s, nm in [(1, 'L'), (-1, 'R')]:
        Vs, Fs = compact(V, F[side == s])
        g = WG[nm]; c = np.array(g['center']); ax = np.array(g['axle'])
        lab = wheel_labels(Vs, Fs, c, ax); lab[lab == 2] = 1   # ring drawn by decal
        o = mesh_obj(f'Wheel_{nm}', Vs, Fs, mat_idx=lab, origin=c)
        for mn in ('Rubber', 'WheelHub', 'CyanRing'): o.data.materials.append(M[mn])
        o["axis_world"] = ax.tolist(); objs[f'Wheel_{nm}'] = o
    # casters (origin at wheel centre; swivel pivot stored)
    d = np.load(os.path.join(WD, 'asm_casters.npz')); V = d['v']; F = d['f'].astype(np.int64)
    CP = caster_pivots(V)
    side = np.sign(V[F[:, 0], 1])
    for s, nm in [(1, 'L'), (-1, 'R')]:
        Vs, Fs = compact(V, F[side == s])
        c = CP[nm]['c']
        o = mesh_obj(f'Caster_{nm}', Vs, Fs, origin=c)
        o.data.materials.append(M['Rubber'])
        o["axis_world"] = CP[nm]['ax'].tolist()
        o["pivot_world"] = [float(CP[nm]['pivot'][0]), float(CP[nm]['pivot'][1]), float(c[2] + CP[nm]['r'])]
        objs[f'Caster_{nm}'] = o
    objs['_caster_info'] = CP
    # decals: smooth-edged textured overlays draped on the scanned surfaces
    for nm, par, mat in [('DisplayDecal', 'Middle', 'DisplayGlass'), ('WindowDecal', 'Middle', 'SensorGlass'),
                         ('RingDecal_L', 'Wheel_L', 'CyanRing'), ('RingDecal_R', 'Wheel_R', 'CyanRing')]:
        dd = np.load(os.path.join(WD, 'decal_' + nm.replace('Decal', '').replace('_', '').lower().replace('ringl', 'ring_L').replace('ringr', 'ring_R') + '.npz'))
        P = objs[par]
        Vd = dd['v'] - np.array(P.location[:])
        o = mesh_obj(nm, Vd, dd['f'])
        o.data.materials.append(M[mat])
        ul = o.data.uv_layers.new(name="UVMap"); ul.data.foreach_set("uv", dd['uv'][dd['f'].ravel()].astype(np.float32).ravel())
        o.parent = P
        objs[nm] = o
    # charging base (make_dock.py): placed for the docked pose, plate on the ground
    if dock and os.path.exists(os.path.join(WD, 'asm_dock.npz')):
        dd = np.load(os.path.join(WD, 'asm_dock.npz'))
        o = mesh_obj('Dock', dd['v'], dd['f'], mat_idx=dd['lab'])
        for mn in ('DockBody', 'DockPin'): o.data.materials.append(M[mn])
        objs['Dock'] = o
    # front LED lamps: body-coloured lip, dark gap, chrome bezel, textured lens (see make_led.py)
    for nm in ('L', 'R'):
        fp = os.path.join(WD, f'decal_led_{nm}.npz')
        if not os.path.exists(fp): continue
        dd = np.load(fp); P = objs['Middle']
        o = mesh_obj(f'LedDecal_{nm}', dd['v'] - np.array(P.location[:]), dd['f'], mat_idx=dd['mat'])
        for mn in ('MidBody', 'LedGap', 'LedBezel', 'LEDLens'): o.data.materials.append(M[mn])
        ul = o.data.uv_layers.new(name="UVMap"); ul.data.foreach_set("uv", dd['uv'][dd['f'].ravel()].astype(np.float32).ravel())
        o.parent = P
        objs[f'LedDecal_{nm}'] = o
    if lod:
        skin = None
        for k, o in objs.items():
            if k.startswith('_') or 'Decal' in k: continue
            r = lod.get(k.split('_')[0], 1.0)
            decimate(o, r, 'keep' if k == 'Middle' else None)
            if k in ('Middle', 'Chassis') and skin is not None:
                tuck_under(o, skin, 0.0012 if k == 'Middle' else 0.002)
            if k == 'Shell':                       # single skin decimated first, then thickened
                relax_flips(o)
                skin = surface_bvh(o)              # outer skin, for tucking the decimated parts below it
                bpy.context.view_layer.objects.active = o; o.select_set(True)
                bpy.ops.object.modifier_apply(modifier="Solidify"); o.select_set(False)
    if auto_smooth is None: auto_smooth = bool(lod)
    if auto_smooth:
        for k, o in objs.items():
            if k.startswith('_') or 'Decal' in k: continue
            smooth_by_angle(o, 35 if k != 'Shell' else 60)
    return objs, M
