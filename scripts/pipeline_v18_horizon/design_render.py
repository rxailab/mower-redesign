# Render a redesign concept. usage: blender -b --factory-startup -P design_render.py -- PARAMS.json SHELL.npz OUTPREFIX views res samples
import bpy, sys, os, json, math, numpy as np, mathutils
a = sys.argv[sys.argv.index('--') + 1:]; PJ, NPZ, OUT, VIEWS, RES, SPP = a[0], a[1], a[2], a[3].split(','), int(a[4]), int(a[5])
HERE = os.path.dirname(os.path.abspath(__file__)); P = json.load(open(PJ)); col = P.get('colors', {})
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]; return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
FIN = {'gloss': (0.28, 0, 0.8), 'metallic': (0.34, 0.6, 1.0), 'satin': (0.5, 0, 0.1), 'matte': (0.72, 0, 0),
       'satincoat': (0.42, 0.0, 0.45), 'smoke': (0.16, 0.0, 0.8), 'texture': (0.62, 0.0, 0.0),      # [orbit] shell, smoked lens, moulded-in texture
       'satinmetal': (0.36, 0.55, 0.35), 'glossdark': (0.12, 0.0, 1.0), 'matte_tex': (0.78, 0.0, 0.0)}      # [hz] satin metallic canopy, smoked gloss spine / lid, coarse-grain base
def mat(name, hexc, finish='satin', emit=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']; r, me, cc = FIN[finish]
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = r; b.inputs['Metallic'].default_value = me
    b.inputs['Coat Weight'].default_value = cc; b.inputs['Coat Roughness'].default_value = 0.06
    if emit: b.inputs['Emission Color'].default_value = (*lin(hexc), 1); b.inputs['Emission Strength'].default_value = emit
    return m
def obj(name, V, F, mats, midx=None, smooth_deg=40, normals=None):
    me = bpy.data.meshes.new(name); me.vertices.add(len(V)); me.vertices.foreach_set('co', np.asarray(V, np.float32).ravel())
    F = np.asarray(F, np.int32); me.loops.add(F.size); me.loops.foreach_set('vertex_index', F.ravel()); me.polygons.add(len(F))
    me.polygons.foreach_set('loop_start', np.arange(0, F.size, 3, dtype=np.int32)); me.polygons.foreach_set('loop_total', np.full(len(F), 3, np.int32))
    if midx is not None: me.polygons.foreach_set('material_index', np.asarray(midx, np.int32))
    me.polygons.foreach_set('use_smooth', np.ones(len(F), bool)); me.update()
    if normals is None or me.validate(): me.validate() if normals is None else print('WARNING: mesh', name, 'was changed by validate; custom normals may not fit')
    o = bpy.data.objects.new(name, me); sc.collection.objects.link(o)
    for m in mats: me.materials.append(m)
    bpy.context.view_layer.objects.active = o; o.select_set(True)
    if normals is not None:
        me.normals_split_custom_set(np.asarray(normals, np.float64).reshape(-1, 3).tolist()); o.select_set(False); return o
    if name in ('LidKit', 'LidWell', 'HingePod', 'Camera', 'Struts', 'Mount', 'CoverKit', 'StopKey'):      # [fix3-stop] StopKey
        bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT'); bpy.ops.mesh.remove_doubles(threshold=1e-6)
        bpy.ops.mesh.normals_make_consistent(inside=False); bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth_deg)); o.select_set(False)
    return o
bpy.ops.import_scene.gltf(filepath=os.path.join(HERE, 'base.glb'))          # chassis, wheels, casters (web model)
ring = col.get('ring', '#2FC6DC')
for m in bpy.data.materials:
    if m.name.startswith('CyanRing'):
        b = m.node_tree.nodes['Principled BSDF']; b.inputs['Base Color'].default_value = (*lin(ring), 1)
        b.inputs['Emission Color'].default_value = (*lin(ring), 1); b.inputs['Emission Strength'].default_value = 1.4
ch = bpy.data.objects.get('Chassis')                                   # no mudguards: drop the chassis struts that carried them
sys.path.insert(0, HERE); import strutskit as _sk; _d0 = np.load(NPZ); _SKS = _sk.from_npz(_d0) or _sk.spec(P)      # [fix3-struts] the cut spec (npz first, then params)
_SKC = (_d0['struts_cap_v'], _d0['struts_cap_f'], _d0['struts_cap_cut']) if 'struts_cap_v' in _d0.files else None; del _d0      # [fix3-struts] the caps (generator: shapely)
if ch and _SKS['mode'] == 'cut':      # [fix3-struts] the real modified chassis: capped plane cuts of the old mudguard pin + front hook only; rim flange and drive housings stay
    _hl = None      # [fix3-struts]
    if os.environ.get('STRUTS_HL'):      # [fix3-struts] review renders: the cut faces in a signal colour
        ch.data.materials.append(mat('StrutCut', os.environ['STRUTS_HL'] if os.environ['STRUTS_HL'].startswith('#') else '#FF6A2B', 'matte')); _hl = len(ch.data.materials) - 1      # [fix3-struts]
    print('struts cut:', json.dumps(_sk.cut_bmesh(ch, _SKS, caps=_SKC, cap_mat=_hl)))      # [fix3-struts]
elif ch:      # [fix3-struts] legacy (npz / params without 'struts'): the f2 vertex deletion
    import bmesh
    bm = bmesh.new(); bm.from_mesh(ch.data); mw = ch.matrix_world
    PO = json.load(open(os.path.join(HERE, 'mounts.json')))['posts'] if os.path.exists(os.path.join(HERE, 'mounts.json')) else []      # the screw posts on the tub rim stay
    kill = [v for v in bm.verts if (lambda w: abs(w.y) > 0.160 and w.z > 0.135 and -0.06 < w.x < 0.29 and all((w.x - p_[0]) ** 2 + (w.y - p_[1]) ** 2 > 0.0068 ** 2 for p_ in PO))(mw @ v.co)]
    bmesh.ops.delete(bm, geom=kill, context='VERTS'); bm.to_mesh(ch.data); bm.free()
_PS = (P.get('ports3') or {}).get('pocket_standin')      # [fix4-front2] A4: the charger contact pockets of the chassis scan are coarse / crumpled and show through the
if ch and _PS:      # [fix4-front2] windows in headlit close-ups. Render stand-in of the moulded chassis (like the pin stand-ins): the scan faces in the pocket region are
    import bmesh      # [fix4-front2] replaced by fitted planes - V flanks (portcheck / pocketfit: mouth y at x 302, flank angle to x), floor, ceiling, chassis face x 302.5
    _ps = _PS if isinstance(_PS, dict) else {}      # [fix4-front2]
    _xf, _zf, _zc = _ps.get('face_x', 0.3025), _ps.get('floor_z', 0.1425), _ps.get('ceil_z', 0.1672)      # [fix4-front2]
    _pk = _ps.get('pockets', [[0.00886, 49.8, 0.05049, 53.2], [-0.00493, 49.9, -0.04546, 52.5]])      # [fix4-front2] [y inner, deg, y outer, deg] at x 302 (fitted, z 152.6)
    _ylo, _yhi, _zlo, _zhi = -0.056, 0.057, 0.1390, 0.1680      # [fix4-front2]
    bm = bmesh.new(); bm.from_mesh(ch.data); mw = ch.matrix_world      # [fix4-front2]
    _kill = [f_ for f_ in bm.faces if (lambda c_: _ylo < c_.y < _yhi and ((0.279 < c_.x < _xf - 0.0002 and _zlo < c_.z < _zhi) or (_xf - 0.0002 <= c_.x < 0.304 and _zf < c_.z < _zhi)))(mw @ f_.calc_center_median())]      # [fix4-front2]
    bmesh.ops.delete(bm, geom=_kill, context='FACES'); bm.to_mesh(ch.data); bm.free()      # [fix4-front2]
    _V, _F = [], []      # [fix4-front2]
    def _quad(a_, b_, c_, d_):
        n_ = len(_V); _V.extend([a_, b_, c_, d_]); _F.extend([(n_, n_ + 1, n_ + 2), (n_, n_ + 2, n_ + 3)])
    _mouths = []
    for yi_, ai_, yo_, ao_ in _pk:      # [fix4-front2] plan polyline mouth(inner) -> apex (R 2 round) -> mouth(outer), flanks extended to the face plane
        s_ = 1.0 if yo_ > yi_ else -1.0; ti_, to_ = math.tan(math.radians(ai_)), math.tan(math.radians(ao_))
        t_ = (abs(yo_ - yi_)) / (ti_ + to_); xa_ = 0.302 - t_; ya_ = yi_ + s_ * ti_ * t_
        yi2_ = yi_ - s_ * ti_ * (_xf - 0.302); yo2_ = yo_ + s_ * to_ * (_xf - 0.302); _mouths.append(sorted((yi2_, yo2_)))
        di_ = np.array([_xf - xa_, yi2_ - ya_]); di_ /= np.linalg.norm(di_); do_ = np.array([_xf - xa_, yo2_ - ya_]); do_ /= np.linalg.norm(do_)
        bis_ = di_ + do_; bis_ /= np.linalg.norm(bis_); hf_ = math.acos(float(np.clip(di_ @ do_, -1, 1))) / 2; rr_ = 0.002; cc_ = np.array([xa_, ya_]) + bis_ * rr_ / math.sin(hf_)
        a0_ = math.atan2(*(np.array([xa_, ya_]) + di_ * rr_ / math.tan(hf_) - cc_)[::-1]); a1_ = math.atan2(*(np.array([xa_, ya_]) + do_ * rr_ / math.tan(hf_) - cc_)[::-1])
        if a1_ - a0_ > math.pi: a1_ -= 2 * math.pi
        if a0_ - a1_ > math.pi: a1_ += 2 * math.pi
        arc_ = [cc_ + rr_ * np.array([math.cos(q_), math.sin(q_)]) for q_ in np.linspace(a0_, a1_, 9)]
        pl_ = [np.array([_xf, yi2_])] + arc_ + [np.array([_xf, yo2_])]
        for p_, q_ in zip(pl_[:-1], pl_[1:]): _quad((p_[0], p_[1], _zf), (q_[0], q_[1], _zf), (q_[0], q_[1], _zc), (p_[0], p_[1], _zc))      # flanks + apex
        m_ = 0.5 * (pl_[0] + pl_[-1])
        for p_, q_ in zip(pl_[:-1], pl_[1:]):      # floor / ceiling fans from the mouth mid-point (the V is convex)
            n_ = len(_V); _V.extend([(m_[0], m_[1], _zf), (p_[0], p_[1], _zf), (q_[0], q_[1], _zf), (m_[0], m_[1], _zc), (p_[0], p_[1], _zc), (q_[0], q_[1], _zc)]); _F.extend([(n_, n_ + 1, n_ + 2), (n_ + 3, n_ + 5, n_ + 4)])
    _ms = sorted(_mouths); _ys = [_ylo] + [y_ for m_ in _ms for y_ in m_] + [_yhi]
    _quad((_xf, _ylo, _zc), (_xf, _yhi, _zc), (_xf, _yhi, _zhi), (_xf, _ylo, _zhi))      # face plane round the mouths
    for y0_, y1_ in zip(_ys[0::2], _ys[1::2]): _quad((_xf, y0_, _zf), (_xf, y1_, _zf), (_xf, y1_, _zc), (_xf, y0_, _zc))
    _pso = obj('PocketStandin', np.array(_V), np.array(_F), [ch.data.materials[0] if ch.data.materials else mat('ChassisDark', '#1E2023', 'satin')], smooth_deg=20)      # [fix4-front2]
    print('pocket stand-in: %d faces of the chassis scan replaced, mouths (mm) %s' % (len(_kill), [[round(y_ * 1000, 2) for y_ in m_] for m_ in _ms]))      # [fix4-front2]
M = {'body': mat('Body', col.get('body', '#C9CDD2'), col.get('body_finish', 'metallic')),
     'roof': mat('RoofPanel', col.get('roof', '#1B1D21'), col.get('roof_finish', 'gloss')),
     'visor': mat('Visor', col.get('visor', '#07080A'), col.get('visor_finish', 'gloss')), 'light': mat('Light', col.get('light', '#DFF4FF'), 'gloss', 6.0),
     'lid': mat('Lid', col.get('lid', col.get('body', '#C9CDD2')), col.get('lid_finish', col.get('body_finish', 'metallic'))),
     'accent': mat('Accent', col.get('accent', '#E8552F'), 'satin'), 'seam': mat('Seam', '#050505', 'matte'),
     'skirt': mat('Skirt', col.get('skirt', '#2A2C30'), 'satin'), 'dark': mat('Dark', '#0B0B0C', 'matte'),
     'stop': mat('Stop', col.get('stop', '#E8552F'), col.get('stop_finish', 'satin')), 'glass': mat('Glass', '#0A0B0D', 'gloss'),      # [fix3-stop] colors.stop (STOP red)  [fix4-stop2] colors.stop_finish (A6: gloss 2K cap)
     'kdark': mat('KitDark', col.get('kit', col.get('inner', '#33363B')), col.get('kit_finish', 'satin')), 'rubber': mat('Rubber', '#070708', 'matte'), 'steel': mat('Steel', '#C9CDD2', 'metallic'),      # [v18b] KitDark: lid underside, ribs, edge walls, well = the one dark interior colour
     'arm': mat('HingeArm', col.get('arm', '#595D65'), col.get('arm_finish', 'metallic')), 'liner': mat('Liner', col.get('liner', '#6E727A'), 'satin'), 'strut': mat('StrutBody', '#0E0F11', 'gloss'),
     'bezel': mat('LensBezel', '#7C828A', 'metallic'), 'lens': mat('LensGlass', '#05070D', 'gloss')}
M['core'] = mat('Core', col.get('core', '#2A2C30'), col.get('core_finish', 'texture'))      # [orbit] dark core: moulded-in fine texture
if col.get('core_finish', 'texture') == 'texture':
    nt_ = M['core'].node_tree; tx_ = nt_.nodes.new('ShaderNodeTexNoise'); tx_.inputs['Scale'].default_value = float(col.get('core_tex_scale', 2600.0)); tx_.inputs['Detail'].default_value = 2.0
    co_ = nt_.nodes.new('ShaderNodeTexCoord'); nt_.links.new(co_.outputs['Object'], tx_.inputs['Vector'])
    bu_ = nt_.nodes.new('ShaderNodeBump'); bu_.inputs['Strength'].default_value = float(col.get('core_tex_strength', 0.12)); bu_.inputs['Distance'].default_value = 0.0002
    nt_.links.new(tx_.outputs['Fac'], bu_.inputs['Height']); nt_.links.new(bu_.outputs['Normal'], nt_.nodes['Principled BSDF'].inputs['Normal'])
M['inner'] = mat('Inner', col.get('inner', '#3C3F45'), 'satin'); M['screw'] = mat('Screw', '#8E9299', 'metallic'); M['screw'].node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = 0.3; M['screw'].node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value = 1.0
def two_sided(m, hexc='#2E3035'):          # the body is a single skin: seen from inside it is unpainted dark plastic
    nt = m.node_tree; b = nt.nodes['Principled BSDF']; d_ = nt.nodes.new('ShaderNodeBsdfPrincipled'); d_.inputs['Base Color'].default_value = (*lin(hexc), 1); d_.inputs['Roughness'].default_value = 0.55
    if os.environ.get('BACKDEBUG'):          # [v18c] debug: back faces seen directly by the camera are flat magenta (with a closed wall every one is a hole)
        em_ = nt.nodes.new('ShaderNodeEmission'); em_.inputs['Color'].default_value = (1, 0, 1, 1); em_.inputs['Strength'].default_value = 3.0
        lp_ = nt.nodes.new('ShaderNodeLightPath'); m2_ = nt.nodes.new('ShaderNodeMixShader'); nt.links.new(lp_.outputs['Is Camera Ray'], m2_.inputs['Fac'])
        nt.links.new(d_.outputs['BSDF'], m2_.inputs[1]); nt.links.new(em_.outputs['Emission'], m2_.inputs[2]); d_ = type('S', (), {'outputs': {'BSDF': m2_.outputs['Shader']}})()
    g_ = nt.nodes.new('ShaderNodeNewGeometry'); mx = nt.nodes.new('ShaderNodeMixShader'); nt.links.new(g_.outputs['Backfacing'], mx.inputs['Fac'])
    nt.links.new(b.outputs['BSDF'], mx.inputs[1]); nt.links.new(d_.outputs['BSDF'], mx.inputs[2]); nt.links.new(mx.outputs['Shader'], nt.nodes['Material Output'].inputs['Surface'])
d = np.load(NPZ); SIDE_ = d['side'] if 'side' in d.files else None; HASWALL = SIDE_ is not None and bool((SIDE_ == 1).any())      # [v18c] real wall: 0 outer, 1 inner, 2 rim
if not HASWALL or os.environ.get('BACKDEBUG'):                    # single skin: fake the inside with a back-face shader (not needed once the solids are closed)
    for k_ in ('body', 'roof', 'visor', 'accent', 'core') + (('lid', 'light', 'glass') if os.environ.get('BACKDEBUG') else ()): two_sided(M[k_])
M['steel'].node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = 0.22; M['steel'].node_tree.nodes['Principled BSDF'].inputs['Metallic'].default_value = 1.0
M['lens'].node_tree.nodes['Principled BSDF'].inputs['Roughness'].default_value = 0.05; M['lens'].node_tree.nodes['Principled BSDF'].inputs['Coat Weight'].default_value = 1.0
k = np.load(os.path.join(HERE, 'keep.npz')); kl = k['lab']
kc = k['v'][k['f']].mean(1); kk = ~((kl == 6) & (kc[:, 0] > 0.27))
if P.get('nose'): kk &= ~((kl == 0) & (kc[:, 0] > 0.255))
if P.get('skirt'): kk &= (kl != 0)                               # one-piece shell: the scanned skirt is gone, only the front bumper is kept      # old shell's front rim: replaced by the new nose, which sits on the bumper        # charger-opening internals are replaced by clean sleeves
sys.path.insert(0, HERE); import tail as _tail; kv = k['v'].copy(); kv[:, 1] = _tail.make(P)(kv[:, 0], kv[:, 1])
obj('Skirt', kv, k['f'][kk], [M['skirt'], M['dark']], np.where((kl == 7) | (kl == 6), 1, 0)[kk])
M['glow'] = mat('Glow', col.get('visor', '#0A0B0D'), col.get('visor_finish', 'smoke'))      # [orbit] diffused light behind the smoked lens round each light line
_gb = M['glow'].node_tree.nodes['Principled BSDF']; _gb.inputs['Emission Color'].default_value = (*lin(col.get('light', '#F2FBFF')), 1); _gb.inputs['Emission Strength'].default_value = float(col.get('glow', 0.5))
M['ring'] = mat('SensorRing', col.get('ring_sensor', '#5E646C'), 'metallic')      # [v18] sensor rims seen through the smoked lens
M['inner_skin'] = mat('InnerSkin', col.get('kit', col.get('inner', '#34373C')), col.get('kit_finish', 'satin'))      # [v18c] inside of the walls + bare edge faces = the one dark interior colour
if HASWALL and os.environ.get('BACKDEBUG'): two_sided(M['inner_skin'])

if os.environ.get('ZEBRA'):      # [review] reflection-line (zebra) shader on the visible plastics: stripes of the reflection vector
    _axis = os.environ.get('ZEBRA_AXIS', 'Z'); _fq = float(os.environ.get('ZEBRA', '14'))
    for _k in ('body', 'lid', 'core', 'visor'):
        _m = M[_k]; _nt = _m.node_tree
        for _n in list(_nt.nodes):
            if _n.type != 'OUTPUT_MATERIAL': _nt.nodes.remove(_n)
        _tc = _nt.nodes.new('ShaderNodeTexCoord'); _sx = _nt.nodes.new('ShaderNodeSeparateXYZ'); _nt.links.new(_tc.outputs['Reflection'], _sx.inputs[0])
        _mu = _nt.nodes.new('ShaderNodeMath'); _mu.operation = 'MULTIPLY'; _mu.inputs[1].default_value = _fq; _nt.links.new(_sx.outputs[_axis], _mu.inputs[0])
        _fr = _nt.nodes.new('ShaderNodeMath'); _fr.operation = 'FRACT'; _nt.links.new(_mu.outputs[0], _fr.inputs[0])
        _gt = _nt.nodes.new('ShaderNodeMath'); _gt.operation = 'GREATER_THAN'; _gt.inputs[1].default_value = 0.5; _nt.links.new(_fr.outputs[0], _gt.inputs[0])
        _sh = _nt.nodes.new('ShaderNodeMath'); _sh.operation = 'MULTIPLY_ADD'; _sh.inputs[1].default_value = 0.85; _sh.inputs[2].default_value = 0.05; _nt.links.new(_gt.outputs[0], _sh.inputs[0])
        _em = _nt.nodes.new('ShaderNodeEmission'); _nt.links.new(_sh.outputs[0], _em.inputs['Strength']); _em.inputs['Color'].default_value = (1, 1, 1, 1)
        _nt.links.new(_em.outputs[0], _nt.nodes['Material Output'].inputs['Surface'])
order = ['body', 'roof', 'visor', 'light', 'lid', 'accent', 'glass', 'core', 'glow', 'ring', 'inner_skin']; M['glass'] = M['roof'] if False else M['glass']
if col.get('base'):      # [hz] slot 11: the textured base below K2 (coarse moulded-in grain)
    M['base'] = mat('Base', col['base'], col.get('base_finish', 'matte_tex')); order.append('base')
    nt_ = M['base'].node_tree; tx_ = nt_.nodes.new('ShaderNodeTexNoise'); tx_.inputs['Scale'].default_value = float(col.get('base_tex_scale', 1400.0)); tx_.inputs['Detail'].default_value = 3.0
    co_ = nt_.nodes.new('ShaderNodeTexCoord'); nt_.links.new(co_.outputs['Object'], tx_.inputs['Vector'])
    bu_ = nt_.nodes.new('ShaderNodeBump'); bu_.inputs['Strength'].default_value = float(col.get('base_tex_strength', 0.25)); bu_.inputs['Distance'].default_value = 0.0003
    nt_.links.new(tx_.outputs['Fac'], bu_.inputs['Height']); nt_.links.new(bu_.outputs['Normal'], nt_.nodes['Principled BSDF'].inputs['Normal'])
lab = d['lab']; LIDOPEN = 'lidopen' in VIEWS
midx_ = np.where(SIDE_ == 1, 10, lab) if HASWALL else lab                       # [v18c] inner skin -> slot 10; bare rim faces already carry lab 10
part = d['part'] if 'part' in d.files else np.zeros(len(lab), np.uint8); HASCOVER = bool((part == 1).any())      # 1 = snap-on top cover
def sub_(m_):
    f_ = d['f'][m_]; u_ = np.unique(f_); rm_ = -np.ones(len(d['v']), np.int64); rm_[u_] = np.arange(len(u_)); return d['v'][u_], rm_[f_]
sb_ = (lab != 4) & (part == 0); v_, f_ = sub_(sb_); shell = obj('NewShell', v_, f_, [M[n] for n in order], midx_[sb_], normals=d['ln'][sb_])
if HASCOVER:
    sc_ = (lab != 4) & (part == 1); v_, f_ = sub_(sc_); obj('TopCover', v_, f_, [M[n] for n in order], midx_[sc_], normals=d['ln'][sc_])
    if 'ckit_v' in d.files and len(d['ckit_f']): obj('CoverKit', d['ckit_v'], d['ckit_f'], [M['inner'], M['inner']], d['ckit_lab'], smooth_deg=35)
lidF = d['f'][lab == 4]; u = np.unique(lidF); rm = -np.ones(len(d['v']), np.int64); rm[u] = np.arange(len(u))
M['legend'] = mat('LegendW', '#F4F4F2', 'satin')
lid = obj('Lid', d['v'][u], rm[lidF], [M['lid'], M['stop'], M['legend'], M['light']], d['lid_paint'][lab == 4] if 'lid_paint' in d.files else None, normals=d['ln'][lab == 4])      # [fix1] E4 STOP mark; [fix2] STOP pad, stop square, status halo
KITM = [M['kdark'], M['rubber'], M['steel'], M['arm'], M['liner'], M['strut'], M['lid']]
KITM += [mat('SectionMetal', '#C9CED6', 'satin'), mat('SectionDark', '#E2703F', 'matte'), mat('SectionLid', '#3F8FE0', 'matte'), mat('SectionSpring', '#5E6A78', 'satin')]      # [fix4-hinge2] section.py caps of the kit solids (7 metal, 8 dark mouldings / TPE = body colour, 9 lid = cover colour, 10 spiral spring)
kitm = obj('LidKit', d['kitm_v'], d['kitm_f'], KITM, d['kitm_lab'], smooth_deg=35)
stopk = None
if 'stop_v' in d.files and len(d['stop_f']):      # [fix3-stop] raised STOP key on the lid (stopkit.py): cap + STOP inlay, halo boss, carrier, arms, springs
    M['halo'] = mat('Halo', col.get('halo', '#F2FBFF'), 'satin', float(col.get('halo_emit', 2.0)))      # [fix3-stop]
    sv_ = d['stop_v'].copy()      # [fix3-stop] STOPSTROKE (m): the cap pressed - carrier on the parallel arms, rod + tip held once the tip lands on the key, springs / boot squashed
    if os.environ.get('STOPSTROKE') and 'stop_kin' in d.files:
        s_ = float(os.environ['STOPSTROKE']); xP_, zu_, zl_, L_, g_ = [float(t) for t in d['stop_kin'][[0, 1, 2, 3, -1]]]; xk_ = float(d['stop_axis'][0]); th_ = math.asin(min(s_ / L_, 1.0))
        pv_ = np.zeros(len(sv_), np.int16); pv_[d['stop_f'].ravel()] = np.repeat(d['stop_part'], 3); dx_, dz_ = -L_ * (1 - math.cos(th_)), -L_ * math.sin(th_)
        mv_ = np.isin(pv_, [61, 63]) | ((pv_ == 69) & (np.abs(sv_[:, 0] - xk_) < 0.004)); sv_[mv_, 0] += dx_; sv_[mv_, 2] += dz_
        sr_ = min(s_, g_); sv_[np.isin(pv_, [65, 67]), 2] -= sr_
        for pc_, top_ in ((66, True), (64, True)):
            m_ = pv_ == pc_
            if m_.any(): z_ = sv_[m_, 2]; a_, b_ = z_.min(), z_.max(); w_ = (z_ - a_) / max(b_ - a_, 1e-9); sv_[m_, 2] = z_ + w_ * dz_ - (1 - w_) * (sr_ if pc_ == 66 else 0.0); sv_[m_, 0] += w_ * dx_
        ma_ = pv_ == 68; zc_ = np.where(sv_[ma_, 2] > 0.5 * (zu_ + zl_), zu_, zl_); rx_ = sv_[ma_, 0] - xP_; rz_ = sv_[ma_, 2] - zc_
        sv_[ma_, 0] = xP_ + rx_ * math.cos(th_) + rz_ * math.sin(th_); sv_[ma_, 2] = zc_ - rx_ * math.sin(th_) + rz_ * math.cos(th_)
    stopk = obj('StopKey', sv_, d['stop_f'], [M['stop'], M['legend'], M['kdark'], M['rubber'], M['steel'], M['arm'], M['halo'], M['lid']], d['stop_lab'], smooth_deg=35)      # [fix3-stop]
if 'kitf_part' in d.files and (d['kitf_part'] == 23).any():          # [v18b] the hinge pods are their own object (can be hidden for mechanism views)
    pm_ = d['kitf_part'] == 23; pn_ = (d['kitf_part'] == 42) if 'lid_pinion' in d.files else np.zeros(len(pm_), bool)      # [fix4-hinge2] the pinion: own object, turns with the lid (set_lid)
    for nm_, m_ in (('LidWell', ~pm_ & ~pn_), ('HingePod', pm_)) + ((('HingePinion', pn_),) if pn_.any() else ()):      # [fix4-hinge2]
        u_ = np.unique(d['kitf_f'][m_]); r_ = -np.ones(len(d['kitf_v']), np.int64); r_[u_] = np.arange(len(u_)); obj(nm_, d['kitf_v'][u_], r_[d['kitf_f'][m_]], KITM, d['kitf_lab'][m_], smooth_deg=35)
else: obj('LidWell', d['kitf_v'], d['kitf_f'], KITM, d['kitf_lab'], smooth_deg=35)
if 'cam_v' in d.files: obj('Camera', d['cam_v'], d['cam_f'], [M['bezel'], M['dark'], M['lens'], M['light']], d['cam_lab'], smooth_deg=50)
if 'plinth_v' in d.files: obj('Plinth', d['plinth_v'], d['plinth_f'], [M['skirt']], smooth_deg=30)
if 'mount_v' in d.files and len(d['mount_f']): obj('Mount', d['mount_v'], d['mount_f'], [M['inner'], M['dark'], M['steel'], M['screw']], d['mount_lab'], smooth_deg=35)
if 'sleeve_v' in d.files: obj('PortSleeves', d['sleeve_v'], d['sleeve_f'], [M['dark']], smooth_deg=30)
if 'cap_v' in d.files and len(d['cap_f']):      # [v18c] section caps written by section.py: the plastic cross-section of the body (warm) and the cover (cool)
    obj('Section', d['cap_v'], d['cap_f'], [mat('SectionBody', '#E2703F', 'matte'), mat('SectionCover', '#3F8FE0', 'matte')], d['cap_part'], smooth_deg=1)
import meshkit
c = np.load(os.path.join(HERE, 'control.npz')); cl = c['lab']
if 'ctlsi_v' in d.files:      # [fix1] D6 / F10: clean stand-in over the scanned controls (smoked display window, STOP key cap with a flat legend)
    csi_ = obj('ControlsSI', d['ctlsi_v'], d['ctlsi_f'], [M['glass'], M['stop']], d['ctlsi_lab'], smooth_deg=35)
    for p_ in csi_.data.polygons:      # [fix2] F-S2: the planar tops (long fan triangles) render flat; only the edge rolls are smooth
        if p_.normal.z > 0.995: p_.use_smooth = False
    zc0_ = float(np.load(os.path.join(HERE, 'control.npz'))['v'][:, 2].min())      # [fix2] F-N2: the control tower (chassis side) under the panel, so it never floats
    cvv_ = np.load(os.path.join(HERE, 'control.npz'))['v']; tw_ = meshkit.box(((cvv_[:, 0].min() + cvv_[:, 0].max()) / 2, 0.0, zc0_ - 0.004), (np.ptp(cvv_[:, 0]) - 0.012, np.ptp(cvv_[:, 1]) - 0.012, 0.0085))
    obj('CtlTower', tw_[0], tw_[1], [M['dark']], smooth_deg=20)
    kx_, ky_, ka_, kb_, kw_, kh_ = [float(t_) for t_ in d['ctlsi_key']]
    bpy.ops.object.text_add(); tx_ = bpy.context.active_object; tx_.name = 'StopLegend'; tx_.data.body = 'STOP'; tx_.data.align_x = 'CENTER'; tx_.data.align_y = 'CENTER'
    tx_.data.size = 0.6 * kw_; tx_.data.extrude = 0.0; tx_.data.offset_x = 0.0
    _fb = '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'      # [fix4-stop2] A7: one STOP typography (the cap inlay is Liberation Sans Bold, stop_glyphs.json)
    if os.path.exists(_fb): tx_.data.font = bpy.data.fonts.load(_fb)      # [fix4-stop2]
    tx_.data.materials.append(mat('Legend', '#F4F4F2', 'satin'))
    tx_.rotation_mode = 'ZYX'; tx_.rotation_euler = (0.0, math.atan(-kb_), -math.pi / 2); tx_.location = (kx_, ky_, ka_ + kb_ * kx_ + 0.00006)      # reads from behind the mower, as the key
obj('Controls', c['v'], c['f'], [M['dark'], M['stop'], M['glass']], np.where(cl == 1, 1, np.where(cl == 2, 2, 0)))
if 'hzl_v' in d.files and len(d['hzl_f']):      # [hz] light line / pupil / dot lenses (stage-1 stand-in): lit strip, smoked windows, dark edges
    obj('HzLight', d['hzl_v'], d['hzl_f'], [M['light'], M['lens'], mat('HzLensEdge', '#0C0D10', 'satin')], d['hzl_lab'], smooth_deg=40)
if 'lights_kit_v' in d.files:      # [fix3-lights] separate smoked lens, sealed lamp modules, bosses, harness, halo feed (lightkit.py)
    LS_, LL_ = d['lights_lens_side'], d['lights_lens_lab']; lo_ = np.select([LL_ == 3, LL_ == 6, LL_ == 9], [1, 2, 3], 0); ro_ = np.where(LL_ == 10, 5, lo_)      # rim: the roll keeps the outer paint      # [fix3-lights]
    li_ = np.select([LL_ == 3, LL_ == 6, LL_ == 9], [1, 2, 3], 4)                               # inner face: windows clear, rings silver print, the rest black dead-front print      # [fix3-lights]
    M['lens_print'] = mat('LensPrint', '#08090B', 'matte'); M['lens_edge'] = mat('LensEdge', '#0C0D10', 'satin')      # [fix3-lights]
    if len(d['lights_lens_f']): obj('LightLens', d['lights_lens_v'], d['lights_lens_f'], [M['visor'], M['light'], M['glass'], M['ring'], M['lens_print'], M['lens_edge']], np.where(LS_ == 0, lo_, np.where(LS_ == 1, li_, ro_)),      # [fix3-lights]
        normals=d['lights_lens_vn'][d['lights_lens_f']] if 'lights_lens_vn' in d.files else None)      # [fix3-lights]
    LKM_ = [mat('LampHousing', '#17181B', 'satin'), mat('LightGuide', col.get('light', '#DFF4FF'), 'gloss', 1.2), mat('Reflector', '#E9E9E4', 'satin'), mat('PCB', '#1E3B2C', 'satin'),      # [fix3-lights]
            mat('LED', '#F6EBC0', 'gloss', 4.0), mat('Foam', '#0B0B0B', 'matte'), M['steel'], mat('Sensor', '#0A0B0F', 'gloss'), M['inner_skin'], mat('Connector', '#3B3E44', 'satin'),      # [fix3-lights]
            mat('Cable', '#111214', 'satin'), mat('TPE', '#0D0D0E', 'matte'), mat('HaloFeed', col.get('light', '#DFF4FF'), 'gloss', 2.0)]      # [fix3-lights]
    KN_ = [str(k_) for k_ in d['lights_kinds']]; kk_ = d['lights_kit_kind']; ke_ = kk_ == KN_.index('emitter')      # [fix3-lights]
    for nm_, m_ in (('LightKit', ~ke_), ('HaloEmitter', ke_)):      # [fix3-lights]
        if m_.any():      # [fix3-lights]
            u_ = np.unique(d['lights_kit_f'][m_]); r_ = -np.ones(len(d['lights_kit_v']), np.int64); r_[u_] = np.arange(len(u_))      # [fix3-lights]
            obj(nm_, d['lights_kit_v'][u_], r_[d['lights_kit_f'][m_]], LKM_, d['lights_kit_lab'][m_], smooth_deg=35)      # [fix3-lights]
    if len(d['lights_harn_f']): obj('Harness', d['lights_harn_v'], d['lights_harn_f'], LKM_, d['lights_harn_lab'], smooth_deg=50)      # [fix3-lights]
    if len(d['lights_halo_f']): obj('LightHalo', d['lights_halo_v'], d['lights_halo_f'], LKM_, d['lights_halo_lab'], smooth_deg=35)      # [fix3-lights]
    if 'lights_cap_v' in d.files and len(d['lights_cap_f']):      # section caps of the kit (section.py): the part's own colour, the lens cap smoked blue-grey      # [fix3-lights]
        LKC_ = [mat('Sec' + n_, h_, 'matte') for n_, h_ in (('Housing', '#80868F'), ('Guide', '#BFE8FF'), ('Reflector', '#F4F4F0'), ('PCB', '#2F8052'), ('LED', '#F6EBC0'), ('Foam', '#6B5A48'),      # [fix3-lights]
                ('Metal', '#C9CDD2'), ('Sensor', '#353A44'), ('Boss', '#E2703F'), ('Conn', '#A3A9B1'), ('Cable', '#4A4A4A'), ('TPE', '#5E5850'), ('Clear', '#D8F0FF'), ('Lens', '#7A68A0'))]      # [fix3-lights]
        cl_ = d['lights_cap_lab'].astype(np.int64); hc_ = cl_ == 98; cl_ = np.where(cl_ == 99, len(LKC_) - 1, np.where(hc_, 12, cl_))      # [fix3-lights] [fix3-int] 98 = halo feed cap
        for nm_, m_ in (('LightCaps', ~hc_), ('LightHaloCap', hc_)):      # [fix3-int] the halo feed's cap is on the lid (moves in set_lid)
            if m_.any(): u_ = np.unique(d['lights_cap_f'][m_]); r_ = -np.ones(len(d['lights_cap_v']), np.int64); r_[u_] = np.arange(len(u_)); obj(nm_, d['lights_cap_v'][u_], r_[d['lights_cap_f'][m_]], LKC_, cl_[m_], smooth_deg=1)      # [fix3-int]
# cyclorama, lights, camera (same studio as the product renders)
R = 0.6; pts = [(y, 0.0) for y in np.linspace(-4, 1.0, 10)] + [(1.0 + R * math.sin(math.radians(q)), R - R * math.cos(math.radians(q))) for q in range(5, 91, 5)] + [(1.0 + R, z) for z in np.linspace(R + 0.3, 4, 6)]
verts = []; faces = []
for (y, z) in pts: verts += [(-6, y, z), (6, y, z)]
for i in range(len(pts) - 1): faces.append((2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2))
me = bpy.data.meshes.new('cyc'); me.from_pydata(verts, [], faces); cyc = bpy.data.objects.new('Backdrop', me); sc.collection.objects.link(cyc)
for p_ in me.polygons: p_.use_smooth = True
fm = bpy.data.materials.new('Floor'); fm.use_nodes = True; fb = fm.node_tree.nodes['Principled BSDF']; fb.inputs['Base Color'].default_value = (0.55, 0.55, 0.56, 1); fb.inputs['Roughness'].default_value = 0.7; me.materials.append(fm)
def area(name, loc, size, energy, sy):
    L = bpy.data.lights.new(name, 'AREA'); L.energy = energy; L.shape = 'RECTANGLE'; L.size = size; L.size_y = sy
    ob = bpy.data.objects.new(name, L); sc.collection.objects.link(ob); ob.location = loc; ob.visible_camera = False
    ob.rotation_euler = (mathutils.Vector((0, 0, 0.05)) - mathutils.Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
area('Key', (-0.15, -0.3, 1.7), 1.8, 90, 1.2); area('StripL', (-1.2, -0.6, 0.7), 0.5, 14, 1.6); area('StripR', (1.2, 0.3, 0.8), 0.5, 14, 1.6)
area('Back', (-0.3, 1.3, 1.0), 1.5, 18, 0.3); area('Fill', (0.2, -1.8, 0.6), 2.0, 6, 2.0)
w = bpy.data.worlds.new('W'); sc.world = w; w.use_nodes = True; w.node_tree.nodes['Background'].inputs[0].default_value = (0.035, 0.035, 0.038, 1)
cam = bpy.data.objects.new('Cam', bpy.data.cameras.new('Cam')); sc.collection.objects.link(cam); sc.camera = cam
sc.render.engine = 'CYCLES'; sc.cycles.device = 'GPU'
pr = bpy.context.preferences.addons['cycles'].preferences; pr.compute_device_type = 'OPTIX'; pr.get_devices()
for dv in pr.devices: dv.use = (dv.type == 'OPTIX')
sc.cycles.samples = SPP; sc.cycles.use_denoising = not os.environ.get('NODENOISE')
sc.view_settings.view_transform = 'AgX'; sc.view_settings.look = 'AgX - Punchy'; sc.view_settings.exposure = -1.1
if os.environ.get('VIEWXF'):      # [fix4-stop2] A6 colour-accurate review renders: VIEWXF=Standard (no look), EXPOSURE=ev
    sc.view_settings.view_transform = os.environ['VIEWXF']; sc.view_settings.look = 'None'; sc.view_settings.exposure = float(os.environ.get('EXPOSURE', 0.0))
sc.render.resolution_x = RES; sc.render.resolution_y = int(RES * 2 / 3)
VW = {'hero': ((1.35, -1.25, 0.95), (0.0, 0.0, 0.11), 70), 'rear': ((-1.35, -1.25, 0.9), (0.0, 0.0, 0.11), 70), 'side': ((0.05, -2.0, 0.5), (0.0, 0.0, 0.12), 70),
      'front': ((1.9, -0.45, 0.7), (0.0, 0.0, 0.11), 70), 'top': ((0.5, -0.95, 2.5), (0.0, 0.0, 0.1), 70), 'lidopen': ((-0.75, -0.95, 1.05), (-0.03, 0.0, 0.2), 85),
      'face': ((1.35, 0.0, 0.24), (0.2, 0.0, 0.165), 85), 'face34': ((1.05, -0.62, 0.36), (0.2, 0.0, 0.16), 85)}
DOCK_LIFT = 0.0173; dock = None
if any(v.startswith('dock') for v in VIEWS):          # charging base, placed for the docked pose (mower stands on its plate)
    dd = np.load(os.path.join(HERE, '..', 'work', 'asm_dock.npz')); dv_, df_, dl_ = dd['v'], dd['f'], dd['lab']      # [fix3-ports]
    _P3 = P.get('ports3') or {}      # [fix3-ports]
    if _P3.get('pin_standin'):      # [fix3-ports] clean pins (portcheck.py's pin model: fitted axes, d 7.94, domed tip) over the scanned pins, whose underside is open
        _pb = meshkit.Bag(); _r = float(_P3.get('pin_standin_r', 0.00397))
        for _p in _P3['pins']:
            _ax = lambda x_: np.array([x_, _p['y'] + _p['dy'] * (x_ - _p['x']), _p['z'] + _p['dz'] * (x_ - _p['x']) + DOCK_LIFT])      # dock-local (undo DOCK_LIFT)
            _a = _ax(float(_P3.get('tip_x', 0.2905)) + _r); _b = _ax(0.3505); _u = (_b - _a) / np.linalg.norm(_b - _a)
            _pb.add(*meshkit.lathe(_a, _u, [(-_r * np.cos(q_), _r * np.sin(q_)) for q_ in np.linspace(0, np.pi / 2, 10)] + [(np.linalg.norm(_b - _a), _r)], 64), 1)
            _c = _ax(0.351); _pb.add(*meshkit.sweep(np.array([[0.3495, _c[1], _c[2]], [0.3525, _c[1], _c[2]]]), meshkit.rrect(0.0186, 0.0150, 0.003, 8), up=(0, 0, 1.0)), 1)      # collar 15 x 18.6 (scan)
        _k = dl_ != 1; df_, dl_ = df_[_k], dl_[_k]      # [fix3-ports] the scanned pins (rods and ragged collars) are replaced
        _sv, _sf, _sl = _pb.out(); dv_ = np.r_[dv_, _sv]; df_ = np.r_[df_, _sf + len(dd['v'])]; dl_ = np.r_[dl_, _sl]      # [fix3-ports]
    dock = obj('Dock', dv_, df_, [mat('DockBody', '#2B2D31', 'satin'), mat('DockPin', '#C9A46A', 'metallic')], dl_, smooth_deg=35)      # [fix3-ports] dv_/df_/dl_
    dock.location.z = -DOCK_LIFT
    dock.location.y = float((P.get('ports3') or {}).get('dock_dy', 0.0))      # [fix3-ports] seated pose: pins centred in the chassis contact pockets (asm_dock was aligned to the old shell)
    bpy.context.view_layer.update()      # [fix3-ports] matrix_world current before CLIP / CLIPY cut the dock
VW.update({'dockhero': ((1.7, -1.5, 1.05), (0.08, 0.0, 0.11), 70), 'dockrear': ((-1.45, -1.35, 0.95), (0.08, 0.0, 0.11), 70), 'dockside': ((0.1, -2.1, 0.5), (0.08, 0.0, 0.12), 70),
           'dockfront': ((1.15, -0.95, 0.5), (0.28, 0.0, 0.15), 85), 'docklid': ((-0.9, -1.1, 1.15), (0.05, 0.0, 0.17), 75), 'docklidtop': ((-0.25, -0.75, 1.75), (0.06, 0.0, 0.15), 70),
           'lidtop': ((-0.3, -0.55, 1.45), (-0.04, 0.0, 0.2), 80), 'lidfront': ((0.95, -0.85, 1.1), (-0.03, 0.0, 0.2), 80)})
import meshkit
LIFT = float(os.environ.get('LIFT', 0)); LIFT2 = float(os.environ.get('LIFT2', LIFT)) if HASCOVER else LIFT      # exploded view: body raised off the chassis, top cover raised off the body
LIFTB = mathutils.Matrix.Translation((0, 0, LIFT)); LIFTM = mathutils.Matrix.Translation((0, 0, LIFT2))
for n_ in ('NewShell', 'Camera', 'PortSleeves', 'Mount', 'Section') + (() if HASCOVER else ('Controls',)):
    if bpy.data.objects.get(n_): bpy.data.objects[n_].matrix_world = LIFTB
for n_ in ('TopCover', 'CoverKit', 'LidWell', 'HingePod', 'HingePinion'):      # [fix4-hinge2] HingePinion
    if bpy.data.objects.get(n_): bpy.data.objects[n_].matrix_world = LIFTM
for n_ in ('LightLens', 'LightKit', 'Harness', 'LightCaps', 'HzLight'):      # [fix3-lights] the lamp modules and the body-side harness belong to the body  [hz] HzLight
    if bpy.data.objects.get(n_): bpy.data.objects[n_].matrix_world = LIFTB      # [fix3-lights]
if LIFT > 0 and os.path.exists(os.path.join(HERE, 'mounts.json')):      # exploded view: a thin line from every fastening point of the chassis up to its place in the body
    import meshkit as _mk; MJ_ = json.load(open(os.path.join(HERE, 'mounts.json'))); gb = _mk.Bag()
    for x_, y_, z_, _r in MJ_['posts']: gb.add(*_mk.tube((x_, y_, z_), (x_, y_, z_ + LIFT + 0.003), 0.0007, 10), 0)
    for u_ in MJ_['under']: gb.add(*_mk.tube((u_['x'], u_['y'], u_['z']), (u_['x'], u_['y'], u_['z'] + LIFT), 0.0007, 10), 1)
    gv, gf, gl = gb.out(); obj('Guides', gv, gf, [mat('GuideA', '#FF6A2B', 'satin', 3.0), mat('GuideB', '#3FA9FF', 'satin', 3.0)], gl)
piv = mathutils.Vector((float(d['lid_pivot'][0]), 0, float(d['lid_pivot'][1]))); OPEN = float(d['lid_open'][0]); struts = None; lid_state = None
def set_lid(deg):                                                       # swing the lid and everything fixed to it about the hinge axis; rebuild the gas struts
    global struts, lid_state
    if lid_state == deg: return
    lid_state = deg; T = LIFTM @ mathutils.Matrix.Translation(piv) @ mathutils.Matrix.Rotation(math.radians(-deg), 4, 'Y') @ mathutils.Matrix.Translation(-piv)
    lid.matrix_world = T; kitm.matrix_world = T
    for hn3_ in ('LightHalo', 'LightHaloCap'):      # [fix3-lights] the halo feed is part of the lid  [fix3-int] + its section cap
        if bpy.data.objects.get(hn3_): bpy.data.objects[hn3_].matrix_world = T
    if stopk is not None: stopk.matrix_world = T      # [fix3-stop] the STOP key moves with the lid
    if bpy.data.objects.get('HingePinion') and 'lid_pinion' in d.files:      # [fix4-hinge2] the pinion meshes with the tongue's sector gear: -ratio x the lid angle about its own axis
        cp_ = mathutils.Vector((float(d['lid_pinion'][0]), 0, float(d['lid_pinion'][1]))); n_ = float(d['lid_pinion'][2])
        bpy.data.objects['HingePinion'].matrix_world = LIFTM @ mathutils.Matrix.Translation(cp_) @ mathutils.Matrix.Rotation(math.radians(n_ * deg), 4, 'Y') @ mathutils.Matrix.Translation(-cp_)
    if struts is not None: bpy.data.objects.remove(struts, do_unlink=True); struts = None
    if len(d['strut_n']) == 0: return                                   # friction hinges: no gas struts
    bag = meshkit.Bag(); R = meshkit.roty(deg); pv = np.array([piv.x, 0, piv.z])
    for N_, M_ in zip(d['strut_n'], d['strut_m']): meshkit.strut(bag, N_, pv + R @ (M_ - pv))
    sv, sf, sl = bag.out(); struts = obj('Struts', sv, sf, KITM, sl, smooth_deg=35); struts.matrix_world = LIFTM
VW.update({'railclose': ((0.22, -0.72, 0.72), (-0.13, -0.135, 0.272), 150), 'camclose': ((0.8, -0.33, 0.31), (0.305, -0.004, 0.170), 170),
           'camside': ((0.60, -0.62, 0.23), (0.30, -0.004, 0.170), 190), 'waistclose': ((0.0, -1.1, 0.33), (-0.14, -0.17, 0.155), 140),
           'bibclose': ((1.0, -0.42, 0.44), (0.29, -0.01, 0.185), 125), 'tailclose': ((-1.0, -0.75, 0.42), (-0.25, -0.05, 0.18), 110),
           'lidmech': ((-0.60, -0.52, 0.66), (-0.15, -0.02, 0.30), 105), 'lidside': ((-0.06, -1.5, 0.42), (-0.06, 0.0, 0.31), 110),
           'lidin': ((0.72, -0.33, 0.80), (-0.05, 0.0, 0.275), 90), 'lidhinge': ((0.42, -0.36, 0.56), (-0.14, 0.02, 0.295), 150), 'lidunder': ((0.55, -0.42, 0.30), (-0.09, 0.0, 0.36), 105), 'topclose': ((-0.05, -0.35, 1.25), (-0.05, 0.0, 0.28), 90)})
for mc_ in filter(None, os.environ.get('MATCOL', '').split(',')):          # review renders: MATCOL="KitDark=#50545C:satin,..." recolours a material
    mn_, mv_ = mc_.split('='); hx_, fin_ = (mv_.split(':') + ['satin'])[:2]; m_ = bpy.data.materials.get(mn_)
    if m_: b_ = m_.node_tree.nodes['Principled BSDF']; b_.inputs['Base Color'].default_value = (*lin(hx_), 1); b_.inputs['Roughness'].default_value = FIN[fin_][0]; b_.inputs['Metallic'].default_value = FIN[fin_][1]; b_.inputs['Coat Weight'].default_value = FIN[fin_][2]
for hn_ in filter(None, os.environ.get('HIDE', '').split(',')):
    if bpy.data.objects.get(hn_): bpy.data.objects[hn_].hide_render = True
    if hn_ == 'LidKit' and bpy.data.objects.get('StopKey'): bpy.data.objects['StopKey'].hide_render = True      # [fix3-stop] hidden with the lid kit
    if hn_ == 'Chassis' and bpy.data.objects.get('PocketStandin'): bpy.data.objects['PocketStandin'].hide_render = True      # [fix4-front2] the pocket stand-in hides with the chassis
    for gp_, ch_ in (('LidKit', 'LightHalo'), ('LidKit', 'LightHaloCap'), ('Controls', 'HaloEmitter'), ('NewShell', 'LightLens'), ('NewShell', 'LightKit'), ('NewShell', 'Harness'), ('NewShell', 'HzLight')):      # [fix3-int] the halo feed is on the lid,
        if hn_ == gp_ and bpy.data.objects.get(ch_): bpy.data.objects[ch_].hide_render = True      # [fix3-int] its emitter on the panel, the lamp modules + harness in the body: hidden with them
for cv_ in filter(None, os.environ.get('CUSTOM_VIEWS', '').split(';')):      # extra views: name=x:y:z:tx:ty:tz:lens;...
    nm_, q_ = cv_.split('='); q_ = [float(t_) for t_ in q_.split(':')]; VW[nm_] = (tuple(q_[:3]), tuple(q_[3:6]), q_[6])
if os.environ.get('CLIP'):                                               # [v18c] CLIP=axis:value:sign - keep the side sign * (coord - value) > 0 (section.py's plane)
    import bmesh
    ax_, v_, s_ = os.environ['CLIP'].split(':'); ai_ = 'xyz'.index(ax_); pc_ = mathutils.Vector((0, 0, 0)); pc_[ai_] = float(v_); pn_ = mathutils.Vector((0, 0, 0)); pn_[ai_] = -1.0 if s_ == '-' else 1.0
    for o_ in list(sc.objects):
        if o_.type != 'MESH' or o_.name in ('Backdrop', 'Section', 'LightCaps', 'LightHaloCap'): continue      # [fix3-lights] LightCaps
        mw_ = o_.matrix_world; bm_ = bmesh.new(); bm_.from_mesh(o_.data)
        bmesh.ops.bisect_plane(bm_, geom=bm_.verts[:] + bm_.edges[:] + bm_.faces[:], plane_co=mw_.inverted() @ pc_, plane_no=(mw_.to_3x3().transposed() @ pn_).normalized(), clear_inner=True)
        bm_.to_mesh(o_.data); bm_.free()
if os.environ.get('CLIPY'):                                              # section view: cut every mesh at y = CLIPY and drop the side y < CLIPY
    import bmesh
    cy_ = float(os.environ['CLIPY'])
    for o_ in list(sc.objects):
        if o_.type != 'MESH' or o_.name in ('Backdrop', 'Section', 'LightCaps', 'LightHaloCap'): continue      # [v18c] the caps already lie on the plane  [fix3-lights] LightCaps
        mw_ = o_.matrix_world; co_ = mw_.inverted() @ mathutils.Vector((0, cy_, 0)); no_ = (mw_.to_3x3().transposed() @ mathutils.Vector((0, 1, 0))).normalized()
        bm_ = bmesh.new(); bm_.from_mesh(o_.data); bmesh.ops.bisect_plane(bm_, geom=bm_.verts[:] + bm_.edges[:] + bm_.faces[:], plane_co=co_, plane_no=no_, clear_inner=True)
        bm_.to_mesh(o_.data); bm_.free()
if os.environ.get('OBJLIST'): print('OBJECTS:', json.dumps(sorted((o_.name, len(o_.data.polygons) if o_.type == 'MESH' else 0, o_.hide_render) for o_ in sc.objects)))      # [fix3-int] render audit
for v in VIEWS:
    if dock is not None: dock.hide_render = not v.startswith('dock'); cyc.location.z = -DOCK_LIFT if v.startswith('dock') else 0.0
    set_lid((float(os.environ['LIDDEG']) if os.environ.get('LIDDEG') else OPEN) if 'lid' in v else 0.0)      # LIDDEG: any lid angle (review / animation)
    loc, tgt, lens = VW[v]; cam.location = loc; cam.data.lens = lens
    if os.environ.get('HEADLIGHT'):                                      # views into the body from below: a soft light travelling with the camera
        hl_ = bpy.data.objects.get('Head') or area('Head', loc, 0.6, float(os.environ['HEADLIGHT']), 0.6); hl_ = bpy.data.objects['Head']
        hl_.location = loc; hl_.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
    dxy = mathutils.Vector(tgt) - mathutils.Vector(loc); cyc.rotation_euler.z = math.atan2(dxy.y, dxy.x) - math.pi / 2
    cam.rotation_euler = dxy.to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f'{OUT}_{v}.png'; bpy.ops.render.render(write_still=True)
