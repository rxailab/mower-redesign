"""Export the v18 design for the web viewer, or as one full-resolution model.
usage:  blender -b --factory-startup -P export_web.py -- PARAMS.json DESIGN.npz OUTDIR
  web (default): OUTDIR/body.glb, cover.glb, lid.glb, lower.glb, panel.glb, design.json and the design part of hotspots.json
                 (OUTDIR/hotspots_base.json - wheels, casters, charging base - is appended unchanged). Compress afterwards with gltfpack.
  FULL=1:        OUTDIR/NAME.blend + NAME.glb, every part in its own collection, no decimation (NAME defaults to the npz name);
                 BASE=path/to/base.glb adds the chassis, drive wheels and casters.
Runs next to the v18 generator (keep.npz, control.npz, mounts.json, tail.py, meshkit.py in this folder).
Layers (viewer groups): lid (lid skin, STOP pad, lid kit) / cover (snap-on cover, cover kit, lid well, hinge pods) /
body (one-piece body, deck and screws, port sleeves, light kit) / lower (front bumper) / panel (controls: fixed on the chassis)."""
import bpy, sys, os, json, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
a = sys.argv[sys.argv.index('--') + 1:]; PJ, NPZ, OUT = a[0], a[1], a[2]
FULL = os.environ.get('FULL') == '1'
P = json.load(open(PJ)); col = P.get('colors', {}); d = np.load(NPZ)
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene

def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]; return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
FIN = {'gloss': (0.28, 0, 0.8), 'metallic': (0.34, 0.6, 1.0), 'satin': (0.5, 0, 0.1), 'matte': (0.72, 0, 0),          # same finishes as design_render.py
       'satincoat': (0.42, 0.0, 0.45), 'smoke': (0.16, 0.0, 0.8), 'texture': (0.62, 0.0, 0.0),
       'satinmetal': (0.36, 0.55, 0.35), 'glossdark': (0.12, 0.0, 1.0), 'matte_tex': (0.78, 0.0, 0.0)}      # [hz] Horizon finishes (design_render.py)
def mat(name, hexc, finish='satin', emit=0.0, emit_hex=None):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']; r, me, cc = FIN[finish]
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = r; b.inputs['Metallic'].default_value = me
    b.inputs['Coat Weight'].default_value = cc; b.inputs['Coat Roughness'].default_value = 0.06
    if emit: b.inputs['Emission Color'].default_value = (*lin(emit_hex or hexc), 1); b.inputs['Emission Strength'].default_value = emit
    return m
# material names are what the viewer keys on (app.js: setPaint, setup)
M = {'BodyPaint': mat('BodyPaint', col.get('body', '#CDD0CF'), col.get('body_finish', 'satincoat')),
     'DeckPaint': mat('DeckPaint', col.get('roof', '#0A0B0D'), col.get('roof_finish', 'gloss')),
     'Visor': mat('Visor', col.get('visor', '#0A0B0D'), col.get('visor_finish', 'smoke')),
     'LampLight': mat('LampLight', col.get('light', '#F2FBFF'), 'gloss', emit=1.0),
     'LidPaint': mat('LidPaint', col.get('lid', col.get('body', '#CDD0CF')), col.get('lid_finish', col.get('body_finish', 'satincoat'))),
     'Accent': mat('Accent', col.get('accent', '#FF6A2B'), 'satin'),
     'Glass': mat('Glass', '#0A0B0D', 'gloss'),
     'CorePaint': mat('CorePaint', col.get('core', '#2A2C30'), col.get('core_finish', 'texture')),
     'Glow': mat('Glow', col.get('visor', '#0A0B0D'), col.get('visor_finish', 'smoke'), emit=float(col.get('glow', 0.5)), emit_hex=col.get('light', '#F2FBFF')),
     'SensorRing': mat('SensorRing', col.get('ring_sensor', '#5E646C'), 'metallic'),
     'InnerSkin': mat('InnerSkin', col.get('kit', col.get('inner', '#34373C')), col.get('kit_finish', 'satin')),
     'StopButton': mat('StopButton', col.get('stop', '#E8552F'), col.get('stop_finish', 'satin')), 'StopLegend': mat('StopLegend', '#F4F4F2', 'satin'),      # [fix3-int] colors.stop (STOP red)  [fix4-stop2] colors.stop_finish
     'DisplayGlass': mat('DisplayGlass', '#0A0B0D', 'gloss'), 'SkirtPaint': mat('SkirtPaint', col.get('skirt', '#25272B'), 'satin'),
     'Dark': mat('Dark', '#0B0B0C', 'matte'), 'KitDark': mat('KitDark', col.get('kit', col.get('inner', '#33363B')), col.get('kit_finish', 'satin')),
     'KitRubber': mat('KitRubber', '#070708', 'matte'), 'KitSteel': mat('KitSteel', '#C9CDD2', 'metallic'),
     'KitArm': mat('KitArm', col.get('arm', '#595D65'), col.get('arm_finish', 'metallic')), 'KitLiner': mat('KitLiner', col.get('liner', '#6E727A'), 'satin'),
     'KitStrut': mat('KitStrut', '#0E0F11', 'gloss'), 'Inner': mat('Inner', col.get('inner', '#3C3F45'), 'satin'), 'Screw': mat('Screw', '#8E9299', 'metallic')}
M.update({'Halo': mat('Halo', col.get('halo', '#F2FBFF'), 'satin', emit=float(col.get('halo_emit', 2.0))), 'LensPrint': mat('LensPrint', '#08090B', 'matte'),      # [fix3-int] f3 parts
          'LensEdge': mat('LensEdge', '#0C0D10', 'satin'), 'LampHousing': mat('LampHousing', '#17181B', 'satin'), 'LightGuide': mat('LightGuide', col.get('light', '#DFF4FF'), 'gloss', emit=1.2),
          'Reflector': mat('Reflector', '#E9E9E4', 'satin'), 'PCB': mat('PCB', '#1E3B2C', 'satin'), 'LED': mat('LED', '#F6EBC0', 'gloss', emit=4.0), 'Foam': mat('Foam', '#0B0B0B', 'matte'),
          'Sensor': mat('Sensor', '#0A0B0F', 'gloss'), 'Connector': mat('Connector', '#3B3E44', 'satin'), 'Cable': mat('Cable', '#111214', 'satin'), 'TPE': mat('TPE', '#0D0D0E', 'matte'),
          'HaloFeed': mat('HaloFeed', col.get('light', '#DFF4FF'), 'gloss', emit=2.0)})
LKM = ['LampHousing', 'LightGuide', 'Reflector', 'PCB', 'LED', 'Foam', 'KitSteel', 'Sensor', 'InnerSkin', 'Connector', 'Cable', 'TPE', 'HaloFeed']      # [fix3-int] = design_render LKM_ (lightkit.MAT)
for n_, (r_, me_) in {'KitSteel': (0.22, 1.0), 'Screw': (0.3, 1.0)}.items():
    b_ = M[n_].node_tree.nodes['Principled BSDF']; b_.inputs['Roughness'].default_value = r_; b_.inputs['Metallic'].default_value = me_
SH = ['BodyPaint', 'DeckPaint', 'Visor', 'LampLight', 'LidPaint', 'Accent', 'Glass', 'CorePaint', 'Glow', 'SensorRing', 'InnerSkin']      # = design_render.py 'order'
if col.get('base'):      # [hz] the dark textured base below K2 (label 11)
    M['BasePaint'] = mat('BasePaint', col['base'], col.get('base_finish', 'matte_tex')); SH = SH + ['BasePaint']
M['HzLens'] = mat('HzLens', '#05070D', 'gloss'); M['HzLensEdge'] = mat('HzLensEdge', '#0C0D10', 'satin')      # [hz] light line / pupil / dot lenses
KIT = ['KitDark', 'KitRubber', 'KitSteel', 'KitArm', 'KitLiner', 'KitStrut', 'LidPaint']

def obj(name, V, F, mats, midx=None, normals=None, smooth_deg=35, weld=True, ratio=None, flat_up=False):
    me = bpy.data.meshes.new(name); me.vertices.add(len(V)); me.vertices.foreach_set('co', np.asarray(V, np.float32).ravel())
    F = np.asarray(F, np.int32); me.loops.add(F.size); me.loops.foreach_set('vertex_index', F.ravel()); me.polygons.add(len(F))
    me.polygons.foreach_set('loop_start', np.arange(0, F.size, 3, dtype=np.int32)); me.polygons.foreach_set('loop_total', np.full(len(F), 3, np.int32))
    if midx is not None: me.polygons.foreach_set('material_index', np.asarray(midx, np.int32))
    me.polygons.foreach_set('use_smooth', np.ones(len(F), bool)); me.update()
    o = bpy.data.objects.new(name, me); sc.collection.objects.link(o)
    for m in mats: me.materials.append(M[m])
    bpy.context.view_layer.objects.active = o; o.select_set(True)
    if normals is not None:
        me.normals_split_custom_set(np.asarray(normals, np.float64).reshape(-1, 3).tolist())
    else:
        if weld:
            bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT'); bpy.ops.mesh.remove_doubles(threshold=1e-6)
            bpy.ops.mesh.normals_make_consistent(inside=False); bpy.ops.object.mode_set(mode='OBJECT')
        if ratio:
            md = o.modifiers.new('dec', 'DECIMATE'); md.ratio = ratio; bpy.ops.object.modifier_apply(modifier=md.name)
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(smooth_deg))
        if flat_up:                                   # planar tops (long fan triangles) render flat, as in design_render.py
            for p_ in o.data.polygons:
                if p_.normal.z > 0.995: p_.use_smooth = False
    o.select_set(False); print(f'{name:14s} {len(o.data.polygons):8d} faces'); return o
def empty(name):
    e = bpy.data.objects.new(name, None); sc.collection.objects.link(e); return e
def sub(V, F, m):
    f_ = F[m]; u_ = np.unique(f_); rm_ = -np.ones(len(V), np.int64); rm_[u_] = np.arange(len(u_)); return V[u_], rm_[f_]

lab = d['lab']; part = d['part']; side = d['side']
midx = np.where(side == 1, 10, lab)                                    # inner skin -> InnerSkin; bare rim faces already carry lab 10
body = empty('BodyRoot'); cover = empty('CoverRoot'); lidr = empty('LidRoot'); lower = empty('LowerRoot'); panel = empty('PanelRoot')
m_ = (lab != 4) & (part == 0); v_, f_ = sub(d['v'], d['f'], m_); obj('NewShell', v_, f_, SH, midx[m_], normals=d['ln'][m_]).parent = body
m_ = (lab != 4) & (part == 1); v_, f_ = sub(d['v'], d['f'], m_); obj('TopCover', v_, f_, SH, midx[m_], normals=d['ln'][m_]).parent = cover
m_ = lab == 4; v_, f_ = sub(d['v'], d['f'], m_)
obj('Lid', v_, f_, ['LidPaint', 'StopButton', 'StopLegend', 'LampLight'], d['lid_paint'][m_] if 'lid_paint' in d.files else None, normals=d['ln'][m_]).parent = lidr      # [fix3-int] f3: no lid paint (raised STOP key)      # STOP pad, stop square, status halo
obj('LidKit', d['kitm_v'], d['kitm_f'], KIT, d['kitm_lab']).parent = lidr
pm_ = d['kitf_part'] == 23                                             # hinge pods: their own object, as in the renders
for nm_, mm_ in (('LidWell', ~pm_), ('HingePod', pm_)):
    v_, f_ = sub(d['kitf_v'], d['kitf_f'], mm_); obj(nm_, v_, f_, KIT, d['kitf_lab'][mm_]).parent = cover
if 'ckit_v' in d.files and len(d['ckit_f']): obj('CoverKit', d['ckit_v'], d['ckit_f'], ['Inner', 'Inner'], d['ckit_lab']).parent = cover
if 'mount_v' in d.files and len(d['mount_f']): obj('Mount', d['mount_v'], d['mount_f'], ['Inner', 'Dark', 'KitSteel', 'Screw'], d['mount_lab']).parent = body
if 'sleeve_v' in d.files: obj('PortSleeves', d['sleeve_v'], d['sleeve_f'], ['Dark'], smooth_deg=30).parent = body
EXTRA = [('hzl', 'HzLight', 'body', ['LampLight', 'HzLens', 'HzLensEdge']),      # [hz] light line, pupil, dot
         ('stop', 'StopKey', 'lid', ['StopButton', 'StopLegend', 'KitDark', 'KitRubber', 'KitSteel', 'KitArm', 'Halo', 'LidPaint']),      # [fix3-int] stopkit.MAT order
         ('lights_harn', 'Harness', 'body', LKM), ('lights_halo', 'LightHalo', 'lid', LKM)]      # [fix3] parts added by the fix-round-3 branches: (npz prefix, object name, group, materials)
if 'lights_lens_v' in d.files and len(d['lights_lens_f']):      # [fix3-int] the smoked lenses left the body mesh (lightkit): outer paint / inner print / rolled rim, own shading normals
    LS_, LL_ = d['lights_lens_side'], d['lights_lens_lab']; lo_ = np.select([LL_ == 3, LL_ == 6, LL_ == 9], [1, 2, 3], 0); li_ = np.select([LL_ == 3, LL_ == 6, LL_ == 9], [1, 2, 3], 4)
    obj('LightLens', d['lights_lens_v'], d['lights_lens_f'], ['Visor', 'LampLight', 'Glass', 'SensorRing', 'LensPrint', 'LensEdge'], np.where(LS_ == 0, lo_, np.where(LS_ == 1, li_, np.where(LL_ == 10, 5, lo_))),
        normals=d['lights_lens_vn'][d['lights_lens_f']] if 'lights_lens_vn' in d.files else None).parent = body
    KN_ = [str(k_) for k_ in d['lights_kinds']]; ke_ = d['lights_kit_kind'] == KN_.index('emitter')
    for nm_, m_, gp_ in (('LightKit', ~ke_, body), ('HaloEmitter', ke_, panel)):      # the halo emitter is a control-panel part
        if m_.any(): v_, f_ = sub(d['lights_kit_v'], d['lights_kit_f'], m_); obj(nm_, v_, f_, LKM, d['lights_kit_lab'][m_]).parent = gp_
for pre_, nm_, grp_, mats_ in EXTRA:
    if pre_ + '_v' in d.files and len(d[pre_ + '_f']):
        obj(nm_, d[pre_ + '_v'], d[pre_ + '_f'], mats_, d[pre_ + '_lab'] if pre_ + '_lab' in d.files else None).parent = {'body': body, 'cover': cover, 'lid': lidr, 'lower': lower, 'panel': panel}[grp_]

# front bumper (as scanned, the old skirt removed) bent by the tail plan like in the renders, and the plinth under the nose
import tail as _tail, meshkit
k = np.load(os.path.join(HERE, 'keep.npz')); kl = k['lab']; kc = k['v'][k['f']].mean(1); kk = ~((kl == 6) & (kc[:, 0] > 0.27))
if P.get('nose'): kk &= ~((kl == 0) & (kc[:, 0] > 0.255))
if P.get('skirt'): kk &= (kl != 0)
kv = k['v'].copy(); kv[:, 1] = _tail.make(P)(kv[:, 0], kv[:, 1]); v_, f_ = sub(kv, k['f'], kk)
obj('Bumper', v_, f_, ['SkirtPaint', 'Dark'], np.where((kl == 7) | (kl == 6), 1, 0)[kk], smooth_deg=40, weld=False, ratio=None if FULL else 0.085).parent = lower
if 'plinth_v' in d.files: obj('Plinth', d['plinth_v'], d['plinth_f'], ['SkirtPaint'], smooth_deg=30).parent = lower

# controls (fixed on the chassis): the scanned panel, the clean stand-in over it, the tower under it and the STOP legend on the real key
c = np.load(os.path.join(HERE, 'control.npz')); cl = c['lab']
obj('Controls', c['v'], c['f'], ['Dark', 'StopButton', 'DisplayGlass'], np.where(cl == 1, 1, np.where(cl == 2, 2, 0)), smooth_deg=40, ratio=None if FULL else 0.3).parent = panel
if 'ctlsi_v' in d.files:
    obj('ControlsSI', d['ctlsi_v'], d['ctlsi_f'], ['DisplayGlass', 'StopButton'], d['ctlsi_lab'], flat_up=True).parent = panel
    cv_ = c['v']; tw_ = meshkit.box(((cv_[:, 0].min() + cv_[:, 0].max()) / 2, 0.0, float(cv_[:, 2].min()) - 0.004), (np.ptp(cv_[:, 0]) - 0.012, np.ptp(cv_[:, 1]) - 0.012, 0.0085))
    obj('CtlTower', tw_[0], tw_[1], ['Dark'], smooth_deg=20).parent = panel
    kx_, ky_, ka_, kb_, kw_, kh_ = [float(t_) for t_ in d['ctlsi_key']]
    bpy.ops.object.text_add(); tx_ = bpy.context.active_object; tx_.name = 'StopLegend'; tx_.data.body = 'STOP'; tx_.data.align_x = 'CENTER'; tx_.data.align_y = 'CENTER'
    tx_.data.size = 0.6 * kw_; tx_.data.materials.append(M['StopLegend'])
    _fb = '/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf'      # [fix4-stop2] A7: one STOP typography (the cap inlay is Liberation Sans Bold)
    if os.path.exists(_fb): tx_.data.font = bpy.data.fonts.load(_fb)      # [fix4-stop2]
    tx_.rotation_mode = 'ZYX'; tx_.rotation_euler = (0.0, math.atan(-kb_), -math.pi / 2); tx_.location = (kx_, ky_, ka_ + kb_ * kx_ + 0.00006)      # reads from behind, as the key
    bpy.ops.object.convert(target='MESH'); tx_ = bpy.context.active_object; tx_.parent = panel; tx_.select_set(False)

def export(root, path):
    bpy.ops.object.select_all(action='DESELECT'); root.select_set(True)
    for o in root.children: o.select_set(True)
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True, export_apply=True, export_normals=True,
                              export_texcoords=False, export_materials='EXPORT', export_cameras=False, export_lights=False, export_animations=False)
    print('exported', os.path.basename(path), os.path.getsize(path) // 1024, 'KB')
os.makedirs(OUT, exist_ok=True)
ROOTS = {'body': body, 'cover': cover, 'lid': lidr, 'lower': lower, 'panel': panel}
MJ = json.load(open(os.path.join(HERE, 'mounts.json')))

def chassis_cut(objs):          # chassis struts of the old mudguards beside the wheels (see the renderer): same cut as the renders and the viewer
    import strutskit as _sk; _S = _sk.from_npz(d)      # [fix3-int] fix-struts: the capped plane cuts of the old mudguard pin + hook only (as design_render)
    if _S and _S['mode'] == 'cut':
        for o in objs:
            if o.type == 'MESH' and o.name == 'Chassis': print('struts cut:', json.dumps(_sk.cut_bmesh(o, _S, caps=(d['struts_cap_v'], d['struts_cap_f'], d['struts_cap_cut']))))
        return
    import bmesh
    CUT = json.load(open(os.path.join(HERE, 'chassis_cut.json'))) if os.path.exists(os.path.join(HERE, 'chassis_cut.json')) else None
    n_ = 0
    for o in objs:
        if o.type != 'MESH': continue
        bm = bmesh.new(); bm.from_mesh(o.data); mw = o.matrix_world; dead = []
        for v_ in bm.verts:
            p_ = mw @ v_.co
            if abs(p_.y) > 0.160 and p_.z > 0.135 and -0.06 < p_.x < 0.29 and not any((p_.x - q[0]) ** 2 + (p_.y - q[1]) ** 2 < 0.0068 ** 2 for q in MJ['posts']):
                dead.append(v_)
        n_ += len(dead); bmesh.ops.delete(bm, geom=dead, context='VERTS'); bm.to_mesh(o.data); bm.free()
    print('chassis trimmed:', n_, 'vertices', '(cut file present)' if CUT else '')

if FULL:
    NAME = os.environ.get('NAME', os.path.splitext(os.path.basename(NPZ))[0])
    CN = {'body': 'Body (one-piece, skirt, deck, screws, light kit)', 'cover': 'Top cover (snap-on, lid well, hinge pods)', 'lid': 'Lid (STOP pad, lid kit)',
          'lower': 'Front bumper', 'panel': 'Control panel (STOP key, display)'}
    def to_col(objs, cname):
        c_ = bpy.data.collections.new(cname); sc.collection.children.link(c_)
        for o in objs:
            for u in list(o.users_collection): u.objects.unlink(o)
            c_.objects.link(o)
    for k_, r_ in ROOTS.items():
        if r_.children: to_col([r_, *r_.children], CN[k_])
        else: bpy.data.objects.remove(r_)
    BASE = os.environ.get('BASE')
    if BASE:
        before = set(bpy.data.objects); bpy.ops.import_scene.gltf(filepath=BASE); new = [o for o in bpy.data.objects if o not in before]
        ch = bpy.data.objects.get('Chassis'); chassis_cut([ch, *ch.children_recursive] if ch else [])
        to_col(new, 'Chassis, drive wheels, casters (original)')
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, NAME + '.blend'), compress=True)
    bpy.ops.export_scene.gltf(filepath=os.path.join(OUT, NAME + '.glb'), export_format='GLB', use_selection=False, export_yup=True, export_apply=True,
                              export_normals=True, export_materials='EXPORT', export_cameras=False, export_lights=False, export_animations=False)
    for ext in ('.blend', '.glb'): print('saved', NAME + ext, os.path.getsize(os.path.join(OUT, NAME + ext)) // 1024, 'KB')
    sys.exit(0)
for n_, r_ in ROOTS.items(): export(r_, os.path.join(OUT, n_ + '.glb'))

# ---- numbers the viewer needs (glTF axes: x forward, y up, z = the mower's right)
g2 = lambda p: [round(float(p[0]), 5), round(float(p[2]), 5), round(float(-p[1]), 5)]
V = d['v']; F = d['f']; FC = V[F].mean(1); pv = d['lid_pivot']
def at(m):            # centre of the faces in mask m
    return FC[m].mean(0)
info = {'version': 'v18', 'wall': True, 'wall_t': float(d['wall_t'][0]),
        'lid': {'pivot': [round(float(pv[0]), 5), round(float(pv[1]), 5), 0.0], 'open_deg': float(d['lid_open'][0])},
        'posts': [g2(p_[:3]) for p_ in MJ['posts']], 'under': [g2([u_['x'], u_['y'], u_['z']]) for u_ in MJ.get('under', [])],
        'sizes': {n: os.path.getsize(os.path.join(OUT, n + '.glb')) for n in ROOTS}}
info['dock_dy'] = float((P.get('ports3') or {}).get('dock_dy', 0.0))
if col.get('base'): info['paint_map'] = {'body': ['BodyPaint'], 'deck': ['DeckPaint', 'LidPaint']}; info['variant'] = 'horizon'      # [hz] palettes: canopy / spine + lid (the lid is a segment of the spine)      # [web] seated docking pose (fix3-ports): the viewer shifts the charging base by this in y
json.dump(info, open(os.path.join(OUT, 'design.json'), 'w'), indent=1)
if P.get('struts'):          # [web] the exact chassis cuts for the viewer (chassis_cut.js): same spec and caps as the renderer
    import subprocess          # strutskit needs shapely, which Blender's Python lacks: run it with the design venv
    subprocess.run([os.path.join(HERE, '..', 'venv', 'bin', 'python'), 'strutskit.py', os.path.abspath(PJ), os.path.abspath(os.path.join(OUT, 'chassis_cut.json'))], cwd=HERE, check=True); print('wrote chassis_cut.json')

# hotspot anchors measured on the geometry (texts: hotspots_text.json next to this file, keyed by id)
o0 = side == 0
lidm = lab == 4; lv = V[np.unique(F[lidm])]; lidc = lv.mean(0); lidz = float(lv[:, 2].max())
if 'stop_v' in d.files:      # [fix3-int] the raised STOP key: anchor on its cap top
    _cap = d['stop_v'][np.unique(d['stop_f'][d['stop_lab'] <= 1])]; stop = np.array([float(d['stop_axis'][0]), float(d['stop_axis'][1]), float(_cap[:, 2].max())])
else: stop = at(lidm & (d['lid_paint'] == 1)); stop[2] = float(V[np.unique(F[lidm & (d['lid_paint'] == 1)])][:, 2].max())
if not (lab == 9).any() and 'lights_lens_v' in d.files:      # [fix3-int] band faces live in the lens solids: anchors from their outer faces
    V = np.r_[V, d['lights_lens_v']]; F = np.r_[F, d['lights_lens_f'] + len(d['v'])]; FC = V[F].mean(1); n_ = len(d['lights_lens_f'])
    lab = np.r_[lab, d['lights_lens_lab']]; part = np.r_[part, np.zeros(n_, part.dtype)]; o0 = np.r_[o0, d['lights_lens_side'] == 0]; lidm = np.r_[lidm, np.zeros(n_, bool)]
HZ = 'hzl_v' in d.files and len(d['hzl_f']) > 0      # [hz] Horizon: light line / pupil / dot live in the hzl arrays
if HZ:
    HV, HF, HL = d['hzl_v'], d['hzl_f'], d['hzl_lab']; HC = HV[HF].mean(1)
    def _hz(m, front):
        p = HC[m].mean(0); vv = HV[np.unique(HF[m])]; p[0] = float(vv[:, 0].max() if front else vv[:, 0].min()); return p
    pupil = _hz((HL == 1) & (HC[:, 0] > 0.2), True); rear = _hz((HL == 1) & (HC[:, 0] < -0.2), False)
    def band(front, sgn): return _hz((HL == 0) & ((HC[:, 0] > 0.2) if front else (HC[:, 0] < -0.2)) & (sgn * HC[:, 1] > 0.05), front)
else:
    pupil = at(o0 & (lab == 9) & (FC[:, 0] > 0.2)); pupil[0] = float(V[np.unique(F[o0 & (lab == 9) & (FC[:, 0] > 0.2)])][:, 0].max())
    def band(front, sgn):          # light segment of a lens band
        m = o0 & (lab == 3) & ((FC[:, 0] > 0.2) if front else (FC[:, 0] < -0.2)) & (sgn * FC[:, 1] > 0.03); p = at(m)
        vv = V[np.unique(F[m])]; p[0] = float(vv[:, 0].max() if front else vv[:, 0].min()); return p
    rear = at(o0 & (lab == 2) & (FC[:, 0] < -0.2) & (np.abs(FC[:, 1]) < 0.01)); rear[0] = float(V[np.unique(F[o0 & (lab == 2) & (FC[:, 0] < -0.2) & (np.abs(FC[:, 1]) < 0.01)])][:, 0].min())
ps = P.get('port_slot', {}); slz = 0.5 * (ps.get('z0', 0.1315) + ps.get('z1', 0.1605))
m_ = o0 & (part == 0) & (np.abs(FC[:, 1] - 0.045) < 0.004) & (np.abs(FC[:, 2] - (ps.get('z1', 0.1605) + 0.002)) < 0.002) & (FC[:, 0] > 0.2)
slot = np.array([float(FC[m_, 0].max()) if m_.any() else 0.31, 0.045, slz])
CV = V[np.unique(F[(part == 1) & (lab != 4) & o0])]
def top_z(vv, x, y, r=0.005):
    m = np.hypot(vv[:, 0] - x, vv[:, 1] - y) < r; return float(vv[m, 2].max())
cvp = [-0.215, -0.125, top_z(CV, -0.215, -0.125)]
BV = V[np.unique(F[(part == 0) & o0 & (lab == 7)])]
orb = BV[(np.abs(BV[:, 0] - (-0.20 if HZ else 0.119)) < 0.004) & (BV[:, 1] < -0.15)]; orbp = orb[np.argmax(orb[:, 2])] if len(orb) else np.array([0.119, -0.17, 0.24])      # [hz] Horizon: K1 on the rear flank
SCV = d['mount_v'][np.unique(d['mount_f'][d['mount_lab'] == 3])]
sp_ = min(MJ['posts'], key=lambda p_: (p_[0] - 0.06) ** 2 + (p_[1] + 0.16) ** 2)
hp_ = d['kitf_v'][np.unique(d['kitf_f'][d['kitf_part'] == 23])]; hinge = hp_[hp_[:, 1] < 0].mean(0) if len(hp_) else np.array([pv[0], -0.05, pv[1]])
A = {'lid': dict(part='lid', pos=g2([lidc[0] - 0.04, 0.035, lidz]), normal=[0, 1, 0]),
     'stop': dict(part='lid', pos=g2(stop), normal=[0, 1, 0]),
     'display': dict(part='base', pos=[-0.0689, 0.2545, -0.0032], normal=[0, 1, 0], lid=True),
     'hinge': dict(part='cover', pos=g2(hinge), normal=[-0.5, 0.85, 0], lid=True),
     'pupil': dict(part='body', front=True, pos=g2(pupil), normal=[1, 0, 0], lift=0.03, liftDir=[0.6, 0.8, 0]),
     'ledL': dict(part='body', front=True, pos=g2(band(True, 1)), normal=[1, 0, -0.25], lift=0.03),
     'ledR': dict(part='body', front=True, pos=g2(band(True, -1)), normal=[1, 0, 0.25], lift=0.03),
     'slot': dict(part='body', front=True, pos=g2(slot), normal=[1, 0, 0], lift=0.03, liftDir=[0.7, -0.5, 0.5]),
     'tail': dict(part='body', pos=g2(rear), normal=[-1, 0, 0], lift=0.03),
     'cover': dict(part='cover', pos=g2(cvp), normal=[-0.25, 0.9, 0.35], lift=0.035),
     'partline': dict(part='body', pos=g2(orbp), normal=[0, 0.6, 0.8], lift=0.03),
     'spine': dict(part='cover', pos=g2(at(o0 & (lab == 1) & (np.abs(FC[:, 1]) < 0.01) & (FC[:, 0] > 0.2) & (FC[:, 0] < 0.24))), normal=[0.6, 0.8, 0], lift=0.03) if (lab == 1).any() else None,
     'screws': dict(part='body', explode=True, pos=g2([sp_[0], sp_[1], top_z(SCV, sp_[0], sp_[1], 0.006)]), normal=[0, 1, 0], lift=0.04)}
TX = json.load(open(os.path.join(HERE, 'hotspots_text.json')))
HS = [dict(id=k_, **A[k_], **TX[k_]) for k_ in TX if A.get(k_)]
old = json.load(open(os.path.join(OUT, 'hotspots_base.json')))           # wheels, casters, charging base: unchanged parts
json.dump(HS + old, open(os.path.join(OUT, 'hotspots.json'), 'w'), ensure_ascii=False, indent=1)
print('hotspots', [h['id'] for h in HS + old])
