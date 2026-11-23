# Render a redesign concept. usage: blender -b --factory-startup -P design_render.py -- PARAMS.json SHELL.npz OUTPREFIX views res samples
import bpy, sys, os, json, math, numpy as np, mathutils
a = sys.argv[sys.argv.index('--') + 1:]; PJ, NPZ, OUT, VIEWS, RES, SPP = a[0], a[1], a[2], a[3].split(','), int(a[4]), int(a[5])
HERE = os.path.dirname(os.path.abspath(__file__)); P = json.load(open(PJ)); col = P.get('colors', {})
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene
def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]; return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
FIN = {'gloss': (0.28, 0, 0.8), 'metallic': (0.34, 0.6, 1.0), 'satin': (0.5, 0, 0.1), 'matte': (0.72, 0, 0)}
def mat(name, hexc, finish='satin', emit=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']; r, me, cc = FIN[finish]
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = r; b.inputs['Metallic'].default_value = me
    b.inputs['Coat Weight'].default_value = cc; b.inputs['Coat Roughness'].default_value = 0.06
    if emit: b.inputs['Emission Color'].default_value = (*lin(hexc), 1); b.inputs['Emission Strength'].default_value = emit
    return m
def obj(name, V, F, mats, midx=None, smooth_deg=40):
    me = bpy.data.meshes.new(name); me.vertices.add(len(V)); me.vertices.foreach_set('co', np.asarray(V, np.float32).ravel())
    F = np.asarray(F, np.int32); me.loops.add(F.size); me.loops.foreach_set('vertex_index', F.ravel()); me.polygons.add(len(F))
    me.polygons.foreach_set('loop_start', np.arange(0, F.size, 3, dtype=np.int32)); me.polygons.foreach_set('loop_total', np.full(len(F), 3, np.int32))
    if midx is not None: me.polygons.foreach_set('material_index', np.asarray(midx, np.int32))
    me.polygons.foreach_set('use_smooth', np.ones(len(F), bool)); me.update(); me.validate()
    o = bpy.data.objects.new(name, me); sc.collection.objects.link(o)
    for m in mats: me.materials.append(m)
    bpy.context.view_layer.objects.active = o; o.select_set(True)
    if name in ('NewShell', 'Lid'):
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
if ch:
    import bmesh
    bm = bmesh.new(); bm.from_mesh(ch.data); mw = ch.matrix_world
    kill = [v for v in bm.verts if (lambda w: abs(w.y) > 0.160 and w.z > 0.135 and -0.06 < w.x < 0.29)(mw @ v.co)]
    bmesh.ops.delete(bm, geom=kill, context='VERTS'); bm.to_mesh(ch.data); bm.free()
M = {'body': mat('Body', col.get('body', '#C9CDD2'), col.get('body_finish', 'metallic')),
     'roof': mat('RoofPanel', col.get('roof', '#1B1D21'), col.get('roof_finish', 'gloss')),
     'visor': mat('Visor', col.get('visor', '#07080A'), col.get('visor_finish', 'gloss')), 'light': mat('Light', col.get('light', '#DFF4FF'), 'gloss', 6.0),
     'lid': mat('Lid', col.get('lid', col.get('body', '#C9CDD2')), col.get('lid_finish', col.get('body_finish', 'metallic'))),
     'accent': mat('Accent', col.get('accent', '#E8552F'), 'satin'), 'seam': mat('Seam', '#050505', 'matte'),
     'skirt': mat('Skirt', col.get('skirt', '#2A2C30'), 'satin'), 'dark': mat('Dark', '#0B0B0C', 'matte'),
     'stop': mat('Stop', '#E8552F', 'satin'), 'glass': mat('Glass', '#0A0B0D', 'gloss')}
k = np.load(os.path.join(HERE, 'keep.npz')); kl = k['lab']
kc = k['v'][k['f']].mean(1); kk = ~((kl == 6) & (kc[:, 0] > 0.27))
if P.get('nose'): kk &= ~((kl == 0) & (kc[:, 0] > 0.255))      # old shell's front rim: replaced by the new nose, which sits on the bumper        # charger-opening internals are replaced by clean sleeves
sys.path.insert(0, HERE); import tail as _tail; kv = k['v'].copy(); kv[:, 1] = _tail.make(P)(kv[:, 0], kv[:, 1])
obj('Skirt', kv, k['f'][kk], [M['skirt'], M['dark']], np.where((kl == 7) | (kl == 6), 1, 0)[kk])
d = np.load(NPZ)
order = ['body', 'roof', 'visor', 'light', 'lid', 'accent']
lab = d['lab']; LIDOPEN = 'lidopen' in VIEWS
shell = obj('NewShell', d['v'], d['f'][lab != 4], [M[n] for n in order], lab[lab != 4], smooth_deg=P.get('smooth_deg', 75))
lidF = d['f'][lab == 4]; u = np.unique(lidF); rm = -np.ones(len(d['v']), np.int64); rm[u] = np.arange(len(u))
lid = obj('Lid', d['v'][u], rm[lidF], [M['lid']], smooth_deg=P.get('smooth_deg', 75))
seam = obj('Seam', d['v_seam'] if 'v_seam' in d.files else d['seam_v'], d['seam_f'], [M['seam']])
if 'line_v' in d.files: obj('Lines', d['line_v'], d['line_f'], [M['light'], M['accent'], M['roof']], np.where(d['line_lab'] == 3, 0, np.where(d['line_lab'] == 1, 2, 1)))
if 'decal_v' in d.files:
    dk = obj('FrontDecals', d['decal_v'], d['decal_f'], [M[n] for n in order], d['decal_lab'], smooth_deg=80)
    dk.visible_shadow = False
if 'plinth_v' in d.files: obj('Plinth', d['plinth_v'], d['plinth_f'], [M['skirt']], smooth_deg=30)
if 'sleeve_v' in d.files: obj('PortSleeves', d['sleeve_v'], d['sleeve_f'], [M['dark']], smooth_deg=30)
c = np.load(os.path.join(HERE, 'control.npz')); cl = c['lab']
obj('Controls', c['v'], c['f'], [M['dark'], M['stop'], M['glass']], np.where(cl == 1, 1, np.where(cl == 2, 2, 0)))
poly = d['lid_poly']; cen = poly.mean(0)                                       # tray under the lid
tv = np.c_[np.r_[poly, [cen]], np.full(len(poly) + 1, 0.2440)]; n = len(poly)
obj('Tray', tv, np.c_[np.arange(n), (np.arange(n) + 1) % n, np.full(n, n)], [M['dark']])
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
sc.cycles.samples = SPP; sc.cycles.use_denoising = True
sc.view_settings.view_transform = 'AgX'; sc.view_settings.look = 'AgX - Punchy'; sc.view_settings.exposure = -1.1
sc.render.resolution_x = RES; sc.render.resolution_y = int(RES * 2 / 3)
VW = {'hero': ((1.35, -1.25, 0.95), (0.0, 0.0, 0.11), 70), 'rear': ((-1.35, -1.25, 0.9), (0.0, 0.0, 0.11), 70), 'side': ((0.05, -2.0, 0.5), (0.0, 0.0, 0.12), 70),
      'front': ((1.9, -0.45, 0.7), (0.0, 0.0, 0.11), 70), 'top': ((0.5, -0.95, 2.5), (0.0, 0.0, 0.1), 70), 'lidopen': ((-0.75, -0.95, 1.05), (-0.03, 0.0, 0.2), 85),
      'face': ((1.35, 0.0, 0.24), (0.2, 0.0, 0.165), 85), 'face34': ((1.05, -0.62, 0.36), (0.2, 0.0, 0.16), 85)}
DOCK_LIFT = 0.0173; dock = None
if any(v.startswith('dock') for v in VIEWS):          # charging base, placed for the docked pose (mower stands on its plate)
    dd = np.load(os.path.join(HERE, '..', 'work', 'asm_dock.npz'))
    dock = obj('Dock', dd['v'], dd['f'], [mat('DockBody', '#2B2D31', 'satin'), mat('DockPin', '#C9A46A', 'metallic')], dd['lab'], smooth_deg=35)
    dock.location.z = -DOCK_LIFT
VW.update({'dockhero': ((1.7, -1.5, 1.05), (0.08, 0.0, 0.11), 70), 'dockrear': ((-1.45, -1.35, 0.95), (0.08, 0.0, 0.11), 70), 'dockside': ((0.1, -2.1, 0.5), (0.08, 0.0, 0.12), 70),
           'dockfront': ((1.15, -0.95, 0.5), (0.28, 0.0, 0.15), 85), 'docklid': ((-0.9, -1.1, 1.15), (0.05, 0.0, 0.17), 75), 'docklidtop': ((-0.25, -0.75, 1.75), (0.06, 0.0, 0.15), 70),
           'lidtop': ((-0.3, -0.55, 1.45), (-0.04, 0.0, 0.2), 80), 'lidfront': ((0.95, -0.85, 1.1), (-0.03, 0.0, 0.2), 80)})
lid_opened = False
for v in VIEWS:
    if dock is not None: dock.hide_render = not v.startswith('dock'); cyc.location.z = -DOCK_LIFT if v.startswith('dock') else 0.0
    if 'lid' in v and not lid_opened:
        lid_opened = True                                                       # hinge the lid open about its rear edge
        px = poly[:, 0].min(); zc = float(d['v'][u][:, 2][np.abs(d['v'][u][:, 0] - px) < 0.01].mean())
        piv = mathutils.Vector((px, 0, zc)); rot = mathutils.Matrix.Rotation(math.radians(-62), 4, 'Y')
        sol = lid.modifiers.new('s', 'SOLIDIFY'); sol.thickness = 0.003; sol.offset = -1
        lid.matrix_world = mathutils.Matrix.Translation(piv) @ rot @ mathutils.Matrix.Translation(-piv); seam.hide_render = True
    loc, tgt, lens = VW[v]; cam.location = loc; cam.data.lens = lens
    dxy = mathutils.Vector(tgt) - mathutils.Vector(loc); cyc.rotation_euler.z = math.atan2(dxy.y, dxy.x) - math.pi / 2
    cam.rotation_euler = dxy.to_track_quat('-Z', 'Y').to_euler()
    sc.render.filepath = f'{OUT}_{v}.png'; bpy.ops.render.render(write_still=True)
