# Product renders of the ORIGINAL (unmodified) mower for the AIRAJ site, straight from mower_assembly.blend.
# usage (repo root):
#   blender -b mower_assembly.blend -P scripts/airaj/render_r1.py -- OUTPREFIX VIEWS RES SPP
#   VIEWS: comma list of names from VW below or from CUSTOM_VIEWS="name=x:y:z:tx:ty:tz:lens;..." (metres, Z up, front of the mower = +X)
# env:
#   PALETTE=id | PALETTE_LIST=id,id|all    colour schemes from viewer/original/models/palettes.json (default: the blend's own black)
#   MODE=studio (default)  transparent film, the original softbox lights, a shadow catcher floor (RGBA, shadow in alpha)
#   MODE=sun               transparent film + shadow catcher, a warm low sun from SUN="azimuth_deg:elevation_deg" and a sky fill
#                          (for compositing onto garden photos)
#   MODE=backdrop          the blend's own grey cyclorama (opaque)
#   DOCK=1                 the charging base (scripts/work/asm_dock.npz) under the mower, mower lifted onto the plate
#   ASPECT=w:h             frame aspect (default 3:2)   EXPOSURE=ev   PITCH="view=deg" (mower pitched nose-up, e.g. on a slope)
import bpy, sys, os, json, math, mathutils
import numpy as np

argv = sys.argv[sys.argv.index('--') + 1:]
OUT, VIEWS, RES, SPP = argv[0], argv[1].split(','), int(argv[2]), int(argv[3])
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sc = bpy.context.scene; MODE = os.environ.get('MODE', 'studio')
for im in bpy.data.images:      # the blend points at textures in a deleted work folder: use the copies kept in the repo
    if im.name in ('display.png', 'window.png'): im.filepath = os.path.join(ROOT, 'scripts', 'pipeline', 'tex_generated', im.name); im.reload()
MOWER = [o for o in bpy.data.objects if o.type == 'MESH' and o.name != 'Backdrop' and o.parent is None]

# ---- colour schemes (same mapping as scripts/mower_scene.apply_palette)
FINISH = {'shell': {'gloss': (0.30, 0.0, 0.8, 0.06), 'metallic': (0.36, 0.55, 1.0, 0.05), 'satin': (0.50, 0.0, 0.15, 0.2), 'matte': (0.75, 0.0, 0.0, 0.3)},
          'middle': {'gloss': (0.28, 0.0, 0.45, 0.08), 'metallic': (0.40, 0.5, 0.6, 0.08), 'satin': (0.52, 0.0, 0.0, 0.3), 'matte': (0.72, 0.0, 0.0, 0.3)}}
def hex_lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)
def bsdf(name): return bpy.data.materials[name].node_tree.nodes["Principled BSDF"]
def set_bsdf(name, col, rough, metal, coat, coat_rough):
    p = bsdf(name); p.inputs["Base Color"].default_value = (*col, 1); p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal; p.inputs["Coat Weight"].default_value = coat; p.inputs["Coat Roughness"].default_value = coat_rough
def stripe_nodes():
    nt = bpy.data.materials['ShellPaint'].node_tree; N = nt.nodes; Lk = nt.links; b = N["Principled BSDF"]
    if "StripeMix" in N: return N
    tc = N.new("ShaderNodeTexCoord"); sp = N.new("ShaderNodeSeparateXYZ"); Lk.new(tc.outputs["Object"], sp.inputs[0])
    ab = N.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"; Lk.new(sp.outputs["Y"], ab.inputs[0])
    g = N.new("ShaderNodeMath"); g.operation = "GREATER_THAN"; g.inputs[1].default_value = 0.011; Lk.new(ab.outputs[0], g.inputs[0])
    l = N.new("ShaderNodeMath"); l.operation = "LESS_THAN"; l.inputs[1].default_value = 0.056; Lk.new(ab.outputs[0], l.inputs[0])
    mm = N.new("ShaderNodeMath"); mm.operation = "MULTIPLY"; Lk.new(g.outputs[0], mm.inputs[0]); Lk.new(l.outputs[0], mm.inputs[1])
    on = N.new("ShaderNodeMath"); on.operation = "MULTIPLY"; on.name = "StripeOn"; Lk.new(mm.outputs[0], on.inputs[0])
    mx = N.new("ShaderNodeMix"); mx.data_type = "RGBA"; mx.name = "StripeMix"; Lk.new(on.outputs[0], mx.inputs["Factor"])
    Lk.new(mx.outputs["Result"], b.inputs["Base Color"]); return N
def apply_palette(pal):
    sc_ = hex_lin(pal['shell']['hex']); set_bsdf('ShellPaint', sc_, *FINISH['shell'][pal['shell']['finish']])
    set_bsdf('MidBody', hex_lin(pal['middle']['hex']), *FINISH['middle'][pal['middle']['finish']])
    if pal.get('pod'): bsdf('PodLight').inputs["Base Color"].default_value = (*hex_lin(pal['pod']), 1)
    N = stripe_nodes(); N["StripeOn"].inputs[1].default_value = 1.0 if pal.get('stripe') else 0.0
    N["StripeMix"].inputs["A"].default_value = (*sc_, 1); N["StripeMix"].inputs["B"].default_value = (*hex_lin(pal.get('stripe') or '#000000'), 1)
    if pal.get('ring'):
        rl = hex_lin(pal['ring']); mx = max(rl) or 1.0; e = tuple(c / mx for c in rl); rp = bsdf('CyanRing')
        rp.inputs["Base Color"].default_value = (*[0.78 * c for c in e], 1); rp.inputs["Emission Color"].default_value = (*e, 1)
PALS = {p['id']: p for p in json.load(open(os.path.join(ROOT, 'viewer', 'original', 'models', 'palettes.json')))['schemes']}

# ---- charging base
if os.environ.get('DOCK'):
    dd = np.load(os.path.join(ROOT, 'scripts', 'work', 'asm_dock.npz'))
    me = bpy.data.meshes.new('Dock'); V = dd['v'].astype(np.float32); F = dd['f'].astype(np.int32)
    me.vertices.add(len(V)); me.vertices.foreach_set('co', V.ravel()); me.loops.add(F.size); me.loops.foreach_set('vertex_index', F.ravel())
    me.polygons.add(len(F)); me.polygons.foreach_set('loop_start', np.arange(0, F.size, 3, dtype=np.int32)); me.polygons.foreach_set('loop_total', np.full(len(F), 3, np.int32))
    me.polygons.foreach_set('material_index', dd['lab'].astype(np.int32)); me.polygons.foreach_set('use_smooth', np.ones(len(F), bool)); me.update(); me.validate()
    for nm, col, r, mt in (('DockBody', (0.028, 0.029, 0.032), 0.55, 0.0), ('DockPin', (0.80, 0.60, 0.36), 0.3, 1.0)):
        m = bpy.data.materials.new(nm); m.use_nodes = True; p = m.node_tree.nodes["Principled BSDF"]
        p.inputs["Base Color"].default_value = (*col, 1); p.inputs["Roughness"].default_value = r; p.inputs["Metallic"].default_value = mt; me.materials.append(m)
    sc.collection.objects.link(bpy.data.objects.new('Dock', me))
    for o in MOWER: o.location.z += 0.0173          # DOCK_LIFT: the mower stands on the plate

# ---- stage
bd = bpy.data.objects.get('Backdrop')
if MODE in ('studio', 'sun'):
    sc.render.film_transparent = True
    if bd: bd.hide_render = True
    bpy.ops.mesh.primitive_plane_add(size=12, location=(0, 0, 0)); cat = bpy.context.active_object; cat.name = 'ShadowCatcher'; cat.is_shadow_catcher = True
    if os.environ.get('CATCHER_PITCH'): cat.rotation_euler = (0.0, math.radians(-float(os.environ['CATCHER_PITCH'])), 0.0)      # ground of a slope (rises towards +x)
    if MODE == 'sun':
        for o in bpy.data.objects:
            if o.type == 'LIGHT': o.hide_render = True
        az, el = (float(v) for v in os.environ.get('SUN', '150:14').split(':'))
        L = bpy.data.lights.new('Sun', 'SUN'); L.energy = float(os.environ.get('SUN_E', '4.5')); L.angle = math.radians(float(os.environ.get('SUN_ANGLE', '1.5'))); L.color = (1.0, 0.86, 0.68)
        so = bpy.data.objects.new('Sun', L); sc.collection.objects.link(so)
        d = mathutils.Vector((math.cos(math.radians(el)) * math.cos(math.radians(az)), math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))))
        so.rotation_euler = (-d).to_track_quat('-Z', 'Y').to_euler()
        w = sc.world; w.use_nodes = True; w.node_tree.nodes["Background"].inputs[0].default_value = (0.42, 0.52, 0.66, 1); w.node_tree.nodes["Background"].inputs[1].default_value = float(os.environ.get('SKY', '0.55'))
        if os.environ.get('FILL'):      # soft fill from behind the camera (a bright garden / sky in front of the mower)
            Lf = bpy.data.lights.new('SunFill', 'AREA'); Lf.energy = float(os.environ['FILL']); Lf.size = 3.0; Lf.color = (1.0, 0.95, 0.88)
            fo = bpy.data.objects.new('SunFill', Lf); sc.collection.objects.link(fo); fo['follow_cam'] = True
        sc.view_settings.look = 'AgX - Base Contrast'
if os.environ.get('MASK') == 'wheels':      # wheel mask pass: everything else held out (still occludes), no shadow catcher
    keep_ = ('Wheel_L', 'Wheel_R', 'Caster_L', 'Caster_R')
    for o in bpy.data.objects:
        if o.type != 'MESH' or o.name == 'Backdrop': continue
        top_ = o
        while top_.parent: top_ = top_.parent
        o.is_holdout = top_.name not in keep_
    sc.render.film_transparent = True
    for nm_ in ('ShadowCatcher', 'Backdrop'):
        if bpy.data.objects.get(nm_): bpy.data.objects[nm_].hide_render = True
sc.render.engine = 'CYCLES'; sc.cycles.samples = SPP; sc.cycles.use_denoising = True
try:
    pr = bpy.context.preferences.addons['cycles'].preferences; pr.compute_device_type = 'OPTIX'; pr.get_devices()
    for dv in pr.devices: dv.use = (dv.type == 'OPTIX')
    sc.cycles.device = 'GPU' if any(dv.use for dv in pr.devices) else 'CPU'
except Exception: sc.cycles.device = 'CPU'
if os.environ.get('EXPOSURE'): sc.view_settings.exposure = float(os.environ['EXPOSURE'])
aw, ah = (float(v) for v in os.environ.get('ASPECT', '3:2').split(':'))
sc.render.resolution_x = RES; sc.render.resolution_y = int(round(RES * ah / aw)); sc.render.film_transparent = sc.render.film_transparent
cam = sc.camera
VW = {'hero': ((1.35, -1.25, 0.95), (0.0, 0.0, 0.11), 70), 'front': ((1.9, -0.45, 0.7), (0.0, 0.0, 0.11), 70), 'side': ((0.05, -2.0, 0.5), (0.0, 0.0, 0.12), 70),
      'rear': ((-1.35, -1.25, 0.9), (0.0, 0.0, 0.11), 70), 'top': ((0.5, -0.95, 2.5), (0.0, 0.0, 0.1), 70), 'panel': ((-0.62, -0.12, 0.78), (-0.07, 0.0, 0.245), 90),
      'sensor': ((0.78, -0.22, 0.36), (0.29, 0.0, 0.2), 90), 'wheel': ((0.42, -0.85, 0.22), (0.12, -0.2, 0.115), 90), 'dockhero': ((1.6, -1.45, 1.0), (0.05, 0.0, 0.12), 70),
      'dockpins': ((0.55, -0.32, 0.36), (0.33, -0.02, 0.16), 110), 'stopclose': ((0.12, -0.2, 0.5), (0.005, 0.0, 0.255), 110)}
for cv in filter(None, os.environ.get('CUSTOM_VIEWS', '').split(';')):
    k, v = cv.split('='); x = [float(t) for t in v.split(':')]; VW[k] = ((x[0], x[1], x[2]), (x[3], x[4], x[5]), x[6])
PITCH = {k: float(v) for k, v in (t.split('=') for t in filter(None, os.environ.get('PITCH', '').split(',')))}
M0 = {o.name: o.matrix_world.copy() for o in MOWER}
def shoot(prefix):
    for v in VIEWS:
        loc, tgt, lens = VW[v]; cam.location = loc; cam.data.lens = lens
        cam.rotation_euler = (mathutils.Vector(tgt) - mathutils.Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
        R = mathutils.Matrix.Rotation(math.radians(-PITCH.get(v, 0.0)), 4, 'Y')
        for o in MOWER: o.matrix_world = R @ M0[o.name]
        fo = bpy.data.objects.get('SunFill')
        if fo:      # 2.5 m out on the camera side, 1.2 m up, aimed at the mower
            dv = (mathutils.Vector(loc) - mathutils.Vector(tgt)); dv.z = 0; dv.normalize(); fo.location = mathutils.Vector(tgt) + dv * 2.5 + mathutils.Vector((0, 0, 1.2))
            fo.rotation_euler = (mathutils.Vector(tgt) - fo.location).to_track_quat('-Z', 'Y').to_euler()
        sc.render.filepath = f"{prefix}_{v}.png"; bpy.ops.render.render(write_still=True)
if os.environ.get('PALETTE_LIST'):
    want = os.environ['PALETTE_LIST']; sel = list(PALS.values()) if want == 'all' else [PALS[i] for i in want.split(',')]
    for p in sel: apply_palette(p); shoot(f"{OUT}_{p['id']}")
else:
    if os.environ.get('PALETTE'): apply_palette(PALS[os.environ['PALETTE']])
    shoot(OUT)
