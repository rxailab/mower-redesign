"""Export only base.glb (chassis, drive wheels, casters, ring decals) for the viewer.
The hub dish of each drive wheel gets analytic shading normals from the fitted surface of revolution
(wheel_dish.json, written by smooth_hub_rim.py), so the decimated dish reflects the environment smoothly.
usage: blender -b --factory-startup -P export_base.py -- WORKDIR OUTDIR"""
import bpy, sys, os, json, mathutils
import numpy as np
argv = sys.argv[sys.argv.index('--') + 1:]
WD, OUT = argv[0], argv[1]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mower_scene
bpy.ops.wm.read_factory_settings(use_empty=True)
LOD = {'Shell': 0.045, 'Middle': 0.075, 'Chassis': 0.024, 'Wheel': 0.028, 'Caster': 0.04, 'Dock': 0.05}
objs, M = mower_scene.build(WD, color='black', lod=LOD, dock=False)
g2 = lambda v: [float(v[0]), float(v[2]), float(-v[1])]          # Blender Z-up -> glTF Y-up
def empty(name, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None); bpy.context.scene.collection.objects.link(e)
    e.location = loc; e.parent = parent; return e
base = empty('Base')
objs['Chassis'].parent = base
DISH = json.load(open(os.path.join(WD, 'wheel_dish.json')))
for nm in 'LR':
    w = objs[f'Wheel_{nm}']; w.parent = base
    w['role'] = 'wheel'; w['axis'] = g2(w['axis_world']); del w['axis_world']
    c = objs[f'Caster_{nm}']; pv = list(c['pivot_world']); cl = c.location.copy()
    sw = empty(f'CasterSwivel_{nm}', loc=pv, parent=base); sw['role'] = 'swivel'
    c.parent = sw; c.location = cl - mathutils.Vector(pv)
    c['role'] = 'caster'; c['axis'] = g2(c['axis_world']); del c['axis_world']; del c['pivot_world']
    # analytic dish normals: t = f(r) + b x + cc y  ->  n ~ ax - f'(r) rhat - b e1 - cc e2
    D = DISH[nm]; me = w.data
    ax, e1, e2 = (np.array(D[k]) for k in ('ax', 'e1', 'e2'))
    bins = np.array(D['bins']); prof = np.array(D['prof']); dprof = np.gradient(prof, bins)
    co = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get('co', co); P = co.reshape(-1, 3) + np.array(w.location[:]) - np.array(D['c'])
    t = P @ ax; x = P @ e1; y = P @ e2; r = np.hypot(x, y)
    tgt = np.interp(r, bins, prof) + D['b'] * x + D['cc'] * y
    rh = (x[:, None] * e1 + y[:, None] * e2) / np.maximum(r, 1e-9)[:, None]
    G = ax[None, :] - np.interp(r, bins, dprof)[:, None] * rh - D['b'] * e1 - D['cc'] * e2
    G /= np.linalg.norm(G, axis=1, keepdims=True)
    cn = np.zeros(len(me.loops) * 3); me.corner_normals.foreach_get('vector', cn); cn = cn.reshape(-1, 3)
    lv = np.zeros(len(me.loops), np.int64); me.loops.foreach_get('vertex_index', lv)
    lp = np.zeros(len(me.loops), np.int64)
    for p in me.polygons: lp[p.loop_start:p.loop_start + p.loop_total] = p.index
    fn = np.zeros(len(me.polygons) * 3); me.polygons.foreach_get('normal', fn); fn = fn.reshape(-1, 3)
    on = (np.abs(t - tgt) < 0.0008) & (t > 0.012) & (r < 0.0806)
    wv = np.clip((0.0806 - r) / 0.0008, 0, 1) * on
    wl = wv[lv] * (fn[lp] @ ax > 0.3)                       # only corners of faces that look out along the axle
    new = cn * (1 - wl[:, None]) + G[lv] * wl[:, None]
    new /= np.linalg.norm(new, axis=1, keepdims=True)
    dev = np.degrees(np.arccos(np.clip(np.sum(cn * G[lv], 1), -1, 1)))[wl > 0.99]
    print(f"Wheel_{nm}: dish corners {int((wl > 0.99).sum())} of {len(lv)}, old normal deviation deg mean {dev.mean():.2f} p95 {np.percentile(dev, 95):.2f}")
    me.normals_split_custom_set(new.tolist())
def descendants(o):
    out = [o]
    for ch in o.children: out += descendants(ch)
    return out
bpy.ops.object.select_all(action='DESELECT')
for o in descendants(base): o.select_set(True)
path = os.path.join(OUT, 'base.glb')
bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True,
                          export_extras=True, export_apply=True, export_texcoords=True, export_normals=True,
                          export_materials='EXPORT', export_image_format='AUTO', export_cameras=False,
                          export_lights=False, export_animations=False)
print("exported", path, os.path.getsize(path), "bytes")
