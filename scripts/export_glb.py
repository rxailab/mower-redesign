"""Export decimated, web-ready GLB models for the viewer.
usage: blender -b --factory-startup -P export_glb.py -- WORKDIR OUTDIR"""
import bpy, sys, os, json, mathutils
argv = sys.argv[sys.argv.index('--') + 1:]
WD, OUT = argv[0], argv[1]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mower_scene
bpy.ops.wm.read_factory_settings(use_empty=True)
LOD = {'Shell': 0.045, 'Middle': 0.075, 'Chassis': 0.024, 'Wheel': 0.028, 'Caster': 0.04, 'Dock': 0.05}
objs, M = mower_scene.build(WD, color='black', lod=LOD, dock=True)
g2 = lambda v: [float(v[0]), float(v[2]), float(-v[1])]          # Blender Z-up -> glTF Y-up
def empty(name, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None); bpy.context.scene.collection.objects.link(e)
    e.location = loc; e.parent = parent; return e
top = empty('Top'); mid = empty('MiddleLayer'); base = empty('Base'); dockroot = empty('DockRoot')
objs['Shell'].parent = top
objs['Middle'].parent = mid
objs['Chassis'].parent = base
if 'Dock' in objs: objs['Dock'].parent = dockroot
info = {}
for nm in 'LR':
    w = objs[f'Wheel_{nm}']; w.parent = base
    w['role'] = 'wheel'; w['axis'] = g2(w['axis_world']); del w['axis_world']
    c = objs[f'Caster_{nm}']; pv = list(c['pivot_world']); cl = c.location.copy()
    sw = empty(f'CasterSwivel_{nm}', loc=pv, parent=base); sw['role'] = 'swivel'
    c.parent = sw; c.location = cl - mathutils.Vector(pv)
    c['role'] = 'caster'; c['axis'] = g2(c['axis_world']); del c['axis_world']; del c['pivot_world']
    info[nm] = {'wheel_center': g2(w.location), 'wheel_axis': list(w['axis']), 'caster_pivot': g2(pv),
                'caster_center': g2(cl), 'caster_axis': list(c['axis'])}
for o in bpy.data.objects:
    if o.type == 'MESH':
        print(f"{o.name:16s} faces {len(o.data.polygons):9d}")
def descendants(o):
    out = [o]
    for ch in o.children: out += descendants(ch)
    return out
def export(root, path):
    bpy.ops.object.select_all(action='DESELECT')
    for o in descendants(root): o.select_set(True)
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True,
                              export_extras=True, export_apply=True, export_texcoords=True, export_normals=True,
                              export_materials='EXPORT', export_image_format='AUTO', export_cameras=False,
                              export_lights=False, export_animations=False)
    print("exported", path, os.path.getsize(path) // 1024, "KB")
os.makedirs(OUT, exist_ok=True)
export(top, os.path.join(OUT, 'top.glb'))
export(mid, os.path.join(OUT, 'middle.glb'))
export(base, os.path.join(OUT, 'base.glb'))
if 'Dock' in objs: export(dockroot, os.path.join(OUT, 'dock.glb'))
info['dock_lift'] = mower_scene.DOCK_LIFT
json.dump(info, open(os.path.join(OUT, 'rig.json'), 'w'), indent=1)
