"""Export only the charging base as models/dock.glb (fast; the mower GLBs are untouched).
usage: blender -b --factory-startup -P export_dock.py -- WORKDIR OUTDIR"""
import bpy, sys, os, json, numpy as np
a = sys.argv[sys.argv.index('--') + 1:]; WD, OUT = a[0], a[1]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mower_scene as ms
bpy.ops.wm.read_factory_settings(use_empty=True)
M = ms.make_materials(WD)
dd = np.load(os.path.join(WD, 'asm_dock.npz'))
o = ms.mesh_obj('Dock', dd['v'], dd['f'], mat_idx=dd['lab'])
for mn in ('DockBody', 'DockPin'): o.data.materials.append(M[mn])
ms.decimate(o, 0.05); ms.smooth_by_angle(o, 35)
root = bpy.data.objects.new('DockRoot', None); bpy.context.scene.collection.objects.link(root); o.parent = root
bpy.ops.object.select_all(action='DESELECT'); root.select_set(True); o.select_set(True)
p = os.path.join(OUT, 'dock.glb')
bpy.ops.export_scene.gltf(filepath=p, export_format='GLB', use_selection=True, export_yup=True, export_apply=True,
                          export_normals=True, export_materials='EXPORT', export_cameras=False, export_lights=False, export_animations=False)
print('exported dock.glb', os.path.getsize(p) // 1024, 'KB', len(o.data.polygons), 'faces')
