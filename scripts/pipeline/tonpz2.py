import bpy, numpy as np, sys
a=sys.argv[sys.argv.index('--')+1:]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=a[0], forward_axis='Y', up_axis='Z')
m=bpy.context.selected_objects[0].data
n=len(m.vertices); co=np.empty(n*3); m.vertices.foreach_get("co",co)
m.calc_loop_triangles()
f=np.empty(len(m.loop_triangles)*3,dtype=np.int32); m.loop_triangles.foreach_get("vertices",f)
np.savez(a[1],v=co.reshape(-1,3),f=f.reshape(-1,3)); print("OK",n,len(f)//3)
