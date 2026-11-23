import bpy, numpy as np
def load(path, name="shell"):
    d=np.load(path); V=d['v']; F=d['f']
    m=bpy.data.meshes.new(name)
    m.vertices.add(len(V)); m.vertices.foreach_set("co",V.astype(np.float32).ravel())
    m.loops.add(F.size); m.loops.foreach_set("vertex_index",F.astype(np.int32).ravel())
    m.polygons.add(len(F)); m.polygons.foreach_set("loop_start",np.arange(0,F.size,3,dtype=np.int32)); m.polygons.foreach_set("loop_total",np.full(len(F),3,np.int32))
    m.update(); m.validate()
    o=bpy.data.objects.new(name,m); bpy.context.scene.collection.objects.link(o)
    extra={k:d[k] for k in d.files if k not in('v','f')}
    return o, extra
