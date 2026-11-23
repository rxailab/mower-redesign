import bpy, sys, numpy as np, mathutils, math, json, os
argv=sys.argv[sys.argv.index('--')+1:]
WD,PARTS,OUT,VIEWS,SAMPLES,RES=argv[0],argv[1],argv[2],argv[3].split(','),int(argv[4]),int(argv[5])
SAVE=argv[6] if len(argv)>6 else None
sys.path.insert(0,WD); import meshio_b
bpy.ops.wm.read_factory_settings(use_empty=True)
DEC=float(os.environ.get("DECIMATE","1.0"))
def mkmat(name,col,rough,coat=0.0,coatr=0.1,spec=0.5):
    mt=bpy.data.materials.new(name); mt.use_nodes=True; p=mt.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value=(*col,1); p.inputs["Roughness"].default_value=rough
    p.inputs["Specular IOR Level"].default_value=spec; p.inputs["Coat Weight"].default_value=coat; p.inputs["Coat Roughness"].default_value=coatr
    return mt,p
def loadpart(name):
    o,_=meshio_b.load(f"{PARTS}/asm_{name}.npz",name); m=o.data
    n=np.empty(len(m.polygons)*3); m.polygons.foreach_get("normal",n); n=n.reshape(-1,3)
    c=np.empty(len(m.polygons)*3); m.polygons.foreach_get("center",c); c=c.reshape(-1,3)
    top=c[:,2]>np.percentile(c[:,2],90)
    if n[top,2].mean()<0: m.flip_normals(); print(name,"flipped")
    m.polygons.foreach_set("use_smooth",np.ones(len(m.polygons),bool)); m.update()
    if DEC<1.0 and name!="shell":
        d=o.modifiers.new("Dec",'DECIMATE'); d.ratio=DEC
        bpy.context.view_layer.objects.active=o; o.select_set(True); bpy.ops.object.modifier_apply(modifier="Dec"); o.select_set(False)
    return o
o=loadpart("shell")
sol=o.modifiers.new("Solidify",'SOLIDIFY'); sol.thickness=float(os.environ.get("THICK","0.0025")); sol.offset=-1; sol.use_even_offset=False; sol.thickness_clamp=1.0; sol.use_rim=True; sol.use_quality_normals=True
mat,b=mkmat("ShellPaint",(0.015,0.015,0.017),0.4,0.8,0.07)
COLOR=os.environ.get("COLOR","black")
PAINT={"black":(0.015,0.015,0.017),"yellow":(0.88,0.58,0.0),"blue":(0.0,0.035,0.33),"red":(0.42,0.0,0.004),"stripes":(0.78,0.78,0.76)}
b.inputs["Base Color"].default_value=(*PAINT[COLOR],1)
if COLOR!="black": b.inputs["Roughness"].default_value=0.3
if COLOR=="stripes":
    nt=mat.node_tree; N=nt.nodes; Lk=nt.links
    tc=N.new("ShaderNodeTexCoord"); sp=N.new("ShaderNodeSeparateXYZ"); Lk.new(tc.outputs["Object"],sp.inputs[0])
    ab=N.new("ShaderNodeMath"); ab.operation="ABSOLUTE"; Lk.new(sp.outputs["Y"],ab.inputs[0])
    g=N.new("ShaderNodeMath"); g.operation="GREATER_THAN"; g.inputs[1].default_value=0.011; Lk.new(ab.outputs[0],g.inputs[0])
    l=N.new("ShaderNodeMath"); l.operation="LESS_THAN"; l.inputs[1].default_value=0.056; Lk.new(ab.outputs[0],l.inputs[0])
    mm=N.new("ShaderNodeMath"); mm.operation="MULTIPLY"; Lk.new(g.outputs[0],mm.inputs[0]); Lk.new(l.outputs[0],mm.inputs[1])
    mx=N.new("ShaderNodeMix"); mx.data_type="RGBA"; Lk.new(mm.outputs[0],mx.inputs["Factor"])
    mx.inputs["A"].default_value=(*PAINT["stripes"],1); mx.inputs["B"].default_value=(0.002,0.018,0.17,1)
    Lk.new(mx.outputs["Result"],b.inputs["Base Color"])
o.data.materials.append(mat)
MIDC={"black":(0.045,0.047,0.05)}.get(COLOR,(0.03,0.031,0.034))
loadpart("middle").data.materials.append(mkmat("MidPlastic",MIDC,0.5)[0])
loadpart("chassis").data.materials.append(mkmat("Chassis",(0.02,0.02,0.022),0.6)[0])
rub=mkmat("Rubber",(0.028,0.028,0.028),0.72,spec=0.35)[0]
loadpart("wheels").data.materials.append(rub); loadpart("casters").data.materials.append(rub)
# cyclorama backdrop
bpy.ops.mesh.primitive_plane_add(size=1); bd=bpy.context.active_object; bd.name="Backdrop"
bm=bd.data; import bmesh
bmo=bmesh.new(); 
prof=[]; R=0.6
for i in range(0,21):
    a=math.radians(90*i/20); prof.append((-1.5+R-R*math.sin(a) if False else 0, 0))
bmo.free()
bpy.data.objects.remove(bd)
verts=[];faces=[]
pts=[(y,0.0) for y in np.linspace(-4,1.0,10)]+[(1.0+R*math.sin(math.radians(a)),R-R*math.cos(math.radians(a))) for a in range(5,91,5)]+[(1.0+R,z) for z in np.linspace(R+0.3,4,6)]
W=6
for (y,z) in pts: verts+= [(-W,y,z),(W,y,z)]
for i in range(len(pts)-1): faces.append((2*i,2*i+1,2*i+3,2*i+2))
me=bpy.data.meshes.new("cyc"); me.from_pydata(verts,[],faces); me.update()
cyc=bpy.data.objects.new("Backdrop",me); bpy.context.scene.collection.objects.link(cyc)
for p in me.polygons: p.use_smooth=True
bm_=bpy.data.materials.new("Floor"); bm_.use_nodes=True; fb=bm_.node_tree.nodes["Principled BSDF"]
fb.inputs["Base Color"].default_value=(0.55,0.55,0.56,1); fb.inputs["Roughness"].default_value=0.7
me.materials.append(bm_)
sc=bpy.context.scene
# lights: big softbox key, strip rims, fill
def area(name,loc,size,energy,shape='RECTANGLE',sy=None):
    L=bpy.data.lights.new(name,'AREA'); L.energy=energy; L.shape=shape; L.size=size; L.cycles.max_bounces=64
    if sy: L.size_y=sy
    ob=bpy.data.objects.new(name,L); sc.collection.objects.link(ob); ob.location=loc; ob.visible_camera=False
    d=mathutils.Vector((0,0,0.05))-mathutils.Vector(loc); ob.rotation_euler=d.to_track_quat('-Z','Y').to_euler(); return ob
area("Key",(-0.15,-0.3,1.7),1.8,90,'RECTANGLE',1.2)
area("StripL",(-1.2,-0.6,0.7),0.5,14,'RECTANGLE',1.6)
area("StripR",(1.2,0.3,0.8),0.5,14,'RECTANGLE',1.6)
area("Back",(-0.3,1.3,1.0),1.5,18,'RECTANGLE',0.3)
area("Fill",(0.2,-1.8,0.6),2.0,6)
w=bpy.data.worlds.new("W"); sc.world=w; w.use_nodes=True; w.node_tree.nodes["Background"].inputs[0].default_value=(0.035,0.035,0.038,1)
cam=bpy.data.objects.new("Cam",bpy.data.cameras.new("Cam")); sc.collection.objects.link(cam); sc.camera=cam
cam.data.lens=70
sc.render.engine='CYCLES'; sc.cycles.device='GPU'
prefs=bpy.context.preferences.addons['cycles'].preferences; prefs.compute_device_type='OPTIX'; prefs.get_devices()
for dv in prefs.devices: dv.use=(dv.type=='OPTIX')
sc.cycles.samples=SAMPLES; sc.cycles.use_denoising=True
sc.view_settings.view_transform='AgX'; sc.view_settings.look='AgX - Base Contrast'; sc.view_settings.exposure=-1.1
if os.environ.get('COLOR','black') in ('yellow','blue','red'): sc.view_settings.look='AgX - Punchy'
sc.render.resolution_x=RES; sc.render.resolution_y=int(RES*2/3)
VW={'hero':((-1.25,-1.35,0.95),(0.0,0.0,0.11),70),
    'front':((-1.9,-0.55,0.7),(0.0,0.0,0.11),70),
    'rear':((1.35,-1.25,0.9),(0.0,0.0,0.11),70),
    'side':((0.05,-2.0,0.5),(0.0,0.0,0.12),70),
    'top':((-0.5,-0.95,2.5),(0.0,0.0,0.1),70),
    'crear':((0.75,-0.45,0.45),(0.28,0.0,0.17),85),
    'crear2':((0.8,0.35,0.35),(0.3,0.0,0.15),85),
    'cside':((0.2,-0.8,0.5),(0.2,-0.15,0.17),85),
    'cfront':((0.55,-0.3,0.4),(0.25,0.0,0.065),100),
    'cfront2':((0.6,0.25,0.3),(0.26,0.0,0.06),100),
    'ctail':((-0.8,-0.3,0.3),(-0.29,0.0,0.07),100),
    'ctail2':((-0.7,0.35,0.2),(-0.27,0.08,0.06),100),
    'cstrip':((-0.15,0.4,0.4),(-0.14,0.097,0.1),100),
    'close':((0.30,-0.42,0.30),(0.12,-0.08,0.07),70),
    'closerim':((-0.35,-0.45,0.22),(-0.17,-0.15,0.05),70)}
for v in VIEWS:
    loc,tgt,lens=VW[v]; cam.location=loc; cam.data.lens=lens
    dxy=mathutils.Vector(tgt)-mathutils.Vector(loc); cyc.rotation_euler.z=math.atan2(dxy.y,dxy.x)-math.pi/2
    cam.rotation_euler=(mathutils.Vector(tgt)-mathutils.Vector(loc)).to_track_quat('-Z','Y').to_euler()
    sc.render.filepath=f"{OUT}_{v}.png"; bpy.ops.render.render(write_still=True)
if SAVE: bpy.ops.wm.save_as_mainfile(filepath=SAVE)
