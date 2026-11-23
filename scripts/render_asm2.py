import bpy, sys, numpy as np, mathutils, math, os
argv=sys.argv[sys.argv.index('--')+1:]
WD,OUT,VIEWS,SAMPLES,RES=argv[0],argv[1],argv[2].split(','),int(argv[3]),int(argv[4])
SAVE=argv[5] if len(argv)>5 else None
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
import mower_scene
bpy.ops.wm.read_factory_settings(use_empty=True)
COLOR=os.environ.get("COLOR","black")
DOCK=bool(os.environ.get("DOCK"))
objs,M=mower_scene.build(WD,color=COLOR,dock=DOCK)
if DOCK:                                   # mower parked on the charging base
    for k,o in objs.items():
        if k.startswith('_') or k=='Dock' or o.parent is not None: continue
        o.location.z+=mower_scene.DOCK_LIFT
PAL=os.environ.get("PALETTE")
if PAL:
    pals={p['id']:p for p in mower_scene.load_palettes(os.environ.get("PALETTES"))['schemes']}
    mower_scene.apply_palette(M,pals[PAL])
if os.environ.get("EXPLODE"):
    e=float(os.environ["EXPLODE"])
    objs['Shell'].location.z+=0.46*e; objs['Middle'].location.z+=0.26*e   # same lifts as the web viewer
if COLOR=="stripes":
    mat=M['ShellPaint']; nt=mat.node_tree; N=nt.nodes; Lk=nt.links; b=N["Principled BSDF"]
    tc=N.new("ShaderNodeTexCoord"); sp=N.new("ShaderNodeSeparateXYZ"); Lk.new(tc.outputs["Object"],sp.inputs[0])
    ab=N.new("ShaderNodeMath"); ab.operation="ABSOLUTE"; Lk.new(sp.outputs["Y"],ab.inputs[0])
    g=N.new("ShaderNodeMath"); g.operation="GREATER_THAN"; g.inputs[1].default_value=0.011; Lk.new(ab.outputs[0],g.inputs[0])
    l=N.new("ShaderNodeMath"); l.operation="LESS_THAN"; l.inputs[1].default_value=0.056; Lk.new(ab.outputs[0],l.inputs[0])
    mm=N.new("ShaderNodeMath"); mm.operation="MULTIPLY"; Lk.new(g.outputs[0],mm.inputs[0]); Lk.new(l.outputs[0],mm.inputs[1])
    mx=N.new("ShaderNodeMix"); mx.data_type="RGBA"; Lk.new(mm.outputs[0],mx.inputs["Factor"])
    mx.inputs["A"].default_value=(*mower_scene.PAINT["stripes"],1); mx.inputs["B"].default_value=(0.002,0.018,0.17,1)
    Lk.new(mx.outputs["Result"],b.inputs["Base Color"])
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
if os.environ.get('LOOK'): sc.view_settings.look=os.environ['LOOK']
if os.environ.get('EXPOSURE'): sc.view_settings.exposure=float(os.environ['EXPOSURE'])
sc.render.resolution_x=RES; sc.render.resolution_y=int(RES*2/3)
VW={'hero':((1.35,-1.25,0.95),(0.0,0.0,0.11),70),
    'front':((1.9,-0.45,0.7),(0.0,0.0,0.11),70),
    'side':((0.05,-2.0,0.5),(0.0,0.0,0.12),70),
    'rear':((-1.35,-1.25,0.9),(0.0,0.0,0.11),70),
    'top':((0.5,-0.95,2.5),(0.0,0.0,0.1),70),
    'panel':((-0.62,-0.12,0.78),(-0.07,0.0,0.245),90),
    'sensor':((0.78,-0.22,0.36),(0.29,0.0,0.2),90),
    'wheel':((0.42,-0.85,0.22),(0.12,-0.2,0.115),90),
    'exploded':((2.15,-1.95,1.5),(0.0,0.0,0.34),62),
    'gaprear':((-0.72,-0.42,0.42),(-0.2,-0.08,0.17),70),
    'gapside':((0.0,-0.75,0.28),(0.05,-0.15,0.18),70),
    'led':((0.66,-0.2,0.27),(0.29,-0.12,0.168),110),
    'ledL':((0.66,0.2,0.27),(0.29,0.12,0.168),110),
    'ledgraze':((0.42,-0.36,0.19),(0.29,-0.12,0.168),120),
    'frontlow':((1.25,0.0,0.3),(0.0,0.0,0.15),70),
    'dockhero':((1.6,-1.45,1.0),(0.05,0.0,0.12),70),
    'dockside':((0.15,-1.25,0.32),(0.2,0.0,0.12),85),
    'dockpins':((0.55,-0.32,0.36),(0.33,-0.02,0.16),110),
    'stopclose':((0.12,-0.2,0.5),(0.005,0.0,0.255),110),
    'podclose':((0.55,-0.25,0.42),(0.26,-0.02,0.22),110),
    'sidelow':((-0.25,-0.85,0.2),(-0.1,-0.1,0.15),80),
    'rearlow':((-0.95,-0.35,0.3),(-0.2,0.0,0.15),75)}
def shoot(prefix):
    for v in VIEWS:
        loc,tgt,lens=VW[v]; cam.location=loc; cam.data.lens=lens
        dxy=mathutils.Vector(tgt)-mathutils.Vector(loc); cyc.rotation_euler.z=math.atan2(dxy.y,dxy.x)-math.pi/2
        cam.rotation_euler=(mathutils.Vector(tgt)-mathutils.Vector(loc)).to_track_quat('-Z','Y').to_euler()
        sc.render.filepath=f"{prefix}_{v}.png"; bpy.ops.render.render(write_still=True)
# PALETTE_LIST=id1,id2,... (or "all"): one scene build, every listed colour scheme rendered in turn
if os.environ.get("PALETTE_LIST"):
    allp=mower_scene.load_palettes(os.environ.get("PALETTES"))['schemes']
    want=os.environ["PALETTE_LIST"]; sel=allp if want=="all" else [p for p in allp if p['id'] in want.split(',')]
    for p in sel:
        mower_scene.apply_palette(M,p); shoot(f"{OUT}_{p['id']}")
else:
    shoot(OUT)
if SAVE: bpy.ops.wm.save_as_mainfile(filepath=SAVE)
