"""Export the redesigned upper body for the web viewer: models/body.glb, lid.glb, lower.glb, design.json, hotspots.json.
usage: blender -b --factory-startup -P export_design_web.py -- DESIGN_DIR PARAMS.json SHELL.npz OUTDIR
DESIGN_DIR holds keep.npz (lower skirt + bumper, as scanned) and control.npz (STOP key, display)."""
import bpy, sys, os, json, math, numpy as np
a = sys.argv[sys.argv.index('--') + 1:]; DD, PJ, NPZ, OUT = a[0], a[1], a[2], a[3]
P = json.load(open(PJ)); col = P.get('colors', {}); d = np.load(NPZ)
bpy.ops.wm.read_factory_settings(use_empty=True); sc = bpy.context.scene

def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]; return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
def mat(name, hexc, rough, metal=0.0, coat=0.0, emit=0.0):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = rough; b.inputs['Metallic'].default_value = metal
    b.inputs['Coat Weight'].default_value = coat; b.inputs['Coat Roughness'].default_value = 0.06
    if emit: b.inputs['Emission Color'].default_value = (*lin(hexc), 1); b.inputs['Emission Strength'].default_value = emit
    return m
M = {'BodyPaint': mat('BodyPaint', col.get('body', '#B4B9C0'), 0.34, 0.6, 1.0), 'DeckPaint': mat('DeckPaint', col.get('roof', '#0B0C0E'), 0.28, 0, 0.8),
     'Visor': mat('Visor', col.get('visor', '#0C0D0F'), 0.5), 'LampLight': mat('LampLight', col.get('light', '#F2FBFF'), 0.3, emit=1.0),
     'LidPaint': mat('LidPaint', col.get('lid', col.get('body', '#B4B9C0')), 0.34, 0.6, 1.0), 'Accent': mat('Accent', col.get('accent', '#FF6A2B'), 0.5, 0, 0.1),
     'Glass': mat('Glass', '#0A0B0D', 0.2, 0, 0.8), 'SkirtPaint': mat('SkirtPaint', col.get('skirt', '#17181B'), 0.55), 'Dark': mat('Dark', '#0B0B0C', 0.72),
     'StopButton': mat('StopButton', '#E8552F', 0.45), 'DisplayGlass': mat('DisplayGlass', '#0A0B0D', 0.2, 0, 0.6),
     'KitDark': mat('KitDark', '#33363B', 0.5), 'KitRubber': mat('KitRubber', '#070708', 0.75), 'KitSteel': mat('KitSteel', '#C9CDD2', 0.22, 1.0),
     'KitArm': mat('KitArm', '#595D65', 0.34, 0.6), 'KitLiner': mat('KitLiner', '#6E727A', 0.5), 'KitStrut': mat('KitStrut', '#0E0F11', 0.28, 0, 0.8),
     'LensBezel': mat('LensBezel', '#7C828A', 0.3, 0.8), 'LensGlass': mat('LensGlass', '#05070D', 0.05, 0, 1.0)}

def obj(name, V, F, mats, midx=None, normals=None, smooth_deg=35, weld=True, ratio=None):
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
    o.select_set(False); print(f'{name:12s} {len(o.data.polygons):8d} faces'); return o
def empty(name):
    e = bpy.data.objects.new(name, None); sc.collection.objects.link(e); return e

lab = d['lab']; SH = ['BodyPaint', 'DeckPaint', 'Visor', 'LampLight', 'LidPaint', 'Accent', 'Glass']
KIT = ['KitDark', 'KitRubber', 'KitSteel', 'KitArm', 'KitLiner', 'KitStrut', 'LidPaint']
body = empty('BodyRoot'); lidr = empty('LidRoot'); lower = empty('LowerRoot')
def compact(V, F):
    u = np.unique(F); rm = -np.ones(len(V), np.int64); rm[u] = np.arange(len(u)); return V[u], rm[F]
v_, f_ = compact(d['v'], d['f'][lab != 4]); obj('NewShell', v_, f_, SH, lab[lab != 4], normals=d['ln'][lab != 4]).parent = body
obj('Camera', d['cam_v'], d['cam_f'], ['LensBezel', 'Dark', 'LensGlass', 'LampLight'], d['cam_lab'], smooth_deg=50).parent = body
obj('PortSleeves', d['sleeve_v'], d['sleeve_f'], ['Dark'], smooth_deg=30).parent = body
obj('LidWell', d['kitf_v'], d['kitf_f'], KIT, d['kitf_lab']).parent = body
c = np.load(os.path.join(DD, 'control.npz')); cl = c['lab']
obj('Controls', c['v'], c['f'], ['Dark', 'StopButton', 'DisplayGlass'], np.where(cl == 1, 1, np.where(cl == 2, 2, 0)), smooth_deg=40, ratio=0.3).parent = body
v_, f_ = compact(d['v'], d['f'][lab == 4]); obj('LidSkin', v_, f_, ['LidPaint'], normals=d['ln'][lab == 4]).parent = lidr
obj('LidKit', d['kitm_v'], d['kitm_f'], KIT, d['kitm_lab']).parent = lidr
k = np.load(os.path.join(DD, 'keep.npz')); kl = k['lab']; kc = k['v'][k['f']].mean(1); kk = ~((kl == 6) & (kc[:, 0] > 0.27))
if P.get('nose'): kk &= ~((kl == 0) & (kc[:, 0] > 0.255))
v_, f_ = compact(k['v'], k['f'][kk])
obj('Skirt', v_, f_, ['SkirtPaint', 'Dark'], np.where((kl == 7) | (kl == 6), 1, 0)[kk], smooth_deg=40, weld=False, ratio=0.085).parent = lower
if 'plinth_v' in d.files: obj('Plinth', d['plinth_v'], d['plinth_f'], ['SkirtPaint'], smooth_deg=30).parent = lower

def export(root, path):
    bpy.ops.object.select_all(action='DESELECT'); root.select_set(True)
    for o in root.children: o.select_set(True)
    bpy.ops.export_scene.gltf(filepath=path, export_format='GLB', use_selection=True, export_yup=True, export_apply=True, export_normals=True,
                              export_texcoords=False, export_materials='EXPORT', export_cameras=False, export_lights=False, export_animations=False)
    print('exported', os.path.basename(path), os.path.getsize(path) // 1024, 'KB')
os.makedirs(OUT, exist_ok=True)
export(body, os.path.join(OUT, 'body.glb')); export(lidr, os.path.join(OUT, 'lid.glb')); export(lower, os.path.join(OUT, 'lower.glb'))

# ---- numbers the viewer needs (glTF axes: x forward, y up, z = the mower's right)
g2 = lambda p: [round(float(p[0]), 5), round(float(p[2]), 5), round(float(-p[1]), 5)]
V = d['v']; lv = V[np.unique(d['f'][lab == 4])]; pv = d['lid_pivot']
def face_x(y, z, r=0.004):          # frontmost body point near (y, z) on the nose
    m = (V[:, 0] > 0.2) & (np.abs(V[:, 1] - y) < r) & (np.abs(V[:, 2] - z) < r); return float(V[m, 0].max())
info = {'lid': {'pivot': [round(float(pv[0]), 5), round(float(pv[1]), 5), 0.0], 'open_deg': float(d['lid_open'][0]),
                'strut_n': [g2(p) for p in d['strut_n']], 'strut_m': [g2(p) for p in d['strut_m']]},
        'sizes': {n: os.path.getsize(os.path.join(OUT, n + '.glb')) for n in ('body', 'lid', 'lower')}}
json.dump(info, open(os.path.join(OUT, 'design.json'), 'w'), indent=1)
cv = d['cam_v']; cam_o = np.array([cv[:, 0].max(), 0.5 * (cv[:, 1].min() + cv[:, 1].max()), P['camera']['cz']]); lamp = np.array(P['front_shapes'][0]['pts']); ly, lz = lamp[:, 0].mean(), lamp[:, 1].mean()
tl = P['rear_shapes'][0]; tail_x = float(V[(np.abs(V[:, 1]) < 0.01) & (np.abs(V[:, 2] - tl['cz']) < 0.006), 0].min())
slot_x = face_x(0.11, 0.145, 0.006); lidc = lv.mean(0); lidz = float(lv[:, 2].max())
HS = [
 dict(id='lid', part='lid', pos=g2([lidc[0] + 0.05, 0.0, lidz]), normal=[0, 1, 0], lid=True, title='翻盖',
      text='与车顶齐平的盖板，盖住 STOP 停止键和显示屏。后部是两个鹅颈铰链，两侧各有一根气撑杆，前缘是按压式锁扣；盖板开启约 65°。合盖时四周只留一圈细缝，下面有一圈密封圈。',
      view=dict(cam=[0.62, 0.62, 0.52], target=[-0.05, 0.3, 0.0])),
 dict(id='stop', part='body', pos=[0.0032, 0.2623, -0.0009], normal=[0, 1, 0], lid=True, title='STOP 停止键',
      text='盖板下方、控制面板最前端的橙色按键，用于让割草机立即停止工作。点击本标注会自动打开盖板。',
      view=dict(cam=[0.5, 0.62, 0.3], target=[-0.02, 0.27, 0.0])),
 dict(id='display', part='body', pos=[-0.0689, 0.2545, -0.0032], normal=[0, 1, 0], lid=True, title='显示屏与按键',
      text='盖板下方的深色面板：单色液晶屏和一排五个按键。点击本标注会自动打开盖板。',
      view=dict(cam=[0.42, 0.66, 0.34], target=[-0.06, 0.27, 0.0])),
 dict(id='camera', part='body', front=True, pos=g2(cam_o), normal=[1, 0, 0], lift=0.03, liftDir=[0.6, 0.8, 0], title='摄像头模组',
      text='黑色车头下沿的竖直视窗，正对前方。中间是带金属环的主摄像头；面对车头看，左侧是红外传感器，右侧是状态指示灯。模组与下方两个充电口在同一条中心线上。',
      view=dict(cam=[0.86, 0.3, 0.2], target=[0.29, 0.18, 0.0])),
 dict(id='ledL', part='body', front=True, pos=g2([face_x(ly, lz), ly, lz]), normal=[1.0, 0.0, -0.4], lift=0.03, title='LED 前灯（左）',
      text='车头左角的无边框 LED 灯条，外端上扬。', view=dict(cam=[0.9, 0.3, -0.38], target=[0.28, 0.17, -0.07])),
 dict(id='ledR', part='body', front=True, pos=g2([face_x(-ly, lz), -ly, lz]), normal=[1.0, 0.0, 0.4], lift=0.03, title='LED 前灯（右）',
      text='车头右角的无边框 LED 灯条，外端上扬。', view=dict(cam=[0.9, 0.3, 0.38], target=[0.28, 0.17, 0.07])),
 dict(id='slot', part='body', front=True, pos=g2([slot_x, -0.11, 0.145]), normal=[1, 0, 0], lift=0.03, liftDir=[0.7, -0.5, 0.5], title='充电口',
      text='前脸下部的黑色凹槽，中间是两个带银色边框的充电口。回充时，充电座立柱上的两根充电针插入其中。',
      view=dict(cam=[0.9, 0.24, 0.14], target=[0.3, 0.15, 0.0])),
 dict(id='tail', part='body', pos=g2([tail_x, 0.0, tl['cz']]), normal=[-1, 0, 0], lift=0.03, title='尾灯与腰线',
      text='车尾中央的白色尾灯条。下方的橙色腰线沿黑色下裙带环绕车尾，在两个后轮后方顺着斜边收到车身下沿。',
      view=dict(cam=[-0.95, 0.38, 0.42], target=[-0.26, 0.18, 0.0])),
]
old = json.load(open(os.path.join(OUT, 'hotspots_base.json')))           # wheel, caster, charging base: unchanged parts
json.dump(HS + old, open(os.path.join(OUT, 'hotspots.json'), 'w'), ensure_ascii=False, indent=1)
print('hotspots', [h['id'] for h in HS + old])
