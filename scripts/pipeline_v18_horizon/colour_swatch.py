# [fix4-stop2] A6 colour-accurate swatch card (review finding A6: "do not sign off the safety red from the AgX product renders").
# Each colour of the STOP area is rendered as a flat plaque with the RENDERER's own material (design_render.mat: same base colour, finish,
# coat) next to a WHITE plaque of the same finish, in the 45/0 geometry of a colour measurement (one light at 45 deg, camera normal to the
# plaques, black surround: no specular in the camera).  Linear render (no view transform) -> reflectance = plaque / white of the same finish
# -> sRGB.  This is what the material IS; the product renders differ only by the studio (soft-box reflections in the gloss coat) and the AgX
# view transform.  Also writes a PNG card (Standard view transform, exposure set from the white plaques).
#   blender -b --factory-startup -P colour_swatch.py -- PARAMS.json OUT_PREFIX      -> OUT_PREFIX.json, OUT_PREFIX.png
import bpy, sys, os, json, math, numpy as np
argv = sys.argv[sys.argv.index('--') + 1:]; P = json.load(open(argv[0])); OUT = argv[1]; col = P.get('colors', {})
HERE = os.path.dirname(os.path.abspath(__file__))
FIN = {'gloss': (0.28, 0, 0.8), 'metallic': (0.34, 0.6, 1.0), 'satin': (0.5, 0, 0.1), 'matte': (0.72, 0, 0),
       'satincoat': (0.42, 0.0, 0.45), 'smoke': (0.16, 0.0, 0.8), 'texture': (0.62, 0.0, 0.0)}      # = design_render.FIN
def lin(h):
    h = h.lstrip('#'); c = [int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)]; return [x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c]
def srgb(x):
    x = np.clip(x, 0, 1); return np.where(x <= 0.0031308, 12.92 * x, 1.055 * x ** (1 / 2.4) - 0.055)
def mat(name, hexc, finish):
    m = bpy.data.materials.new(name); m.use_nodes = True; b = m.node_tree.nodes['Principled BSDF']; r, me, cc = FIN[finish]
    b.inputs['Base Color'].default_value = (*lin(hexc), 1); b.inputs['Roughness'].default_value = r; b.inputs['Metallic'].default_value = me
    b.inputs['Coat Weight'].default_value = cc; b.inputs['Coat Roughness'].default_value = 0.06; return m
SW = [('STOP red chip', col.get('stop', '#C8102E'), 'gloss'), ('STOP red', col.get('stop', '#C8102E'), col.get('stop_finish', 'satin')), ('legend', '#F4F4F2', 'satin'),      # chip = smooth gloss colour reference (what a colour chip / RAL card is measured on)
      ('lid', col.get('lid', '#CDD0CF'), col.get('lid_finish', 'satincoat')), ('halo ring', col.get('halo', '#F2FBFF'), 'satin'),
      ('socket / kit', col.get('kit', '#34373C'), col.get('kit_finish', 'satin'))]
for o in list(bpy.data.objects): bpy.data.objects.remove(o)
sc = bpy.context.scene; S = 0.04; G = 0.012; obs = []
for i, (nm, hx, fin) in enumerate(SW):
    for j, (h_, tag) in enumerate(((hx, 'c'), ('#FFFFFF', 'w'))):
        bpy.ops.mesh.primitive_plane_add(size=S, location=(i * (S + G), -j * (S + G), 0)); o = bpy.context.active_object; o.data.materials.append(mat('%s_%s' % (nm, tag), h_, fin)); obs.append(o)
cx = (len(SW) - 1) * (S + G) / 2; cy = -(S + G) / 2
L = bpy.data.lights.new('L45', 'AREA'); L.energy = 40.0; L.size = 0.6; lo = bpy.data.objects.new('L45', L); sc.collection.objects.link(lo)
lo.location = (cx, cy - 1.2, 1.2); lo.rotation_euler = (math.radians(45), 0, 0)          # 45 deg incidence, from -y
cam = bpy.data.objects.new('Cam', bpy.data.cameras.new('Cam')); sc.collection.objects.link(cam); sc.camera = cam
cam.data.type = 'ORTHO'; cam.data.ortho_scale = len(SW) * (S + G) + 0.01; cam.location = (cx, cy, 1.0)
w = bpy.data.worlds.new('W'); sc.world = w; w.use_nodes = True; w.node_tree.nodes['Background'].inputs[0].default_value = (0, 0, 0, 1)
sc.render.engine = 'CYCLES'; sc.cycles.samples = 64; sc.cycles.use_denoising = True
try:
    pr = bpy.context.preferences.addons['cycles'].preferences; pr.compute_device_type = 'OPTIX'; pr.get_devices(); sc.cycles.device = 'GPU'
    for dv in pr.devices: dv.use = (dv.type == 'OPTIX')
except Exception: sc.cycles.device = 'CPU'
RX = 1000; sc.render.resolution_x = RX; sc.render.resolution_y = int(RX * 2 * (S + G) / (len(SW) * (S + G) + 0.01)) + 20
sc.view_settings.view_transform = 'Standard'; sc.view_settings.look = 'None'; sc.view_settings.exposure = 0.0
sc.render.image_settings.file_format = 'OPEN_EXR'; exr = OUT + '_lin.exr'; sc.render.filepath = exr; bpy.ops.render.render(write_still=True)
im = bpy.data.images.load(exr); W_, H_ = im.size; px = np.array(im.pixels[:]).reshape(H_, W_, 4)[::-1, :, :3]      # top row first
def patch(o):      # 40 % central patch of a plaque, in image pixels
    sx = W_ / cam.data.ortho_scale; ox, oy = o.location.x - cam.location.x, o.location.y - cam.location.y; r = 0.2 * S * sx
    u = W_ / 2 + ox * sx; v = H_ / 2 - oy * sx; return np.median(px[int(v - r):int(v + r), int(u - r):int(u + r)].reshape(-1, 3), 0)
rep = dict(method='45/0 geometry, linear Cycles render, reflectance = plaque / white plaque of the same finish (design_render.mat materials)', swatches={})
wmax = 0.0
for i, (nm, hx, fin) in enumerate(SW):
    c_, w_ = patch(obs[2 * i]), patch(obs[2 * i + 1]); wmax = max(wmax, float(w_.max())); refl = c_ / np.maximum(w_, 1e-9); s_ = np.round(srgb(refl) * 255).astype(int)
    spec = np.array([int(hx.lstrip('#')[k:k + 2], 16) for k in (0, 2, 4)])
    import colorsys; hsv_ = colorsys.rgb_to_hsv(*(s_ / 255.0))
    rep['swatches'][nm] = dict(spec=hx, finish=fin, measured_srgb=s_.tolist(), hsv=[round(hsv_[0] * 360), round(hsv_[1], 2), round(hsv_[2], 2)], delta_max=int(np.abs(s_ - spec).max()), ok_10=bool(np.abs(s_ - spec).max() <= 10))
sc.render.image_settings.file_format = 'PNG'; sc.view_settings.exposure = -math.log2(wmax); sc.render.filepath = OUT + '.png'; bpy.ops.render.render(write_still=True)
rep['png'] = OUT + '.png'; rep['exposure_ev'] = round(sc.view_settings.exposure, 3); rep['ok'] = bool(rep['swatches']['STOP red chip']['ok_10'])      # the A6 item (the moulded finish adds its own surface haze at 45/0); the others are for reference
json.dump(rep, open(OUT + '.json', 'w'), indent=1); print('SWATCH', json.dumps(rep))
