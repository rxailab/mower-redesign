# Put a MODE=sun render of the mower (transparent film, shadow catcher) into a garden photo.
# usage: python composite.py BACKPLATE.png RENDER.png OUT.png DX DY [WARM] [GRASS_PX]
#   the backplate is scaled to the render's width; the render is shifted by (DX, DY) px (render pixels);
#   WARM: colour multiply for the mower (golden hour, e.g. 1.05,1.0,0.9); GRASS_PX: height of the grass blades that cover
#   the wheel bottoms (their tips are taken from the photo itself)
import sys
import numpy as np
from PIL import Image, ImageFilter
from scipy import ndimage as nd

bgp, rp, out = sys.argv[1:4]; dx, dy = int(sys.argv[4]), int(sys.argv[5])
warm = np.array([float(v) for v in (sys.argv[6] if len(sys.argv) > 6 else '1.05,1.0,0.92').split(',')], np.float32)
gpx = float(sys.argv[7]) if len(sys.argv) > 7 else 12.0
R = Image.open(rp).convert('RGBA'); W, H = R.size
SCALE = float(sys.argv[9]) if len(sys.argv) > 9 else 1.0      # the mower smaller / larger in the photo (render scaled about its top-left)
if SCALE != 1.0: R = R.resize((int(W * SCALE), int(H * SCALE)), Image.LANCZOS)
bg = Image.open(bgp).convert('RGB'); bg = bg.resize((W, int(round(bg.height * W / bg.width))), Image.LANCZOS).filter(ImageFilter.UnsharpMask(1.2, 40, 2))
lay = Image.new('RGBA', bg.size, (0, 0, 0, 0)); lay.paste(R, (dx, dy), R)
L = np.asarray(lay).astype(np.float32) / 255; B = np.asarray(bg).astype(np.float32) / 255
rgb, a = L[..., :3].copy(), L[..., 3].copy()
# thin gaps between body parts render see-through on the transparent film: close 1-2 px holes inside the silhouette (dark, opaque)
s0 = a > 0.97; hole = nd.binary_closing(s0, structure=np.ones((3, 3)), iterations=2) & ~s0
rgb[hole] = np.minimum(rgb[hole], 0.035) if False else 0.03; a[hole] = 1.0
# shadow-catcher pixels carry only alpha (black, partial); the mower itself is opaque
solid = a > 0.985
WM = sys.argv[8] if len(sys.argv) > 8 else None      # optional wheel-mask render (MASK=wheels): grass only in front of the tyres
if WM:
    wl = Image.new('RGBA', bg.size, (0, 0, 0, 0)); wr = Image.open(WM).convert('RGBA'); wr = wr.resize(R.size, Image.LANCZOS) if wr.size != R.size else wr; wl.paste(wr, (dx, dy), wr)
    solid = solid & (np.asarray(wl)[..., 3] > 128)
rgb = np.clip(rgb * warm, 0, 1)
rgb = nd.gaussian_filter(rgb, (0.5, 0.5, 0))                                  # the photo is a touch softer than the render
# contact shadow: the silhouette's footprint, pushed down and blurred, darkens the grass right under the mower (occlusion)
sil = nd.binary_fill_holes(a > 0.97); foot = nd.shift(sil.astype(np.float32), (0.006 * sil.shape[1], 0), order=0)
cs = nd.gaussian_filter(foot, 0.006 * sil.shape[1]) * (1 - sil); B = B * (1 - 0.5 * np.clip(cs * 1.6, 0, 1))[..., None]
out_ = B * (1 - a[..., None]) + rgb * a[..., None]
# grass in front of the wheel / skirt bottoms: per column the lowest solid pixel; a band above it gets the photo back,
# with a ragged top edge from the photo's own bright blade tips
ys = np.arange(solid.shape[0])[:, None]; low = np.where(solid.any(0), (solid * ys).max(0), -1)
lum = B.mean(-1); tips = np.clip((lum - nd.gaussian_filter(lum, 3)) * 8 + 0.5, 0, 1)
rng = np.random.default_rng(3); n = nd.gaussian_filter(rng.random(lum.shape), (0.8, 2.5)); n = (n - n.min()) / (np.ptp(n) + 1e-9)
reach = gpx * (0.35 + 0.65 * n) * (0.6 + 0.4 * tips)                         # blade height per pixel
over = np.clip((ys - (low[None, :] - reach)) / 2.0, 0, 1) * (low[None, :] >= 0) * solid
over = nd.gaussian_filter(over, 0.6)
# the blades stand in the mower's shadow: as dark as the shadowed grass just below the tyres (median shadow alpha there)
band = (~solid) & (a > 0.05) & (ys > low[None, :]) & (ys < low[None, :] + 3 * gpx) & (low[None, :] >= 0)
sh_ = float(np.median(a[band])) if band.any() else 0.5
out_ = out_ * (1 - over[..., None]) + B * (1 - sh_) * over[..., None]
print('blade shadow alpha %.2f' % sh_)
Image.fromarray((np.clip(out_, 0, 1) * 255).astype(np.uint8)).save(out)
print('composite', out, Image.open(out).size)
