# Bake the AIRAJ site images (airaj-site/assets/img) from the renders and composites.
# usage: python bake.py RENDERS_DIR HERO_COMPOSITE.png SLOPE_COMPOSITE.png OUT_DIR
#   RENDERS_DIR: render_r1.py outputs c_<id>_hero.png (all palettes), k_panel2 / k_sensor2 / k_stopr / k_side.png, d_dockhero.png
import sys, os
import numpy as np
from PIL import Image

R, HERO, SLOPE, O = sys.argv[1:5]; os.makedirs(O, exist_ok=True)
IDS = ['black', 'silver', 'stripes', 'yellow', 'blue', 'red', 'brg', 'sage', 'sand', 'taro']


def save(im, name, q=82):
    im.save(os.path.join(O, name), 'WEBP', quality=q, method=6); print(name, im.size, round(os.path.getsize(os.path.join(O, name)) / 1024), 'kB')


def cutout(src, w, h, pad=0.06):
    """Transparent render -> tight 3:2 (w:h) crop round the mower and its shadow, alpha kept."""
    a = Image.open(src).convert('RGBA'); al = np.asarray(a)[..., 3]
    ys, xs = np.nonzero(al > 10); x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
    cw, ch = (x1 - x0) * (1 + 2 * pad), (y1 - y0) * (1 + 2 * pad); r = w / h
    if cw / ch < r: cw = ch * r
    else: ch = cw / r
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    box = (int(cx - cw / 2), int(cy - ch / 2), int(cx + cw / 2), int(cy + ch / 2))
    c = a.crop(box).resize((w, h), Image.LANCZOS)
    # fade the soft shadow out towards the edges (a cropped shadow reads as a box on the page)
    yy, xx = np.mgrid[0:h, 0:w]; e = np.minimum.reduce([xx / (0.1 * w), (w - 1 - xx) / (0.1 * w), yy / (0.1 * h), (h - 1 - yy) / (0.1 * h)])
    f = np.clip(e, 0, 1); f = f * f * (3 - 2 * f); arr = np.asarray(c).copy(); arr[..., 3] = (arr[..., 3] * f).astype(np.uint8)
    return Image.fromarray(arr, 'RGBA')


def studio(src, w, h):
    """Transparent render on a soft studio backdrop (light centre, slightly darker edges)."""
    a = Image.open(src).convert('RGBA').resize((w, h), Image.LANCZOS)
    yy, xx = np.mgrid[0:h, 0:w]; d = np.hypot((xx - w * 0.5) / w, (yy - h * 0.42) / h)
    c0, c1 = np.array([238, 238, 234], float), np.array([214, 214, 208], float)
    bg = c0 + (c1 - c0) * np.clip(d / 0.75, 0, 1)[..., None]
    b = Image.fromarray(bg.astype(np.uint8)).convert('RGBA'); b.alpha_composite(a); return b.convert('RGB')


# hero (16:9 desktop, portrait for phones)
H = Image.open(HERO).convert('RGB'); W_, H_ = H.size
hero = H.crop((0, 120, W_, 120 + int(W_ * 9 / 16))); save(hero, 'hero.webp', 80); save(hero.resize((1280, 720), Image.LANCZOS), 'hero-1280.webp', 80)
save(H.crop((1180, 0, 2180, H_)).resize((900, int(900 * H_ / 1000)), Image.LANCZOS), 'hero-portrait.webp', 80)      # phones: round the mower (right of centre)
# colour cards + quick links
for i in IDS: save(cutout(os.path.join(R, f'c_{i}_hero.png'), 1200, 800), f'r1-{i}.webp', 84)
save(cutout(os.path.join(R, 'c_black_hero.png'), 320, 213, 0.03), 'q-r1.webp', 84)
save(cutout(os.path.join(R, 'd_dockhero.png'), 320, 213, 0.03), 'q-dock.webp', 84)
# technology panels
save(studio(os.path.join(R, 'k_panel2.png'), 1600, 1067), 't-rtk.webp')
save(Image.open(SLOPE).convert('RGB').resize((1600, 1067), Image.LANCZOS), 't-slope.webp')
save(studio(os.path.join(R, 'k_sensor2.png'), 1600, 1067), 't-sensor.webp')
save(studio(os.path.join(R, 'd_dockhero.png'), 1600, 1067), 't-dock.webp')
save(studio(os.path.join(R, 'k_stopr.png'), 1600, 1067), 't-stop.webp')
# specs figure
save(cutout(os.path.join(R, 'k_side.png'), 1600, 1067, 0.04), 's-side.webp', 84)
