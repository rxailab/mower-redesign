# Bake the product-site renders onto the page colour and convert them to WebP for viewer/site/img.
# usage: bake_site.py RENDERS OUT
#   RENDERS:          render_site.sh output (transparent film + shadow catcher, straight-alpha RGBA PNG)
#   OUT:              viewer/site/img
#
# How a render is baked:
#   - Near the machine (its own pixels and a few pixels around them) the exact over-composite is used:
#     rgb * alpha + page * (1 - alpha). Edge pixels keep their anti-aliasing and contact shadow, so the
#     machine gets no light outline.
#   - On the open floor the shadow catcher also leaves a faint, noisy veil that would show as a box on
#     the page. There the darkening is smoothed, faint speckle is dropped, it may only darken the page,
#     and it fades out towards every border of the image.
#   - Between the two zones the result blends over a few pixels, so there is no seam.
import os, sys, glob
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt, gaussian_filter

PAPER = (245, 245, 247); TILE = (22, 22, 23); HZ = (240, 240, 242)


def bake(src, bg):
    A = np.asarray(Image.open(src).convert('RGBA')).astype(np.float32)
    rgb, al = A[..., :3], A[..., 3:4] / 255.0
    B = np.array(bg, np.float32)
    exact = rgb * al + B * (1 - al)                             # straight-alpha over-composite
    obj = al[..., 0] >= 0.995                                   # the machine itself
    dist = distance_transform_edt(~obj)                         # px from the machine
    near = dist <= 3
    floor = np.minimum(exact, B)                                # the floor may only darken the page
    D = (B - floor).mean(-1)                                    # darkening of the page colour
    w = (~near).astype(np.float32)
    gw = gaussian_filter(w, 3)
    Ds = gaussian_filter(D * w, 3) / np.maximum(gw, 1e-6)       # smoothed over open floor only
    ok = gw > 0.2
    if not ok.all():                                            # under / next to the machine: nearest smoothed value
        idx = distance_transform_edt(~ok, return_distances=False, return_indices=True)
        Ds = Ds[idx[0], idx[1]]
    h, wd = D.shape
    yy = np.minimum(np.arange(h), np.arange(h)[::-1])[:, None] / (0.12 * h)
    xx = np.minimum(np.arange(wd), np.arange(wd)[::-1])[None, :] / (0.12 * wd)
    fe = np.clip(np.minimum(yy, xx), 0, 1); fe = fe * fe * (3 - 2 * fe)          # feather towards the borders
    Ds = np.clip(Ds - 5.0, 0, None) * 1.1 * fe                  # drop faint speckle, keep real shadows
    soft = B - Ds[..., None]
    t = np.clip((dist - 2.0) / 5.0, 0, 1)[..., None]            # 0 at the machine .. 1 from 7 px out
    t = t * t * (3 - 2 * t)
    out = exact * (1 - t) + soft * t
    return Image.fromarray(np.clip(out + 0.5, 0, 255).astype(np.uint8), 'RGB')


def save(src, dst, w=None, q=84, bg=PAPER):
    if not os.path.exists(src):
        print('MISSING', src); return
    im = bake(src, bg) if bg is not None else Image.open(src).convert('RGB')
    if w and im.width > w:
        im = im.resize((w, round(im.height * w / im.width)), Image.LANCZOS)
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    im.save(dst, 'WEBP', quality=q, method=6)


if __name__ == '__main__':
    R, O = sys.argv[1:3]
    stills = {'o_hero': ('hero', 2400, PAPER), 'o_stopkey': ('stop', 2400, (0, 0, 0)), 'o_pupil': ('pupil', 1600, TILE),
              'pt_dockport': ('dockport', 1600, None),
              'h_hero': ('h_hero', 2400, HZ), 'h_rear2': ('h_rear', 2400, HZ)}
    for a, (b, w, bg) in stills.items():
        save(f'{R}/{a}.png', f'{O}/{b}.webp', w, bg=bg)
    for p in ['orbit', 'lunar', 'graphite', 'glacier', 'sage', 'sand']:
        save(f'{R}/o_c_{p}.png', f'{O}/c_{p}.webp', 2000)
    for i in range(72):
        save(f'{R}/tt_t{i:02d}.png', f'{O}/tt/{i:02d}.webp', 1440, 78)
    for i in range(31):
        save(f'{R}/lid_lid{i:02d}.png', f'{O}/lid/{i:02d}.webp', 1440, 80)
    for i in range(24):
        save(f'{R}/exs_ex{i:02d}.png', f'{O}/ex/{i:02d}.webp', 1440, 80)
    for i in range(36):
        save(f'{R}/dk_dockin{i:02d}.png', f'{O}/dock/{i:02d}.webp', 1440, 80)
        save(f'{R}/sl_slope{i:02d}.png', f'{O}/slope/{i:02d}.webp', 1440, 80)
    files = glob.glob(O + '/**/*.webp', recursive=True)
    print('webp files', len(files), round(sum(os.path.getsize(f) for f in files) / 1e6, 1), 'MB')
