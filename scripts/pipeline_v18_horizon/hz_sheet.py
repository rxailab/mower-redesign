# [hz] labelled contact sheets of the Horizon stage-1 set (hz_renders.sh output; prefix h = Horizon, o = Orbit f3).
# usage: python hz_sheet.py RENDERDIR   -> RENDERDIR/sheet_key.png, sheet_horizon.png, sheet_details.png, sheet_compare.png
import sys, os
from PIL import Image, ImageDraw, ImageFont
R = sys.argv[1]
def font(s):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/dejavu/DejaVuSans.ttf'):
        if os.path.exists(p): return ImageFont.truetype(p, s)
    return ImageFont.load_default()
F1, F2 = font(40), font(25)
def sheet(name, title, tiles, cols, W=3000, note=''):
    tw = W // cols; ims = []
    for path, lab in tiles:
        if not os.path.exists(path): print('missing', path); continue
        im = Image.open(path).convert('RGB'); im = im.resize((tw, int(im.height * tw / im.width)), Image.LANCZOS); ims.append((im, lab))
    th = max(i.height for i, _ in ims); rows = (len(ims) + cols - 1) // cols; top = 70 + 34 * len(note.split('\n')) if note else 70
    S = Image.new('RGB', (W, top + rows * (th + 40)), (245, 245, 244)); d = ImageDraw.Draw(S)
    d.text((20, 14), title, fill=(15, 15, 15), font=F1)
    for k, ln in enumerate(note.split('\n') if note else []): d.text((20, 64 + 34 * k), ln, fill=(70, 70, 70), font=F2)
    for k, (im, lab) in enumerate(ims):
        x = (k % cols) * tw; y = top + (k // cols) * (th + 40); d.text((x + 10, y + 6), lab, fill=(30, 30, 30), font=F2); S.paste(im, (x, y + 38))
    S.save(os.path.join(R, name)); print(os.path.join(R, name), S.size)
p = lambda n: os.path.join(R, n + '.png')
T = 'v18 "Horizon" stage 1 - exterior'
NOTE = ('silver satin-metallic canopy (snap-on cover) over the dark one-piece body; ONE level part line K1 z 180 all round (0.8 gap, R0.8 / R0.5, cover 0.5 proud);\n'
        'gloss smoked spine 188 wide (2K, edges = a 2.4 x 0.9 smooth groove) from the nose K1 over the roof to the tail K1, the lid a 164-wide segment inside it, red STOP key on its axis;\n'
        '3 mm light line 7 mm under K1 ending on R128 about each hub, light-pupil-light at the nose, dot at the tail; K2 z 123 = 0.5 mm base step; trapezoid flanks 12.7 deg')
sheet('sheet_key.png', T + ' - key views', [(p('h_hero'), 'hero'), (p('h_face34'), 'face 3/4'), (p('h_side'), 'side'), (p('h_rear2'), 'rear 3/4 (85 mm)'),
      (p('h_top'), 'top'), (p('h_stopkey'), 'closed lid: STOP key on the spine'), (p('h_nose'), 'nose: spine on K1, light, pupil'), (p('h_haunch'), 'haunch: light ends on R128, K2 step'),
      (p('h_joint'), 'K1 joint close-up (rear flank)'), (p('o_hero'), 'Orbit f3 hero (comparison)'), (p('o_rear2'), 'Orbit f3 rear 3/4'), (p('h_secK1'), 'section x -200: K1 labyrinth + deck land')], 4, note=NOTE)
sheet('sheet_horizon.png', T + ' - product views', [(p('h_hero'), 'hero'), (p('h_face34'), 'face 3/4'), (p('h_front'), 'front'), (p('h_side'), 'side'),
      (p('h_rear'), 'rear 3/4'), (p('h_rear2'), 'rear 3/4 (85 mm)'), (p('h_top'), 'top'), (p('h_lidopen'), 'lid open (75 deg)')], 4, note=NOTE)
sheet('sheet_details.png', T + ' - close-ups and sections', [(p('h_stopkey'), 'closed lid, raised red STOP key'), (p('h_nose'), 'nose corner'), (p('h_spine'), 'spine groove lands on K1'),
      (p('h_pupil'), 'camera pupil in the light line'), (p('h_tail'), 'tail: dot, light, spine on K1'), (p('h_haunch'), 'haunch: light end on R128, base step'), (p('h_joint'), 'K1 joint (0.8 gap, R0.8 / R0.5, 0.5 step)'),
      (p('h_secK1'), 'section x -200: labyrinth, drained land'), (p('h_secwheel'), 'section x 120: body rail, cover, deck over the rim'), (p('h_secnose'), 'section y -30: nose, light, slot'), (p('h_secbase'), 'section x -150: K2 base step')], 4)
pairs = []
for v, lab in (('hero', 'hero'), ('face34', 'face 3/4'), ('side', 'side'), ('rear2', 'rear 3/4'), ('top', 'top'), ('stopkey', 'STOP key'), ('nose', 'nose'), ('tail', 'tail'), ('haunch', 'haunch'), ('joint', 'joint')):
    pairs += [(p('h_' + v), 'Horizon - ' + lab), (p('o_' + v), 'Orbit f3 - ' + lab)]
sheet('sheet_compare.png', 'v18 Horizon (left) vs Orbit f3 (right) - same cameras', pairs, 4)
