# [fix2] labelled contact sheets of the final set.  usage: python sheet2.py RENDERDIR OUTDIR V17DIR
import sys, os
from PIL import Image, ImageDraw, ImageFont
R, O, V17 = sys.argv[1], sys.argv[2], sys.argv[3]
def font(s):
    for p in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/dejavu/DejaVuSans.ttf'):
        if os.path.exists(p): return ImageFont.truetype(p, s)
    return ImageFont.load_default()
F1, F2 = font(40), font(26)
def sheet(name, title, tiles, cols, W=3000, note=''):
    tw = W // cols; ims = []
    for path, lab in tiles:
        if not os.path.exists(path): print('missing', path); continue
        im = Image.open(path).convert('RGB'); im = im.resize((tw, int(im.height * tw / im.width)), Image.LANCZOS); ims.append((im, lab))
    th = max(i.height for i, _ in ims); rows = (len(ims) + cols - 1) // cols; top = 70 + (36 if note else 0)
    S = Image.new('RGB', (W, top + rows * (th + 40)), (245, 245, 244)); d = ImageDraw.Draw(S)
    d.text((20, 14), title, fill=(15, 15, 15), font=F1)
    if note: d.text((20, 64), note, fill=(70, 70, 70), font=F2)
    for k, (im, lab) in enumerate(ims):
        x = (k % cols) * tw; y = top + (k // cols) * (th + 40); d.text((x + 10, y + 6), lab, fill=(30, 30, 30), font=F2); S.paste(im, (x, y + 38))
    S.save(os.path.join(O, name)); print(name, S.size)
p = lambda n: os.path.join(R, n + '.png')
T = 'v18 "Orbit" fix 2 (f2)'
sheet('overview.png', T + ' - product views', [(p('p_hero'), 'hero'), (p('p_face34'), 'face 3/4'), (p('p_front'), 'front'), (p('p_side'), 'side'),
      (p('p_rear'), 'rear 3/4'), (p('p_rear2'), 'rear2 (85 mm)'), (p('p_top'), 'top'), (p('p_dockhero'), 'on the dock')], 4,
      note='light snap-on cover over the dark one-piece body; one part line (level brows, R128 wheel orbit, R90 tail sweep, R20 fillets); G2 superellipse shoulder; one roof graphic (STOP pad + status halo)')
sheet('overview_more.png', T + ' - more product views', [(p('p_rearstr'), 'rear, straight'), (p('p_sideup'), 'side, from above'), (p('p_plan'), 'plan')], 3)
sheet('lid.png', T + ' - lid over STOP + display', [(p('l_closedtop'), 'closed: flush lid, STOP pad + halo'), (p('l_stoppad'), 'STOP pad close-up (2K, stop square, status halo)'),
      (p('l_lid30'), 'opening, 30 deg'), (p('l_lidlatch30'), 'push-push latch side, 30 deg'), (p('l_lidopen'), 'at the 75 deg stop'), (p('l_lid75F'), '75 deg, from the front (underside ribs)'),
      (p('l_lidhingeX'), 'hinge side, cover hidden (arc tongues, pods)'), (p('l_lidhingesec'), 'section y -54: hinge (arc tongue in its guide)'), (p('l_latchsec'), 'section y 0: STOP plunger over the key, latch')], 3,
      note='virtual-pivot arc-track hinge, stop 75 deg, push-push latch; pressing the STOP pad drives the lid into the latch over-travel: the plunger presses the real STOP key, then the lid opens')
sheet('construction.png', T + ' - construction (2.5 mm walls, closed solids)', [(p('c_explode'), 'exploded, from above'), (p('c_explodelow'), 'exploded, from below'), (p('c_explodec'), 'deck + screws (exploded)'),
      (p('c_deck'), 'cover hidden: 2.5 mm deck, 14 screws'), (p('c_deckN'), 'deck, nose'), (p('c_deckT'), 'deck, tail'), (p('c_coverin'), 'cover inside: legs, pins, pads'),
      (p('c_secnose'), 'section y -60: nose joint, deck, datum pin'), (p('c_sectail'), 'section y -60: tail joint'), (p('c_secorbit'), 'section x 120: orbit joint'),
      (p('c_secY'), 'section y -60, full length'), (p('c_secX'), 'section x 120, full width')], 3,
      note='section caps: body (core) orange, cover blue; every wall 2.5 mm, edges rolled R0.8 / R0.5 / R1')
sheet('details.png', T + ' - detail close-ups', [(p('d_fband'), 'front lens band'), (p('d_pupil'), 'pupil: camera / IR / LED'), (p('d_rband'), 'tail band'),
      (p('d_orbitc'), 'orbit + R20 fillet'), (p('d_orbfil'), 'brow-orbit fillet'), (p('d_risec'), 'tail sweep (R90) + R20 into the base line'),
      (p('d_shut'), 'lid shut line + gutter'), (p('d_portc'), 'service slot: two charger ports behind a web (headlight)'), (p('d_port'), 'service slot, 3/4 (headlight)'),
      (p('d_tailcorner'), 'tail corner'), (p('d_floor'), 'nose corner floor, integral flange (headlight)'), (p('l_stoppad'), 'STOP pad')], 4)
rows = []
for v in ('hero', 'front', 'rear', 'side', 'top'):
    rows += [(os.path.join(V17, 'a_cov_%s.png' % v), 'v17  ' + v), (p('p_' + v), 'v18 f2  ' + v)]
sheet('compare_v17_v18.png', 'v17 (left) vs v18 "Orbit" f2 (right) - same cameras', rows, 2)
