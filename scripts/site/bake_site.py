# Bake the product-site renders onto the page colour and convert them to WebP for viewer/site/img
import os, glob
from PIL import Image
import sys
# usage: bake_site.py RENDERS SECTION_RENDERS OUT   (RENDERS: render_site.sh output; SECTION_RENDERS: the f4 cut-away renders c_secorbit, h4_lidgear30, s_stopsec, c_secnose; OUT: viewer/site/img)
R, F, O = sys.argv[1:4]
os.makedirs(O+'/tt', exist_ok=True); os.makedirs(O+'/lid', exist_ok=True)
PAPER=(245,245,247); TILE=(22,22,23)
def save(src, dst, w=None, q=84, bg=PAPER):
    if not os.path.exists(src): print('MISSING', src); return
    im=Image.open(src).convert('RGBA')
    if bg is not None:          # bake the page colour in: the shadow catcher leaves a faint light veil that shows as a box on the page
        import numpy as np
        A=np.asarray(im).astype(np.float32); al=A[...,3:4]/255.0; B=np.array(bg,np.float32)
        out=A[...,:3]*al+B*(1-al)
        floor=(al[...,0]<0.995)                      # shadow-catcher floor: it may only darken the page colour, never lighten it
        out[floor]=np.minimum(out[floor], B)
        h_,w_=al.shape[:2]; yy=np.minimum(np.arange(h_),np.arange(h_)[::-1])[:,None]/(0.12*h_); xx=np.minimum(np.arange(w_),np.arange(w_)[::-1])[None,:]/(0.12*w_)
        wgt=np.clip(np.minimum(yy,xx),0,1); wgt=wgt*wgt*(3-2*wgt)              # feather the floor into the page colour towards every border
        from PIL import ImageFilter
        dl=np.where(floor, (B[None,None,:]-out).mean(-1), 0.0).clip(0,255)          # how much the floor darkens the page (shadow)
        from scipy.ndimage import gaussian_filter; dl=gaussian_filter(dl.astype(np.float32), 3)   # smooth the render noise
        dl=np.clip(dl-5.0,0,None)*1.1*wgt                                              # drop faint speckle, keep real shadows, feather to the borders
        out[floor]=B-dl[floor][:,None]
        im=Image.fromarray(np.clip(out+0.5,0,255).astype(np.uint8),'RGB')
    if w and im.width>w: im=im.resize((w, round(im.height*w/im.width)), Image.LANCZOS)
    im.save(dst, 'WEBP', quality=q, method=6)
m={'o_hero':'hero','o_stopkey':'stop','o_pupil':'pupil','o_orbitc':'orbit','o_rband':'rband','o_face34':'face34','o_dockhero':'dock','h_hero':'h_hero','h_rear2':'h_rear','o_dockport':'dockport','o_dockwide':'dockwide'}
for a,b in m.items(): save(f'{R}/{a}.png', f'{O}/{b}.webp', 2400 if b in ('hero','stop','h_hero','h_rear','dockwide') else 1600, bg=TILE if b in ('pupil','orbit','rband','dockport') else (0,0,0) if b=='stop' else (240,240,242) if b.startswith('h_') else PAPER)
for p in ['orbit','lunar','graphite','glacier','sage','sand']: save(f'{R}/o_c_{p}.png', f'{O}/c_{p}.webp', 2000)
for i in range(72): save(f'{R}/tt_t{i:02d}.png', f'{O}/tt/{i:02d}.webp', 1440, 78)
for i in range(31): save(f'{R}/lid_lid{i:02d}.png', f'{O}/lid/{i:02d}.webp', 1440, 80)
for L in ['base','body','cover','lid']: save(f'{R}/ex_{L}_ex.png', f'{O}/ex_{L}.webp', 1600, 84, bg=PAPER if L=='base' else None)      # upper layers keep alpha
for a,b in {'c_secorbit':'sec_wall','h4_lidgear30':'sec_hinge','s_stopsec':'sec_stop','c_secnose':'sec_lamp'}.items(): save(f'{F}/{a}.png', f'{O}/{b}.webp', 1600, 82, bg=None)
tot=sum(os.path.getsize(f) for f in glob.glob(O+'/**/*.webp', recursive=True)); print('webp files', len(glob.glob(O+'/**/*.webp', recursive=True)), round(tot/1e6,1), 'MB')
