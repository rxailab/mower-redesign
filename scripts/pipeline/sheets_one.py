# Overview sheets for the one-piece body renders.  usage: python sheets_one.py OUTDIR NAME
import sys, os
from PIL import Image
D, N = sys.argv[1], sys.argv[2]
def sheet(name, views, cols, w=1000):
    ims = [Image.open(os.path.join(D, v if v.endswith('.png') else f'{N}_{v}.png')).convert('RGB') for v in views]; h = int(w * ims[0].size[1] / ims[0].size[0])
    rows = (len(ims) + cols - 1) // cols; o = Image.new('RGB', (cols * w + (cols + 1) * 8, rows * h + (rows + 1) * 8), (20, 21, 24))
    for i, im in enumerate(ims): o.paste(im.resize((w, h), Image.LANCZOS), (8 + (i % cols) * (w + 8), 8 + (i // cols) * (h + 8)))
    o.save(os.path.join(D, name)); print(name, o.size)
sheet('overall_overview.png', ['hero', 'face34', 'front', 'rear2', 'side', 'top'], 3)
sheet('tail_overview.png', ['rear2', 'rearstr', 'rearlow', 'tailclose', 'slotL', 'valL'], 3)
sheet('fastening_overview.png', ['fastening_map.png', 'explode', 'explode2', 'under', 'tailin', 'wellA'], 3)
sheet('lid_overview.png', ['lidopen', 'lidhinge', 'lidunder', 'lidside'], 2)
