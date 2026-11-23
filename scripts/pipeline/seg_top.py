import numpy as np,sys; sys.path.insert(0,'.')
from segu import depth_map
from scipy import ndimage as ndi
from PIL import Image
d=np.load('asm_middle2.npz'); V=d['v']
px=0.0003
D,to_px=depth_map(V,(0,0,0),(1,0,0),(0,1,0),(0,0,1),px,-0.27,0.13,-0.12,0.12)
np.save('top_H.npy',D)
det=D-ndi.gaussian_filter(D,6)   # 1.8mm
g=np.hypot(*np.gradient(D,px))
def norm(a,lo,hi): return (np.clip((a-lo)/(hi-lo),0,1)*255).astype(np.uint8)
Image.fromarray(norm(D,np.percentile(D,5),np.percentile(D,99.9))).save('top_H.png')
Image.fromarray(norm(det,-0.0006,0.0006)).save('top_det.png')
Image.fromarray(norm(g,0,1.5)).save('top_grad.png')
print(D.shape)
