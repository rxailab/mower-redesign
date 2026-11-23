import numpy as np,sys; sys.path.insert(0,'.')
from scipy import ndimage as ndi
from PIL import Image
D=np.load('top_H.npy'); px=0.0003; umin,vmax=-0.27,0.12
g=np.hypot(*np.gradient(D,px))
def rc2xy(r,c): return umin+(c+0.5)*px, vmax-(r+0.5)*px
def grow(seed,free):
    lab,_=ndi.label(free); m=lab==lab[seed]; return m
# STOP: raised plateau
sr,sc=400,905
lvl=D[sr-5:sr+5,sc-5:sc+5].mean()
stop=grow((sr,sc),D>lvl-0.004); stop=ndi.binary_fill_holes(stop)
# display: low-gradient plateau region from seed, then fill holes + closing
dr,dc=400,690
disp=grow((dr,dc),g<0.22); disp=ndi.binary_closing(disp,iterations=3); disp=ndi.binary_fill_holes(disp)
disp=ndi.binary_opening(disp,iterations=2)
for nm,m in [("stop",stop),("disp",disp)]:
    r,c=np.nonzero(m); x0,y1=rc2xy(r.min(),c.min()); x1,y0=rc2xy(r.max(),c.max())
    print(nm,"px",m.sum(),"x",round(x0,4),round(x1,4),"y",round(y0,4),round(y1,4))
np.savez('seg_top_masks.npz',stop=stop,disp=disp,px=px,umin=umin,vmax=vmax)
# overlay
base=np.clip((D-np.percentile(D,5))/(np.percentile(D,99.9)-np.percentile(D,5)),0,1)
det=D-ndi.gaussian_filter(D,6); sh=np.clip(0.5+det/0.0012,0,1)
rgb=np.stack([sh,sh,sh],-1)
rgb[stop]=rgb[stop]*0.5+np.array([1,0.35,0.1])*0.5
rgb[disp]=rgb[disp]*0.5+np.array([0.1,0.5,1])*0.5
Image.fromarray((rgb*255).astype(np.uint8)).save('seg_top_overlay.png')
