import numpy as np
from scipy import ndimage as ndi
def depth_map(V, origin, right, up, dirv, px, umin, umax, vmin, vmax, near_cut=None):
    """ortho depth image looking along -dirv (dirv points to camera). returns D (rows=v descending), and helper to map points"""
    R=np.asarray(right,float); U=np.asarray(up,float); Dv=np.asarray(dirv,float); o=np.asarray(origin,float)
    P=V-o; u=P@R; v=P@U; d=P@Dv
    W=int(np.ceil((umax-umin)/px)); H=int(np.ceil((vmax-vmin)/px))
    m=(u>=umin)&(u<umax)&(v>=vmin)&(v<vmax)
    if near_cut is not None: m&=d<near_cut
    j=((u[m]-umin)/px).astype(int); i=((vmax-v[m])/px).astype(int)
    D=np.full((H,W),-np.inf); np.maximum.at(D,(i,j),d[m])
    # fill splat gaps
    filled=ndi.grey_dilation(np.where(np.isfinite(D),D,-1e9),size=3)
    D2=np.where(np.isfinite(D),D,filled); D2[D2<-1e8]=np.nan
    D2=ndi.median_filter(np.nan_to_num(D2,nan=np.nanmin(D2)),size=3)
    def to_px(X):
        Q=np.asarray(X)-o; return ((vmax-Q@U)/px).astype(int),((Q@R-umin)/px).astype(int),Q@Dv
    return D2,to_px
