import numpy as np,json
from scipy import ndimage as ndi
from scipy.interpolate import RegularGridInterpolator
G=json.load(open('mid_tex_geom.json'))
def rrect_boundary(cx,cy,a,b,r,N):
    # sample rounded-rect perimeter uniformly by arc length (counter-clockwise)
    segs=[]
    corners=[(cx+a-r,cy+b-r,0),(cx-a+r,cy+b-r,90),(cx-a+r,cy-b+r,180),(cx+a-r,cy-b+r,270)]
    pts=[]
    for (ox,oy,ang0) in corners:
        for t in np.linspace(0,90,40,endpoint=False):
            th=np.radians(ang0+t); pts.append((ox+r*np.cos(th),oy+r*np.sin(th)))
        # straight edge to next corner handled by resampling below
    pts=np.array(pts+[pts[0]])
    seg=np.linalg.norm(np.diff(pts,axis=0),axis=1); s=np.r_[0,np.cumsum(seg)]
    ts=np.linspace(0,s[-1],N,endpoint=False)
    return np.c_[np.interp(ts,s,pts[:,0]),np.interp(ts,s,pts[:,1])]
def ring_grid(B,cx,cy,K=40,gamma=1.6):
    N=len(B); P=[]
    for k in range(K+1):
        sc=1-(k/K)**gamma if k<K else 0.0
        sc=(1-k/K)**1.0
        P.append(np.c_[cx+(B[:,0]-cx)*sc,cy+(B[:,1]-cy)*sc])
    P=np.array(P)   # (K+1,N,2)
    V=P.reshape(-1,2); F=[]
    for k in range(K):
        for i in range(N):
            a=k*N+i; b=k*N+(i+1)%N; c=(k+1)*N+(i+1)%N; d=(k+1)*N+i
            F.append((a,b,c)); F.append((a,c,d))
    return V,np.array(F)
# ---- display decal
D=np.load('top_H.npy'); px=0.0003; umin,vmax=-0.27,0.12
Ds=ndi.gaussian_filter(ndi.maximum_filter(D,size=7),1.5)
rows=vmax-(np.arange(D.shape[0])+0.5)*px; cols=umin+(np.arange(D.shape[1])+0.5)*px
interp=RegularGridInterpolator((rows[::-1],cols),Ds[::-1],bounds_error=False,fill_value=None)
d=G['display']; r=G['display_r']; cx=(d['x0']+d['x1'])/2; cy=(d['y0']+d['y1'])/2; a=(d['x1']-d['x0'])/2; b=(d['y1']-d['y0'])/2
B=rrect_boundary(cx,cy,a,b,r,480); V2,F=ring_grid(B,cx,cy,K=48)
z=interp(np.c_[V2[:,1],V2[:,0]])+0.00035
V=np.c_[V2,z]
uv=np.c_[(d['y1']-V[:,1])/(d['y1']-d['y0']),(V[:,0]-d['x0'])/(d['x1']-d['x0'])]
np.savez('decal_display.npz',v=V,f=F,uv=uv); print("display decal",V.shape,F.shape)
# ---- window decal (plane fit to window-labelled vertices)
M=np.load('asm_middle2_lab.npz'); MV=M['v']; MF=M['f']; L=M['lab']
pf=G['pod_frame']; o=np.array(pf['o']); R=np.array(pf['right']); U=np.array(pf['up']); Dv=np.array(pf['dv'])
wv=MV[np.unique(MF[L==4])]-o; u=wv@R; v=wv@U; w=wv@Dv
A=np.c_[np.ones_like(u),u,v]; coef=np.linalg.lstsq(A,w,rcond=None)[0]; print("window plane resid mm",np.std(A@coef-w)*1000)
W=G['window']; wcx=(W['u0']+W['u1'])/2; wcy=(W['v0']+W['v1'])/2; wa=(W['u1']-W['u0'])/2; wb=(W['v1']-W['v0'])/2
B=rrect_boundary(wcx,wcy,wa,wb,0.0012,360); P2,F=ring_grid(B,wcx,wcy,K=24)
ww=coef[0]+coef[1]*P2[:,0]+coef[2]*P2[:,1]+0.0005
V=o+P2[:,0:1]*R+P2[:,1:2]*U+ww[:,None]*Dv
uv=np.c_[(P2[:,0]-W['u0'])/(W['u1']-W['u0']),(P2[:,1]-W['v0'])/(W['v1']-W['v0'])]
np.savez('decal_window.npz',v=V,f=F,uv=uv); print("window decal",V.shape)
# ---- wheel ring decals: every vertex sits 0.35 mm above the highest local hub surface
from scipy.spatial import cKDTree
WG=json.load(open('wheel_geom.json')); Wv=np.load('asm_wheels.npz')['v']
for nm,s in [('L',1),('R',-1)]:
    c=np.array(WG[nm]['center']); ax=np.array(WG[nm]['axle'])
    P=Wv[np.sign(Wv[:,1])==s]-c; t=P@ax
    e1=np.cross(ax,[0,0,1.0]); e1/=np.linalg.norm(e1); e2=np.cross(ax,e1)
    outer=t>0.012; Q=np.c_[P[outer]@e1,P[outer]@e2]; to=t[outer]; T=cKDTree(Q)
    rb=np.linspace(0.0815,0.0910,9); NT=320; th=np.linspace(0,2*np.pi,NT,endpoint=False)   # hub lip next to the tyre
    V=[];uvl=[]
    TL=[]
    for j,r0 in enumerate(rb):                       # highest local surface under each ring sample
        pts=np.c_[r0*np.cos(th),r0*np.sin(th)]
        nb=T.query_ball_point(pts,0.0016)
        tl=np.array([to[i].max() if len(i) else np.nan for i in nb])
        TL.append(np.where(np.isnan(tl),np.nanmedian(tl),tl))
    tflat=np.max(np.array(TL),axis=0)                # flat across the ring width, follows the wheel around
    tflat=np.convolve(np.r_[tflat[-3:],tflat,tflat[:3]],np.ones(7)/7,'valid')
    for j,r0 in enumerate(rb):
        for k in range(NT):
            V.append(c+ax*(tflat[k]+0.0004)+r0*(np.cos(th[k])*e1+np.sin(th[k])*e2)); uvl.append((k/NT,j/(len(rb)-1)))
    V=np.array(V); F=[]
    for j in range(len(rb)-1):
        for k in range(NT):
            a_=j*NT+k; b_=j*NT+(k+1)%NT; c_=(j+1)*NT+(k+1)%NT; d_=(j+1)*NT+k
            F.append((a_,c_,b_)); F.append((a_,d_,c_))   # wound so the normal faces outward (+axle)
    np.savez(f'decal_ring_{nm}.npz',v=V,f=np.array(F),uv=np.array(uvl)); print("ring",nm,V.shape,"t range mm",round(float(np.min(V@ax-c@ax))*1000,2),round(float(np.max(V@ax-c@ax))*1000,2))
