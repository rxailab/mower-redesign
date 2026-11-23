import numpy as np,sys,json; sys.path.insert(0,'.')
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
from segmesh import face_normals
d=np.load('asm_middle2.npz'); V=d['v']; F=d['f'].astype(np.int64)
FN,FA=face_normals(V,F)
# orientation: make normals outward (top faces up)
C=V[F].mean(1)
top=C[:,2]>np.percentile(C[:,2],95)
if FN[top,2].mean()<0: FN=-FN; F=F[:,[0,2,1]]; print("flipped winding to outward")
L=np.zeros(len(F),np.int8)   # 0 body
NAMES=['MidBody','StopButton','DisplayGlass','PodLight','SensorGlass','LEDLens','SlotInner']
M=np.load('seg_top_masks.npz'); px=float(M['px']); umin=float(M['umin']); vmax=float(M['vmax'])
Ht=np.load('top_H.npy')
def topmask_lookup(mask,X):
    c=((X[:,0]-umin)/px).astype(int); r=((vmax-X[:,1])/px).astype(int)
    ok=(r>=0)&(r<mask.shape[0])&(c>=0)&(c<mask.shape[1]); out=np.zeros(len(X),bool); out[ok]=mask[r[ok],c[ok]]
    h=np.full(len(X),-9.0); h[ok]=Ht[r[ok],c[ok]]; return out,h
# 1 STOP: within dilated mask and above panel level (top surface + side walls)
stop=M['stop']; stop_d=ndi.binary_dilation(stop,iterations=4)
inS,hS=topmask_lookup(stop_d,C)
panel=np.median(Ht[ndi.binary_dilation(stop,iterations=30)&~ndi.binary_dilation(stop,iterations=15)])
sel=inS&(C[:,2]>panel+0.0015); L[sel]=1; print("stop faces",sel.sum(),"panel z",round(panel,4))
# 2 display: rounded rect, faces near the top surface, upward facing
G=json.load(open('disp_geom.json')); dg=G['display']; R=G['display_r']
def rrsdf(x,y,x0,x1,y0,y1,r):
    cx,cy,a,b=(x0+x1)/2,(y0+y1)/2,(x1-x0)/2,(y1-y0)/2
    qx=np.abs(x-cx)-(a-r); qy=np.abs(y-cy)-(b-r)
    return np.hypot(np.maximum(qx,0),np.maximum(qy,0))+np.minimum(np.maximum(qx,qy),0)-r
_,hD=topmask_lookup(np.ones_like(stop),C)
sel=(rrsdf(C[:,0],C[:,1],dg['x0'],dg['x1'],dg['y0'],dg['y1'],R)<0)&(C[:,2]>hD-0.0015)&(FN[:,2]>0.5)&(L==0)
L[sel]=2; print("display faces",sel.sum())
# 3 pod (light grey): inside shell small-opening footprint (dilated) and above plate level
S=np.load('asm_shell.npz'); SV=S['v']
h=0.002; lo=np.array([-0.4,-0.3]); cell=lambda X:((X[:,:2]-lo)/h).astype(int)
cov=np.zeros((400,300),bool); ij=cell(SV); cov[ij[:,0],ij[:,1]]=True
cov=ndi.binary_closing(cov,iterations=2); holes=ndi.binary_fill_holes(cov)&~cov
lab,nl=ndi.label(holes)
ij=cell(C); ok=(ij[:,0]>=0)&(ij[:,0]<400)&(ij[:,1]>=0)&(ij[:,1]<300)
best=None
for i in range(1,nl+1):
    ctr=np.argwhere(lab==i).mean(0)*h+lo
    if ctr[0]>0.15: best=i
hole0=lab==best
hole=ndi.binary_dilation(hole0,iterations=1)
inH=np.zeros(len(C),bool); inH[ok]=hole[ij[ok,0],ij[ok,1]]
he=ndi.binary_erosion(hole0,iterations=2); inHe=np.zeros(len(C),bool); inHe[ok]=he[ij[ok,0],ij[ok,1]]
# top-visible height per cell inside the opening
Hc=np.full(hole.shape,-9.0); np.maximum.at(Hc,(ij[ok&inHe,0],ij[ok&inHe,1]),C[ok&inHe,2])
vals=Hc[he]; plate=np.percentile(vals[vals>-9],5)
r_,c_=np.nonzero(hole0); print("opening x",round(lo[0]+r_.min()*h,3),round(lo[0]+(r_.max()+1)*h,3),"y",round(lo[1]+c_.min()*h,3),round(lo[1]+(c_.max()+1)*h,3))
# lowest shell rim point around the opening (pod front face runs down to it)
from scipy.sparse import coo_matrix
SFc=S['f'].astype(np.int64); Es=np.sort(np.concatenate([SFc[:,[0,1]],SFc[:,[1,2]],SFc[:,[2,0]]]),1); ks=Es[:,0]*len(SV)+Es[:,1]; uk,cn=np.unique(ks,return_counts=True)
bvs=np.unique(np.stack([uk[cn==1]//len(SV),uk[cn==1]%len(SV)],1).ravel()); BV=SV[bvs]
bij=cell(BV); okb=(bij[:,0]>=0)&(bij[:,0]<400)&(bij[:,1]>=0)&(bij[:,1]<300)
near=np.zeros(len(BV),bool); hd=ndi.binary_dilation(hole0,iterations=2); near[okb]=hd[bij[okb,0],bij[okb,1]]
rimz=BV[near,2].min(); print("shell rim min z around opening",round(rimz,4))
Vraw=np.load('asm_middle2_raw.npz')['v']
if len(Vraw)==len(V):
    disp=np.linalg.norm(V-Vraw,axis=1); pushedF=(disp[F]>0.0005).any(1)   # faces pushed under the shell lip stay dark
else:
    pushedF=np.zeros(len(F),bool)
sel=inH&(C[:,2]>min(plate-0.003,rimz-0.004))&(L==0)&~pushedF; L[sel]=3; print("pod faces",sel.sum(),"plate z",round(plate,4),"excluded pushed",int((inH&pushedF).sum()))
# 4 sensor window on pod front face
o=np.array([0.297,0,0.217]); right=np.array([0,1.0,0]); up=np.array([0.131,0,0.991]); up/=np.linalg.norm(up); dv=np.cross(right,up); dv/=np.linalg.norm(dv)
if dv[0]<0: dv=-dv
P=C-o; u=P@right; v=P@up; w=P@dv
win={"u0":-0.0286,"u1":0.0200,"v0":-0.0092,"v1":0.0148}
sel=(rrsdf(u,v,win['u0'],win['u1'],win['v0'],win['v1'],0.0012)<-0.0002)&(np.abs(w)<0.004)&(FN@dv>0.6)
L[sel]=4; print("window faces",sel.sum())
# 5 LED lenses: left from region growing, right by mirror
FR=np.load('seg_front_regions.npz'); ledL=FR['led_L']
CL=C[ledL]; L[ledL]=5
T=cKDTree(np.c_[CL[:,1],CL[:,2]]*[-1,1])
xlo,xhi=CL[:,0].min()-0.003,CL[:,0].max()+0.003
cand=(C[:,1]<0)&(C[:,0]>xlo)&(C[:,0]<xhi)&(FN[:,0]>0.5)
dd,_=T.query(np.c_[C[cand,1],C[cand,2]],workers=8)
idx=np.nonzero(cand)[0][dd<0.0006]; L[idx]=5; print("LED faces L",len(ledL),"R",len(idx))
# 6 slot interior: inside slot opening (y,z) and recessed behind front face
slot=(C[:,0]>0.26)&(C[:,1]>-0.058)&(C[:,1]<0.043)&(C[:,2]>0.134)&(C[:,2]<0.1594)
xf=np.percentile(C[slot&(FN[:,0]>0.8),0],95)
sel=slot&(C[:,0]<xf-0.002)&(L==0); L[sel]=6; print("slot faces",sel.sum(),"front x",round(xf,4))
# majority smoothing of label borders (edge-adjacent faces)
E=np.concatenate([F[:,[0,1]],F[:,[1,2]],F[:,[2,0]]]); E=np.sort(E,1); fi=np.tile(np.arange(len(F)),3)
key=E[:,0].astype(np.int64)*len(V)+E[:,1]; oo=np.argsort(key); ks=key[oo]; fs=fi[oo]
same=ks[1:]==ks[:-1]; A_=fs[:-1][same]; B_=fs[1:][same]
PF=np.concatenate([A_,B_]); PN=np.concatenate([B_,A_]); so=np.argsort(PF,kind='stable'); PF=PF[so]; PN=PN[so]
first=np.r_[0,np.nonzero(PF[1:]!=PF[:-1])[0]+1]; grp=np.repeat(first,np.diff(np.r_[first,len(PF)])); rank=np.arange(len(PF))-grp
keep=rank<3
for it in range(3):
    nb=np.full((len(F),3),-1,np.int64); nb[PF[keep],rank[keep]]=L[PN[keep]]
    n0,n1,n2=nb[:,0],nb[:,1],nb[:,2]
    maj=np.where((n0==n1)&(n0>=0),n0,np.where((n1==n2)&(n1>=0),n1,np.where((n0==n2)&(n0>=0),n0,-1)))
    chg=(maj>=0)&(maj!=L)
    L[chg]=maj[chg].astype(L.dtype); print("label smoothing pass",it,"changed",int(chg.sum()))
# UVs for textured regions (per vertex of those faces)
geom={"display":dg,"display_r":R,"lcd":G['lcd'],"dots":G['dots'],"window":win,"pod_frame":{"o":o.tolist(),"right":right.tolist(),"up":up.tolist(),"dv":dv.tolist()}}
json.dump(geom,open('mid_tex_geom.json','w'),indent=1)
np.savez('asm_middle2_lab.npz',v=V,f=F,lab=L)
print("counts",{NAMES[i]:int((L==i).sum()) for i in range(len(NAMES))})
