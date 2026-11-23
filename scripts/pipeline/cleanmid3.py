import numpy as np,sys; sys.path.insert(0,'.'); from geo import vnormals
from scipy.sparse import csr_matrix
from scipy.spatial import cKDTree
d=np.load('asm_middle2_raw.npz'); V=d['v'].copy(); F=d['f'].astype(np.int64); nv=len(V)
S=np.load('asm_shell.npz'); SV=S['v']; SF=S['f'].astype(np.int64)
WH=np.load('asm_wheels.npz')['v']
def uedges(F,nv):
    E=np.sort(np.concatenate([F[:,[0,1]],F[:,[1,2]],F[:,[2,0]]]),1); k=E[:,0]*nv+E[:,1]; u,c=np.unique(k,return_counts=True); return np.stack([u//nv,u%nv],1),c
E,c=uedges(F,nv); bv=np.zeros(nv,bool); bv[E[c==1].ravel()]=True
Aa=csr_matrix((np.ones(2*len(E)),(np.r_[E[:,0],E[:,1]],np.r_[E[:,1],E[:,0]])),shape=(nv,nv)); da=np.asarray(Aa.sum(1)).ravel()
# 1. wheel clearance: push verts radially out of wheel envelope (+3mm)
for s in (-1,1):
    w=WH[np.sign(WH[:,1])==s]; cx=(w[:,0].min()+w[:,0].max())/2; cz=(w[:,2].min()+w[:,2].max())/2; r=(w[:,2].max()-w[:,2].min())/2
    ylo,yhi=w[:,1].min()-0.002,w[:,1].max()+0.002
    rr=np.hypot(V[:,0]-cx,V[:,2]-cz); m=(V[:,1]>ylo)&(V[:,1]<yhi)&(rr<r+0.003)
    k=(r+0.003)/np.maximum(rr[m],1e-6); V[m,0]=cx+(V[m,0]-cx)*k; V[m,2]=cz+(V[m,2]-cz)*k
    print("wheel",s,"pushed",m.sum())
# 2. smooth push-under the shell
SN,_=vnormals(SV,SF)
if SN[SV[:,2]>np.percentile(SV[:,2],90),2].mean()<0: SN=-SN
Es,cs=uedges(SF,len(SV)); isb=np.zeros(len(SV),bool); isb[Es[cs==1].ravel()]=True
T=cKDTree(SV); dd,ii=T.query(V,workers=8); n=SN[ii]; off=V-SV[ii]; sd=np.sum(off*n,1)
lat=np.linalg.norm(off-sd[:,None]*n,axis=1)
rimpoke=isb[ii]&(dd<0.006)&(lat<0.003)&(sd>-0.001)      # pokes through the shell right at an opening rim
print("rim poke verts",rimpoke.sum())
need=np.where(((dd<0.02)&(lat<0.0015)&~isb[ii])|rimpoke,np.maximum(0,sd+0.0032),0.0)
t=need.copy()
for it in range(25):
    avg=(Aa@t)/np.maximum(da,1); t=np.maximum(need,0.85*avg)
print("pushed verts",(t>1e-5).sum(),"max push mm",round(t.max()*1000,2))
V=V-t[:,None]*n
np.savez('asm_middle2.npz',v=V,f=F)
