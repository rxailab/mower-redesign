# middle-layer fit to the top shell: wheel clearance, seat the shell edges, tuck covered areas under the shell
import numpy as np,sys; sys.path.insert(0,'.'); from geo import vnormals
from scipy.sparse import csr_matrix
from scipy.spatial import cKDTree
d=np.load('asm_middle2_raw.npz'); V=d['v'].copy(); F=d['f'].astype(np.int64); nv=len(V)
S=np.load('asm_shell.npz'); SV=S['v']; SF=S['f'].astype(np.int64)
WH=np.load('asm_wheels.npz')['v']
def uedges(F,nv):
    E=np.sort(np.concatenate([F[:,[0,1]],F[:,[1,2]],F[:,[2,0]]]),1); k=E[:,0]*nv+E[:,1]; u,c=np.unique(k,return_counts=True); return np.stack([u//nv,u%nv],1),c
E,c=uedges(F,nv)
Aa=csr_matrix((np.ones(2*len(E)),(np.r_[E[:,0],E[:,1]],np.r_[E[:,1],E[:,0]])),shape=(nv,nv)); da=np.asarray(Aa.sum(1)).ravel()
# 1. wheel clearance
for s in (-1,1):
    w=WH[np.sign(WH[:,1])==s]; cx=(w[:,0].min()+w[:,0].max())/2; cz=(w[:,2].min()+w[:,2].max())/2; r=(w[:,2].max()-w[:,2].min())/2
    ylo,yhi=w[:,1].min()-0.002,w[:,1].max()+0.002
    rr=np.hypot(V[:,0]-cx,V[:,2]-cz); m=(V[:,1]>ylo)&(V[:,1]<yhi)&(rr<r+0.003)
    k=(r+0.003)/np.maximum(rr[m],1e-6); V[m,0]=cx+(V[m,0]-cx)*k; V[m,2]=cz+(V[m,2]-cz)*k
SN,_=vnormals(SV,SF)
if SN[SV[:,2]>np.percentile(SV[:,2],90),2].mean()<0: SN=-SN
Es,cs=uedges(SF,len(SV)); bidx=np.unique(Es[cs==1].ravel()); isb=np.zeros(len(SV),bool); isb[bidx]=True
# 2. seat the shell edges: lift middle surface that sits below the shell's inner edge, within 9 mm of the edge
MN,_=vnormals(V,F)
pe=SV[bidx]-0.0025*SN[bidx]; ne=SN[bidx]; Tb=cKDTree(pe)
db,ib=Tb.query(V,workers=8)
near=db<0.012
off=V[near]-pe[ib[near]]; n=ne[ib[near]]
h=np.sum(off*n,1); lat=np.linalg.norm(off-h[:,None]*n,axis=1)
facing=np.abs(np.sum(MN[near]*n,1))>0.3
wgt=np.clip(1-(lat-0.002)/0.007,0,1); wgt=wgt*wgt*(3-2*wgt)
lift=np.where((h<-0.0003)&(h>-0.006)&facing,(-h-0.0003)*wgt,0.0)
L=np.zeros(nv); L[np.nonzero(near)[0]]=lift
for it in range(6):                                  # smooth the lift field a little (no spikes)
    L=np.maximum(L*0.6,(Aa@L)/np.maximum(da,1)*0.95)*(L>0)+np.where(L>0,0,0)
Nv=np.zeros_like(V); Nv[np.nonzero(near)[0]]=n
V+=L[:,None]*Nv
print("seat: lifted verts",int((L>1e-5).sum()),"max lift mm",round(L.max()*1000,2),"median mm",round(np.median(L[L>1e-5])*1000,2) if (L>1e-5).any() else 0)
# 3. tuck covered areas under the shell (no spill outside the shell's footprint)
T=cKDTree(SV); dd,ii=T.query(V,workers=8); n=SN[ii]; off=V-SV[ii]; sd=np.sum(off*n,1)
lat=np.linalg.norm(off-sd[:,None]*n,axis=1)
covered=(dd<0.02)&(lat<0.0015)&~isb[ii]
rimpoke=isb[ii]&(dd<0.006)&(lat<0.003)&(sd>-0.001)
allowed=covered|rimpoke
need=np.where(allowed,np.maximum(0,sd+0.0028),0.0)
t=need.copy()
for it in range(25):
    avg=(Aa@t)/np.maximum(da,1); t=np.where(allowed,np.maximum(need,0.85*avg),0.0)
print("tuck: pushed verts",int((t>1e-5).sum()),"max mm",round(t.max()*1000,2),"rim poke",int(rimpoke.sum()))
V=V-t[:,None]*n
np.savez('asm_middle2.npz',v=V,f=F)
