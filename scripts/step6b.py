# anisotropic edge-band smoothing: smooth only along the direction parallel to the nearest boundary
import numpy as np, sys, time; sys.path.insert(0,sys.argv[1]); from geo import *
d=np.load(sys.argv[2]); V=d['v'].copy(); F=d['f']; nv=len(V)
RINGS=int(sys.argv[4]); NIT=int(sys.argv[5]); P=float(sys.argv[6])
E,cnt,_=unique_edges(F,nv); be=E[cnt==1]
isb=np.zeros(nv,bool); isb[be]=True
ring=np.full(nv,10**9); ring[isb]=0; band=isb.copy()
for r in range(1,RINGS+1):
    nb=band.copy(); nb[E[band[E[:,0]],1]]=True; nb[E[band[E[:,1]],0]]=True
    ring[nb&~band]=r; band=nb
halo=band.copy(); halo[E[band[E[:,0]],1]]=True; halo[E[band[E[:,1]],0]]=True
gl=np.where(halo)[0]; loc=-np.ones(nv,np.int64); loc[gl]=np.arange(len(gl)); n=len(gl)
El=loc[E[halo[E[:,0]]&halo[E[:,1]]]]; bel=loc[be]
X=V[gl].copy(); rl=ring[gl]; mov=rl<=RINGS; isbl=rl==0
# tangent tensor at boundary
Tn=np.zeros((n,3))
bt=X[bel[:,1]]-X[bel[:,0]]; bt/=np.linalg.norm(bt,axis=1)[:,None]
T=np.zeros((n,3,3)); 
for a in(0,1): np.add.at(T,bel[:,a],bt[:,:,None]*bt[:,None,:])
# smooth tangent along boundary a bit, then propagate inward ring by ring
deg=np.bincount(El.ravel(),minlength=n)
def avgT(T):
    S=np.zeros_like(T)
    for a,b in((0,1),(1,0)): np.add.at(S,El[:,a],T[El[:,b]])
    return S
known=isbl.copy()
nrm=lambda T: T/np.maximum(np.trace(T,axis1=1,axis2=2),1e-12)[:,None,None]
T=nrm(T)
for r in range(1,RINGS+2):
    S=avgT(T*known[:,None,None]); new=(rl==r)|((~known)&(np.trace(S,axis1=1,axis2=2)>0)&(rl>RINGS))
    new&=~known; T[new]=nrm(S[new]); known|=new
for it in range(10):  # relax tensors inside band (boundary fixed)
    S=nrm(avgT(T)); upd=mov&~isbl; T[upd]=S[upd]
# edge weights
e=X[El[:,1]]-X[El[:,0]]; e/=np.linalg.norm(e,axis=1)[:,None]
Tm=(T[El[:,0]]+T[El[:,1]])/2
w=0.08+np.einsum("ei,eij,ej->e",e,Tm,e).clip(0,1)**P
print("band",mov.sum(),"w mean",w.mean())
fall=np.clip((RINGS-rl)/(RINGS*0.4),0,1)*0.5   # full strength up to 60% of band, fades out
W=np.bincount(El[:,0],w,n)+np.bincount(El[:,1],w,n)
bdeg=np.bincount(bel.ravel(),minlength=n)
t=time.time()
for i in range(NIT):
    S=np.empty_like(X)
    for k in range(3): S[:,k]=np.bincount(El[:,0],w*X[El[:,1],k],n)+np.bincount(El[:,1],w*X[El[:,0],k],n)
    L=S/np.maximum(W,1e-9)[:,None]-X
    Sb=np.empty_like(X)
    for k in range(3): Sb[:,k]=np.bincount(bel[:,0],X[bel[:,1],k],n)+np.bincount(bel[:,1],X[bel[:,0],k],n)
    L[isbl]=(Sb/np.maximum(bdeg,1)[:,None]-X)[isbl]
    X[mov]+=(fall[:,None]*L)[mov]
print("time",time.time()-t)
V[gl[mov]]=X[mov]
dm=np.linalg.norm(V-d['v'],axis=1); print("move mm p50/p99/max",np.percentile(dm[band],[50,99])*1000,dm.max()*1000)
np.savez(sys.argv[3],v=V,f=F)
