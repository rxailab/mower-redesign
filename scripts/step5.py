# edge-band refinement: remove notches on boundary outlines and ripples near edges
import numpy as np, sys, time; sys.path.insert(0,sys.argv[1]); from geo import *
d=np.load(sys.argv[2]); V=d['v'].copy(); F=d['f']; nv=len(V)
RINGS=int(sys.argv[4]); NB=int(sys.argv[5]); NS=int(sys.argv[6])
E,cnt,_=unique_edges(F,nv); be=E[cnt==1]
isb=np.zeros(nv,bool); isb[be]=True
band=isb.copy()
for r in range(RINGS):
    nb=band.copy(); nb[E[band[E[:,0]],1]]=True; nb[E[band[E[:,1]],0]]=True; band=nb
# local index: band + 1-ring halo
halo=band.copy(); halo[E[band[E[:,0]],1]]=True; halo[E[band[E[:,1]],0]]=True
gl=np.where(halo)[0]; loc=-np.ones(nv,np.int64); loc[gl]=np.arange(len(gl))
El=loc[E[halo[E[:,0]]&halo[E[:,1]]]]; bel=loc[be]
mov=band[gl]; isbl=isb[gl]; inner=mov&~isbl
X=V[gl].copy(); n=len(gl); print("band verts",band.sum(),"local",n)
deg=np.bincount(El.ravel(),minlength=n); bdeg=np.bincount(bel.ravel(),minlength=n)
def lap(X,Ed,dg):
    S=np.empty_like(X)
    for k in range(3): S[:,k]=np.bincount(Ed[:,0],X[Ed[:,1],k],n)+np.bincount(Ed[:,1],X[Ed[:,0],k],n)
    return S/np.maximum(dg,1)[:,None]-X
t=time.time()
# 1) outline: low-passband Taubin along boundary curves
for i in range(NB):
    for lam in(0.5,-0.505):
        X[isbl]+=lam*lap(X,bel,bdeg)[isbl]
print("curve",time.time()-t)
# 2) band surface: low-passband Taubin; boundary verts follow curve + surface normal component
Fl_mask=halo[F].all(1); Fl=loc[F[Fl_mask]]
def vn(X):
    fn=np.cross(X[Fl[:,1]]-X[Fl[:,0]],X[Fl[:,2]]-X[Fl[:,0]]); N=np.zeros_like(X)
    for k in range(3): N[:,k]=np.bincount(Fl.ravel(),np.repeat(fn[:,k],3),n)
    return N/np.maximum(np.linalg.norm(N,axis=1),1e-20)[:,None]
for i in range(NS):
    if i%20==0: N=vn(X)
    for lam in(0.5,-0.505):
        L=lap(X,El,deg); X[inner]+=lam*L[inner]
        Ln=(L*N).sum(1,keepdims=True)*N; B=lap(X,bel,bdeg)
        X[isbl]+=lam*(0.6*Ln[isbl]+0.5*B[isbl])
print("band",time.time()-t)
V[gl[mov]]=X[mov]
print("max move mm",np.abs(V-d['v']).max()*1000)
np.savez(sys.argv[3],v=V,f=F)
