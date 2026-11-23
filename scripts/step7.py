import numpy as np, sys; sys.path.insert(0,sys.argv[1]); from geo import *
d=np.load(sys.argv[2]); V=d['v'].copy(); F=d['f']; nv=len(V); R=int(sys.argv[4]); NIT=int(sys.argv[5])
E,cnt,_=unique_edges(F,nv); be=E[cnt==1]
isb=np.zeros(nv,bool); isb[be]=True; band=isb.copy()
for r in range(R):
    nb=band.copy(); nb[E[band[E[:,0]],1]]=True; nb[E[band[E[:,1]],0]]=True; band=nb
mv=band&~isb
sel=band[E[:,0]]|band[E[:,1]]; Es=E[sel]
deg=np.bincount(Es.ravel(),minlength=nv)
bdeg=np.bincount(be.ravel(),minlength=nv)
for i in range(NIT):
    for lam in (0.5,-0.52):
        S=np.zeros((nv,3))
        for k in range(3): S[:,k]=np.bincount(Es[:,0],V[Es[:,1],k],nv)+np.bincount(Es[:,1],V[Es[:,0],k],nv)
        L=S/np.maximum(deg,1)[:,None]-V; V[mv]+=lam*L[mv]
        Sb=np.zeros((nv,3))
        for k in range(3): Sb[:,k]=np.bincount(be[:,0],V[be[:,1],k],nv)+np.bincount(be[:,1],V[be[:,0],k],nv)
        Lb=Sb/np.maximum(bdeg,1)[:,None]-V; V[isb]+=lam*Lb[isb]
print("max move mm",np.abs(V-d['v']).max()*1000)
np.savez(sys.argv[3],v=V,f=F)
