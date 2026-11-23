import numpy as np
def edges_of(F):
    E=np.sort(np.concatenate([F[:,[0,1]],F[:,[1,2]],F[:,[2,0]]]),1).astype(np.int64)
    return E
def unique_edges(F,nv):
    E=edges_of(F); key=E[:,0]*nv+E[:,1]
    u,inv,cnt=np.unique(key,return_inverse=True,return_counts=True)
    return np.stack([u//nv,u%nv],1),cnt,inv
def components(nv,E):
    p=np.arange(nv)
    for _ in range(1000):
        a=p[E[:,0]];b=p[E[:,1]]
        lo=np.minimum(a,b)
        np.minimum.at(p,E[:,0],lo); np.minimum.at(p,E[:,1],lo)
        p2=p[p]
        while not np.array_equal(p2,p): p=p2; p2=p[p]
        if np.all(p[E[:,0]]==p[E[:,1]]): break
    return p
def compact(V,F,keepf):
    F=F[keepf]; used=np.unique(F); remap=-np.ones(len(V),np.int64); remap[used]=np.arange(len(used))
    return V[used],remap[F]
def laplacian(V,E,w=None,mask=None):
    nv=len(V); deg=np.bincount(E[:,0],minlength=nv)+np.bincount(E[:,1],minlength=nv)
    S=np.zeros_like(V)
    for k in range(3):
        S[:,k]=np.bincount(E[:,0],V[E[:,1],k],nv)+np.bincount(E[:,1],V[E[:,0],k],nv)
    return S/np.maximum(deg,1)[:,None]-V
def vnormals(V,F):
    fn=np.cross(V[F[:,1]]-V[F[:,0]],V[F[:,2]]-V[F[:,0]])
    N=np.zeros_like(V)
    for k in range(3): N[:,k]=np.bincount(F.ravel(),np.repeat(fn[:,k],3),len(V))
    return N/np.maximum(np.linalg.norm(N,axis=1),1e-20)[:,None], fn
