import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
def face_normals(V,F):
    n=np.cross(V[F[:,1]]-V[F[:,0]],V[F[:,2]]-V[F[:,0]]); a=np.linalg.norm(n,axis=1); return n/np.maximum(a,1e-20)[:,None],a/2
def ray_pick(V,origin,d,tol=0.0012):
    d=np.asarray(d,float); d/=np.linalg.norm(d); P=V-np.asarray(origin)
    t=P@d; perp=np.linalg.norm(P-t[:,None]*d,axis=1); m=perp<tol
    i=np.nonzero(m)[0]; return i[np.argmin(t[i])]   # closest to origin along ray (origin outside, d pointing in)
def grow(V,F,FN,seed_v,box_lo,box_hi,ang_nb=20,ang_seed=35,vert_mask=None):
    """flood fill faces from faces touching seed vertex, within box, across edges with small normal change"""
    C=V[F].mean(1); inbox=np.all((C>=box_lo)&(C<=box_hi),1)
    fid=np.nonzero(inbox)[0]; Fl=F[fid]; Nl=FN[fid]
    seeds=np.nonzero(np.any(Fl==seed_v,1))[0]
    sn=Nl[seeds].mean(0); sn/=np.linalg.norm(sn)
    ok=(Nl@sn)>np.cos(np.radians(ang_seed))
    # edge adjacency
    E=np.concatenate([Fl[:,[0,1]],Fl[:,[1,2]],Fl[:,[2,0]]]); E=np.sort(E,1); fidx=np.tile(np.arange(len(Fl)),3)
    key=E[:,0].astype(np.int64)*len(V)+E[:,1]; o=np.argsort(key); ks=key[o]; fs=fidx[o]
    same=ks[1:]==ks[:-1]; a=fs[:-1][same]; b=fs[1:][same]
    good=ok[a]&ok[b]&(np.sum(Nl[a]*Nl[b],1)>np.cos(np.radians(ang_nb)))
    A=coo_matrix((np.ones(good.sum()),(a[good],b[good])),shape=(len(Fl),len(Fl)))
    k,lab=connected_components(A,directed=False)
    region=lab==lab[seeds[0]]
    return fid[region],sn
