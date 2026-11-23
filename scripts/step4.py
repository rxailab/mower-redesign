import numpy as np, sys, time; sys.path.insert(0,sys.argv[1]); from geo import *
d=np.load(sys.argv[2]); V=d['v']; F=d['f']; NIT=int(sys.argv[4])
def bstruct(F,nv):
    E,cnt,_=unique_edges(F,nv); return E,E[cnt==1]
# 1 fill tiny holes
nv=len(V); E,be=bstruct(F,nv)
lab=components(nv,be); bv=np.unique(be); l,lc=np.unique(lab[bv],return_counts=True)
newF=[]
for L in l[lc<=12]:
    vs=bv[lab[bv]==L]; ed=be[np.isin(be[:,0],vs)]
    # order loop
    adj={}
    for a,b in ed: adj.setdefault(a,[]).append(b); adj.setdefault(b,[]).append(a)
    loop=[vs[0]]; prev=None
    while True:
        nx=[x for x in adj[loop[-1]] if x!=prev]
        if not nx or nx[0]==loop[0]: break
        prev=loop[-1]; loop.append(nx[0])
    for i in range(1,len(loop)-1): newF.append([loop[0],loop[i],loop[i+1]])
if newF:
    nf=np.array(newF)
    # orient consistent with neighbor face: check edge direction
    F=np.vstack([F,nf])
    E,cnt,_=unique_edges(F,nv)
print("filled tris",len(newF))
# fix orientation of filled tris: compare normal with vertex normal
N,fn=vnormals(V,F[:len(F)-len(newF)])
for i in range(len(F)-len(newF),len(F)):
    f=F[i]; n=np.cross(V[f[1]]-V[f[0]],V[f[2]]-V[f[0]])
    if n@N[f].sum(0)<0: F[i]=f[[0,2,1]]
# 2 erode ragged boundary: remove faces touching boundary for k rings, then drop slivers (faces w/ 2 boundary edges) repeatedly
for ring in range(5):
    E,be=bstruct(F,nv); isb=np.zeros(nv,bool); isb[be]=True
    F=F[~isb[F].any(1)]
for it in range(20):
    E,cnt,inv=unique_edges(F,nv); fb=(cnt[inv]==1).reshape(3,-1).T.sum(1)
    if (fb>=2).sum()==0: break
    F=F[fb<2]
# drop small pieces
E,cnt,_=unique_edges(F,nv); lab=components(nv,E)
u,c=np.unique(lab[F[:,0]],return_counts=True); keep=u[c>1000]
V,F=compact(V,F,np.isin(lab[F[:,0]],keep)); nv=len(V); print("after erosion",nv,len(F),"pieces kept",len(keep))
# remove bowtie verts issues: boundary vertex with boundary-degree != 2
E,be=bstruct(F,nv); bdeg=np.bincount(be.ravel(),minlength=nv)
print("bad bverts",(bdeg>2).sum())
isb=bdeg>0; good_b=(bdeg==2)
# 3 smooth boundary curves (Taubin on boundary graph)
t=time.time()
def blap(V):
    S=np.zeros_like(V)
    for k in range(3): S[:,k]=np.bincount(be[:,0],V[be[:,1],k],nv)+np.bincount(be[:,1],V[be[:,0],k],nv)
    return S/np.maximum(bdeg,1)[:,None]-V
for i in range(1500):
    for lam in(0.5,-0.53):
        V[good_b]+=lam*blap(V)[good_b]
# 4 surface Taubin, boundary fixed for first part, then all lightly
inner=~isb
for i in range(NIT):
    V[inner]+=0.5*laplacian(V,E)[inner]
for i in range(60):
    for lam in(0.6,-0.63):
        V[inner]+=lam*laplacian(V,E)[inner]
# final joint pass: boundary rides along with neighbours lightly
for i in range(150):
    L=laplacian(V,E); B=blap(V)
    if i%10==0: N,_=vnormals(V,F)
    Ln=(L*N).sum(1,keepdims=True)*N
    V[inner]+=0.5*L[inner]; V[good_b]+=0.4*B[good_b]+0.5*Ln[good_b]
print("smooth time",time.time()-t)
np.savez(sys.argv[3],v=V,f=F)
