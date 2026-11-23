# keep outer skin, level, analyze boundary loops
import numpy as np, sys; sys.path.insert(0,sys.argv[1]); from geo import *
d=np.load(sys.argv[2]); V=d['v']; F=d['f'].astype(np.int64); nv=len(V)
E,cnt,_=unique_edges(F,nv)
lab=components(nv,E); u,c=np.unique(lab,return_counts=True); big=u[np.argmax(c)]
V,F=compact(V,F,lab[F[:,0]]==big); nv=len(V); print("outer skin",nv,len(F))
# PCA frame
c0=V.mean(0); _,_,vt=np.linalg.svd(V[::20]-c0,full_matrices=False)
if np.linalg.det(vt)<0: vt[1]*=-1
V=(V-c0)@vt.T
# make dome up: rim lower than centre
r2=(V[:,0]/V[:,0].std())**2+(V[:,1]/V[:,1].std())**2
if np.corrcoef(r2,V[:,2])[0,1]>0: V[:,2]*=-1; V[:,1]*=-1
# level: lowest point in each quadrant, fit plane
pts=[]
for sx in(-1,1):
    for sy in(-1,1):
        m=(np.sign(V[:,0])==sx)&(np.sign(V[:,1])==sy); i=np.argmin(V[m,2]); pts.append(V[m][i])
pts=np.array(pts); print("feet",pts)
A=np.c_[pts[:,0],pts[:,1],np.ones(4)]; a,b,cc=np.linalg.lstsq(A,pts[:,2],rcond=None)[0]
n=np.array([-a,-b,1]); n/=np.linalg.norm(n); print("tilt deg",np.degrees(np.arccos(n[2])))
z=np.array([0,0,1.]); ax=np.cross(n,z); s=np.linalg.norm(ax); 
if s>1e-9:
    ax/=s; ang=np.arcsin(s); K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]])
    R=np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K; V=V@R.T
V[:,0]-=(V[:,0].max()+V[:,0].min())/2; V[:,1]-=(V[:,1].max()+V[:,1].min())/2; V[:,2]-=V[:,2].min()
print("bbox",V.min(0),V.max(0))
# boundary loops
E,cnt,_=unique_edges(F,nv); be=E[cnt==1]
lab=components(nv,be); bv=np.unique(be); l,lc=np.unique(lab[bv],return_counts=True)
print("boundary loops",len(l),"sizes",sorted(lc)[::-1][:15])
np.savez(sys.argv[3],v=V,f=F)
