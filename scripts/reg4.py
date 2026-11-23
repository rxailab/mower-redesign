import numpy as np,sys; sys.path.insert(0,'.'); from geo import vnormals
from scipy.spatial import cKDTree
rng=np.random.default_rng(2)
d=np.load('s10.npz'); S=d['v']; SF=d['f'].astype(np.int64)
N,_=vnormals(S,SF)
if np.mean(np.sum(N*(S-S.mean(0)),1))<0: N=-N
idx=rng.choice(len(S),40000,replace=False); P=(S-0.0025*N)[idx]
d=np.load('mid_up.npz'); M=d['v']; MF=d['f'].astype(np.int64); MN,_=vnormals(M,MF)
sel=rng.choice(len(M),2000000,replace=False); Ms=M[sel]; MNs=MN[sel]; T=cKDTree(Ms)
G=np.load('reg180.npz'); R,t=G['R'],G['t']
# point-to-plane refine
for k in range(60):
    Q=P@R.T+t; dd,ii=T.query(Q,workers=8); m=dd<min(0.006,np.quantile(dd,0.85))
    q=Q[m]; p=Ms[ii[m]]; n=MNs[ii[m]]
    A=np.c_[np.cross(q,n),n]; b=np.sum((p-q)*n,1)
    x=np.linalg.lstsq(A,b,rcond=None)[0]; w=x[:3]; th=np.linalg.norm(w)
    if th>0:
        K=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])/th; dR=np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K
    else: dR=np.eye(3)
    R=dR@R; t=dR@t+x[3:]
Q=P@R.T+t; dd,ii=T.query(Q,workers=8); sd=np.sum((Q-Ms[ii])*MNs[ii],1)
near=dd<0.006
print("med",np.median(dd)*1000,"<3mm",(dd<0.003).mean(),"<10",(dd<0.01).mean())
print("signed among near q05 q25 q50 q75",np.quantile(sd[near],[.05,.25,.5,.75])*1000)
# middle normals orientation: check sign on top surface (should point +z there)
top=Ms[:,2]>Ms[:,2].max()-0.01; print("mid normal z on top mean",MNs[top,2].mean())
# middle -> shell frame
Ri=R.T; ti=-R.T@t
np.savez('mid_to_world.npz',R=Ri,t=ti)
