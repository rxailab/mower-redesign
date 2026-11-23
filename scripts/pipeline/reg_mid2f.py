import numpy as np,sys; sys.path.insert(0,'.'); from geo import vnormals
from scipy.spatial import cKDTree
rng=np.random.default_rng(11)
M=np.load('asm_middle2_raw.npz'); V=M['v'].copy(); F=M['f'].astype(np.int64)
V[:,1]*=-1; F=F[:,[0,2,1]]          # mirror across y=0 to fix handedness
O=np.load('asm_middle_raw.npz'); OV=O['v']; OF=O['f'].astype(np.int64); ON,_=vnormals(OV,OF)
sel=rng.choice(len(OV),2500000,replace=False); Ts=OV[sel]; TN=ON[sel]; T=cKDTree(Ts)
P0=V[rng.choice(len(V),60000,replace=False)]
def run(R,t,lim,it=60):
    for k in range(it):
        Q=P0@R.T+t; dd,ii=T.query(Q,workers=8); m=dd<min(lim,np.quantile(dd,0.75))
        q=Q[m]; p=Ts[ii[m]]; n=TN[ii[m]]
        A=np.c_[np.cross(q,n),n]; b=np.sum((p-q)*n,1); x=np.linalg.lstsq(A,b,rcond=None)[0]; w=x[:3]; th=np.linalg.norm(w)
        K=np.array([[0,-w[2],w[1]],[w[2],0,-w[0]],[-w[1],w[0],0]])/max(th,1e-12); dR=np.eye(3)+np.sin(th)*K+(1-np.cos(th))*K@K
        R=dR@R; t=dR@t+x[3:]
    dd,_=T.query(P0@R.T+t,workers=8); return R,t,dd
# coarse trimmed point-to-point first
R=np.eye(3); t=np.zeros(3)
for k in range(40):
    Q=P0@R.T+t; dd,ii=T.query(Q,workers=8); m=dd<=np.quantile(dd,0.6)
    A=Q[m];B=Ts[ii[m]];ca,cb=A.mean(0),B.mean(0)
    U,sv,Wt=np.linalg.svd((A-ca).T@(B-cb)); D=np.diag([1,1,np.linalg.det(Wt.T@U.T)]); dR=Wt.T@D@U.T; R=dR@R; t=dR@(t-ca)+cb
for lim in [0.006,0.003,0.002]:
    R,t,dd=run(R,t,lim); print("lim",lim,"med mm",round(np.median(dd)*1000,3),"<1mm",round((dd<0.001).mean(),3),"<2mm",round((dd<0.002).mean(),3))
V2=V@R.T+t
np.savez('asm_middle2_raw.npz',v=V2,f=F); print("bbox",V2.min(0).round(3),V2.max(0).round(3))
# record full transform from raw OBJ (mm) to world: world = Rm @ (S raw) ...
G=np.load('mid2_to_world.npz'); Rold,told=G['R'],G['t']   # world_prev = raw@Rold.T + told
Mir=np.diag([1,-1,1.0])
Rn=R@Mir@Rold; tn=R@(Mir@told)+t
np.savez('mid2_to_world.npz',R=Rn,t=tn,mirrored=True)
raw=np.load('mid2_raw.npz')['v'][:5]; print("check",np.abs(raw@Rn.T+tn-V2[:5]).max())
