import numpy as np
from scipy.spatial import cKDTree
rng=np.random.default_rng(3)
B=np.load('base_up.npz')['v']; Bs=B[rng.choice(len(B),2000000,replace=False)]
a=np.radians(190.5); Y=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
Bs=Bs@Y.T
M=np.load('mid_up.npz')['v']; W=np.load('mid_to_world.npz'); Mw=M@W['R'].T+W['t']
P=Mw[rng.choice(len(Mw),30000,replace=False)]
T=cKDTree(Bs)
print("base bbox",Bs.min(0),Bs.max(0)); print("mid bbox",Mw.min(0),Mw.max(0))
def icp(R,t,it=80,trim=0.4):
    for k in range(it):
        Q=P@R.T+t; dd,ii=T.query(Q,workers=8); m=dd<=np.quantile(dd,trim)
        A=Q[m];Bb=Bs[ii[m]];ca,cb=A.mean(0),Bb.mean(0)
        U,sv,Wt=np.linalg.svd((A-ca).T@(Bb-cb)); D=np.diag([1,1,np.linalg.det(Wt.T@U.T)]); dR=Wt.T@D@U.T
        R=dR@R; t=dR@(t-ca)+cb
    dd,_=T.query(P@R.T+t,workers=8); return R,t,dd
res=[]
cx=lambda X:(X.max(0)+X.min(0))/2
for dyaw in [-6,0,6]:
  b=np.radians(dyaw); R=np.array([[np.cos(b),-np.sin(b),0],[np.sin(b),np.cos(b),0],[0,0,1]])
  for dx in [-0.03,0,0.03]:
    for zt in [0.03,0.06,0.09]:
      Q=P@R.T; t=cx(Bs)-cx(Q); t[0]+=dx; t[2]=zt-Q[:,2].min()
      R2,t2,dd=icp(R,t); res.append((np.median(dd),R2,t2,dyaw,dx,zt))
      print(f"dyaw{dyaw} dx{dx} z{zt}: med {np.median(dd)*1000:.2f} q40 {np.quantile(dd,.4)*1000:.2f} <3mm {(dd<0.003).mean():.2f}",flush=True)
res.sort(key=lambda r:np.quantile(r[0],1)); 
np.savez('reg_mid_base.npz',R=res[0][1],t=res[0][2],Y=Y); print("best",res[0][0],res[0][3:])
