import numpy as np,sys; sys.path.insert(0,'.'); from geo import vnormals
from scipy.spatial import cKDTree
rng=np.random.default_rng(1)
d=np.load('s10.npz'); S=d['v']; SF=d['f'].astype(np.int64)
N,_=vnormals(S,SF)
if np.mean(np.sum(N*(S-S.mean(0)),1))<0: N=-N
idx=rng.choice(len(S),20000,replace=False); P=(S-0.0025*N)[idx]
d=np.load('mid_up.npz'); M=d['v']; Ms=M[rng.choice(len(M),1000000,replace=False)]; T=cKDTree(Ms)
def icp(R,t,it=80,trim=0.6):
    for k in range(it):
        Q=P@R.T+t; dd,ii=T.query(Q,workers=8); m=dd<=np.quantile(dd,trim)
        A=Q[m];B=Ms[ii[m]];ca,cb=A.mean(0),B.mean(0)
        U,sv,Wt=np.linalg.svd((A-ca).T@(B-cb)); D=np.diag([1,1,np.linalg.det(Wt.T@U.T)]); dR=Wt.T@D@U.T
        R=dR@R; t=dR@(t-ca)+cb
    dd,_=T.query(P@R.T+t,workers=8); return R,t,dd
res=[]
for dyaw in [-5,0,5]:
  a=np.radians(180+dyaw); R=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
  for dx in [-0.04,-0.02,0,0.02,0.04]:
    for dz in [-0.02,-0.06]:
      Q=P@R.T; t=np.array([(Ms[:,0].max()+Ms[:,0].min())/2-(Q[:,0].max()+Q[:,0].min())/2+dx,(Ms[:,1].max()+Ms[:,1].min())/2-(Q[:,1].max()+Q[:,1].min())/2,Ms[:,2].max()-Q[:,2].max()+dz])
      R2,t2,dd=icp(R,t); res.append((np.median(dd),R2,t2))
      print(f"dyaw{dyaw} dx{dx} dz{dz}: med {np.median(dd)*1000:.2f} <3mm {(dd<0.003).mean():.2f} <10mm {(dd<0.01).mean():.2f}",flush=True)
res.sort(key=lambda r:r[0]); np.savez('reg180.npz',R=res[0][1],t=res[0][2]); print("best",res[0][0])
