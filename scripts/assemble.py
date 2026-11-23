import numpy as np
def load(n,rt):
    d=np.load(n); V=d['v']; F=d['f'].astype(np.int64)
    if rt: T=np.load(rt); V=V@T['R'].T+T['t']
    return V,F
def sub(V,F,mask):
    F=F[mask]; u=np.unique(F); r=-np.ones(len(V),np.int64); r[u]=np.arange(len(u)); return V[u],r[F]
S,SF=load('s10.npz',None); M,MF=load('mid_up.npz','mid_to_world.npz'); B,BF=load('base_up.npz','base_to_world2.npz')
fl=np.load('base_up_flab.npy'); cnt=np.bincount(fl); o=np.argsort(-cnt)
parts={'shell':(S,SF),'middle':(M,MF),'chassis':sub(B,BF,np.isin(fl,[o[0],o[5]])),'wheels':sub(B,BF,np.isin(fl,o[1:3])),'casters':sub(B,BF,np.isin(fl,o[3:5]))}
# ground plane from wheel/caster bottoms
pts=[]
for k in ['wheels','casters']:
    V,F=parts[k]
    for s in (-1,1):
        m=np.sign(V[:,1])==s; i=np.argmin(V[m,2]); pts.append(V[m][i])
pts=np.array(pts); print(pts)
A=np.c_[pts[:,0],pts[:,1],np.ones(4)]; a,b,c=np.linalg.lstsq(A,pts[:,2],rcond=None)[0]
n=np.array([-a,-b,1]); n/=np.linalg.norm(n); z=np.array([0,0,1.]); ax=np.cross(n,z); s=np.linalg.norm(ax)
R=np.eye(3)
if s>1e-9:
    ax/=s; ang=np.arcsin(s); K=np.array([[0,-ax[2],ax[1]],[ax[2],0,-ax[0]],[-ax[1],ax[0],0]]); R=np.eye(3)+np.sin(ang)*K+(1-np.cos(ang))*K@K
print("tilt deg",np.degrees(np.arccos(n[2])))
allV=np.vstack([v for v,f in parts.values()])@R.T
cx=(allV[:,0].max()+allV[:,0].min())/2; cy=(allV[:,1].max()+allV[:,1].min())/2
gz=(pts@R.T)[:,2].mean()
for k,(V,F) in parts.items():
    V=V@R.T-np.array([cx,cy,gz]); np.savez(f'asm_{k}.npz',v=V,f=F); print(k,len(V),len(F),V.min(0).round(3),V.max(0).round(3))
