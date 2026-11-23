# Fixed parts and constraints for the upper-body redesign.
#  keep.npz     : lower skirt of the middle shell + bumper + front face with the two charger openings (unchanged)
#  control.npz  : STOP button + display area of the old middle (goes under the new lid, original position)
#  constraints.json : belt outline (plan), cut heights, wheel boxes; chassis_top.npy : chassis height map
import numpy as np, json
W='../work/'
M=np.load(W+'asm_middle2_lab.npz'); V=M['v']; F=M['f'].astype(np.int64); L=M['lab']
C=V[F].mean(1)
ss=lambda a,b,x: np.clip((x-a)/(b-a),0,1)**2*(3-2*np.clip((x-a)/(b-a),0,1))
def zcut(x): return 0.135-0.012*ss(0.20,0.25,x)      # front: new fascia reaches down to the bumper top
wheelzone=(C[:,0]>-0.03)&(C[:,0]<0.262)&(np.abs(C[:,1])>0.168)
keep=((C[:,2]<zcut(C[:,0]))&~wheelzone)|(L==7)|((L==6)&(C[:,0]>0.27)&(C[:,0]<0.3045)&(C[:,2]<0.162))   # + charger-opening interiors
def sub(mask):
    f=F[mask]; u=np.unique(f); r=-np.ones(len(V),np.int64); r[u]=np.arange(len(u)); return V[u],r[f],L[mask]
kv,kf,kl=sub(keep); np.savez('keep.npz',v=kv,f=kf.astype(np.int32),lab=kl); print('keep faces',len(kf))
ctl=(C[:,0]>-0.128)&(C[:,0]<0.034)&(np.abs(C[:,1])<0.062)&(C[:,2]>0.243)
cv,cf,cl=sub(ctl); np.savez('control.npz',v=cv,f=cf.astype(np.int32),lab=cl); print('control faces',len(cf),'z',cv[:,2].min().round(4),cv[:,2].max().round(4))
# belt outline: outer contour of the kept skirt just below the cut, as half-width per x (both sides) and nose/tail x
body=keep&(L!=7)
P=V[np.unique(F[body])]; band=P[P[:,2]>zcut(P[:,0])-0.006]
xs=np.arange(-0.33,0.335,0.005); yl=[];yr=[]
for x0 in xs:
    m=np.abs(band[:,0]-x0)<0.004
    yl.append(float(band[m,1].max()) if m.sum()>5 else None); yr.append(float(band[m,1].min()) if m.sum()>5 else None)
print('belt x range',band[:,0].min().round(4),band[:,0].max().round(4))
Ch=np.load(W+'asm_chassis.npz')['v']; h=0.005
gx=((Ch[:,0]+0.34)/h).astype(int); gy=((Ch[:,1]+0.25)/h).astype(int); H=np.zeros((136,100)); np.maximum.at(H,(gx,gy),Ch[:,2]); np.save('chassis_top.npy',H)
json.dump({'xs':xs.tolist(),'yl':yl,'yr':yr,'x_tail':float(band[:,0].min()),'x_nose':float(band[:,0].max()),
           'wheel':{'x0':0.003,'x1':0.235,'y_in':0.175,'z_top':0.229},'ctl_top':float(cv[:,2].max())},open('constraints.json','w'))
print('yl',[None if v is None else round(v,3) for v in yl][::6])
