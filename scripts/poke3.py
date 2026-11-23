import numpy as np
exec(open('poke2.py').read().split('for dz in')[0])
Bs=Bw[::3]; c=np.array([0.1,0,0.0])
def ev(dx,dz,pd,rd=0):
    p=np.radians(pd); r=np.radians(rd)
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    X=(Bs-c)@(Rx@Ry).T+c+np.array([dx,0,dz]); ij=((X[:,:2]-lo)/h).astype(int); hz=H[ij[:,0],ij[:,1]]; m=hz>-9
    ex=X[m,2]-hz[m]; return (ex>0.0015).sum(), ((ex<0)&(ex>-0.004)).sum()
res=[]
for pd in np.arange(-4,4.1,1.0):
  for dz in np.arange(-0.02,0.001,0.0025):
    for dx in [-0.01,-0.005,0,0.005,0.01]:
      pk,ct=ev(dx,dz,pd); res.append((pk,ct,pd,dz,dx))
res.sort(key=lambda r:(r[0]))
for r in res[:15]: print(r)
print("--- fewest move with pk<300")
ok=[r for r in res if r[0]<300]; ok.sort(key=lambda r:abs(r[3])+abs(r[4])+0.003*abs(r[2]))
for r in ok[:10]: print(r)
