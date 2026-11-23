import numpy as np,sys,json
from scipy import ndimage as ndi
D=np.load('top_H.npy'); px=0.0003; umin,vmax=-0.27,0.12
Ds=ndi.gaussian_filter(D,1.2)
c=lambda x:int(round((x-umin)/px-0.5)); r=lambda y:int(round((vmax-y)/px-0.5))
X=lambda cc:umin+(cc+0.5)*px; Y=lambda rr:vmax-(rr+0.5)*px
def edge_row(y,xa,xb,sign):   # find strongest step along x in [xa,xb]; sign +1 rising with x, -1 falling
    rows=[r(y+dy) for dy in np.linspace(-0.004,0.004,9)]
    prof=Ds[rows,:].mean(0); gx=np.gradient(prof,px)
    a,b=c(xa),c(xb); i=a+np.argmax(sign*gx[a:b]); return X(i), sign*gx[i]
def edge_col(x,ya,yb,sign):   # step along +y
    cols=[c(x+dx) for dx in np.linspace(-0.004,0.004,9)]
    prof=Ds[:,cols].mean(1)[::-1]; gy=np.gradient(prof,px)   # reversed so index increases with y
    H=Ds.shape[0]; ia=H-1-r(ya); ib=H-1-r(yb); i=ia+np.argmax(sign*gy[ia:ib]); return Y(H-1-i), sign*gy[i]
# display outer borders (plateau raised): left rising, right falling, bottom rising (with +y), top falling
L=edge_row(0.0015,-0.120,-0.105,+1); R=edge_row(0.0015,-0.030,-0.018,-1)
B=edge_col(-0.068,-0.065,-0.050,+1); T=edge_col(-0.068,0.052,0.068,-1)
print("display L,R,B,T",L,R,B,T)
# LCD inner rect edges (could be step up or down): search both signs pick strongest abs
def best_row(y,xa,xb):
    e1=edge_row(y,xa,xb,1); e2=edge_row(y,xa,xb,-1); return e1 if e1[1]>e2[1] else e2
def best_col(x,ya,yb):
    e1=edge_col(x,ya,yb,1); e2=edge_col(x,ya,yb,-1); return e1 if e1[1]>e2[1] else e2
lL=best_row(0.0026,-0.083,-0.074); lR=best_row(0.0026,-0.045,-0.036); lB=best_col(-0.06,-0.043,-0.034); lT=best_col(-0.06,0.039,0.048)
print("lcd L,R,B,T",lL,lR,lB,lT)
# dots refine: local max of detail in 3mm windows around estimates
det=ndi.gaussian_filter(D-ndi.gaussian_filter(D,10),2)
dots=[]
for y0 in [-0.0318,-0.0144,0.003,0.0201,0.0375]:
    x0=-0.0936; rr0,cc0=r(y0),c(x0); w=10
    sub=det[rr0-w:rr0+w+1,cc0-w:cc0+w+1]; i,j=np.unravel_index(np.argmax(sub),sub.shape)
    dots.append([X(cc0-w+j),Y(rr0-w+i)])
print("dots",np.round(dots,4).tolist())
disp={"x0":L[0],"x1":R[0],"y0":B[0],"y1":T[0]}; lcd={"x0":lL[0],"x1":lR[0],"y0":lB[0],"y1":lT[0]}
json.dump({"display":disp,"display_r":0.009,"lcd":lcd,"dots":dots},open('disp_geom.json','w'),indent=1)
