import numpy as np,json
d=np.load('asm_wheels.npz'); V=d['v']; F=d['f'].astype(np.int64)
out={}
for s,nm in [(1,'L'),(-1,'R')]:   # +y = left side when facing +x
    m=np.sign(V[:,1])==s; W=V[m]
    c=W.mean(0); _,sv,vt=np.linalg.svd(W[::7]-c,full_matrices=False); ax=vt[2]
    if ax[1]*s<0: ax=-ax          # axle points outward
    # center in plane: bbox center of projected coords
    e1=np.cross(ax,[0,0,1.0]); e1/=np.linalg.norm(e1); e2=np.cross(ax,e1)
    P=W-c; a1=P@e1; a2=P@e2; t=P@ax
    cen=c+e1*(a1.max()+a1.min())/2+e2*(a2.max()+a2.min())/2
    P=W-cen; t=P@ax; r=np.linalg.norm(P-t[:,None]*ax,axis=1)
    print(nm,"axle",ax.round(4),"center",cen.round(4),"Rmax",r.max().round(4),"axial range",t.min().round(4),t.max().round(4))
    # outer-face profile: for radius bins, max axial coordinate (outermost surface)
    bins=np.arange(0,0.118,0.002); prof=[]
    for b0 in bins:
        mm=(r>=b0)&(r<b0+0.002)
        prof.append(t[mm].max() if mm.any() else np.nan)
    print("  r(mm):outer_t(mm) ", " ".join(f"{b*1000:.0f}:{p*1000:.1f}" for b,p in zip(bins,prof)))
    out[nm]={"axle":ax.tolist(),"center":cen.tolist(),"rmax":float(r.max()),"tmax":float(t.max()),"tmin":float(t.min())}
json.dump(out,open('wheel_geom.json','w'),indent=1)
