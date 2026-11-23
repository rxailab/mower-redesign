import numpy as np,sys,json; sys.path.insert(0,'.')
from segmesh import face_normals,ray_pick,grow
d=np.load('asm_middle2.npz'); V=d['v']; F=d['f'].astype(np.int64)
FN,FA=face_normals(V,F)
out={}
# LED lenses: rays along -x at y=+-0.125, z=0.169
for nm,y in [("led_L",0.125),("led_R",-0.125)]:
    for z in [0.169,0.167,0.171]:
        vi=ray_pick(V,(0.6,y,z),(-1,0,0)); print(nm,"hit",V[vi].round(4))
        reg,sn=grow(V,F,FN,vi,np.array([0.2,min(y*0.5,y*1.5)-0.01,0.14]),np.array([0.36,max(y*0.5,y*1.5)+0.01,0.2]),ang_nb=15,ang_seed=30)
        C=V[F[reg]].mean(1); print("  faces",len(reg),"normal",sn.round(3),"bbox",C.min(0).round(4),C.max(0).round(4))
        if len(reg)>2000: break
    out[nm]=reg
# pod window: ray along view dir
dv=np.array([1,0,0.25]); dv/=np.linalg.norm(dv)
for p in [(0.30008,-0.004,0.2197),(0.30008,0.004,0.2197),(0.30008,-0.012,0.2197)]:
    o=np.array(p)+dv*0.5; vi=ray_pick(V,o,-dv); print("window hit",V[vi].round(4))
    reg,sn=grow(V,F,FN,vi,np.array([0.22,-0.05,0.18]),np.array([0.36,0.05,0.27]),ang_nb=12,ang_seed=20)
    C=V[F[reg]].mean(1); print("  faces",len(reg),"normal",sn.round(3),"bbox",C.min(0).round(4),C.max(0).round(4))
    if len(reg)>3000: break
out['window']=reg
np.savez('seg_front_regions.npz',**out)
