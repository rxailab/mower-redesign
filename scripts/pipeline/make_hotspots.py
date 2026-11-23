import numpy as np,json,sys
from scipy import ndimage as ndi
OUT=sys.argv[1]
g=lambda v:[round(float(v[0]),4),round(float(v[2]),4),round(float(-v[1]),4)]   # world Z-up -> glTF Y-up
D=np.load('top_H.npy'); px=0.0003; umin,vmax=-0.27,0.12
def htop(x,y): return float(D[int((vmax-y)/px),int((x-umin)/px)])
G=json.load(open('mid_tex_geom.json')); dg=G['display']
M=np.load('seg_top_masks.npz'); stop=M['stop']; r_,c_=np.nonzero(stop)
sx=umin+(c_.mean()+0.5)*px; sy=vmax-(r_.mean()+0.5)*px
d=np.load('asm_middle2_lab.npz'); V=d['v']; F=d['f']; L=d['lab']
C=V[F].mean(1)
ledL=C[L==5]; ledL=ledL[ledL[:,1]>0].mean(0); ledR=C[L==5]; ledR=ledR[ledR[:,1]<0].mean(0)
pf=G['pod_frame']; o=np.array(pf['o']); R=np.array(pf['right']); U=np.array(pf['up']); dv=np.array(pf['dv'])
W=G['window']; win=o+R*(W['u0']+W['u1'])/2+U*(W['v0']+W['v1'])/2+dv*0.004
WG=json.load(open('wheel_geom.json')); rig=json.load(open(OUT+'/rig.json'))
wc=np.array(WG['R']['center']); wa=np.array(WG['R']['axle'])
cc_g=rig['R']['caster_center']; cc=np.array([cc_g[0],-cc_g[2],cc_g[1]])
cx=(dg['x0']+dg['x1'])/2; cy=(dg['y0']+dg['y1'])/2
cax=np.array(rig['R']['caster_axis']); cax=np.array([cax[0],-cax[2],cax[1]])   # glTF -> world
H=[
 dict(id='stop',part='middle',pos=g([sx,sy,htop(sx,sy)+0.002]),normal=g([0,0,1]),title='STOP 停止键',
      text='控制面板最前端的橙色按键，表面压印“STOP”字样，用于让割草机停止工作。',
      view=dict(cam=g([-0.42,-0.16,0.66]),target=g([-0.02,0,0.25]))),
 dict(id='display',part='middle',pos=g([cx,cy,htop(cx,cy)+0.002]),normal=g([0,0,1]),title='显示屏与按键',
      text='深色透明面板下的单色液晶屏；下方一排五个按键，从左到右依次为：电源、START（下方印 ▲）、OK、房子图标（下方印 ▼）、信号图标。',
      view=dict(cam=g([-0.46,-0.12,0.6]),target=g([-0.06,0,0.25]))),
 dict(id='sensor',part='middle',pos=g(win),normal=g(dv),title='前部传感器集群',
      text='车头凸起的浅灰色传感器罩。黑色玻璃窗内（面对车头，从左到右）依次为：小方形传感器、带镀膜的小镜头、较大的摄像头镜头。',
      view=dict(cam=g([0.86,-0.2,0.34]),target=g([0.29,0,0.21]))),
 dict(id='ledL',part='middle',pos=g(ledL+[0.003,0,0]),normal=g([1,0.3,0]),lift=0.03,title='LED 灯带（左）',
      text='前脸左侧的透明灯罩，内部为 LED 灯带。',view=dict(cam=g([0.9,0.35,0.3]),target=g([0.28,0.06,0.16]))),
 dict(id='ledR',part='middle',pos=g(ledR+[0.003,0,0]),normal=g([1,-0.3,0]),lift=0.03,title='LED 灯带（右）',
      text='前脸右侧的透明灯罩，内部为 LED 灯带。',view=dict(cam=g([0.9,-0.35,0.3]),target=g([0.28,-0.06,0.16]))),
 dict(id='slot',part='middle',pos=g([0.309,-0.0075,0.147]),normal=g([1,0,0]),lift=0.03,title='前保险杠开口',
      text='前保险杠中部的长方形开口，内部可见左右两组模块。',view=dict(cam=g([0.9,-0.12,0.24]),target=g([0.3,0,0.15]))),
 dict(id='rtk',part='middle',pos=g([-0.1913,0.0029,htop(-0.1913,0.0029)+0.002]),normal=g([0,0,1]),title='RTK 天线罩',
      text='控制面板后部的圆形凸罩，表面压印“RTK”字样，是 RTK 定位天线所在的位置。',
      view=dict(cam=g([-0.62,-0.25,0.62]),target=g([-0.17,0,0.24]))),
 dict(id='wheel',part='base',attach='Wheel_R',pos=g(wc+wa*0.026),normal=g(wa),liftDir=g([0.35,-0.25,1.0]),lift=0.16,title='驱动轮',
      text='大直径凸齿花纹驱动轮，轮毂外缘有一圈青色饰环。在轮子上按住拖动即可转动它。',
      view=dict(cam=g([0.35,-0.95,0.3]),target=g([0.12,-0.2,0.115]))),
 dict(id='caster',part='base',attach='Caster_R',focus='xray',pos=g(cc+cax*0.03),normal=g(cax),liftDir=g([-1.0,-0.35,0.2]),lift=0.09,title='万向轮',
      text='车尾底部的两个万向轮，可绕竖直轴 360° 转向。打开本说明时外壳会自动变为半透明；按住万向轮拖动即可调整朝向，“行驶演示”中它会自动转到跟随方向。',
      view=dict(cam=g([-0.28,-0.78,0.075]),target=g([-0.21,-0.1,0.055]))),
]
json.dump(H,open(OUT+'/hotspots.json','w'),ensure_ascii=False,indent=1)
print(json.dumps([(h['id'],h['pos']) for h in H]))
