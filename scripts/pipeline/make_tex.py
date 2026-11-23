import numpy as np,json,math
from PIL import Image,ImageDraw,ImageFilter,ImageFont
G=json.load(open('mid_tex_geom.json'))
FB='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def lin_grad(w,h,top,bot):
    a=np.linspace(0,1,h)[:,None,None]; img=(np.array(top)[None,None,:]*(1-a)+np.array(bot)[None,None,:]*a)
    return np.repeat(img,w,1)
# ---------------- display ----------------
d=G['display']; x0,x1,y0,y1=d['x0'],d['x1'],d['y0'],d['y1']
Iw=2048; Ih=int(round(Iw*(x1-x0)/(y1-y0)))
pxm=Iw/(y1-y0)                      # pixels per metre
col=lambda y:(y1-y)/(y1-y0)*Iw; row=lambda x:(x1-x)/(x1-x0)*Ih
bg=lin_grad(Iw,Ih,(30,34,39),(20,23,27))
img=Image.fromarray(bg.astype(np.uint8)).convert('RGBA'); dr=ImageDraw.Draw(img)
# glass edge highlight
r=int(G['display_r']*pxm)
dr.rounded_rectangle((10,10,Iw-11,Ih-11),radius=r-10,outline=(70,76,84,255),width=6)
# LCD
l=G['lcd']; L=(col(l['y1']),row(l['x1']),col(l['y0']),row(l['x0']))
lcd=lin_grad(int(L[2]-L[0]),int(L[3]-L[1]),(172,178,176),(150,156,154)).astype(np.uint8)
lim=Image.fromarray(lcd).convert('RGBA')
# subtle inner shadow
sh=Image.new('L',lim.size,0); ImageDraw.Draw(sh).rectangle((0,0,lim.size[0]-1,lim.size[1]-1),outline=255,width=26); sh=sh.filter(ImageFilter.GaussianBlur(18))
lim=Image.composite(Image.new('RGBA',lim.size,(110,116,114,255)),lim,sh.point(lambda v:int(v*0.55)))
img.alpha_composite(lim,(int(L[0]),int(L[1])))
dr=ImageDraw.Draw(img)
dr.rectangle(L,outline=(18,20,22,255),width=8)
# buttons
bw,bh=int(0.0122*pxm),int(0.0102*pxm); br=int(0.0028*pxm)
fbig=ImageFont.truetype(FB,int(bh*0.30)); fok=ImageFont.truetype(FB,int(bh*0.42))
W=(236,240,236,255)
labels=['power','START','OK','home','signal']
dots=sorted(G['dots'],key=lambda p:-p[1])     # +y first = viewer's left
for k,(bx,by) in enumerate(dots):
    cx,cy=col(by),row(bx)
    box=(cx-bw/2,cy-bh/2,cx+bw/2,cy+bh/2)
    # soft drop shadow
    shadow=Image.new('RGBA',img.size,(0,0,0,0)); ImageDraw.Draw(shadow).rounded_rectangle((box[0]+6,box[1]+14,box[2]+6,box[3]+14),radius=br,fill=(0,0,0,150))
    img.alpha_composite(shadow.filter(ImageFilter.GaussianBlur(10)))
    dr=ImageDraw.Draw(img)
    btn=lin_grad(int(bw),int(bh),(66,72,66),(36,40,37)).astype(np.uint8)
    bim=Image.fromarray(btn).convert('RGBA'); m=Image.new('L',bim.size,0); ImageDraw.Draw(m).rounded_rectangle((0,0,bim.size[0]-1,bim.size[1]-1),radius=br,fill=255)
    img.paste(bim,(int(box[0]),int(box[1])),m); dr=ImageDraw.Draw(img)
    dr.rounded_rectangle(box,radius=br,outline=(88,95,88,255),width=4)
    s=bh*0.24; lw=max(6,int(bh*0.06))
    t=labels[k]
    if t=='power':
        dr.arc((cx-s,cy-s,cx+s,cy+s),start=-55,end=235,fill=W,width=lw)
        dr.line((cx,cy-s*1.15,cx,cy-s*0.05),fill=W,width=lw)
    elif t=='home':
        dr.line([(cx-s*1.05,cy-s*0.05),(cx,cy-s*1.05),(cx+s*1.05,cy-s*0.05)],fill=W,width=lw,joint='curve')
        dr.line([(cx-s*0.75,cy-s*0.3),(cx-s*0.75,cy+s*0.95),(cx+s*0.75,cy+s*0.95),(cx+s*0.75,cy-s*0.3)],fill=W,width=lw,joint='curve')
        dr.rectangle((cx-s*0.22,cy+s*0.25,cx+s*0.22,cy+s*0.95),fill=W)
    elif t=='signal':
        for i in range(4):
            hgt=s*(0.45+0.42*i); xx=cx-s*1.1+i*s*0.42
            dr.rectangle((xx,cy+s*0.95-hgt,xx+s*0.26,cy+s*0.95),fill=W)
        for i,rr in enumerate([s*0.9,s*1.35]):
            dr.arc((cx+s*0.55-rr,cy+s*0.1-rr,cx+s*0.55+rr,cy+s*0.1+rr),start=-50,end=50,fill=W,width=int(lw*0.8))
    else:
        f=fbig if t=='START' else fok
        dr.text((cx,cy),t,fill=W,font=f,anchor='mm')
    if t in ('START','home'):
        ty=cy+bh/2+bh*0.42; ts=bh*0.18
        if t=='START': pts=[(cx,ty-ts),(cx-ts*1.1,ty+ts*0.8),(cx+ts*1.1,ty+ts*0.8)]
        else: pts=[(cx,ty+ts),(cx-ts*1.1,ty-ts*0.8),(cx+ts*1.1,ty-ts*0.8)]
        dr.polygon(pts,fill=W)
# faint glass reflection streak
refl=Image.new('L',img.size,0); ImageDraw.Draw(refl).polygon([(Iw*0.05,0),(Iw*0.30,0),(Iw*0.12,Ih),(-Iw*0.13,Ih)],fill=28)
img=Image.composite(Image.new('RGBA',img.size,(255,255,255,255)),img,refl.filter(ImageFilter.GaussianBlur(60)))
img.convert('RGB').save('tex/display.png'); print("display tex",img.size)
# ---------------- sensor window ----------------
w=G['window']; u0,u1,v0,v1=w['u0'],w['u1'],w['v0'],w['v1']
Jw=2048; Jh=int(round(Jw*(v1-v0)/(u1-u0))); pm=Jw/(u1-u0)
cu=lambda u:(u-u0)/(u1-u0)*Jw; rv=lambda v:(v1-v)/(v1-v0)*Jh
bg=lin_grad(Jw,Jh,(14,16,20),(6,7,9))
im=Image.fromarray(bg.astype(np.uint8)).convert('RGBA'); dr=ImageDraw.Draw(im)
vc=rv(0.0040)
# ToF module
cx=cu(-0.0178); hw,hh=0.0023*pm,0.0028*pm
dr.rounded_rectangle((cx-hw,vc-hh,cx+hw,vc+hh),radius=int(0.0005*pm),fill=(92,94,96,255),outline=(120,122,124,255),width=5)
s=0.0007*pm; dr.rectangle((cx-s,vc-s,cx+s,vc+s),fill=(215,212,205,255),outline=(60,58,55,255),width=4)
# small lens (iridescent)
cx=cu(-0.0052)
def ring(cx,cy,rr,fill=None,outline=None,width=1):
    dr.ellipse((cx-rr,cy-rr,cx+rr,cy+rr),fill=fill,outline=outline,width=width)
ring(cx,vc,0.0042*pm,fill=(40,40,46,255))
ring(cx,vc,0.0036*pm,fill=(150,150,165,255))
for i,col_ in enumerate([(120,110,190),(90,160,210),(170,120,200),(60,60,90)]):
    ring(cx,vc,(0.0032-i*0.0005)*pm,fill=col_+(255,))
ring(cx,vc,0.0011*pm,fill=(20,24,30,255)); ring(cx,vc,0.0004*pm,fill=(120,200,170,255))
# big lens
cx=cu(0.0080)
ring(cx,vc,0.0066*pm,fill=(32,32,35,255)); ring(cx,vc,0.0060*pm,fill=(70,70,74,255))
ring(cx,vc,0.0052*pm,fill=(18,18,20,255)); ring(cx,vc,0.0034*pm,fill=(10,10,12,255),outline=(55,55,60,255),width=8)
ring(cx,vc,0.0012*pm,fill=(40,18,22,255)); ring(cx,vc,0.0005*pm,fill=(120,50,60,255))
# glass sheen
refl=Image.new('L',im.size,0); ImageDraw.Draw(refl).polygon([(Jw*0.0,0),(Jw*0.22,0),(Jw*0.05,Jh),(-Jw*0.17,Jh)],fill=26)
im=Image.composite(Image.new('RGBA',im.size,(255,255,255,255)),im,refl.filter(ImageFilter.GaussianBlur(40)))
ImageDraw.Draw(im).rounded_rectangle((4,4,Jw-5,Jh-5),radius=int(0.0012*pm),outline=(40,44,50,255),width=8)
im.convert('RGB').save('tex/window.png'); print("window tex",im.size)
