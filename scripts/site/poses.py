# Pose tables for the product-site motion sequences (design_render.py POSEMAP).
# usage: python poses.py dock|slope N  -> three lines: CUSTOM_VIEWS, POSEMAP, view list
import sys, math
kind, N = sys.argv[1], int(sys.argv[2])
LIFT = 0.0173; XF, XR = 0.117, -0.200            # drive-wheel and caster contact x (mower frame)
def h(x):                                          # dock plate top (dock views: floor at -LIFT, plate top at 0); rear ramp 40 mm
    return -LIFT + LIFT * min(1, max(0, (x + 0.345) / 0.04))
views, poses, names = [], [], []
for i in range(N):
    t = i / (N - 1)
    if kind == 'dock':
        e = 1 - (1 - t) ** 2.2                       # slows down as it docks
        dx = -1.05 * (1 - e); zf, zr = h(XF + dx), h(XR + dx)
        p = math.degrees(math.atan2(zf - zr, XF - XR)); dz = zr - XR * math.sin(math.radians(p))
        n = f'dockin{i:02d}'; views.append(f'{n}=1.05:-1.80:0.95:-0.45:0.0:0.12:44'); poses.append(f'{n}={dx:.5f}:{dz:.5f}:{p:.4f}:{dx + 1.05:.5f}')
    else:
        th = 19.29; s = -0.60 + 1.20 * t; c, sn = math.cos(math.radians(th)), math.sin(math.radians(th))
        n = f'slope{i:02d}'; views.append(f'{n}=0.55:-3.15:0.80:0.0:0.0:0.02:50'); poses.append(f'{n}={s * c:.5f}:{s * sn:.5f}:{th:.3f}:{s + 0.60:.5f}')
    names.append(n)
print(';'.join(views)); print(','.join(poses)); print(','.join(names))
