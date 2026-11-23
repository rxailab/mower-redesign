# Fastening points of the chassis, measured from its mesh -> mounts.json (read by design_gen.py).
#   posts : screw posts round the tub rim (9 mm cylinders, hole open at the top)        -> screws from above, through wells in the shell
#   under : through holes in the front wings / rear corner sockets                       -> screws from below into bosses of the shell
#   clips : windows in the rear apron                                                     -> snap hooks on the inside of the tail skirt
import numpy as np, json, sys
C = np.load(sys.argv[1]); N = np.load(sys.argv[2])
def fit(p):
    A = np.c_[2 * p[:, 0], 2 * p[:, 1], np.ones(len(p))]; b = (p[:, :2] ** 2).sum(1); (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None); return cx, cy, np.sqrt(c + cx * cx + cy * cy)
def post(x, y, wall=True):
    for it in range(3):
        m = np.hypot(C[:, 0] - x, C[:, 1] - y) < 0.0065; c = C[m]; n = N[m]; r = np.hypot(c[:, 0] - x, c[:, 1] - y)
        top = (n[:, 2] > 0.9) & (r < 0.004); zt = np.percentile(c[top, 2], 90)
        if wall:
            rad = ((c[:, 0] - x) * n[:, 0] + (c[:, 1] - y) * n[:, 1]) / np.maximum(r, 1e-9)
            w = (np.abs(n[:, 2]) < 0.25) & (r > 0.0035) & (r < 0.0056) & (rad > 0.7) & (c[:, 2] > zt - 0.012) & (c[:, 2] < zt - 0.001)
            if w.sum() > 200: x, y, rr = fit(c[w])
            else: wall = False
        if not wall:
            t2 = (n[:, 2] > 0.9) & (np.abs(c[:, 2] - zt) < 0.0008) & (r < 0.0048); x, y = c[t2, 0].mean(), c[t2, 1].mean(); rr = float(np.percentile(np.hypot(c[t2, 0] - x, c[t2, 1] - y), 97))
    return [round(float(x), 4), round(float(y), 4), round(float(zt), 4), round(float(rr) * 1000, 2)]
def hole(x, y, rmin, rmax, zr):
    for it in range(3):
        m = (np.hypot(C[:, 0] - x, C[:, 1] - y) < rmax + 0.001) & (C[:, 2] > zr[0]) & (C[:, 2] < zr[1]); c = C[m]; n = N[m]; r = np.hypot(c[:, 0] - x, c[:, 1] - y)
        rad = ((c[:, 0] - x) * n[:, 0] + (c[:, 1] - y) * n[:, 1]) / np.maximum(r, 1e-9); w = (np.abs(n[:, 2]) < 0.25) & (r > rmin) & (r < rmax) & (rad < -0.7)
        x, y, rr = fit(c[w])
    return float(x), float(y), float(rr)
out = {'posts': [], 'under': [], 'clips': []}
for x, y in [(-0.2910, -0.0025), (-0.2620, 0.1200), (-0.2620, -0.1220)]: out['posts'].append(post(x, y, wall=False))
for x, y in [(-0.1521, 0.1491), (-0.1499, -0.1519), (-0.0423, 0.1501), (-0.0403, -0.1509), (0.0575, 0.1601), (0.0597, -0.1597), (0.1771, 0.1631), (0.1795, -0.1609),
             (0.2577, 0.1071), (0.2591, -0.1037), (0.2893, 0.0019)]: out['posts'].append(post(x, y))
for (x, y), zr in [((0.2472, 0.1408), (0.1545, 0.1585)), ((0.2490, -0.1378), (0.1545, 0.1585)), ((0.2777, 0.0716), (0.1545, 0.1585))]:
    cx, cy, rr = hole(x, y, 0.0035, 0.006, zr); m = (np.hypot(C[:, 0] - cx, C[:, 1] - cy) < 0.007) & (N[:, 2] > 0.9) & (C[:, 2] > 0.158) & (C[:, 2] < 0.165)
    out['under'].append({'x': round(cx, 4), 'y': round(cy, 4), 'z': round(float(np.percentile(C[m, 2], 80)), 4), 'kind': 'front', 'r_cbore': round(rr * 1000, 2)})
for (x, y) in [(-0.2660, 0.1723), (-0.2630, -0.1759)]:
    cx, cy, rr = hole(x, y, 0.004, 0.0065, (0.080, 0.110)); m = (np.hypot(C[:, 0] - cx, C[:, 1] - cy) < 0.0045) & (N[:, 2] > 0.9) & (C[:, 2] > 0.07) & (C[:, 2] < 0.082)
    mz = (np.hypot(C[:, 0] - cx, C[:, 1] - cy) < 0.008) & (C[:, 2] > 0.08) & (C[:, 2] < 0.13) & (np.abs(N[:, 2]) < 0.25)
    out['under'].append({'x': round(cx, 4), 'y': round(cy, 4), 'z': round(float(np.percentile(C[m, 2], 80)), 4), 'kind': 'rear', 'r_socket': round(rr * 1000, 2), 'z_socket_top': round(float(np.percentile(C[mz, 2], 99)), 4)})
# clip windows: gaps in the rear-facing wall of the apron near z 0.100 .. 0.105
rear = (N[:, 0] < -0.3) & (C[:, 0] < -0.28)
for y0 in (0.113, 0.0525, -0.002, -0.057, -0.117):
    best = None
    for yy in np.arange(y0 - 0.008, y0 + 0.008, 0.0005):
        m = rear & (np.abs(C[:, 1] - yy) < 0.0006) & (C[:, 2] > 0.085) & (C[:, 2] < 0.12); z = np.sort(C[m, 2])
        if len(z) < 5: continue
        gp = np.diff(z); k = int(np.argmax(gp))
        if gp[k] > 0.003: best = (best or []) + [(yy, z[k], z[k + 1], C[m][C[m, 2] <= z[k] + 1e-9, 0].min(), C[m][C[m, 2] >= z[k + 1] - 1e-9, 0].min())]
    b = np.array(best); out['clips'].append({'y0': round(float(b[:, 0].min()), 4), 'y1': round(float(b[:, 0].max()), 4), 'z0': round(float(np.median(b[:, 1])), 4), 'z1': round(float(np.median(b[:, 2])), 4),
                                             'x_below': round(float(np.median(b[:, 3])), 4), 'x_above': round(float(np.median(b[:, 4])), 4)})
json.dump(out, open(sys.argv[3], 'w'), indent=1); print(json.dumps(out))
