# Slim tail: narrows the wide rear of the lower skirt (behind the drive wheels) and, with the same mapping, the belt outline
# of the new body, so both still meet. P['tail'] = {"width": [[x, half_width], ...], "y0": 0.165}; without it nothing changes.
# Only numpy (also imported by Blender's Python).
import numpy as np, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
def _w0():
    c = os.path.join(HERE, 'tail_w0.json')
    if os.path.exists(c): return json.load(open(c))
    V = np.load(os.path.join(HERE, 'keep.npz'))['v']; xs = np.arange(-0.335, -0.02, 0.005); w = []
    for x in xs:
        m = np.abs(V[:, 0] - x) < 0.004; w.append(float(np.abs(V[m, 1]).max()) if m.any() else 0.0)
    w = np.maximum.accumulate(np.array(w))            # widest so far from the tail forwards: one smooth envelope
    d = {'xs': xs.tolist(), 'w': w.tolist()}; json.dump(d, open(c, 'w')); return d
def make(P):
    t = P.get('tail')
    if not t: return lambda x, y: y
    W = _w0(); pts = np.array(sorted(t['width'])); y0 = t.get('y0', 0.165); x1 = t.get('x_front', -0.028)
    def f(x, y):
        x = np.asarray(x, float); y = np.asarray(y, float)
        w0 = np.interp(x, W['xs'], W['w']); wt = np.interp(x, pts[:, 0], pts[:, 1])
        k = np.where(w0 > wt, (wt - y0) / np.maximum(w0 - y0, 1e-6), 1.0); k = np.clip(k, 0.05, 1.0)
        a = np.abs(y); out = np.where((a > y0) & (x < x1), np.sign(y) * (y0 + (a - y0) * k), y)
        return out
    return f
