# Exact lid-to-controls clearance (no distance cut-off) at 5 deg steps. usage: python lidsim_ctl.py DESIGN.npz
import sys, json, numpy as np
sys.argv = [sys.argv[0], sys.argv[1]]
import lidsim as S
S.FIX = ['controls']; S.FAR = 1.0; S.ANG = np.r_[S.OVER, np.arange(0, S.OPEN + 0.01, 5.0)]
from multiprocessing import Pool
if __name__ == '__main__':
    ys = S.ys[::2]
    with Pool(14) as pool: RR = pool.map(S.work, ys)
    best = {}
    for y, out in RR:
        for (a, b, th), (dd, hit) in out.items():
            if th not in best or dd < best[th][0]: best[th] = (dd, a, y, hit)
    print(json.dumps({'%g' % t: [round(v[0] * 1000, 2), v[1], round(v[2] * 1000, 1), v[3]] for t, v in sorted(best.items())}))
