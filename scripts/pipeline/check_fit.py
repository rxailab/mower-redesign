# Does the tail of the body clear the chassis?  usage: python check_fit.py SHELL.npz CHASSIS_FACE_CENTRES.npy
# Horizontal slices; in each one the skin and the chassis are compared in polar coordinates about (-0.12, 0).
import numpy as np, sys
d = np.load(sys.argv[1]); V = d['v']; C = np.load(sys.argv[2]); C = C[(C[:, 0] < -0.02) & (C[:, 2] > 0.045)]
cx = -0.12; worst = []; nb = 180
for z0 in np.arange(0.05, 0.245, 0.005):
    s = V[(np.abs(V[:, 2] - z0) < 0.004) & (V[:, 0] < 0.0)]; c = C[np.abs(C[:, 2] - z0) < 0.0025]
    if len(s) < 50 or len(c) < 50: continue
    a_s = np.arctan2(s[:, 1], -(s[:, 0] - cx)); r_s = np.hypot(s[:, 0] - cx, s[:, 1]); a_c = np.arctan2(c[:, 1], -(c[:, 0] - cx)); r_c = np.hypot(c[:, 0] - cx, c[:, 1])
    bs = np.clip(((a_s + np.pi / 2) / np.pi * nb).astype(int), -1, nb); bc = np.clip(((a_c + np.pi / 2) / np.pi * nb).astype(int), -1, nb)
    ok_s = (bs >= 0) & (bs < nb); ok_c = (bc >= 0) & (bc < nb); Rs = np.full(nb, np.nan); Rc = np.zeros(nb)
    rmin = np.full(nb, 9.0); np.minimum.at(rmin, bs[ok_s], r_s[ok_s]); Rs = np.where(rmin < 8, rmin, np.nan)      # innermost skin point per direction (conservative)
    np.maximum.at(Rc, bc[ok_c], r_c[ok_c]); gap = Rs - Rc; gap[Rc == 0] = np.nan
    if np.isfinite(gap).any():
        k = int(np.nanargmin(gap)); ang = -90 + (k + 0.5) * 180 / nb; worst.append((z0, float(np.nanmin(gap)), ang))
for z0, gp, ang in worst: print('z %.3f  smallest gap skin - chassis %+6.1f mm  (direction %+.0f deg from straight back)' % (z0, gp * 1000, ang))
print('MIN GAP mm', round(min(w[1] for w in worst) * 1000, 1))
