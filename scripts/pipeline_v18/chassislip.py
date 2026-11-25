"""[v20] Clean stand-in for the chassis's finned rear apron (the 'diffuser' under the raised tail hem).

The scanned apron is lumpy; renders and the web viewer drop its faces and show this solid instead: the same outline (envelope r(theta)
about c, measured on the scan and mirrored), fins / channels at the measured pitch (one on the centre line), a rolled lower edge, a ledge
back to the tub wall on top, a flat underside and end caps.  Spec: params 'chassis_lip'.  Pure numpy (used by design_render.py inside
Blender and by export_web.py).
"""
import math
import numpy as np


def _sm(u):
    u = np.clip(u, 0, 1); return u * u * (3 - 2 * u)


def funcs(spec):
    ak = np.radians(np.array(spec['ang_deg'], float)); rk = np.array(spec['r'], float) + spec.get('proud', 0.0008); wk = np.array(spec['r_wall'], float)
    return (lambda a: np.interp(a, ak, rk)), (lambda a: np.interp(a, ak, wk))


def drop_mask(C, spec):
    """Faces of the scanned chassis to drop: centroid C (n, 3) in the world frame (+X forward, +Y left, Z up) inside the apron band."""
    cx, cy = spec['c']; zb, zt = spec['z']; amax = math.radians(spec.get('ang_max', 80.0)); _, wf = funcs(spec)
    a = np.arctan2(C[:, 1] - cy, -(C[:, 0] - cx)); r = np.hypot(C[:, 0] - cx, C[:, 1] - cy)
    return (C[:, 2] > zb - 0.004) & (C[:, 2] < zt + 0.0008) & (np.abs(a) <= amax + 0.03) & (r > wf(a) + 0.0012)


def build(spec, na=1601, nz=71):
    """-> V (n, 3), F (m, 3) triangles, outward-facing, world frame."""
    cx, cy = spec['c']; zb, zt = spec['z']; amax = math.radians(spec.get('ang_max', 80.0)); g = spec['groove']; pit = spec['pitch']
    rf, wf = funcs(spec)
    A = np.linspace(-amax, amax, na); R = rf(A); W = wf(A) - 0.002
    P2 = np.c_[cx - R * np.cos(A), cy + R * np.sin(A)]; S = np.r_[0, np.cumsum(np.hypot(*np.diff(P2, axis=0).T))]; S = S - np.interp(0.0, A, S)
    ds = (S + 0.5 * pit) % pit - 0.5 * pit                                        # to the nearest channel (one on the centre line)
    gs = _sm((g['w'] / 2 + 0.0008 - np.abs(ds)) / 0.0016)
    Z = np.linspace(zb, zt, nz); rows = []
    for z in Z:
        gz = _sm(min((z - g['z'][0]) / 0.004, (g['z'][1] - z) / 0.004))
        rr = R - g['d'] * gs * gz - 0.004 * (1 - np.clip((z - zb) / 0.005, 0, 1)) ** 2      # channels; rolled lower edge
        rows.append(np.c_[cx - rr * np.cos(A), cy + rr * np.sin(A), np.full(na, z)])
    for z in (zt, zb): rows.append(np.c_[cx - W * np.cos(A), cy + W * np.sin(A), np.full(na, z)])
    V = np.concatenate(rows); it, ib = nz * na, (nz + 1) * na; Q = []
    I = np.arange(nz * na).reshape(nz, na)
    Q.append(np.c_[I[:-1, :-1].ravel(), I[:-1, 1:].ravel(), I[1:, 1:].ravel(), I[1:, :-1].ravel()])          # outer finned face
    j = np.arange(na - 1); o = (nz - 1) * na + j
    Q.append(np.c_[o, it + j, it + j + 1, o + 1])                                  # top ledge back to the tub wall
    Q.append(np.c_[j, j + 1, ib + j + 1, ib + j])                                  # flat underside
    Q.append(np.c_[it + j, ib + j, ib + j + 1, it + j + 1])                        # inner face (hidden, closes the solid)
    Q = np.concatenate(Q); F = np.r_[Q[:, [0, 1, 2]], Q[:, [0, 2, 3]]]
    for jj, flip in ((0, False), (na - 1, True)):                                  # end caps: fan over the outer column + the two inner points
        ring = [i * na + jj for i in range(nz)] + [it + jj, ib + jj]
        tri = np.array([[ring[0], ring[k], ring[k + 1]] for k in range(1, len(ring) - 1)])
        F = np.r_[F, tri[:, ::-1] if flip else tri]
    # orient every face outward: away from the band's mid surface (radius (R+W)/2 at the face's angle, mid height)
    Cn = V[F].mean(1); n = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    a = np.arctan2(Cn[:, 1] - cy, -(Cn[:, 0] - cx)); rm = 0.5 * (rf(a) + wf(a)); zm = 0.5 * (zb + zt)
    ctr = np.c_[cx - rm * np.cos(a), cy + rm * np.sin(a), np.full(len(F), zm)]
    flip = np.sum(n * (Cn - ctr), 1) < 0; F[flip] = F[flip][:, ::-1]
    return V, F
