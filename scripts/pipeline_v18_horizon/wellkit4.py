# [fix4-int] Closed solids for the lid-well kit parts that no round-4 branch owned (review G5 leftovers), used by lidmech when
# lid.closed_well_kit is true (fix4_int.json). Builds on prismkit.slab_union (fix4-hinge2): one closed, oriented 2-manifold per part.
import numpy as np, shapely
import prismkit


def boxes_solid(boxes, frame):
    """union of axis boxes given as (u0, u1, w0, w1, v0, v1) in a profile frame (u, v) extruded along w (lidmech.extrude's box tuples:
    shapely.box(u0, v0, u1, v1) from w0 to w1) -> ONE closed solid: the w range is cut at every box end and each slab gets the union of
    the boxes that span it (touching boxes no longer share coincident faces / edges)."""
    ws = sorted({round(float(b[k]), 12) for b in boxes for k in (2, 3)}); slabs = []
    for w0, w1 in zip(ws[:-1], ws[1:]):
        g = [shapely.box(b[0], b[4], b[1], b[5]) for b in boxes if b[2] <= w0 + 1e-12 and b[3] >= w1 - 1e-12]
        if g: slabs.append((prismkit.clean(shapely.union_all(g)), w0, w1))
    return prismkit.slab_union(slabs, frame)


def lip_solid(PC, NC, s, z, t):
    """the TPE skirt lip round the control panel as a closed ring solid (was a zero-thickness sheet: OPEN). Its section is the f3 polyline
    (s along NC = into the panel, z) per loop point - s: (3,), z: (n, 3) - thickened by t on its UPPER side (the lower side and the tip on
    the panel top stay where they were); the root rises up the floor's hole-edge wall (vertical end face, fused into the well)."""
    from meshkit import stack
    n = len(PC); S = np.broadcast_to(np.asarray(s, float), (n, 3)); Z = np.asarray(z, float)
    d1 = np.stack([S[:, 1] - S[:, 0], Z[:, 1] - Z[:, 0]], 1); d2 = np.stack([S[:, 2] - S[:, 1], Z[:, 2] - Z[:, 1]], 1)
    nrm = lambda d: np.stack([-d[:, 1], d[:, 0]], 1) / np.linalg.norm(d, axis=1, keepdims=True)      # upper side (d runs into the panel, ds > 0)
    n1, n2 = nrm(d1), nrm(d2); nb = n1 + n2; nb = nb / np.maximum((nb * n1).sum(1, keepdims=True), 1e-9)      # mitred at the bend
    up = [np.stack([S[:, 0], Z[:, 0] + t / np.maximum(n1[:, 1], 0.2)], 1), np.stack([S[:, 1], Z[:, 1]], 1) + t * nb, np.stack([S[:, 2], Z[:, 2]], 1) + t * n2]
    lo = [np.stack([S[:, k], Z[:, k]], 1) for k in range(3)]
    to3 = lambda q: np.c_[prismkit.defold(PC, PC + NC * q[:, :1]), q[:, 1]]      # defold: no crossing sections at the panel's tight corners
    R = [to3(q) for q in lo + up[::-1]] + [to3(lo[0])]
    return prismkit.weld_clean(*stack(R, closed=True))
