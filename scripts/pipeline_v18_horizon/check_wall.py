# Do the hidden parts stay inside?  usage: python check_cover.py SHELL.npz [--json OUT.json]
# 1. (all versions) every vertex of the body's deck / screws (mount_*) must lie under the cover's OUTER skin, every vertex of the cover's legs
#    (ckit_*) inside the body's outer skin, the body's inner parts inside the body's outer skin.
# 2. (real wall: npz has `side`) clearances to the INNER skins (signed distance, exact point-to-triangle):
#    - body parts vs the cover's inner skin: nothing may enter the cover wall (deck, screws, pads ... need room under it);
#    - cover legs vs the body wall: legs must not cut into it;
#    - body parts vs the body wall: fused parts (boss, rib, bracket, clip, deck edge) start inside it, loose parts (screw, pad, pilot, slot) must not;
#    - lid-well kit vs the cover's inner skin.
#    Inner-skin normals point into the cavity, so d > 0 = in the cavity, d < 0 = inside the wall material (or beyond).
import numpy as np, sys, json
from scipy.spatial import cKDTree
d = np.load(sys.argv[1]); V = d['v'].astype(float); F = d['f']; part = d['part']; lab = d['lab']
side = d['side'] if 'side' in d.files else np.zeros(len(F), np.uint8); WALL = bool((side == 1).any()); out = {}
def side_(Fs, pts, tag):
    P = V[Fs]; n = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]); n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-20); c = P.mean(1)
    dist, i = cKDTree(c).query(pts); o = ((pts - c[i]) * n[i]).sum(1)          # > 0: on the outer side of the nearest face
    bad = (o > 0.0002) & (dist < 0.004); print('%s: %d of %d vertices outside the skin (worst %.2f mm)' % (tag, bad.sum(), len(pts), o[bad].max() * 1000 if bad.any() else 0.0))
    out[tag] = int(bad.sum()); return pts[bad]
outer = side == 0; lid = lab == 4
b1 = side_(F[(part == 1) & outer & ~lid], d['mount_v'][d['mount_v'][:, 2] > 0.15], 'body deck / screws vs cover')
b2 = side_(F[(part == 0) & outer & ~lid], d['ckit_v'], 'cover legs / flange vs body')
b3 = side_(F[(part == 0) & outer & ~lid], d['mount_v'], 'body inner parts vs body skin')
for b in (b1, b2, b3):
    if len(b): print('   e.g.', np.round(b[:4], 4).tolist())
if WALL:
    # each moulding is now a closed solid (outer + inner skin + rims, all normals pointing out of the material): signed distance to it
    # is < 0 inside the plastic, > 0 = air gap to it
    sys.path.insert(0, '.'); from shell_wall import SurfDist
    SOL = {pa: SurfDist(V, F[(part == pa) & ~lid]) for pa in (0, 1)}
    def per_vertex_kind(fk, Fk, nv):
        k = np.full(nv, -1); k[Fk.ravel()] = np.repeat(fk, 3); return k
    mv = d['mount_v']; mk = per_vertex_kind(d['mount_kind'], d['mount_f'], len(mv)) if 'mount_kind' in d.files else np.zeros(len(mv), int)
    MKN = [str(x) for x in d['mount_kinds']] if 'mount_kinds' in d.files else ['all']
    cv_ = d['ckit_v']; ck = per_vertex_kind(d['ckit_kind'], d['ckit_f'], len(cv_)) if 'ckit_kind' in d.files else np.zeros(len(cv_), int)
    CKN = [str(x) for x in d['ckit_kinds']] if 'ckit_kinds' in d.files else ['all']
    U = 0.010; TOL = 0.0001
    def stats(X, tag, own=None):
        r = {}
        for pa, nm in ((1, 'cover'), (0, 'body')):
            s_, _ = SOL[pa](X, U); m = np.isfinite(s_)
            if not m.any(): continue
            inn = m & (s_ < -TOL); gap = m & ~inn
            r[nm] = dict(inside=int(inn.sum()), depth_max_mm=round(float(-s_[inn].min()) * 1000, 2) if inn.any() else 0.0,
                         gap_min_mm=round(float(s_[gap].min()) * 1000, 2) if gap.any() else None, gap_under_0p3=int((gap & (s_ < 0.0003)).sum()))
            if pa != own and inn.any(): j = np.argsort(s_[inn])[:2]; r[nm]['worst_at_mm'] = (X[inn][j] * 1000).round(1).tolist()
        out[tag] = r; print('%-20s %s' % (tag, r)); return r
    print('--- hidden parts vs the walls (closed solids). inside = vertices inside the plastic (fused parts: expected in their own moulding),')
    print('    depth = deepest, gap_min = smallest air gap of the rest (mm)')
    for k, nm in enumerate(MKN):
        m = mk == k
        if m.any(): stats(mv[m], 'body ' + nm, own=0)
    for k, nm in enumerate(CKN):
        m = ck == k
        if m.any(): stats(cv_[m], 'cover ' + nm, own=1)
    stats(d['kitf_v'], 'lid well kit', own=1)
if '--json' in sys.argv: json.dump(out, open(sys.argv[sys.argv.index('--json') + 1], 'w'), indent=1)
