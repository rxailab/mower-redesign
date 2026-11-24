# [fix4-qa] Solid audit of every part of a design npz (+ fixed hardware, informative only): for each connected component
#   closed      0 boundary and 0 non-manifold edges (after welding coincident vertices on a 0.1 um grid)
#   oriented    0 unpaired directed edges (consistent winding) and signed volume > 0 (normals point out of the material)
#   self-x      triangle pairs that cross inside the component (faces sharing a vertex excluded; BVH in 20 mm x slabs).
#               Components above --sx-max faces are tested in --sx-windows random 20 mm cubes ("sampled").
#   degenerate  zero-area / collapsed faces (reported, not a failure)
# Parts: see qa_parts.py (the lid moulding = lab 4 skin + kitm underside + edge, audited as ONE composite solid).
# usage (inside design/):  ../venv/bin/python solidaudit.py NPZ [--json OUT.json] [--base dev/base_parts.npz] [--params P.json]
#                          [--no-hw] [--no-selfx] [--sx-max 650000] [--sx-windows 48] [--only 'glob,glob']
#   (re-executes itself in Blender; or: blender -b -P solidaudit.py -- NPZ ...)
# exit 0 = every design part closed, oriented, no self-intersection; 1 = at least one design part fails (hardware never fails).
# runtime: web npz 35 s, full npz 70 s / 1.9 GB.
import sys, os, json, time, fnmatch
HERE_ = os.path.dirname(os.path.abspath(sys.argv[sys.argv.index('-P') + 1] if '-P' in sys.argv else __file__)); sys.path.insert(0, HERE_)
import qa_parts as QP
QP.ensure_blender(os.path.join(HERE_, 'solidaudit.py'))
import numpy as np
from mathutils.bvhtree import BVHTree

a = QP.script_args(); NPZ = a[0]
def opt(k, dv=None):
    return a[a.index(k) + 1] if k in a else dv
OUT = opt('--json'); BASE = opt('--base', os.path.join(QP.HERE, 'dev', 'base_parts.npz')); PARAMS = opt('--params')
SXMAX = int(opt('--sx-max', 650000)); SXWIN = int(opt('--sx-windows', 48)); ONLY = [p for p in (opt('--only') or '').split(',') if p]
os.chdir(QP.HERE); t0 = time.time()
d, parts, info = QP.build(NPZ, BASE, PARAMS, hardware='--no-hw' not in a)
if ONLY: parts = [p for p in parts if any(fnmatch.fnmatch(p.group, g) or fnmatch.fnmatch(p.name, g) for g in ONLY)]
print('parts %d, built %.1f s' % (len(parts), time.time() - t0), flush=True)


def selfx(V, F, slab=0.02, box=None):
    """non-adjacent crossing triangle pairs (indices into F) of one component; box = (lo, hi) restricts the test to a window"""
    T = V[F]; x = T[:, :, 0]; lo, hi = x.min(1), x.max(1); sel = np.ones(len(F), bool)
    if box is not None: sel = ((T.max(1) >= box[0]) & (T.min(1) <= box[1])).all(1)
    found = set()
    for xs in np.arange(lo[sel].min(), hi[sel].max() + 1e-9, slab) if sel.any() else []:
        m = np.nonzero(sel & (hi >= xs) & (lo <= xs + slab))[0]
        if len(m) < 2: continue
        tr = BVHTree.FromPolygons(T[m].reshape(-1, 3).tolist(), np.arange(len(m) * 3).reshape(-1, 3).tolist(), all_triangles=True, epsilon=0.0)
        pr = np.array(tr.overlap(tr), np.int64).reshape(-1, 2)
        if not len(pr): continue
        pr = pr[pr[:, 0] < pr[:, 1]]; ga, gb = m[pr[:, 0]], m[pr[:, 1]]
        share = (F[ga][:, :, None] == F[gb][:, None, :]).any((1, 2)); ga, gb = ga[~share], gb[~share]
        found.update(zip(ga.tolist(), gb.tolist()))
    return np.array(sorted(found), np.int64).reshape(-1, 2)


rng = np.random.default_rng(11); R = {}; t1 = time.time()
for p in parts:
    st = QP.edge_stats(p.F, p.V); st['frame'] = p.frame; st['center_mm'] = (p.c * 1000).round(2).tolist()
    st['oriented'] = st['unpaired_dir'] == 0 and st['dup_dir'] == 0; st['inverted'] = st['vol_mm3'] < 0
    if '--no-selfx' not in a:
        if len(p.F) <= SXMAX: P_ = selfx(p.V, p.F); st['selfx_mode'] = 'full'
        else:
            st['selfx_mode'] = 'sampled %d windows of 20 mm' % SXWIN; Pl = []
            for c in p.V[rng.choice(len(p.V), SXWIN, replace=False)]: Pl.append(selfx(p.V, p.F, box=(c - 0.01, c + 0.01)))
            P_ = np.unique(np.concatenate(Pl), axis=0) if Pl else np.zeros((0, 2), np.int64)
        st['selfx_pairs'] = int(len(P_))
        if len(P_):
            cm = (p.V[p.F[P_[:, 0]]].mean(1) + p.V[p.F[P_[:, 1]]].mean(1)) / 2 * 1000; k = np.round(cm / 5).astype(np.int64)
            _, iv, cnt = np.unique(k, axis=0, return_inverse=True, return_counts=True); iv = iv.ravel()
            st['selfx_clusters'] = [dict(n=int(cnt[q]), at_mm=cm[iv == q].mean(0).round(2).tolist()) for q in np.argsort(-cnt)[:6]]
    st['hw'] = p.hw or p.render
    st['ok'] = bool(st['closed'] and st['oriented'] and not st['inverted'] and not st.get('selfx_pairs'))
    R[p.name] = st
print('audited %.1f s' % (time.time() - t1), flush=True)

# table: one line per failing component (design parts first), then a per-group summary
fails = [n for n, s in R.items() if not s['ok'] and not s['hw']]; hwf = [n for n, s in R.items() if not s['ok'] and s['hw']]
hdr = '%-24s %8s %7s %6s %6s %6s %11s %7s %6s  %s' % ('component', 'faces', 'bound', 'nonman', 'unpair', 'degen', 'vol mm3', 'selfx', 'frame', 'center mm / self-x at')
print('\n=== SOLID AUDIT %s: %d components, %d design parts failing, %d hardware / render stand-ins not clean (info)' % (os.path.basename(NPZ), len(R), len(fails), len(hwf)))
print(hdr)
for n in fails + hwf:
    s = R[n]; why = []
    if not s['closed']: why.append('OPEN')
    if s['inverted']: why.append('INVERTED')
    if not s['oriented']: why.append('WINDING')
    if s.get('selfx_pairs'): why.append('SELF-X')
    at = s.get('selfx_clusters', [{}])[0].get('at_mm') if s.get('selfx_pairs') else s['center_mm']
    print('%-24s %8d %7d %6d %6d %6d %11.1f %7s %6s  %s %s%s' % (n[:24], s['faces'], s['boundary'], s['nonmanifold'], s['unpaired_dir'], s['degenerate'] + s['zero_area'],
          s['vol_mm3'], s.get('selfx_pairs', '-'), s['frame'], at, '+'.join(why), '' if not s['hw'] else ' (hw/render, info)'))
grp = {}
for n, s in R.items():
    g = n.split('#')[0]; G = grp.setdefault(g, dict(n=0, faces=0, open=0, inverted=0, winding=0, selfx=0, degenerate=0))
    G['n'] += 1; G['faces'] += s['faces']; G['open'] += not s['closed']; G['inverted'] += s['inverted']; G['winding'] += not s['oriented']
    G['selfx'] += bool(s.get('selfx_pairs')); G['degenerate'] += s['degenerate'] + s['zero_area']
print('\n%-22s %5s %9s %5s %5s %5s %5s %7s' % ('group', 'comps', 'faces', 'open', 'inv', 'wind', 'selfx', 'degen'))
for g in sorted(grp):
    G = grp[g]; print('%-22s %5d %9d %5d %5d %5d %5d %7d' % (g, G['n'], G['faces'], G['open'], G['inverted'], G['winding'], G['selfx'], G['degenerate']))
rep = dict(info=info, time_s=round(time.time() - t0, 1), failing=fails, hardware_not_clean=hwf, groups=grp, components=R)
if OUT: json.dump(rep, open(OUT, 'w'), indent=1); print('wrote', OUT)
print('SOLIDAUDIT %s: %d design components fail (%.0f s)' % ('OK' if not fails else 'FAIL', len(fails), time.time() - t0), flush=True)
sys.exit(1 if fails else 0)
