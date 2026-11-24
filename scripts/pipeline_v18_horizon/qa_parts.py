# [fix4-qa] Shared part model of the QA tools interfere.py and solidaudit.py (Blender Python: numpy 1.24 + mathutils / bmesh,
# no scipy / shapely). Loads a design npz (+ dev/base_parts.npz, keep.npz, control.npz), cuts the chassis exactly as the renderer
# does (strutskit.cut_bmesh with the npz caps), welds every mesh family, splits it into its connected components and gives every
# component a group name, a motion frame and the poses the renderer uses:
#   frame 'fixed' : body, cover, cover kit, body mount kit, port frame, nose floor, lid-well kit (kitf), lamps, harness, hardware
#   frame 'lid'   : lid moulding (lab 4 skin + kitm underside + kitm edge = ONE composite part 'lid'), the other kitm parts, the
#                   STOP parts that do not move with the cap (halo boss, bore, bracket, bumper) and the halo feed pipe
#   frame 'stop'  : STOP parts that move with the cap stroke (stopkit.MOVING) - lid rotation, then the renderer's STOPSTROKE motion
# World: metres, +X forward, +Y left, Z up.  All reported coordinates are mm.
import os, sys, re, math, json, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BLENDER = os.environ.get('BLENDER', os.path.expanduser('~/blender-4.2.3-linux-x64/blender'))


def ensure_blender(script):
    """Run under Blender: from the venv / system python re-exec the script in Blender (exit code passes through)."""
    try:
        import bpy  # noqa: F401
        return
    except ImportError:
        pass
    sys.stdout.flush()
    os.execv(BLENDER, [BLENDER, '-b', '--factory-startup', '--python-exit-code', '2', '-P', os.path.abspath(script), '--'] + sys.argv[1:])


def script_args():
    return sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else sys.argv[1:]


# ---------------------------------------------------------------------------------------------------------------- name maps
def _dict_src(path, var):
    """dict(a=1, b=2) literal of `var` in a module that Blender cannot import (shapely / scipy)."""
    s = open(os.path.join(HERE, path)).read(); m = re.search(r'^%s\s*=\s*dict\(([^)]*)\)' % var, s, re.M)
    return {k: int(v) for k, v in re.findall(r'(\w+)\s*=\s*(\d+)', m.group(1))} if m else {}


def name_maps():
    PM = _dict_src('lidmech.py', 'PM'); PM.update(_dict_src('hingekit.py', 'PM3'))
    PF = _dict_src('lidmech.py', 'PF'); PF.update(_dict_src('hingekit.py', 'PF3'))
    SP = _dict_src('stopkit.py', 'PART')
    m = re.search(r"^MOVING\s*=\s*\(([^)]*)\)", open(os.path.join(HERE, 'stopkit.py')).read(), re.M)
    SMOV = tuple(re.findall(r"'(\w+)'", m.group(1))) if m else ()
    return PM, PF, SP, SMOV


def latch_over():
    m = re.search(r'latch_over=([0-9.eE+-]+)', open(os.path.join(HERE, 'lidmech.py')).read())
    return float(m.group(1)) if m else 0.0015


# ---------------------------------------------------------------------------------------------------------------- mesh utils
def weld(V, F, tol=1e-7):
    """weld coincident vertices (0.1 um grid); returns V_w, F_w, first original index per welded vertex"""
    F = np.asarray(F, np.int64)
    u = np.unique(F); V = np.asarray(V, float)
    key = np.round(V[u] / tol).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    rm = -np.ones(len(V), np.int64); rm[u] = inv.ravel()
    return V[u][first], rm[F], u[first]


def components(nv, F):
    """connected components of faces over shared (welded) vertices: hook-and-compress union find (vectorised, numpy 1.24)"""
    E = np.concatenate([F[:, [0, 1]], F[:, [1, 2]]]); p = np.arange(nv)
    while len(E):
        a = p[E[:, 0]]; b = p[E[:, 1]]; m = a != b
        E, a, b = E[m], a[m], b[m]
        if not len(E): break
        lo = np.minimum(a, b); hi = np.maximum(a, b); o = np.lexsort((lo, hi)); hs, ls = hi[o], lo[o]
        fst = np.r_[True, hs[1:] != hs[:-1]]; p[hs[fst]] = np.minimum(p[hs[fst]], ls[fst])
        while True:
            pp = p[p]
            if np.array_equal(pp, p): break
            p = pp
    lab = p[F[:, 0]]; u, inv = np.unique(lab, return_inverse=True)
    return inv.ravel(), len(u)


def edge_stats(F, V=None):
    """closure / orientation of a welded triangle set: boundary, non-manifold, unpaired directed edges, degenerate faces, volume"""
    deg = (F[:, 0] == F[:, 1]) | (F[:, 1] == F[:, 2]) | (F[:, 0] == F[:, 2]); G = F[~deg]
    e = np.concatenate([G[:, [0, 1]], G[:, [1, 2]], G[:, [2, 0]]]); n = int(F.max()) + 1 if len(F) else 1
    es = np.sort(e, 1); ku = es[:, 0] * n + es[:, 1]; _, cnt = np.unique(ku, return_counts=True)
    dk = e[:, 0] * n + e[:, 1]; dks = np.sort(dk); rk = e[:, 1] * n + e[:, 0]
    pos = np.searchsorted(dks, rk); pos = np.minimum(pos, len(dks) - 1); unp = int((dks[pos] != rk).sum()) if len(dks) else 0
    _, dc = np.unique(dk, return_counts=True)
    r = dict(faces=int(len(F)), boundary=int((cnt == 1).sum()), nonmanifold=int((cnt > 2).sum()), unpaired_dir=unp,
             dup_dir=int((dc > 1).sum()), degenerate=int(deg.sum()))
    if V is not None:
        T = V[F]; r['vol_mm3'] = round(float(np.einsum('ij,ij->i', T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6) * 1e9, 3)
        ar = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1); r['zero_area'] = int((ar < 1e-12).sum())
    r['closed'] = r['boundary'] == 0 and r['nonmanifold'] == 0
    return r


# ---------------------------------------------------------------------------------------------------------------- poses
def rot_lid(V, deg, piv):
    """the renderer's set_lid: rotation about the hinge axis (parallel to Y through piv) by -deg about +Y (deg > 0 opens)"""
    if deg == 0: return V
    t = math.radians(-deg); c, s = math.cos(t), math.sin(t); R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return (V - piv) @ R.T + piv


def rot_pinion(V, deg, d):      # [fix4-hinge2]
    """the hinge pinion in lid pose deg: it meshes with the tongue's sector gear, so it turns -ratio x deg about its own axis (parallel to Y)"""
    cx, cz, n = [float(t) for t in d['lid_pinion'][:3]]; return rot_lid(V, -n * deg, np.array([cx, 0.0, cz]))


def stop_stroke(sv, pv, d, s, gap):
    """the renderer's STOPSTROKE (design_render.py) on the stop_v vertices (lid frame, closed pose). s = cap stroke (m).
    [fix4-qa] gap = tip gap to the SCANNED key (stop_kin[4], the physical part) instead of the stand-in gap the renderer uses."""
    sv = sv.copy()
    if s <= 0: return sv
    xP, zu, zl, L = [float(t) for t in d['stop_kin'][:4]]; xk = float(d['stop_axis'][0]); th = math.asin(min(s / L, 1.0))
    dx, dz = -L * (1 - math.cos(th)), -L * math.sin(th)
    mv = np.isin(pv, [61, 62, 63]) | ((pv == 69) & (np.abs(sv[:, 0] - xk) < 0.004)); sv[mv, 0] += dx; sv[mv, 2] += dz
    sr = min(s, gap); sv[np.isin(pv, [65, 67]), 2] -= sr
    for pc, top in ((66, True), (64, True)):
        m = pv == pc
        if m.any():
            z = sv[m, 2]; a, b = z.min(), z.max(); w = (z - a) / max(b - a, 1e-9)
            sv[m, 2] = z + w * dz - (1 - w) * (sr if pc == 66 else 0.0); sv[m, 0] += w * dx
    ma = pv == 68
    if ma.any():
        zc = np.where(sv[ma, 2] > 0.5 * (zu + zl), zu, zl); rx = sv[ma, 0] - xP; rz = sv[ma, 2] - zc
        sv[ma, 0] = xP + rx * math.cos(th) + rz * math.sin(th); sv[ma, 2] = zc - rx * math.sin(th) + rz * math.cos(th)
    return sv


# ---------------------------------------------------------------------------------------------------------------- chassis cut
def cut_chassis(B, d, log=print):
    """dev/base_parts Chassis with the fix-struts cuts exactly as design_render (bisect + capped, strutskit.cut_bmesh)"""
    import bpy
    sys.path.insert(0, HERE); import strutskit as SK
    me = bpy.data.meshes.new('qa_ch'); me.from_pydata(B['Chassis_v'].tolist(), [], B['Chassis_f'].tolist())
    ob = bpy.data.objects.new('QAChassis', me); bpy.context.scene.collection.objects.link(ob)
    sp = SK.from_npz(d); rep = None
    if sp and sp.get('mode') == 'cut':
        caps = (d['struts_cap_v'], d['struts_cap_f'], d['struts_cap_cut']) if 'struts_cap_v' in d.files else None
        rep = SK.cut_bmesh(ob, sp, caps=caps)
    me.calc_loop_triangles()
    cv = np.zeros(len(me.vertices) * 3); me.vertices.foreach_get('co', cv)
    cf = np.zeros(len(me.loop_triangles) * 3, np.int64); me.loop_triangles.foreach_get('vertices', cf)
    log('chassis cut: %s' % json.dumps(rep))
    return cv.reshape(-1, 3), cf.reshape(-1, 3), rep


# ---------------------------------------------------------------------------------------------------------------- families
class Part:
    pass      # attributes: name group frame V F src srcidx stats c lo hi hw render composite (+ solid, sfi, su set by the tools)

    def __repr__(self): return '<%s %s %d f>' % (self.name, self.frame, len(self.F))


def families(d, B=None, chassis=None, params=None, hardware=True):
    """[(group, frame, V, F, split, src, srcidx_fn)] - src = name of the npz vertex array the part comes from (STOP stroke)"""
    PM, PF, SP, SMOV = name_maps(); PMI = {v: k for k, v in PM.items()}; PFI = {v: k for k, v in PF.items()}; SPI = {v: k for k, v in SP.items()}
    out = []
    V, F, lab, part = d['v'], d['f'].astype(np.int64), d['lab'], d['part']
    out.append(('body', 'fixed', V, F[(part == 0) & (lab != 4)], True, None))
    out.append(('cover', 'fixed', V, F[(part == 1) & (lab != 4)], True, None))
    # the lid moulding: A-surface skin (lab 4) + underside plate + edge strip (kitm 1, 7) = one composite part (not split)
    km = d['kitm_part']; lidm = np.isin(km, [PM.get('underside', 1), PM.get('edge', 7)])
    Vl = np.r_[V, d['kitm_v']]; Fl = np.r_[F[lab == 4], d['kitm_f'][lidm].astype(np.int64) + len(V)]
    out.append(('lid', 'lid', Vl, Fl, False, None))
    for k in np.unique(km):
        if lidm[km == k].all(): continue
        out.append(('lid.' + PMI.get(int(k), str(int(k))), 'lid', d['kitm_v'], d['kitm_f'][km == k], True, None))
    kf = d['kitf_part']
    for k in np.unique(kf): out.append(('well.' + PFI.get(int(k), str(int(k))), 'gear' if (PFI.get(int(k)) == 'pinion' and 'lid_pinion' in d.files) else 'fixed', d['kitf_v'], d['kitf_f'][kf == k], True, None))      # [fix4-hinge2] frame 'gear': the pinion turns with the lid (rot_pinion)
    if 'stop_v' in d.files and len(d['stop_f']):
        sp_ = d['stop_part']
        for k in np.unique(sp_):
            nm = SPI.get(int(k), str(int(k)))
            out.append(('stop.' + nm, 'stop' if nm in SMOV else 'lid', d['stop_v'], d['stop_f'][sp_ == k], True, 'stop_v'))
    CK = [str(x) for x in d['ckit_kinds']]
    for k in np.unique(d['ckit_kind']): out.append(('ckit.' + CK[int(k)], 'fixed', d['ckit_v'], d['ckit_f'][d['ckit_kind'] == k], True, None))
    MK = [str(x) for x in d['mount_kinds']]
    for k in np.unique(d['mount_kind']): out.append(('mount.' + MK[int(k)], 'fixed', d['mount_v'], d['mount_f'][d['mount_kind'] == k], True, None))
    if 'sleeve_v' in d.files and len(d['sleeve_f']): out.append(('portframe', 'fixed', d['sleeve_v'], d['sleeve_f'], True, None))
    if 'plinth_v' in d.files and len(d['plinth_f']): out.append(('nosefloor', 'fixed', d['plinth_v'], d['plinth_f'], True, None))
    if 'lights_lens_v' in d.files:
        for i, nm in ((0, 'front'), (1, 'rear')):
            m = d['lights_lens_id'] == i
            if m.any(): out.append(('lens.' + nm, 'fixed', d['lights_lens_v'], d['lights_lens_f'][m], True, None))
        LK = [str(x) for x in d['lights_kinds']]
        for k in np.unique(d['lights_kit_kind']): out.append(('lamp.' + LK[int(k)], 'fixed', d['lights_kit_v'], d['lights_kit_f'][d['lights_kit_kind'] == k], True, None))
        for k in np.unique(d['lights_harn_kind']): out.append(('harn.' + LK[int(k)], 'fixed', d['lights_harn_v'], d['lights_harn_f'][d['lights_harn_kind'] == k], True, None))
        if len(d['lights_halo_f']): out.append(('halo_feed', 'lid', d['lights_halo_v'], d['lights_halo_f'], True, None))
    if 'ctlsi_v' in d.files and len(d['ctlsi_f']): out.append(('render.ctl_standin', 'fixed', d['ctlsi_v'], d['ctlsi_f'], True, None))
    if hardware:
        c = np.load(os.path.join(HERE, 'control.npz')); out.append(('hw.controls', 'fixed', c['v'], c['f'], True, None))
        if 'ctlsi_v' in d.files:      # the renderer's CtlTower box (render stand-in under the panel)
            sys.path.insert(0, HERE); import meshkit
            cv = c['v']; zc0 = float(cv[:, 2].min())
            tw = meshkit.box(((cv[:, 0].min() + cv[:, 0].max()) / 2, 0.0, zc0 - 0.004), (np.ptp(cv[:, 0]) - 0.012, np.ptp(cv[:, 1]) - 0.012, 0.0085))
            out.append(('render.ctl_tower', 'fixed', np.asarray(tw[0], float), np.asarray(tw[1], np.int64), False, None))
        if B is not None:
            for n in ('Wheel_L', 'Wheel_R', 'Caster_L', 'Caster_R'): out.append(('hw.' + n.lower(), 'fixed', B[n + '_v'], B[n + '_f'], False, None))
        if chassis is not None: out.append(('hw.chassis', 'fixed', chassis[0], chassis[1], True, None))
        k = np.load(os.path.join(HERE, 'keep.npz')); kl = k['lab']; kc = k['v'][k['f']].mean(1)
        kk = ~((kl == 6) & (kc[:, 0] > 0.27)) & ~((kl == 0) & (kc[:, 0] > 0.255)) & (kl != 0)      # the renderer's Skirt object (skirt + nose set)
        kv = k['v'].copy()
        if params:
            sys.path.insert(0, HERE); import tail as _tail; kv[:, 1] = _tail.make(params)(kv[:, 0], kv[:, 1])
        out.append(('hw.bumper', 'fixed', kv, k['f'][kk], True, None))
    return out


def build(npz, base_parts=None, params=None, hardware=True, cut=True, log=print, min_faces=1):
    """-> (d, parts, info). parts = list of Part (one per connected component; the lid composite is one part)."""
    d = np.load(npz); B = np.load(base_parts) if (hardware and base_parts) else None
    P = json.load(open(params)) if params else None
    ch = None
    if hardware and B is not None:
        if cut: cv, cf, rep = cut_chassis(B, d, log); ch = (cv, cf)
        else: ch = (B['Chassis_v'], B['Chassis_f'])
    parts = []
    for grp, frame, Vf, Ff, split, src in families(d, B, ch, P, hardware):
        Ff = np.asarray(Ff, np.int64)
        if not len(Ff): continue
        Vw, Fw, first = weld(Vf, Ff)
        if split: lab, n = components(len(Vw), Fw)
        else: lab, n = np.zeros(len(Fw), np.int64), 1
        sub = []
        for i in range(n):
            Fi = Fw[lab == i]
            if len(Fi) < min_faces: continue
            u = np.unique(Fi); rm = -np.ones(len(Vw), np.int64); rm[u] = np.arange(len(u)); Vi = Vw[u]
            sub.append((Vi, rm[Fi], first[u]))
        sub.sort(key=lambda t: (round(float(t[0][:, 0].mean()), 4), round(float(t[0][:, 1].mean()), 4), round(float(t[0][:, 2].mean()), 4)))
        for i, (Vi, Fi, si) in enumerate(sub):
            p = Part(); p.group = grp; p.name = grp if len(sub) == 1 else '%s#%d' % (grp, i); p.frame = frame; p.V = Vi; p.F = Fi
            p.src = src; p.srcidx = si; p.stats = None; p.lo = Vi.min(0); p.hi = Vi.max(0); p.c = (p.lo + p.hi) / 2
            p.hw = grp.startswith('hw.'); p.render = grp.startswith('render.'); p.composite = not split and grp == 'lid'
            parts.append(p)
    info = dict(npz=os.path.abspath(npz), parts=len(parts), faces=int(sum(len(p.F) for p in parts)),
                lid_pivot=[float(d['lid_pivot'][0]), 0.0, float(d['lid_pivot'][1])], lid_open=float(d['lid_open'][0]))
    lever = float(d['lid_latch_x'][0]) - float(d['lid_pivot'][0]) if 'lid_latch_x' in d.files else None
    info['lid_over_deg'] = round(-math.degrees(latch_over() / lever), 4) if lever else 0.0
    if 'stop_kin' in d.files:
        info['stop_stroke_mm'] = round(float(d['stop_axis'][3]) * 1000, 3); info['stop_tip_gap_mm'] = round(float(d['stop_kin'][4]) * 1000, 3)
    return d, parts, info
