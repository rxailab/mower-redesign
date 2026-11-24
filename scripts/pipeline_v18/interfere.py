# [fix4-qa] Pairwise interference matrix of a design npz: EVERY pair of parts (connected components, qa_parts.py) that is not on the
# allow-list of designed contacts / fused joints below must not touch, in every pose:
#   fixed x fixed, lid x lid ........ closed pose
#   lid x fixed ..................... lid swept over the angles (default: latch over-travel, 0, 0.3, 1, 2, 5, 10, 20, 30, 45, 60, open)
#   stop x fixed .................... the same sweep with the cap at rest, plus the cap stroke (default 0, tip gap, 3.0, full) at the
#                                     closed pose and at the latch over-travel
#   stop x lid, stop x stop ......... the cap stroke at the closed pose and at the over-travel (STOP parts move as the renderer's STOPSTROKE)
#   hardware x hardware ............. not tested (chassis, wheels, casters, bumper, controls are fixed hardware)
# Per pair and pose: surface crossings (BVH triangle overlap, clustered), penetration depth (sample points of one part that lie inside
# the other - ray parity, 2 of 3 rays (closed) / 4 of 5 rays (open shells, e.g. the scans), depth = distance to the other's surface,
# both directions, sampled within 5 mm of the crossings), a
# containment test when nothing crosses, and otherwise the minimum surface distance (bounded nearest-point search from area samples
# of both parts). Winding is never used: kit parts are often inside-out (solidaudit.py).
# Result classes: VIOLATION (fails), ALLOWED (designed contact / fusion), RENDER (render-only stand-in, info), NEAR (gap < --warn, info).
# usage (inside design/):
#   ../venv/bin/python interfere.py NPZ [--json OUT.json] [--base dev/base_parts.npz] [--params P.json] [--angles a,b,..] [--strokes mm,..]
#         [--near 1.0] [--warn 0.5] [--fail-below 0] [--dens 0.25] [--only 'glob,glob'] [--no-sweep] [--no-hw] [--quiet]
#         [--baseline OLD.json]  (exit 1 only for violations NEW against OLD's group pairs or > 0.02 mm deeper)
#         [--baseline-out B.json] (small file for --baseline: violation groups + summary; checks/f3/f3*.interf.base.json = the f3 state)
#         (with --only, 'cleared' lists every baseline pair outside the selection)
# runtime (single-threaded BVH queries): web npz 1 min 50 s / 2.2 GB, full npz 2 min 50 s / 2.8 GB (run the full one under heavy.lock).
#   (re-executes itself in Blender; or: blender -b -P interfere.py -- NPZ ...).  exit 0 = clean, 1 = violations.
# STOP stroke: as the renderer (cap + carrier on the parallel links, boot / compliance spring squashed), but the rod + tip stop where the
# tip lands on the REAL (scanned) key in that lid pose (ray down from the tip), not at the nominal gap.
import sys, os, json, time, math, fnmatch
HERE_ = os.path.dirname(os.path.abspath(sys.argv[sys.argv.index('-P') + 1] if '-P' in sys.argv else __file__)); sys.path.insert(0, HERE_)
import qa_parts as QP
QP.ensure_blender(os.path.join(HERE_, 'interfere.py'))
import numpy as np
from mathutils.bvhtree import BVHTree
from mathutils import Vector

# ======================================================================================================== ALLOW-LIST
# (group A, group B, kind, limit mm, when, why).  Groups are qa_parts names; fnmatch patterns, '|' = alternatives; order-free.
#   'fused'   : one moulding / an un-booleaned union of primitives / an insert, a bonded or mated part - any overlap (depth not measured)
#   'contact' : designed touch: allowed while the penetration depth stays <= limit mm (None = depth not limited; deeper = VIOLATION)
# when: None (every pose) or a python expression of a (lid angle, deg), s (cap stroke, mm), OPEN, OVER (deg), GAP, SMAX (mm).
# Sources: lidsim.DESIGNED / SEAL, hingekit.DESIGNED3, lightcheck.DESIGNED, check_wall (fused parts), deckcheck / kitcheck, stopkit
# header (STOP guide / return / plunger / seal), hingekit.spring_unit, design_gen kit2 comments, CHANGES.txt 6.7 / 6.9.
# Anything not listed must keep a gap > 0. NOT listed on purpose (review findings): ckit leg / barb x chassis (G1), stop.rspring x lid
# (G2), halo_feed x lid (G3), the pod solid round the drum / idler (M2), body kit x chassis apart from the designed seats.
ALLOW = [
    # --- body / cover mouldings and what is moulded on them
    ('cover', 'ckit.*', 'fused', None, None, 'cover kit (legs, barbs, pins, seat pads, stakes, flange) moulded on the cover (check_wall: fused only into the cover)'),
    ('ckit.*', 'ckit.*', 'fused', None, None, 'one cover-kit feature from overlapping primitives (leg stem + leg + barb, pin + ribs)'),
    ('body', 'mount.deck|mount.boss|mount.rib|mount.bracket|mount.clip|mount.well|mount.hem_lip|mount.other|mount.seat|mount.drain', 'fused', None, None,
     'body kit moulded on the body wall (check_wall: boss, rib, bracket, clip, deck edge start in the wall; seat flange fused 0.6, CHANGES 6.9)'),
    ('mount.*', 'mount.*', 'fused', None, None, 'body kit = one moulding with the deck (bosses, ribs, brackets, clips, pads, wells, pilot / slot inlays) + screws through pads / wells'),
    ('body', 'portframe', 'fused', None, None, 'port frame = part of the body moulding (fix-ports; lightcheck housing|sleeves note)'),
    ('portframe', 'portframe', 'fused', None, None, 'port frame: 6 overlapping solids of one moulding (un-booleaned union, review G7)'),
    ('body', 'nosefloor', 'fused', None, None, 'nose floor joined to the body (union; review G6 whitelist)'),
    ('nosefloor', 'nosefloor', 'fused', None, None, 'nose floor primitives (union)'),
    # --- lamp modules, harness (lightcheck.DESIGNED and the module build in lightkit)
    ('body', 'lamp.boss', 'fused', None, None, 'lightcheck: module boss moulded on the wall'),
    ('body', 'lamp.gasket', 'contact', 0.3, None, 'lightcheck: foam gasket on the inner skin, local extra squeeze <= 0.3'),
    ('lamp.housing', 'lens.*', 'contact', 0.4, None, 'lightcheck: laser weld land (collapse 0.05 nominal, <= 0.4 local)'),
    ('lamp.boss', 'lamp.boss', 'fused', None, None, 'boss primitives (union)'),
    ('lamp.shroud', 'lamp.shroud', 'fused', None, None, 'connector shroud primitives (union)'),
    ('lamp.camera', 'lamp.camera', 'fused', None, None, 'camera module primitives (union)'),
    ('lamp.emitter', 'lamp.emitter', 'fused', None, None, 'halo emitter: board + LED'),
    ('lamp.screw', 'lamp.boss|lamp.ear', 'fused', None, None, 'module screw through the ear into the boss'),
    ('lamp.ear', 'lamp.boss', 'contact', 0.05, None, 'module ear seated on the boss'),
    ('lamp.housing', 'lamp.ear|lamp.shroud|lamp.vent', 'fused', None, None, 'moulded on the housing (ears, connector shroud, vent)'),
    ('lamp.housing', 'lamp.gasket', 'contact', 0.05, None, 'gasket on the housing flange'),
    ('lamp.housing', 'lamp.camera', 'contact', 0.15, None, 'camera module seated in the housing nose (0.1 measured on f3)'),
    ('lamp.pcb', 'lamp.led|lamp.camera|lamp.tof', 'contact', 0.05, None, 'components on the PCB'),
    ('lamp.foam', 'lamp.tof', 'contact', 0.15, None, 'crosstalk foam round the ToF sensor (compressed)'),
    ('lamp.emitter', 'hw.controls', 'fused', None, None, 'lightcheck: halo emitter board on the control panel (supplier part)'),
    ('harn.cable', 'harn.grommet|harn.plug|lamp.shroud', 'fused', None, None, 'cable through its grommet / into its plug / into the lamp connector'),
    ('harn.plug', 'lamp.shroud', 'fused', None, None, 'plug mated in the lamp connector shroud'),
    ('harn.plug', 'harn.plug', 'fused', None, None, 'plug primitives (union)'),
    ('harn.svc', 'harn.svc', 'fused', None, None, 'service connector pair, mated'),
    ('harn.clip', 'harn.clip', 'fused', None, None, 'clip primitives (union)'),
    ('harn.chassis_lead', 'harn.chassis_lead|harn.svc', 'fused', None, None, 'chassis lead primitives; lead into the service connector'),
    ('harn.clip|harn.cradle|harn.grommet', 'mount.deck', 'fused', None, None, 'lightcheck: clip / cradle moulded on the deck, grommet in the deck hole'),
    ('harn.svc', 'harn.cradle|mount.deck', 'contact', 0.05, None, 'lightcheck: connector sits in the cradle'),
    ('harn.chassis_lead', 'hw.chassis', 'fused', None, None, 'lightcheck: lead enters the chassis grommet at its end'),
    # --- body on the chassis (mounts.json fastening points)
    ('mount.screw', 'hw.chassis', 'fused', None, None, 'screw threaded into the chassis post'),
    ('mount.pad|mount.pilot', 'hw.chassis', 'contact', 0.3, None, 'seat pad on the post top / pilot inlay at the screw hole (bearing face on the scanned post: <= 0.3 scan noise)'),
    ('mount.boss', 'hw.chassis', 'contact', 0.3, None, 'design_gen: front boss standing on the chassis wing (bearing face at its foot: <= 0.3 scan noise)'),
    # --- lid moulding and what is moulded / inserted on it
    ('lid', 'lid.rib|lid.boss|lid.striker', 'fused', None, None, 'moulded on the lid underside'),
    ('lid.rib|lid.boss|lid.striker', 'lid.rib|lid.boss|lid.striker', 'fused', None, None, 'lid moulding: ribs, bosses and the striker post join'),
    ('lid.tongue|lid.blade|lid.lug', 'lid.boss|lid.rib|lid.tongue|lid.blade|lid.lug', 'fused', None, None, 'hinge tongue / blade / lug: metal inserts in the lid bosses, lug on the tongue (fix-hinge)'),
    ('lid', 'stop.bore|stop.bracket|stop.halo', 'fused', None, None, 'stopkit: bore boss, link bracket and opal halo boss moulded into the lid skin (halo fused 1.6, CHANGES 6.7.2)'),
    ('halo_feed', 'stop.halo', 'fused', None, None, 'CHANGES 6.7.2: pipe top fused 0.1 into the opal boss (same 2K shot)'),
    # --- STOP key (stopkit header: parallel links, return springs, plunger + compliance spring, boot)
    ('stop.arm', 'stop.arm', 'fused', None, None, 'link U-channel from primitives'),
    ('stop.pin', 'stop.arm|stop.bracket|stop.carrier', 'fused', None, None, 'link pins through the arms, the rear bracket and the carrier'),
    ('stop.rspring', 'stop.rspring', 'fused', None, None, 'torsion spring: coil + legs'),
    ('stop.rspring', 'stop.arm|stop.pin|stop.bracket', 'contact', 0.05, None, 'return spring on its pin, leg on the link'),
    ('stop.rod', 'stop.tip', 'fused', None, None, 'TPE tip on the plunger rod'),
    ('stop.cap', 'stop.carrier', 'fused', None, None, 'cap snapped on the carrier'),
    ('stop.boot', 'stop.carrier|stop.halo|stop.bore', 'fused', None, None, 'silicone boot: bead on the stem, bead in the halo-boss bore'),
    ('stop.bore', 'stop.halo|stop.bumper', 'fused', None, None, 'bore inside the halo boss; TPE up-stop bumper bonded under the bore boss'),
    ('stop.arm', 'stop.bumper', 'contact', 0.05, 's == 0', 'up-stop: upper link on the TPE bumper at rest'),
    ('stop.cspring', 'stop.carrier|stop.rod', 'contact', 0.05, None, 'compliance spring between the rod shoulder and the carrier'),
    ('stop.rod', 'stop.carrier', 'contact', 0.1, None, 'rod head on its shoulder in the stem (16 N preload)'),
    ('stop.carrier', 'stop.halo', 'contact', 0.05, 's >= SMAX - 0.01', 'down-stop: carrier stop ring on the halo boss at full stroke'),
    ('stop.tip', 'hw.controls', 'contact', 1.5, 's > 0 or a < 0', 'lidsim DESIGNED: the TPE tip presses the real STOP key (0.3..1.5 switch travel)'),
    ('stop.boot', 'lid', 'contact', 0.3, 's > 0', 'rolling boot: the renderer-style linear squash of the boot (STOPSTROKE) is only an approximation'),
    # --- [fix4-stop2] STOP key v4 (stop4.py header): dark socket + opal ring insert in the lid, light post + magnet, POM bush, lip seal, pivot cover
    ('lid|lid.rib', 'stop.socket|stop.post|stop.pcover|stop.bush', 'fused', None, None, '[fix4-stop2] socket = the lid\'s dark shot through the skin cut (bridges the plate, joins the ribs); light post moulded on the underside / tied to the front ring rib; pivot cover snapped into the plate'),
    ('stop.halo', 'stop.socket', 'fused', None, None, '[fix4-stop2] opal ring light guide insert-moulded in the socket pocket'),
    ('stop.post|stop.socket|stop.pcover', 'stop.post|stop.socket|stop.pcover', 'fused', None, None, '[fix4-stop2] post (+ its rib gusset) and socket = one dark lid shot; pivot cover tray from plates'),
    ('halo_feed', 'stop.socket|stop.post|stop.halo', 'fused', None, None, '[fix4-stop2] the clear pipe of the ring insert: through the socket bridge, sleeved in the dark post, into the ring'),
    ('stop.magnet', 'stop.post', 'fused', None, None, '[fix4-stop2] lid-closed magnet in its closed pocket in the post'),
    ('stop.bush', 'stop.socket|stop.bore|stop.bumper', 'fused', None, None, '[fix4-stop2] POM guide bush pressed into the socket bore / bore ring; TPE bumper + lip bonded under it'),
    ('stop.bore', 'stop.socket', 'fused', None, None, '[fix4-stop2] bore ring = part of the socket moulding under the plate'),
    ('stop.boot', 'stop.socket|stop.bush', 'contact', 0.05, 's == 0', '[fix4-stop2] boot bead on the socket bore rim'),
    ('stop.boot', 'stop.socket|stop.bush', 'contact', 0.6, 's > 0', '[fix4-stop2] rolling boot: the model squashes it linearly (STOPSTROKE) instead of rolling it into the bore'),
    ('stop.carrier', 'stop.socket', 'contact', 0.05, 's >= SMAX - 0.01', '[fix4-stop2] down-stop: the shroud\'s castellated rim on the socket at full stroke'),
    ('stop.carrier', 'stop.bumper', 'contact', 0.3, None, '[fix4-stop2] TPE lip seal (2nd barrier) wipes the stem; the lip follows the 0.22 link-arc runout of the stem at full stroke'),
    ('stop.rspring', 'stop.arm', 'contact', 0.75, 's > 0', '[fix4-stop2] return-spring leg under the lower link: the leg turns with the link, the model keeps it static -> theta x 7.6 mm <= 0.62 at full stroke'),
    ('stop.pcover', 'stop.bracket', 'fused', None, None, '[fix4-stop2] pivot cover clipped to the bracket'),
    # --- lid well, latch, hinge cassette (lidsim DESIGNED / SEAL, hingekit.spring_unit)
    ('lid', 'well.seal', 'contact', 2.0, 'a <= 0.3', 'lidsim SEAL: bulb seal compressed at closed (designed), lifts off when the lid opens'),
    ('lid.striker', 'well.claw', 'contact', 1.6, 'a <= 2.0', 'lidsim DESIGNED: latch rotor holds the striker bar; pushed through it at the 1.5 latch over-travel (push-push)'),
    ('lid.lug', 'well.bumper', 'contact', 0.05, 'a >= OPEN - 0.01', 'lidsim DESIGNED: open stop at open_deg'),
    ('well.well', 'well.drain|well.ear|well.pocket|well.lip|well.latch|well.pod', 'fused', None, None, 'moulded on the lid well (drains, staked ears, pockets, lip, latch housing, hinge pods)'),
    ('well.pod', 'well.pod', 'fused', None, None, 'pod primitives (union)'),
    ('well.guide', 'well.guide|well.pod|well.bumper', 'fused', None, None, 'guide rails / cheeks of the hinge cassette in the pod; TPE open-stop bumper in the guide slot'),
    ('well.claw', 'well.claw|well.latch', 'fused', None, None, 'latch rotor (primitives) on its housing pin'),
    ('well.damper', 'well.damper', 'fused', None, None, 'drum + spiral + damper + axle (one spring unit)'),
    ('well.idler', 'well.idler', 'fused', None, None, 'idler pulley on its pin'),
    ('well.spring', 'well.idler|well.damper', 'contact', 0.05, None, 'hingekit: tape over the idler, wound on the drum'),
    # --- [fix4-front2] front-face parts (appended at the end so the indices of the rules above stay as in the QA report)
    ('lamp.carrier', 'lamp.carrier', 'fused', None, None, '[fix4-front2] light-bar carrier channel + its end plate: one white PC moulding (un-booleaned, G5)'),
    ('lamp.housing', 'lamp.pad', 'fused', None, None, '[fix4-front2] m9 crush pads moulded on the housing flange'),
    ('body', 'lamp.pad', 'contact', 0.1, None, '[fix4-front2] m9 crush pads = the flush datums, pushed onto the inner skin by the wave springs (crush <= 0.1)'),
    ('harn.cable', 'harn.svc', 'fused', None, None, '[fix4-front2] S12 body cable into the sealed service plug (wire-seal boot)'),
    ('harn.cable', 'harn.clip', 'contact', 0.05, None, '[fix4-front2] P-clip holds the cable (designed touch)'),
]
# review finding ids shown next to a VIOLATION of these group pairs (context only - they still fail)
REFS = [('ckit.leg|ckit.barb', 'hw.chassis', 'G1'), ('ckit.barb', 'mount.deck', 'G1 region (x 118 legs)'), ('stop.rspring', 'lid', 'G2'),
        ('halo_feed', 'lid', 'G3'), ('stop.carrier', 'stop.tip', 'S6 (stem on the tip: compliance travel used up)'),
        ('well.damper|well.idler|well.spring', 'well.pod', 'M2 (pod solid round drum / idler: no cavity)'), ('well.damper', 'well.well', 'M2 (axial room)'),
        ('mount.pad', 'hw.chassis', 'new: seat pad under the scanned post top (mounts.json post z)'),
        ('mount.bracket', 'hw.chassis', 'new: rear socket boss / bracket plate x chassis apron'),
        ('mount.rib|mount.boss|mount.pilot', 'hw.chassis', 'new: body kit x real chassis'), ('ckit.pin', 'mount.deck', 'new: nose pin x deck hole')]
# ======================================================================================================== options
a = QP.script_args(); NPZ = a[0]
def opt(k, dv=None):
    return a[a.index(k) + 1] if k in a else dv
OUT = opt('--json'); BASE = opt('--base', os.path.join(QP.HERE, 'dev', 'base_parts.npz')); PARAMS = opt('--params')
NEAR = float(opt('--near', 1.0)) / 1000; WARN = float(opt('--warn', 0.5)) / 1000; FAILB = float(opt('--fail-below', 0.0)) / 1000
BASEL = opt('--baseline'); BOUT = opt('--baseline-out'); DENS = float(opt('--dens', 0.25)) * 1e-6; ONLY = [p for p in (opt('--only') or '').split(',') if p]; QUIET = '--quiet' in a
TOUCH = 0.005e-3      # |gap| below 5 um = touching
os.chdir(QP.HERE); t0 = time.time()
d, parts, info = QP.build(NPZ, BASE, PARAMS, hardware='--no-hw' not in a)
OPEN, OVER = info['lid_open'], info['lid_over_deg']; GAPMM = info.get('stop_tip_gap_mm', 0.0); SMAX = info.get('stop_stroke_mm', 0.0)
ANG = [float(t) for t in opt('--angles').split(',')] if opt('--angles') else [OVER, 0.0, 0.3, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 45.0, 60.0, OPEN]
STR = [float(t) for t in opt('--strokes').split(',')] if opt('--strokes') else ([0.0, GAPMM, 3.0, SMAX] if SMAX else [0.0])
if '--no-sweep' in a: ANG = [0.0]; STR = [0.0]
ANG = sorted(set(round(t, 4) for t in ANG) | {0.0}); STR = sorted(set(round(t, 4) for t in STR) | {0.0})
PIV = np.array(info['lid_pivot']); N = len(parts)
print('parts %d (%d faces), built %.1f s; angles %s; strokes %s mm' % (N, info['faces'], time.time() - t0, ANG, STR), flush=True)
for p in parts:
    st = QP.edge_stats(p.F); p.stats = st
    p.solid = st['closed'] or st['boundary'] < 0.2 * len(p.F)           # parity inside test (majority of 5 rays on open shells, e.g. the scans)
    p.tight = st['closed'] or st['boundary'] < 0.01 * len(p.F)          # containment test (no crossing at all) only on (nearly) closed shells
# STOP stroke: per-vertex part id of stop_v
if 'stop_v' in d.files and len(d['stop_f']):
    PV = np.zeros(len(d['stop_v']), np.int16); PV[d['stop_f'].ravel()] = np.repeat(d['stop_part'], 3); SV = {}
    def stop_V(s, g):
        k = (s, round(g, 7))
        if k not in SV: SV[k] = QP.stop_stroke(d['stop_v'], PV, d, s / 1000.0, g)
        return SV[k]
LAND = {}
def landing(ang):
    """tip-to-real-key gap (m) along the stroke (lid-frame -z) in lid pose ang, cap at rest; nominal gap without the controls"""
    if ang not in LAND:
        tip = [p for p in parts if p.group == 'stop.tip']; ctl = [p for p in parts if p.group == 'hw.controls']; g = GAPMM / 1000.0
        if tip and ctl:
            t = math.radians(-ang); dr = Vector((math.sin(t) * -1.0, 0.0, -math.cos(t))); tc = tree(ctl[0], (0.0, 0.0)); best = np.inf
            for v in QP.rot_lid(tip[0].V, ang, PIV):
                h = tc.ray_cast(Vector(v), dr, 0.02)
                if h[0] is not None: best = min(best, h[3])
            g = float(best) if np.isfinite(best) else 1.0
        LAND[ang] = g
    return LAND[ang]

def posed(p, pose):
    ang, s = pose
    if p.frame == 'fixed': return p.V
    if p.frame == 'gear': return QP.rot_pinion(p.V, ang, d)      # [fix4-hinge2] hinge pinion: meshes with the tongue, turns -ratio x the lid angle
    V = stop_V(s, landing(ang))[p.srcidx] if (p.frame == 'stop' and s > 0) else p.V
    return QP.rot_lid(V, ang, PIV)

def pkey(p, pose):
    return None if p.frame == 'fixed' else (pose[0], 0.0 if p.frame in ('lid', 'gear') else pose[1])      # [fix4-hinge2] gear

def applies(p, q, pose):
    ang, s = pose; F = {p.frame, q.frame}
    if 'gear' in F: return s == 0 and (F != {'gear'} or ang == 0)      # [fix4-hinge2] the pinion vs everything over the lid sweep (it turns with the lid)
    if F <= {'fixed'} or F <= {'lid'}: return ang == 0 and s == 0
    if F == {'lid', 'fixed'}: return s == 0
    if F == {'stop', 'fixed'}: return s == 0 or ang in (0.0, round(OVER, 4))
    return ang in (0.0, round(OVER, 4))      # stop x lid, stop x stop: the stroke at closed + over-travel (the tip lands earlier there)

def gm(g, pat):
    return any(fnmatch.fnmatch(g, t) for t in pat.split('|'))
def pairm(p, q, ga, gb):
    return (gm(p.group, ga) and gm(q.group, gb)) or (gm(p.group, gb) and gm(q.group, ga))
def rule_for(p, q, pose):
    ang, s = pose
    for i, (ga, gb, kind, lim, when, why) in enumerate(ALLOW):
        if pairm(p, q, ga, gb) and (when is None or eval(when, {}, dict(a=ang, s=s, OPEN=OPEN, OVER=OVER, GAP=GAPMM, SMAX=SMAX))): return i
    return None
def ref_for(p, q):
    return next((r for ga, gb, r in REFS if pairm(p, q, ga, gb)), None)

# ======================================================================================================== geometry kernels
import zlib      # [fix4-int] one RNG stream PER PART (seed = crc32 of its name): a branch that adds or drops parts no longer shifts the samples
prng = lambda *k: np.random.default_rng([7] + [zlib.crc32(str(t).encode()) for t in k])      # [fix4-int] of unrelated parts (was: one stream, seed 7)
for p in parts:      # area samples as (face, barycentric) so they follow any pose (also the deforming boot / spring)
    T = p.V[p.F]; ar = 0.5 * np.linalg.norm(np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0]), axis=1)
    ns = np.clip((ar / DENS).astype(np.int64), 0, 400); fi = np.repeat(np.arange(len(p.F)), ns); u = prng(p.name).random((len(fi), 2)); fl = u.sum(1) > 1; u[fl] = 1 - u[fl]      # [fix4-int]
    p.sfi, p.su = fi, u
SCACHE = {}; TCACHE = {}
def samples(p, pose):
    k = (p.name, pkey(p, pose))
    if k not in SCACHE:
        V = posed(p, pose); T = V[p.F]; S = T[p.sfi, 0] + (T[p.sfi, 1] - T[p.sfi, 0]) * p.su[:, :1] + (T[p.sfi, 2] - T[p.sfi, 0]) * p.su[:, 1:]
        SCACHE[k] = np.r_[V, T.mean(1), S]
    return SCACHE[k]
def tree(p, pose):
    k = (p.name, pkey(p, pose))
    if k not in TCACHE: TCACHE[k] = BVHTree.FromPolygons(posed(p, pose).tolist(), p.F.tolist(), all_triangles=True)
    return TCACHE[k]
def bbox(p, pose):
    V = posed(p, pose); return V.min(0), V.max(0)
D3 = [Vector(v).normalized() for v in ((0.5773, 0.5774, 0.5775), (-0.7071, 0.1003, -0.7002), (0.2017, -0.9013, 0.3891))]
D5 = D3 + [Vector(v).normalized() for v in ((0.0213, 0.0137, 0.9997), (-0.3317, 0.6101, -0.7195))]
def inside(tr, p, solid_closed):
    votes = 0; dirs = D3 if solid_closed else D5
    for dr in dirs:
        o = Vector(p); n = 0
        for _ in range(400):
            loc, nor, idx, dist = tr.ray_cast(o, dr, 2.0)
            if loc is None: break
            n += 1; o = loc + dr * 1e-6
        votes += n & 1
    return votes * 2 > len(dirs) if solid_closed else votes >= 4      # open shells (scans, kit sheets): 4 of 5 rays must agree
def crop(S, lo, hi, m):
    return S[((S > lo - m) & (S < hi + m)).all(1)]
def depth_into(X, Y, pose, region, cap=6000):
    """samples of X (in region) that lie inside Y -> (n inside, max depth, where)"""
    if not Y.solid: return 0, 0.0, None
    lo, hi = bbox(Y, pose); S = crop(crop(samples(X, pose), lo, hi, 0.0), region[0], region[1], 0.0)
    if len(S) > cap: S = S[prng(X.name, Y.name, pose).choice(len(S), cap, replace=False)]      # [fix4-int] per pair + pose
    tY = tree(Y, pose); n = 0; best = (0.0, None); cl = Y.stats['closed']
    for q in S:
        if inside(tY, q, cl):
            n += 1; r = tY.find_nearest(Vector(q))
            if r[3] is not None and r[3] > best[0]: best = (r[3], q)
    return n, best[0], best[1]
def min_dist(X, Y, pose, bound):
    lo, hi = bbox(Y, pose); S = crop(samples(X, pose), lo, hi, bound)
    if not len(S): return None, None
    tY = tree(Y, pose); best = (np.inf, None)
    for q in S:
        r = tY.find_nearest(Vector(q), bound)
        if r[0] is not None and r[3] < best[0]: best = (r[3], q)
    return best

def measure(p, q, pose, full=True):
    """-> dict: x (crossing triangle pairs), at (crossing cluster), pen (max depth, m), pen_at, n_in, contained, gap (m), gap_at"""
    tp, tq = tree(p, pose), tree(q, pose); pr = tp.overlap(tq); r = dict(x=len(pr))
    if pr:
        pr = np.array(pr, np.int64); Vp, Vq = posed(p, pose), posed(q, pose)
        cm = (Vp[p.F[pr[:, 0]]].mean(1) + Vq[q.F[pr[:, 1]]].mean(1)) / 2
        key = np.round(cm / 0.004).astype(np.int64); _, iv, cnt = np.unique(key, axis=0, return_inverse=True, return_counts=True); iv = iv.ravel()
        r['clusters'] = [dict(n=int(cnt[k]), at_mm=(cm[iv == k].mean(0) * 1000).round(2).tolist()) for k in np.argsort(-cnt)[:4]]
        r['x_box_mm'] = [(cm.min(0) * 1000).round(2).tolist(), (cm.max(0) * 1000).round(2).tolist()]
        if not full: return r
        reg = (cm.min(0) - 0.005, cm.max(0) + 0.005); n1, d1, w1 = depth_into(p, q, pose, reg); n2, d2, w2 = depth_into(q, p, pose, reg)
        r['n_in'] = n1 + n2; r['pen'] = max(d1, d2); w = w1 if d1 >= d2 else w2
        r['pen_at'] = None if w is None else (np.asarray(w) * 1000).round(2).tolist(); r['gap'] = -r['pen']
        return r
    # no crossing: containment (a connected surface either lies fully inside a solid or fully outside it), else the gap
    lp, hp = bbox(p, pose); lq, hq = bbox(q, pose)
    for X, Y, lx, hx, ly, hy in ((p, q, lp, hp, lq, hq), (q, p, lq, hq, lp, hp)):
        if Y.tight and (lx >= ly).all() and (hx <= hy).all():
            Vx = posed(X, pose); tY = tree(Y, pose); pts = Vx[np.linspace(0, len(Vx) - 1, 3).astype(int)]
            if sum(inside(tY, v, Y.stats['closed']) for v in pts) >= 2:
                S = samples(X, pose); dd = [tY.find_nearest(Vector(v))[3] for v in S[:: max(1, len(S) // 4000)]]
                j = int(np.argmax(dd)); r.update(contained='%s in %s' % (X.name, Y.name), pen=float(dd[j]), pen_at=(S[:: max(1, len(S) // 4000)][j] * 1000).round(2).tolist(), gap=-float(dd[j]))
                return r
    g1, w1 = min_dist(p, q, pose, NEAR); g2, w2 = min_dist(q, p, pose, NEAR)
    cand = [(g, w) for g, w in ((g1, w1), (g2, w2)) if g is not None and np.isfinite(g)]
    if cand:
        g, w = min(cand, key=lambda t: t[0]); r['gap'] = float(g); r['gap_at'] = (np.asarray(w) * 1000).round(2).tolist()
    return r

# ======================================================================================================== the sweep
POSES = [(0.0, 0.0)] + [(t, 0.0) for t in ANG if t != 0] + [(t, s) for t in (0.0, round(OVER, 4)) for s in STR if s > 0]
FR = np.array([p.frame for p in parts]); HW = np.array([p.hw for p in parts]); RD = np.array([p.render for p in parts])
sel = np.ones(N, bool) if not ONLY else np.array([any(fnmatch.fnmatch(p.group, g) or fnmatch.fnmatch(p.name, g) for g in ONLY) for p in parts])
RES = {}; npairs = 0
for pose in POSES:
    tp0 = time.time()
    LO = np.array([bbox(p, pose)[0] for p in parts]); HI = np.array([bbox(p, pose)[1] for p in parts])
    ov = ((LO[:, None, :] <= HI[None, :, :] + NEAR) & (LO[None, :, :] <= HI[:, None, :] + NEAR)).all(2)
    ov &= ~(HW[:, None] & HW[None, :]); ov &= sel[:, None] | sel[None, :]; ov = np.triu(ov, 1)
    if pose != (0.0, 0.0): ov &= ((FR[:, None] != 'fixed') | (FR[None, :] != 'fixed'))
    I, J = np.nonzero(ov); nj = 0
    for i, j in zip(I.tolist(), J.tolist()):
        p, q = parts[i], parts[j]
        if not applies(p, q, pose): continue
        ri = rule_for(p, q, pose); full = ri is None or ALLOW[ri][2] != 'fused'
        r = measure(p, q, pose, full); nj += 1
        g = r.get('gap'); inter = r['x'] > 0 or 'contained' in r or (g is not None and g < TOUCH)
        if ri is not None:
            kind, lim = ALLOW[ri][2], ALLOW[ri][3]
            cls = 'ALLOWED' if (kind == 'fused' or lim is None or r.get('pen', 0.0) <= lim / 1000 + 1e-9) else 'VIOLATION'
        elif p.render or q.render: cls = 'RENDER' if inter else ('NEAR' if g is not None and g < WARN else 'ok')
        elif inter or (g is not None and g < FAILB): cls = 'VIOLATION'
        elif g is not None and g < WARN: cls = 'NEAR'
        else: cls = 'ok'
        if cls == 'ok' and ri is None: continue
        r.update(cls=cls, rule=ri, pose=dict(lid_deg=pose[0], stroke_mm=pose[1]))
        RES.setdefault((p.name, q.name), []).append(r)
        if not QUIET and cls in ('VIOLATION', 'RENDER'): print('  %-9s %-22s %-22s lid %6.2f s %4.2f  x %5d  pen %6.3f  gap %s' % (cls, p.name, q.name, pose[0], pose[1], r['x'], r.get('pen', 0) * 1000, None if g is None else round(g * 1000, 3)), flush=True)
    npairs += nj
    for k in [k for k in TCACHE if k[1] is not None]: del TCACHE[k]
    for k in [k for k in SCACHE if k[1] is not None]: del SCACHE[k]
    print('pose lid %.3f deg stroke %.2f mm: %d pairs measured (%.1f s)' % (pose[0], pose[1], nj, time.time() - tp0), flush=True)

# ======================================================================================================== report
PN = {p.name: p for p in parts}; ORD = {'VIOLATION': 0, 'RENDER': 1, 'ALLOWED': 2, 'NEAR': 3}
def worst(rs):
    return min(rs, key=lambda r: (ORD[r['cls']], r.get('gap') if r.get('gap') is not None else 1.0, -r['x']))
rows = []
for (an, bn), rs in RES.items():
    w = worst(rs); cls = min((r['cls'] for r in rs), key=lambda c: ORD[c])
    rows.append(dict(a=an, b=bn, ga=PN[an].group, gb=PN[bn].group, cls=cls, rule=w['rule'], why=None if w['rule'] is None else ALLOW[w['rule']][5],
                     worst=w, poses_bad=[r['pose'] for r in rs if r['cls'] == cls][:12], n_poses=len(rs),
                     a_closed=PN[an].stats['closed'], b_closed=PN[bn].stats['closed'], ref=ref_for(PN[an], PN[bn]) if cls == 'VIOLATION' else None))
rows.sort(key=lambda r: (ORD[r['cls']], r['worst'].get('gap') if r['worst'].get('gap') is not None else 1.0))
viol = [r for r in rows if r['cls'] == 'VIOLATION']
used = sorted({r['rule'] for r in rows if r['rule'] is not None})
def fmt(r):
    w = r['worst']; at = w.get('pen_at') or (w.get('clusters') or [{}])[0].get('at_mm') or w.get('gap_at')
    g = w.get('gap'); pz = w['pose']
    return '%-9s %-24s %-24s %6s %8s %6s  lid %6.2f s %4.2f  at %s%s%s' % (r['cls'], r['a'][:24], r['b'][:24], w['x'], '-' if g is None else '%.3f' % (g * 1000),
           w.get('n_in', '-'), pz['lid_deg'], pz['stroke_mm'], at, ('  [%s]' % w['contained']) if 'contained' in w else '', ('  <%s>' % r['ref']) if r.get('ref') else '')
print('\n=== INTERFERENCE %s: %d parts, %d poses, %d pair-poses measured, %.0f s' % (os.path.basename(NPZ), N, len(POSES), npairs, time.time() - t0))
print('%-9s %-24s %-24s %6s %8s %6s  %s' % ('class', 'part A', 'part B', 'x', 'gap mm', 'n_in', 'worst pose / where (mm; pen_at = deepest point, else crossing / nearest)'))
for r in rows:
    if r['cls'] in ('VIOLATION', 'RENDER', 'NEAR'): print(fmt(r))
GV = {}
for r in viol:
    k = tuple(sorted((r['ga'], r['gb']))); g = GV.setdefault(k, dict(n=0, pen=0.0, x=0, ref=r.get('ref')))
    g['n'] += 1; g['x'] += r['worst']['x']; g['pen'] = max(g['pen'], -(r['worst'].get('gap') or 0.0))
print('--- violations per group pair (n components, crossings, deepest mm):')
for k, g in sorted(GV.items(), key=lambda t: -t[1]['pen']): print('  %-22s %-22s n %3d  x %6d  deepest %.3f  %s' % (k[0], k[1], g['n'], g['x'], g['pen'] * 1000, g['ref'] or ''))
print('--- allowed (designed contacts / fusions), per rule:')
for i in used:
    rr = [r for r in rows if r['rule'] == i]; dp = max((r['worst'].get('pen', 0.0) for r in rr), default=0.0)
    print('  [%2d] %-14s %-14s %-7s %3d pairs, deepest %.3f mm  - %s' % (i, ALLOW[i][0], ALLOW[i][1], ALLOW[i][2], len(rr), dp * 1000, ALLOW[i][5][:90]))
unused = [i for i in range(len(ALLOW)) if i not in used]
if unused: print('--- allow-list rules not hit (stale?):', ', '.join('[%d] %s|%s' % (i, ALLOW[i][0], ALLOW[i][1]) for i in unused))
rep = dict(info=info, poses=[dict(lid_deg=t, stroke_mm=s) for t, s in POSES], near_mm=NEAR * 1000, warn_mm=WARN * 1000, fail_below_mm=FAILB * 1000,
           time_s=round(time.time() - t0, 1), n_violations=len(viol), summary={c: sum(1 for r in rows if r['cls'] == c) for c in ORD},
           allow=[dict(i=i, a=x[0], b=x[1], kind=x[2], limit_mm=x[3], when=x[4], why=x[5], pairs=sum(1 for r in rows if r['rule'] == i)) for i, x in enumerate(ALLOW)],
           parts={p.name: dict(group=p.group, frame=p.frame, faces=int(len(p.F)), closed=p.stats['closed'], boundary=p.stats['boundary'], center_mm=(p.c * 1000).round(2).tolist()) for p in parts},
           pairs=rows)
rep['violation_groups'] = [dict(a=k[0], b=k[1], n=g['n'], crossings=g['x'], deepest_mm=round(g['pen'] * 1000, 3), ref=g['ref']) for k, g in GV.items()]
rep['tip_landing_gap_mm'] = {('%g' % k): round(v * 1000, 3) for k, v in LAND.items()}
fail = len(viol)
if BASEL:      # regression gate: only violation group pairs that are new, or deeper by > 0.02 mm, count
    old = {(g['a'], g['b']): g['deepest_mm'] for g in json.load(open(BASEL)).get('violation_groups', [])}
    reg = [g for g in rep['violation_groups'] if (g['a'], g['b']) not in old or g['deepest_mm'] > old[(g['a'], g['b'])] + 0.02]
    fixed = [k for k in old if k not in {(g['a'], g['b']) for g in rep['violation_groups']}]
    rep['baseline'] = dict(file=BASEL, regressions=reg, cleared=['%s|%s' % k for k in fixed]); fail = len(reg)
    print('--- against baseline %s: %d new / deeper violation group pairs %s; cleared: %s' % (BASEL, len(reg), ['%s|%s %.3f' % (g['a'], g['b'], g['deepest_mm']) for g in reg], rep['baseline']['cleared']))
if OUT: json.dump(rep, open(OUT, 'w'), indent=1, default=float); print('wrote', OUT)
if BOUT: json.dump({k: rep[k] for k in ('info', 'summary', 'n_violations', 'violation_groups', 'tip_landing_gap_mm')}, open(BOUT, 'w'), indent=1, default=float); print('wrote', BOUT)
print('tip landing gap on the real key (mm, per lid angle):', rep['tip_landing_gap_mm'])
print('INTERFERE %s: %d violations, %d render-only, %d allowed, %d near (< %.2f mm)' % ('OK' if not viol else 'FAIL', len(viol), rep['summary']['RENDER'], rep['summary']['ALLOWED'], rep['summary']['NEAR'], WARN * 1000), flush=True)
sys.exit(1 if fail else 0)
