# Concept parameters (JSON). All lengths in metres. World frame: +X forward, +Y left, Z up, ground z=0.
Fixed and NOT editable: chassis, wheels (x 0.003..0.235, |y| 0.175..0.23, top z 0.229), casters, the dark lower skirt, bumper
and front face with the two charger openings (below z~0.135 rear/sides, ~0.16 at the front). The new upper body starts at that belt.
Controls (orange STOP + display) stay at their original place: x -0.128..0.034, |y| < 0.062, top at z 0.2617. The lid must cover them.

- roof: [[x, z], ...] roof height along the centre line (PCHIP). Must stay >= 0.268 over x -0.13..0.04 and clear the chassis
  (check output). Default [[-0.32,0.262],[-0.18,0.272],[0,0.274],[0.2,0.258],[0.31,0.225]].
- crown: sideways drop of the roof (m at |y|=0.2), default 0.006.
- profile: [[t, q, p], ...] wall-to-roof section from belt (t=0) to centre line (t=1): q = how far inward in plan (0..1),
  p = how far up towards the roof (0..1). Default [[0,0,0],[0.22,0.05,0.55],[0.4,0.3,0.93],[1,1,1]].
  Steep p with small q = upright wall; a knot pair close together = a shoulder line.
- profile_mode: "smooth" (curved surfaces) or "linear" (flat facets with hard creases = faceted / low-poly look).
- smooth_deg: shading crease angle (default 32; lower = more visible hard edges).
- spine_scale (0.3..0.8, default 0.55), spine_center (default -0.02): how long the roof ridge is.
- y_pinch (<= 0.168, default 0.166): half-width of the body beside the wheels (its lower edge is the belt line everywhere).
- roof_smooth (default 60), smooth_from (default 0.12): how fast the step behind/in front of the wheels fades out upwards.
- roof_panel_t: faces with t above this get the second colour "roof" (default off). 
- bands: [{kind: "visor"|"light"|"accent"|"roof"|"body", t0, t1, u0, u1, x_min, x_max}] strips on the surface.
  u runs along the belt outline of each half: 0 = nose, ~0.12 = front corner, 0.15..0.62 = beside the wheels, 1 = tail.
  Example front light bar: {"kind":"light","t0":0.10,"t1":0.13,"u0":0,"u1":0.11}; sensor visor: {"kind":"visor","t0":0.14,"t1":0.26,"u0":0,"u1":0.09}.
- lid: {shape: "rrect"|"hex"|"trapezoid"|"shield"|"ellipse", cx, length, width, r, cut, width_front}. It is a region of the roof
  surface itself, so it is always flush; a 1.4 mm dark seam marks it. Must cover the controls (check output lid_covers_controls).
- lines: [{kind: "light"|"accent", pts: [[x,y],...], width, mirror}] thin strips drawn on the roof (plan coordinates).
- colors: {body, body_finish, roof, roof_finish, lid, lid_finish, skirt, accent, light, ring} hex colours; finishes gloss|metallic|satin|matte.
- n_outline (default 420), n_rows (default 110): mesh resolution.

Run:  ./run_concept.sh NAME            (reads NAME.json; prints the check JSON; writes NAME_sheet.png with 6 views: hero, face (straight-on front), face34 (front three-quarter close-up), rear, side, lid open)
The check JSON must have "ok": true. Look at NAME_sheet.png (and single views NAME_hero.png etc.) with the Read tool.

# Version 3: continuous "car body" surfacing (no visible edges)
The whole surface is now faired (Taubin smoothing, belt edge fixed), so hard creases cannot survive; use these to shape it:
- fair (default 60): fairing iterations. More = softer, larger radii everywhere (try 60..250).
- profile_mode must stay "smooth". Avoid knots that sit close together (they make tight radii).
- Waist behind / ahead of the exposed wheels (the body is a blend of a full-width surface and a narrow one, along raked S-curves):
  rear_rake (0.45), rear_blend (0.03), rear_sweep (0.9), front_rake (0.25), front_blend (0.02), front_sweep (0.5).
  Bigger blend/sweep = longer, gentler transition; rake slants it rearwards (rear) / forwards (front) with height.
- sculpt: [{x, t, sx, st, amp, rake, lift}] soft Gaussian bulges (amp > 0, metres, e.g. 0.006 for a haunch over the rear)
  or hollows (amp < 0, e.g. a light-catching scoop along the flank). x = centre along the body, t = where on the wall/roof
  section (0 belt .. 1 centre line), sx/st = sizes, rake = slant, lift = how much it also raises the surface (ratio).
- smooth_deg now defaults to 75 (no shaded hard edges). Default mesh is finer (n_outline 640, n_rows 170); leave it.
New check outputs (all must pass for "ok": true):
- min_surface_radius_mm >= 8 : tightest radius anywhere on the body above the belt (99.9th percentile). Aim for 12+.
- max_crease_deg: informational, largest angle between neighbouring mesh quads.
- chassis_exposed_cm2 <= 2 : chassis frame beside the wheels reaches z = 0.236 at |y| 0.148..0.158; the wall beside the wheels
  must stay upright (little inward lean, small q) until above z ~0.24 or the black frame shows next to the wheels.
- max_half_width_beside_wheels_mm <= 171 : now measured around the real wheel circle (centre x 0.119, z 0.1145, r 0.116) and above it.

# Version 4: the front face is now fully part of the new body
The new skin reaches down to the top of the black bumper at the front (z ~0.123); the old flat scanned front face is gone.
The two charger openings are cut into the new fascia automatically at their fixed place (y -52..-9 mm and 0..44 mm, z 134..158 mm);
they must stay clear: do not place lamps over them. port_margin (default 0.0015) widens the cut-outs.
- front_shapes: [{kind: "light"|"visor"|"accent"|"roof"|"body", shape: "rrect"|"ellipse", cy, cz, w, h, r, tilt, mirror}] or
  {kind, pts: [[y, z], ...], r, mirror}: graphics drawn in the FRONT VIEW (y = left/right, z = height, metres) and projected onto
  the nose. mirror (default true) repeats the shape at -y; use mirror false with cy 0 for a centred element. tilt in degrees.
  Usable front area: |y| < ~0.20, z from ~0.125 (bumper top) up to where the nose turns into the hood (~0.20-0.23, see renders).
  Typical cars: slim DRL blades high at the outer corners, a calm dark sensor band or "grille" graphic in the middle (the mower
  needs a camera window: a visor about 50 x 24 mm around y 0, z ~0.19..0.22), ports low in the centre, a lower intake graphic.
- front_x_min (default 0.17): how far back front_shapes may wrap around the corners.
- The nose shape itself comes from roof (front knots), profile (the first knots: wall lean and height), spine_scale/spine_center
  and sculpt items placed at x ~0.25..0.31. A rounder, more car-like nose: let the roof start falling earlier and keep q small low down.

# Version 5: crisp front graphics and port sleeves
front_shapes are now rendered as separate finely triangulated decals lying on the nose, so their outlines are perfectly smooth
(no stair-stepping) and later shapes sit on top of earlier ones. Shapes are automatically cut clear of the charger openings.
The two charger openings get dark sleeves running back into the body (no internals visible). "bands" are still mesh-based and
can look jagged: prefer front_shapes for anything on the nose. crisp_front (default true).

# Version 6: character lines over the top ("not a bar of soap")
- ridges: [{kind: "step"|"crease", pts: [[x, y], ...], amp, width, fade}] soft lines running front to rear (plan coordinates of
  the left half, mirrored automatically). "step": the area inboard of the line is raised by amp (m), giving a sculpted centre
  dome / power bulge whose edge reads as a line; "crease": a soft raised ridge along the line. width (default 0.012) = softness,
  fade (default 0.05) = how the line fades in/out at its ends. Typical amp 0.003..0.007. ridge_fair (default 3) rounds them.
  With ridges the radius limit is 3 mm (lines are meant to read), without ridges it stays 8 mm.

# Version 7: flat deck + slim tail
- ridges kind "deck": {kind: "deck", pts: [[x, y], ...], flat: 0..1, width}: inboard of the line the roof is lifted towards the
  centre-line height (flat = how much of the cross crown is removed), giving a calm top that turns into the sides along a
  shoulder line running nose to tail. Subtle and well placed; preferred over "step"/"crease".
- tail: {"width": [[x, half_width], ...]} narrows the wide rear of the black lower skirt behind the wheels (scanned 0.236) and the
  body's belt with it (tail.py, used by generator and renderer). The chassis is 0.190 wide at x = -0.28: keep half_width >= 0.198 there.

# Version 8: faceted bodies with a top panel (two-tone "tech" look)
- panels: [{pts: [[x, y], ...] (left edge in plan, front to rear; mirrored), r, raise (m, + up / - sunk), chamfer (m, width of the
  flat bevel), kind: "roof"|"visor"|..., t_min, edges: [{kind: "light"|"accent"|"roof", width, off, x0, x1}]}]: a plan polygon on the
  top that is raised/sunk with a chamfer and painted in the second tone; edges draw strips along its outline (x0..x1 = which part).
- min_radius: radius limit for the check (crisp faceted concepts: 1.5). profile_mode "linear" + fair 6..20 + smooth_deg 20..30
  gives flat facets with small rounded chamfers. panel_fair (default 0) softens panel chamfers.
- lines kind may also be "roof" (second-tone strip).

# Version 9: narrower nose and waist, layered details
- nose: {"width": [[x, half_width], ...], "tiers": [{"inset", "height"}]}: the body no longer spans the whole bumper at the front
  (x >= 0.262). The chassis needs half_width >= 0.16 @ x 0.27, 0.14 @ 0.28, 0.12 @ 0.29, 0.07 @ 0.30. A dark plinth (flat cap at the
  skirt top + optional stepped tiers) covers what the body leaves open. Keep front_shapes within the narrower face (|y| < ~0.13).
- waist: same for x <= -0.03 (behind the wheels and the tail). Chassis above the belt: 0.156 wide for x -0.22..0, 0.174 at x -0.25..-0.30.
- panels may use "poly": [[x, y], ...] (closed polygon on the left side, mirrored) instead of pts: rails, pads, vents.
- nose_profile: [[z, dx], ...] shears the front face in x by height (chin forwards, brow back) so the nose is raked in side view.
  Chassis limit: at z 0.185 the face must stay at x >= 0.307 (|y| < 0.05); the bumper reaches x 0.331 below z 0.115.
- belt_drop_front (0.0035), nose.cap_z (0.1172), nose.tiers []: the nose sits flush on the bumper top (no overlap).
- front_recess: [{pts: [[y, z], ...] (front view, symmetric), r, depth, chamfer, kind}] a real pocket pressed into the front face
  (bevelled walls, painted "visor" black by default, with a crisp rim line). Depth <= ~0.008 (chassis behind the face).

# Version 10: exact edges, real camera, waist line, lid mechanism (design_gen.py + meshkit.py)
After fairing, the body becomes an unstructured mesh and every feature edge is cut into it along the iso-line of a distance
field (split / paint / cham / refine in design_gen.py). Outlines no longer depend on the grid. Shading normals are computed
in the generator ("ln" in the npz) and set as custom normals by the renderer. Decals, ribbons, seam strip and tray are gone.
- panels: + fillet (m), paint "outer"|"inner" (which chamfer edge the colour stops at), groove {width, kind}, fine (max edge
  length for local refinement, for small radii). edges: {kind, width, off, x0, taper0, z1, taper1} pin stripes with pointed ends.
  The first panel's t_min should stay above the nose face (0.33) so its cut lines do not run down to the slot.
- bib {z_low, r, tab, tab_margin, tab_fillet}: outline of the first panel on the front face, drawn in the front view; "tab" =
  index of the front_recess (camera window) that hangs below it as one outline.
- front_recess: + cy (number or "ports" = centre of the charger openings, -4.3 mm), fillet, fine, kind "none".
  flat windows should have depth >= 0.0008 so the top wall is not degenerate.
- port_bezel {width, raise, kind}, port_margin, port_back_x (flat back wall of the openings, hides the chassis contacts).
- front_shapes / rear_shapes: painted exactly (lamps, tail lamp). dy "ports" shifts a shape with the charger openings.
- camera {recess, cz, r, proud, ir, ir_dy, led}: real lens (bezel, barrel, glass dome) + IR sensor + status light.
- rocker {x_low, x_top, h, r, line, kind}: dark band round the tail with the accent line on top; behind each wheel the band ends
  on a raked edge (from x_top at height h down to x_low at the belt) and the line follows it down to the body's lower edge.
- lid {.., gap, open_deg, hinge_y, floor_z}: lid is its own solid (2 mm rim, 6 mm liner, rib frame). Well with flange, ledge and
  seal; two gooseneck hinges (axis 26 mm behind the opening, 8.5 mm under the roof), two gas struts, push latch + striker.
  npz: kitm_* (moves with the lid), kitf_* (fixed), strut_n / strut_m (ball studs), lid_pivot, lid_open.
- renderer: views railclose, camclose, camside, waistclose, bibclose, tailclose, topclose, lidin, lidhinge, lidunder, lidside;
  env CUSTOM_VIEWS="name=x:y:z:tx:ty:tz:lens;..." adds views, HIDE=Obj1,Obj2 hides objects, NODENOISE=1.

# Version 11: one-piece body (skirt integrated), fastening to the chassis, raked tail, lid without gas struts
The scanned lower skirt is gone: the body runs from the roof down to its own hem as ONE surface. Only the front bumper is kept
from keep.npz (the renderer drops the skirt when the parameter file has "skirt"). No waist plinth any more.
- tail_plan {x_tip, x_c, w, n, width [[x, half_width], ...]}: plan outline behind the wheels. From x_c back to x_tip a superellipse
  (exponent n: 2 = ellipse, 3.3 = full corners; the chassis apron needs n >= ~3 with w 0.204), ahead of x_c the width curve.
- skirt {n, depth, flare [[x, m], ...], bulge, hem [[|y|, z], ...], hem_by "y"|"x", arch_r, lip}: rows below the belt (t < 0).
  hem = height of the lower edge by |y|: low (0.05) along the sides and round the corners, raised in the middle of the tail,
  where the finned chassis apron shows as a diffuser. Beside the wheels and at the nose the lower edge is the belt as before;
  behind each wheel it follows an arc (radius arch_r about the wheel centre). lip = return flange along the edge.
- tail_rake {z0, z1, dx, x_a, x_b}: above z0 the rear face leans forwards (dx at z1 and above); fades out along the sides.
- roof: the knots behind x = -0.22 now fall to the tail (fastback); the shoulder line drops round the corners with them.
- y_pinch 0.171 (was 0.168): the chassis screw posts beside the wheels need the room (see check below).
- rocker {pts [[x, height above belt], ...], r, line, kind}: free upper edge of the dark valance (negative heights = on the skirt).
- rear_recess: like front_recess, drawn in the rear view (light-bar slot). rear_shapes as before (light bar).
- lid.struts (default false): no gas struts; each hinge pin carries a friction cartridge (holds the lid at any angle).
- mounts {well_r, well_chamfer, seat, tower_r, hole_r, fine}: fastening to the chassis. Positions are NOT parameters: they are
  measured on the chassis mesh by mounts_from_chassis.py -> mounts.json:
    posts (14) screw posts round the tub rim, hole open at the top -> screws from above. Each gets a well through the body
               (chamfered rim, seat `seat` above the post, screw hole) and inside a tower with ribs standing on the post.
    under (5)  through holes in the front wings (3) and the rear corner sockets (2) -> screws from below into bosses of the body
               (front: long ribbed bosses from the skin; rear: boss in the socket on a bracket from the skirt wall).
    clips (5)  windows in the rear apron -> snap hooks on the inside of the tail wall.
  npz: mount_v / mount_f / mount_lab (0 inner plastic, 1 dark, 2 steel, 3 screw). Renderer object "Mount".
- Checks: "mounts" in the check JSON (all posts / holes / clips served, shallowest well >= 2 mm). check_fit.py SHELL.npz
  CHASSIS_FACE_CENTRES.npy prints the gap between the tail skin and the chassis per height.
- Renderer: the body is a single skin; its inside is shaded as dark plastic. env LIFT=0.15 raises the whole body (exploded view),
  HEADLIGHT=energy adds a light at the camera (views from below), HIDE=Backdrop,Chassis,... as before. run_one.sh renders all.

# Version 12: snap-on top cover that hides the screws
- cover {t, z_nose, gap, x_n0, x_n1, blend, wall, inset, round, lip, opening_margin, rings, cell, smooth}: everything above a seam
  is a separate part (npz: per-face "part", 1 = cover; the lid is cut out of it as before). The seam runs round the body just
  under the shoulder: along the sides and the tail it is the line t = `t` (0.18, about 10 mm under the shoulder edge, falling
  with it towards the tail), across the tail it is level at `z_tail` (0.1745, just above the light bar), round the nose it is
  the height `z_nose` (0.191, between the camera window and the screw behind it). gap = width of the joint.
  With a cover there are no wells in the outer surface. Under the cover the body carries a deck: a ledge at the height of the
  screw seats (through the post tops, left and right separately), lowered with a rounded transition wherever the cover comes
  closer than `inset`. The 14 screws from above sit on it (pad underneath on each post; a thinner seat and a flat-head screw
  where the cover is close: nose centre). The middle is open (controls, chassis). The cover has a return flange along its
  edge and one snap leg between each pair of screws (through a slot in the deck); legs are left out where there is no height.
  The bosses for the screws from below hang from the deck where it is above them.
  npz: ckit_v / ckit_f / ckit_lab (cover flange + legs). Renderer objects "TopCover", "CoverKit"; env LIFT2 raises the cover
  (LIFT raises the body), e.g. LIFT=0.2 LIFT2=0.42 for a three-level exploded view. The controls stay on the chassis.
- Checks: "mounts.cover" in the check JSON (hidden_screws, snap_legs, min_seat_mm, min_room_over_screw_head_mm,
  min_post_to_deck_edge_mm). check_cover.py SHELL.npz tests that no hidden part reaches through either skin.
- The rails are back at 18 mm (the wells that made them narrower are gone); the light-bar slot sits 5 mm lower, clear of the seam.
- run_cov.sh renders everything; mount_map.py ... cover draws the map on the plan view with the cover taken off.
