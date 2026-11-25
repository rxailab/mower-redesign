# AIRAJ R1 site images

The AIRAJ site (`airaj-site/`) shows the ORIGINAL, unmodified mower (`mower_assembly.blend`). All images come from these scripts.

1. Product renders (Blender 4.2, Cycles), run from the repo root:

   ```bash
   B=~/blender-4.2.3-linux-x64/blender; R=/path/to/renders
   PALETTE_LIST=all $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/c hero 1800 64
   CUSTOM_VIEWS='wheel2=0.55:-0.95:0.3:0.12:-0.2:0.12:85;sensor2=0.95:-0.35:0.42:0.29:0.0:0.2:75;panel2=-0.55:-0.35:0.85:-0.06:0.0:0.24:70;stopr=-0.30:-0.20:0.52:0.0:0.0:0.25:95' \
     $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/k sensor2,wheel2,panel2,stopr,side 1800 64
   DOCK=1 $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/d dockhero 1800 64
   # slope: mower and shadow plane pitched 19 deg, soft overcast light
   MODE=sun SUN=200:60 SUN_E=1.6 SUN_ANGLE=30 SKY=2.4 FILL=50 CATCHER_PITCH=19 PITCH=slope=19 CUSTOM_VIEWS='slope=-0.15:-2.3:0.32:0.12:0.0:0.2:45' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/sl2 slope 2304 96
   MODE=sun MASK=wheels CATCHER_PITCH=19 PITCH=slope=19 CUSTOM_VIEWS='slope=-0.15:-2.3:0.32:0.12:0.0:0.2:45' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/slm slope 2304 8
   ```

2. Hero photo (`src/hero_generated.png`): generated with an image model (gpt-image-2, image edit, 1536 x 1024, quality high) with the six
   reference renders in `src/ref/` as input images, so the model draws this exact mower into a garden with one consistent perspective
   (a render pasted into a photo did not match the lawn's perspective). References: render_r1.py views hero, low34 (1.75:-1.15:0.42 ->
   0:0:0.12, 50 mm), side, front, rear34 (-1.35:-1.15:0.85, 60 mm), top3 (0.6:-0.9:2.2, 60 mm) on a light background, cropped to the mower.
   Prompt: photorealistic wide lifestyle photograph for a website hero banner: the robot lawn mower from the reference images mowing a large,
   freshly striped lawn in an English garden in the early evening; camera about 40 cm above the grass, 2.5 m away, 40 mm lens, looking
   slightly down; front three-quarter view, driving towards the lower left, in the right half of the frame; size and perspective consistent
   with the lawn (66 x 47 x 26 cm), wheels on the grass with blades over the tyre bottoms, soft shadow; hedges, a tree and a stone patio in
   soft focus, golden low sun from the back right; the left half quieter and darker for overlaid text; keep the mower's design exactly as in
   the references (each part named), no logos, text, people or other machines.

3. Slope composite (`src/grass_bank.png`, generated from the prompt below; the render pasted in with matching soft light):

   ```bash
   python scripts/airaj/composite.py scripts/airaj/src/grass_bank.png $R/sl2_slope.png slope.png 520 300 1.0,1.0,1.0 9 $R/slm_slope.png 0.62
   ```

4. Bake to WebP: `python scripts/airaj/bake.py $R scripts/airaj/src/hero_generated.png slope.png airaj-site/assets/img`
   (hero: 16:9 crop at 2048 / 1280 wide, and a 4:3 crop round the mower for phones, where the headline sits below the picture)

The logo (`airaj-site/assets/img/logo.svg`) is redrawn as vector paths from the AIRAJ logo image.

Prompt for the slope photo (1536 x 1024, quality high):
- grass_bank.png: photograph of a steep, freshly mowed grass bank in an English country garden on a bright overcast morning, camera low at
  the bottom of the slope looking up the bank (rising lower left to upper right at about 20 deg); a few daisies, a dry-stone wall and trees
  softly out of focus at the top; soft even daylight; no people, animals, mowers, robots, machines, tools, text, logos or watermarks.
