# AIRAJ R1 site images

The AIRAJ site (`airaj-site/`) shows the ORIGINAL, unmodified mower (`mower_assembly.blend`). All images come from these scripts.

1. Product renders (Blender 4.2, Cycles), run from the repo root:

   ```bash
   B=~/blender-4.2.3-linux-x64/blender; R=/path/to/renders
   PALETTE_LIST=all $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/c hero 1800 64
   CUSTOM_VIEWS='wheel2=0.55:-0.95:0.3:0.12:-0.2:0.12:85;sensor2=0.95:-0.35:0.42:0.29:0.0:0.2:75;panel2=-0.55:-0.35:0.85:-0.06:0.0:0.24:70;stopr=-0.30:-0.20:0.52:0.0:0.0:0.25:95' \
     $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/k sensor2,wheel2,panel2,stopr,side 1800 64
   DOCK=1 $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/d dockhero 1800 64
   # garden hero: low warm sun from behind right, soft fill from the camera side; wheel mask for the grass in front of the tyres
   MODE=sun SUN=105:15 SUN_E=5 SKY=1.3 FILL=60 CUSTOM_VIEWS='lawnhero=1.69:-1.41:0.45:0.0:0.0:0.10:32' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/sun2 lawnhero 2304 128
   MODE=sun MASK=wheels CUSTOM_VIEWS='lawnhero=1.69:-1.41:0.45:0.0:0.0:0.10:32' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/wm lawnhero 2304 8
   # slope: mower and shadow plane pitched 19 deg, soft overcast light
   MODE=sun SUN=200:60 SUN_E=1.6 SUN_ANGLE=30 SKY=2.4 FILL=50 CATCHER_PITCH=19 PITCH=slope=19 CUSTOM_VIEWS='slope=-0.15:-2.3:0.32:0.12:0.0:0.2:45' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/sl2 slope 2304 96
   MODE=sun MASK=wheels CATCHER_PITCH=19 PITCH=slope=19 CUSTOM_VIEWS='slope=-0.15:-2.3:0.32:0.12:0.0:0.2:45' $B -b mower_assembly.blend -P scripts/airaj/render_r1.py -- $R/slm slope 2304 8
   ```

2. Composites into the garden photos (`src/`, generated with an image model, gpt-image-2, from the prompts below):

   ```bash
   python scripts/airaj/composite.py scripts/airaj/src/garden_sunset.png $R/sun2_lawnhero.png hero.png 560 210 1.05,1.0,0.9 15 $R/wm_lawnhero.png 0.94
   python scripts/airaj/composite.py scripts/airaj/src/grass_bank.png $R/sl2_slope.png slope.png 520 300 1.0,1.0,1.0 9 $R/slm_slope.png 0.62
   ```

3. Bake to WebP: `python scripts/airaj/bake.py $R hero.png slope.png airaj-site/assets/img`

The logo (`airaj-site/assets/img/logo.svg`) is redrawn as vector paths from the AIRAJ logo image.

Prompts (1536 x 1024, quality high):
- garden_sunset.png: photograph of an empty, freshly mowed lawn in a modern British back garden at golden hour, camera at knee height
  (about 45 cm) looking slightly down; faint mowing stripes; empty lawn in the centre-left foreground; warm low sun from the back right,
  long soft shadows; contemporary house with glass doors, stone patio, clipped hedges and flower borders softly out of focus; no people,
  animals, mowers, robots, machines, tools, text, logos or watermarks.
- grass_bank.png: photograph of a steep, freshly mowed grass bank in an English country garden on a bright overcast morning, camera low at
  the bottom of the slope looking up the bank (rising lower left to upper right at about 20 deg); a few daisies, a dry-stone wall and trees
  softly out of focus at the top; soft even daylight; same exclusions.
