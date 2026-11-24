# [fix2] f2.json = f1.json + the fix-round-2 changes (finding IDs in the comments). usage: python mk_f2.py OUT.json [overrides.json ...]
import json, sys
P = json.load(open('f1.json'))
def merge(a, b):
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(a.get(k), dict): merge(a[k], v)
        else: a[k] = v
F2 = {
  # A1 / A6 / A7: one G2 superellipse shoulder, the same absolute corner (34 x 26.5, n 3.4) along the whole body, round the nose and the tail,
  #   for the pinched and the full-width surface; 2 deg draft (tumblehome) on the walls (R2E11: the flanks get positive draft for the pull)
  'section': {'a': 0.034, 'b': 0.0265, 'n': 3.4, 't1': 0.28, 't2': 0.52, 'draft_deg': 0.0},
  'section_wide': {'draft_deg': 1.5},      # R2E11: drafted haunch / nose / tail walls; beside the wheels the wall stays vertical (post 10, 171 envelope)
  'y_pinch': 0.1709,                                                 # the faired vertical wall beside the wheels overshot 171.0 by 0.01 (measured on the final skin)
  'crown': 0.003,                                                    # A7: flatter, tauter roof (was 0.006)
  'waist_tuck': {'amount': 0.2},                                     # A3: less haunch dome (was 0.35)
  'nose_rake': {'z1': 0.19}, 'tail_rake': {'z1': 0.19},             # G3 / A8: back to the brow (the forehead no longer leans 25-33 deg / turns concave)
  'cover': {'orbit': {'explicit': True,                              # G4: explicit side-view part line, every corner a true R20 circle
                      'tail': {'mode': 'sweep', 'x_top': -0.262, 'R': 0.09, 'fillet': 0.02},     # G2 / A3 / R2E2: study f2a (R128 tail orbit) vs f2b (R90 sweep): f2b
                      'zone_ny': [0.15, 0.995], 'zone_x_ramp': 0.08},                                         # A1 (2) / F-S1: spread the zone sink
            'kit2': True, 'deck_cell': 0.0008, 'deck_edge': 0.0005, 'deck_notch': [[0.103, 0.1535, 0.144, 0.0062]], 'legs': {'t_neck': 0.0015}, 'datum': {'slot': 0.009, 'wall': 0.001}},                     # R2E1 / R2E5 / R2E6 / R2E8 / R2E3 / F-S5
  'lid': {'stop_pad': {'x': 0.018, 'r': 0.009, 'icon': 0.0055, 'icon_r': 0.001, 'halo_gap': 0.0015, 'halo_w': 0.0012}, 'stop_mark': None,    # A2 / G5 / R2E4
          'stakes': {'pin_r': 0.00075},                                                                                       # R2E8 / F-N1
          'rib_layout': 'v2', 'ring_off': [0.03, 0.05]},                                                                     # G9: two closed rings, mirrored ribs
  'nose': {'corner_floor': {'v2': True, 'z_top': 0.1135, 't': 0.0025, 'clear': 0.0015, 'hull_in': 0.001, 'under': 0.002, 'roll': 0.0005}},   # F-M1 / R2E7 / G8 / A9
  'controls_standin': {'symmetric': True, 'glass_r': 0.004, 'key_r': 0.004, 'plunger_gap': 0.0005},                     # G7 / R2E4
  'skirt': {'flare': [[-0.4, -0.003], [0.4, -0.003]]},          # A4: the lower rear half tucks in 3 mm at the hem (was flared out 3 mm): the lower tail face turns into shadow
  'port_slot': {'y': 0.060, 'z0': 0.1315, 'z1': 0.1605, 'web': 0.003},                                                 # A5 / L7
}
merge(P, F2)
P['rear_recess'] = []                                                # G1 / R2E9 / A4: the three decorative rear grooves are deleted
P['panels'] = []                                                     # A2 / G5: the roof status stroke is gone; the status light is the halo round the STOP pad
for f in sys.argv[2:]: merge(P, json.load(open(f)))
json.dump(P, open(sys.argv[1], 'w'), indent=None); print('wrote', sys.argv[1])
