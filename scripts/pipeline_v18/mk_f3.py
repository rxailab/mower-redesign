# [fix3] f3.json = f2.json + the fix-round-3 overrides (one file per open issue: fix3_<key>.json). usage: python mk_f3.py OUT.json [overrides.json ...]
# web variant: python mk_f3.py f3_web.json fix3_*.json web.json
import json, sys
P = json.load(open('f2.json'))
def merge(a, b):
    for k, v in b.items():
        if k.startswith('//'): continue                      # comment keys
        if isinstance(v, dict) and isinstance(a.get(k), dict): merge(a[k], v)
        else: a[k] = v
for f in sys.argv[2:]: merge(P, json.load(open(f)))
json.dump(P, open(sys.argv[1], 'w'), indent=None); print('wrote', sys.argv[1])
