# claude.ai pages only serve text: ship each .glb as base64 text chunks + models/manifest.json
import base64, json, os, shutil, sys, glob
src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
for f in glob.glob(os.path.join(out, '*.b64.*.txt')): os.remove(f)
man = {}; CH = 8_000_000
for k in ('body', 'cover', 'lid', 'lower', 'panel', 'base', 'dock'):
    b = base64.b64encode(open(os.path.join(src, f'{k}.glb'), 'rb').read()).decode()
    names = []
    for i in range(0, len(b), CH):
        nm = f'{k}.glb.b64.{i // CH}.txt'; open(os.path.join(out, nm), 'w').write(b[i:i + CH]); names.append('models/' + nm)
    man[k] = {'size': len(b), 'chunks': names}
json.dump(man, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
for j in ('hotspots.json', 'palettes.json', 'dock_profile.json', 'design.json'): shutil.copy(os.path.join(src, j), os.path.join(out, j))
print(json.dumps({k: (v['size'], len(v['chunks'])) for k, v in man.items()}))
