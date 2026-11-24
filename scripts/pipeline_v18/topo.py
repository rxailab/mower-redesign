# [fix1] final-npz topology per moulding (lid excluded): boundary / non-manifold edges, plus coincident-vertex count (float32)
import numpy as np, sys, json
d = np.load(sys.argv[1]); F = d['f'].astype(np.int64); part = d['part']; lab = d['lab']; V = d['v']; out = {}
for pa in (0, 1):
    m = (part == pa) & (lab != 4); Fs = F[m]; e = np.concatenate([Fs[:, [0, 1]], Fs[:, [1, 2]], Fs[:, [2, 0]]])
    key = np.minimum(e[:, 0], e[:, 1]) * len(V) + np.maximum(e[:, 0], e[:, 1]); _, c = np.unique(key, return_counts=True)
    # directed check: every undirected edge used exactly twice, once in each direction
    dk = e[:, 0] * len(V) + e[:, 1]; rk = e[:, 1] * len(V) + e[:, 0]; same = int((~np.isin(rk, dk)).sum())
    out[pa] = dict(faces=int(m.sum()), boundary_edges=int((c == 1).sum()), nonmanifold_edges=int((c > 2).sum()), unpaired_directed=same)
print(json.dumps(out))
