// [fix3-struts] Reference implementation of struts_cut.json for the web viewer / exporter (plain arrays, no three.js needed).
// Replaces trimChassis() (viewer/app.js), which deleted every triangle touching |z| > 0.160, y > 0.135, -0.06 < x < 0.29 (glTF axes).
// cutChassis(pos, idx, spec, frame) -> {position: Float32Array, index: Uint32Array, report}
//   pos: Float32Array of WORLD positions (apply the mesh's matrixWorld first), idx: triangle indices, spec: parsed struts_cut.json,
//   frame: 'gltf' (viewer: x, z_up, -y) or 'world' (+X fwd, +Y left, Z up).
// Per cut, in order: triangles inside the box are dropped, triangles crossing the cut plane near the box are clipped to the kept side,
// then the cap triangles (flat, already trimmed by the other boxes) are appended. Use the chassis material for the caps.
// three.js usage:  const g = mesh.geometry; mesh.updateWorldMatrix(true, false); const p = g.attributes.position.clone().applyMatrix4(mesh.matrixWorld);
//   const r = cutChassis(p.array, g.index.array, spec); g.setAttribute('position', new THREE.BufferAttribute(r.position.map(...inverse...), 3)); ...
//   (or, simplest, bake: mesh.geometry = new geometry from r with mesh.matrix reset to identity), then g.computeVertexNormals() for the new faces.
export function cutChassis(pos, idx, spec, frame = 'gltf') {
  const P = Array.from(pos), out = [], rep = {};
  let tris = []; for (let i = 0; i < idx.length; i += 3) tris.push([idx[i], idx[i + 1], idx[i + 2]]);
  const pt = (k) => [P[3 * k], P[3 * k + 1], P[3 * k + 2]];
  const add = (q) => { P.push(q[0], q[1], q[2]); return P.length / 3 - 1; };
  for (const c of spec.cuts) {
    const b0 = c[frame].box_min, b1 = c[frame].box_max, p0 = c[frame].plane_point, n = c[frame].plane_normal;
    const inb = (q, e) => q[0] > b0[0] - e && q[0] < b1[0] + e && q[1] > b0[1] - e && q[1] < b1[1] + e && q[2] > b0[2] - e && q[2] < b1[2] + e;
    const sd = (q) => (q[0] - p0[0]) * n[0] + (q[1] - p0[1]) * n[1] + (q[2] - p0[2]) * n[2];
    const next = []; let dropped = 0, clipped = 0;
    for (const t of tris) {
      const Q = t.map(pt);
      if (!Q.some((q) => inb(q, 5e-4))) { next.push(t); continue; }
      if (Q.every((q) => inb(q, 1e-6))) { dropped++; continue; }                 // wholly inside the box (removed side)
      const s = Q.map(sd);
      if (s.every((v) => v <= 1e-9)) { next.push(t); continue; }                // kept side
      if (!Q.some((q) => inb(q, 1e-6))) { next.push(t); continue; }             // only near the box, not in it: untouched
      if (s.every((v) => v >= -1e-9)) {                                         // removed side, partly outside the box rectangle: keep (air)
        if (Q.some((q) => inb(q, -1e-6))) { dropped++; continue; } next.push(t); continue; }
      const poly = [];                                                          // clip to s <= 0 (Sutherland-Hodgman on one triangle)
      for (let i = 0; i < 3; i++) {
        const j = (i + 1) % 3;
        if (s[i] <= 0) poly.push(t[i]);
        if ((s[i] < 0 && s[j] > 0) || (s[i] > 0 && s[j] < 0)) { const u = s[i] / (s[i] - s[j]); poly.push(add(Q[i].map((v, k) => v + (Q[j][k] - v) * u))); }
      }
      for (let k = 1; k + 1 < poly.length; k++) next.push([poly[0], poly[k], poly[k + 1]]);
      clipped++;
    }
    tris = next;
    const cp = c.cap.gltf_positions, ci = c.cap.indices, base = P.length / 3;
    for (let i = 0; i < cp.length; i += 3) {
      const g = [cp[i], cp[i + 1], cp[i + 2]]; P.push(...(frame === 'gltf' ? g : [g[0], -g[2], g[1]]));
    }
    for (let i = 0; i < ci.length; i += 3) tris.push([base + ci[i], base + ci[i + 1], base + ci[i + 2]]);
    rep[c.id] = { dropped, clipped, cap_triangles: ci.length / 3 };
  }
  return { position: Float32Array.from(P), index: Uint32Array.from(tris.flat()), report: rep };
}
