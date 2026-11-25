// 割草机 3D 查看器 — three.js r160
// Coordinates (glTF, metres): +X = front of the mower, +Y = up, -Z = the mower's left side.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { cutChassis } from './chassis_cut.js';

window.__mowerStarted = true;           // tells the boot watchdog in index.html that the module loaded
const $ = (id) => document.getElementById(id);
const canvas = $('c');
const panel = $('panel');
const mqMobile = window.matchMedia('(max-width: 760px)');

function fail(html) {
  $('barFill').classList.add('error');
  $('loaderText').innerHTML = html;
}
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
// inside a sandboxed frame (e.g. a shared web page) the browser blocks downloads: hide the screenshot button there
if (window.top !== window.self) $('shot').hidden = true;

// ---------------------------------------------------------------- renderer / scene
let renderer;
try {
  renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: 'high-performance' });
} catch (err) {
  fail('无法创建 WebGL 三维画面。<br>请使用最新版 Chrome / Edge / Firefox / Safari，并在浏览器设置中开启“硬件加速”。');
  throw err;
}
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 0.86;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
const buildEnv = () => { scene.environment = pmrem.fromScene(new RoomEnvironment(renderer), 0.04).texture; };
buildEnv();

const camera = new THREE.PerspectiveCamera(32, 1, 0.02, 40);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 0.3;
controls.maxDistance = 4.5;
controls.maxPolarAngle = Math.PI * 0.495;
controls.autoRotateSpeed = 1.2;

const VIEWS = {
  iso:   { cam: [1.08, 0.74, 1.0], target: [0, 0.11, 0] },
  front: { cam: [1.55, 0.38, 0.0], target: [0, 0.12, 0] },
  rear:  { cam: [-1.55, 0.42, 0.0], target: [0, 0.12, 0] },
  left:  { cam: [0.0, 0.34, -1.6], target: [0, 0.12, 0] },
  right: { cam: [0.0, 0.34, 1.6], target: [0, 0.12, 0] },
  top:   { cam: [-0.0015, 1.75, 0.0], target: [0, 0.1, 0] },     // front of the mower at the top of the screen
};

// narrow / portrait screens need the camera further away to keep the whole mower in frame
// frame the model in the area the side panel leaves free (portrait tablets lose ~40% of the width to it)
function freeAspect() {
  const w = canvas.clientWidth || 1, h = canvas.clientHeight || 1, pn = document.getElementById('panel');
  if (pn && !pn.classList.contains('collapsed') && !mqMobile.matches) {
    const left = pn.getBoundingClientRect().left;
    if (left > 0 && left < w) return Math.max(200, left - 8) / h;
  }
  return w / h;
}
let dockShown = true;                    // the charging base is larger than the mower: frame a little wider
const fitScale = () => (dockShown ? 1.14 : 1) * Math.max(1, 1.0 / Math.max(freeAspect(), 0.3));
const vec = (a) => new THREE.Vector3().fromArray(a);
function fitted(cam, target) {
  const t = vec(target);
  return vec(cam).sub(t).multiplyScalar(fitScale()).add(t);
}
camera.aspect = (canvas.clientWidth || 1) / (canvas.clientHeight || 1);
let lastFit = fitScale();
camera.position.copy(fitted(VIEWS.iso.cam, VIEWS.iso.target));
controls.target.fromArray(VIEWS.iso.target);
controls.maxDistance = 4.5 * lastFit;

// lights
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f96, 0.35));
const key = new THREE.DirectionalLight(0xffffff, 1.7);
key.position.set(0.9, 2.4, 1.1);
key.castShadow = true;
key.shadow.mapSize.set(2048, 2048);
Object.assign(key.shadow.camera, { left: -1.0, right: 1.0, top: 1.0, bottom: -1.0, near: 0.5, far: 5 });
key.shadow.bias = -0.0004;
key.shadow.normalBias = 0.02;
scene.add(key, key.target);
const rim = new THREE.DirectionalLight(0xdfe8ff, 0.5);
rim.position.set(-1.5, 1.2, -1.2);
scene.add(rim);

// ground: shadow catcher + soft contact blob
const ground = new THREE.Mesh(new THREE.PlaneGeometry(12, 12), new THREE.ShadowMaterial({ opacity: 0.2 }));
ground.rotation.x = -Math.PI / 2;
ground.receiveShadow = true;
scene.add(ground);
function blobTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 256;
  const g = c.getContext('2d'); const gr = g.createRadialGradient(128, 128, 10, 128, 128, 128);
  gr.addColorStop(0, 'rgba(0,0,0,0.55)'); gr.addColorStop(0.55, 'rgba(0,0,0,0.25)'); gr.addColorStop(1, 'rgba(0,0,0,0)');
  g.fillStyle = gr; g.fillRect(0, 0, 256, 256);
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; return t;
}
const blob = new THREE.Mesh(new THREE.PlaneGeometry(0.95, 0.72),
  new THREE.MeshBasicMaterial({ map: blobTexture(), transparent: true, depthWrite: false, opacity: 0.6 }));
blob.rotation.x = -Math.PI / 2; blob.position.y = 0.0006; blob.renderOrder = -1;
scene.add(blob);

const groups = { lid: new THREE.Group(), cover: new THREE.Group(), body: new THREE.Group(), lower: new THREE.Group(), base: new THREE.Group(), dock: new THREE.Group() };
const SHELLS = ['lid', 'cover', 'body', 'lower'];      // everything that covers the chassis
// the mower (lid, top cover, one-piece body, front bumper, chassis) moves as one unit: it sits on the charging base and can drive off it
const mower = new THREE.Group(); mower.name = 'Mower';
mower.add(groups.lid, groups.cover, groups.body, groups.lower, groups.base);
scene.add(mower, groups.dock);

// ---------------------------------------------------------------- render scheduling
let dirty = true, occlusionDirty = true, lastMotion = 0;
const requestRender = () => { dirty = true; occlusionDirty = true; };
controls.addEventListener('change', requestRender);
controls.addEventListener('start', () => clearActiveView());

function applyViewOffset() {
  // shift the projection so the model is centred in the area not covered by the control panel
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  const open = !panel.classList.contains('collapsed');
  const r = panel.getBoundingClientRect();
  if (!open) camera.clearViewOffset();
  else if (mqMobile.matches) camera.setViewOffset(w, h, 0, Math.max(0, h - r.top) / 2, w, h);
  else camera.setViewOffset(w, h, Math.max(0, w - r.left) / 2, 0, w, h);
  camera.updateProjectionMatrix();
  const f = fitScale();                          // panel opened/closed: keep the model fitted to the free area
  if (Math.abs(f - lastFit) > 1e-3) {
    const off = camera.position.clone().sub(controls.target).multiplyScalar(f / lastFit);   // offset first, then move
    camera.position.copy(controls.target).add(off);
    lastFit = f; controls.maxDistance = 4.5 * f;
  }
  requestRender();
}

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  const f = fitScale();
  if (Math.abs(f - lastFit) > 1e-3) {           // keep the framing when the window / phone orientation changes
    const off = camera.position.clone().sub(controls.target).multiplyScalar(f / lastFit);
    camera.position.copy(controls.target).add(off);
    lastFit = f;
  }
  controls.maxDistance = 4.5 * f;
  applyViewOffset();
  renderer.render(scene, camera);               // draw now: resizing clears the canvas
  updateOverlays();
  dirty = false;
}
new ResizeObserver(resize).observe(canvas);

canvas.addEventListener('webglcontextlost', (e) => e.preventDefault());
canvas.addEventListener('webglcontextrestored', () => { buildEnv(); requestRender(); });

// ---------------------------------------------------------------- loading
const MODEL_VERSION = '2026-11-25a';     // bump when the models change, so browsers do not reuse cached copies
const FILES = ['body', 'cover', 'lid', 'lower', 'panel', 'base', 'dock'].map((k) => [k, `models/${k}.glb?v=${MODEL_VERSION}`]);
const GROUP_OF = { panel: 'base' };      // the control panel (STOP key, display) is fixed to the chassis
// the host compresses the models and sends no Content-Length: use the known sizes as the progress total
const SIZES = { body: 9303724, cover: 2955016, lid: 1016052, lower: 479300, panel: 266068, base: 13671280, dock: 5486764 };
const prog = {};
for (const k in SIZES) prog[k] = [0, SIZES[k]];
function updateBar() {
  let l = 0, t = 0;
  for (const [k] of FILES) { const p = prog[k]; if (p) { l += p[0]; t += p[1]; } }
  const pct = t ? Math.min(100, Math.round((l / t) * 100)) : 0;
  $('barFill').style.width = pct + '%';
  $('loaderText').textContent = `${pct}%  ·  ${(l / 1048576).toFixed(1)} / ${(t / 1048576).toFixed(1)} MB`;
}
if (location.protocol === 'file:') {
  fail('浏览器不允许直接从本地文件加载模型。<br>请双击运行“启动查看器”脚本（或 serve.py），它会在浏览器中打开本页面。');
} else {
  const gl = new GLTFLoader(); gl.setMeshoptDecoder(MeshoptDecoder);      // the design models are meshopt-compressed (gltfpack)
  // Web-hosted copy: the host only serves text, so each .glb ships as base64 text chunks (models/manifest.json)
  const B64 = new Uint8Array(128); 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/'.split('').forEach((c, i) => { B64[c.charCodeAt(0)] = i; });
  function decodeB64(s) {
    let n = s.length; while (n && s.charCodeAt(n - 1) === 61) n--;       // strip '=' padding
    const out = new Uint8Array(Math.floor((n * 3) / 4));
    let o = 0, i = 0;
    for (; i + 4 <= n; i += 4) {
      const v = (B64[s.charCodeAt(i)] << 18) | (B64[s.charCodeAt(i + 1)] << 12) | (B64[s.charCodeAt(i + 2)] << 6) | B64[s.charCodeAt(i + 3)];
      out[o++] = v >> 16; out[o++] = (v >> 8) & 255; out[o++] = v & 255;
    }
    if (n - i === 2) { const v = (B64[s.charCodeAt(i)] << 18) | (B64[s.charCodeAt(i + 1)] << 12); out[o++] = v >> 16; }
    else if (n - i === 3) { const v = (B64[s.charCodeAt(i)] << 18) | (B64[s.charCodeAt(i + 1)] << 12) | (B64[s.charCodeAt(i + 2)] << 6); out[o++] = v >> 16; out[o++] = (v >> 8) & 255; }
    return out.buffer;
  }
  let manifest = null;
  async function loadChunks(k) {
    const parts = manifest[k];
    prog[k] = [0, parts.size];
    const texts = await Promise.all(parts.chunks.map(async (url, i) => {
      const r = await fetch(url); if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
      const t = await r.text(); prog[k][0] += t.length; updateBar(); return t;
    }));
    return decodeB64(texts.join(''));
  }
  const loadOnce = ([k, url]) => (window.MOWER_B64
    ? (manifest ? Promise.resolve() : fetch('models/manifest.json').then((r) => r.json()).then((m) => { manifest = m; }))
        .then(() => loadChunks(k))
        .then((buf) => new Promise((res, rej) => gl.parse(buf, '', (g) => res([k, g]), rej)))
    : new Promise((res, rej) =>
        gl.load(url, (g) => res([k, g]), (e) => { const t = Math.max(e.total || 0, SIZES[k] || 0, e.loaded); prog[k] = [e.loaded, t]; updateBar(); }, rej)));
  // network hiccups (slow mobile links, a CDN edge warming up) are retried before giving up
  const retry = (fn, tries = 3) => fn().catch((err) => {
    if (tries <= 1) throw err;
    return new Promise((r) => setTimeout(r, 900 * (4 - tries))).then(() => retry(fn, tries - 1));
  });
  const loadOne = (f) => retry(() => loadOnce(f));
  const loadJSON = (name) => retry(() => fetch(window.MOWER_B64 ? `models/${name}` : `models/${name}?v=${MODEL_VERSION}`).then((r) => { if (!r.ok) throw new Error(`${name}: HTTP ${r.status}`); return r.json(); }));
  Promise.all([...FILES.map(loadOne), loadJSON('hotspots.json'), loadJSON('palettes.json'), loadJSON('dock_profile.json'), loadJSON('design.json'), loadJSON('chassis_cut.json').catch(() => null)])
    .then((res) => {
      CUT = res.pop(); DESIGN = res.pop(); dockProfile = res.pop(); const pal = res.pop(); const hs = res.pop();
      for (const [k, g] of res) groups[GROUP_OF[k] || k].add(g.scene);
      setup(hs, pal);
      $('loader').classList.add('done');
    })
    .catch((err) => { console.error(err); fail('模型加载失败：' + (err && err.message ? err.message : err) + '<br>请检查网络连接后刷新页面重试。'); });
}

// ---------------------------------------------------------------- materials & rig
const mats = {};
const matsAll = {};                 // every material instance per name (a name can occur in several meshes)
const rig = {};
const proxies = [];
const proxyMat = new THREE.MeshBasicMaterial({ visible: false });
// colour schemes (models/palettes.json): body + lid, top deck, accent lines, wheel light ring
let PALETTES = [];
let paletteId = null;
let DESIGN = null;                  // models/design.json: lid hinge axis, opening angle, chassis screw posts
let CUT = null;                     // models/chassis_cut.json (v18): the exact cuts of the two old mudguard features per side
// finish presets: [roughness, metalness, clearcoat, clearcoatRoughness] (same values as the Blender renders)
const FINISH = {
  body: { gloss: [0.30, 0, 0.8, 0.06], metallic: [0.36, 0.55, 1.0, 0.05], satin: [0.50, 0, 0.15, 0.2], matte: [0.75, 0, 0, 0.3], satincoat: [0.42, 0, 0.45, 0.06], satinmetal: [0.36, 0.55, 0.35, 0.06] },
  deck: { gloss: [0.26, 0, 0.8, 0.06], metallic: [0.36, 0.55, 1.0, 0.05], satin: [0.5, 0, 0.1, 0.2], matte: [0.72, 0, 0, 0.3], texture: [0.62, 0, 0, 0.3], glossdark: [0.12, 0, 1.0, 0.05] },
};
const FINISH_ZH = { gloss: '亮面', metallic: '金属漆', satin: '缎面', matte: '哑光', satincoat: '缎面清漆', texture: '细纹理', satinmetal: '缎面金属', glossdark: '烟色亮面' };
// state.wheel: + = rolling forward; state.swivel: 0 = caster trailing behind its pivot
const state = { wheel: { L: 0, R: 0 }, swivel: { L: 0, R: 0 }, spin: { L: 0, R: 0 }, explode: 0, dock: 0, lid: 0 };
const LIFT = { lid: 0.5, cover: 0.4, body: 0.24, lower: 0 };      // the front bumper stays on the chassis
const AX_X = new THREE.Vector3(1, 0, 0), AX_Z = new THREE.Vector3(0, 0, 1);
const FWD_SPIN = new THREE.Vector3(0, 0, -1);   // angular-velocity direction of a wheel rolling towards +X
const UP = new THREE.Vector3(0, 1, 0);
let ghost = false;
let hotspotGhost = false;

function setup(hotspotData, paletteData) {
  scene.traverse((o) => {
    if (!o.isMesh || !o.material) return;
    o.castShadow = true; o.receiveShadow = true;
    const m = o.material; mats[m.name] = mats[m.name] || m;
    (matsAll[m.name] = matsAll[m.name] || []).includes(m) || matsAll[m.name].push(m);
    m.envMapIntensity = 1.0;
  });
  // material tuning (matched to the product renders)
  if (mats.DisplayGlass) { mats.DisplayGlass.envMapIntensity = 0.55; mats.DisplayGlass.roughness = 0.22; }
  if (mats.CyanRing) mats.CyanRing.emissiveIntensity = 0.8;   // colour comes from the scheme (setPaint)
  if (mats.StopButton) mats.StopButton.envMapIntensity = 0.5;
  for (const m of matsAll.LampLight || []) { m.emissive.set(0xf2fbff); m.emissiveIntensity = 1.15; m.color.set(0xf2fbff); }
  for (const n of ['BodyPaint', 'LidPaint']) for (const m of matsAll[n] || []) m.envMapIntensity = 0.8;
  for (const n of ['DeckPaint', 'Glass', 'LensGlass']) for (const m of matsAll[n] || []) m.envMapIntensity = 0.9;
  for (const n of ['Visor', 'SkirtPaint', 'Dark', 'KitDark', 'KitRubber']) for (const m of matsAll[n] || []) m.envMapIntensity = 0.5;
  for (const n of ['CorePaint', 'BasePaint', 'InnerSkin', 'Inner']) for (const m of matsAll[n] || []) m.envMapIntensity = 0.6;      // v18: dark one-piece body, inside of the walls
  for (const m of matsAll.Visor || []) m.envMapIntensity = 1.0;                                                   // v18: smoked lens bands read as glass
  addInnerSkins();
  trimChassis();
  if (DESIGN && DESIGN.dock_dy) groups.dock.position.z = -DESIGN.dock_dy;      // v18: seated pose, pins centred in the chassis contact pockets
  rigLid();
  buildGuides();
  PALETTES = (paletteData && paletteData.schemes) || [];
  buildSwatches();
  setPaint((paletteData && paletteData.default) || (PALETTES[0] && PALETTES[0].id));

  // rig: drive wheels pivot on their axle; casters swivel about the vertical and spin on their axle
  const v3 = (a) => new THREE.Vector3().fromArray(a).normalize();
  for (const s of ['L', 'R']) {
    const w = groups.base.getObjectByName(`Wheel_${s}`);
    const sw = groups.base.getObjectByName(`CasterSwivel_${s}`);
    const c = groups.base.getObjectByName(`Caster_${s}`);
    if (!w || !sw || !c) { console.warn('rig node missing', s); continue; }
    const wa = v3(w.userData.axis);
    rig[`wheel${s}`] = { node: w, axis: wa, q0: w.quaternion.clone(), radius: 0.1148, fwd: Math.sign(wa.dot(FWD_SPIN)) || 1 };
    const trail = new THREE.Vector3(c.position.x, 0, c.position.z);
    rig[`swivel${s}`] = { node: sw, q0: sw.quaternion.clone(), neutral: Math.PI - Math.atan2(-trail.z, trail.x) };
    rig[`caster${s}`] = { node: c, axis: v3(c.userData.axis), q0: c.quaternion.clone(), radius: 0.05, trail };
    addProxy(w, wa, 0.119, 0.07, 'wheel', s);
    addProxy(c, rig[`caster${s}`].axis, 0.056, 0.05, 'caster', s);
    addCasterFork(s);
  }
  // explode lifts derived from the part sizes so the layers clear each other completely
  const bb = (k) => new THREE.Box3().setFromObject(groups[k]);
  const bLid = bb('lid'), bCover = bb('cover'), bBody = bb('body'), bBase = bb('base');
  LIFT.body = Math.max(0.2, bBase.max.y - bBody.min.y + 0.03);                       // the hem clears the control panel
  LIFT.cover = LIFT.body + Math.max(0.1, bBody.max.y - bCover.min.y + 0.12);         // room to look at the screws on the deck
  LIFT.lid = LIFT.cover + Math.max(0.1, bCover.max.y - bLid.min.y + 0.07);
  if (mats.DockPin) { mats.DockPin.metalness = 1; mats.DockPin.roughness = 0.3; }
  if (mats.DockBody) mats.DockBody.envMapIntensity = 0.6;
  buildHotspots(hotspotData);          // before applyRig so attached anchors are measured in the rest pose
  buildPartLabels({ lid: bLid, cover: bCover, body: bBody, base: bBase });
  applyRig();
  setLid(0);
  setDock(0);                          // start parked on the charging base
  requestRender();
  // small scripting hook (handy for automation / debugging in the browser console)
  window.mowerViewer = {
    state, applyRig, setExplode, animateExplode, setDock, animateDock, setLid, animateLid, design: () => DESIGN, mower, setPaint, setGhost, flyTo, palettes: () => PALETTES, paletteId: () => paletteId, renderer, scene, fit: () => ({ lastFit, f: fitScale(), aspect: freeAspect() }), camera, controls, groups, rig, VIEWS, LIFT,
    debugHotspots: () => hotspots.map((h) => {
      const p = h.surf.getWorldPosition(new THREE.Vector3()); const cd = camera.position.clone().sub(p).normalize();
      const n = h.nLocal.clone().applyQuaternion(h.surf.parent.getWorldQuaternion(new THREE.Quaternion()));
      return { id: h.id, occ: h.occluded, facing: +n.dot(cd).toFixed(2), hidden: h.el.classList.contains('hidden') };
    }),
    settle: () => new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(() => r(true)))),
    screenOf: (name, dy = 0) => {
      const o = scene.getObjectByName(name); const p = o.getWorldPosition(new THREE.Vector3()); p.y += dy;
      p.project(camera); const r = canvas.getBoundingClientRect();
      return [r.left + ((p.x + 1) / 2) * r.width, r.top + ((1 - p.y) / 2) * r.height];
    },
  };
}

function addProxy(node, axis, r, h, kind, side) {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), proxyMat);
  m.quaternion.setFromUnitVectors(UP, axis);
  m.userData.drag = { kind, side };
  node.add(m);
  proxies.push(m);
}

// The scan only contains the caster tyres; add a simple fork so the swivel reads clearly.
const forkMat = new THREE.MeshStandardMaterial({ color: 0x2b2f35, roughness: 0.45, metalness: 0.35 });
function addCasterFork(s) {
  const sw = rig[`swivel${s}`].node, c = rig[`caster${s}`];
  const co = c.node.position.clone();                    // wheel centre relative to the pivot
  const a = c.axis.clone(); a.y = 0; a.normalize();
  let tmin = Infinity, tmax = -Infinity;                 // tyre width along the axle
  c.node.traverse((o) => {
    if (!o.isMesh || o.userData.drag) return;
    const p = o.geometry.attributes.position;
    for (let i = 0; i < p.count; i += 7) { const t = p.getX(i) * c.axis.x + p.getY(i) * c.axis.y + p.getZ(i) * c.axis.z; tmin = Math.min(tmin, t); tmax = Math.max(tmax, t); }
  });
  const half = isFinite(tmin) ? (tmax - tmin) / 2 : 0.02;
  const d = new THREE.Vector3(co.x, 0, co.z).normalize();
  const basis = new THREE.Matrix4().makeBasis(d, UP, new THREE.Vector3().crossVectors(d, UP));
  const fork = new THREE.Group(); fork.name = `CasterFork_${s}`;
  const add = (geo, pos, m4) => { const m = new THREE.Mesh(geo, forkMat); if (m4) m.quaternion.setFromRotationMatrix(m4); m.position.copy(pos); m.castShadow = true; fork.add(m); };
  add(new THREE.CylinderGeometry(0.011, 0.011, 0.03, 20), new THREE.Vector3(0, 0.018, 0));
  const len = Math.hypot(co.x, co.z) + 0.03;
  add(new THREE.BoxGeometry(len, 0.006, 2 * half + 0.02), new THREE.Vector3(co.x / 2, 0.006, co.z / 2), basis);
  const plateTop = 0.009, plateBot = co.y - 0.01, ph = plateTop - plateBot;
  const pb = new THREE.Matrix4().makeBasis(d, UP, a);
  for (const sgn of [-1, 1]) {
    const pos = new THREE.Vector3(co.x, (plateTop + plateBot) / 2, co.z).addScaledVector(a, sgn * (half + 0.006));
    add(new THREE.BoxGeometry(0.03, ph, 0.004), pos, pb);
  }
  add(new THREE.CylinderGeometry(0.0045, 0.0045, 2 * half + 0.026, 16), co.clone(), new THREE.Matrix4().makeBasis(d, a, new THREE.Vector3().crossVectors(d, a)));
  sw.add(fork);
}

const tmpQ = new THREE.Quaternion();
function applyRig() {
  for (const s of ['L', 'R']) {
    const w = rig[`wheel${s}`], sw = rig[`swivel${s}`], c = rig[`caster${s}`];
    if (!w) continue;
    w.node.quaternion.copy(w.q0).multiply(tmpQ.setFromAxisAngle(w.axis, state.wheel[s] * w.fwd));
    sw.node.quaternion.copy(sw.q0).multiply(tmpQ.setFromAxisAngle(UP, sw.neutral + state.swivel[s]));
    c.node.quaternion.copy(c.q0).multiply(tmpQ.setFromAxisAngle(c.axis, state.spin[s]));
  }
  syncSliders();
  requestRender();
}

function applyFinish(m, hex, finish, part) {
  const [r, me, cc, ccr] = FINISH[part][finish] || FINISH[part].satin;
  m.color.set(hex); m.roughness = r; m.metalness = me;
  if ('clearcoat' in m) { m.clearcoat = cc; m.clearcoatRoughness = ccr; }
}
function setPaint(id) {
  const p = PALETTES.find((x) => x.id === id); if (!p) return;
  paletteId = p.id;
  // which materials take the two scheme colours: v18 Orbit = cover + lid / dark body; Horizon (design.json paint_map) = canopy / spine + lid
  const PM = (DESIGN && DESIGN.paint_map) || { body: ['BodyPaint', 'LidPaint'], deck: ['DeckPaint', 'CorePaint'] };
  for (const n of PM.body) for (const m of matsAll[n] || []) applyFinish(m, p.body.hex, p.body.finish, 'body');
  for (const n of PM.deck) for (const m of matsAll[n] || []) applyFinish(m, p.deck.hex, p.deck.finish, 'deck');
  for (const m of matsAll.Accent || []) m.color.set(p.accent);
  // wheel light ring: emissive LED colour normalised to full brightness, dim base colour underneath
  const e = new THREE.Color(p.ring || '#2fc6dc'); const mx = Math.max(e.r, e.g, e.b) || 1; e.multiplyScalar(1 / mx);
  for (const m of matsAll.CyanRing || []) { m.emissive.copy(e); m.color.copy(e).multiplyScalar(0.12); }
  document.querySelectorAll('.swatch').forEach((b) => {
    const on = b.dataset.id === p.id;
    b.classList.toggle('active', on); b.setAttribute('aria-checked', String(on)); b.tabIndex = on ? 0 : -1;
  });
  const lab = $('schemeName');
  if (lab) {                                      // built with DOM nodes, not innerHTML
    const nm = document.createElement('b'); nm.textContent = p.zh;
    const en = document.createElement('span'); en.textContent = p.en;
    const dot = document.createElement('i'); dot.className = 'ring-dot'; dot.style.background = '#' + e.getHexString(THREE.SRGBColorSpace);
    lab.replaceChildren(nm, ' ', en, document.createElement('br'),
      `上盖 ${FINISH_ZH[p.body.finish] || ''} · ${DESIGN && DESIGN.variant === 'horizon' ? '中脊饰带' : '车身'} ${FINISH_ZH[p.deck.finish] || ''} · 轮毂灯环 `, dot);
  }
  requestRender();
}
function buildSwatches() {
  const box = $('swatches'); if (!box) return;
  box.textContent = '';
  PALETTES.forEach((p, i) => {
    const b = document.createElement('button');
    b.className = 'swatch'; b.type = 'button'; b.dataset.id = p.id;
    b.setAttribute('role', 'radio'); b.setAttribute('aria-checked', 'false'); b.tabIndex = -1;
    b.title = `${p.zh}（${p.en}）`; b.setAttribute('aria-label', `${p.zh}：上盖、车身与轮毂灯环配色`);
    b.style.setProperty('--c1', p.body.hex); b.style.setProperty('--c2', p.deck.hex); b.style.setProperty('--s', p.accent); b.style.setProperty('--r', p.ring || '#2fc6dc');
    b.addEventListener('click', () => setPaint(p.id));
    b.addEventListener('keydown', (e) => {          // radio-group arrow keys
      const step = { ArrowRight: 1, ArrowDown: 1, ArrowLeft: -1, ArrowUp: -1 }[e.key];
      if (!step) return;
      e.preventDefault(); const all = box.querySelectorAll('.swatch'); const n = all[(i + step + all.length) % all.length];
      setPaint(n.dataset.id); n.focus();
    });
    box.appendChild(b);
  });
}

// see-through shells: lets the user see and grab the chassis, drive wheels and casters
function setGhost(on) {
  ghost = on;
  $('ghost').checked = on;
  for (const k of SHELLS) groups[k].traverse((o) => {
    if (!o.isMesh || o.userData.drag) return;
    const m = o.material;
    if (!m.userData.ghostSave) m.userData.ghostSave = { transparent: m.transparent, opacity: m.opacity, depthWrite: m.depthWrite };
    if (on) { m.transparent = true; m.opacity = 0.15; m.depthWrite = false; }
    else Object.assign(m, m.userData.ghostSave);
    m.needsUpdate = true;
    o.castShadow = !on && !o.userData.inner;
  });
  syncGuides();
  requestRender();
}

// ---------------------------------------------------------------- explode
function setExplode(e) {
  const prev = state.explode;
  state.explode = e;
  if (e > 0.001 && (state.lid > 0 || lidAnim)) { lidAnim = null; setLid(0); }      // parts come apart with the lid shut
  for (const k of SHELLS) groups[k].position.y = e * LIFT[k];
  syncGuides();
  const lb = $('lidBtn'); if (lb) { lb.disabled = e > 0.02; $('lid').disabled = e > 0.02; }
  if (!fly) {                                   // camera follows: keep the growing stack in frame
    const dy = (e - prev) * LIFT.lid * 0.5;
    controls.target.y += dy; camera.position.y += dy;
    const k = (1 + 0.5 * e) / (1 + 0.5 * prev);
    const off = camera.position.clone().sub(controls.target).multiplyScalar(k);
    camera.position.copy(controls.target).add(off);
  }
  $('explode').value = Math.round(e * 100);
  $('explode').setAttribute('aria-valuetext', `${Math.round(e * 100)}%`);
  $('explodeVal').textContent = Math.round(e * 100) + '%';
  $('explodeBtn').textContent = e > 0.5 ? '合拢复原' : '展开分解';
  requestRender();
}
let explodeAnim = null;
function animateExplode(to) {
  if (fly) {                                    // a camera flight is under way: re-aim it at the final stack
    const k = (1 + 0.5 * to) / (1 + 0.5 * state.explode);
    fly.t1.y += (to - state.explode) * LIFT.lid * 0.5; fly.s1.radius *= k;
  }
  explodeAnim = { from: state.explode, to, t: 0, dur: reduceMotion ? 0.01 : 1.1 };
}

// exploded view: a thin line from every fastening point of the chassis up to its place in the lifted body
// (orange = the 14 screws that end up under the top cover, blue = the 5 screws that go in from below)
const guides = new THREE.Group(); guides.name = 'Guides';
function buildGuides() {
  const geo = new THREE.CylinderGeometry(0.0008, 0.0008, 1, 8); geo.translate(0, 0.5, 0);
  const add = (list, color) => {
    const m = new THREE.MeshBasicMaterial({ color });
    for (const p of list || []) { const c = new THREE.Mesh(geo, m); c.position.fromArray(p); c.raycast = () => {}; guides.add(c); }
  };
  add(DESIGN.posts, 0xff6a2b); add(DESIGN.under, 0x3fa9ff);
  guides.visible = false; mower.add(guides);
}
function syncGuides() {
  guides.visible = state.explode > 0.02 && groups.body.visible && groups.base.visible && !ghost;
  const len = Math.max(1e-4, state.explode * LIFT.body + 0.003);
  for (const c of guides.children) c.scale.y = len;
}

// ---------------------------------------------------------------- lid: hinge, opening
// v18: a hidden virtual-pivot arc-track hinge - the lid turns about a fixed axis (parallel to Z) that lies just outside the rear gap,
// up to a stop at 75 deg; no gas struts. The lid and its well ride on the top cover.
let lidRig = null, lidAnim = null;
function rigLid() {
  const L = DESIGN.lid, pivot = new THREE.Group(); pivot.name = 'LidHinge'; pivot.position.fromArray(L.pivot);
  for (const ch of [...groups.lid.children]) { ch.position.sub(pivot.position); pivot.add(ch); }
  groups.lid.add(pivot);
  lidRig = { pivot, open: THREE.MathUtils.degToRad(L.open_deg), p: vec(L.pivot) };
}
function setLid(a) {
  if (!lidRig) return;
  state.lid = a; lidRig.pivot.rotation.z = a;
  const f = a / lidRig.open, to = lidAnim ? lidAnim.to : a;
  $('lid').value = Math.round(f * 100); $('lid').setAttribute('aria-valuetext', `${Math.round(THREE.MathUtils.radToDeg(a))} 度`);
  $('lidVal').textContent = Math.round(THREE.MathUtils.radToDeg(a)) + '°';
  $('lidBtn').textContent = to > lidRig.open / 2 ? '合上盖板' : '打开盖板';
  requestRender();
}
function animateLid(to) {
  if (!lidRig || state.explode > 0.02) return;
  lidAnim = { from: state.lid, to, t: 0, dur: reduceMotion ? 0.01 : Math.max(0.35, 1.3 * Math.abs(to - state.lid) / lidRig.open) };
  setLid(state.lid);
}
// The body and the top cover are single skins: seen from inside they are unpainted dark plastic (same as in the renders).
// (v17 only: the v18 parts are closed solids, see DESIGN.wall)
// Each skin mesh gets a twin that shares its geometry and only draws the back faces.
const innerMat = new THREE.MeshStandardMaterial({ color: 0x2e3035, roughness: 0.6, metalness: 0, side: THREE.BackSide });
innerMat.name = 'ShellInner';
function addInnerSkins() {
  if (DESIGN && DESIGN.wall) return;          // v18: both parts are closed 2.5 mm solids with a real inner skin
  for (const name of ['NewShell', 'TopCover']) {
    const root = scene.getObjectByName(name); if (!root) continue;
    const meshes = []; root.traverse((o) => { if (o.isMesh) meshes.push(o); });
    for (const o of meshes) {
      o.material.side = THREE.FrontSide;
      const tw = new THREE.Mesh(o.geometry, innerMat); tw.name = o.name + '_inner';
      tw.userData.inner = true; tw.castShadow = false; tw.receiveShadow = true; tw.raycast = () => {};
      o.add(tw);
    }
  }
}
// The redesigned body is narrower beside the drive wheels and has no mudguards: the chassis struts that carried the
// old mudguards would stick out there, so they are left out (same as in the product renders). The screw posts on the
// tub rim stay: the body is screwed to them.
function trimChassis() {
  const ch = groups.base.getObjectByName('Chassis'); if (!ch) return;
  ch.updateWorldMatrix(true, true);
  if (CUT) { cutChassisMeshes(ch); return; }
  const v = new THREE.Vector3(), posts = (DESIGN && DESIGN.posts) || [];
  const nearPost = (p) => posts.some((q) => (p.x - q[0]) ** 2 + (p.z - q[2]) ** 2 < 0.0068 ** 2);
  ch.traverse((o) => {
    if (!o.isMesh || !o.geometry.index) return;
    const pos = o.geometry.attributes.position, idx = o.geometry.index.array, cut = new Uint8Array(pos.count);
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
      if (Math.abs(v.z) > 0.160 && v.y > 0.135 && v.x > -0.06 && v.x < 0.29 && !nearPost(v)) cut[i] = 1;
    }
    const keep = new idx.constructor(idx.length); let n = 0;
    for (let i = 0; i < idx.length; i += 3) {
      if (cut[idx[i]] || cut[idx[i + 1]] || cut[idx[i + 2]]) continue;
      keep[n++] = idx[i]; keep[n++] = idx[i + 1]; keep[n++] = idx[i + 2];
    }
    o.geometry.setIndex(new THREE.BufferAttribute(keep.slice(0, n), 1));
  });
}

// v18: the chassis keeps its rim flange and drive-unit housings; only the old mudguard locating pin and front hook are cut, each by one
// plane with a flat cap (models/chassis_cut.json, chassis_cut.js). Positions are cut in the base-model frame, then mapped back.
function cutChassisMeshes(ch) {
  const toBase = new THREE.Matrix4().copy(groups.base.matrixWorld).invert();
  ch.traverse((o) => {
    if (!o.isMesh || !o.geometry.index) return;
    const g = o.geometry, rel = toBase.clone().multiply(o.matrixWorld), inv = rel.clone().invert();
    const n0 = g.attributes.position.count, p = g.attributes.position.clone().applyMatrix4(rel);
    const r = cutChassis(p.array, g.index.array, CUT, 'gltf');
    const pos = new THREE.BufferAttribute(r.position, 3).applyMatrix4(inv), n1 = pos.count;
    const nor = new Float32Array(n1 * 3);
    if (g.attributes.normal) nor.set(g.attributes.normal.array.subarray(0, n0 * 3));
    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3(), idx = r.index;
    for (let i = 0; i < idx.length; i += 3) {                    // new vertices (clip points, caps): normal from their faces
      const [i0, i1, i2] = [idx[i], idx[i + 1], idx[i + 2]];
      if (i0 < n0 && i1 < n0 && i2 < n0) continue;
      a.fromBufferAttribute(pos, i0); b.fromBufferAttribute(pos, i1); c.fromBufferAttribute(pos, i2);
      const fn = b.sub(a).cross(c.sub(a));
      for (const k of [i0, i1, i2]) if (k >= n0) { nor[3 * k] += fn.x; nor[3 * k + 1] += fn.y; nor[3 * k + 2] += fn.z; }
    }
    for (let k = n0; k < n1; k++) { const l = Math.hypot(nor[3 * k], nor[3 * k + 1], nor[3 * k + 2]) || 1; nor[3 * k] /= l; nor[3 * k + 1] /= l; nor[3 * k + 2] /= l; }
    const ng = new THREE.BufferGeometry(); ng.setAttribute('position', pos); ng.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
    ng.setIndex(new THREE.BufferAttribute(idx, 1)); o.geometry = ng; g.dispose();
  });
}

// ---------------------------------------------------------------- charging base: parked pose and driving off
// state.dock = distance (m) the mower has reversed away from the parked pose; the charging pins are at the front,
// so it backs straight out. Wheel contact heights follow the plate profile (models/dock_profile.json).
let dockProfile = null, dockAnim = null;
const DOCK_END = 0.85, X_DRIVE = 0.12, X_CASTER = -0.22;
function plateZ(track, x) {
  if (!dockProfile || !groups.dock.visible) return 0;
  const a = dockProfile[track], f = (x - dockProfile.x0) / dockProfile.dx, i = Math.floor(f);
  if (i < 0 || i >= a.length - 1) return 0;
  return a[i] + (a[i + 1] - a[i]) * (f - i);
}
function setDock(d) {
  const prev = state.dock; state.dock = d;
  const hd = plateZ('drive', X_DRIVE - d), hc = plateZ('caster', X_CASTER - d), span = X_DRIVE - X_CASTER;
  mower.position.set(-d, hc + (hd - hc) * (-X_CASTER / span), 0);
  mower.rotation.z = Math.atan2(hd - hc, span);
  blob.position.x = -d; blob.position.y = Math.min(hd, hc) + 0.0006;
  key.position.x = 0.9 - d * 0.5; key.target.position.x = -d * 0.5;
  const ds = -(d - prev);                       // + = forwards
  if (ds !== 0) {
    for (const s of ['L', 'R']) {
      const w = rig[`wheel${s}`], sw = rig[`swivel${s}`], c = rig[`caster${s}`]; if (!w) continue;
      state.wheel[s] += ds / w.radius;
      const want = ds < 0 ? Math.PI : 0;        // casters trail behind the direction of travel
      state.swivel[s] += Math.atan2(Math.sin(want - state.swivel[s]), Math.cos(want - state.swivel[s])) * Math.min(1, Math.abs(ds) / 0.05);
      const axisNow = c.axis.clone().applyAxisAngle(UP, sw.neutral + state.swivel[s]);
      state.spin[s] += (ds / c.radius) * axisNow.dot(FWD_SPIN);
    }
    applyRig();
    if (!fly) {                                 // camera follows half-way and pulls back so base and mower stay in frame
      const dx = -(d - prev) * 0.5; controls.target.x += dx; camera.position.x += dx;
      const k = (1 + 0.8 * d / DOCK_END) / (1 + 0.8 * prev / DOCK_END);
      const off = camera.position.clone().sub(controls.target).multiplyScalar(k);
      camera.position.copy(controls.target).add(off);
    }
  }
  const away = (dockAnim ? dockAnim.to : d) > DOCK_END / 2, b = $('dockBtn');
  if (b) { b.textContent = away ? '返回充电座' : '驶离充电座'; b.disabled = !groups.dock.visible; }
  requestRender();
}
function animateDock(to) {
  $('drive').checked = false;
  dockAnim = { from: state.dock, to, t: 0, dur: reduceMotion ? 0.01 : Math.max(0.6, 4.2 * Math.abs(to - state.dock) / DOCK_END) };
  setDock(state.dock);
}
const easeIO = (x) => (x < 0.5 ? 2 * x * x : 1 - Math.pow(-2 * x + 2, 2) / 2);

// ---------------------------------------------------------------- camera flights (spherical, around the target)
let fly = null;
function flyTo(cam, target, dur = 0.9) {
  const t1 = vec(target), p1 = fitted(cam, target);
  const s0 = new THREE.Spherical().setFromVector3(camera.position.clone().sub(controls.target));
  const s1 = new THREE.Spherical().setFromVector3(p1.clone().sub(t1));
  if (s1.phi < 0.02) s1.theta = Math.atan2(cam[0] - target[0], cam[2] - target[2]);   // top view: keep its heading
  const dth = Math.atan2(Math.sin(s1.theta - s0.theta), Math.cos(s1.theta - s0.theta));
  fly = { t0: controls.target.clone(), t1, s0, s1, dth, t: 0, dur: reduceMotion ? 0.01 : dur };
  $('autoRotate').checked = false; controls.autoRotate = false;
}
const ease = (x) => (x < 0.5 ? 4 * x * x * x : 1 - Math.pow(-2 * x + 2, 3) / 2);

// ---------------------------------------------------------------- hotspots, leader lines, part labels
const hotspots = [];
const svgNS = 'http://www.w3.org/2000/svg';
function buildHotspots(list) {
  const host = $('hotspots'), svg = $('leaders');
  list.forEach((h, i) => {
    const parent = (h.attach && groups.base.getObjectByName(h.attach)) || (h.part === 'lid' && lidRig ? lidRig.pivot : groups[h.part]);   // the lid marker swings with the lid
    const worldPos = vec(h.pos), worldN = vec(h.normal || [0, 1, 0]).normalize();
    const surf = new THREE.Object3D(), mark = new THREE.Object3D();
    parent.add(surf); parent.add(mark);
    parent.updateWorldMatrix(true, false);
    const invQ = parent.getWorldQuaternion(new THREE.Quaternion()).invert();
    const nLocal = worldN.clone().applyQuaternion(invQ);
    surf.position.copy(parent.worldToLocal(worldPos.clone()));
    // the marker floats off the feature (along liftDir, else the normal) so it never covers what it labels
    const liftDir = h.liftDir ? vec(h.liftDir).normalize() : worldN;
    mark.position.copy(parent.worldToLocal(worldPos.clone().addScaledVector(liftDir, h.lift ?? 0.035)));
    const el = document.createElement('button');
    el.className = 'hs'; el.type = 'button';
    el.innerHTML = `<span class="dot">${i + 1}</span><span class="tip">${h.title}</span>`;
    el.setAttribute('aria-label', `${i + 1}. ${h.title}`);
    el.addEventListener('click', (ev) => { ev.stopPropagation(); openInfo(i); });
    host.appendChild(el);
    const line = document.createElementNS(svgNS, 'line'); svg.appendChild(line);
    hotspots.push({ ...h, surf, mark, nLocal, el, line, occluded: false });
  });
}
let activeHotspot = -1;
function openInfo(i) {
  const h = hotspots[i];
  activeHotspot = i;
  hotspots.forEach((x, j) => x.el.classList.toggle('active', j === i));
  $('infoTitle').textContent = `${i + 1}. ${h.title}`;
  $('infoText').textContent = h.text;
  $('info').hidden = false;
  $('infoClose').focus({ preventScroll: true });
  clearActiveView();
  if (mqMobile.matches && !panel.classList.contains('collapsed')) setPanel(false);   // phones: the sheet would cover the part
  if (h.lid && state.explode < 0.02 && (lidAnim ? lidAnim.to : state.lid) < lidRig.open / 2) animateLid(lidRig.open);     // the part is under the lid
  if (h.explode && (explodeAnim ? explodeAnim.to : state.explode) < 0.5) animateExplode(1);                                // the part is under the top cover
  // parts on the nose: parked on the base, the charging tower stands right in front of them, so the mower backs off first
  if (h.front && groups.dock.visible && (dockAnim ? dockAnim.to : state.dock) < DOCK_END / 2) animateDock(DOCK_END);
  if (h.focus === 'xray' && !ghost) { setGhost(true); hotspotGhost = true; }
  else if (h.focus !== 'xray' && hotspotGhost) { setGhost(false); hotspotGhost = false; }
  if (h.view) {
    const e = explodeAnim ? explodeAnim.to : state.explode;    // aim at where the part ends up
    const off = new THREE.Vector3(h.part === 'dock' ? 0 : -(dockAnim ? dockAnim.to : state.dock), e * (LIFT[h.part] || 0), 0);
    flyTo(h.view.cam.map((v, k) => v + off.getComponent(k)), h.view.target.map((v, k) => v + off.getComponent(k)));
  }
}
function closeInfo() {
  $('info').hidden = true;
  hotspots.forEach((x) => x.el.classList.remove('active'));
  if (hotspotGhost) { setGhost(false); hotspotGhost = false; }
  if (activeHotspot >= 0 && hotspots[activeHotspot]) hotspots[activeHotspot].el.focus({ preventScroll: true });
  activeHotspot = -1;
}
$('infoClose').addEventListener('click', closeInfo);
window.addEventListener('keydown', (e) => { if (e.key === 'Escape' && !$('info').hidden) closeInfo(); });

const partLabels = [];
function buildPartLabels(boxes) {
  const host = $('partlabels');
  const defs = [['lid', '盖板（含 STOP 键）'], ['cover', '卡扣式上盖'], ['body', '一体式车身（14 颗螺丝）'], ['base', '底盘（含前保险杠）']];
  for (const [k, text] of defs) {
    const el = document.createElement('div'); el.className = 'plabel'; el.textContent = text; host.appendChild(el);
    const b = boxes[k]; const corners = [];
    for (const x of [b.min.x, b.max.x]) for (const y of [b.min.y, b.max.y]) for (const z of [b.min.z, b.max.z]) corners.push(new THREE.Vector3(x, y, z));
    partLabels.push({ el, k, corners });
  }
}

const pv = new THREE.Vector3(), pm = new THREE.Vector3(), camDir = new THREE.Vector3(), nW = new THREE.Vector3(), wq = new THREE.Quaternion();
function toScreen(v, w, h) { return [((v.x + 1) / 2) * w, ((1 - v.y) / 2) * h]; }
const coarseHS = window.matchMedia('(pointer: coarse)').matches;
function updateOverlays() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  const show = $('showHotspots').checked;
  for (const hs of hotspots) {
    const visibleGroup = groups[hs.part].visible;
    hs.surf.getWorldPosition(pv); hs.mark.getWorldPosition(pm);
    camDir.copy(camera.position).sub(pv).normalize();
    nW.copy(hs.nLocal).applyQuaternion(hs.surf.parent.getWorldQuaternion(wq));
    const facing = nW.dot(camDir) > 0.05;
    pv.project(camera); pm.project(camera);
    const onScreen = pm.z < 1 && Math.abs(pm.x) < 1.05 && Math.abs(pm.y) < 1.05;
    const underLid = hs.lid && hs.part !== 'lid' && state.lid < 0.2;        // STOP / display: marker stays visible above the closed lid
    const underCover = hs.explode && state.explode < 0.2;                   // screws: marker stays visible above the fitted cover
    const vis = show && visibleGroup && facing && onScreen && (!hs.occluded || ghost || underLid || underCover);
    hs.el.classList.toggle('hidden', !vis);
    hs.el.tabIndex = vis ? 0 : -1;
    const [mx, my] = toScreen(pm, w, h), [sx, sy] = toScreen(pv, w, h);
    hs._p = { mx, my, sx, sy, z: pm.z, vis };
  }
  // markers that would overlap are pushed apart (nearest keeps its place), so every one stays readable and tappable
  const R = coarseHS ? 40 : 27, placed = [];
  for (const hs of hotspots.filter((x) => x._p.vis).sort((a, b) => a._p.z - b._p.z)) {
    const p = hs._p;
    for (let pass = 0; pass < 3; pass++) for (const q of placed) {
      let dx = p.mx - q.mx, dy = p.my - q.my; const d = Math.hypot(dx, dy);
      if (d >= R) continue;
      if (d < 0.5) { dx = 1; dy = -0.4; }
      const n = Math.hypot(dx, dy); p.mx = q.mx + (dx / n) * R; p.my = q.my + (dy / n) * R;
    }
    placed.push(p);
  }
  for (const hs of hotspots) {
    const { mx, my, sx, sy, vis } = hs._p;
    hs.el.style.translate = `${mx}px ${my}px`;
    hs.line.setAttribute('x1', sx); hs.line.setAttribute('y1', sy); hs.line.setAttribute('x2', mx); hs.line.setAttribute('y2', my);
    hs.line.style.opacity = vis ? 1 : 0;
  }
  const op = THREE.MathUtils.smoothstep(state.explode, 0.25, 0.6);
  for (const pl of partLabels) {
    const g = groups[pl.k]; g.updateWorldMatrix(true, false);
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const c of pl.corners) {
      pv.copy(c).applyMatrix4(g.matrixWorld).project(camera);
      const [x, y] = toScreen(pv, w, h); x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
    }
    // place the label beside the part, on whichever side has room inside the free (uncovered) area
    const lw = pl.el.offsetWidth || 120, cy = (y0 + y1) / 2;
    const pr = panel.getBoundingClientRect();
    const freeRight = (!panel.classList.contains('collapsed') && !mqMobile.matches) ? pr.left - 8 : w - 8;
    let mode, x, y = cy;
    if (x0 - 12 - lw >= 8) { mode = 'left'; x = x0 - 12; }
    else if (x1 + 12 + lw <= freeRight) { mode = 'right'; x = x1 + 12; }
    else { mode = 'above'; x = Math.min(Math.max((x0 + x1) / 2, lw / 2 + 8), freeRight - lw / 2); y = y0 - 6; }   // narrow screens: above the part
    pl._p = { mode, x, y, lw, show: g.visible };
  }
  const shown = partLabels.filter((pl) => pl._p.show).sort((a, b) => b._p.y - a._p.y);   // bottom label first
  for (let i = 1; i < shown.length; i++) {
    const a = shown[i - 1]._p, b = shown[i]._p;
    const ax = a.mode === 'above' ? a.x - a.lw / 2 : (a.mode === 'left' ? a.x - a.lw : a.x), bx = b.mode === 'above' ? b.x - b.lw / 2 : (b.mode === 'left' ? b.x - b.lw : b.x);
    if (ax < bx + b.lw && bx < ax + a.lw && a.y - b.y < 28) b.y = a.y - 28;
  }
  for (const pl of partLabels) {
    const { mode, x, y, show } = pl._p;
    pl.el.style.opacity = show ? op : 0;
    pl.el.style.translate = `${x}px ${y}px`;
    pl.el.classList.toggle('right', mode === 'right');
    pl.el.classList.toggle('above', mode === 'above');
  }
}

// marker occlusion: a small depth pass when the view comes to rest
const depthMat = new THREE.MeshDepthMaterial({ depthPacking: THREE.RGBADepthPacking });
let occRT = null;
const px4 = new Uint8Array(4);
const UNPACK = [255 / 256 / (256 * 256 * 256), 255 / 256 / (256 * 256), 255 / 256 / 256, 255 / 256];
const viewZ = (d) => (camera.near * camera.far) / ((camera.far - camera.near) * d - camera.far);
function computeOcclusion() {
  if (!hotspots.length || !$('showHotspots').checked) return;
  const w = Math.max(1, Math.floor(canvas.clientWidth / 2)), h = Math.max(1, Math.floor(canvas.clientHeight / 2));
  if (!occRT || occRT.width !== w || occRT.height !== h) { if (occRT) occRT.dispose(); occRT = new THREE.WebGLRenderTarget(w, h); }
  const hide = [ground, blob, ...proxies, ...(ghost ? SHELLS.map((k) => groups[k]) : [])];
  const saved = hide.map((o) => o.visible); hide.forEach((o) => { o.visible = false; });
  const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
  scene.overrideMaterial = depthMat; renderer.setClearColor(0xffffff, 1);
  renderer.setRenderTarget(occRT); renderer.render(scene, camera); renderer.setRenderTarget(null);
  scene.overrideMaterial = null; renderer.setClearColor(cc, ca);
  hide.forEach((o, i) => { o.visible = saved[i]; });
  for (const hs of hotspots) {
    hs.surf.getWorldPosition(pv); pv.project(camera);
    const x = Math.floor(((pv.x + 1) / 2) * w), y = Math.floor(((pv.y + 1) / 2) * h);
    if (x < 0 || y < 0 || x >= w || y >= h) { hs.occluded = false; continue; }
    renderer.readRenderTargetPixels(occRT, x, y, 1, 1, px4);
    const d = (px4[0] * UNPACK[0] + px4[1] * UNPACK[1] + px4[2] * UNPACK[2] + px4[3] * UNPACK[3]) / 255;
    const dMark = (pv.z + 1) / 2;
    hs.occluded = viewZ(d) - viewZ(dMark) > 0.012;         // something is > 12 mm in front of the anchor
  }
}

// ---------------------------------------------------------------- dragging wheels / casters
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let drag = null;
const tip = document.createElement('div');
tip.className = 'plabel cursor-tip';
$('partlabels').appendChild(tip);
const DRAG_TIPS = { wheel: { L: '拖动以转动左驱动轮', R: '拖动以转动右驱动轮' }, caster: { L: '拖动以调整左万向轮朝向', R: '拖动以调整右万向轮朝向' } };

function setRay(e) {
  const r = canvas.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
}
function isUnder(o, node) { while (o) { if (o === node) return true; o = o.parent; } return false; }
function pickProxy(e, precise = true) {
  if (!groups.base.visible) return null;
  setRay(e);
  const hit = raycaster.intersectObjects(proxies, false)[0];
  if (!hit) return null;
  const d = hit.object.userData.drag;
  const node = d.kind === 'wheel' ? rig[`wheel${d.side}`].node : rig[`swivel${d.side}`].node;
  let point = hit.point.clone();
  if (precise) {                 // only grab parts that are actually visible under the cursor
    const blockers = [groups.base];
    if (!ghost) for (const k of SHELLS) if (groups[k].visible) blockers.push(groups[k]);
    const first = raycaster.intersectObjects(blockers, true).find((x) => x.object.isMesh && !x.object.userData.drag);
    if (first) {
      if (isUnder(first.object, node)) point = first.point.clone();
      else if (first.distance < hit.distance - 0.008) return null;
    }
  }
  return { ...d, point };
}
function dragFrame(d) {
  if (d.kind === 'wheel') {
    const w = rig[`wheel${d.side}`];
    const c = w.node.getWorldPosition(new THREE.Vector3());
    const a = w.axis.clone().applyQuaternion(w.node.parent.getWorldQuaternion(new THREE.Quaternion())).normalize();
    return { c, a };
  }
  const sw = rig[`swivel${d.side}`];
  const c = sw.node.getWorldPosition(new THREE.Vector3());
  const a = UP.clone().applyQuaternion(sw.node.parent.getWorldQuaternion(new THREE.Quaternion())).normalize();
  return { c, a };
}
function angleOnPlane(frame) {
  if (Math.abs(raycaster.ray.direction.dot(frame.a)) < 0.3) return null;    // too edge-on: use the tangent method
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(frame.a, frame.c);
  const p = new THREE.Vector3();
  if (!raycaster.ray.intersectPlane(plane, p)) return null;
  const u = Math.abs(frame.a.y) < 0.9 ? UP.clone() : new THREE.Vector3(1, 0, 0);
  u.sub(frame.a.clone().multiplyScalar(u.dot(frame.a))).normalize();
  const v = new THREE.Vector3().crossVectors(frame.a, u);
  p.sub(frame.c);
  return Math.atan2(p.dot(v), p.dot(u));
}
// screen-space fallback: rotation from the mouse motion along the projected tangent at the grabbed point
function tangentDelta(dr, dx, dy) {
  const { c, a } = dr.frame;
  const rel = dr.grab.clone().sub(c);
  const rperp = rel.clone().sub(a.clone().multiplyScalar(rel.dot(a)));
  const R = Math.max(rperp.length(), 0.02);
  const T = new THREE.Vector3().crossVectors(a, rperp).normalize();
  const r = canvas.getBoundingClientRect();
  const s0 = dr.grab.clone().project(camera), s1 = dr.grab.clone().addScaledVector(T, 0.01).project(camera);
  const tx = ((s1.x - s0.x) / 2) * r.width / 0.01, ty = (-(s1.y - s0.y) / 2) * r.height / 0.01;   // px per metre
  const t2 = tx * tx + ty * ty;
  if (t2 < 1e-6) return (dr.kind === 'wheel' ? -dy : dx) * 0.012;
  return (dx * tx + dy * ty) / (t2 * R);
}

canvas.addEventListener('pointerdown', (e) => {
  if (drag) {                                   // a second finger: give the gesture back to the orbit controls
    endDrag({ pointerId: drag.id });
    return;
  }
  if (e.button !== 0) return;
  const d = pickProxy(e);
  if (!d) return;
  e.stopImmediatePropagation(); e.preventDefault();
  controls.enabled = false;
  canvas.setPointerCapture(e.pointerId);
  const frame = dragFrame(d);
  drag = { ...d, frame, grab: d.point, last: angleOnPlane(frame), x: e.clientX, y: e.clientY, id: e.pointerId };
  canvas.classList.add('grabbing');
  tip.style.opacity = 0;
  $('drive').checked = false;
}, { capture: true });

let hoverT = 0;
canvas.addEventListener('pointermove', (e) => {
  if (drag) {
    if (e.pointerId !== drag.id) return;
    setRay(e);
    const ang = angleOnPlane(drag.frame);
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    const delta = (ang !== null && drag.last !== null) ? Math.atan2(Math.sin(ang - drag.last), Math.cos(ang - drag.last)) : tangentDelta(drag, dx, dy);
    drag.last = ang; drag.x = e.clientX; drag.y = e.clientY;
    drag.grab.sub(drag.frame.c).applyAxisAngle(drag.frame.a, delta).add(drag.frame.c);   // the grabbed point turns with the part
    if (drag.kind === 'wheel') state.wheel[drag.side] += delta * rig[`wheel${drag.side}`].fwd;
    else state.swivel[drag.side] += delta;
    applyRig();
    return;
  }
  if (e.buttons || e.pointerType === 'touch') return;
  const now = performance.now();
  if (now - hoverT < 90) return;                // the visibility test raycasts the full model: throttle it
  hoverT = now;
  const d = pickProxy(e);
  canvas.classList.toggle('grab', !!d);
  if (d) {
    tip.textContent = DRAG_TIPS[d.kind][d.side]; tip.style.opacity = 1;
    tip.style.translate = `${e.clientX + 16}px ${e.clientY + 18}px`;
  } else tip.style.opacity = 0;
});
function endDrag(e) {
  if (!drag || (e.pointerId !== undefined && e.pointerId !== drag.id)) return;
  if (canvas.hasPointerCapture(drag.id)) canvas.releasePointerCapture(drag.id);
  drag = null; controls.enabled = true; canvas.classList.remove('grabbing');
  occlusionDirty = true;
}
canvas.addEventListener('pointerup', endDrag);
canvas.addEventListener('pointercancel', endDrag);
canvas.addEventListener('pointerleave', () => { if (!drag) tip.style.opacity = 0; });

// ---------------------------------------------------------------- UI wiring
function clearActiveView() {
  document.querySelectorAll('[data-view]').forEach((b) => { b.classList.remove('active'); b.setAttribute('aria-pressed', 'false'); });
}
document.querySelectorAll('[data-view]').forEach((b) => b.addEventListener('click', () => {
  const v = VIEWS[b.dataset.view];
  const e = explodeAnim ? explodeAnim.to : state.explode, off = e * LIFT.lid * 0.5;    // keep the exploded stack centred and in frame
  const dk = groups.dock.visible ? state.dock / DOCK_END : 0;   // mower away from the base: frame both
  const t = [v.target[0] - state.dock * 0.5, v.target[1] + off, v.target[2]];
  const cam = v.cam.map((x, i) => t[i] + (x - v.target[i]) * (1 + 0.5 * e) * (1 + 0.8 * dk));
  flyTo(cam, t);
  clearActiveView(); b.classList.add('active'); b.setAttribute('aria-pressed', 'true');
}));
$('autoRotate').addEventListener('change', (e) => { controls.autoRotate = e.target.checked; requestRender(); });
$('explode').addEventListener('input', (e) => { explodeAnim = null; setExplode(e.target.value / 100); clearActiveView(); });
$('explodeBtn').addEventListener('click', () => { animateExplode(state.explode > 0.5 ? 0 : 1); clearActiveView(); });
for (const [id, k] of [['showLid', 'lid'], ['showCover', 'cover'], ['showBody', 'body'], ['showLower', 'lower']]) $(id).addEventListener('change', (e) => { groups[k].visible = e.target.checked; syncGuides(); requestRender(); });
$('lid').addEventListener('input', (e) => { lidAnim = null; if (lidRig) setLid((e.target.value / 100) * lidRig.open); });
$('lidBtn').addEventListener('click', () => { if (lidRig) animateLid((lidAnim ? lidAnim.to : state.lid) > lidRig.open / 2 ? 0 : lidRig.open); });
$('showBase').addEventListener('change', (e) => { groups.base.visible = e.target.checked; blob.visible = e.target.checked; syncGuides(); requestRender(); });
$('showHotspots').addEventListener('change', requestRender);
$('showDock').addEventListener('change', (e) => {    // without the base the mower simply stands on the ground
  groups.dock.visible = dockShown = e.target.checked; dockAnim = null;
  setDock(state.dock); applyViewOffset();
});
$('dockBtn').addEventListener('click', () => { animateDock((dockAnim ? dockAnim.to : state.dock) > DOCK_END / 2 ? 0 : DOCK_END); clearActiveView(); });
$('ghost').addEventListener('change', (e) => { hotspotGhost = false; setGhost(e.target.checked); });
$('resetWheels').addEventListener('click', () => {
  $('drive').checked = false;
  Object.assign(state.wheel, { L: 0, R: 0 }); Object.assign(state.swivel, { L: 0, R: 0 }); Object.assign(state.spin, { L: 0, R: 0 });
  applyRig();
});
$('drive').addEventListener('change', requestRender);

const deg = (r) => Math.round(THREE.MathUtils.radToDeg(Math.atan2(Math.sin(r), Math.cos(r))));
const SLIDERS = [['wheelL', 'wheel', 'L'], ['wheelR', 'wheel', 'R'], ['casterL', 'swivel', 'L'], ['casterR', 'swivel', 'R']];
for (const [id, k, s] of SLIDERS) {
  $(id).addEventListener('input', (e) => {
    $('drive').checked = false;
    state[k][s] = THREE.MathUtils.degToRad(+e.target.value);
    applyRig();
  });
}
function syncSliders() {
  for (const [id, k, s] of SLIDERS) {
    const el = $(id); const d = deg(state[k][s]);
    if (document.activeElement !== el) el.value = d;
    el.setAttribute('aria-valuetext', `${d} 度`);
    $(`${id}Out`).textContent = d + '°';
  }
}

$('shot').addEventListener('click', () => {
  const cc = renderer.getClearColor(new THREE.Color()), ca = renderer.getClearAlpha();
  renderer.setClearColor(0xe9ebee, 1);           // opaque background for the saved image
  renderer.render(scene, camera);
  canvas.toBlob((b) => {
    const a = document.createElement('a'); a.href = URL.createObjectURL(b); a.download = '割草机截图.png';
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 2000);
  }, 'image/png');
  renderer.setClearColor(cc, ca);
  requestRender();
});

const toggle = $('panelToggle');
function setPanel(open) {
  panel.classList.toggle('collapsed', !open);
  panel.inert = !open;
  toggle.setAttribute('aria-expanded', String(open));
  toggle.textContent = open ? '✕' : '☰';
  applyViewOffset();
}
toggle.addEventListener('click', () => setPanel(panel.classList.contains('collapsed')));
panel.addEventListener('transitionend', applyViewOffset);
setPanel(!mqMobile.matches);
const coarse = window.matchMedia('(pointer: coarse)').matches;
mqMobile.addEventListener('change', (e) => { if (coarse) applyViewOffset(); else setPanel(!e.matches); });
if (window.matchMedia('(pointer: coarse)').matches) $('hint').textContent = '单指拖动旋转\u00a0· 双指缩放/平移\u00a0· 点编号看说明\u00a0· 按住轮子拖动可转动';

// ---------------------------------------------------------------- loop
const clock = new THREE.Clock();
const DRIVE_SPEED = 0.25;            // m/s of the (virtual) ground
function driveStep(dt) {
  for (const s of ['L', 'R']) {
    const w = rig[`wheel${s}`]; if (!w) continue;
    state.wheel[s] += dt * DRIVE_SPEED / w.radius;
    const sw = rig[`swivel${s}`], c = rig[`caster${s}`];
    state.swivel[s] += Math.atan2(Math.sin(-state.swivel[s]), Math.cos(-state.swivel[s])) * Math.min(1, dt * 2.5);  // casters swing to trail
    const axisNow = c.axis.clone().applyAxisAngle(UP, sw.neutral + state.swivel[s]);
    state.spin[s] += dt * (DRIVE_SPEED / c.radius) * axisNow.dot(FWD_SPIN);
  }
  applyRig();
}

function tick() {
  requestAnimationFrame(tick);
  const dt = Math.min(clock.getDelta(), 0.25);   // real elapsed time keeps tweens on schedule at low frame rates
  let active = false;
  if (fly) {
    fly.t += dt / fly.dur; const k = ease(Math.min(1, fly.t));
    controls.target.lerpVectors(fly.t0, fly.t1, k);
    const r = THREE.MathUtils.lerp(fly.s0.radius, fly.s1.radius, k);
    const phi = THREE.MathUtils.lerp(fly.s0.phi, fly.s1.phi, k);
    camera.position.setFromSphericalCoords(r, phi, fly.s0.theta + fly.dth * k).add(controls.target);
    if (fly.t >= 1) fly = null;
    active = true;
  }
  if (explodeAnim) {
    explodeAnim.t += dt / explodeAnim.dur; const k = ease(Math.min(1, explodeAnim.t));
    setExplode(explodeAnim.from + (explodeAnim.to - explodeAnim.from) * k);
    if (explodeAnim.t >= 1) explodeAnim = null;
    active = true;
  }
  if (lidAnim) {
    lidAnim.t += dt / lidAnim.dur; const k = easeIO(Math.min(1, lidAnim.t));
    const a = lidAnim; if (a.t >= 1) lidAnim = null;
    setLid(a.from + (a.to - a.from) * k);
    active = true;
  }
  if (dockAnim) {
    dockAnim.t += dt / dockAnim.dur; const k = easeIO(Math.min(1, dockAnim.t));
    const a = dockAnim; if (a.t >= 1) dockAnim = null;
    setDock(a.from + (a.to - a.from) * k);
    active = true;
  }
  if ($('drive').checked && !drag) { driveStep(dt); active = true; }
  const moved = controls.update(dt);
  const now = performance.now();
  if (moved || active || controls.autoRotate) { lastMotion = now; occlusionDirty = true; }
  if (moved || active || dirty || controls.autoRotate) {
    renderer.render(scene, camera);
    updateOverlays();
    dirty = false;
  }
  if (occlusionDirty && now - lastMotion > 150 && !drag) {   // view at rest: refresh marker occlusion
    occlusionDirty = false;
    computeOcclusion();
    updateOverlays();
  }
}
resize();
tick();
