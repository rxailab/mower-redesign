// Orbit product page: scroll-driven image sequences, the lawn-path drawing, reveals, counters, colour picker.
(() => {
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
  const vh = () => window.innerHeight;
  const pad2 = (i) => String(i).padStart(2, '0');

  // ---------------------------------------------- reveal on scroll
  const io = new IntersectionObserver((es) => es.forEach((e) => { if (e.isIntersecting) { e.target.classList.add('in'); io.unobserve(e.target); } }),
    { threshold: 0.15, rootMargin: '0px 0px -6% 0px' });
  document.querySelectorAll('.reveal').forEach((el) => io.observe(el));

  // ---------------------------------------------- counters
  const cio = new IntersectionObserver((es) => es.forEach((e) => {
    if (!e.isIntersecting) return; cio.unobserve(e.target);
    const el = e.target, to = parseFloat(el.dataset.to), dec = +(el.dataset.dec || 0);
    if (reduce) { el.textContent = to.toFixed(dec); return; }
    const t0 = performance.now(), dur = 1400;
    const step = (t) => { const k = clamp((t - t0) / dur), v = to * (1 - Math.pow(1 - k, 3)); el.textContent = v.toFixed(dec); if (k < 1) requestAnimationFrame(step); };
    el.textContent = (0).toFixed(dec); requestAnimationFrame(step);
  }), { threshold: 0.6 });
  document.querySelectorAll('.num[data-to]').forEach((el) => cio.observe(el));

  // ---------------------------------------------- image sequences on a sticky canvas
  const seqs = [...document.querySelectorAll('.seq')].map((sec) => {
    const n = +sec.dataset.frames, dir = sec.dataset.seq, canvas = sec.querySelector('canvas'), ctx = canvas.getContext('2d');
    return { sec, n, dir, canvas, ctx, imgs: new Array(n), loaded: new Array(n).fill(false), started: false, cur: -1, want: 0, caps: [...sec.querySelectorAll('.cap')] };
  });
  function load(s) {
    if (s.started) return; s.started = true;
    const order = []; for (let st = 8; st >= 1; st = Math.floor(st / 2)) for (let i = 0; i < s.n; i += st) if (!order.includes(i)) order.push(i);   // coarse first, then fill in
    order.forEach((i) => { const im = new Image(); im.decoding = 'async'; im.onload = () => { s.loaded[i] = true; if (s.cur < 0 || Math.abs(i - s.want) < Math.abs(s.cur - s.want)) draw(s, s.want, true); }; im.src = `site/img/${s.dir}/${pad2(i)}.webp`; s.imgs[i] = im; });
  }
  function sizeCanvas(s) {
    const r = s.canvas.getBoundingClientRect(), dpr = Math.min(devicePixelRatio || 1, 2);
    const w = Math.round(r.width * dpr), h = Math.round(r.height * dpr);
    if (s.canvas.width !== w || s.canvas.height !== h) { s.canvas.width = w; s.canvas.height = h; s.cur = -1; }
  }
  function draw(s, want, force) {
    s.want = want; let k = -1, best = 1e9;
    for (let i = 0; i < s.n; i++) if (s.loaded[i] && Math.abs(i - want) < best) { best = Math.abs(i - want); k = i; }
    if (k < 0 || (k === s.cur && !force)) return;
    const im = s.imgs[k], cw = s.canvas.width, ch = s.canvas.height, sc = Math.min(cw / im.naturalWidth, ch / im.naturalHeight);
    const w = im.naturalWidth * sc, h = im.naturalHeight * sc;
    s.ctx.clearRect(0, 0, cw, ch); s.ctx.drawImage(im, (cw - w) / 2, (ch - h) / 2, w, h); s.cur = k;
  }
  const near = new IntersectionObserver((es) => es.forEach((e) => { if (e.isIntersecting) { const s = seqs.find((q) => q.sec === e.target); if (s) load(s); } }), { rootMargin: '150% 0px' });
  seqs.forEach((s) => near.observe(s.sec));

  function progress(sec) { const r = sec.getBoundingClientRect(); return clamp(-r.top / Math.max(1, r.height - vh())); }
  function caps(list, p) {
    for (const c of list) { const [a, b] = c.dataset.range.split(',').map(Number); c.classList.toggle('show', p >= a && p <= b); }
  }

  // ---------------------------------------------- lawn path: edge loop first, then parallel rows, around the flower bed
  const lawnSec = document.getElementById('path'), lawn = document.getElementById('lawn');
  let lawnSegs = [], lawnTotal = 0, mower = null, nogo = null, lawnCaps = [];
  if (lawnSec && lawn) {
    const NS = 'http://www.w3.org/2000/svg';
    // lawn outline, clockwise on screen; the notch at the top right is the flower bed
    const P = [[96, 150], [180, 104], [330, 82], [520, 74], [700, 80], [700, 246], [930, 246], [950, 330], [946, 450], [912, 532],
      [760, 560], [520, 566], [300, 560], [150, 540], [84, 470], [70, 340], [76, 230]];
    const dOf = (pts, close) => 'M' + pts.map((p) => p[0].toFixed(1) + ' ' + p[1].toFixed(1)).join(' L') + (close ? ' Z' : '');
    document.getElementById('lawnShape').setAttribute('d', dOf(P, true));
    document.getElementById('lawnClipPath').setAttribute('d', dOf(P, true));
    const offset = (pts, o) => {                      // inward offset (mitred); inward normal of a clockwise screen polygon is (-dy, dx)
      const n = pts.length, lines = [];
      for (let i = 0; i < n; i++) {
        const a = pts[i], b = pts[(i + 1) % n], dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy), nx = -dy / L, ny = dx / L;
        lines.push([[a[0] + nx * o, a[1] + ny * o], [dx, dy]]);
      }
      return pts.map((_, i) => {
        const [p1, d1] = lines[(i - 1 + n) % n], [p2, d2] = lines[i], den = d1[0] * d2[1] - d1[1] * d2[0];
        if (Math.abs(den) < 1e-9) return p2;
        const t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den;
        return [p1[0] + d1[0] * t, p1[1] + d1[1] * t];
      });
    };
    const BAND = 44, ROW = 26;
    const E = offset(P, BAND / 2), I = offset(P, BAND - 2);
    // 1 from the dock up the left side to the top-left corner, then once round the edge
    const start = [E[15][0] + 4, 346];
    const edge = [start, E[16], ...E, E[0]];
    // 2 rows inside the edge band, alternating direction
    const xsAt = (poly, y) => {
      const xs = [];
      for (let i = 0; i < poly.length; i++) {
        const a = poly[i], b = poly[(i + 1) % poly.length];
        if ((a[1] - y) * (b[1] - y) < 0) xs.push(a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]));
      }
      return xs.sort((p, q) => p - q);
    };
    const yTop = Math.min(...I.map((p) => p[1])) + ROW / 2, yBot = Math.max(...I.map((p) => p[1])) - ROW / 2;
    const segs = [{ d: dOf(edge), cls: 'edge', w: BAND }];
    let prev = E[0], k = 0;
    for (let y = yTop; y <= yBot + 0.1; y += ROW, k++) {
      const xs = xsAt(I, y); if (xs.length < 2) continue;
      const a = [xs[0] + ROW / 2, y], b = [xs[xs.length - 1] - ROW / 2, y], row = k % 2 ? [b, a] : [a, b];
      segs.push({ d: dOf([prev, ...row]), cls: k % 2 ? 'rb' : 'ra', w: ROW + 2 });
      prev = row[1];
    }
    const g = document.getElementById('mowed');
    lawnSegs = segs.map((s) => {
      const el = document.createElementNS(NS, 'path'); el.setAttribute('d', s.d); el.setAttribute('class', s.cls); el.setAttribute('stroke-width', s.w);
      g.appendChild(el); const len = el.getTotalLength(); el.style.strokeDasharray = `${len} ${len + 1}`; el.style.strokeDashoffset = len;
      const r = { el, len, c0: lawnTotal }; lawnTotal += len; return r;
    });
    mower = document.getElementById('mower'); nogo = document.getElementById('nogo'); lawnCaps = [...lawnSec.querySelectorAll('.cap')];
  }
  let lawnLast = -1;
  function drawLawn(p) {
    const D = clamp((p - 0.02) / 0.9) * lawnTotal; if (Math.abs(D - lawnLast) < 0.5) return; lawnLast = D;
    let at = null;
    for (const s of lawnSegs) {
      const vis = clamp(D - s.c0, 0, s.len); s.el.style.strokeDashoffset = s.len - vis;
      if (!at && D <= s.c0 + s.len) at = { s, l: vis };
    }
    if (!at) { const s = lawnSegs[lawnSegs.length - 1]; at = { s, l: s.len }; }
    const p1 = at.s.el.getPointAtLength(at.l), p0 = at.s.el.getPointAtLength(Math.max(0, at.l - 6)), p2 = at.s.el.getPointAtLength(Math.min(at.s.len, at.l + 6));
    const ang = Math.atan2(p2.y - p0.y, p2.x - p0.x) * 180 / Math.PI;
    mower.setAttribute('transform', `translate(${p1.x.toFixed(1)} ${p1.y.toFixed(1)}) rotate(${(isFinite(ang) ? ang : -90).toFixed(1)})`);
  }

  // ---------------------------------------------- nav colour over dark sections
  const nav = document.getElementById('gnav'), darks = [...document.querySelectorAll('.dark')];
  const hero = document.getElementById('heroShot');

  let ticking = false;
  function frame() {
    ticking = false;
    for (const s of seqs) {
      const r = s.sec.getBoundingClientRect(); if (r.bottom < -vh() || r.top > 2 * vh()) continue;
      sizeCanvas(s); const p = progress(s.sec); draw(s, Math.round(p * (s.n - 1))); caps(s.caps, p);
    }
    if (lawnSegs.length) {
      const r = lawnSec.getBoundingClientRect();
      if (r.bottom > -vh() && r.top < 2 * vh()) { const p = progress(lawnSec); drawLawn(p); caps(lawnCaps, p); nogo.classList.toggle('on', p >= 0.72); }
    }
    if (hero && !reduce) { const y = window.scrollY; if (y < vh() * 1.5) hero.style.transform = `translate3d(0, ${(y * 0.12).toFixed(1)}px, 0) scale(${(1 - y * 0.00008).toFixed(4)})`; }
    const onDark = darks.some((d) => { const r = d.getBoundingClientRect(); return r.top <= 26 && r.bottom >= 26; });
    nav.classList.toggle('on-dark', onDark);
  }
  const req = () => { if (!ticking) { ticking = true; requestAnimationFrame(frame); } };
  addEventListener('scroll', req, { passive: true }); addEventListener('resize', req); req();

  // ---------------------------------------------- colour picker (cross-fade)
  const img = document.getElementById('colorImg'), img2 = document.getElementById('colorImg2'), sws = [...document.querySelectorAll('.sw')];
  const names = {}; sws.forEach((b) => { names[b.dataset.img] = b.textContent.trim(); const p = new Image(); p.src = `site/img/${b.dataset.img}.webp`; });
  function pick(b) {
    sws.forEach((x) => { const on = x === b; x.classList.toggle('on', on); x.setAttribute('aria-checked', String(on)); });
    const src = `site/img/${b.dataset.img}.webp`; if (img.getAttribute('src') === src) return;
    img2.onload = () => { img2.style.opacity = 1; setTimeout(() => { img.src = src; img.alt = `${names[b.dataset.img]}配色`; img2.style.transition = 'none'; img2.style.opacity = 0; void img2.offsetWidth; img2.style.transition = ''; }, 620); };
    img2.src = src;
  }
  sws.forEach((b, i) => {
    b.addEventListener('click', () => pick(b));
    b.addEventListener('keydown', (e) => { const d = { ArrowRight: 1, ArrowLeft: -1 }[e.key]; if (!d) return; e.preventDefault(); const n = sws[(i + d + sws.length) % sws.length]; pick(n); n.focus(); });
  });
})();
