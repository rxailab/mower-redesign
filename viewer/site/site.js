// Orbit product page: scroll-driven image sequences, exploded layers, text scrub, reveals, counters, colour picker.
(() => {
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
  const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
  const smooth = (x) => x * x * (3 - 2 * x);
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

  // ---------------------------------------------- text scrub (characters light up while the paragraph scrolls past)
  const scrub = document.getElementById('scrub'); let chars = [];
  if (scrub) {
    const txt = scrub.textContent; scrub.textContent = '';
    for (const ch of txt) { const s = document.createElement('span'); s.textContent = ch; scrub.appendChild(s); chars.push(s); }
  }

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

  // ---------------------------------------------- exploded layers
  const ex = document.querySelector('.explode'), layers = ex ? [...ex.querySelectorAll('.layer')] : [], exCaps = ex ? [...ex.querySelectorAll('.cap')] : [];
  const LIFT = [0, 0.2, 0.42, 0.62];          // fraction of the layer box height each layer rises at full explode

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
    if (ex) {
      const r = ex.getBoundingClientRect();
      if (r.bottom > -vh() && r.top < 2 * vh()) {
        const p = progress(ex), q = smooth(clamp((p - 0.1) / 0.55)), H = layers[0].getBoundingClientRect().height;
        layers.forEach((l, i) => { l.style.transform = `translate3d(0, ${(-LIFT[i] * q * H).toFixed(1)}px, 0)`; });
        caps(exCaps, p);
      }
    }
    if (chars.length) {
      const r = scrub.getBoundingClientRect(), p = clamp((vh() * 0.85 - r.top) / (r.height + vh() * 0.45)), n = Math.round(p * chars.length);
      chars.forEach((c, i) => c.classList.toggle('lit', i < n));
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
