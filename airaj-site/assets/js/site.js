// AIRAJ R1 site: colour cards, technology tabs, help-me-choose estimator, mobile menu, reveal, register form.
(() => {
  const $ = (s, r = document) => r.querySelector(s), $$ = (s, r = document) => [...r.querySelectorAll(s)];
  const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;

  // ---- colour cards (renders: assets/img/r1-<id>.webp; colours as in the original palettes)
  const COLOURS = [
    { id: 'black', name: 'Obsidian Black', fam: 'classic', tag: 'Most popular', sub: 'Gloss black shell, graphite body, cyan wheel light.', c: ['#212123', '#4A4D52', '#2FC6DC'] },
    { id: 'silver', name: 'Silver Arrow', fam: 'classic', sub: 'Metallic silver over graphite, teal wheel light.', c: ['#B5BBC1', '#2B2F34', '#00CDBE'] },
    { id: 'stripes', name: 'Le Mans Stripes', fam: 'classic', tag: 'Signature', sub: 'Gloss white with twin blue stripes, navy body.', c: ['#ECEDEA', '#2A4C93', '#1D2840'] },
    { id: 'yellow', name: 'Signal Yellow', fam: 'bold', sub: 'High-visibility gloss yellow, charcoal body.', c: ['#F4D014', '#2E2C2A', '#FFD23F'] },
    { id: 'blue', name: 'Sapphire Blue', fam: 'bold', sub: 'Metallic sapphire over graphite, blue wheel light.', c: ['#1A47B0', '#26282C', '#2F6BFF'] },
    { id: 'red', name: 'Crimson', fam: 'bold', sub: 'Deep gloss red, graphite body, ice-blue wheel light.', c: ['#B8102E', '#2A2B2D', '#4FC3FF'] },
    { id: 'brg', name: 'Racing Green', fam: 'bold', sub: 'British racing green with a cream body.', c: ['#1E4532', '#E6D9B8', '#3DDC97'] },
    { id: 'sage', name: 'Sage & Pine', fam: 'natural', sub: 'Soft sage shell, pine-green body.', c: ['#829474', '#2D3631', '#B4EC8E'] },
    { id: 'sand', name: 'Rally Sand', fam: 'natural', sub: 'Warm sand shell, dark stone body.', c: ['#BCAA88', '#35332F', '#2FC6DC'] },
    { id: 'taro', name: 'Taro Lilac', fam: 'natural', tag: 'New', sub: 'Pale lilac shell, plum body, violet wheel light.', c: ['#AC93D3', '#3D2E52', '#9B6BFF'] },
  ];
  const cards = $('#cards');
  if (cards) {
    cards.innerHTML = COLOURS.map((k) => `
      <article class="card rv" data-fam="${k.fam}">
        ${k.tag ? `<span class="card-tag">${k.tag}</span>` : '<span class="card-tag">&nbsp;</span>'}
        <div class="card-img"><img src="assets/img/r1-${k.id}.webp" alt="AIRAJ R1 in ${k.name}" width="1200" height="800" loading="lazy"></div>
        <h3>R1 &middot; ${k.name}</h3>
        <p class="sub">${k.sub}</p>
        <div class="dots" aria-hidden="true">${k.c.map((c) => `<i style="--c:${c}"></i>`).join('')}</div>
        <p class="price">Price announced at launch</p>
        <div class="acts"><a class="btn btn-line" href="#specs">View specs</a><a class="btn btn-orange" href="#register">Register</a></div>
      </article>`).join('');
  }
  $$('.tabs [role=tab]').forEach((t) => t.addEventListener('click', () => {
    $$('.tabs [role=tab]').forEach((x) => x.setAttribute('aria-selected', String(x === t)));
    const f = t.dataset.fam; $$('.card', cards).forEach((c) => { c.hidden = !(f === 'all' || c.dataset.fam === f); });
  }));

  // ---- technology tabs (click, arrow keys)
  const ttabs = $$('.tech-tabs [role=tab]');
  const show = (t) => {
    ttabs.forEach((x) => { const on = x === t; x.setAttribute('aria-selected', String(on)); x.tabIndex = on ? 0 : -1; const p = document.getElementById(x.getAttribute('aria-controls')); p.hidden = !on; p.classList.toggle('on', on); });
  };
  ttabs.forEach((t, i) => {
    t.tabIndex = i === 0 ? 0 : -1;
    t.addEventListener('click', () => show(t));
    t.addEventListener('keydown', (e) => { const d = { ArrowRight: 1, ArrowLeft: -1 }[e.key]; if (!d) return; e.preventDefault(); const n = ttabs[(i + d + ttabs.length) % ttabs.length]; show(n); n.focus(); });
  });
  // links to the dock image open its tab first
  $$('a[href="#tech-dock"]').forEach((a) => a.addEventListener('click', () => show($('#tt-dock'))));

  // ---- help me choose (design targets: 130 m2/h, 90 min per charge, 35 % slope, 1000 m2)
  const area = $('#area'), cuts = $('#cuts');
  const fmtH = (h) => (h < 1 ? `${Math.max(10, Math.round(h * 60 / 5) * 5)} min` : `${Math.round(h * 2) / 2} h`);
  function est() {
    if (!area) return;
    const a = +area.value, n = +cuts.value, slope = +($('input[name=slope]:checked') || { value: 0 }).value;
    $('#areaOut').textContent = `${a.toLocaleString('en-GB')} m²`; $('#cutsOut').textContent = n;
    const one = a / 130, charges = Math.max(1, Math.ceil(one / 1.5));
    $('#oneCut').textContent = fmtH(one); $('#charges').textContent = charges; $('#weekly').textContent = fmtH(one * n);
    const v = $('#verdict'); let msg = 'R1 is a good fit.', warn = false;
    if (slope > 35) { msg = 'Too steep for R1: it is designed for slopes up to 35%.'; warn = true; }
    else if (a > 1000) { msg = 'Bigger than R1 is designed for (1,000 m²). Expect long mowing days.'; warn = true; }
    else if (one * n > 7 * 10) { msg = 'That is a lot of mowing for one R1. Try fewer cuts per week.'; warn = true; }
    else if (slope === 35) msg = 'R1 is a good fit. Steep parts take a little longer.';
    v.textContent = msg; v.classList.toggle('warn', warn);
  }
  [area, cuts, ...$$('input[name=slope]')].forEach((el) => el && el.addEventListener('input', est)); est();

  // ---- mobile menu
  const burger = $('#burger'), nav = $('#nav');
  const setMenu = (open) => { burger.setAttribute('aria-expanded', String(open)); burger.setAttribute('aria-label', open ? 'Close menu' : 'Open menu'); nav.classList.toggle('open', open); };
  burger.addEventListener('click', () => setMenu(burger.getAttribute('aria-expanded') !== 'true'));
  $$('a', nav).forEach((a) => a.addEventListener('click', () => setMenu(false)));
  addEventListener('keydown', (e) => { if (e.key === 'Escape') setMenu(false); });

  // ---- counters + reveal
  $$('.sec-head, .steps li, .why-grid li, .tp, .spec-groups dl, .faq details, .num').forEach((el) => el.classList.add('rv'));
  const io = new IntersectionObserver((es) => es.forEach((e) => { if (e.isIntersecting) { e.target.classList.add('in'); io.unobserve(e.target); } }), { threshold: 0.12 });
  $$('.rv').forEach((el) => (reduce ? el.classList.add('in') : io.observe(el)));
  const cio = new IntersectionObserver((es) => es.forEach((e) => {
    if (!e.isIntersecting) return; cio.unobserve(e.target); const el = e.target, to = +el.dataset.to;
    if (reduce || !to) { el.textContent = to.toLocaleString('en-GB'); return; }
    const t0 = performance.now(); const step = (t) => { const k = Math.min(1, (t - t0) / 1200); el.textContent = Math.round(to * (1 - Math.pow(1 - k, 3))).toLocaleString('en-GB'); if (k < 1) requestAnimationFrame(step); };
    requestAnimationFrame(step);
  }), { threshold: 0.6 });
  $$('.num b[data-to]').forEach((el) => cio.observe(el));

  // ---- register form: no sign-up service is connected yet, so nothing is sent or stored
  const form = $('#regForm');
  if (form) form.addEventListener('submit', (e) => {
    e.preventDefault(); const msg = $('#regMsg'), em = $('#email'), ok = $('#consent');
    if (!em.value || !em.checkValidity()) { msg.textContent = 'Please enter a valid e-mail address.'; em.focus(); return; }
    if (!ok.checked) { msg.textContent = 'Please tick the box to agree to receive e-mails.'; ok.focus(); return; }
    msg.textContent = 'Online registration opens soon. Nothing has been sent yet: please check back at launch.';
  });
})();
