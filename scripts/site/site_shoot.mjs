// usage: node site_shoot.mjs URL OUTDIR W H [mobile]
import puppeteer from 'puppeteer-core';
const [url, out, W, H, mob] = process.argv.slice(2);
const b = await puppeteer.launch({ executablePath: '/usr/bin/google-chrome', headless: 'new', args: ['--use-angle=gl-egl', '--use-gl=angle', '--ignore-gpu-blocklist', '--enable-gpu'] });
const p = await b.newPage(); await p.setViewport({ width: +W, height: +H, deviceScaleFactor: mob ? 2 : 1, isMobile: !!mob, hasTouch: !!mob });
const logs = []; p.on('console', (m) => { if (m.type() === 'error' || m.type() === 'warning') logs.push(m.type() + ': ' + m.text()); }); p.on('pageerror', (e) => logs.push('PAGEERROR ' + e.message));
p.on('requestfailed', (r) => logs.push('REQFAIL ' + r.url()));
await p.goto(url, { waitUntil: 'networkidle2' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
await sleep(2500);
const total = await p.evaluate(() => document.documentElement.scrollHeight);
const secs = await p.evaluate(() => [...document.querySelectorAll('main > section')].map((s) => ({ cls: s.className, top: s.offsetTop, h: s.offsetHeight })));
console.log('height', total, JSON.stringify(secs));
let i = 0;
const shot = async (y, tag) => { await p.evaluate((y) => window.scrollTo(0, y), y); await sleep(1300); await p.screenshot({ path: `${out}/${String(i++).padStart(2, '0')}_${tag}.png` }); };
for (const s of secs) {
  if (s.h > +H * 1.5) { for (const f of [0.05, 0.35, 0.7, 0.97]) await shot(Math.round(s.top + f * (s.h - +H)), s.cls.split(' ')[0] + '_' + f); }
  else { await shot(s.top, s.cls.split(' ')[0]); if (s.h > +H * 1.05) await shot(s.top + s.h - +H, s.cls.split(' ')[0] + '_end'); }
}
console.log(logs.slice(0, 20).join('\n'));
await b.close();
