// usage: node pages_shoot.mjs BASE OUTDIR  -> screenshot the 3D pages, report errors
import puppeteer from 'puppeteer-core';
const [base, out] = process.argv.slice(2);
const b = await puppeteer.launch({ executablePath: '/usr/bin/google-chrome', headless: 'new', args: ['--use-angle=gl-egl', '--use-gl=angle', '--ignore-gpu-blocklist', '--enable-gpu'] });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
for (const [path, tag] of [['viewer.html', 'viewer'], ['horizon/', 'horizon'], ['original/', 'original']]) {
  const p = await b.newPage(); await p.setViewport({ width: 1440, height: 900 });
  const logs = []; p.on('pageerror', (e) => logs.push('PAGEERROR ' + e.message)); p.on('console', (m) => { if (m.type() === 'error') logs.push('err: ' + m.text()); }); p.on('requestfailed', (r) => logs.push('REQFAIL ' + r.url()));
  await p.goto(base + path, { waitUntil: 'networkidle0', timeout: 120000 }); await sleep(6000);
  await p.screenshot({ path: `${out}/${tag}.png` }); console.log(tag, logs.length ? logs.slice(0, 5).join(' | ') : 'no errors'); await p.close();
}
await b.close();
