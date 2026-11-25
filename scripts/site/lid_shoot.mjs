// usage: node lid_shoot.mjs URL OUTDIR  -> lid section at three scroll points, three viewports
import puppeteer from 'puppeteer-core';
const [url, out] = process.argv.slice(2);
const b = await puppeteer.launch({ executablePath: '/usr/bin/google-chrome', headless: 'new' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
for (const [W, H, mob] of [[390, 844, 1], [768, 1024, 1], [1440, 900, 0]]) {
  const p = await b.newPage(); await p.setViewport({ width: W, height: H, deviceScaleFactor: mob ? 2 : 1, isMobile: !!mob, hasTouch: !!mob });
  await p.goto(url, { waitUntil: 'networkidle2' }); await sleep(800);
  const s = await p.evaluate(() => { const e = document.getElementById('lid'); return { top: e.offsetTop, h: e.offsetHeight }; });
  for (const f of [0.1, 0.5, 0.9]) { await p.evaluate((y) => scrollTo(0, y), Math.round(s.top + f * (s.h - H))); await sleep(1500); await p.screenshot({ path: `${out}/lid_${W}_${f}.png` }); }
  await p.close();
}
await b.close();
