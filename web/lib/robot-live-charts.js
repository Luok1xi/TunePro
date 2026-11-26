// SPDX-License-Identifier: GPL-3.0-or-later
// 无上限帧率零 GC 实时示波器：预分配类型化数组列缓冲（零每帧对象分配）+ ResizeObserver + 主题色缓存
// 每像素列保留 [min, max] 极值不丢超调尖峰，重启处自动断开，零第三方依赖。

const TICKS = [0.2, 0.5, 1, 2, 5, 10, 20, 30, 60, 120, 300, 600];

function niceRange(lo, hi) {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [-1, 1];
  if (lo === hi) { const d = Math.abs(lo) * 0.1 || 1; return [lo - d, hi + d]; }
  const pad = (hi - lo) * 0.08;
  return [lo - pad, hi + pad];
}
const fmt = v => Math.abs(v) >= 1000 ? Math.round(v).toString() : Math.abs(v) >= 10 ? v.toFixed(0) : Math.abs(v) >= 1 ? v.toFixed(1) : v.toFixed(2);

function makeColPool(cap) {
  return {
    cap,
    len: 0,
    has: new Uint8Array(cap),
    min: new Float64Array(cap),
    max: new Float64Array(cap),
    first: new Float64Array(cap),
    last: new Float64Array(cap),
    firstRun: new Int32Array(cap),
    lastRun: new Int32Array(cap),
    lo: Infinity,
    hi: -Infinity,
  };
}

function ensurePool(pool, need) {
  if (pool && pool.cap >= need) {
    pool.len = need;
    pool.has.fill(0, 0, need);
    pool.lo = Infinity;
    pool.hi = -Infinity;
    return pool;
  }
  const next = makeColPool(Math.max(need, 256));
  next.len = need;
  return next;
}

export function createChart(canvas, store, { series, fixed = {}, unit = {} }) {
  const ctx = canvas.getContext('2d', { alpha: true });
  let w = 0, h = 0, dpr = 1, cached = null, palette = null;
  const PAD = { l: 58, r: 48, t: 10, b: 18 };

  // Pre-allocated column pools per series (reused across frames -> zero GC churn at 120/240 FPS!)
  const mainPools = series.map(() => null);
  const ghostPools = series.map(() => null);
  const scales = new Float64Array(series.length);

  function measure() {
    const r = canvas.getBoundingClientRect();
    const nextDpr = Math.min(2, devicePixelRatio || 1);
    const nextW = Math.max(10, Math.round(r.width));
    const nextH = Math.max(10, Math.round(r.height));
    if (nextW === w && nextH === h && nextDpr === dpr) return;
    w = nextW; h = nextH; dpr = nextDpr;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    cached = null;
  }

  function resolvePalette() {
    const style = getComputedStyle(canvas);
    const css = name => style.getPropertyValue(name).trim();
    palette = {
      grid: css('--rl-grid') || 'rgba(242,241,237,0.08)',
      text: css('--rl-axis') || '#85837c',
      colors: series.map(s => (s.color.startsWith('--') ? (css(s.color) || '#6aaeff') : s.color)),
    };
  }

  const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(measure) : null;
  ro?.observe(canvas);
  const onTheme = () => { palette = null; };
  window.addEventListener('tunepro-theme', onTheme);
  measure();

  function fillColumns(pool, src, key, k, t0, t1, plotW) {
    const nCols = Math.ceil(plotW) + 1;
    pool = ensurePool(pool, nCols);
    if (!src?.has(key) || t1 <= t0) return pool;
    const invSpan = plotW / (t1 - t0);
    const { has, min, max, first, last, firstRun, lastRun } = pool;
    let run = 0, gap = true, prevSeg = -1, lo = Infinity, hi = -Infinity;
    const size = src.size;
    for (let i = Math.max(0, src.find(t0) - 1); i < size; i++) {
      const t = src.t(i);
      if (t > t1) break;
      const v = src.v(key, i) * k;
      if (!Number.isFinite(v)) { gap = true; continue; }
      const seg = src.seg(i);
      if (gap || seg !== prevSeg) { run++; gap = false; prevSeg = seg; }
      const x = Math.round((t - t0) * invSpan);
      if (x < 0 || x >= nCols) continue;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
      if (!has[x]) {
        has[x] = 1;
        min[x] = max[x] = first[x] = last[x] = v;
        firstRun[x] = lastRun[x] = run;
      } else {
        if (v < min[x]) min[x] = v;
        if (v > max[x]) max[x] = v;
        last[x] = v;
        lastRun[x] = run;
      }
    }
    pool.lo = lo;
    pool.hi = hi;
    return pool;
  }

  function draw({ t0, t1, ghost, hoverT, marks = [] }) {
    if (!w || !h) measure();
    if (!palette) resolvePalette();
    const plotW = Math.max(1, w - PAD.l - PAD.r), plotH = Math.max(1, h - PAD.t - PAD.b);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const { grid, text, colors } = palette;
    ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
    for (let i = 0; i < series.length; i++) scales[i] = series[i].scale ? series[i].scale() : 1;

    const revision = store.version, ghostRevision = ghost?.store.version;
    const reusable = revision !== undefined && (!ghost || ghostRevision !== undefined) && cached
      && cached.revision === revision && cached.ghostStore === ghost?.store && cached.ghostRevision === ghostRevision && cached.offset === ghost?.offset
      && cached.t0 === t0 && cached.t1 === t1 && cached.width === plotW
      && cached.scales.every((k, i) => k === scales[i]);

    if (!reusable) {
      for (let i = 0; i < series.length; i++) {
        const s = series[i], k = scales[i];
        mainPools[i] = fillColumns(mainPools[i], store, s.key, k, t0, t1, plotW);
        ghostPools[i] = ghost ? fillColumns(ghostPools[i], ghost.store, s.key, k, t0 - ghost.offset, t1 - ghost.offset, plotW) : null;
      }
      cached = { revision, ghostStore: ghost?.store, ghostRevision, offset: ghost?.offset, t0, t1, width: plotW, scales: Float64Array.from(scales) };
    }

    const range = {};
    for (const axis of ['l', 'r']) {
      if (fixed[axis]) { range[axis] = fixed[axis]; continue; }
      let lo = Infinity, hi = -Infinity;
      for (let i = 0; i < series.length; i++) {
        if ((series[i].axis || 'l') !== axis) continue;
        const mp = mainPools[i], gp = ghostPools[i];
        if (mp && mp.lo < lo) lo = mp.lo;
        if (mp && mp.hi > hi) hi = mp.hi;
        if (gp && gp.lo < lo) lo = gp.lo;
        if (gp && gp.hi > hi) hi = gp.hi;
      }
      if (lo !== Infinity) range[axis] = niceRange(lo, hi);
    }

    const rL = range.l || [-1, 1], rR = range.r || [-1, 1];
    const scaleL = plotH / (rL[1] - rL[0]), scaleR = plotH / (rR[1] - rR[0]);
    const yL = v => PAD.t + plotH - (v - rL[0]) * scaleL;
    const yR = v => PAD.t + plotH - (v - rR[0]) * scaleR;

    // Batched grid pass (1 stroke for all horizontal + vertical lines)
    ctx.strokeStyle = grid; ctx.lineWidth = 1; ctx.fillStyle = text;
    ctx.beginPath();
    for (let k = 0; k <= 4; k++) { const yy = Math.round(PAD.t + plotH * k / 4) + 0.5; ctx.moveTo(PAD.l, yy); ctx.lineTo(PAD.l + plotW, yy); }
    const span = (t1 - t0) / 1000, step = TICKS.find(s => span / s <= 6) || 600;
    const xTicks = [];
    for (let s = Math.ceil(t0 / 1000 / step) * step; s * 1000 <= t1; s += step) {
      const xx = Math.round(PAD.l + (s * 1000 - t0) / (t1 - t0) * plotW) + 0.5;
      ctx.moveTo(xx, PAD.t); ctx.lineTo(xx, PAD.t + plotH);
      xTicks.push([xx, `${+s.toFixed(1)}s`]);
    }
    ctx.stroke();

    ctx.textAlign = 'center';
    for (let i = 0; i < xTicks.length; i++) ctx.fillText(xTicks[i][1], xTicks[i][0], h - 4);

    for (const axis of ['l', 'r']) {
      const r = range[axis]; if (!r) continue;
      ctx.textAlign = axis === 'l' ? 'right' : 'left';
      const xx = axis === 'l' ? PAD.l - 6 : PAD.l + plotW + 6;
      ctx.fillText(fmt(r[1]) + (unit[axis] ? ` ${unit[axis]}` : ''), xx, PAD.t + 9);
      ctx.fillText(fmt(r[0]), xx, PAD.t + plotH);
      if (r[0] < 0 && r[1] > 0) {
        ctx.save(); ctx.strokeStyle = text; ctx.globalAlpha = 0.32; ctx.setLineDash([2, 3]);
        const zy = Math.round((axis === 'l' ? yL(0) : yR(0))) + 0.5;
        ctx.beginPath(); ctx.moveTo(PAD.l, zy); ctx.lineTo(PAD.l + plotW, zy); ctx.stroke(); ctx.restore();
      }
    }

    const maxGap = Math.max(3, plotW / 40);
    const drawPool = (pool, isRight, color, alpha, dash, width) => {
      if (!pool || pool.lo === Infinity) return;
      const yFn = isRight ? yR : yL;
      const { len, has, min, max, first, last, firstRun, lastRun } = pool;
      ctx.save(); ctx.strokeStyle = color; ctx.globalAlpha = alpha; ctx.lineWidth = width;
      if (dash.length) ctx.setLineDash(dash);
      ctx.lineJoin = 'round';
      ctx.beginPath();
      let prevX = -1, prevRun = -1;
      for (let x = 0; x < len; x++) {
        if (!has[x]) continue;
        const X = PAD.l + x;
        if (prevX >= 0 && prevRun === firstRun[x] && (x - prevX) <= maxGap) ctx.lineTo(X, yFn(first[x]));
        else ctx.moveTo(X, yFn(first[x]));
        if (max[x] !== min[x]) { ctx.lineTo(X, yFn(max[x])); ctx.lineTo(X, yFn(min[x])); }
        ctx.lineTo(X, yFn(last[x]));
        prevX = x;
        prevRun = lastRun[x];
      }
      ctx.stroke(); ctx.restore();
    };

    for (let i = 0; i < series.length; i++) {
      if (ghostPools[i]) drawPool(ghostPools[i], series[i].axis === 'r', colors[i], 0.42, [4, 3], 1.2);
    }
    for (let i = 0; i < series.length; i++) {
      drawPool(mainPools[i], series[i].axis === 'r', colors[i], 1, series[i].dash || [], 1.7);
    }

    if (marks.length) {
      ctx.save(); ctx.strokeStyle = text; ctx.lineWidth = 1; ctx.setLineDash([2, 3]);
      ctx.beginPath();
      for (const t of marks) {
        if (t < t0 || t > t1) continue;
        const x = PAD.l + (t - t0) / (t1 - t0) * plotW;
        ctx.moveTo(x, PAD.t); ctx.lineTo(x, PAD.t + plotH);
      }
      ctx.stroke(); ctx.restore();
    }

    const readout = [];
    if (Number.isFinite(hoverT) && hoverT >= t0 && hoverT <= t1) {
      const xx = PAD.l + (hoverT - t0) / (t1 - t0) * plotW;
      ctx.save(); ctx.strokeStyle = text; ctx.globalAlpha = 0.55; ctx.beginPath(); ctx.moveTo(xx, PAD.t); ctx.lineTo(xx, PAD.t + plotH); ctx.stroke(); ctx.restore();
      for (let i = 0; i < series.length; i++) {
        const s = series[i];
        const v = nearest(store, s.key, hoverT) * scales[i];
        if (!Number.isFinite(v)) continue;
        ctx.fillStyle = colors[i];
        ctx.beginPath();
        ctx.arc(xx, (s.axis === 'r' ? yR(v) : yL(v)), 3.2, 0, 6.2832);
        ctx.fill();
        readout.push({ label: s.label, v, color: s.color });
      }
    }
    return { readout, plot: { left: PAD.l, width: plotW } };
  }

  function destroy() {
    ro?.disconnect();
    window.removeEventListener('tunepro-theme', onTheme);
  }
  return { draw, destroy };
}

export function nearest(src, key, t) {
  if (!src?.has(key)) return NaN;
  const i = src.find(t);
  let best = NaN, distance = Infinity;
  for (const direction of [-1, 1]) {
    for (let d = 0; d < 400; d++) {
      const j = direction < 0 ? i - 1 - d : i + d;
      if (j < 0 || j >= src.size) break;
      const v = src.v(key, j); if (!Number.isFinite(v)) continue;
      const delta = Math.abs(src.t(j) - t);
      if (delta < distance) { best = v; distance = delta; }
      break;
    }
  }
  return best;
}
