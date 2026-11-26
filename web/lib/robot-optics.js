// SPDX-License-Identifier: GPL-3.0-or-later
// 高性能物理动效与光学引擎（静止 0% CPU，零空闲 rAF 循环）：
//   1. .tp-optic-btn      — 318° 顶部物理光源折射边 + 悬停光泽池 + 按压水波纹
//   2. .tp-glass          — 轻量化指针高光坐标 (--mx / --my)，一帧合并一次写
//   3. mountFluidTabs()   — 分段/标签栏真实弹簧滑动指示器（只动 transform，不触发布局重排）
//   4. mountEffortSlider()— 2 阶阻尼弹簧 5 档磁吸胶囊滑块（带拖动速度液态挤压形变 + 滚动文字 + 充能闪电）

const reduceMedia = matchMedia('(prefers-reduced-motion: reduce)');
const finePointer = matchMedia('(hover: hover) and (pointer: fine)');
const enhancedBtns = new WeakSet();
const enhancedGlass = new WeakSet();
const enhancedTabs = new WeakSet();
const REST_ANGLE = 318;

export function motionOff() {
  return reduceMedia.matches || document.documentElement.dataset.motion === 'off';
}

/* ---------- 1. 物理折射边主按钮 (.tp-optic-btn) ---------- */
function enhanceOpticButton(el) {
  if (!el || enhancedBtns.has(el)) return;
  enhancedBtns.add(el);
  if (!el.querySelector(':scope > .tp-optic-label')) {
    const lbl = document.createElement('span');
    lbl.className = 'tp-optic-label';
    lbl.append(...el.childNodes);
    el.append(lbl);
  }
  el.insertAdjacentHTML(
    'afterbegin',
    '<span class="tp-optic-pool-wrap" aria-hidden="true"><i class="tp-optic-pool"></i></span>'
    + '<span class="tp-optic-rim" aria-hidden="true"><i></i></span>',
  );
  const poolWrap = el.firstElementChild;
  const pool = poolWrap.firstElementChild;
  const lobe = el.querySelector('.tp-optic-rim > i');
  let rect = null, raf = 0, px = 0, py = 0, over = false, press = false;

  const disabled = () => el.matches(':disabled, [aria-disabled="true"]');
  const sync = () => {
    el.classList.toggle('is-hot', over || press);
    el.classList.toggle('is-press', press);
    if (!over && !press) {
      pool.style.translate = '';
      lobe.style.rotate = '';
    }
  };
  const aim = () => {
    raf = 0;
    if (!rect || motionOff() || !(over || press)) return;
    const nx = Math.max(-1, Math.min(1, (px - rect.left - rect.width / 2) / Math.max(1, rect.width / 2)));
    const ny = Math.max(-1, Math.min(1, (py - rect.top - rect.height / 2) / Math.max(1, rect.height / 2)));
    lobe.style.rotate = `${(REST_ANGLE + nx * 18 + ny * 9).toFixed(1)}deg`;
    pool.style.translate = `${(nx * rect.width * 0.22).toFixed(1)}px 0`;
  };
  const onMove = e => { px = e.clientX; py = e.clientY; if (!raf) raf = requestAnimationFrame(aim); };
  const spawnRipple = (x, y) => {
    if (motionOff()) return;
    const r = document.createElement('i');
    r.className = 'tp-optic-ripple';
    r.style.cssText = `--rx:${x.toFixed(1)}px;--ry:${y.toFixed(1)}px`;
    const clean = () => r.remove();
    r.addEventListener('animationend', clean, { once: true });
    setTimeout(clean, 850);
    poolWrap.append(r);
  };

  el.addEventListener('pointerenter', e => {
    if (e.pointerType !== 'mouse' || disabled()) return;
    rect = el.getBoundingClientRect();
    over = true; sync(); onMove(e);
  });
  el.addEventListener('pointermove', onMove, { passive: true });
  el.addEventListener('pointerleave', e => { if (e.pointerType === 'mouse') { over = false; sync(); } });
  el.addEventListener('pointerdown', e => {
    if (disabled() || e.button > 0) return;
    rect = el.getBoundingClientRect();
    press = true; sync(); onMove(e);
    spawnRipple(e.clientX - rect.left, e.clientY - rect.top);
    const up = () => { press = false; sync(); };
    addEventListener('pointerup', up, { once: true });
    addEventListener('pointercancel', up, { once: true });
  });
}

/* ---------- 2. 玻璃镜面轻高光 (.tp-glass) ---------- */
function enhanceGlassSurface(el) {
  if (!el || enhancedGlass.has(el) || !finePointer.matches) return;
  enhancedGlass.add(el);
  let raf = 0, px = 0, py = 0, rect = null;
  const paint = () => {
    raf = 0;
    if (motionOff() || !rect) return;
    el.style.setProperty('--mx', `${(px - rect.left).toFixed(0)}px`);
    el.style.setProperty('--my', `${(py - rect.top).toFixed(0)}px`);
  };
  el.addEventListener('pointerenter', () => { rect = el.getBoundingClientRect(); }, { passive: true });
  el.addEventListener('pointermove', e => {
    if (e.pointerType !== 'mouse' || motionOff()) return;
    px = e.clientX; py = e.clientY;
    if (!rect) rect = el.getBoundingClientRect();
    if (!raf) raf = requestAnimationFrame(paint);
  }, { passive: true });
  el.addEventListener('pointerleave', () => {
    rect = null;
    el.style.removeProperty('--mx');
    el.style.removeProperty('--my');
  });
}

/* ---------- 3. 弹簧滑动指示底板 / 下划线（真实宽度 + GPU 平移，零圆角/描边畸变） ---------- */
export function mountFluidTabs(container, { selector = 'button', activeSelector = '[aria-selected="true"], [aria-pressed="true"]', kind = 'line' } = {}) {
  if (!container || enhancedTabs.has(container)) return { sync() {} };
  enhancedTabs.add(container);
  container.classList.add('tp-fluid-group', `tp-fluid-${kind}`);
  const ind = document.createElement('span');
  ind.className = 'tp-fluid-ind';
  ind.setAttribute('aria-hidden', 'true');
  container.prepend(ind);

  let ready = false;
  let lastContainerW = 0;
  let lastSwitchTs = 0;
  const sync = (instant = false) => {
    const active = container.querySelector(`${selector}:is(${activeSelector})`);
    if (!active || !active.offsetWidth) {
      ind.style.opacity = '0';
      lastContainerW = container.clientWidth;
      return;
    }
    const x = active.offsetLeft;
    const w = active.offsetWidth;
    if (instant || !ready || motionOff()) {
      ind.style.transition = 'none';
      ind.style.width = `${w}px`;
      ind.style.transform = `translate3d(${x}px, 0, 0)`;
      ind.style.opacity = '1';
      ind.getBoundingClientRect();
      ind.style.transition = '';
      ready = true;
    } else {
      lastSwitchTs = performance.now();
      ind.style.width = `${w}px`;
      ind.style.transform = `translate3d(${x}px, 0, 0)`;
      ind.style.opacity = '1';
    }
    lastContainerW = container.clientWidth;
  };

  const ro = typeof ResizeObserver !== 'undefined'
    ? new ResizeObserver(() => {
        const curW = container.clientWidth;
        const deltaW = Math.abs(curW - lastContainerW);
        // Never kill an active tab-switch slide due to minor font-weight or scrollbar reflow
        const majorResize = !ready || (deltaW > 24 && (performance.now() - lastSwitchTs) > 280);
        sync(majorResize);
      })
    : null;
  ro?.observe(container);
  const mo = new MutationObserver(() => sync(false));
  mo.observe(container, { subtree: true, attributes: true, attributeFilter: ['aria-selected', 'aria-pressed'] });
  document.fonts?.ready?.then(() => {
    if ((performance.now() - lastSwitchTs) > 280) sync(true);
  });
  requestAnimationFrame(() => sync(true));
  return { sync };
}

/* ---------- 4. 2 阶高刚度跟手弹簧 5 档磁吸推理强度胶囊滑块（含 Max 超频冲击波） ---------- */
export function mountEffortSlider(trackEl, { levels = 5, initialIndex = 2, onChange, onSnap, onMaxHit } = {}) {
  const fillEl = trackEl.querySelector('[data-eff-fill]');
  const thumbEl = trackEl.querySelector('[data-eff-thumb]');
  const dots = [...trackEl.querySelectorAll('[data-eff-dot]')];

  let idx = Math.max(0, Math.min(levels - 1, initialIndex));
  let pos = idx / (levels - 1);
  let vel = 0;
  let targetPos = pos;
  let dragging = false;
  let raf = 0;
  let lastTs = 0;
  let trackRect = null;

  const stepSpring = (dt, stiffness, damping) => {
    const disp = pos - targetPos;
    const accel = -stiffness * disp - damping * vel;
    vel += accel * dt;
    pos += vel * dt;
    if (Math.abs(pos - targetPos) < 0.0005 && Math.abs(vel) < 0.002) {
      pos = targetPos;
      vel = 0;
      return false;
    }
    return true;
  };

  const render = () => {
    const w = trackEl.clientWidth || 260;
    const thumbSize = 30;
    const pad = 4;
    const travel = Math.max(1, w - thumbSize - pad * 2);
    const clamped = Math.max(-0.02, Math.min(1.02, pos));
    const tx = pad + clamped * travel;
    const fillW = Math.max(thumbSize + pad * 2, tx + thumbSize + pad);

    // Snappy liquid jelly squish proportional to velocity
    const speed = Math.min(1, Math.abs(vel) * 0.18);
    const sx = dragging ? 1 + speed * 0.24 : 1 + speed * 0.14;
    const sy = dragging ? 1 - speed * 0.12 : 1 - speed * 0.07;

    fillEl.style.width = `${Math.max(36, Math.min(w, fillW)).toFixed(1)}px`;
    thumbEl.style.transform = `translate3d(${tx.toFixed(2)}px, 0, 0) scale(${sx.toFixed(3)}, ${sy.toFixed(3)})`;

    for (let i = 0; i < dots.length; i++) {
      const dotNorm = i / (levels - 1);
      dots[i].dataset.filled = String(dotNorm <= clamped + 0.03);
      dots[i].dataset.active = String(i === idx);
    }
  };

  const loop = ts => {
    raf = 0;
    const dt = Math.min(0.032, Math.max(0.003, (ts - (lastTs || ts - 8)) / 1000));
    lastTs = ts;
    // High-stiffness critically-fast spring -> instant finger tracking, zero mushy lag!
    const moving = motionOff()
      ? ((pos = targetPos), (vel = 0), false)
      : stepSpring(dt, dragging ? 960 : 680, dragging ? 48 : 38);
    render();
    if (moving || dragging) raf = requestAnimationFrame(loop);
  };

  const wake = () => {
    if (!raf) { lastTs = 0; raf = requestAnimationFrame(loop); }
  };

  const notifyChange = (prev, next) => {
    onChange?.(next);
    onSnap?.(next);
    if (next === levels - 1 && prev !== levels - 1) onMaxHit?.();
  };

  const setIndex = (next, { notify = true, kick = true } = {}) => {
    const clamped = Math.max(0, Math.min(levels - 1, Math.round(next)));
    const prev = idx;
    const changed = clamped !== prev;
    idx = clamped;
    targetPos = idx / (levels - 1);
    if (kick && changed && !motionOff()) vel += (targetPos > pos ? 1.8 : -1.8);
    wake();
    if (changed && notify) notifyChange(prev, idx);
  };

  const normFromClientX = clientX => {
    const r = trackRect || trackEl.getBoundingClientRect();
    const pad = 19;
    return Math.max(0, Math.min(1, (clientX - r.left - pad) / Math.max(1, r.width - pad * 2)));
  };

  trackEl.addEventListener('pointerdown', e => {
    if (e.button > 0) return;
    trackEl.setPointerCapture?.(e.pointerId);
    trackRect = trackEl.getBoundingClientRect();
    dragging = true;
    trackEl.classList.add('is-dragging');
    const norm = normFromClientX(e.clientX);
    const nearest = Math.round(norm * (levels - 1));
    targetPos = norm * 0.78 + (nearest / (levels - 1)) * 0.22;
    if (nearest !== idx) {
      const prev = idx;
      idx = nearest;
      notifyChange(prev, idx);
    }
    wake();
  });

  trackEl.addEventListener('pointermove', e => {
    if (!dragging) return;
    const norm = normFromClientX(e.clientX);
    const nearest = Math.round(norm * (levels - 1));
    targetPos = norm * 0.78 + (nearest / (levels - 1)) * 0.22;
    if (nearest !== idx) {
      const prev = idx;
      idx = nearest;
      notifyChange(prev, idx);
    }
    wake();
  });

  const finishDrag = () => {
    if (!dragging) return;
    dragging = false;
    trackRect = null;
    trackEl.classList.remove('is-dragging');
    targetPos = idx / (levels - 1);
    wake();
  };
  trackEl.addEventListener('pointerup', finishDrag);
  trackEl.addEventListener('pointercancel', finishDrag);

  trackEl.addEventListener('keydown', e => {
    if (e.key === 'ArrowRight' || e.key === 'ArrowUp') { e.preventDefault(); setIndex(idx + 1); }
    else if (e.key === 'ArrowLeft' || e.key === 'ArrowDown') { e.preventDefault(); setIndex(idx - 1); }
  });

  const ro = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(() => render()) : null;
  ro?.observe(trackEl);
  requestAnimationFrame(() => render());

  return {
    getIndex: () => idx,
    setIndex,
    render,
  };
}

export function mountOptics(scope = document) {
  scope.querySelectorAll?.('.tp-optic-btn').forEach(enhanceOpticButton);
  scope.querySelectorAll?.('.tp-glass').forEach(enhanceGlassSurface);
}
