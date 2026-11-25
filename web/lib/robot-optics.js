// SPDX-License-Identifier: GPL-3.0-or-later
// 轻量物理光感交互引擎（零空闲 rAF 循环）：
//   1. 为主操作胶囊 (.tp-optic-btn) 挂载 318° 顶部光源折射边与柔和光泽池，悬停时高光微幅随光标偏转，按下荡开一圈涟漪
//   2. 为毛玻璃面板 (.tp-glass) 追踪悬停光标位置 (--mx / --my)，呈现细腻的镜面反射高光
// 尊重 data-motion="off" 与系统 prefers-reduced-motion。

const reduceMedia = matchMedia('(prefers-reduced-motion: reduce)');
const finePointer = matchMedia('(hover: hover) and (pointer: fine)');
const enhancedBtns = new WeakSet();
const enhancedGlass = new WeakSet();
const REST_ANGLE = 318;

function motionOff() {
  return reduceMedia.matches || document.documentElement.dataset.motion === 'off';
}

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
  const onMove = e => {
    px = e.clientX;
    py = e.clientY;
    if (!raf) raf = requestAnimationFrame(aim);
  };
  const spawnRipple = (x, y) => {
    if (motionOff()) return;
    const r = document.createElement('i');
    r.className = 'tp-optic-ripple';
    r.style.cssText = `--rx:${x.toFixed(1)}px;--ry:${y.toFixed(1)}px`;
    const clean = () => r.remove();
    r.addEventListener('animationend', clean, { once: true });
    setTimeout(clean, 950);
    poolWrap.append(r);
  };

  el.addEventListener('pointerenter', e => {
    if (e.pointerType !== 'mouse' || disabled()) return;
    rect = el.getBoundingClientRect();
    over = true;
    sync();
    onMove(e);
  });
  el.addEventListener('pointermove', onMove, { passive: true });
  el.addEventListener('pointerleave', e => {
    if (e.pointerType === 'mouse') { over = false; sync(); }
  });
  el.addEventListener('pointerdown', e => {
    if (disabled() || e.button > 0) return;
    rect = el.getBoundingClientRect();
    press = true;
    sync();
    onMove(e);
    spawnRipple(e.clientX - rect.left, e.clientY - rect.top);
    const up = () => { press = false; sync(); };
    addEventListener('pointerup', up, { once: true });
    addEventListener('pointercancel', up, { once: true });
  });
}

function enhanceGlassSurface(el) {
  if (!el || enhancedGlass.has(el) || !finePointer.matches) return;
  enhancedGlass.add(el);
  let raf = 0, px = 0, py = 0;
  const paint = () => {
    raf = 0;
    if (motionOff()) return;
    const r = el.getBoundingClientRect();
    if (!r.width || !r.height) return;
    el.style.setProperty('--mx', `${(px - r.left).toFixed(1)}px`);
    el.style.setProperty('--my', `${(py - r.top).toFixed(1)}px`);
  };
  el.addEventListener('pointermove', e => {
    if (e.pointerType !== 'mouse' || motionOff()) return;
    px = e.clientX;
    py = e.clientY;
    if (!raf) raf = requestAnimationFrame(paint);
  }, { passive: true });
  el.addEventListener('pointerleave', () => {
    el.style.removeProperty('--mx');
    el.style.removeProperty('--my');
  });
}

export function mountOptics(scope = document) {
  scope.querySelectorAll?.('.tp-optic-btn').forEach(enhanceOpticButton);
  scope.querySelectorAll?.('.tp-glass').forEach(enhanceGlassSurface);
}
