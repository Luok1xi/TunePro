// SPDX-License-Identifier: GPL-3.0-or-later
// 双拓扑（两串一并 2S1P & 传统三串）远程热调控制台：
//   - 支持「两串一并 2S1P」（视觉外环→陀螺角速度直出 PWM_turn ∥ 均速并联环 PWM_speed）与「传统三串」一键切换
//   - 内置活的控制环路拓扑舞台（可点击各环节点直达对应调参页，运行时展示信号流光脉冲）
//   - 不停车持续运转 + 80 ms 去重节流无扰热改 PID + 快捷命令行 + Esc/Space 最高优先级硬件急停 + 400 ms 看门狗心跳
import { esc } from './esc.js';
import { commands, parseQuickCommand, STOP_REASONS } from './robot-live-model.js';
import { VBAT_COMP_MIN, VBAT_USB } from './robot-live-data.js';
import { mountFluidTabs } from './robot-optics.js';

const MODE_NAMES = ['外环循迹', '速度环·左轮', '速度环·右轮', '均速/双轮', '角速度·正向', '角速度·反向', '速度·多阶阶跃', '角速度·正反阶跃'];
const WHEEL_NAMES = ['左轮', '右轮', '双轮'];
const WHEEL_CODES = ['L', 'R', 'B'];
const STEP_NAMES = { 0: '单步定时', 1: '多阶阶跃', 2: '正反阶跃', 3: '持续运转' };
const PRO_ERRORS = {
  CRC: '校验失败（串口丢字节），请重试',
  FORMAT: '命令格式有误',
  BUSY: '车端正在传输数据，请稍候',
  MOVING: '当前正在运行中：同环内可直接变速/改 PID，跨大环（如速度环↔角速度环/外环）或切拓扑请先按 Esc 停车',
  REVISION: '参数版本已变化，正在自动同步最新参数…',
  NO_IMU: 'IMU 未就绪，无法发车',
  ZERO_TARGET: '目标速度或角速度为 0，请先设非零目标',
  LOW_BAT: '电池低于 11.50 V，车端拒绝带负压运行，请先换满电电池',
  START_REFUSED: '车端拒绝发车（请检查 IMU 或等待导出结束）',
  NO_CAPTURE: '车上还没有本趟板载波形，请先跑一趟',
  RANGE: '参数超出固件安全范围',
  KEY: '当前固件不识别该参数键（如旧版固件暂不支持扩展键 yff/topo）',
};

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function fmt(v, d = 2) { return Number.isFinite(v) ? v.toFixed(d) : '—'; }

export function mountTunePro(host, { stageHost = null, camStreamHost = null, isConnected, sendRaw, onLog, onFocusLoop, onTopologyChange, initialTopology = '2s1p', onResetLegacy }) {
  const ac = new AbortController(), on = (el, ev, fn, opt) => el.addEventListener(ev, fn, { signal: ac.signal, ...opt });
  const roots = [host, stageHost, camStreamHost].filter(Boolean);
  const $ = q => { for (const r of roots) { const el = r.querySelector(q); if (el) return el; } return null; };
  const $$ = q => roots.flatMap(r => [...r.querySelectorAll(q)]);

  let topology = initialTopology === '3cascade' ? '3cascade' : '2s1p';
  let activeTab = 'speed';
  let rid = 1;
  let params = null;
  let prevParams = null;
  let armed = false;
  let running = false;
  let stopReason = 0;
  let vbatMv = NaN;
  let remoteLaunched = false;
  let pingTimer = 0;
  let pendingTimer = 0;
  let imgMode = 0;
  let imgFps = 0;
  const parseImgFps = v => { const n = Number(v); return Number.isFinite(n) && n >= 0 ? Math.round(n) : 0; };
  const fmtImgFps = f => (f === 0 ? '无上限' : `${f} fps`);
  let lastReply = '连接串口后自动同步参数；勾选「解锁远程发车」后，点任意速度/角速度预设或输「B 140」「W 300」即可持续运转，运行中拖滑条无扰热生效！';
  let writeQueue = [];
  let flushing = false;
  let liveTimers = new Map();
  let cmdHistory = [];
  let cmdHistIdx = -1;

  host.className = 'rtp-panel';
  host.dataset.activeLoop = activeTab;
  host.dataset.running = 'false';
  document.documentElement.dataset.activeLoop = activeTab;
  if (stageHost) stageHost.dataset.activeLoop = activeTab;

  const stageHTML = `
    <div class="rtp-stage" data-rtp-stage>
      <div class="rtp-stage-head">
        <span class="rtp-stage-title" data-stage-title>控制拓扑 · 两串一并（2S1P）</span>
        <span class="rtp-stage-desc" data-stage-desc>转向直出 PWM 绕开轮速滞后 · 纵向均速环并联叠加</span>
      </div>
      <div class="rtp-flow" data-stage-flow></div>
    </div>`;

  const imgBarHTML = `
    <div class="rtp-img-bar" aria-label="实时压缩图传">
      <div class="rl-seg" role="group">
        <button type="button" data-rtp-img="0" aria-pressed="true" disabled>关</button>
        <button type="button" data-rtp-img="1" aria-pressed="false" disabled title="188×120 二值图 RLE 压缩，115200 无线可达 8~12 FPS">二值</button>
        <button type="button" data-rtp-img="2" aria-pressed="false" disabled title="94×60 4位灰度压缩，无线约 3 FPS">灰94</button>
        <button type="button" data-rtp-img="3" aria-pressed="false" disabled title="47×30 8位灰度压缩，无线约 6 FPS">灰47</button>
      </div>
      <select data-rtp-img-fps aria-label="图传帧率" disabled>
        ${[0, 60, 30, 15, 10, 6].map(f => `<option value="${f}"${f === 0 ? ' selected' : ''}>${f === 0 ? '无上限' : `${f} fps`}</option>`).join('')}
      </select>
    </div>`;

  if (stageHost) stageHost.innerHTML = stageHTML;
  if (camStreamHost) camStreamHost.innerHTML = imgBarHTML;

  host.innerHTML = `
    ${stageHost ? '' : stageHTML}

    <!-- 安全状态栏与发车/急停 -->
    <div class="rtp-safety-bar">
      <div class="rtp-status-group">
        <span class="rtp-badge" data-rtp-state data-level="idle">未连接</span>
        <span class="rtp-badge" data-rtp-vbat data-level="idle">电池 —</span>
        <span class="rtp-badge" data-rtp-mode>模式 —</span>
        <span class="rtp-badge" data-rtp-wdog data-level="idle">看门狗待命</span>
      </div>
      <div class="rtp-launch-group">
        <button type="button" class="rtp-sync-btn" data-rtp-sync disabled>同步</button>
        <label class="rtp-arm-toggle" title="防止误触：勾选一次后即可持续一键发车/切速，急停或断联后自动上锁">
          <input type="checkbox" data-rtp-arm disabled>
          <span>解锁发车</span>
        </label>
        <button type="button" class="rtp-go-btn" data-rtp-go disabled>远程发车</button>
        <button type="button" class="rtp-stop-btn" data-rtp-stop disabled title="最高优先级硬件急停（快捷键 Space / Esc）">急停 · Esc</button>
      </div>
    </div>

    <!-- 极简底线快捷命令行 -->
    <form class="rtp-quick-bar" data-rtp-quick-form>
      <span class="rtp-quick-tag">CMD</span>
      <input type="text" data-rtp-quick-input disabled autocomplete="off" spellcheck="false"
        placeholder="B 140 · W -300 · vkp 38 · ykp 18 · yff 1.85 · fan 40 · s">
      <button type="submit" class="tp-btn-subtle" data-rtp-quick-send disabled>↵</button>
    </form>

    <div class="rtp-banner" data-rtp-banner role="status">${esc(lastReply)}</div>

    <!-- 三环细线 Tab -->
    <div class="rtp-tabs-row">
      <div class="rtp-loop-tabs" role="tablist" aria-label="控制环路调参">
        <button type="button" role="tab" data-rtp-tab="speed" aria-selected="true">① 均速并联</button>
        <button type="button" role="tab" data-rtp-tab="yaw" aria-selected="false">② 角速度直出</button>
        <button type="button" role="tab" data-rtp-tab="outer" aria-selected="false">③ 视觉外环</button>
      </div>
      ${camStreamHost ? '' : imgBarHTML}
    </div>

    <div class="rtp-body">
      <!-- Tab 1: 均速并联环 / 速度内环（滑条置顶，紧邻实时曲线与画面） -->
      <section class="rtp-tab-panel" data-rtp-panel="speed">
        <div class="rtp-controls-grid">
          <div class="rtp-box rtp-pid-box">
            <h5><span data-speed-pid-title>均速并联环 PI（运行中无扰热生效）</span><small>80ms 热节流</small></h5>
            <!-- 2S1P 模式同步均速滑条 -->
            <div data-vavg-block>
              ${gainRowHTML('vkp', '均速 Kp', 0, 80, 0.5, 38.0)}
              ${gainRowHTML('vki', '均速 Ki', 0, 10, 0.1, 1.2)}
            </div>
            <!-- 左右独立轮速（3-Cascade 常显，2S1P 可展开微调） -->
            <div class="rtp-lr-extra" data-lr-block>
              ${gainRowHTML('lkp', '左轮 Kp', 0, 80, 0.5, 15.0)}
              ${gainRowHTML('lki', '左轮 Ki', 0, 10, 0.1, 1.5)}
              ${gainRowHTML('rkp', '右轮 Kp', 0, 80, 0.5, 15.0)}
              ${gainRowHTML('rki', '右轮 Ki', 0, 10, 0.1, 1.5)}
            </div>
            <div class="rl-row rtp-gain-actions">
              <button type="button" data-rtp-toggle-lr>左右微调</button>
              <button type="button" data-rtp-copy-lr disabled title="把左轮 Kp/Ki 同步复制到右轮（运行中可用）">左→右同步</button>
              <button type="button" data-rtp-undo disabled>撤销</button>
              <button type="button" data-rtp-cap disabled title="停车后读取最近 4 秒 4ms 高清板载波形">取最近波形</button>
            </div>
          </div>

          <div class="rtp-box">
            <h5>轮侧 · 热切目标速 · 负压 <small data-speed-mode-badge>2S1P 并联支路</small></h5>
            <div class="rtp-preset-row">
              <span class="rtp-mini-label">一键直切</span>
              ${[0, 60, 100, 140, 180, 220, 260].map(v => `<button type="button" class="rtp-chip${v === 0 ? ' rtp-chip-zero' : ''}" data-rtp-run-speed="${v}" disabled>${v === 0 ? '0 降零' : v}</button>`).join('')}
            </div>
            <div class="rl-row">
              <label>目标速 <input type="number" data-rtp-field="test_speed" min="0" max="350" step="10" value="140" disabled></label>
              <button type="button" class="tp-btn-accent" data-rtp-go-speed disabled title="停着时直接持续转，转着时无缝切到该速度">转 / 切速</button>
              <label title="仅多阶阶跃模式使用">低档 <input type="number" data-rtp-field="test_low" min="0" max="350" step="10" value="90" disabled></label>
              <button type="button" data-rtp-apply-low disabled>改低档</button>
            </div>
            <div class="rl-row">
              <div class="rl-seg" role="group" aria-label="指定车轮">
                <button type="button" data-rtp-wheel="2" aria-pressed="true" disabled>双轮</button>
                <button type="button" data-rtp-wheel="0" aria-pressed="false" disabled>左轮</button>
                <button type="button" data-rtp-wheel="1" aria-pressed="false" disabled>右轮</button>
              </div>
              <div class="rl-seg" role="group" aria-label="测试方式">
                <button type="button" data-rtp-step="3" aria-pressed="true" disabled title="一直转直到手动停车，运行中可随时无停顿变速和改 PID">持续</button>
                <button type="button" data-rtp-step="1" aria-pressed="false" disabled title="0→低档→高档→低档→0（3.2s），自动跳过首沿静摩擦">多阶</button>
                <button type="button" data-rtp-step="0" aria-pressed="false" disabled title="0→目标→0（2s 定时单步）">单步</button>
              </div>
            </div>
            <div class="rl-row">
              <label class="rtp-check"><input type="checkbox" data-rtp-field="tune_fan" disabled> 负压</label>
              <label><input type="number" data-rtp-field="fan_pct" min="0" max="80" step="5" value="40" disabled> %</label>
              <button type="button" data-rtp-apply-fan disabled>应用负压</button>
            </div>
          </div>
        </div>
        <p class="rtp-guide" data-guide-speed></p>
      </section>

      <!-- Tab 2: 角速度内环（滑条置顶，边看陀螺波形与画面边热调） -->
      <section class="rtp-tab-panel" data-rtp-panel="yaw" hidden>
        <div class="rtp-controls-grid">
          <div class="rtp-box rtp-pid-box">
            <h5><span data-yaw-pid-title>角速度串级内环 · 直出 PWM 增益</span><small data-yaw-unit>PWM / (°/s)</small></h5>
            <div data-yff-wrap>
              ${gainRowHTML('yff', '前馈 Kff', 0, 6.0, 0.05, 1.85)}
            </div>
            ${gainRowHTML('ykp', '角速度 Kp', 0, 40.0, 0.5, 18.0)}
            ${gainRowHTML('yki', '角速度 Ki', 0, 2.0, 0.02, 0.12)}
            <div class="rl-row rtp-gain-actions">
              <span class="rl-row" data-yaw-presets>
                <span class="rtp-mini-label">参考</span>
                <button type="button" class="rtp-chip" data-yaw-preset="suction" disabled title="开负压高抓地参考：Kp=18.0, Kff=1.85, Ki=0">开负压 18.0</button>
                <button type="button" class="rtp-chip" data-yaw-preset="dry" disabled title="0%负压干地柔顺参考：Kp=6.5, Kff=1.85, Ki=0">干地 6.5</button>
              </span>
              <button type="button" data-rtp-undo disabled>撤销</button>
              <button type="button" data-rtp-cap disabled>取波形</button>
            </div>
          </div>

          <div class="rtp-box">
            <h5>原地正反热切角速度 · 负压 <small data-yaw-mode-badge>直出 PWM_turn</small></h5>
            <div class="rtp-preset-row">
              <span class="rtp-mini-label">一键正反</span>
              ${[-450, -300, -200, 0, 200, 300, 450, 550].map(w => `<button type="button" class="rtp-chip${w === 0 ? ' rtp-chip-zero' : ''}" data-rtp-run-yaw="${w}" disabled>${w > 0 ? `+${w}` : w === 0 ? '0 降零' : w}</button>`).join('')}
            </div>
            <div class="rl-row">
              <label>角速 °/s <input type="number" data-rtp-field="test_yaw" min="-600" max="600" step="20" value="300" disabled></label>
              <button type="button" class="tp-btn-accent" data-rtp-go-yaw disabled>转 / 切角速</button>
            </div>
            <div class="rl-row">
              <div class="rl-seg" role="group" aria-label="角速度测试方式">
                <button type="button" data-rtp-yawstep="3" aria-pressed="true" disabled title="持续原地转，随时点正负角速度不停车直接反向">持续转</button>
                <button type="button" data-rtp-yawstep="2" aria-pressed="false" disabled title="0 → +ω → −ω → +ω → 0（3.2s 自动交替）">正反阶跃</button>
                <button type="button" data-rtp-yawstep="0" aria-pressed="false" disabled title="2s 定时单步">单步</button>
              </div>
              <label class="rtp-check"><input type="checkbox" data-rtp-field="tune_fan_yaw" disabled> 负压</label>
              <label><input type="number" data-rtp-field="fan_pct_yaw" min="0" max="80" step="5" value="40" disabled>%</label>
              <button type="button" data-rtp-apply-fan-yaw disabled>应用</button>
            </div>
          </div>
        </div>
        <p class="rtp-guide" data-guide-yaw></p>
      </section>

      <!-- Tab 3: 视觉外环（外环 PD 滑条置顶，紧邻实时摄像头画面） -->
      <section class="rtp-tab-panel" data-rtp-panel="outer" hidden>
        <div class="rtp-controls-grid">
          <div class="rtp-box rtp-pid-box">
            <h5>外环位置式 PD（盯着左侧画面实时热调）<small>可选 ±20 速度带</small></h5>
            ${gainRowHTML('dirp_cur', '外环 Kp', 0, 40, 0.2, 8.5)}
            ${gainRowHTML('dird_cur', '外环 Kd', 0, 120, 0.5, 6.0)}
            <div class="rl-row rtp-gain-actions">
              <label class="rtp-check"><input type="checkbox" data-rtp-band checked> 同步写入 ±20 速度带</label>
              <button type="button" data-rtp-undo disabled>撤销</button>
            </div>
          </div>

          <div class="rtp-box">
            <h5>竞赛循迹模式与速度 / 偏航上限 <small>外环输出 ω_des</small></h5>
            <div class="rl-row">
              <button type="button" class="tp-btn-accent" data-rtp-race-mode disabled>切回竞赛循迹</button>
              <span class="rl-sub" data-rtp-race-hint>当前模式：—</span>
            </div>
            <div class="rl-row">
              <label>直道 <input type="number" data-rtp-field="race_speed" min="40" max="350" step="5" value="125" disabled></label>
              <label>弯道 <input type="number" data-rtp-field="turn_speed" min="40" max="300" step="5" value="80" disabled></label>
              <label>角限 <input type="number" data-rtp-field="yaw_cap" min="100" max="1500" step="20" value="920" disabled></label>
            </div>
            <div class="rl-row">
              <label>负压 % <input type="number" data-rtp-field="fan_pct_outer" min="0" max="80" step="5" value="40" disabled></label>
              <button type="button" data-rtp-apply-outer disabled>应用速度与负压</button>
            </div>
          </div>
        </div>
        <p class="rtp-guide" data-guide-outer></p>
      </section>
    </div>
  `;

  function gainRowHTML(key, label, min, max, step, defVal) {
    return `<div class="rtp-gain-row" data-gain-row="${key}">
      <div class="rtp-g-top">
        <span class="rtp-g-label">${label}</span>
        <div class="rtp-g-controls">
          <button type="button" class="rtp-nudge" data-rtp-nudge="${key}" data-dir="-1" data-step="${step}" disabled aria-label="减小${label}">−</button>
          <input type="number" data-rtp-num="${key}" min="${min}" max="${max}" step="${step}" value="${defVal}" disabled aria-label="${label}数值">
          <button type="button" class="rtp-nudge" data-rtp-nudge="${key}" data-dir="1" data-step="${step}" disabled aria-label="增大${label}">+</button>
          <button type="button" class="rtp-apply-one" data-rtp-send-gain="${key}" disabled>写入</button>
        </div>
      </div>
      <input type="range" data-rtp-slider="${key}" min="${min}" max="${max}" step="${step}" value="${defVal}" disabled aria-label="${label}滑条">
    </div>`;
  }

  const GAIN_SCALE = { vkp: 100, vki: 1000, lkp: 100, lki: 1000, rkp: 100, rki: 1000, yff: 1000, ykp: 10000, yki: 10000, dirp_cur: 1000, dird_cur: 1000 };
  const GAIN_DEC   = { vkp: 2,   vki: 3,    lkp: 2,   lki: 3,    rkp: 2,   rki: 3,    yff: 2,    ykp: 2,     yki: 3,     dirp_cur: 2,    dird_cur: 2 };
  let showLrIn2s1p = false;
  let nodeValRefs = { outer: null, yaw: null, speed: null, mixer: null };
  let nodeValCache = { outer: '', yaw: '', speed: '', mixer: '' };
  const vbatElRef = $('[data-rtp-vbat]');
  let lastVbatTxt = '', lastVbatLvl = '';

  function updateVbatBadge() {
    let txt = '电池 —', lvl = 'idle';
    if (Number.isFinite(vbatMv) && vbatMv >= VBAT_USB) {
      txt = `电池 ${(vbatMv / 1000).toFixed(2)} V${vbatMv < VBAT_COMP_MIN ? ' · 偏低!' : ''}`;
      lvl = vbatMv < 10500 ? 'bad' : vbatMv < VBAT_COMP_MIN ? 'warn' : 'good';
    } else if (Number.isFinite(vbatMv) && vbatMv > 0) {
      txt = '仅 USB 供电';
      lvl = 'warn';
    }
    if (txt !== lastVbatTxt) { lastVbatTxt = txt; vbatElRef.textContent = txt; }
    if (lvl !== lastVbatLvl) { lastVbatLvl = lvl; vbatElRef.dataset.level = lvl; }
  }

  function renderStageDiagram() {
    const is2s1p = topology === '2s1p';
    $('[data-stage-title]').textContent = is2s1p ? '控制拓扑 · 两串一并（2S1P）' : '控制拓扑 · 传统三串（3-Cascade）';
    $('[data-stage-desc]').textContent = is2s1p
      ? '点击任意环切换环绕流光与对应滑条 · 转向直出 PWM_turn ∥ 纵向均速 PWM_speed'
      : '点击任意环切换环绕流光与对应滑条 · 外环 → 角速度中环 → 左右轮速 PI';
    const flow = $('[data-stage-flow]');
    if (is2s1p) {
      flow.innerHTML = `
        <div class="rtp-flow-branches">
          <div class="rtp-flow-row">
            <span class="rtp-branch-tag" data-kind="series">两串 · 转向</span>
            <button type="button" class="rtp-node" data-node="outer" data-jump-tab="outer" aria-current="${activeTab === 'outer'}">
              <span class="rtp-node-orb"></span><span><strong>视觉外环</strong><br><small data-node-val="outer">err — px → ω</small></span>
            </button>
            <span class="rtp-arrow"></span>
            <button type="button" class="rtp-node" data-node="yaw" data-jump-tab="yaw" aria-current="${activeTab === 'yaw'}">
              <span class="rtp-node-orb"></span><span><strong>角速度直出 PWM</strong><br><small data-node-val="yaw">gyro — °/s</small></span>
            </button>
            <span class="rtp-arrow"></span>
            <span class="rtp-node" data-node="mixer">
              <span class="rtp-node-orb"></span><span><strong>非对称混控 · ±8500</strong><br><small data-node-val="mixer">CM — · DM —</small></span>
            </span>
          </div>
          <div class="rtp-flow-row">
            <span class="rtp-branch-tag" data-kind="parallel">一并 · 纵向</span>
            <button type="button" class="rtp-node" data-node="speed" data-jump-tab="speed" aria-current="${activeTab === 'speed'}">
              <span class="rtp-node-orb"></span><span><strong>均速并联环 (v_avg)</strong><br><small data-node-val="speed">v — cnt/2ms → PWM_speed</small></span>
            </button>
            <span class="rtp-arrow"></span>
            <span class="tp-sub" style="font-family:var(--font-mono)">并联汇入 PWM_speed ± PWM_turn</span>
          </div>
        </div>`;
    } else {
      flow.innerHTML = `
        <div class="rtp-flow-row">
          <span class="rtp-branch-tag" data-kind="series">三串 · 级联</span>
          <button type="button" class="rtp-node" data-node="outer" data-jump-tab="outer" aria-current="${activeTab === 'outer'}">
            <span class="rtp-node-orb"></span><span><strong>视觉外环</strong><br><small data-node-val="outer">err — px</small></span>
          </button>
          <span class="rtp-arrow"></span>
          <button type="button" class="rtp-node" data-node="yaw" data-jump-tab="yaw" aria-current="${activeTab === 'yaw'}">
            <span class="rtp-node-orb"></span><span><strong>角速度中环</strong><br><small data-node-val="yaw">gyro — °/s → Δv</small></span>
          </button>
          <span class="rtp-arrow"></span>
          <button type="button" class="rtp-node" data-node="speed" data-jump-tab="speed" aria-current="${activeTab === 'speed'}">
            <span class="rtp-node-orb"></span><span><strong>左/右独立轮速内环</strong><br><small data-node-val="speed">L/R — cnt/2ms</small></span>
          </button>
          <span class="rtp-arrow"></span>
          <span class="rtp-node" data-node="mixer">
            <span class="rtp-node-orb"></span><span><strong>双轮 PWM</strong><br><small data-node-val="mixer">L/R —</small></span>
          </span>
        </div>`;
    }
    nodeValRefs = {
      outer: flow.querySelector('[data-node-val="outer"]'),
      yaw: flow.querySelector('[data-node-val="yaw"]'),
      speed: flow.querySelector('[data-node-val="speed"]'),
      mixer: flow.querySelector('[data-node-val="mixer"]'),
    };
    nodeValCache = { outer: '', yaw: '', speed: '', mixer: '' };
  }

  function applyTopologyUI() {
    const is2s1p = topology === '2s1p';
    $('[data-rtp-tab="speed"]').textContent = is2s1p ? '① 均速并联' : '① 左右轮速';
    $('[data-rtp-tab="yaw"]').textContent = is2s1p ? '② 角速度直出' : '② 角速度中环';
    $('[data-rtp-tab="outer"]').textContent = '③ 视觉外环';

    $('[data-guide-speed]').innerHTML = is2s1p
      ? '<b>两串一并 · 均速支路</b>：纵向只控双轮均速 <code>0.5×(encL+encR) → PWM_speed</code>，拖动上方均速 Kp/Ki 自动同步写入双轮并热生效；需单轮硬件点检时可切「左轮 / 右轮」。'
      : '<b>传统三串 · 速度内环</b>：选好轮侧并勾选解锁后，点下方任意速度直接持续运转；运行中可无停顿直切目标速并拖滑条实时微调左右轮增量 PI。';
    $('[data-guide-yaw]').innerHTML = is2s1p
      ? '<b>两串一并 · 转向直出 PWM</b>：陀螺角速度环直接输出差动 <code>PWM_turn = Kff·ω + Kp·(ω−gz)</code>，绕开轮速环 31 ms 滞后！开负压参考 <b>Kp ≈ 18.0</b>，0% 干地参考 <b>Kp ≈ 6.5</b>，循迹建议 <b>Ki = 0</b>；在示波器可切「共模 / 差模 PWM」观察转向力矩裕度。'
      : '<b>传统三串 · 角速度中环</b>：在赛道上开启负压做原地旋转，点下方 <b>+300 / -300 °/s</b> 可不停车直接正反无缝切换，边转边调角速度 Kp/Ki 观察陀螺跟踪滞后。';
    $('[data-guide-outer]').innerHTML = is2s1p
      ? '<b>两串一并 · 视觉外环</b>：均速环与直出角速度环稳住后切回竞赛循迹；开负压时可将角速上限放宽至 <b>920~1380 °/s</b>，Kp 决定过弯与直角入弯速度，Kd 负责直道阻尼抑振。'
      : '<b>传统三串 · 视觉外环</b>：速度环与角速度环稳住后切回竞赛循迹；先小弯再 90° 直角，Kp 管转向快慢，Kd 管直道阻尼，运行中可实时微调。';

    $('[data-speed-mode-badge]').textContent = is2s1p ? '2S1P 并联均速支路' : '左右独立轮速内环';
    $('[data-speed-pid-title]').textContent = is2s1p ? '均速并联环 PI（同步双轮热生效）' : '左右轮增量 PI（运行中无扰热生效）';
    $('[data-yaw-mode-badge]').textContent = is2s1p ? '直出 PWM_turn（零轮速滞后）' : '输出轮差 Δv';
    $('[data-yaw-pid-title]').textContent = is2s1p ? '角速度串级内环 · 直出 PWM 增益' : '角速度中环 PI（输出轮速差）';
    $('[data-yaw-unit]').textContent = is2s1p ? 'PWM / (°/s)' : 'cnt / (°/s)';

    $('[data-vavg-block]').hidden = !is2s1p;
    $('[data-lr-block]').hidden = is2s1p && !showLrIn2s1p;
    $('[data-rtp-toggle-lr]').hidden = !is2s1p;
    $('[data-rtp-toggle-lr]').textContent = showLrIn2s1p ? '收起左右独立微调' : '展开左右独立微调';
    $('[data-yff-wrap]').hidden = !is2s1p;
    $('[data-yaw-presets]').hidden = !is2s1p;

    // Adjust yaw slider scale cleanly between 2S1P (direct PWM 0..40) and 3-Cascade (0..1.5)
    const ykpSl = $('[data-rtp-slider="ykp"]'), ykpNum = $('[data-rtp-num="ykp"]');
    const ykiSl = $('[data-rtp-slider="yki"]'), ykiNum = $('[data-rtp-num="yki"]');
    const ykpNudges = $$('[data-rtp-nudge="ykp"]'), ykiNudges = $$('[data-rtp-nudge="yki"]');
    if (is2s1p) {
      ykpSl.max = '40'; ykpSl.step = '0.5'; ykpNum.max = '60'; ykpNum.step = '0.5';
      ykiSl.max = '2';  ykiSl.step = '0.02'; ykiNum.max = '10'; ykiNum.step = '0.02';
      ykpNudges.forEach(b => { b.dataset.step = '0.5'; });
      ykiNudges.forEach(b => { b.dataset.step = '0.02'; });
      GAIN_DEC.ykp = 2; GAIN_DEC.yki = 3;
    } else {
      ykpSl.max = '1.5'; ykpSl.step = '0.01'; ykpNum.max = '5'; ykpNum.step = '0.01';
      ykiSl.max = '0.2'; ykiSl.step = '0.001'; ykiNum.max = '2'; ykiNum.step = '0.001';
      ykpNudges.forEach(b => { b.dataset.step = '0.01'; });
      ykiNudges.forEach(b => { b.dataset.step = '0.001'; });
      GAIN_DEC.ykp = 4; GAIN_DEC.yki = 4;
    }

    renderStageDiagram();
    if (params) {
      syncInputsFromParams(params);
    } else {
      const defYkp = is2s1p ? 18.0 : 0.22;
      const defYki = is2s1p ? 0.12 : 0.011;
      ykpSl.value = String(defYkp); ykpNum.value = String(defYkp);
      ykiSl.value = String(defYki); ykiNum.value = String(defYki);
    }
  }

  function setBanner(text, level = 'info') {
    lastReply = text;
    const el = $('[data-rtp-banner]');
    el.textContent = text;
    el.dataset.level = level;
  }

  function currentWheelIdx() {
    if (!params) return topology === '2s1p' ? 2 : 0;
    return params.mode >= 1 && params.mode <= 3 ? params.mode - 1 : (params.wheel ?? 0);
  }

  function inSpeedLoop() { return params && [1, 2, 3, 6].includes(params.mode); }
  function inYawLoop() { return params && [4, 5, 7].includes(params.mode); }

  function updateUI() {
    const conn = isConnected();
    const parked = conn && !running;
    const ready = conn && Boolean(params);
    host.dataset.running = String(running);
    if (stageHost) stageHost.dataset.running = String(running);

    const stateEl = $('[data-rtp-state]');
    stateEl.textContent = !conn ? '未连接' : running ? (params?.step_mode === 3 ? '持续运转中' : '运行中') : stopReason ? `已停 · ${STOP_REASONS[stopReason] || '码 ' + stopReason}` : '已停稳';
    stateEl.dataset.level = !conn ? 'idle' : running ? 'good' : stopReason && stopReason !== 20 && stopReason !== 26 ? 'bad' : 'idle';

    updateVbatBadge();

    const modeEl = $('[data-rtp-mode]');
    modeEl.textContent = params ? `${MODE_NAMES[params.mode] || params.mode} · ${STEP_NAMES[params.step_mode] || ''} · rev ${params.rev}` : '模式：待同步';

    const wdogEl = $('[data-rtp-wdog]');
    wdogEl.textContent = remoteLaunched && running ? '看门狗 400ms 心跳中' : '看门狗待命';
    wdogEl.dataset.level = remoteLaunched && running ? 'good' : 'idle';

    $('[data-rtp-sync]').disabled = !conn;
    $('[data-rtp-stop]').disabled = !conn;
    $('[data-rtp-arm]').disabled = !ready;
    if (!conn) { armed = false; $('[data-rtp-arm]').checked = false; }
    $('[data-rtp-go]').disabled = !ready || !parked || !armed;
    $('[data-rtp-go]').textContent = params ? `远程发车 · ${launchSummary()}` : '远程发车';

    $('[data-rtp-quick-input]').disabled = !conn;
    $('[data-rtp-quick-send]').disabled = !conn;

    $$('[data-rtp-img], [data-rtp-img-fps]').forEach(el => { el.disabled = !conn; });
    $$('[data-rtp-img]').forEach(b => b.setAttribute('aria-pressed', String(Number(b.dataset.rtpImg) === imgMode)));

    const liveGainOk = ready;
    $$('[data-rtp-slider], [data-rtp-num], [data-rtp-nudge], [data-rtp-send-gain], [data-rtp-copy-lr], [data-rtp-apply-fan], [data-rtp-apply-fan-yaw], [data-rtp-apply-low], [data-rtp-apply-outer], [data-yaw-preset]').forEach(el => { el.disabled = !liveGainOk; });
    $$('[data-rtp-field]').forEach(el => { el.disabled = !liveGainOk; });
    $$('[data-rtp-undo]').forEach(el => { el.disabled = !liveGainOk || !prevParams; });

    const speedRunOk = ready && (parked || inSpeedLoop());
    $$('[data-rtp-wheel], [data-rtp-run-speed], [data-rtp-go-speed]').forEach(el => { el.disabled = !speedRunOk; });
    const yawRunOk = ready && (parked || inYawLoop());
    $$('[data-rtp-run-yaw], [data-rtp-go-yaw]').forEach(el => { el.disabled = !yawRunOk; });
    $$('[data-rtp-step], [data-rtp-yawstep], [data-rtp-race-mode], [data-rtp-cap]').forEach(el => { el.disabled = !ready || !parked; });

    if (params) {
      const wheelIdx = currentWheelIdx();
      $$('[data-rtp-wheel]').forEach(b => b.setAttribute('aria-pressed', String(Number(b.dataset.rtpWheel) === wheelIdx)));
      const spdStep = params.mode === 6 ? 1 : params.step_mode === 3 ? 3 : 0;
      $$('[data-rtp-step]').forEach(b => b.setAttribute('aria-pressed', String(spdStep === Number(b.dataset.rtpStep))));
      const yawStep = params.mode === 7 ? 2 : params.step_mode === 3 ? 3 : 0;
      $$('[data-rtp-yawstep]').forEach(b => b.setAttribute('aria-pressed', String(yawStep === Number(b.dataset.rtpYawstep))));
      $('[data-rtp-race-hint]').textContent = `当前模式：${MODE_NAMES[params.mode] || params.mode} · 负压 ${Math.round(params.fan_duty / 100)}%`;
    }
  }

  function launchSummary() {
    if (!params) return '';
    const fanTxt = (params.mode === 0 || params.tune_fan) && params.fan_duty > 0 ? ` · 负压${Math.round(params.fan_duty / 100)}%` : ' · 关负压';
    if (params.mode === 0) return `循迹 ${params.race_speed}${fanTxt}`;
    if (params.mode === 6) return `${WHEEL_NAMES[params.wheel]}多阶 ${params.test_low}→${params.test_speed}${fanTxt}`;
    if (params.mode === 7) return `正反±${params.test_yaw}°/s${fanTxt}`;
    if (params.mode <= 3) return `${WHEEL_NAMES[params.mode - 1]} ${params.step_mode === 3 ? '持续' : ''}${params.test_speed}${fanTxt}`;
    return `${params.mode === 4 ? '+' : '-'}${params.test_yaw}°/s${fanTxt}`;
  }

  function syncInputsFromParams(p) {
    const active = document.activeElement;
    const setIfIdle = (sel, val) => { const el = $(sel); if (el && el !== active) el.value = val; };
    const fanPct = Math.round(p.fan_duty / 100);
    setIfIdle('[data-rtp-field="test_low"]', p.test_low);
    setIfIdle('[data-rtp-field="test_speed"]', p.test_speed);
    setIfIdle('[data-rtp-field="test_yaw"]', p.mode === 5 ? -p.test_yaw : p.test_yaw);
    setIfIdle('[data-rtp-field="race_speed"]', p.race_speed);
    setIfIdle('[data-rtp-field="turn_speed"]', p.turn_speed);
    setIfIdle('[data-rtp-field="yaw_cap"]', p.yaw_cap);
    $('[data-rtp-field="tune_fan"]').checked = Boolean(p.tune_fan);
    $('[data-rtp-field="tune_fan_yaw"]').checked = Boolean(p.tune_fan);
    setIfIdle('[data-rtp-field="fan_pct"]', fanPct);
    setIfIdle('[data-rtp-field="fan_pct_yaw"]', fanPct);
    setIfIdle('[data-rtp-field="fan_pct_outer"]', fanPct);
    imgMode = p.img_mode;
    imgFps = parseImgFps(p.img_fps);
    const fpsSel = $('[data-rtp-img-fps]');
    if (fpsSel) {
      if (![...fpsSel.options].some(o => Number(o.value) === imgFps)) {
        const opt = document.createElement('option');
        opt.value = String(imgFps);
        opt.textContent = fmtImgFps(imgFps);
        fpsSel.append(opt);
      }
      fpsSel.value = String(imgFps);
    }

    const viewVals = {
      ...p,
      vkp: (p.lkp + p.rkp) / 2,
      vki: (p.lki + p.rki) / 2,
      yff: Number.isFinite(p.yff) ? p.yff : 1.85,
    };
    for (const k of Object.keys(GAIN_SCALE)) {
      const v = viewVals[k];
      if (!Number.isFinite(v)) continue;
      const num = $(`[data-rtp-num="${k}"]`), sl = $(`[data-rtp-slider="${k}"]`);
      if (num && num !== active && !liveTimers.has(k)) num.value = Number(v.toFixed(GAIN_DEC[k]));
      if (sl && sl !== active && !liveTimers.has(k)) {
        if (v > Number(sl.max)) sl.max = String(Math.ceil(v * 1.4));
        sl.value = String(v);
      }
    }
  }

  function nextRid() { return ++rid; }

  async function requestSync() {
    if (!isConnected()) return;
    setBanner('正在向车端读取参数快照 (#M)…', 'info');
    await sendRaw(commands.proMeta(nextRid()), { paced: true });
    clearTimeout(pendingTimer);
    pendingTimer = setTimeout(() => {
      if (!params) {
        setBanner('未收到 TunePro 车端回复（若车端为旧版固件，实时 50Hz @M2 波形仍可用；刷入支持 TunePro 协议的固件后可解锁全部热调功能）。', 'warn');
        onResetLegacy?.();
      }
    }, 1800);
  }

  async function enqueueWrites(ops) {
    if (!isConnected() || !params) { setBanner('请先点「同步参数」', 'warn'); return; }
    if (!prevParams && ops.some(o => o.kind === 'gain')) prevParams = { ...params };
    for (const op of ops) {
      const tag = op.type === 'band' ? `band_${op.bandKind}` : op.key;
      const idx = writeQueue.findIndex(q => (q.type === 'band' ? `band_${q.bandKind}` : q.key) === tag);
      if (idx >= 0) writeQueue[idx] = op;
      else writeQueue.push(op);
    }
    if (!flushing) flushQueue();
  }

  async function flushQueue() {
    if (flushing || !writeQueue.length) return;
    flushing = true;
    try {
      while (writeQueue.length && isConnected() && params) {
        const op = writeQueue.shift();
        const cmd = op.type === 'band'
          ? commands.proWriteBand(nextRid(), params.rev, op.bandKind, op.lo, op.hi, op.val)
          : commands.proWrite(nextRid(), params.rev, op.key, op.val);
        if (!cmd) { setBanner(`参数 ${op.key || op.bandKind} 超出范围`, 'bad'); continue; }
        const expectedRev = params.rev + 1;
        setBanner(`${running ? '运行中热改 ' : '正在写入 '}${op.label || op.key}…`, 'info');
        const ok = await sendRaw(cmd, { paced: true });
        if (!ok) { writeQueue = []; break; }
        const got = await waitForRev(expectedRev, 1500);
        if (!got) {
          setBanner(`写入 ${op.label || op.key} 未收到回读确认，正在重新同步…`, 'warn');
          writeQueue = [];
          await requestSync();
          break;
        }
      }
    } finally {
      flushing = false;
      updateUI();
    }
  }

  let revWaiter = null;
  function waitForRev(targetRev, timeoutMs) {
    return new Promise(resolve => {
      if (params && params.rev >= targetRev) { resolve(true); return; }
      const timer = setTimeout(() => { if (revWaiter?.resolve === resolve) revWaiter = null; resolve(false); }, timeoutMs);
      revWaiter = { targetRev, resolve: ok => { clearTimeout(timer); revWaiter = null; resolve(ok); } };
    });
  }

  function startWatchdog() {
    stopWatchdog();
    pingTimer = setInterval(() => {
      if (!isConnected() || !running || !remoteLaunched) { stopWatchdog(); return; }
      sendRaw(commands.proPing(), { paced: false, urgent: true });
    }, 400);
  }
  function stopWatchdog() {
    if (pingTimer) { clearInterval(pingTimer); pingTimer = 0; }
  }

  async function emergencyStop(reasonText = '手动点击急停') {
    armed = false;
    $('[data-rtp-arm]').checked = false;
    writeQueue = [];
    for (const tm of liveTimers.values()) clearTimeout(tm);
    liveTimers.clear();
    remoteLaunched = false;
    stopWatchdog();
    if (!isConnected()) return;
    setBanner(`紧急停车已触发（${reasonText}）！发送最高优先级截断 #! + #X STOP`, 'bad');
    onLog?.('error', `远程急停：${reasonText}`);
    await sendRaw(commands.proEmergencyStop() + commands.proStop(), { urgent: true, paced: false });
    updateUI();
  }

  async function sendRunContinuous(kind, target) {
    if (!isConnected() || !params) { setBanner('请先连接串口并同步参数', 'warn'); return; }
    const k = String(kind).toUpperCase();
    const isYaw = k === 'W';
    const val = Math.round(Number(target) || 0);
    if (!running && val !== 0 && !armed) {
      setBanner('安全保护：请先勾选上方「解锁远程发车」一次，之后即可连续点选速度/发车！', 'warn');
      return;
    }
    const cmd = commands.proRun(k, val);
    if (!cmd) { setBanner(`目标值 ${val} 超出安全范围`, 'bad'); return; }
    if (!running && val !== 0) {
      if (params.tune_fan && params.fan_duty > 0 && Number.isFinite(vbatMv) && vbatMv >= VBAT_USB && vbatMv < VBAT_COMP_MIN) {
        setBanner(`警告：电池 ${(vbatMv / 1000).toFixed(2)} V 低于 11.70 V 比赛红线！仍尝试发车（<11.50 V 车端拒绝）。`, 'warn');
      }
      remoteLaunched = true;
    }
    const desc = isYaw ? `角速度 ${val > 0 ? `+${val}` : val} °/s` : `${{ L: '左轮', R: '右轮', B: '双轮均速' }[k]} 速度 ${val}`;
    setBanner(running ? `不停车无缝切目标 → ${desc}…` : val === 0 ? `已预设 ${desc}` : `一键持续发车 → ${desc}…`, 'info');
    onLog?.('event', `${running ? '热切目标' : '一键持续发车'}：${desc}`);
    const ok = await sendRaw(cmd, { urgent: running, paced: !running });
    if (ok && (running || val !== 0)) startWatchdog();
    else if (!ok && !running) { remoteLaunched = false; setBanner('指令发送失败', 'bad'); }
    updateUI();
  }

  async function remoteLaunch() {
    if (!isConnected() || !params || !armed || running) return;
    const fanWillRun = (params.mode === 0 || params.tune_fan) && params.fan_duty > 0;
    if (fanWillRun && Number.isFinite(vbatMv) && vbatMv >= VBAT_USB && vbatMv < VBAT_COMP_MIN) {
      setBanner(`警告：电池 ${(vbatMv / 1000).toFixed(2)} V 低于 11.70 V 比赛红线！仍尝试发车（<11.50 V 车端将硬拒绝）。`, 'warn');
    }
    const token = fanWillRun ? 1 : 2;
    remoteLaunched = true;
    setBanner(`已发送远程发车指令：${launchSummary()}（等待车端校零与软启…）`, 'info');
    onLog?.('event', `远程发车：${launchSummary()}`);
    const ok = await sendRaw(commands.proGo(nextRid(), params.rev, token), { paced: true });
    if (ok) startWatchdog();
    else { remoteLaunched = false; setBanner('远程发车命令发送失败', 'bad'); }
    updateUI();
  }

  async function sendSingleGain(key) {
    if (!params) return;
    const val = Number($(`[data-rtp-num="${key}"]`).value);
    if (!Number.isFinite(val) || val < 0) return;
    const scaled = Math.round(val * GAIN_SCALE[key]);
    prevParams = { ...params };
    if (key === 'vkp') {
      await enqueueWrites([
        { key: 'lkp', kind: 'gain', val: scaled, label: `均速 Kp(左) = ${val}` },
        { key: 'rkp', kind: 'gain', val: scaled, label: `均速 Kp(右) = ${val}` },
      ]);
      return;
    }
    if (key === 'vki') {
      await enqueueWrites([
        { key: 'lki', kind: 'gain', val: scaled, label: `均速 Ki(左) = ${val}` },
        { key: 'rki', kind: 'gain', val: scaled, label: `均速 Ki(右) = ${val}` },
      ]);
      return;
    }
    if ((key === 'dirp_cur' || key === 'dird_cur') && $('[data-rtp-band]').checked) {
      const spd = params.race_speed || 125;
      const lo = clamp(spd - 20, 10, 400), hi = clamp(spd + 20, lo, 400);
      await enqueueWrites([{ type: 'band', kind: 'gain', bandKind: key === 'dirp_cur' ? 'p' : 'd', lo, hi, val: scaled, label: `外环 ${key === 'dirp_cur' ? 'Kp' : 'Kd'} [${lo}~${hi}] = ${val}` }]);
    } else {
      await enqueueWrites([{ key, kind: 'gain', val: scaled, label: `${key} = ${val}` }]);
    }
  }

  function scheduleLiveGain(key) {
    clearTimeout(liveTimers.get(key));
    liveTimers.set(key, setTimeout(() => {
      liveTimers.delete(key);
      sendSingleGain(key);
    }, 80));
  }

  async function undoGain() {
    if (!prevParams || !params) return;
    const snap = prevParams;
    prevParams = null;
    const ops = [];
    for (const k of ['lkp', 'lki', 'rkp', 'rki', 'ykp', 'yki', 'dirp_cur', 'dird_cur']) {
      if (Math.abs((snap[k] ?? 0) - (params[k] ?? 0)) > 1e-6) {
        ops.push({ key: k, val: Math.round(snap[k] * GAIN_SCALE[k]), label: `撤销 ${k}→${fmt(snap[k], GAIN_DEC[k])}` });
      }
    }
    if (Number.isFinite(snap.yff) && Number.isFinite(params.yff) && Math.abs(snap.yff - params.yff) > 1e-6) {
      ops.push({ key: 'yff', val: Math.round(snap.yff * 1000), label: `撤销 yff→${fmt(snap.yff, 2)}` });
    }
    if (ops.length) await enqueueWrites(ops);
    else setBanner('当前增益与撤销点一致', 'info');
  }

  async function runQuickInput(raw) {
    const text = raw.trim();
    if (!text) return;
    cmdHistory.unshift(text);
    if (cmdHistory.length > 30) cmdHistory.pop();
    cmdHistIdx = -1;
    const parsed = parseQuickCommand(text);
    if (!parsed) {
      setBanner(`无法识别快捷指令「${text}」：示例 B 140 / L 120 / W -300 / vkp 38 / ykp 18 / yff 1.85 / fan 40 / s`, 'warn');
      return;
    }
    if (parsed.action === 'stop') { await emergencyStop(`快捷指令 ${text}`); return; }
    if (!params) { setBanner('请先点「同步参数」确认车端连接', 'warn'); return; }
    if (parsed.action === 'run') {
      if (parsed.kind === 'W') { switchTab('yaw'); $('[data-rtp-field="test_yaw"]').value = parsed.target; }
      else { switchTab('speed'); $('[data-rtp-field="test_speed"]').value = parsed.target; }
      await sendRunContinuous(parsed.kind, parsed.target);
    } else if (parsed.action === 'tgt') {
      const kind = inYawLoop() ? 'W' : WHEEL_CODES[currentWheelIdx()];
      await sendRunContinuous(kind, parsed.target);
    } else if (parsed.action === 'write') {
      if (parsed.key === 'vkp') {
        await enqueueWrites([
          { key: 'lkp', kind: 'gain', val: parsed.val, label: `均速 Kp(左)=${parsed.display}` },
          { key: 'rkp', kind: 'gain', val: parsed.val, label: `均速 Kp(右)=${parsed.display}` },
        ]);
      } else if (parsed.key === 'vki') {
        await enqueueWrites([
          { key: 'lki', kind: 'gain', val: parsed.val, label: `均速 Ki(左)=${parsed.display}` },
          { key: 'rki', kind: 'gain', val: parsed.val, label: `均速 Ki(右)=${parsed.display}` },
        ]);
      } else if ((parsed.key === 'dirp_cur' || parsed.key === 'dird_cur') && $('[data-rtp-band]').checked) {
        const spd = params.race_speed || 125;
        const lo = clamp(spd - 20, 10, 400), hi = clamp(spd + 20, lo, 400);
        await enqueueWrites([{ type: 'band', kind: 'gain', bandKind: parsed.key === 'dirp_cur' ? 'p' : 'd', lo, hi, val: parsed.val, label: `${text} [${lo}~${hi}]` }]);
      } else {
        await enqueueWrites([{ key: parsed.key, kind: parsed.kind, val: parsed.val, label: `${parsed.key} = ${parsed.display}` }]);
      }
    }
  }

  function switchTab(tab) {
    activeTab = tab;
    host.dataset.activeLoop = tab;
    document.documentElement.dataset.activeLoop = tab;
    if (stageHost) stageHost.dataset.activeLoop = tab;
    $$('[data-rtp-tab]').forEach(b => b.setAttribute('aria-selected', String(b.dataset.rtpTab === tab)));
    $$('[data-rtp-panel]').forEach(p => { p.hidden = p.dataset.rtpPanel !== tab; });
    $$('[data-jump-tab]').forEach(n => n.setAttribute('aria-current', String(n.dataset.jumpTab === tab)));
    onFocusLoop?.(tab);
  }

  function setTopology(next, { sendToCar = false } = {}) {
    const norm = next === '3cascade' ? '3cascade' : '2s1p';
    if (norm === topology && !sendToCar) return;
    topology = norm;
    applyTopologyUI();
    if (sendToCar && isConnected() && params && !running) {
      enqueueWrites([{ key: 'topo', val: norm === '2s1p' ? 1 : 0, label: norm === '2s1p' ? '切换两串一并(2S1P)' : '切换传统三串' }]);
    }
  }

  // ---------- 事件处理 ----------
  on($('[data-rtp-quick-form]'), 'submit', async e => {
    e.preventDefault();
    const inp = $('[data-rtp-quick-input]');
    const txt = inp.value;
    inp.value = '';
    await runQuickInput(txt);
  });
  on($('[data-rtp-quick-input]'), 'keydown', e => {
    if (e.key === 'ArrowUp' && cmdHistory.length) {
      e.preventDefault();
      cmdHistIdx = Math.min(cmdHistory.length - 1, cmdHistIdx + 1);
      e.target.value = cmdHistory[cmdHistIdx];
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      cmdHistIdx = Math.max(-1, cmdHistIdx - 1);
      e.target.value = cmdHistIdx >= 0 ? cmdHistory[cmdHistIdx] : '';
    }
  });

  const handleClick = async e => {
    const b = e.target.closest('button');
    if (!b || !roots.some(r => r.contains(b))) return;
    const d = b.dataset;
    if (d.jumpTab) { switchTab(d.jumpTab); return; }
    if (d.rtpTab) { switchTab(d.rtpTab); return; }
    if (b.hasAttribute('data-rtp-toggle-lr')) {
      showLrIn2s1p = !showLrIn2s1p;
      $('[data-lr-block]').hidden = !showLrIn2s1p;
      b.textContent = showLrIn2s1p ? '收起左右独立微调' : '展开左右独立微调';
      return;
    }
    if (d.yawPreset && params) {
      prevParams = { ...params };
      const kp = d.yawPreset === 'suction' ? 18.0 : 6.5;
      await enqueueWrites([
        { key: 'yff', kind: 'gain', val: 1850, label: '前馈 Kff = 1.85' },
        { key: 'ykp', kind: 'gain', val: Math.round(kp * 10000), label: `角速度 Kp = ${kp}` },
        { key: 'yki', kind: 'gain', val: 0, label: '角速度 Ki = 0' },
      ]);
      return;
    }
    if (b.hasAttribute('data-rtp-sync')) { requestSync(); return; }
    if (b.hasAttribute('data-rtp-stop')) { emergencyStop('点击红色急停按钮'); return; }
    if (b.hasAttribute('data-rtp-go')) { remoteLaunch(); return; }
    if (d.rtpImg !== undefined) {
      imgMode = Number(d.rtpImg);
      imgFps = parseImgFps($('[data-rtp-img-fps]').value);
      updateUI();
      await sendRaw(commands.proImg(imgMode, imgFps), { paced: true });
      setBanner(imgMode ? `已开启实时压缩图传（模式 ${imgMode} · ${fmtImgFps(imgFps)}）` : '已关闭实时图传', 'info');
      return;
    }
    if (d.rtpWheel !== undefined) {
      const w = Number(d.rtpWheel);
      if (running && inSpeedLoop()) {
        enqueueWrites([{ key: 'wheel', val: w, label: `热切轮侧=${WHEEL_NAMES[w]}` }]);
      } else {
        const keepStep = params?.step_mode ?? 3;
        const ops = [{ key: 'wheel', val: w, label: `轮侧=${WHEEL_NAMES[w]}` }];
        if (keepStep !== 1) {
          ops.push({ key: 'mode', val: w + 1, label: `模式=${MODE_NAMES[w + 1]}` });
          if (keepStep === 3) ops.push({ key: 'step_mode', val: 3, label: '保持持续运转' });
        }
        enqueueWrites(ops);
      }
      return;
    }
    if (d.rtpStep !== undefined) {
      enqueueWrites([{ key: 'step_mode', val: Number(d.rtpStep), label: STEP_NAMES[Number(d.rtpStep)] }]);
      return;
    }
    if (d.rtpYawstep !== undefined) {
      const st = Number(d.rtpYawstep);
      const ops = [];
      if (st === 2) ops.push({ key: 'mode', val: 7, label: '角速度自动正反阶跃' });
      else {
        if (!inYawLoop() || params?.mode === 7) ops.push({ key: 'mode', val: 4, label: '角速度原地模式' });
        ops.push({ key: 'step_mode', val: st, label: STEP_NAMES[st] });
      }
      enqueueWrites(ops);
      return;
    }
    if (d.rtpRunSpeed !== undefined) {
      const v = Number(d.rtpRunSpeed);
      $('[data-rtp-field="test_speed"]').value = v;
      await sendRunContinuous(WHEEL_CODES[currentWheelIdx()], v);
      return;
    }
    if (b.hasAttribute('data-rtp-go-speed')) {
      const v = clamp(Math.round(Number($('[data-rtp-field="test_speed"]').value) || 0), 0, 350);
      $('[data-rtp-field="test_speed"]').value = v;
      await sendRunContinuous(WHEEL_CODES[currentWheelIdx()], v);
      return;
    }
    if (b.hasAttribute('data-rtp-apply-low')) {
      const lo = clamp(Math.round(Number($('[data-rtp-field="test_low"]').value) || 0), 0, 350);
      enqueueWrites([{ key: 'test_low', val: lo, label: `多阶低档=${lo}` }]);
      return;
    }
    if (d.rtpRunYaw !== undefined) {
      const w = Number(d.rtpRunYaw);
      $('[data-rtp-field="test_yaw"]').value = w;
      await sendRunContinuous('W', w);
      return;
    }
    if (b.hasAttribute('data-rtp-go-yaw')) {
      const w = clamp(Math.round(Number($('[data-rtp-field="test_yaw"]').value) || 0), -600, 600);
      $('[data-rtp-field="test_yaw"]').value = w;
      await sendRunContinuous('W', w);
      return;
    }
    if (b.hasAttribute('data-rtp-apply-fan') || b.hasAttribute('data-rtp-apply-fan-yaw')) {
      const isYaw = b.hasAttribute('data-rtp-apply-fan-yaw');
      const useFan = $(isYaw ? '[data-rtp-field="tune_fan_yaw"]' : '[data-rtp-field="tune_fan"]').checked ? 1 : 0;
      const pct = clamp(Math.round(Number($(isYaw ? '[data-rtp-field="fan_pct_yaw"]' : '[data-rtp-field="fan_pct"]').value) || 0), 0, 80);
      enqueueWrites([{ key: 'fan', val: pct * 100, label: `负压=${pct}%` }, { key: 'tune_fan', val: useFan, label: useFan ? '调参开负压' : '调参关负压' }]);
      return;
    }
    if (b.hasAttribute('data-rtp-race-mode')) {
      enqueueWrites([{ key: 'mode', val: 0, label: '竞赛循迹模式' }]);
      return;
    }
    if (b.hasAttribute('data-rtp-apply-outer')) {
      const rs = clamp(Math.round(Number($('[data-rtp-field="race_speed"]').value) || 125), 40, 350);
      const ts = clamp(Math.round(Number($('[data-rtp-field="turn_speed"]').value) || 80), 40, 300);
      const yc = clamp(Math.round(Number($('[data-rtp-field="yaw_cap"]').value) || 920), 100, 1500);
      const fp = clamp(Math.round(Number($('[data-rtp-field="fan_pct_outer"]').value) || 0), 0, 80);
      const ops = [];
      if (!running && params?.mode !== 0) ops.push({ key: 'mode', val: 0, label: '外环模式' });
      ops.push(
        { key: 'race_speed', val: rs, label: `直道速=${rs}` },
        { key: 'turn_speed', val: ts, label: `弯道速=${ts}` },
        { key: 'yaw_cap', val: yc, label: `角速上限=${yc}` },
        { key: 'fan', val: fp * 100, label: `负压=${fp}%` },
      );
      enqueueWrites(ops);
      return;
    }
    if (d.rtpNudge) {
      const key = d.rtpNudge, step = Number(d.step) * Number(d.dir);
      const num = $(`[data-rtp-num="${key}"]`), sl = $(`[data-rtp-slider="${key}"]`);
      const next = clamp(+(Number(num.value) + step).toFixed(GAIN_DEC[key]), Number(num.min), Number(num.max));
      num.value = next;
      if (sl) sl.value = String(next);
      sendSingleGain(key);
      return;
    }
    if (d.rtpSendGain) { sendSingleGain(d.rtpSendGain); return; }
    if (b.hasAttribute('data-rtp-copy-lr') && params) {
      prevParams = { ...params };
      enqueueWrites([
        { key: 'rkp', kind: 'gain', val: Math.round(params.lkp * 100), label: `右 Kp=${params.lkp}` },
        { key: 'rki', kind: 'gain', val: Math.round(params.lki * 1000), label: `右 Ki=${params.lki}` },
      ]);
      return;
    }
    if (b.hasAttribute('data-rtp-undo')) { undoGain(); return; }
    if (b.hasAttribute('data-rtp-cap')) {
      setBanner('正在请求板载最近 4 秒高清波形 (#X CAP)…', 'info');
      await sendRaw(commands.proCapture(), { paced: true });
      return;
    }
  };
  for (const r of roots) on(r, 'click', handleClick);
  if (camStreamHost) {
    on(camStreamHost, 'change', e => {
      const t = e.target;
      if (t.matches('[data-rtp-img-fps]') && imgMode > 0) {
        imgFps = parseImgFps(t.value);
        sendRaw(commands.proImg(imgMode, imgFps), { paced: true });
      }
    });
  }

  on(host, 'input', e => {
    const t = e.target;
    if (t.matches('[data-rtp-slider]')) {
      const key = t.dataset.rtpSlider;
      $(`[data-rtp-num="${key}"]`).value = t.value;
      scheduleLiveGain(key);
    } else if (t.matches('[data-rtp-num]')) {
      const key = t.dataset.rtpNum, sl = $(`[data-rtp-slider="${key}"]`);
      if (sl) sl.value = t.value;
    }
  });

  on(host, 'keydown', e => {
    if (e.key !== 'Enter') return;
    const t = e.target;
    if (t.matches('[data-rtp-num]')) { e.preventDefault(); sendSingleGain(t.dataset.rtpNum); }
    else if (t.matches('[data-rtp-field="test_speed"]')) { e.preventDefault(); sendRunContinuous(WHEEL_CODES[currentWheelIdx()], Number(t.value) || 0); }
    else if (t.matches('[data-rtp-field="test_yaw"]')) { e.preventDefault(); sendRunContinuous('W', Number(t.value) || 0); }
  });

  on(host, 'change', e => {
    const t = e.target;
    if (t.matches('[data-rtp-arm]')) {
      armed = t.checked;
      setBanner(armed ? '已解锁：现在直接点下方任意速度/角速度按钮（或输 B 140 / W 300）即可持续运转；随时按 Esc / 空格急停！' : '已重新锁定远程发车', armed ? 'warn' : 'info');
      updateUI();
    } else if (t.matches('[data-rtp-slider]')) {
      clearTimeout(liveTimers.get(t.dataset.rtpSlider));
      liveTimers.delete(t.dataset.rtpSlider);
      sendSingleGain(t.dataset.rtpSlider);
    } else if (t.matches('[data-rtp-img-fps]') && imgMode > 0) {
      imgFps = parseImgFps(t.value);
      sendRaw(commands.proImg(imgMode, imgFps), { paced: true });
    }
  });

  on(window, 'keydown', e => {
    if (!isConnected()) return;
    const inInput = e.target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(e.target.tagName);
    if (e.key === 'Escape' || (e.code === 'Space' && !inInput && (running || armed))) {
      e.preventDefault();
      emergencyStop(`快捷键 ${e.key}`);
    }
  });

  applyTopologyUI();
  mountFluidTabs($('.rtp-loop-tabs'), { kind: 'line' });

  return {
    setTopology,
    getVbatMv: () => vbatMv,
    updateTelemetry({ spd, encL, encR, gyro, yawTgt, errPx, pwmCm, pwmDm, pwmL, pwmR }) {
      const setNode = (k, txt) => {
        const el = nodeValRefs[k];
        if (!el || nodeValCache[k] === txt) return;
        nodeValCache[k] = txt;
        el.textContent = txt;
      };
      if (Number.isFinite(errPx)) setNode('outer', `err ${errPx >= 0 ? '+' : ''}${errPx.toFixed(0)} px`);
      if (Number.isFinite(gyro)) setNode('yaw', `${gyro.toFixed(0)} / ${Number.isFinite(yawTgt) ? yawTgt.toFixed(0) : '—'} °/s`);
      if (Number.isFinite(spd)) {
        setNode('speed', topology === '2s1p'
          ? `v_avg ${spd.toFixed(0)} cnt`
          : `L ${Number.isFinite(encL) ? encL.toFixed(0) : '—'} · R ${Number.isFinite(encR) ? encR.toFixed(0) : '—'}`);
      }
      if (topology === '2s1p' && Number.isFinite(pwmCm) && Number.isFinite(pwmDm)) {
        setNode('mixer', `CM ${Math.round(pwmCm)} · DM ${Math.round(pwmDm)}`);
      } else if (Number.isFinite(pwmL) && Number.isFinite(pwmR)) {
        setNode('mixer', `L ${Math.round(pwmL)} · R ${Math.round(pwmR)}`);
      }
    },
    onConnected() {
      params = null; prevParams = null; armed = false; running = false; remoteLaunched = false; stopReason = 0; vbatMv = NaN;
      $('[data-rtp-arm]').checked = false;
      updateUI();
      setTimeout(() => { if (isConnected() && !params) requestSync(); }, 350);
    },
    onDisconnected() {
      stopWatchdog();
      clearTimeout(pendingTimer);
      for (const tm of liveTimers.values()) clearTimeout(tm);
      liveTimers.clear();
      params = null; armed = false; running = false; remoteLaunched = false; writeQueue = [];
      $('[data-rtp-arm]').checked = false;
      setBanner('串口已断开，远程发车已自动上锁。', 'idle');
      updateUI();
    },
    onEvent(ev) {
      if (ev.type === 'pro-params') {
        clearTimeout(pendingTimer);
        params = ev;
        if (ev.topo === 0 || ev.topo === 1) {
          const nextTopo = ev.topo === 1 ? '2s1p' : '3cascade';
          if (nextTopo !== topology) {
            topology = nextTopo;
            applyTopologyUI();
            onTopologyChange?.(topology);
          }
        }
        syncInputsFromParams(ev);
        if (revWaiter && ev.rev >= revWaiter.targetRev) revWaiter.resolve(true);
        const topoTag = topology === '2s1p' ? `2S1P · 均速 ${fmt((ev.lkp + ev.rkp) / 2, 1)}/${fmt((ev.lki + ev.rki) / 2, 2)} · 角 ${fmt(ev.ykp, 1)}${Number.isFinite(ev.yff) ? ` (ff ${fmt(ev.yff, 2)})` : ''}` : `三串 · 左 ${ev.lkp}/${ev.lki} · 右 ${ev.rkp}/${ev.rki} · 角 ${ev.ykp}/${ev.yki}`;
        setBanner(`参数同步（rev ${ev.rev} · ${MODE_NAMES[ev.mode] || ev.mode} · ${topoTag} · 外 ${ev.dirp_cur}/${ev.dird_cur}）`, 'good');
        updateUI();
      } else if (ev.type === 'pro-ack') {
        setBanner(`${running ? '热改生效：' : '写入确认：'}${ev.key}（rev ${ev.rev}）`, 'good');
      } else if (ev.type === 'pro-run') {
        if (ev.action === 'STOP') {
          running = false; remoteLaunched = false; stopReason = ev.a || 25; stopWatchdog();
          setBanner(`车端已确认紧急停车（停车码 ${stopReason}）`, 'warn');
        } else if (ev.action === 'GO') {
          running = true; stopReason = 0;
          setBanner(`车端已接受远程发车（${MODE_NAMES[ev.a] || ev.a} · 目标 ${ev.b}），看门狗心跳保护中…`, 'good');
        } else if (ev.action === 'RUN') {
          running = ev.b === 1;
          if (running) { stopReason = 0; remoteLaunched = true; startWatchdog(); }
          const kName = { 1: '左轮', 2: '右轮', 3: '双轮均速', 4: '角速度' }[ev.rid] || '目标';
          setBanner(running ? `持续运转生效：${kName} → ${ev.a}（不停车 · 可随时改 PID 或切速）` : `已预设：${kName} = ${ev.a}`, 'good');
        } else if (ev.action === 'TGT') {
          setBanner(`运行中目标已无缝切换为 ${ev.a}`, 'good');
        } else if (ev.action === 'IMG') {
          imgMode = ev.a; imgFps = parseImgFps(ev.b);
          setBanner(imgMode ? `车端已开启实时压缩图传（模式 ${imgMode} · ${fmtImgFps(imgFps)}）` : '车端已关闭实时图传', 'good');
        }
        updateUI();
      } else if (ev.type === 'pro-error') {
        const msg = PRO_ERRORS[ev.code] || `车端拒绝：${ev.code}`;
        setBanner(msg, 'bad');
        if (revWaiter) revWaiter.resolve(false);
        if (ev.code === 'REVISION') requestSync();
        if (ev.code === 'LOW_BAT' || ev.code === 'START_REFUSED' || ev.code === 'NO_IMU') {
          remoteLaunched = false; stopWatchdog();
        }
        updateUI();
      } else if (ev.type === 'm2') {
        const r = ev.row;
        const wasRunning = running;
        const prevStop = stopReason;
        running = r.launch_flag === 1;
        stopReason = r.stop_reason;
        if (wasRunning && !running) {
          remoteLaunched = false;
          stopWatchdog();
        }
        if (params && Number.isFinite(r.cfg_rev) && r.cfg_rev !== params.rev && !flushing) {
          requestSync();
        }
        // Only sweep UI controls when running/stop state actually transitions (eliminates 50Hz DOM storm!)
        if (running !== wasRunning || stopReason !== prevStop) updateUI();
      } else if (ev.type === 'status') {
        const f = ev.fields;
        if (Number.isFinite(f.vbat_mv)) {
          vbatMv = f.vbat_mv;
          updateVbatBadge();
        }
        let changed = false;
        if (f.tune === 1 && Number.isInteger(f.running)) {
          const nextRun = f.running === 1;
          if (nextRun !== running) { running = nextRun; changed = true; }
          if (!running) { remoteLaunched = false; stopWatchdog(); }
          if (Number.isInteger(f.stop) && f.stop !== stopReason) { stopReason = f.stop; changed = true; }
        }
        if (changed) updateUI();
      }
    },
    destroy() {
      stopWatchdog();
      clearTimeout(pendingTimer);
      for (const tm of liveTimers.values()) clearTimeout(tm);
      liveTimers.clear();
      ac.abort();
      host.replaceChildren();
    },
  };
}
