// SPDX-License-Identifier: GPL-3.0-or-later
// AI 自动调参面板（内置零依赖循证规则整定器 + OpenAI 兼容 / Ollama / Claude HTTP 接口）
// 搭配后端 ai_tuner.py 硬安全护栏（步长限幅、@GP1 回读核验、劣化自动回滚、3S 电压底线、实车架空确认）。
import { esc } from './esc.js';

const LOOP_OPTIONS = [
  { id: 'yaw_2s1p', topo: '2s1p', label: '两串一并 · 角速度直出 PWM (yff / ykp / yki)', target: 300, low: 0, unit: '°/s', view: 'yaw' },
  { id: 'speed_2s1p', topo: '2s1p', label: '两串一并 · 均速并联环 (vkp / vki)', target: 150, low: 90, unit: 'cnt/2ms', view: 'speed' },
  { id: 'yaw_3c', topo: '3cascade', label: '传统三串 · 角速度中环 (ykp / yki)', target: 260, low: 0, unit: '°/s', view: 'yaw' },
  { id: 'speed_3c', topo: '3cascade', label: '传统三串 · 双轮速度内环 (lkp/lki/rkp/rki)', target: 140, low: 85, unit: 'cnt/2ms', view: 'speed' },
  { id: 'outer', topo: 'both', label: '视觉外环 · 巡线 PD (dirp / dird)', target: 125, low: 80, unit: 'cnt/2ms', view: 'outer' },
];

const PHASE_LABELS = {
  idle: ['待命', 'idle'],
  starting: ['启动中', 'busy'],
  measuring: ['4ms 阶跃采集中', 'busy'],
  analyzing: ['指标分析 / 生成建议', 'busy'],
  applying: ['写参回读核验', 'busy'],
  rollback: ['劣化自动回滚', 'warn'],
  stopping: ['正在急停', 'bad'],
  done: ['已完成', 'good'],
  stopped_by_user: ['已手动停止', 'warn'],
  error: ['异常中止', 'bad'],
};

const PREF_AI_CFG = 'tp-ai-tuner-cfg-v1';

function loadSavedCfg() {
  try {
    const raw = JSON.parse(localStorage.getItem(PREF_AI_CFG) || '{}');
    return typeof raw === 'object' && raw ? raw : {};
  } catch {
    return {};
  }
}

function saveCfg(partial) {
  try {
    const prev = loadSavedCfg();
    const next = { ...prev, ...partial };
    delete next.api_key; // 密钥永不写入 localStorage
    localStorage.setItem(PREF_AI_CFG, JSON.stringify(next));
  } catch {}
}

function fmtParams(p) {
  if (!p || typeof p !== 'object') return '—';
  return Object.entries(p)
    .map(([k, v]) => `${k}=${Number.isFinite(Number(v)) ? Number(v).toFixed(k.includes('i') || k === 'yff' ? 3 : 2).replace(/\.?0+$/, '') : v}`)
    .join(' · ');
}

export function mountAiTuner(host, { getTransport, isConnected, onLog, onFocusView, onStateBadge }) {
  const saved = loadSavedCfg();
  let open = false;
  let pollTimer = 0;
  let lastSnap = null;
  let lastStatus = null;
  let busyAction = false;

  host.className = 'tp-card ait-panel';
  host.hidden = true;
  host.innerHTML = `
    <div class="ait-head">
      <div class="ait-head-left">
        <div class="ait-title-row">
          <strong class="tp-card-title">AI 自动调参</strong>
          <span class="ait-badge" data-ait-phase data-level="idle">待命</span>
          <span class="ait-port-tag" data-ait-port-tag>未连接</span>
        </div>
        <p class="ait-sub">基于 4 ms 板载阶跃与有界安全护栏自动迭代整定 · 劣化自动回滚至历史最佳稳定参数</p>
      </div>
      <div class="ait-head-actions">
        <button type="button" data-ait-refresh>刷新快照</button>
        <button type="button" data-ait-close>收起面板</button>
      </div>
    </div>

    <div class="ait-warn-bar" data-ait-transport-warn hidden>
      AI 自动调参需通过「本机串口桥」控制与回读，请在顶栏连接方式切回本机串口桥。
    </div>

    <div class="ait-grid">
      <!-- 左侧：调参任务与模型配置 -->
      <div class="ait-box">
        <h5>整定目标与安全护栏 <small data-ait-bounds-hint>护栏限幅生效</small></h5>
        <div class="ait-form-grid">
          <label class="ait-field ait-span-2">
            <span>整定环路</span>
            <select data-ait-loop>
              ${LOOP_OPTIONS.map(o => `<option value="${o.id}">${esc(o.label)}</option>`).join('')}
            </select>
          </label>
          <label class="ait-field">
            <span>阶跃目标 <small data-ait-unit>°/s</small></span>
            <input type="number" data-ait-target value="300" step="10">
          </label>
          <label class="ait-field">
            <span>最大轮数</span>
            <input type="number" data-ait-rounds value="6" min="2" max="12" step="1">
          </label>
          <label class="ait-field">
            <span>偏好策略</span>
            <select data-ait-goal>
              <option value="balanced">均衡（默认）</option>
              <option value="fast_response">响应速度优先</option>
              <option value="low_overshoot">低超调优先</option>
              <option value="stability_first">稳健抗振优先</option>
            </select>
          </label>
          <label class="ait-field ait-check-field" data-ait-dry-wrap title="两串一并角速度环专用：干地无负压时自动把 Kp 上限锁在 14.0">
            <input type="checkbox" data-ait-dry>
            <span>干地限幅 (ykp ≤ 14)</span>
          </label>
        </div>

        <div class="ait-divider"></div>

        <h5>决策引擎 <small>零第三方依赖</small></h5>
        <div class="ait-form-grid">
          <label class="ait-field ait-span-2">
            <span>模式选择</span>
            <select data-ait-provider>
              <option value="rule">内置循证规则整定器（零 API Key · 离线可用）</option>
              <option value="openai">OpenAI 兼容接口（DeepSeek / 通义千问 / OpenAI / vLLM）</option>
              <option value="ollama">Ollama 本地大模型（127.0.0.1:11434）</option>
              <option value="anthropic">Anthropic Claude API</option>
            </select>
          </label>
          <div class="ait-llm-fields ait-span-2" data-ait-llm-box hidden>
            <div class="ait-form-grid">
              <label class="ait-field ait-span-2">
                <span>API Base URL</span>
                <input type="text" data-ait-url placeholder="https://api.deepseek.com/v1" autocomplete="off" spellcheck="false">
              </label>
              <label class="ait-field">
                <span>Model 名称</span>
                <input type="text" data-ait-model placeholder="deepseek-chat" autocomplete="off" spellcheck="false">
              </label>
              <label class="ait-field">
                <span>API Key（仅内存使用，不落盘）</span>
                <input type="password" data-ait-key placeholder="可留空读取环境变量" autocomplete="off">
              </label>
            </div>
          </div>
          <label class="ait-field ait-span-2">
            <span>调参前补充说明（可选）</span>
            <input type="text" data-ait-note placeholder="例如：车重偏大、弯道别超调超过 6%，优先保稳态精度" maxlength="260">
          </label>
        </div>

        <div class="ait-realcar-gate" data-ait-realcar-gate hidden>
          <label class="ait-gate-chk">
            <input type="checkbox" data-ait-realcar-ok>
            <span>实车安全联锁：已将车轮架空离地、确认 3S 电压 ≥ 11.7V 且随时准备按 Esc 急停</span>
          </label>
        </div>

        <div class="ait-actions">
          <button type="button" class="tp-btn-accent" data-ait-start disabled>开始 AI 自动调参</button>
          <button type="button" data-ait-step disabled title="只做 1 趟 4 ms 阶跃基线测量，不修改参数">单步测基线</button>
          <button type="button" data-ait-rollback disabled title="恢复并回读验证当前会话最佳稳定参数">恢复最佳参数</button>
          <button type="button" class="ait-stop-btn" data-ait-stop disabled>停止并急停</button>
        </div>
      </div>

      <!-- 右侧：实时实验看板与迭代历史 -->
      <div class="ait-box ait-board">
        <div class="ait-board-head">
          <h5>实验进度与最佳参数</h5>
          <span class="ait-progress-txt" data-ait-progress>0 / 6 轮</span>
        </div>
        <div class="ait-status-line" data-ait-msg>连接 MOCK0 虚拟小车或实车串口后，即可点击「开始 AI 自动调参」。</div>

        <div class="ait-best-card" data-ait-best>
          <div>
            <span class="ait-best-lbl">当前验证最佳稳定参数</span>
            <strong class="ait-best-params" data-ait-best-params>尚无已验证轮次</strong>
          </div>
          <div class="ait-best-metrics" data-ait-best-metrics>等待基线测量</div>
        </div>

        <div class="ait-bounds-strip" data-ait-cur-params>当前车端参数：未同步</div>

        <div class="ait-table-wrap">
          <table class="ait-table">
            <thead>
              <tr>
                <th>轮次</th>
                <th>参数快照</th>
                <th>上升</th>
                <th>超调</th>
                <th>静差</th>
                <th>饱和</th>
                <th>状态</th>
                <th>动作与依据</th>
              </tr>
            </thead>
            <tbody data-ait-tbody>
              <tr><td colspan="8" class="ait-empty">暂无实验记录。点击「开始 AI 自动调参」或「单步测基线」开始测量。</td></tr>
            </tbody>
          </table>
        </div>

        <div class="ait-skill-hint">
          <span>外部 Agent 接入：</span>
          <code>skills/tc264-pid-tuner/SKILL.md</code>
          <span>· CLI：</span>
          <code>python skills/tc264-pid-tuner/scripts/tunepro_ai_cli.py snapshot</code>
        </div>
      </div>
    </div>
  `;

  const $ = q => host.querySelector(q);
  const elLoop = $('[data-ait-loop]');
  const elTarget = $('[data-ait-target]');
  const elRounds = $('[data-ait-rounds]');
  const elGoal = $('[data-ait-goal]');
  const elDry = $('[data-ait-dry]');
  const elDryWrap = $('[data-ait-dry-wrap]');
  const elProvider = $('[data-ait-provider]');
  const elLlmBox = $('[data-ait-llm-box]');
  const elUrl = $('[data-ait-url]');
  const elModel = $('[data-ait-model]');
  const elKey = $('[data-ait-key]');
  const elNote = $('[data-ait-note]');
  const elRealGate = $('[data-ait-realcar-gate]');
  const elRealOk = $('[data-ait-realcar-ok]');
  const btnStart = $('[data-ait-start]');
  const btnStep = $('[data-ait-step]');
  const btnRollback = $('[data-ait-rollback]');
  const btnStop = $('[data-ait-stop]');

  // Restore non-secret UI prefs
  if (saved.provider && ['rule', 'openai', 'ollama', 'anthropic'].includes(saved.provider)) elProvider.value = saved.provider;
  if (saved.base_url) elUrl.value = saved.base_url;
  if (saved.model) elModel.value = saved.model;
  if (saved.goal && ['balanced', 'fast_response', 'low_overshoot', 'stability_first'].includes(saved.goal)) elGoal.value = saved.goal;
  if (saved.max_rounds) elRounds.value = String(saved.max_rounds);

  function syncProviderUI() {
    const prov = elProvider.value;
    elLlmBox.hidden = prov === 'rule';
    if (prov === 'ollama') {
      elUrl.placeholder = 'http://127.0.0.1:11434/v1';
      elModel.placeholder = 'qwen2.5:7b';
      if (!elUrl.value.trim()) elUrl.value = 'http://127.0.0.1:11434/v1';
    } else if (prov === 'anthropic') {
      elUrl.placeholder = 'https://api.anthropic.com/v1';
      elModel.placeholder = 'claude-3-5-sonnet-latest';
    } else if (prov === 'openai') {
      elUrl.placeholder = 'https://api.deepseek.com/v1';
      elModel.placeholder = 'deepseek-chat / gpt-4o-mini';
    }
    saveCfg({ provider: prov, base_url: elUrl.value.trim(), model: elModel.value.trim() });
  }

  function syncLoopUI(updateTarget = true) {
    const opt = LOOP_OPTIONS.find(o => o.id === elLoop.value) || LOOP_OPTIONS[0];
    $('[data-ait-unit]').textContent = opt.unit;
    if (updateTarget) elTarget.value = String(opt.target);
    elDryWrap.hidden = opt.id !== 'yaw_2s1p';
  }

  function updateControls() {
    const usingBridge = getTransport() === 'bridge';
    $('[data-ait-transport-warn]').hidden = usingBridge;
    const conn = usingBridge && isConnected();
    const active = Boolean(lastStatus?.active);
    const isMock = Boolean(lastStatus?.is_mock ?? lastSnap?.is_mock);
    const port = lastStatus?.port || lastSnap?.port || '';

    const portTag = $('[data-ait-port-tag]');
    if (!conn) {
      portTag.textContent = '未连接';
      portTag.dataset.level = 'idle';
    } else if (isMock) {
      portTag.textContent = `${port} 虚拟小车 · 安全沙箱`;
      portTag.dataset.level = 'good';
    } else {
      portTag.textContent = `${port} 实车串口 · 需架空确认`;
      portTag.dataset.level = 'warn';
    }

    elRealGate.hidden = !conn || isMock;
    const safeToRun = conn && (isMock || elRealOk.checked);

    btnStart.disabled = !safeToRun || active || busyAction;
    btnStep.disabled = !safeToRun || active || busyAction;
    btnRollback.disabled = !safeToRun || active || busyAction || !lastStatus?.best;
    btnStop.disabled = !conn || (!active && !busyAction);

    elLoop.disabled = active;
    elTarget.disabled = active;
    elRounds.disabled = active;
    elGoal.disabled = active;
    elDry.disabled = active;
    elProvider.disabled = active;

    onStateBadge?.({ active, phase: lastStatus?.phase || 'idle' });
  }

  function renderStatus(st) {
    if (!st) return;
    lastStatus = st;
    const [phaseText, level] = PHASE_LABELS[st.phase] || [st.phase || '待命', 'idle'];
    const phEl = $('[data-ait-phase]');
    phEl.textContent = phaseText;
    phEl.dataset.level = level;

    $('[data-ait-progress]').textContent = `${st.round || 0} / ${st.max_rounds || Number(elRounds.value) || 6} 轮`
      + (st.rejected_count ? ` · 避坑 ${st.rejected_count}` : '');
    $('[data-ait-msg]').textContent = st.message || '就绪';

    if (st.best) {
      const m = st.best.metrics || {};
      $('[data-ait-best-params]').textContent = `第 ${st.best.round} 轮 · ${fmtParams(st.best.parameters)}`;
      $('[data-ait-best-metrics]').textContent =
        `评分 ${st.best.score} · 上升 ${m.rise_ms ?? '—'} ms · 超调 ${m.overshoot_pct ?? '—'}% · 静差 ${m.steady_err_pct ?? '—'}%`;
    } else {
      $('[data-ait-best-params]').textContent = '尚无已验证轮次';
      $('[data-ait-best-metrics]').textContent = '等待基线测量';
    }

    const rows = Array.isArray(st.history) ? st.history : [];
    const tbody = $('[data-ait-tbody]');
    if (!rows.length) {
      tbody.innerHTML = '<tr><td colspan="8" class="ait-empty">暂无实验记录。点击「开始 AI 自动调参」或「单步测基线」开始测量。</td></tr>';
    } else {
      tbody.innerHTML = rows.map(r => {
        const m = r.metrics || {};
        const stLevel = m.status === 'STABLE' ? 'good' : (m.status === 'OSCILLATING' || m.status === 'SATURATED') ? 'bad' : 'warn';
        const actLevel = r.action === 'ROLLBACK' ? 'bad' : (r.action === 'DONE' || r.action === 'HOLD_VERIFY') ? 'good' : 'idle';
        const notes = (r.guardrail_notes || []).length
          ? `<div class="ait-notes">护栏：${esc(r.guardrail_notes.join('；'))}</div>` : '';
        return `<tr class="${r.is_best ? 'is-best' : ''}">
          <td><b>#${r.round}</b>${r.is_best ? '<span class="ait-star">★</span>' : ''}</td>
          <td class="ait-mono">${esc(fmtParams(r.parameters))}</td>
          <td class="ait-mono">${m.rise_ms ?? '—'} ms</td>
          <td class="ait-mono">${m.overshoot_pct ?? '—'}%</td>
          <td class="ait-mono">${m.steady_err_pct ?? '—'}%</td>
          <td class="ait-mono">${m.pwm_sat_pct ?? '—'}%</td>
          <td><span class="ait-tag" data-level="${stLevel}">${esc(m.status || '—')}</span></td>
          <td>
            <span class="ait-tag" data-level="${actLevel}">${esc(r.action || '—')}</span>
            <span class="ait-reason">${esc(r.summary || '')}</span>
            ${notes}
          </td>
        </tr>`;
      }).join('');
    }
    updateControls();
  }

  async function fetchSnapshot() {
    if (getTransport() !== 'bridge') { updateControls(); return; }
    try {
      const qs = new URLSearchParams({ loop: elLoop.value, dry: elDry.checked ? '1' : '0' });
      const r = await fetch(`/api/ai/snapshot?${qs}`, { cache: 'no-store' });
      if (!r.ok) return;
      const snap = await r.json();
      lastSnap = snap;
      const curStr = snap.current_parameters ? fmtParams(snap.current_parameters) : '未同步';
      const bStr = snap.bounds
        ? Object.entries(snap.bounds).map(([k, v]) => `${k}[${v.min}~${v.max}, ±${v.max_step}]`).join(' · ')
        : '';
      $('[data-ait-cur-params]').textContent = `当前车端参数：${curStr}`;
      $('[data-ait-bounds-hint]').textContent = bStr || '护栏限幅生效';
      if (snap.tuner) {
        renderStatus({
          ...snap.tuner,
          connected: snap.connected,
          port: snap.port,
          is_mock: snap.is_mock,
          vbat_mv: snap.vbat_mv,
        });
      }
    } catch {}
  }

  async function pollStatus() {
    try {
      const r = await fetch('/api/ai/status', { cache: 'no-store' });
      if (!r.ok) return;
      const st = await r.json();
      renderStatus(st);
      if (!st.active && pollTimer) {
        clearInterval(pollTimer);
        pollTimer = 0;
        fetchSnapshot();
      }
    } catch {}
  }

  function ensurePolling() {
    if (!pollTimer) pollTimer = setInterval(pollStatus, 650);
  }

  async function startAutoTune() {
    const opt = LOOP_OPTIONS.find(o => o.id === elLoop.value) || LOOP_OPTIONS[0];
    onFocusView?.(opt.view);
    saveCfg({
      provider: elProvider.value,
      base_url: elUrl.value.trim(),
      model: elModel.value.trim(),
      goal: elGoal.value,
      max_rounds: Number(elRounds.value) || 6,
    });
    busyAction = true;
    updateControls();
    try {
      const r = await fetch('/api/ai/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          loop: elLoop.value,
          provider: elProvider.value,
          base_url: elUrl.value.trim(),
          model: elModel.value.trim(),
          api_key: elKey.value.trim(),
          target: Number(elTarget.value) || opt.target,
          low: opt.low,
          max_rounds: Number(elRounds.value) || 6,
          goal: elGoal.value,
          dry_mode: elDry.checked,
          user_note: elNote.value.trim(),
          allow_real_car: elRealOk.checked,
          confirm_wheels_off_ground: elRealOk.checked,
        }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      renderStatus(data);
      ensurePolling();
      onLog?.('event', `启动 AI 自动调参：${opt.label}（${elProvider.value}）`);
    } catch (e) {
      onLog?.('error', `AI 自动调参启动失败：${e.message}`);
      $('[data-ait-msg]').textContent = `启动失败：${e.message}`;
    } finally {
      busyAction = false;
      updateControls();
    }
  }

  async function runOneStep() {
    const opt = LOOP_OPTIONS.find(o => o.id === elLoop.value) || LOOP_OPTIONS[0];
    onFocusView?.(opt.view);
    busyAction = true;
    $('[data-ait-msg]').textContent = `正在执行单次 4 ms 阶跃基线测试（${opt.label}）…`;
    updateControls();
    try {
      const r = await fetch('/api/ai/step-test', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          loop: elLoop.value,
          target: Number(elTarget.value) || opt.target,
          low: opt.low,
          dry_mode: elDry.checked,
          allow_real_car: elRealOk.checked,
          confirm_wheels_off_ground: elRealOk.checked,
        }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      const m = data.metrics;
      const summary = `单步基线完成：上升 ${m.rise_ms} ms · 超调 ${m.overshoot_pct}% · 静差 ${m.steady_err_pct}% · 饱和 ${m.pwm_sat_pct}% · 状态 ${m.status}（评分 ${data.score}）`;
      $('[data-ait-msg]').textContent = summary;
      onLog?.('event', summary);
      await fetchSnapshot();
    } catch (e) {
      $('[data-ait-msg]').textContent = `单步测量失败：${e.message}`;
      onLog?.('error', `单步测量失败：${e.message}`);
    } finally {
      busyAction = false;
      updateControls();
    }
  }

  async function doRollback() {
    busyAction = true;
    updateControls();
    try {
      const r = await fetch('/api/ai/rollback', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          allow_real_car: elRealOk.checked,
          confirm_wheels_off_ground: elRealOk.checked,
        }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      onLog?.('event', `已回滚至第 ${data.restored_round} 轮最佳参数：${fmtParams(data.parameters)}`);
      await fetchSnapshot();
    } catch (e) {
      onLog?.('error', `回滚失败：${e.message}`);
    } finally {
      busyAction = false;
      updateControls();
    }
  }

  async function stopAll(estop = true) {
    try {
      const r = await fetch('/api/ai/stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ estop }),
      });
      if (r.ok) renderStatus(await r.json());
      if (estop) onLog?.('event', '已中止 AI 自动调参并触发急停');
    } catch {}
  }

  elProvider.addEventListener('change', syncProviderUI);
  elUrl.addEventListener('change', syncProviderUI);
  elModel.addEventListener('change', syncProviderUI);
  elLoop.addEventListener('change', () => { syncLoopUI(true); fetchSnapshot(); });
  elDry.addEventListener('change', fetchSnapshot);
  elRealOk.addEventListener('change', updateControls);
  $('[data-ait-refresh]').addEventListener('click', fetchSnapshot);
  $('[data-ait-close]').addEventListener('click', () => setOpen(false));
  btnStart.addEventListener('click', startAutoTune);
  btnStep.addEventListener('click', runOneStep);
  btnRollback.addEventListener('click', doRollback);
  btnStop.addEventListener('click', () => stopAll(true));

  function setOpen(yes) {
    open = Boolean(yes);
    host.hidden = !open;
    onStateBadge?.({ active: Boolean(lastStatus?.active), phase: lastStatus?.phase || 'idle', open });
    if (open) {
      fetchSnapshot();
      if (lastStatus?.active) ensurePolling();
    }
  }

  function setTopology(topo) {
    const want = topo === '3cascade' ? 'yaw_3c' : 'yaw_2s1p';
    const curOpt = LOOP_OPTIONS.find(o => o.id === elLoop.value);
    if (!lastStatus?.active && curOpt && curOpt.topo !== 'both' && curOpt.topo !== topo) {
      elLoop.value = want;
      syncLoopUI(true);
      if (open) fetchSnapshot();
    }
  }

  syncProviderUI();
  syncLoopUI(true);
  updateControls();

  return {
    isOpen: () => open,
    setOpen,
    toggle: () => setOpen(!open),
    setTopology,
    onConnected: () => { elRealOk.checked = false; fetchSnapshot(); },
    onDisconnected: () => { elRealOk.checked = false; if (pollTimer) { clearInterval(pollTimer); pollTimer = 0; } updateControls(); },
    onEmergencyStop: () => { if (lastStatus?.active) stopAll(true); },
  };
}
