// SPDX-License-Identifier: GPL-3.0-or-later
// AI 调参搭档（实时对话 + 物理弹簧推理强度与模型切换浮层 + 4ms 阶跃自主调参）
// 搭配后端 ai_tuner.py 硬安全护栏（步长限幅、@GP1 回读核验、劣化自动回滚、3S 电压底线、实车架空确认）。
import { esc } from './esc.js';
import { mountEffortSlider } from './robot-optics.js';

const LOOP_OPTIONS = [
  { id: 'yaw_2s1p', topo: '2s1p', label: '2S1P · 角速度直出 PWM', target: 300, low: 0, unit: '°/s', view: 'yaw' },
  { id: 'speed_2s1p', topo: '2s1p', label: '2S1P · 均速并联环', target: 150, low: 90, unit: 'cnt', view: 'speed' },
  { id: 'yaw_3c', topo: '3cascade', label: '三串 · 角速度中环', target: 260, low: 0, unit: '°/s', view: 'yaw' },
  { id: 'speed_3c', topo: '3cascade', label: '三串 · 双轮速度内环', target: 140, low: 85, unit: 'cnt', view: 'speed' },
  { id: 'outer', topo: 'both', label: '视觉外环 · 巡线 PD', target: 125, low: 80, unit: 'cnt', view: 'outer' },
];

const EFFORT_LEVELS = [
  { id: 'fast',   label: '极速', en: 'Fast',   rounds: 3, desc: '秒级直答 · 3 轮快测' },
  { id: 'low',    label: '轻量', en: 'Low',    rounds: 4, desc: '简明建议 · 4 轮整定' },
  { id: 'medium', label: '均衡', en: 'Medium', rounds: 6, desc: '速度与深度均衡 · 6 轮' },
  { id: 'high',   label: '深度', en: 'High',   rounds: 8, desc: '多维对比分析 · 8 轮' },
  { id: 'max',    label: '极限', en: 'Max',    rounds: 10, desc: '深度根因与极限裕度 · 10 轮' },
];

const MODEL_PRESETS = [
  { id: 'rule',      label: '内置离线',   provider: 'rule',      url: '',                             model: '' },
  { id: 'local_cli', label: '本机官方 AI', provider: 'local_cli', url: '',                             model: '' },
  { id: 'deepseek',  label: 'DeepSeek',  provider: 'openai',    url: 'https://api.deepseek.com/v1',  model: 'deepseek-chat' },
  { id: 'ollama',    label: 'Ollama',    provider: 'ollama',    url: 'http://127.0.0.1:11434/v1',    model: 'qwen2.5:7b' },
  { id: 'openai',    label: 'OpenAI',    provider: 'openai',    url: 'https://api.openai.com/v1',    model: 'gpt-4o-mini' },
  { id: 'claude',    label: 'Claude',    provider: 'anthropic', url: 'https://api.anthropic.com/v1', model: 'claude-3-5-sonnet-latest' },
];

const LOCAL_CLI_DEFAULTS = [
  { id: 'agy',    label: 'Antigravity', available: false, desc: '本机已登录 Google Antigravity 官方额度' },
  { id: 'claude', label: 'Claude Code', available: false, desc: '本机已登录 Claude 官方额度' },
  { id: 'codex',  label: 'Codex CLI',   available: false, desc: '本机已登录 OpenAI / Codex 官方额度' },
  { id: 'gemini', label: 'Gemini CLI',  available: false, desc: '本机已登录 Google Gemini 官方额度' },
];

const QUICK_PROMPTS = [
  '诊断当前参数与实时曲线',
  '超调偏大怎么压低？',
  '响应偏慢想提速',
  '切干地 0% 负压参考',
];

const PHASE_LABELS = {
  idle: ['待命', 'idle'],
  starting: ['启动中', 'busy'],
  measuring: ['4ms 采集中', 'busy'],
  analyzing: ['推导参数中', 'busy'],
  applying: ['写参核验中', 'busy'],
  rollback: ['劣化回滚中', 'warn'],
  stopping: ['正在急停', 'bad'],
  done: ['已完成', 'good'],
  stopped_by_user: ['已中止', 'warn'],
  error: ['异常中止', 'bad'],
};

const BOLT_SVG = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M13 2L4.5 13.5H12L11 22L19.5 10.5H12L13 2Z"/></svg>';
const RESET_SVG = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M3 12a9 9 0 1 0 3-6.7L3 8"/><path d="M3 3v5h5"/></svg>';
const UP_SVG = '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M12 19V5"/><path d="M6 11l6-6 6 6"/></svg>';

const PREF_AI_CFG = 'tp-ai-tuner-cfg-v2';

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
    delete next.api_key; // 密钥仅存内存，永不写入 localStorage
    localStorage.setItem(PREF_AI_CFG, JSON.stringify(next));
  } catch {}
}

function fmtParams(p) {
  if (!p || typeof p !== 'object') return '—';
  return Object.entries(p)
    .map(([k, v]) => `${k}=${Number.isFinite(Number(v)) ? Number(v).toFixed(k.includes('i') || k === 'yff' ? 3 : 2).replace(/\.?0+$/, '') : v}`)
    .join(' · ');
}

function nowHHMMSS() {
  const d = new Date();
  return `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}:${String(d.getSeconds()).padStart(2, '0')}`;
}

/**
 * Clean, zero-dependency Markdown renderer for AI tuning replies.
 * Eliminates raw **, ###, `, and list markers while highlighting telemetry params & key cues.
 */
function formatInlineMd(rawLine) {
  // 1. Escape HTML first (100% XSS-safe)
  let s = esc(rawLine);
  // 2. Preserve inline `code` spans as placeholders so later rules don't touch inside code
  const codeSlots = [];
  s = s.replace(/`([^`\n]+)`/g, (_, inner) => {
    const idx = codeSlots.length;
    codeSlots.push(`<code class="ait-md-code">${inner}</code>`);
    return `\u0000C${idx}\u0000`;
  });
  // 3. Bold (**text** or __text__) and italic (*text*)
  s = s.replace(/\*\*([^*\n]+?)\*\*/g, '<strong class="ait-md-strong">$1</strong>');
  s = s.replace(/__([^_\n]+?)__/g, '<strong class="ait-md-strong">$1</strong>');
  s = s.replace(/(^|[\s(（，、；：])\*([^*\s][^*\n]*?)\*(?=[\s)），。、；：!?！？]|$)/g, '$1<em>$2</em>');
  // 4. Strip any remaining unmatched raw markdown artifacts (stray ** or `)
  s = s.replace(/\*\*/g, '').replace(/`/g, '');
  // 5. Auto-pill explicit PID parameter assignments (e.g. ykp=16.5, yff=1.85, vkp=38)
  s = s.replace(/\b((?:v|l|r|y)(?:kp|ki|ff)|dir[pd]_cur|yaw_cap|fan_pct)\s*([=≈→])\s*(-?\d+(?:\.\d+)?)/gi,
    '<code class="ait-md-code">$1$2$3</code>');
  // 6. Subtle iMessage semantic cue accents
  s = s.replace(/(超调|过冲|振荡|发散|饱和|急停)/g, '<span class="ait-kw-warn">$1</span>');
  s = s.replace(/(稳定|收敛|最佳|推荐)/g, '<span class="ait-kw-good">$1</span>');
  // 7. Restore code slots
  s = s.replace(/\u0000C(\d+)\u0000/g, (_, idx) => codeSlots[Number(idx)] || '');
  return s;
}

function renderRichMarkdown(rawText) {
  let src = String(rawText || '').replace(/\r\n?/g, '\n');
  // Strip any <<<PARAMS>>> block or partial marker
  src = src.replace(/<<<PARAMS>>>[\s\S]*?(?:<<<END>>>|$)/g, '').replace(/<<<(?:P(?:A(?:R(?:A(?:M(?:S(?:>{0,3})?)?)?)?)?)?)?$/g, '').trim();
  if (!src) return '';

  // Extract fenced code blocks first
  const blocks = [];
  src = src.replace(/```[a-zA-Z0-9_-]*\n?([\s\S]*?)```/g, (_, code) => {
    const idx = blocks.length;
    blocks.push(`<pre class="ait-md-pre"><code>${esc(code.trim())}</code></pre>`);
    return `\n\u0000B${idx}\u0000\n`;
  });

  const lines = src.split('\n');
  const out = [];
  let inList = false;
  let para = [];

  const flushPara = () => {
    if (!para.length) return;
    out.push(`<p class="ait-md-p">${para.map(formatInlineMd).join('<br>')}</p>`);
    para = [];
  };
  const closeList = () => {
    if (inList) { out.push('</ul>'); inList = false; }
  };

  for (const rawLine of lines) {
    const line = rawLine.trim();
    if (!line) {
      flushPara();
      closeList();
      continue;
    }
    const blkM = /^\u0000B(\d+)\u0000$/.exec(line);
    if (blkM) {
      flushPara();
      closeList();
      out.push(blocks[Number(blkM[1])] || '');
      continue;
    }
    if (/^-{3,}$|^\*{3,}$/.test(line)) {
      flushPara();
      closeList();
      out.push('<hr class="ait-md-hr">');
      continue;
    }
    const hM = /^#{1,4}\s+(.+)$/.exec(line);
    if (hM) {
      flushPara();
      closeList();
      out.push(`<div class="ait-md-h">${formatInlineMd(hM[1])}</div>`);
      continue;
    }
    const qM = /^>\s?(.*)$/.exec(line);
    if (qM) {
      flushPara();
      closeList();
      out.push(`<div class="ait-md-quote">${formatInlineMd(qM[1])}</div>`);
      continue;
    }
    const ulM = /^(?:[-*•·]|\d+[.)、])\s+(.+)$/.exec(line);
    if (ulM) {
      flushPara();
      if (!inList) { out.push('<ul class="ait-md-list">'); inList = true; }
      out.push(`<li>${formatInlineMd(ulM[1])}</li>`);
      continue;
    }
    closeList();
    para.push(line);
  }
  flushPara();
  closeList();
  return out.join('');
}

function cleanLiveDelta(chunk) {
  // Keep live streaming text visually crisp (no raw ** or ### clutter flashing mid-stream)
  return String(chunk || '').replace(/\*\*/g, '').replace(/(^|\n)#{1,4}\s+/g, '$1').replace(/`/g, '');
}

export function mountAiTuner(host, { getTransport, isConnected, getLiveTelemetry, onLog, onFocusView, onStateBadge }) {
  const saved = loadSavedCfg();
  let open = false;
  let pollTimer = 0;
  let lastSnap = null;
  let lastStatus = null;
  let busyAction = false;
  let chatBusy = false;
  let runsOpen = false;
  let popoverOpen = false;
  let modelDrawerOpen = false;
  let effortIdx = Math.max(0, Math.min(EFFORT_LEVELS.length - 1, EFFORT_LEVELS.findIndex(e => e.id === (saved.effort || 'medium'))));
  if (effortIdx < 0) effortIdx = 2;
  let cliTool = ['auto', 'claude', 'codex', 'agy', 'gemini'].includes(saved.cli_tool) ? saved.cli_tool : 'auto';
  let localClis = LOCAL_CLI_DEFAULTS.map(c => ({ ...c }));
  let maxBurstTimer = 0;
  let warmupTimer = 0;
  let lastWarmupKey = '';
  let lastAnnouncedRound = 0;
  let wasAutoActive = false;

  const chatHistory = []; // [{ role, text }] sent as context
  const pendingProposals = new Map(); // proposalId -> { loop, params, dry_mode }
  let proposalSeq = 0;

  host.className = 'ait-panel';
  host.hidden = true;
  host.innerHTML = `
    <!-- 顶部状态与轮次表开关 -->
    <div class="ait-top-bar">
      <div class="ait-title-row">
        <strong class="tp-card-title">AI 调参搭档</strong>
        <span class="ait-badge" data-ait-phase data-level="idle">待命</span>
        <span class="ait-port-tag" data-ait-port-tag>未连接</span>
      </div>
      <div class="ait-head-actions">
        <button type="button" class="ait-pill-btn" data-ait-runs-toggle aria-expanded="false">轮次 <span data-ait-progress>0/6</span></button>
        <button type="button" class="ait-pill-btn" data-ait-refresh title="刷新车端参数快照">↻</button>
      </div>
    </div>

    <div class="ait-warn-bar" data-ait-transport-warn hidden>
      AI 调参需通过「本机串口桥」控制与回读，请在顶栏切回本机串口桥。
    </div>

    <!-- 顶部眼平位玻璃输入框（紧邻左侧实时波形与摄像头，边看图边指挥 AI）+ 向下展开的推理强度/模型浮层 -->
    <div class="ait-composer-wrap" data-ait-composer-wrap data-effort-level="${effortIdx}" data-thinking="false">
      <!-- Gemini 风格顶部散射光晕 + 聊天输入胶囊 -->
      <div class="ait-composer-glow" aria-hidden="true"></div>
      <div class="ait-composer">
        <textarea data-ait-chat-input rows="2" placeholder="盯着左侧波形/画面直接指挥 AI，例如：入弯超调太大 / 响应偏慢…"></textarea>
        <div class="ait-composer-bar">
          <span class="ait-ctx-pill" data-ait-ctx-pill>已挂载实时遥测</span>
          <div class="ait-composer-right">
            <button type="button" class="eff-trigger" data-eff-trigger data-level="${effortIdx}" aria-expanded="false" title="选择模型与推理强度">
              <span class="eff-trigger-bolt">${BOLT_SVG}</span>
              <span data-eff-trigger-lvl>${EFFORT_LEVELS[effortIdx].label}</span>
              <span class="eff-trigger-model" data-eff-trigger-model>内置引擎</span>
            </button>
            <button type="button" class="ait-send-btn" data-ait-send title="发送消息 (Enter)" aria-label="发送">
              ${UP_SVG}
            </button>
          </div>
        </div>
      </div>

      <!-- 悬浮毛玻璃推理强度与模型切换浮层（向下展开，完整呈现不被顶部裁切） -->
      <div class="eff-popover" data-eff-popover data-level="${effortIdx}" role="dialog" aria-label="选择推理强度与模型">
        <span class="eff-pop-prism" aria-hidden="true"><i></i></span>
        <div class="eff-pop-head">
          <span class="eff-bolt" data-eff-bolt data-level="${effortIdx}">${BOLT_SVG}</span>
          <div class="eff-head-center">
            <div class="eff-title-slot">
              <strong class="eff-title" data-eff-title>${EFFORT_LEVELS[effortIdx].label} · ${EFFORT_LEVELS[effortIdx].en}</strong>
            </div>
            <button type="button" class="eff-model-btn" data-eff-model-toggle aria-expanded="false">
              <span data-eff-model-label>内置循证引擎</span><span class="eff-chev">›</span>
            </button>
          </div>
          <button type="button" class="eff-reset-btn" data-eff-reset title="重置为默认均衡推理强度" aria-label="重置推理强度">
            ${RESET_SVG}
          </button>
        </div>

        <!-- 5 档物理弹簧磁吸胶囊滑块 -->
        <div class="eff-track" data-eff-track tabindex="0" role="slider" aria-label="推理强度" aria-valuemin="0" aria-valuemax="4" aria-valuenow="${effortIdx}">
          <div class="eff-fill" data-eff-fill></div>
          <div class="eff-dots">
            <span class="eff-dot" data-eff-dot="0"></span>
            <span class="eff-dot" data-eff-dot="1"></span>
            <span class="eff-dot" data-eff-dot="2"></span>
            <span class="eff-dot" data-eff-dot="3"></span>
            <span class="eff-dot" data-eff-dot="4"></span>
          </div>
          <span class="eff-max-wave" data-eff-max-wave aria-hidden="true"></span>
          <div class="eff-thumb" data-eff-thumb></div>
        </div>

        <div class="eff-meta-row">
          <span data-eff-desc>${EFFORT_LEVELS[effortIdx].desc}</span>
          <span data-eff-rounds-badge>默认 ${EFFORT_LEVELS[effortIdx].rounds} 轮</span>
        </div>

        <!-- 点击模型名称平滑展开的模型通道抽屉 -->
        <div class="eff-model-drawer" data-eff-model-drawer>
          <div class="eff-model-drawer-inner">
            <div class="eff-presets" data-eff-presets>
              ${MODEL_PRESETS.map(p => `<button type="button" class="eff-preset-chip" data-eff-preset="${p.id}" aria-pressed="false">${esc(p.label)}</button>`).join('')}
            </div>
            <div class="eff-form-grid">
              <label class="eff-field is-full">
                <span>通道协议</span>
                <select data-ait-provider>
                  <option value="rule">内置循证引擎（零 API Key · 离线可用）</option>
                  <option value="local_cli">本机官方 AI（直连本机已登录额度 · 免 API Key）</option>
                  <option value="openai">OpenAI 兼容（DeepSeek / 通义 / vLLM / OpenAI）</option>
                  <option value="ollama">Ollama 本地大模型</option>
                  <option value="anthropic">Anthropic Claude</option>
                </select>
              </label>
              <div class="eff-cli-box eff-field is-full" data-ait-cli-row hidden>
                <span>本机已登录官方 CLI</span>
                <div class="eff-cli-chips" data-eff-cli-chips></div>
                <div class="eff-cli-note" data-eff-cli-note>直连本机已登录官方额度，免填 API Key。</div>
              </div>
              <label class="eff-field is-full" data-ait-remote-row>
                <span>Base URL</span>
                <input type="text" data-ait-url placeholder="https://api.deepseek.com/v1" autocomplete="off" spellcheck="false">
              </label>
              <label class="eff-field" data-ait-model-row>
                <span>Model 名称</span>
                <input type="text" data-ait-model placeholder="deepseek-chat" autocomplete="off" spellcheck="false">
              </label>
              <label class="eff-field" data-ait-remote-row>
                <span>API Key（仅内存）</span>
                <input type="password" data-ait-key placeholder="留空读环境变量" autocomplete="off">
              </label>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 紧凑任务控制条 -->
    <div class="ait-ctrl-strip">
      <div class="ait-ctrl-row">
        <select data-ait-loop aria-label="整定环路">
          ${LOOP_OPTIONS.map(o => `<option value="${o.id}">${esc(o.label)}</option>`).join('')}
        </select>
        <label class="ait-unit-lbl">目标 <input type="number" data-ait-target value="300" step="10"><span data-ait-unit>°/s</span></label>
        <select data-ait-goal aria-label="偏好策略">
          <option value="balanced">均衡策略</option>
          <option value="fast_response">响应优先</option>
          <option value="low_overshoot">低超调优先</option>
          <option value="stability_first">稳健抗振</option>
        </select>
        <label class="ait-dry-chk" data-ait-dry-wrap title="两串一并角速度环专用：干地无负压时将 Kp 上限锁在 14.0">
          <input type="checkbox" data-ait-dry> 干地限幅
        </label>
      </div>
      <div class="ait-actions">
        <button type="button" class="tp-btn-accent" data-ait-start disabled>自动调参</button>
        <button type="button" data-ait-step disabled title="执行 1 趟 4 ms 阶跃基线测试，并把实时诊断结论发入对话">单步测基线</button>
        <button type="button" data-ait-rollback disabled title="恢复并回读验证当前会话最佳稳定参数">恢复最佳</button>
        <button type="button" class="ait-stop-btn" data-ait-stop disabled>急停</button>
      </div>
    </div>

    <!-- 实车物理串口安全联锁（MOCK0 自动隐藏） -->
    <div class="ait-realcar-gate" data-ait-realcar-gate hidden>
      <label class="ait-gate-chk">
        <input type="checkbox" data-ait-realcar-ok>
        <span>实车安全联锁：已架空车轮、确认 3S ≥ 11.7V 且随时准备按 Esc 急停</span>
      </label>
    </div>

    <!-- 紧凑历史最佳条（有最佳验证轮次时显示） -->
    <div class="ait-best-mini" data-ait-best-mini hidden>
      <span data-ait-best-params>—</span>
      <b data-ait-best-metrics>—</b>
    </div>

    <!-- 可展开的详细轮次表抽屉 -->
    <div class="ait-runs-drawer" data-ait-runs-drawer hidden>
      <div class="ait-status-line" data-ait-msg>连接 MOCK0 虚拟小车或实车串口后即可实时对话或自动整定。</div>
      <div class="ait-bounds-strip" data-ait-cur-params>当前车端参数：未同步</div>
      <div class="ait-table-wrap">
        <table class="ait-table">
          <thead>
            <tr>
              <th>轮</th>
              <th>参数</th>
              <th>上升</th>
              <th>超调</th>
              <th>静差</th>
              <th>状态</th>
              <th>结论</th>
            </tr>
          </thead>
          <tbody data-ait-tbody>
            <tr><td colspan="7" class="ait-empty">暂无阶跃轮次。点击「单步测基线」或「自动调参」开始采集。</td></tr>
          </tbody>
        </table>
      </div>
    </div>

    <!-- 实时交互对话流（紧跟输入框下方，新回复自动定位） -->
    <div class="ait-chat-feed" data-ait-chat-feed role="log" aria-live="polite"></div>
  `;

  const $ = q => host.querySelector(q);
  const $$ = q => [...host.querySelectorAll(q)];

  const elLoop = $('[data-ait-loop]');
  const elTarget = $('[data-ait-target]');
  const elGoal = $('[data-ait-goal]');
  const elDry = $('[data-ait-dry]');
  const elDryWrap = $('[data-ait-dry-wrap]');
  const elProvider = $('[data-ait-provider]');
  const elUrl = $('[data-ait-url]');
  const elModel = $('[data-ait-model]');
  const elKey = $('[data-ait-key]');
  const elRealGate = $('[data-ait-realcar-gate]');
  const elRealOk = $('[data-ait-realcar-ok]');
  const btnStart = $('[data-ait-start]');
  const btnStep = $('[data-ait-step]');
  const btnRollback = $('[data-ait-rollback]');
  const btnStop = $('[data-ait-stop]');
  const elFeed = $('[data-ait-chat-feed]');
  const elChatInput = $('[data-ait-chat-input]');
  const btnSend = $('[data-ait-send]');
  const elPopover = $('[data-eff-popover]');
  const elComposerWrap = $('[data-ait-composer-wrap]');
  const elMaxWave = $('[data-eff-max-wave]');
  const elCliChips = $('[data-eff-cli-chips]');
  const elCliNote = $('[data-eff-cli-note]');
  const btnTrigger = $('[data-eff-trigger]');
  const elBolt = $('[data-eff-bolt]');
  const elEffTitle = $('[data-eff-title]');
  const elModelDrawer = $('[data-eff-model-drawer]');
  const btnModelToggle = $('[data-eff-model-toggle]');
  const btnReset = $('[data-eff-reset]');
  const elTrack = $('[data-eff-track]');
  const elRunsDrawer = $('[data-ait-runs-drawer]');
  const btnRunsToggle = $('[data-ait-runs-toggle]');

  // Restore saved non-secret preferences
  if (saved.provider && ['rule', 'local_cli', 'openai', 'ollama', 'anthropic'].includes(saved.provider)) elProvider.value = saved.provider;
  if (saved.base_url) elUrl.value = saved.base_url;
  if (saved.model) elModel.value = saved.model;
  if (saved.goal && ['balanced', 'fast_response', 'low_overshoot', 'stability_first'].includes(saved.goal)) elGoal.value = saved.goal;

  function currentEffort() { return EFFORT_LEVELS[effortIdx] || EFFORT_LEVELS[2]; }

  function shortModelName() {
    const p = elProvider.value;
    const m = elModel.value.trim();
    if (p === 'rule') return '内置循证引擎';
    if (p === 'local_cli') {
      if (m) return `${m} · 本机`;
      if (cliTool !== 'auto') {
        const hit = localClis.find(c => c.id === cliTool);
        return hit ? hit.label : cliTool;
      }
      const firstAvail = localClis.find(c => c.available);
      return firstAvail ? `${firstAvail.label} · 本机` : '本机官方 AI';
    }
    if (m) return m;
    if (p === 'ollama') return 'qwen2.5:7b';
    if (p === 'anthropic') return 'Claude 3.5';
    return 'OpenAI 兼容';
  }

  function renderLocalCliChips() {
    if (!elCliChips) return;
    const availCount = localClis.filter(c => c.available).length;
    const items = [
      { id: 'auto', label: '自动探测', available: availCount > 0, desc: '自动选用首个可用的本机已登录官方 CLI' },
      ...localClis,
    ];
    elCliChips.innerHTML = items.map(c => (
      `<button type="button" class="eff-cli-chip" data-eff-cli="${esc(c.id)}" data-avail="${c.available ? 'true' : 'false'}" aria-pressed="${cliTool === c.id ? 'true' : 'false'}" title="${esc(c.desc || c.label)}">`
      + `<i class="eff-cli-dot" aria-hidden="true"></i><span>${esc(c.label)}</span></button>`
    )).join('');
    if (elCliNote) {
      const names = localClis.filter(c => c.available).map(c => c.label);
      elCliNote.textContent = names.length
        ? `已检测到本机官方额度：${names.join(' / ')}（直连免 API Key）`
        : '暂未检测到本机 CLI（安装并登录 Claude Code / Codex / agy / Gemini 后即可免 Key 直连）';
    }
  }

  async function fetchLocalClis() {
    try {
      const r = await fetch('/api/ai/local-clis', { cache: 'no-store' });
      if (!r.ok) return;
      const data = await r.json();
      if (Array.isArray(data.clis) && data.clis.length) {
        localClis = data.clis;
        renderLocalCliChips();
        syncProviderUI();
      }
    } catch {}
  }

  function triggerWarmup(force = false) {
    if (elProvider.value !== 'local_cli') return;
    const key = `${cliTool}|${elModel.value.trim()}|${currentEffort().id}`;
    if (!force && key === lastWarmupKey) return;
    clearTimeout(warmupTimer);
    warmupTimer = setTimeout(async () => {
      try {
        const r = await fetch('/api/ai/warmup', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            provider: 'local_cli',
            cli_tool: cliTool,
            model: elModel.value.trim(),
            reasoning_effort: currentEffort().id,
          }),
        });
        if (r.ok) {
          const data = await r.json();
          if (data.ok) lastWarmupKey = key;
        }
      } catch {}
    }, 180);
  }

  function syncProviderUI() {
    const prov = elProvider.value;
    const isRule = prov === 'rule';
    const isLocalCli = prov === 'local_cli';
    $$('[data-ait-cli-row]').forEach(row => { row.hidden = !isLocalCli; });
    $$('[data-ait-remote-row]').forEach(row => { row.hidden = isRule || isLocalCli; });
    $$('[data-ait-model-row]').forEach(row => {
      row.hidden = isRule;
      row.classList.toggle('is-full', isLocalCli);
    });
    if (isLocalCli) {
      elModel.placeholder = '留空默认用该 CLI 官方模型';
    } else if (prov === 'ollama') {
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
    const label = shortModelName();
    $('[data-eff-model-label]').textContent = label;
    $('[data-eff-trigger-model]').textContent = isRule ? '内置引擎' : label;

    const urlLow = elUrl.value.trim().toLowerCase();
    $$('[data-eff-preset]').forEach(btn => {
      const pid = btn.dataset.effPreset;
      const match =
        (pid === 'rule' && isRule) ||
        (pid === 'local_cli' && isLocalCli) ||
        (pid === 'ollama' && prov === 'ollama') ||
        (pid === 'claude' && prov === 'anthropic') ||
        (pid === 'deepseek' && prov === 'openai' && urlLow.includes('deepseek')) ||
        (pid === 'openai' && prov === 'openai' && !urlLow.includes('deepseek'));
      btn.setAttribute('aria-pressed', String(match));
    });

    saveCfg({ provider: prov, cli_tool: cliTool, base_url: elUrl.value.trim(), model: elModel.value.trim(), effort: currentEffort().id });
    if (open && isLocalCli) triggerWarmup();
  }

  function triggerMaxBurst() {
    if (document.documentElement.dataset.motion === 'off') return;
    clearTimeout(maxBurstTimer);
    elPopover.classList.remove('is-max-burst');
    elBolt.classList.remove('is-burst');
    elMaxWave?.classList.remove('is-wave');
    elPopover.offsetWidth; // restart crisp studio prismatic overdrive burst
    elPopover.classList.add('is-max-burst');
    elBolt.classList.add('is-burst');
    elMaxWave?.classList.add('is-wave');
    maxBurstTimer = setTimeout(() => {
      elPopover.classList.remove('is-max-burst');
      elBolt.classList.remove('is-burst');
      elMaxWave?.classList.remove('is-wave');
    }, 740);
  }

  function updateEffortChrome(nextIdx, prevIdx = nextIdx) {
    effortIdx = nextIdx;
    const eff = currentEffort();
    elPopover.dataset.level = String(effortIdx);
    elComposerWrap.dataset.effortLevel = String(effortIdx);
    btnTrigger.dataset.level = String(effortIdx);
    elBolt.dataset.level = String(effortIdx);
    elTrack.setAttribute('aria-valuenow', String(effortIdx));
    $('[data-eff-trigger-lvl]').textContent = eff.label;
    $('[data-eff-desc]').textContent = eff.desc;
    $('[data-eff-rounds-badge]').textContent = `默认 ${eff.rounds} 轮`;

    // Directional vertical rolling animation on level title
    const nextText = `${eff.label} · ${eff.en}`;
    if (elEffTitle.textContent !== nextText) {
      elEffTitle.textContent = nextText;
      elEffTitle.classList.remove('roll-up', 'roll-down');
      elEffTitle.offsetWidth; // trigger single reflow for crisp roll restart
      elEffTitle.classList.add(nextIdx >= prevIdx ? 'roll-up' : 'roll-down');
    }

    if (!lastStatus?.active) {
      $('[data-ait-progress]').textContent = `${lastStatus?.round || 0}/${eff.rounds}`;
    }
    saveCfg({ effort: eff.id });
    if (open && elProvider.value === 'local_cli') triggerWarmup();
  }

  // Mount the 2nd-order high-stiffness spring capsule slider (+ Max overdrive burst)
  const effortSlider = mountEffortSlider(elTrack, {
    levels: EFFORT_LEVELS.length,
    initialIndex: effortIdx,
    onChange(nextIdx) {
      const prev = effortIdx;
      updateEffortChrome(nextIdx, prev);
    },
    onMaxHit() {
      triggerMaxBurst();
    },
  });

  function setPopoverOpen(yes) {
    popoverOpen = Boolean(yes);
    elPopover.classList.toggle('is-open', popoverOpen);
    btnTrigger.setAttribute('aria-expanded', String(popoverOpen));
    if (popoverOpen) requestAnimationFrame(() => effortSlider.render());
  }

  function setRunsOpen(yes) {
    runsOpen = Boolean(yes);
    elRunsDrawer.hidden = !runsOpen;
    btnRunsToggle.setAttribute('aria-expanded', String(runsOpen));
  }

  function syncLoopUI(updateTarget = true) {
    const opt = LOOP_OPTIONS.find(o => o.id === elLoop.value) || LOOP_OPTIONS[0];
    $('[data-ait-unit]').textContent = opt.unit;
    if (updateTarget) elTarget.value = String(opt.target);
    elDryWrap.hidden = opt.id !== 'yaw_2s1p';
  }

  function scrollChatToBottom() {
    requestAnimationFrame(() => { elFeed.scrollTop = elFeed.scrollHeight; });
  }

  function buildSuggestHTML(suggested, notes, loop) {
    if (!suggested || typeof suggested !== 'object' || !Object.keys(suggested).length) return '';
    const pid = `prop_${++proposalSeq}`;
    pendingProposals.set(pid, { loop: loop || elLoop.value, params: suggested, dry_mode: elDry.checked });
    const noteText = notes && notes.length ? notes.join('；') : '已通过安全步长与幅值护栏核验';
    return `
      <div class="ait-suggest-card">
        <div class="ait-suggest-params">${esc(fmtParams(suggested))}</div>
        <div class="ait-suggest-foot">
          <span class="ait-suggest-note">${esc(noteText)}</span>
          <button type="button" class="ait-apply-btn" data-ait-apply-proposal="${pid}">安全写入并核验</button>
        </div>
      </div>`;
  }

  function appendMessage({ role = 'assistant', kind = 'chat', text = '', insights = [], suggested = null, notes = [], loop = null, meta = '' }) {
    const wrap = document.createElement('article');
    wrap.className = 'ait-msg';
    wrap.dataset.role = role;
    wrap.dataset.kind = kind;

    let headHTML = '';
    if (role === 'assistant') {
      headHTML = `<div class="ait-msg-head"><span class="ait-msg-effort">⚡ ${esc(meta || `${currentEffort().label} · ${shortModelName()}`)}</span><span>${nowHHMMSS()}</span></div>`;
    }

    const insHTML = insights && insights.length
      ? `<ul class="ait-insights">${insights.map(line => `<li>${formatInlineMd(line)}</li>`).join('')}</ul>`
      : '';

    const suggestHTML = buildSuggestHTML(suggested, notes, loop);
    const bodyHTML = role === 'assistant' ? `<div class="ait-md-body">${renderRichMarkdown(text)}</div>` : esc(text);

    wrap.innerHTML = `${headHTML}<div class="ait-msg-bubble">${bodyHTML}${insHTML}${suggestHTML}</div>`;
    elFeed.append(wrap);
    scrollChatToBottom();
    return wrap;
  }

  function appendWelcome() {
    const msg = appendMessage({
      role: 'assistant',
      kind: 'welcome',
      meta: `${currentEffort().label} · 实时调参搭档`,
      text: '已接入当前实时波形与车端参数快照。你可以直接告诉我体感问题，或点击「单步测基线 / 自动调参」——每跑完一趟我会立刻给出工程见解与安全步长内的改参卡片。',
    });
    const chips = document.createElement('div');
    chips.className = 'ait-quick-chips';
    chips.innerHTML = QUICK_PROMPTS.map(p => `<button type="button" class="ait-quick-chip" data-ait-prompt="${esc(p)}">${esc(p)}</button>`).join('');
    msg.append(chips);
  }

  async function applyProposal(pid, btnEl) {
    const item = pendingProposals.get(pid);
    if (!item || !btnEl) return;
    const usingBridge = getTransport() === 'bridge';
    if (!usingBridge || !isConnected()) {
      appendMessage({ role: 'assistant', kind: 'live-event', text: '请先在顶栏通过本机串口桥连接 MOCK0 或实车串口后再写入参数。' });
      return;
    }
    const isMock = Boolean(lastStatus?.is_mock ?? lastSnap?.is_mock);
    if (!isMock && !elRealOk.checked) {
      appendMessage({ role: 'assistant', kind: 'live-event', text: '实车安全联锁未勾选：请先确认架空车轮并勾选上方实车安全联锁，再一键写入参数。' });
      return;
    }
    btnEl.disabled = true;
    const oldLabel = btnEl.textContent;
    btnEl.textContent = '写入核验中…';
    try {
      const r = await fetch('/api/ai/apply', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          loop: item.loop,
          parameters: item.params,
          dry_mode: item.dry_mode,
          allow_real_car: elRealOk.checked,
          confirm_wheels_off_ground: elRealOk.checked,
        }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      btnEl.dataset.applied = 'true';
      btnEl.textContent = `已写入 rev ${data.rev} ✓`;
      onLog?.('event', `AI 建议参数已写入并回读确认：${fmtParams(data.applied)} (rev ${data.rev})`);
      fetchSnapshot();
    } catch (e) {
      btnEl.disabled = false;
      btnEl.textContent = oldLabel;
      appendMessage({ role: 'assistant', kind: 'live-event', text: `参数写入被拦截或失败：${e.message}` });
      onLog?.('error', `AI 写入失败：${e.message}`);
    }
  }

  async function sendChatMessage(rawText) {
    const text = (rawText ?? elChatInput.value).trim();
    if (!text || chatBusy) return;
    setPopoverOpen(false);
    elChatInput.value = '';
    elChatInput.style.height = 'auto';

    appendMessage({ role: 'user', text });
    chatHistory.push({ role: 'user', text });
    if (chatHistory.length > 14) chatHistory.splice(0, chatHistory.length - 14);

    chatBusy = true;
    elComposerWrap.dataset.thinking = 'true';
    btnSend.disabled = true;

    // Create live iMessage-style assistant bubble with liquid 3-dot thinking pill -> kinetic token stream
    const streamWrap = document.createElement('article');
    streamWrap.className = 'ait-msg is-streaming';
    streamWrap.dataset.role = 'assistant';
    streamWrap.dataset.kind = 'chat';
    const busyHint = elProvider.value === 'local_cli'
      ? `${shortModelName()} 思考中`
      : `${currentEffort().label}分析中`;
    streamWrap.innerHTML = `
      <div class="ait-msg-head">
        <span class="ait-msg-effort" data-stream-meta>⚡ ${esc(`${currentEffort().label} · ${shortModelName()}`)}</span>
        <span data-stream-time>${nowHHMMSS()}</span>
      </div>
      <div class="ait-msg-bubble" data-stream-bubble>
        <div class="ait-imsg-thinking" data-stream-think>
          <span class="ait-imsg-dots"><i></i><i></i><i></i></span>
          <span>${esc(busyHint)}</span>
        </div>
        <div class="ait-stream-live" data-stream-live hidden><span class="ait-stream-caret" aria-hidden="true"></span></div>
      </div>`;
    elFeed.append(streamWrap);
    scrollChatToBottom();

    const bubbleEl = streamWrap.querySelector('[data-stream-bubble]');
    const thinkEl = streamWrap.querySelector('[data-stream-think]');
    const liveEl = streamWrap.querySelector('[data-stream-live]');
    const caretEl = liveEl.querySelector('.ait-stream-caret');
    const metaEl = streamWrap.querySelector('[data-stream-meta]');
    let sawFirstDelta = false;
    let rawAccum = '';
    let finalData = null;

    const pushDelta = deltaText => {
      if (!deltaText) return;
      rawAccum += deltaText;
      const clean = cleanLiveDelta(deltaText);
      if (!clean) return;
      if (!sawFirstDelta) {
        sawFirstDelta = true;
        thinkEl.hidden = true;
        liveEl.hidden = false;
      }
      const span = document.createElement('span');
      span.className = 'ait-tok-in';
      span.textContent = clean;
      liveEl.insertBefore(span, caretEl);
      elFeed.scrollTop = elFeed.scrollHeight;
    };

    try {
      const live = typeof getLiveTelemetry === 'function' ? (getLiveTelemetry() || {}) : {};
      const cleanLive = {};
      for (const [k, v] of Object.entries(live)) {
        if (Number.isFinite(v)) cleanLive[k] = Number(v.toFixed(2));
      }
      const r = await fetch('/api/ai/chat-stream', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: text,
          history: chatHistory.slice(-8),
          loop: elLoop.value,
          dry_mode: elDry.checked,
          provider: elProvider.value,
          cli_tool: cliTool,
          base_url: elUrl.value.trim(),
          model: elModel.value.trim(),
          api_key: elKey.value.trim(),
          reasoning_effort: currentEffort().id,
          live_telemetry: cleanLive,
        }),
      });
      if (!r.ok || !r.body) {
        const errObj = await r.json().catch(() => ({}));
        throw new Error(errObj.error || `HTTP ${r.status}`);
      }

      const reader = r.body.getReader();
      const dec = new TextDecoder('utf-8');
      let sseBuf = '';
      while (true) {
        const { value, done } = await reader.read();
        if (done) break;
        sseBuf += dec.decode(value, { stream: true });
        let sep;
        while ((sep = sseBuf.indexOf('\n\n')) >= 0) {
          const frame = sseBuf.slice(0, sep);
          sseBuf = sseBuf.slice(sep + 2);
          let evName = 'message';
          let dataLine = '';
          for (const ln of frame.split('\n')) {
            if (ln.startsWith('event:')) evName = ln.slice(6).trim();
            else if (ln.startsWith('data:')) dataLine += ln.slice(5).trim();
          }
          if (!dataLine) continue;
          let payload;
          try { payload = JSON.parse(dataLine); } catch { continue; }
          if (evName === 'delta' && payload.t) pushDelta(payload.t);
          else if (evName === 'done') finalData = payload;
          else if (evName === 'error') throw new Error(payload.error || '对话流中断');
        }
      }

      const data = finalData || { reply: rawAccum || '已完成分析。', model: shortModelName() };
      const secTag = Number.isFinite(data.elapsed_ms) ? ` · ${(data.elapsed_ms / 1000).toFixed(1)}s` : '';
      metaEl.textContent = `⚡ ${currentEffort().label} · ${data.model || shortModelName()}${secTag}`;
      const insHTML = data.insights && data.insights.length
        ? `<ul class="ait-insights">${data.insights.map(line => `<li>${formatInlineMd(line)}</li>`).join('')}</ul>`
        : '';
      const suggestHTML = buildSuggestHTML(data.suggested_params, data.guardrail_notes, data.loop);
      streamWrap.classList.remove('is-streaming');
      bubbleEl.innerHTML = `<div class="ait-md-body ait-md-settle">${renderRichMarkdown(data.reply || rawAccum || '已完成分析。')}</div>${insHTML}${suggestHTML}`;
      chatHistory.push({ role: 'assistant', text: data.reply || rawAccum || '' });
      scrollChatToBottom();
    } catch (e) {
      streamWrap.classList.remove('is-streaming');
      streamWrap.dataset.kind = 'live-event';
      bubbleEl.innerHTML = `<div class="ait-md-body">${renderRichMarkdown(`对话请求失败：${e.message}`)}</div>`;
    } finally {
      chatBusy = false;
      elComposerWrap.dataset.thinking = 'false';
      btnSend.disabled = false;
      elChatInput.focus();
    }
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
      $('[data-ait-ctx-pill]').textContent = '离线模式 · 可直接咨询';
    } else if (isMock) {
      portTag.textContent = `${port} 沙箱`;
      portTag.dataset.level = 'good';
      $('[data-ait-ctx-pill]').textContent = `实时遥测已挂载 · ${port}`;
    } else {
      portTag.textContent = `${port} 实车`;
      portTag.dataset.level = 'warn';
      $('[data-ait-ctx-pill]').textContent = `实车遥测已挂载 · ${port}`;
    }

    elRealGate.hidden = !conn || isMock;
    const safeToRun = conn && (isMock || elRealOk.checked);

    btnStart.disabled = !safeToRun || active || busyAction;
    btnStep.disabled = !safeToRun || active || busyAction;
    btnRollback.disabled = !safeToRun || active || busyAction || !lastStatus?.best;
    btnStop.disabled = !conn || (!active && !busyAction);

    elLoop.disabled = active;
    elTarget.disabled = active;
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

    const maxR = (st.active || (st.round || 0) > 0) ? (st.max_rounds || currentEffort().rounds) : currentEffort().rounds;
    $('[data-ait-progress]').textContent = `${st.round || 0}/${maxR}`;
    $('[data-ait-msg]').textContent = st.message || '就绪';

    const bestMini = $('[data-ait-best-mini]');
    if (st.best) {
      const m = st.best.metrics || {};
      bestMini.hidden = false;
      $('[data-ait-best-params]').textContent = `★ #${st.best.round} ${fmtParams(st.best.parameters)}`;
      $('[data-ait-best-metrics]').textContent = `升 ${m.rise_ms ?? '—'}ms · 冲 ${m.overshoot_pct ?? '—'}% · 分 ${st.best.score}`;
    } else {
      bestMini.hidden = true;
    }

    const rows = Array.isArray(st.history) ? st.history : [];
    const tbody = $('[data-ait-tbody]');
    if (!rows.length) {
      tbody.innerHTML = '<tr><td colspan="7" class="ait-empty">暂无阶跃轮次。点击「单步测基线」或「自动调参」开始采集。</td></tr>';
    } else {
      tbody.innerHTML = rows.map(r => {
        const m = r.metrics || {};
        const stLevel = m.status === 'STABLE' ? 'good' : (m.status === 'OSCILLATING' || m.status === 'SATURATED') ? 'bad' : 'warn';
        const actLevel = r.action === 'ROLLBACK' ? 'bad' : (r.action === 'DONE' || r.action === 'HOLD_VERIFY') ? 'good' : 'idle';
        const notes = (r.guardrail_notes || []).length
          ? `<div class="ait-notes">${esc(r.guardrail_notes.join('；'))}</div>` : '';
        return `<tr class="${r.is_best ? 'is-best' : ''}">
          <td><b>#${r.round}</b>${r.is_best ? '<span class="ait-star">★</span>' : ''}</td>
          <td class="ait-mono">${esc(fmtParams(r.parameters))}</td>
          <td class="ait-mono">${m.rise_ms ?? '—'}ms</td>
          <td class="ait-mono">${m.overshoot_pct ?? '—'}%</td>
          <td class="ait-mono">${m.steady_err_pct ?? '—'}%</td>
          <td><span class="ait-tag" data-level="${stLevel}">${esc(m.status || '—')}</span></td>
          <td><span class="ait-tag" data-level="${actLevel}">${esc(r.action || '—')}</span><span class="ait-reason">${esc(r.summary || '')}</span>${notes}</td>
        </tr>`;
      }).join('');
    }

    // Proactively stream new auto-tune round insights right into the conversation feed!
    if (rows.length > lastAnnouncedRound) {
      for (let i = lastAnnouncedRound; i < rows.length; i++) {
        const r = rows[i];
        const m = r.metrics || {};
        const roundInsights = [
          `参数：${fmtParams(r.parameters)}`,
          `上升 ${m.rise_ms ?? '—'} ms · 超调 ${m.overshoot_pct ?? '—'}% · 静差 ${m.steady_err_pct ?? '—'}% · 饱和 ${m.pwm_sat_pct ?? '—'}% · 评分 ${r.score}`,
        ];
        if (r.acceptance_detail) roundInsights.push(`验收判定：${r.acceptance_detail}`);
        appendMessage({
          role: 'assistant',
          kind: 'live-event',
          meta: `自动调参 · 第 ${r.round} 轮`,
          text: r.summary || `第 ${r.round} 轮 4 ms 阶跃采集完成（${m.status || '—'}）。`,
          insights: roundInsights,
        });
      }
      lastAnnouncedRound = rows.length;
    }

    if (wasAutoActive && !st.active && st.phase === 'done') {
      appendMessage({
        role: 'assistant',
        kind: 'live-event',
        meta: '自动调参收尾',
        text: st.message || '自动调参已完成。',
        insights: st.best ? [`已保留最佳稳定参数（第 ${st.best.round} 轮）：${fmtParams(st.best.parameters)}`] : [],
      });
    }
    wasAutoActive = Boolean(st.active);
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
      if (Array.isArray(snap.local_clis) && snap.local_clis.length) {
        localClis = snap.local_clis;
        renderLocalCliChips();
        syncProviderUI();
      }
      const curStr = snap.current_parameters ? fmtParams(snap.current_parameters) : '未同步';
      $('[data-ait-cur-params]').textContent = `当前车端参数：${curStr}`;
      if (snap.tuner) {
        if (!snap.tuner.active && (snap.tuner.history || []).length === 0) lastAnnouncedRound = 0;
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
    const eff = currentEffort();
    onFocusView?.(opt.view);
    setPopoverOpen(false);
    saveCfg({ provider: elProvider.value, cli_tool: cliTool, base_url: elUrl.value.trim(), model: elModel.value.trim(), goal: elGoal.value, effort: eff.id });
    busyAction = true;
    lastAnnouncedRound = 0;
    wasAutoActive = true;
    updateControls();
    const userNote = elChatInput.value.trim().slice(0, 300);
    if (userNote) {
      elChatInput.value = '';
      elChatInput.style.height = 'auto';
    }
    try {
      const r = await fetch('/api/ai/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          loop: elLoop.value,
          provider: elProvider.value,
          cli_tool: cliTool,
          base_url: elUrl.value.trim(),
          model: elModel.value.trim(),
          api_key: elKey.value.trim(),
          reasoning_effort: eff.id,
          target: Number(elTarget.value) || opt.target,
          low: opt.low,
          max_rounds: eff.rounds,
          goal: elGoal.value,
          dry_mode: elDry.checked,
          user_note: userNote,
          allow_real_car: elRealOk.checked,
          confirm_wheels_off_ground: elRealOk.checked,
        }),
      });
      const data = await r.json();
      if (!r.ok) throw new Error(data.error || `HTTP ${r.status}`);
      appendMessage({
        role: 'assistant',
        kind: 'live-event',
        meta: `${eff.label} · 自动整定启动`,
        text: `开始整定「${opt.label}」（目标 ${Number(elTarget.value) || opt.target} ${opt.unit} · 最多 ${eff.rounds} 轮 · ${shortModelName()}${userNote ? ` · 约束：${userNote}` : ''}），每轮 4 ms 阶跃完成后将实时同步波形特征与结论。`,
      });
      renderStatus(data);
      ensurePolling();
      onLog?.('event', `启动 AI 自动调参：${opt.label}（${elProvider.value} · ${eff.id}）`);
    } catch (e) {
      wasAutoActive = false;
      onLog?.('error', `AI 自动调参启动失败：${e.message}`);
      appendMessage({ role: 'assistant', kind: 'live-event', text: `自动调参启动失败：${e.message}` });
    } finally {
      busyAction = false;
      updateControls();
    }
  }

  async function runOneStep() {
    const opt = LOOP_OPTIONS.find(o => o.id === elLoop.value) || LOOP_OPTIONS[0];
    onFocusView?.(opt.view);
    busyAction = true;
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
      const advice = m.status === 'OSCILLATING'
        ? '出现振荡，建议小步回调 Kp 并压低 Ki。'
        : m.overshoot_pct > 10
          ? `超调 ${m.overshoot_pct}% 偏高，建议适度降低 Kp 或削弱积分。`
          : m.status === 'SLOW_RESPONSE'
            ? `上升 ${m.rise_ms} ms 偏慢，在 PWM 未触顶前可稳步提高 Kp。`
            : '阶跃响应进入稳定窗口，可继续小步细调或直接保留。';
      appendMessage({
        role: 'assistant',
        kind: 'live-event',
        meta: `单步基线 · ${m.status}`,
        text: `4 ms 板载阶跃采集完成（评分 ${data.score}）：${advice}`,
        insights: [
          `参数快照：${fmtParams(data.parameters)}`,
          `上升 ${m.rise_ms} ms · 超调 ${m.overshoot_pct}% · 静差 ${m.steady_err_pct}% · PWM 饱和 ${m.pwm_sat_pct}%`,
        ],
      });
      onLog?.('event', `单步基线完成：上升 ${m.rise_ms} ms · 超调 ${m.overshoot_pct}% · 状态 ${m.status}`);
      await fetchSnapshot();
    } catch (e) {
      appendMessage({ role: 'assistant', kind: 'live-event', text: `单步测量失败：${e.message}` });
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
      appendMessage({
        role: 'assistant',
        kind: 'live-event',
        meta: '回滚核验',
        text: `已恢复并回读确认第 ${data.restored_round} 轮最佳参数：${fmtParams(data.parameters)}。`,
      });
      onLog?.('event', `已回滚至第 ${data.restored_round} 轮最佳参数：${fmtParams(data.parameters)}`);
      await fetchSnapshot();
    } catch (e) {
      appendMessage({ role: 'assistant', kind: 'live-event', text: `回滚失败：${e.message}` });
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

  // ---------- Event Wiring ----------
  btnTrigger.addEventListener('click', e => {
    e.stopPropagation();
    setPopoverOpen(!popoverOpen);
  });
  btnModelToggle.addEventListener('click', () => {
    modelDrawerOpen = !modelDrawerOpen;
    elModelDrawer.classList.toggle('is-open', modelDrawerOpen);
    btnModelToggle.setAttribute('aria-expanded', String(modelDrawerOpen));
  });
  btnReset.addEventListener('click', () => {
    btnReset.classList.remove('is-spinning');
    btnReset.offsetWidth;
    btnReset.classList.add('is-spinning');
    effortSlider.setIndex(2, { notify: true, kick: true });
  });
  $('[data-eff-presets]').addEventListener('click', e => {
    const btn = e.target.closest('[data-eff-preset]');
    if (!btn) return;
    const preset = MODEL_PRESETS.find(p => p.id === btn.dataset.effPreset);
    if (!preset) return;
    elProvider.value = preset.provider;
    elUrl.value = preset.url;
    elModel.value = preset.model;
    syncProviderUI();
  });
  elCliChips.addEventListener('click', e => {
    const btn = e.target.closest('[data-eff-cli]');
    if (!btn) return;
    cliTool = btn.dataset.effCli || 'auto';
    renderLocalCliChips();
    syncProviderUI();
  });

  // Close popover when clicking outside or pressing Escape
  document.addEventListener('pointerdown', e => {
    if (!popoverOpen) return;
    if (!host.querySelector('[data-ait-composer-wrap]').contains(e.target)) setPopoverOpen(false);
  });
  document.addEventListener('keydown', e => {
    if (e.key === 'Escape' && popoverOpen) setPopoverOpen(false);
  });

  // Chat feed delegations (quick prompt chips + one-click apply proposal cards)
  elFeed.addEventListener('click', e => {
    const chip = e.target.closest('[data-ait-prompt]');
    if (chip) { sendChatMessage(chip.dataset.aitPrompt); return; }
    const applyBtn = e.target.closest('[data-ait-apply-proposal]');
    if (applyBtn) { applyProposal(applyBtn.dataset.aitApplyProposal, applyBtn); }
  });

  elChatInput.addEventListener('input', () => {
    elChatInput.style.height = 'auto';
    elChatInput.style.height = `${Math.min(92, elChatInput.scrollHeight)}px`;
  });
  elChatInput.addEventListener('keydown', e => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendChatMessage();
    }
  });
  btnSend.addEventListener('click', () => sendChatMessage());

  elProvider.addEventListener('change', syncProviderUI);
  elUrl.addEventListener('change', syncProviderUI);
  elModel.addEventListener('change', syncProviderUI);
  elGoal.addEventListener('change', () => saveCfg({ goal: elGoal.value }));
  elLoop.addEventListener('change', () => { syncLoopUI(true); fetchSnapshot(); });
  elDry.addEventListener('change', fetchSnapshot);
  elRealOk.addEventListener('change', updateControls);
  btnRunsToggle.addEventListener('click', () => setRunsOpen(!runsOpen));
  $('[data-ait-refresh]').addEventListener('click', fetchSnapshot);
  btnStart.addEventListener('click', startAutoTune);
  btnStep.addEventListener('click', runOneStep);
  btnRollback.addEventListener('click', doRollback);
  btnStop.addEventListener('click', () => stopAll(true));

  function setOpen(yes) {
    open = Boolean(yes);
    host.hidden = !open;
    if (!open) setPopoverOpen(false);
    onStateBadge?.({ active: Boolean(lastStatus?.active), phase: lastStatus?.phase || 'idle', open });
    if (open) {
      fetchSnapshot();
      if (elProvider.value === 'local_cli') triggerWarmup();
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

  renderLocalCliChips();
  syncProviderUI();
  syncLoopUI(true);
  updateEffortChrome(effortIdx, effortIdx);
  appendWelcome();
  updateControls();
  fetchLocalClis();

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
