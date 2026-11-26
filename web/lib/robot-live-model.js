// SPDX-License-Identifier: GPL-3.0-or-later
// TC264 三环远程调参台协议解析器：把串口字节流拆分为遥测、图传、调参回读与状态事件。
// 完整串口通信协议详见 docs/PROTOCOL.md：
//   状态行        "t=… vbat_mv=… cam=… … gray=… encL=… encR=…\r\n"（停车低频状态）
//   命令回复      "exp=1200 ok\r\n" "gain=70 out of range 16..64\r\n" 等，原样显示
//   单帧图像      "\r\nIMG gray|bin 188 120 <seq>\r\n" + 188×120 字节 + "\r\nEND\r\n"（回应 "#I" / "#B"）
//   旧版调参包    A5 | 停车原因 1B | 速度 int16 | 偏差 float | 校验和(前 7 字节之和) | 5A
//   二进制遥测    A5 5A | ver | len | 负载（小端紧排）| CRC16-CCITT-FALSE（ver 到负载末尾，小端）
//                 与旧调参包同为 A5 开头，通过第二字节 5A 区分
//   实时压缩图传  A5 5B | fmt | flags | sliceIdx | sliceTotal | slen | totalLen | 元数据 | 切片负载 | CRC16
//   身份行        "ID fw=<构建标签> hex=... tlm=2 bb=500 params=none"
// 只解析，不计算控制；单帧 IMG 无 CRC，通过结束标记是否紧跟在 188×120 字节后判断丢字节。

export const PACKET_HEAD = 0xa5, PACKET_TAIL = 0x5a, PACKET_LEN = 10;
const TLM_V1 = [['t', 'I'], ['seq', 'I'], ['vis_age_ms', 'H'], ['proc_us', 'H'], ['line_valid', 'B'], ['image_error', 'h'], ['ra_state', 'B'],
  ['imu_angle_flag', 'B'], ['route_k', 'b'], ['route_cur', 'b'], ['launch_flag', 'B'], ['stop_reason', 'B'], ['set_speed', 'h'], ['speed_target', 'h'],
  ['setL', 'h'], ['setR', 'h'], ['encL', 'h'], ['encR', 'h'], ['enc_total_L', 'i'], ['enc_total_R', 'i'], ['pwmL', 'h'], ['pwmR', 'h'],
  ['gyro_dps', 'h', 10], ['yaw_target_dps', 'h', 10], ['angle_out', 'h', 10], ['imu_sum', 'f'], ['target_imu', 'f'], ['vbat_mv', 'H'], ['fan_duty', 'H'],
  ['mode', 'B'], ['ra_seen', 'B'], ['ra_row', 'B'], ['route_m', 'i']];
const TLM_V2 = [...TLM_V1.map(([n, c, s]) => [n, n === 'yaw_target_dps' || n === 'angle_out' ? 'i' : c, s]),
  ['processed_seq', 'I'], ['sample_seq', 'I'], ['record_seq', 'I'], ['flags', 'H']];
const BB_HEADER = [['records', 'H'], ['capacity', 'H'], ['overwritten', 'I'], ['first_record_seq', 'I'], ['last_record_seq', 'I'], ['export_ms', 'I'], ['tx_dropped', 'I'], ['flags', 'H']];
const BB_END = [['records', 'H'], ['export_ms', 'I']];
const SIZE = { I: 4, i: 4, H: 2, h: 2, B: 1, b: 1, f: 4 };
const bytesOf = cols => cols.reduce((n, [, c]) => n + SIZE[c], 0);
// "版本:负载字节数" → 列表；v1 有 63 B（r1）和 70 B（r2）两种
export const TLM_LAYOUTS = new Map([[1, TLM_V1.slice(0, -4)], [1, TLM_V1], [2, TLM_V2], [0x82, BB_HEADER], [0x83, BB_END]].map(([v, cols]) => [`${v}:${bytesOf(cols)}`, cols]));
export const TLM_FLAGS = { sample_present: 1, sample_fresh: 2, route_available: 4, numeric_saturated: 8, vbat_valid: 16 };

// CRC-16/CCITT-FALSE 查表加速（多项式 0x1021、初值 0xFFFF；"123456789" 的校验值是 0x29B1）
const CRC16_TABLE = (() => {
  const t = new Uint16Array(256);
  for (let i = 0; i < 256; i++) {
    let c = i << 8;
    for (let k = 0; k < 8; k++) c = (c & 0x8000) ? (((c << 1) ^ 0x1021) & 0xffff) : ((c << 1) & 0xffff);
    t[i] = c;
  }
  return t;
})();
const LATIN1_DECODER = new TextDecoder('latin1');

export function crc16(bytes, crc = 0xffff) {
  for (let i = 0; i < bytes.length; i++) {
    crc = ((crc << 8) & 0xffff) ^ CRC16_TABLE[((crc >> 8) ^ bytes[i]) & 0xff];
  }
  return crc;
}
function readRow(view, at, cols) {
  const row = {};
  for (const [name, code, scale] of cols) {
    const v = code === 'I' ? view.getUint32(at, true) : code === 'i' ? view.getInt32(at, true) : code === 'H' ? view.getUint16(at, true)
      : code === 'h' ? view.getInt16(at, true) : code === 'B' ? view.getUint8(at) : code === 'b' ? view.getInt8(at) : view.getFloat32(at, true);
    row[name] = scale ? v / scale : v;
    at += SIZE[code];
  }
  return row;
}
export const MAX_FRAME_BYTES = 188 * 120;
const END = [0x0d, 0x0a, 0x45, 0x4e, 0x44, 0x0d, 0x0a];      // "\r\nEND\r\n"
const MAX_LINE = 1024;                                       // 一行状态最长约 200 字节；超过就当乱码丢掉

export const STOP_REASONS = { 0: '正常', 1: '角速度保护', 2: '速度保护', 3: '丢线超过 100 ms', 4: '直角 1 s 没找回线', 20: '定时测试结束', 21: '左轮方向反转保护', 22: '右轮方向反转保护', 23: '左轮堵转判定', 24: '右轮堵转判定', 25: '远程急停', 26: '完成全程停车', 27: '120 s 超时停车', 28: '低速未稳定，未升档', 29: '电池低于 11.70 V', 30: '遥控断联急停（看门狗）', 31: '电池欠压拒绝发车', 32: '电压读数已失效' };

const asciiBytes = s => Uint8Array.from(s, c => c.charCodeAt(0) & 0xff);
export function signCommand(body) {
  const crc = crc16(asciiBytes(body)).toString(16).toUpperCase().padStart(4, '0');
  return `${body}*${crc}\r\n`;
}
function verifyStarCrc(text) {
  const star = text.lastIndexOf('*');
  if (star < 1 || text.length - star !== 5) return null;
  const hex = text.slice(star + 1);
  if (!/^[0-9A-Fa-f]{4}$/.test(hex)) return null;
  const body = text.slice(0, star);
  if (crc16(asciiBytes(body)) !== parseInt(hex, 16)) return null;
  return body;
}
const s8 = v => (v >= 128 ? v - 256 : v);
const s16 = v => (v >= 32768 ? v - 65536 : v);
const s32 = v => (v >= 0x80000000 ? v - 0x100000000 : v);

export function parseM2Line(text) {
  if (typeof text !== 'string' || !text.startsWith('@M2,')) return null;
  const body = verifyStarCrc(text);
  if (!body) return null;
  const parts = body.slice(4).split(',');
  if (parts.length !== 24 || parts.some(p => !/^[0-9A-Fa-f]+$/.test(p))) return null;
  const v = parts.map(p => parseInt(p, 16));
  const flags = v[6];
  return {
    run_id: v[0], seq: v[1], t: v[2],
    route_k: v[3], route_cur: s8(v[4]), ra_state: v[5], flags,
    launch_flag: flags & 1, phase: (flags >> 1) & 1, pwm_on: (flags >> 2) & 1,
    line_valid: (flags >> 3) & 1, held: (flags >> 4) & 1, local: (flags >> 5) & 1,
    imu_angle_flag: (flags >> 6) & 1, launch_delay: (flags >> 7) & 1, gyro_cal: (flags >> 8) & 1,
    image_error: s16(v[7]),
    yaw_target_dps: s16(v[8]) / 10, gyro_dps: s16(v[9]) / 10,
    setL: s16(v[10]) / 10, encL: s16(v[11]), pwmL: s16(v[12]),
    setR: s16(v[13]) / 10, encR: s16(v[14]), pwmR: s16(v[15]),
    fan_duty: v[16], stop_reason: v[17],
    imu_sum: s32(v[18]) / 10, target_imu: s32(v[19]) / 10,
    vis_age_ms: v[20], tx_dropped: v[21], frame_seq: v[22], cfg_rev: v[23],
  };
}

export function parseProLine(text) {
  if (typeof text !== 'string' || !/^@(GP1|WA1|RA1|QX1),/.test(text)) return null;
  const body = verifyStarCrc(text);
  if (!body) return null;
  const p = body.split(',');
  if (p[0] === '@GP1' && (p.length === 25 || p.length === 27)) {
    const n = p.slice(1).map(Number);
    if (n.some(x => !Number.isFinite(x))) return null;
    const base = {
      type: 'pro-params',
      rid: n[0], rev: n[1], mode: n[2], wheel: n[3], step_mode: n[4],
      race_speed: n[5], test_speed: n[6], test_low: n[7], test_yaw: n[8],
      fan_duty: n[9], tune_fan: n[10],
      lkp: n[11] / 100, lki: n[12] / 1000, rkp: n[13] / 100, rki: n[14] / 1000,
      ykp: n[15] / 10000, yki: n[16] / 10000,
      turn_speed: n[17], yaw_cap: n[18],
      dirp_cur: n[19] / 1000, dird_cur: n[20] / 1000,
      img_mode: n[21], img_fps: n[22], dur_ms: n[23],
    };
    if (p.length === 27) {
      base.topo = n[24] ? 1 : 0;
      base.yff = n[25] / 1000;
    }
    return base;
  }
  if (p[0] === '@WA1' && p.length === 5) {
    return { type: 'pro-ack', rid: Number(p[1]), rev: Number(p[2]), key: p[3], value: Number(p[4]) };
  }
  if (p[0] === '@RA1' && p.length === 5) {
    return { type: 'pro-run', action: p[1], rid: Number(p[2]), a: Number(p[3]), b: Number(p[4]) };
  }
  if (p[0] === '@QX1' && p.length === 5) {
    return { type: 'pro-error', rid: Number(p[1]), rev: Number(p[2]), code: p[3], lastRid: Number(p[4]) };
  }
  return null;
}

export function decodeCompressedFrame(fmt, payload) {
  if (fmt === 1) {
    const pixels = new Uint8Array(188 * 120);
    let pos = 0;
    for (let i = 0; i < payload.length && pos < pixels.length; i++) {
      const b = payload[i], val = (b & 0x80) ? 255 : 0, run = b & 0x7f;
      const end = Math.min(pixels.length, pos + run);
      pixels.fill(val, pos, end);
      pos = end;
    }
    return { kind: 'bin', width: 188, height: 120, pixels };
  }
  if (fmt === 2 && payload.length === 2820) {
    const pixels = new Uint8Array(188 * 120);
    for (let i = 0; i < 2820; i++) {
      const b = payload[i];
      for (let bit = 0; bit < 8; bit++) pixels[i * 8 + bit] = (b & (0x80 >> bit)) ? 255 : 0;
    }
    return { kind: 'bin', width: 188, height: 120, pixels };
  }
  if (fmt === 3 && payload.length === 2820) {
    const pixels = new Uint8Array(94 * 60);
    for (let i = 0; i < 2820; i++) {
      const b = payload[i], hi = b >> 4, lo = b & 0x0f;
      pixels[i * 2] = (hi << 4) | hi;
      pixels[i * 2 + 1] = (lo << 4) | lo;
    }
    return { kind: 'gray', width: 94, height: 60, pixels };
  }
  if (fmt === 4 && payload.length === 1410) {
    return { kind: 'gray', width: 47, height: 30, pixels: payload.slice() };
  }
  return null;
}

// 状态行 "k=v k=v …" → { k: 数字 }；不是状态行返回 null
export function parseStatus(line) {
  if (!line.startsWith('t=')) return null;
  const fields = {};
  for (const part of line.trim().split(/\s+/)) {
    const m = /^([a-zA-Z_]+)=(-?\d+)$/.exec(part);
    if (m && Number.isSafeInteger(Number(m[2]))) fields[m[1]] = Number(m[2]);
  }
  return Number.isSafeInteger(fields.t) && fields.t >= 0 ? fields : null;
}

// Only the two verified square-response builds can expand the legacy command limits.
const LIVE_SPEED_BUILD = '2BFB-SQUARE-LIVE-01';
const HOLD_SPEED_BUILD = '2BFB-SQUARE-HOLD-01';
const SQUARE_BUILDS = new Set(['2BFB-SQUARE-01', '2BFB-SQUARE-02', LIVE_SPEED_BUILD, HOLD_SPEED_BUILD]);
const GUARDED_LOW_BUILD = '2BFB-LOW250-02';
const knownTuningProfile = ({ build, profile, speed_limit }) =>
  (SQUARE_BUILDS.has(build) && profile === 'zero-high-zero' && speed_limit === 200)
  || (build === GUARDED_LOW_BUILD && profile === 'low-high-low' && speed_limit === 250);
const hasTuningCaps = caps => caps?.recognized === true && knownTuningProfile(caps)
  && caps.yaw_limit === 360 && caps.wheel_limit === 60;

export function tuningTargetLimit(side, caps) {
  if (hasTuningCaps(caps)) return side >= 4 ? caps.yaw_limit : caps.speed_limit;
  if (caps?.low300Wheel === true && side >= 1 && side <= 3) return 300;
  return 80;
}

function tuningFields(text) {
  const fields = {};
  for (const token of text.trim().split(/ +/)) {
    const m = /^([A-Za-z][A-Za-z0-9_]*)=([A-Za-z0-9_.-]+)$/.exec(token);
    if (!m || Object.hasOwn(fields, m[1])) return null;
    fields[m[1]] = m[2];
  }
  return fields;
}
const boundedField = (fields, key, max) => /^\d+$/.test(fields?.[key] ?? '') && Number.isSafeInteger(Number(fields[key])) && Number(fields[key]) <= max;

export function parseTuningConfig(text) {
  if (typeof text !== 'string' || !text.startsWith('[tune_cfg] ')) return null;
  const fields = tuningFields(text.slice(11));
  if (!fields || !['fan_pct', 'speed_limit', 'yaw_limit', 'wheel_limit'].every(k => boundedField(fields, k, k === 'fan_pct' ? 100 : 2000))
    || !fields.profile || !fields.build) return null;
  const config = { ...fields };
  for (const key of ['fan_pct', 'speed_limit', 'yaw_limit', 'wheel_limit']) config[key] = Number(fields[key]);
  const validLimits = config.yaw_limit === 360 && config.wheel_limit === 60 && config.fan_pct <= 50;
  config.recognized = validLimits && knownTuningProfile(config);
  const low300Wheel = validLimits && config.build === '2BFB-LOW300-01'
    && config.profile === 'low-high-low' && config.speed_limit === 300;
  const liveSpeed = config.recognized && [LIVE_SPEED_BUILD, HOLD_SPEED_BUILD].includes(config.build)
    && /^[01]$/.test(fields.live ?? '');
  return { ...config, low300Wheel, side_limit: config.recognized || low300Wheel ? 5 : 3,
    fan_limit: config.recognized || low300Wheel ? 50 : 0,
    liveSpeed, live: liveSpeed && fields.live === '1', holdSpeed: liveSpeed && config.build === HOLD_SPEED_BUILD };
}

export function parseTuningParams(text, caps = null) {
  const family = /^\[tune\] params /.test(text) ? 'production' : /^\[hwtest\] params /.test(text) ? 'hwtest' : null;
  if (!family) return null;
  const raw = tuningFields(text.slice(text.indexOf('params ') + 7));
  if (!raw) return null;
  const keys = ['kpL', 'kiL', 'kpR', 'kiR', 'prof', 'side', 'target', 'active', 'akp', 'aki'];
  const fields = Object.fromEntries(Object.entries(raw).filter(([key]) => keys.includes(key)).map(([key, value]) => [key, /^\d+$/.test(value) ? Number(value) : NaN]));
  const valid = (key, max) => Number.isInteger(fields[key]) && fields[key] >= 0 && fields[key] <= max;
  if (!['kpL', 'kiL', 'kpR', 'kiR'].every(k => valid(k, 2000))) return null;
  if (family === 'production') {
    const recognized = hasTuningCaps(caps), yaw = fields.side >= 4;
    if (!valid('side', recognized ? 5 : 3) || !valid('target', tuningTargetLimit(fields.side, caps))
      || !valid('active', 1) || (!fields.side && fields.target)) return null;
    if (yaw && !['akp', 'aki'].every(k => valid(k, 2000))) return null;
  } else if (!valid('prof', 3)) return null;
  return { family, ...fields };
}

// null means this reply does not carry the fields for the outstanding write.
export function checkTuningReadback(command, params, config) {
  const [name, ...args] = command.trim().split(' ');
  const expected = args.map(Number);
  if (name === '#F') return config ? config.fan_pct === expected[0] : null;
  if (name === '#J') return config && params ? config.live === true && params.target === expected[0] : null;
  if (!params || params.family !== 'production') return null;
  const fields = { '#K': ['kpL', 'kiL', 'kpR', 'kiR'], '#A': ['akp', 'aki'], '#T': ['side', 'target'] }[name];
  return fields ? fields.every((field, i) => params[field] === expected[i]) : null;
}

function parseSquareHeader(text) {
  const raw = tuningFields(text.slice('[tune2] cap begin '.length));
  if (!raw || !['n', 'dt_ms', 'stride', 'side', 'fan_pct', 'kpL', 'kiL', 'kpR', 'kiR', 'akp', 'aki', 'profile', 'build'].every(k => Object.hasOwn(raw, k))) return null;
  if (!['n', 'dt_ms', 'stride', 'side', 'fan_pct', 'kpL', 'kiL', 'kpR', 'kiR', 'akp', 'aki'].every(k => boundedField(raw, k, 2000))) return null;
  const h = { ...raw, version: 2 };
  for (const key of ['n', 'dt_ms', 'stride', 'side', 'fan_pct', 'kpL', 'kiL', 'kpR', 'kiR', 'akp', 'aki']) h[key] = Number(raw[key]);
  const tunePro = /(?:^|-)TUNEPRO-\d+$/.test(h.build) && ['zero-high-zero', 'step-stair', 'yaw-alt', 'live-hold'].includes(h.profile)
    && (h.dt_ms === 2 || h.dt_ms === 4) && h.fan_pct <= 80;
  const square = h.profile === 'zero-high-zero' && h.dt_ms === 2
    && (SQUARE_BUILDS.has(h.build) || (h.build === GUARDED_LOW_BUILD && h.side >= 4));
  const guarded = h.profile === 'low-high-low' && h.dt_ms === 2 && h.build === GUARDED_LOW_BUILD && h.side <= 3;
  const live = h.dt_ms === 6 && h.side <= 3
    && ((h.profile === 'live-step' && h.build === LIVE_SPEED_BUILD)
      || (h.profile === 'live-hold' && h.build === HOLD_SPEED_BUILD));
  if ((!tunePro && !square && !live && !guarded) || h.n < 1 || h.n > 1000 || h.stride !== 1 || h.side < 1 || h.side > 5 || (!tunePro && h.fan_pct > 50)) return null;
  return h;
}

// 调参包：校验对了才算；返回 { stop, speed, error } 或 null
export function parsePacket(bytes, at = 0) {
  if (bytes.length - at < PACKET_LEN || bytes[at] !== PACKET_HEAD || bytes[at + 9] !== PACKET_TAIL) return null;
  let sum = 0;
  for (let i = 1; i <= 7; i++) sum = (sum + bytes[at + i]) & 0xff;
  if (sum !== bytes[at + 8]) return null;
  const view = new DataView(bytes.buffer, bytes.byteOffset + at, PACKET_LEN);
  const error = view.getFloat32(4, true);
  return Number.isFinite(error) ? { stop: bytes[at + 1], speed: view.getInt16(2, true), error } : null;
}

// 逐飞官方例程（逐飞助手协议，seekfree_assistant.c）自己不停发图：图像头 AA 02 类型 08 宽(LE16) 高(LE16)，紧跟 宽×高 字节灰度，
// 后面跟一段边线点包（AA 03 …），没有结束标记。类型高三位 2 = 总钻风；bit4 = 1 表示这一包不带图像
function officialHeader(b, at) {
  if (b.length - at < 8 || b[at] !== 0xaa || b[at + 1] !== 0x02 || b[at + 3] !== 8) return null;
  const type = b[at + 2], width = b[at + 4] | b[at + 5] << 8, height = b[at + 6] | b[at + 7] << 8;
  return type >> 5 === 2 && !(type & 0x10) && width && height && width * height <= MAX_FRAME_BYTES ? { width, height } : null;
}

// 边线坐标包按声明的长度跳过，不能把两张图之间的状态行和命令回复一起丢掉。
function officialDotsBytes(b) {
  if (b.length < 8 || b[0] !== 0xaa || b[1] !== 3 || b[3] !== 8) return null;
  const type = b[2], axes = type >> 6, channels = type & 15, count = b[4] | b[5] << 8;
  const flags = b[6].toString(2).replaceAll('0', '').length;
  const size = count * channels * (type & 0x20 ? 2 : 1) * (axes === 2 ? 2 : 1);
  return axes <= 2 && !(type & 0x10) && channels >= 1 && channels <= 8 && flags === channels && size <= MAX_FRAME_BYTES ? size : null;
}

// 有状态的解析器：push(任意切开的字节块)，按到达顺序回调 onEvent({type, …})；文件读完调 push.end()，没收到结尾的黑匣子也交出来
//   type: 'status' {fields, line} · 'line' {text} · 'packet' {stop, speed, error} · 'frame' {kind, width, height, seq, pixels}
//         'frame-lost' {kind, seq, reason} · 'junk' {bytes} · 'identity' {fields, line}
//         'tlm' {ver, row} · 'tlm-bad' {reason: 'crc'|'length'|'numeric'} · 'blackbox' {header|null, end, rows, complete, issues}
// 导出命令前调 push.expectBlackbox()；缺头时 v2 字节与实时包相同，必须由调用者说明正在收历史记录。
export function createLiveParser(onEvent) {
  let buf = new Uint8Array(0), frame = null, line = [], officialSeq = 0, skipDots = 0, droppingLine = false, box = null, tuneCap = null, imgAsm = null;
  const take = n => { const head = buf.subarray(0, n); buf = buf.subarray(n); return head; };
  const compressedSlice = () => {
    if (buf.length < 28) return false;
    const fmt = buf[2], flags = buf[3], sliceIdx = buf[4], sliceTotal = buf[5];
    const slen = buf[6] | (buf[7] << 8), totalLen = buf[8] | (buf[9] << 8);
    if (fmt < 1 || fmt > 4 || sliceTotal < 1 || sliceTotal > 16 || sliceIdx >= sliceTotal || slen < 1 || slen > 240 || totalLen < slen || totalLen > 2880) {
      take(2); onEvent({ type: 'tlm-bad', reason: 'img-header' }); return true;
    }
    if (buf.length < 28 + slen) return false;
    const view = new DataView(buf.buffer, buf.byteOffset, 28 + slen);
    if (crc16(buf.subarray(2, 26 + slen)) !== view.getUint16(26 + slen, true)) {
      take(2); onEvent({ type: 'tlm-bad', reason: 'img-crc' }); return true;
    }
    const seq = view.getUint32(10, true), stamp_ms = view.getUint32(14, true), image_error = view.getInt16(18, true);
    const avg_gray = buf[20], route_k = buf[21], lp = view.getUint16(22, true), rp = view.getUint16(24, true);
    const sliceBytes = buf.subarray(26, 26 + slen).slice();
    take(28 + slen);
    line = []; droppingLine = false;
    if (!imgAsm || imgAsm.seq !== seq || imgAsm.fmt !== fmt || imgAsm.totalLen !== totalLen || imgAsm.sliceTotal !== sliceTotal) {
      imgAsm = { seq, fmt, flags, totalLen, sliceTotal, stamp_ms, image_error, avg_gray, route_k, lp, rp, parts: new Array(sliceTotal), got: 0 };
    }
    if (!imgAsm.parts[sliceIdx]) { imgAsm.parts[sliceIdx] = sliceBytes; imgAsm.got++; }
    if (imgAsm.got === imgAsm.sliceTotal) {
      const full = new Uint8Array(imgAsm.totalLen);
      let off = 0;
      for (const part of imgAsm.parts) { full.set(part, off); off += part.length; }
      const decoded = off === imgAsm.totalLen ? decodeCompressedFrame(imgAsm.fmt, full) : null;
      if (decoded) {
        onEvent({
          type: 'frame', ...decoded, seq: imgAsm.seq, compressed: true, compBytes: imgAsm.totalLen, fmt: imgAsm.fmt,
          meta: { stamp_ms: imgAsm.stamp_ms, image_error: imgAsm.image_error, avg_gray: imgAsm.avg_gray, route_k: imgAsm.route_k, lp: imgAsm.lp, rp: imgAsm.rp, valid: !!(imgAsm.flags & 1), running: !!(imgAsm.flags & 2) },
        });
      } else {
        onEvent({ type: 'frame-lost', kind: fmt <= 2 ? 'bin' : 'gray', seq: imgAsm.seq, reason: '压缩图像解码长度异常' });
      }
      imgAsm = null;
    }
    return true;
  };
  const issueBox = issue => { if (box && !box.issues.includes(issue)) box.issues.push(issue); };
  const badTelemetry = reason => { issueBox(`遥测坏包：${reason}`); onEvent({ type: 'tlm-bad', reason }); };
  const tuneIssue = text => { if (tuneCap && !tuneCap.issues.includes(text)) tuneCap.issues.push(text); };
  const closeTuneCap = (ended, issue) => {
    if (!tuneCap) return;
    if (issue) tuneIssue(issue);
    if (tuneCap.rows.length !== Math.ceil(tuneCap.header.n / tuneCap.header.stride)) tuneIssue('记录条数不完整');
    onEvent({ type: 'tune-capture', ...tuneCap, complete: ended && !tuneCap.issues.length });
    tuneCap = null;
  };
  // 黑匣子导出：头 0x82 和尾 0x83 之间的 v2 记录不是实时数据，凑齐后一起交出；条数、导出时刻对不上就标不完整
  const closeBox = (end, issue) => {
    if (!box) return;
    if (issue) issueBox(issue);
    if (!box.header) issueBox('没收到开头');
    if (end && (end.records !== box.rows.length || box.header?.records !== box.rows.length)) issueBox('条数对不上');
    if (end && box.header && end.export_ms !== box.header.export_ms) issueBox('头尾导出时刻不一致');
    if (box.header && (box.header.first_record_seq !== (box.rows[0]?.record_seq ?? 0)
      || box.header.last_record_seq !== (box.rows.at(-1)?.record_seq ?? 0))) issueBox('首尾记录序号不一致');
    onEvent({ type: 'blackbox', header: box.header, end, rows: box.rows, complete: !!end && !box.issues.length, issues: box.issues });
    box = null;
  };
  // 返回 false = 字节还不够，等下一块；版本 / 长度不认识或 CRC 错就丢 2 字节重新找 A5 5A
  const telemetry = () => {
    if (buf.length < 4) return false;
    const ver = buf[2], len = buf[3], cols = TLM_LAYOUTS.get(`${ver}:${len}`);
    if (!cols) { take(2); badTelemetry('length'); return true; }
    if (buf.length < len + 6) return false;
    const view = new DataView(buf.buffer, buf.byteOffset, len + 6);
    if (crc16(buf.subarray(2, len + 4)) !== view.getUint16(len + 4, true)) { take(2); badTelemetry('crc'); return true; }
    const row = readRow(view, 4, cols);
    take(len + 6);
    if (!Object.values(row).every(Number.isFinite)) { badTelemetry('numeric'); return true; }
    if (ver === 0x82) {
      if (box && !box.header && !box.rows.length) box = null;
      closeBox(null, '下一段导出开始前没收到结尾');
      box = { header: row, rows: [], issues: [] };
      if (!row.capacity || row.capacity > 500 || row.records > row.capacity) issueBox('黑匣子容量或条数不合法');
    } else if (ver === 0x83) {
      if (!box) box = { header: null, rows: [], issues: ['没收到开头'] };
      closeBox(row);
    } else if (box) {
      if (ver !== 2) issueBox('黑匣子中混入了其他版本遥测');
      else if (box.rows.length >= Math.min(box.header?.records ?? 500, 500)) issueBox('记录超过声明条数或容量上限');
      else {
        const previous = box.rows.at(-1), delta = previous && (row.record_seq - previous.record_seq) >>> 0;
        if (previous && (!delta || delta >= 0x80000000)) issueBox('记录序号重复或倒退');
        box.rows.push(row);
      }
    } else onEvent({ type: 'tlm', ver, row });
    return true;
  };
  const appendLine = bytes => {
    if (droppingLine || line.length + bytes.length > MAX_LINE) {
      onEvent({ type: 'junk', bytes: line.length + bytes.length });
      line = []; droppingLine = true;
      return;
    }
    for (const byte of bytes) line.push(byte);
  };
  const flushLine = () => {
    if (droppingLine) { droppingLine = false; return; }
    // 半个调参包（抓取开始时截断的）会粘在行首：去掉不可打印字节；状态行从 "t=" 起算
    let text = LATIN1_DECODER.decode(Uint8Array.from(line)).replace(/[^\x20-\x7e]/g, '').trim();
    line = [];
    const at = text.search(/t=\d+ vbat_mv=|@(?:M2|GP1|WA1|RA1|QX1),/);
    if (at > 0) text = text.slice(at);
    if (!text) return;
    if (text.startsWith('[tune2] cap begin')) {
      closeTuneCap(false, '下一趟记录开始前没收到结束标记');
      const header = parseSquareHeader(text);
      if (header) tuneCap = { header, rows: [], issues: [] };
      else onEvent({ type: 'line', text: '[tune2] error invalid capture header' });
      onEvent({ type: 'line', text }); return;
    }
    if (text === '[tune2] cap end') {
      if (tuneCap?.header.version !== 2) tuneIssue('记录头尾版本不一致');
      closeTuneCap(true); onEvent({ type: 'line', text }); return;
    }
    if (tuneCap?.header.version === 2 && text.startsWith('u2,')) {
      const cells = text.slice(3).split(','), r = cells.map(Number), h = tuneCap.header;
      if (cells.length !== 10 || cells.some(c => !/^-?\d+$/.test(c)) || r.some(n => !Number.isSafeInteger(n))
        || r[0] < 0 || r[0] >= h.n || r.slice(1, 9).some(n => n < -32768 || n > 32767)
        || Math.abs(r[4]) > 10000 || Math.abs(r[5]) > 10000 || ![0, 1, 2].includes(r[9])) {
        tuneIssue('记录行格式或范围错误'); return;
      }
      const [index, target, encL, encR, pwmL, pwmR, gyro, setL, setR, phase] = r;
      const previous = tuneCap.rows.at(-1);
      const time = index * h.dt_ms;
      const tunePro = /(?:^|-)TUNEPRO-\d+$/.test(h.build);
      if (previous && index <= previous.index) { tuneIssue('记录序号重复或倒退'); return; }
      if (index !== (previous ? previous.index + 1 : 0)) tuneIssue('记录中间有缺行');
      if (tunePro) {
        const lim = h.side >= 4 ? 600 : 350;
        if (Math.abs(target) > lim) tuneIssue('阶跃目标超出固件上限');
      } else {
        const hold = h.profile === 'live-hold';
        const expectedPhase = hold ? 1 : time < 500 ? 0 : time < (h.profile === 'live-step' ? 5500 : 1500) ? 1 : 2;
        const guarded = h.profile === 'low-high-low';
        const low = tuneCap.rows[0]?.target ?? target;
        const targetLimit = h.side >= 4 ? 360 : guarded ? 250 : 200;
        const invalidPhaseTarget = guarded
          ? low < 1 || low > 150 || (phase === 1 ? Math.min(target, 150) !== low : target !== low)
          : phase !== 1 && target !== 0;
        if (phase !== expectedPhase || invalidPhaseTarget || (hold && target === 0) || Math.abs(target) > targetLimit
          || (h.side === 5 ? target > 0 : target < 0)) tuneIssue('阶跃阶段或目标不符合固件约定');
      }
      tuneCap.rows.push({ index, t: index * h.dt_ms, target, base: target, encL, encR, pwmL, pwmR, gyro_dps: gyro / 10, setL, setR, phase,
        ...(h.side >= 4 ? { yaw_target_dps: target } : {}) });
      return;
    }
    if (tuneCap?.header.version === 2 && !text.startsWith('[tune] cap begin')) tuneIssue('记录中混入其他数据');
    if (text.startsWith('[tune] cap begin')) {
      closeTuneCap(false, '下一趟记录开始前没收到结束标记');
      const m = /^\[tune\] cap begin n=(\d+) dt_ms=2 stride=([12]) side=([123])(?: kpL=(\d+) kiL=(\d+) kpR=(\d+) kiR=(\d+))?$/.exec(text);
      if (m && Number(m[1]) <= 1000 && m.slice(4).every(v => v === undefined || Number(v) <= 2000)) {
        const header = { n: Number(m[1]), dt_ms: 2, stride: Number(m[2]), side: Number(m[3]) };
        if (m[4] !== undefined) ['kpL', 'kiL', 'kpR', 'kiR'].forEach((key, i) => { header[key] = Number(m[i + 4]); });
        tuneCap = { header, rows: [], issues: [] };
      }
      else onEvent({ type: 'line', text: '[tune] error invalid capture header' });
      onEvent({ type: 'line', text }); return;
    }
    if (text === '[tune] cap end') { if (tuneCap?.header.version === 2) tuneIssue('记录头尾版本不一致'); closeTuneCap(true); onEvent({ type: 'line', text }); return; }
    if (tuneCap && tuneCap.header.version !== 2 && text.startsWith('u,')) {
      const cells = text.slice(2).split(','), r = cells.map(Number), h = tuneCap.header;
      if (cells.length !== 7 || cells.some(c => !/^-?\d+$/.test(c)) || r.some(n => !Number.isSafeInteger(n))
        || r[0] < 0 || r[0] >= h.n || r[0] % h.stride || r[1] < 0 || r[1] > 80
        || r.slice(2).some((n, i) => n < (i === 2 || i === 3 ? -10000 : -32768) || n > (i === 2 || i === 3 ? 10000 : 32767))) {
        tuneIssue('记录行格式或范围错误'); return;
      }
      const previous = tuneCap.rows.at(-1);
      if (previous && r[0] <= previous.index) { tuneIssue('记录序号重复或倒退'); return; }
      if (r[0] !== (previous ? previous.index + h.stride : 0)) tuneIssue('记录中间有缺行');
      const [index, base, encL, encR, pwmL, pwmR, gyro] = r;
      tuneCap.rows.push({ index, t: index * h.dt_ms, setL: h.side === 2 ? 0 : base, setR: h.side === 1 ? 0 : base, encL, encR, pwmL, pwmR, gyro_dps: gyro / 10 });
      return;
    }
    if (text.startsWith('@M2,')) {
      const row = parseM2Line(text);
      if (row) onEvent({ type: 'm2', row });
      else onEvent({ type: 'tlm-bad', reason: 'm2-crc' });
      return;
    }
    if (text.startsWith('@GP1,') || text.startsWith('@WA1,') || text.startsWith('@RA1,') || text.startsWith('@QX1,')) {
      const pro = parseProLine(text);
      if (pro) onEvent(pro);
      else onEvent({ type: 'tlm-bad', reason: 'pro-crc' });
      onEvent({ type: 'line', text });
      return;
    }
    const header = /^IMG (gray|bin) (\d+) (\d+) (\d+)$/.exec(text);
    if (header) {
      const [kind, width, height, seq] = [header[1], +header[2], +header[3], +header[4]];
      if (width > 0 && height > 0 && width * height <= MAX_FRAME_BYTES) { frame = { kind, width, height, seq }; return; }
      onEvent({ type: 'frame-lost', kind, seq, reason: `图像头尺寸不对 ${width}×${height}` });
      return;
    }
    if (/^ID fw=/.test(text)) { onEvent({ type: 'identity', fields: Object.fromEntries([...text.matchAll(/(\w+)=(\S+)/g)].map(m => [m[1], m[2]])), line: text }); return; }
    const fields = parseStatus(text);
    onEvent(fields ? { type: 'status', fields, line: text } : { type: 'line', text });
  };
  push.expectBlackbox = () => {
    closeBox(null, '新的导出请求开始前没收到结尾');
    box = { header: null, rows: [], issues: [] };
  };
  push.end = () => {
    closeBox(null, '没收到结尾（导出被打断或文件截断）');
    closeTuneCap(false, '没收到结束标记');
    buf = new Uint8Array(0); frame = null; imgAsm = null; line = []; skipDots = 0; droppingLine = false;
  };
  return push;
  function push(chunk) {
    if (chunk?.length) {
      if (!buf.length) buf = chunk;
      else {
        const next = new Uint8Array(buf.length + chunk.length);
        next.set(buf);
        next.set(chunk, buf.length);
        buf = next;
      }
    }
    for (;;) {
      if (skipDots) {
        skipDots -= take(Math.min(skipDots, buf.length)).length;
        if (skipDots) return;
      }
      if (frame?.official) {
        if (buf.length < frame.width * frame.height) return;
        onEvent({ type: 'frame', ...frame, pixels: take(frame.width * frame.height).slice() });   // official: true = 车自己发的
        frame = null;
        continue;
      }
      if (!frame && buf[0] === 0xaa) {
        if (buf.length < 8) return;
        const head = officialHeader(buf, 0);
        if (head) { take(8); line = []; droppingLine = false; frame = { kind: 'gray', ...head, seq: ++officialSeq, official: true }; continue; }
        const dots = officialDotsBytes(buf);
        if (dots !== null) { take(8); line = []; droppingLine = false; skipDots = dots; continue; }
        appendLine(take(1));                                // 不是图像头：当普通字节
        continue;
      }
      if (frame) {
        const need = frame.width * frame.height + END.length;
        if (buf.length < need) return;
        const pixels = take(frame.width * frame.height).slice(), tail = take(END.length);
        if (END.every((b, i) => tail[i] === b)) onEvent({ type: 'frame', ...frame, pixels });
        else {
          onEvent({ type: 'frame-lost', kind: frame.kind, seq: frame.seq, reason: '结束标记不在 188×120 字节之后：传输中丢了或多了字节' });
          const restart = new Uint8Array(pixels.length + tail.length); restart.set(pixels); restart.set(tail, pixels.length);
          const rest = buf; buf = new Uint8Array(restart.length + rest.length); buf.set(restart); buf.set(rest, restart.length);
          buf = buf.subarray(1);                              // 从图像数据第二个字节起重新按文本找同步，避免死循环
        }
        frame = null;
        continue;
      }
      if (!buf.length) return;
      if (buf[0] === PACKET_HEAD) {
        if (buf.length < 2) return;
        if (buf[1] === PACKET_TAIL) { if (telemetry()) continue; return; }
        if (buf[1] === 0x5b) { if (compressedSlice()) continue; return; }
        if (buf.length < PACKET_LEN) return;                  // 包可能还没收完
        const packet = parsePacket(buf);
        if (packet) { take(PACKET_LEN); onEvent({ type: 'packet', ...packet }); continue; }
        if (buf[9] === PACKET_TAIL) { take(PACKET_LEN); onEvent({ type: 'junk', bytes: PACKET_LEN }); continue; }   // 包头包尾都对、校验错：坏包
        appendLine(take(1));                                // 不是包：当普通字节
        continue;
      }
      const nl = buf.indexOf(0x0a), head = buf.indexOf(PACKET_HEAD), aa = buf.indexOf(0xaa);
      if (head > 0 && (nl < 0 || head < nl) && (aa < 0 || head < aa)) { appendLine(take(head)); continue; }
      if (aa > 0 && (nl < 0 || aa < nl)) { appendLine(take(aa)); continue; }   // 停在可能的官方图像头前
      if (nl < 0) {
        appendLine(take(buf.length));
        return;
      }
      appendLine(take(nl + 1));
      flushLine();
    }
  };
}

// 灰度统计：min / 中位数 / p90 / max / 平均 / ≥250 的点数
export function frameStats(pixels) {
  const counts = new Uint32Array(256);
  let sum = 0;
  for (const p of pixels) { counts[p]++; sum += p; }
  const n = pixels.length, at = q => { let k = 0; for (let v = 0; v < 256; v++) { k += counts[v]; if (k > q) return v; } return 255; };
  let min = 0, max = 255;
  while (min < 255 && !counts[min]) min++;
  while (max > 0 && !counts[max]) max--;
  let saturated = 0;
  for (let v = 250; v < 256; v++) saturated += counts[v];
  return { min, median: at(Math.floor(n / 2)), p90: at(Math.floor(n * 0.9)), max, mean: n ? sum / n : 0, saturated, zero: counts[0] };
}

// 清晰度（对焦用）：相邻像素左右、上下差的平方平均再开方（梯度能量）。差的绝对值平均不行：一条边模糊成渐变，差的总和不变。
// 同一场景、同样光照下越大越清楚；不同场景之间不能比
export function frameSharpness(pixels, width, height) {
  let sum = 0, n = 0;
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const i = y * width + x;
      if (x + 1 < width) { const d = pixels[i + 1] - pixels[i]; sum += d * d; n++; }
      if (y + 1 < height) { const d = pixels[i + width] - pixels[i]; sum += d * d; n++; }
    }
  }
  return n ? Math.sqrt(sum / n) : 0;
}

// 发给车的命令（固件只在停车时执行）；值不合法返回 null
export const commands = {
  // hwtest v4/v5: one bounded line, no start command or arbitrary byte injection.
  tuning: (text, family = 'hwtest', caps = null) => {
    if (typeof text !== 'string' || text.length > 47 || /[^\x20-\x7e]/.test(text)) return null;
    const [name, ...args] = text.trim().split(/ +/);
    const square = family === 'production' && hasTuningCaps(caps);
    const limits = (family === 'production' ? { '#P': [], '#C': [], '#T': [square ? 5 : 3, tuningTargetLimit(Number(args[0]), caps)], '#K': [2000, 2000, 2000, 2000], ...(caps?.fan_limit === 50 ? { '#F': [50] } : {}), ...(square ? { '#A': [2000, 2000] } : {}), ...(square && caps.liveSpeed ? { '#J': [200] } : {}) }
      : family === 'hwtest' ? { '#P': [], '#C': [], '#Q': [3], '#Y': [50], '#F': [3000, 3000], '#K': [2000, 2000, 2000, 2000] } : { '#P': [] })[name];
    if (!Array.isArray(limits) || args.length !== limits.length || args.some((v, i) => !/^\d+$/.test(v) || !Number.isSafeInteger(Number(v)) || Number(v) > limits[i])) return null;
    if (name === '#T' && Number(args[0]) === 0 && Number(args[1]) !== 0) return null;
    if (name === '#J' && Number(args[0]) === 0) return null;
    return [name, ...args.map(Number)].join(' ') + '\n';
  },
  grab: kind => kind === 'bin' ? '#B' : '#I',
  exposure: v => Number.isInteger(v) && v >= 1 && v <= 65535 ? `#E${v}\r` : null,
  gain: v => Number.isInteger(v) && v >= 16 && v <= 64 ? `#G${v}\r` : null,
  // 视频（固件 p26 起，只在下载器口、车停着时）：full = 188×120，half = 94×60（2×2 平均，帧率更高），off = 停
  video: mode => ({ full: 'V', half: 'U', off: 'v' })[mode] ?? null,
  // TunePro-01 CRC16 保护三环快速调参 + 远程发车/急停 + 实时压缩图传指令
  proMeta: rid => signCommand(`#M ${Math.max(1, Math.floor(Number(rid) || 1))}`),
  proWrite: (rid, rev, key, val) => /^[a-z0-9_]{1,20}$/.test(String(key)) && Number.isInteger(val) && val >= 0
    ? signCommand(`#W ${Math.floor(rid)} ${Math.floor(rev)} ${key} ${val}`) : null,
  proWriteBand: (rid, rev, kind, lo, hi, val) => (kind === 'p' || kind === 'd') && [lo, hi, val].every(x => Number.isInteger(x) && x >= 0) && lo <= hi
    ? signCommand(`#D ${Math.floor(rid)} ${Math.floor(rev)} ${kind} ${lo} ${hi} ${val}`) : null,
  proGo: (rid, rev, token = 1) => signCommand(`#X GO ${Math.floor(rid)} ${Math.floor(rev)} ${token === 2 ? 2 : 1}`),
  proRun: (kind, target) => {
    const k = String(kind || '').toUpperCase();
    if (!['L', 'R', 'B', 'W'].includes(k) || !Number.isInteger(target)) return null;
    if (k === 'W' ? (target < -600 || target > 600) : (target < 0 || target > 350)) return null;
    return signCommand(`#X RUN ${k} ${target}`);
  },
  proStop: () => signCommand('#X STOP'),
  proEmergencyStop: () => '#!',
  proPing: () => signCommand('#X PING'),
  proTarget: val => Number.isInteger(val) && val >= -600 && val <= 600 ? signCommand(`#X TGT ${val}`) : null,
  proImg: (mode, fps = 6) => Number.isInteger(mode) && mode >= 0 && mode <= 3 && Number.isInteger(fps) && fps >= 0 && fps <= 120
    ? signCommand(`#X IMG ${mode} ${fps}`) : null,
  proCapture: () => signCommand('#X CAP'),
};

// 极简自然快捷指令解析：如 "L 120"、"R 150"、"B 180"、"W -300"、"lkp 18"、"lki 1.8"、"kp 8.5"、"fan 40"、"s"
export function parseQuickCommand(raw) {
  const s = String(raw || '').trim();
  if (!s) return null;
  const low = s.toLowerCase();
  if (low === 's' || low === 'stop' || low === '!' || low === '停') return { action: 'stop' };
  const parts = s.replace(/[=:]+/g, ' ').trim().split(/\s+/);
  if (parts.length === 1 && /^-?\d+$/.test(parts[0])) {
    const n = Number(parts[0]);
    return n === 0 ? { action: 'stop' } : { action: 'tgt', target: n };
  }
  if (parts.length !== 2) return null;
  const head = parts[0].toLowerCase(), num = Number(parts[1]);
  if (!Number.isFinite(num)) return null;
  if (['l', 'left', '左'].includes(head)) return Number.isInteger(num) && num >= 0 && num <= 350 ? { action: 'run', kind: 'L', target: num } : null;
  if (['r', 'right', '右'].includes(head)) return Number.isInteger(num) && num >= 0 && num <= 350 ? { action: 'run', kind: 'R', target: num } : null;
  if (['b', 'both', '双'].includes(head)) return Number.isInteger(num) && num >= 0 && num <= 350 ? { action: 'run', kind: 'B', target: num } : null;
  if (['w', 'yaw', '角'].includes(head)) return Number.isInteger(num) && num >= -600 && num <= 600 ? { action: 'run', kind: 'W', target: num } : null;
  if (['v', 't', 'spd', 'speed', 'tgt'].includes(head)) return Number.isInteger(num) && num >= -600 && num <= 600 ? { action: 'tgt', target: num } : null;
  const map = {
    vkp: { key: 'vkp', scale: 100, min: 0, max: 200, kind: 'gain' },
    vki: { key: 'vki', scale: 1000, min: 0, max: 20, kind: 'gain' },
    lkp: { key: 'lkp', scale: 100, min: 0, max: 200, kind: 'gain' },
    lki: { key: 'lki', scale: 1000, min: 0, max: 20, kind: 'gain' },
    rkp: { key: 'rkp', scale: 100, min: 0, max: 200, kind: 'gain' },
    rki: { key: 'rki', scale: 1000, min: 0, max: 20, kind: 'gain' },
    ykp: { key: 'ykp', scale: 10000, min: 0, max: 60, kind: 'gain' },
    yki: { key: 'yki', scale: 10000, min: 0, max: 10, kind: 'gain' },
    yff: { key: 'yff', scale: 1000, min: 0, max: 10, kind: 'gain' },
    kp: { key: 'dirp_cur', scale: 1000, min: 0, max: 100, kind: 'gain' },
    dirp: { key: 'dirp_cur', scale: 1000, min: 0, max: 100, kind: 'gain' },
    kd: { key: 'dird_cur', scale: 1000, min: 0, max: 500, kind: 'gain' },
    dird: { key: 'dird_cur', scale: 1000, min: 0, max: 500, kind: 'gain' },
    fan: { key: 'fan', scale: 100, min: 0, max: 80, kind: 'fan' },
    race: { key: 'race_speed', scale: 1, min: 0, max: 400, kind: 'cfg' },
    turn: { key: 'turn_speed', scale: 1, min: 0, max: 400, kind: 'cfg' },
    cap: { key: 'yaw_cap', scale: 1, min: 1, max: 1500, kind: 'cfg' },
  };
  const spec = map[head];
  if (!spec || num < spec.min || num > spec.max) return null;
  const val = spec.key === 'fan' ? Math.round(num) * 100 : Math.round(num * spec.scale);
  return { action: 'write', key: spec.key, val, display: num, kind: spec.kind };
}

// 电压记录里 t 变小 = 车重启过（掉电或复位）；返回这些时刻
export function findRestarts(samples) {
  const out = [];
  for (let i = 1; i < samples.length; i++) if (samples[i].t < samples[i - 1].t) out.push({ index: i, at: samples[i].at, from: samples[i - 1].t, to: samples[i].t });
  return out;
}
