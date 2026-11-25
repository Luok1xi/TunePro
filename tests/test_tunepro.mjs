// SPDX-License-Identifier: GPL-3.0-or-later
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  createLiveParser, crc16, signCommand, parseM2Line, parseProLine,
  decodeCompressedFrame, commands, parseQuickCommand, STOP_REASONS,
} from '../web/lib/robot-live-model.js';
import { createStore, createIngest, crossings } from '../web/lib/robot-live-data.js';
import { speedResponse } from '../web/lib/robot-speed-response.js';

const enc = s => Uint8Array.from(s, c => c.charCodeAt(0) & 0xff);

function makeSlicePacket({ fmt, flags = 1, sliceIdx, sliceTotal, totalLen, seq, stampMs = 1234, errorPx = -7, avgGray = 112, routeK = 33, lp = 180, rp = 175, payload }) {
  const slen = payload.length;
  const pkt = new Uint8Array(28 + slen);
  const v = new DataView(pkt.buffer);
  pkt[0] = 0xa5; pkt[1] = 0x5b; pkt[2] = fmt; pkt[3] = flags;
  pkt[4] = sliceIdx; pkt[5] = sliceTotal;
  v.setUint16(6, slen, true);
  v.setUint16(8, totalLen, true);
  v.setUint32(10, seq, true);
  v.setUint32(14, stampMs, true);
  v.setInt16(18, errorPx, true);
  pkt[20] = avgGray; pkt[21] = routeK;
  v.setUint16(22, lp, true); v.setUint16(24, rp, true);
  pkt.set(payload, 26);
  v.setUint16(26 + slen, crc16(pkt.subarray(2, 26 + slen)), true);
  return pkt;
}

test('M2 50Hz CRC16 parsing, negative signed fields, bad CRC rejection and store ingest', () => {
  // error=-12 (0xFFF4), yaw=-250 (-25.0 -> 0xFF06), gyro=-245 (-24.5 -> 0xFF0B), setL=1250 (125.0), encL=124, pwmL=1850
  const body = '@M2,00000003,0000002A,00001388,21,FE,03,004F,FFF4,FF06,FF0B,04E2,007C,073A,0514,0082,0780,0FA0,19,FFFFFC18,FFFFFC00,0006,00000001,00000100,00000007';
  const signed = signCommand(body).trim();
  const row = parseM2Line(signed);
  assert.ok(row);
  assert.equal(row.run_id, 3);
  assert.equal(row.seq, 0x2a);
  assert.equal(row.t, 5000);
  assert.equal(row.route_k, 33);
  assert.equal(row.route_cur, -2);
  assert.equal(row.image_error, -12);
  assert.equal(row.yaw_target_dps, -25);
  assert.equal(row.gyro_dps, -24.5);
  assert.equal(row.setL, 125);
  assert.equal(row.encL, 124);
  assert.equal(row.fan_duty, 4000);
  assert.equal(row.stop_reason, 25);
  assert.equal(STOP_REASONS[25], '远程急停');
  assert.equal(row.imu_sum, -100);
  assert.equal(row.cfg_rev, 7);

  // Corrupt CRC rejected
  assert.equal(parseM2Line(signed.slice(0, -1) + (signed.endsWith('0') ? '1' : '0')), null);

  // Feed through createLiveParser + createIngest
  const events = [];
  const parse = createLiveParser(ev => events.push(ev));
  parse(enc(signed + '\r\n'));
  assert.equal(events.length, 1);
  assert.equal(events[0].type, 'm2');

  const store = createStore(64), ingest = createIngest(store);
  assert.equal(ingest.push(events[0], 100), true);
  assert.equal(store.last('encL').v, 124);
  assert.equal(store.last('gyro_dps').v, -24.5);
  assert.equal(store.last('fan_duty').v, 4000);
});

test('TunePro CRC16 command builders and @GP1/@WA1/@RA1/@QX1 reply parsers', () => {
  assert.match(commands.proMeta(1), /^#M 1\*[0-9A-F]{4}\r\n$/);
  assert.match(commands.proWrite(2, 5, 'lkp', 1500), /^#W 2 5 lkp 1500\*[0-9A-F]{4}\r\n$/);
  assert.match(commands.proWriteBand(3, 6, 'p', 100, 140, 7200), /^#D 3 6 p 100 140 7200\*[0-9A-F]{4}\r\n$/);
  assert.match(commands.proGo(4, 7, 1), /^#X GO 4 7 1\*[0-9A-F]{4}\r\n$/);
  assert.equal(commands.proEmergencyStop(), '#!');
  assert.equal(commands.proWrite(1, 0, 'bad key!', 10), null);

  const gp = parseProLine(signCommand('@GP1,1,7,6,1,1,125,200,90,350,4000,1,1500,1500,1550,1520,2200,110,80,400,7000,6000,1,8,2000').trim());
  assert.equal(gp.type, 'pro-params');
  assert.equal(gp.rev, 7);
  assert.equal(gp.mode, 6);
  assert.equal(gp.wheel, 1);
  assert.equal(gp.lkp, 15);
  assert.equal(gp.lki, 1.5);
  assert.equal(gp.ykp, 0.22);
  assert.equal(gp.yki, 0.011);
  assert.equal(gp.dirp_cur, 7);
  assert.equal(gp.dird_cur, 6);

  const wa = parseProLine(signCommand('@WA1,2,8,lkp,1600').trim());
  assert.deepEqual(wa, { type: 'pro-ack', rid: 2, rev: 8, key: 'lkp', value: 1600 });
  const ra = parseProLine(signCommand('@RA1,STOP,0,25,0').trim());
  assert.deepEqual(ra, { type: 'pro-run', action: 'STOP', rid: 0, a: 25, b: 0 });
  const qx = parseProLine(signCommand('@QX1,4,8,LOW_BAT,3').trim());
  assert.deepEqual(qx, { type: 'pro-error', rid: 4, rev: 8, code: 'LOW_BAT', lastRid: 3 });
});

test('A5 5B sliced RLE binary, 1-bit binary, GRAY4 and GRAY8 frames reassemble cleanly across interleaved @M2 and CRC corruption', () => {
  // Build synthetic 188x120 binary track stripe (cols 80..107 white) and RLE-encode it
  const binSrc = new Uint8Array(188 * 120);
  for (let y = 0; y < 120; y++) for (let x = 80; x < 108; x++) binSrc[y * 188 + x] = 255;
  const rle = [];
  for (let i = 0; i < binSrc.length;) {
    const cur = binSrc[i] ? 1 : 0;
    let run = 1; i++;
    while (i < binSrc.length && run < 127 && (binSrc[i] ? 1 : 0) === cur) { run++; i++; }
    rle.push((cur ? 0x80 : 0) | run);
  }
  const rleBytes = Uint8Array.from(rle);
  assert.ok(rleBytes.length < 700, `RLE should compress track well (<700 B), got ${rleBytes.length}`);

  const totalSlices = Math.ceil(rleBytes.length / 240);
  const m2 = enc(signCommand('@M2,00000001,00000001,00000100,00,00,00,0008,0005,0000,0000,0000,0000,0000,0000,0000,0000,0000,00,00000000,00000000,0004,00000000,00000005,00000001'));
  const streamChunks = [];
  // Inject a damaged slice first to prove CRC rejection & clean resync
  const badSlice = makeSlicePacket({ fmt: 1, sliceIdx: 0, sliceTotal: 2, totalLen: 400, seq: 99, payload: new Uint8Array(200) });
  badSlice[40] ^= 0xff;
  streamChunks.push(badSlice, m2);
  for (let s = 0; s < totalSlices; s++) {
    const part = rleBytes.subarray(s * 240, Math.min(rleBytes.length, (s + 1) * 240));
    streamChunks.push(makeSlicePacket({ fmt: 1, flags: 3, sliceIdx: s, sliceTotal: totalSlices, totalLen: rleBytes.length, seq: 100, errorPx: -7, avgGray: 118, routeK: 33, payload: part }));
    streamChunks.push(m2);
  }
  const combined = new Uint8Array(streamChunks.reduce((n, c) => n + c.length, 0));
  let off = 0;
  for (const c of streamChunks) { combined.set(c, off); off += c.length; }

  // Feed in arbitrary 17-byte misaligned chunks
  const events = [];
  const parse = createLiveParser(ev => events.push(ev));
  for (let i = 0; i < combined.length; i += 17) parse(combined.subarray(i, i + 17));

  const frames = events.filter(e => e.type === 'frame');
  const m2s = events.filter(e => e.type === 'm2');
  assert.equal(frames.length, 1);
  assert.equal(m2s.length, totalSlices + 1);
  assert.equal(frames[0].width, 188);
  assert.equal(frames[0].height, 120);
  assert.equal(frames[0].compressed, true);
  assert.equal(frames[0].meta.image_error, -7);
  assert.equal(frames[0].meta.avg_gray, 118);
  assert.deepEqual(frames[0].pixels, binSrc);

  // Verify GRAY4 (94x60) & GRAY8 (47x30) direct decoders
  const g4 = decodeCompressedFrame(3, new Uint8Array(2820).fill(0xa5));
  assert.equal(g4.width, 94); assert.equal(g4.height, 60);
  assert.equal(g4.pixels[0], 0xaa); assert.equal(g4.pixels[1], 0x55);
  const g8 = decodeCompressedFrame(4, new Uint8Array(1410).fill(123));
  assert.equal(g8.width, 47); assert.equal(g8.height, 30); assert.equal(g8.pixels[0], 123);
});

test('Multi-step stair 2nd-edge analysis skips 1st static-friction edge and computes crossings', () => {
  const rows = [];
  // 0..296ms phase 0 (0), 300..1196ms phase 1 (100), 1200..2096ms phase 2 (200), 2100..2796ms phase 1 (100)
  for (let i = 0; i < 700; i++) {
    const t = i * 4;
    const phase = t < 300 ? 0 : t < 1200 ? 1 : t < 2100 ? 2 : 1;
    const tgt = phase === 0 ? 0 : phase === 1 ? 100 : 200;
    let encL = tgt;
    if (phase === 2) {
      const dt = t - 1200;
      encL = dt < 40 ? 100 + dt * 2.5 : dt < 80 ? 210 : 200 + (Math.sin(dt / 30) * 1.2);
    }
    rows.push({ index: i, t, setL: tgt, setR: 0, encL, encR: 0, pwmL: 2000, pwmR: 0, gyro_dps: 0, phase });
  }
  const res = speedResponse({ header: { dt_ms: 4, stride: 1, side: 1, profile: 'step-stair', build: 'F125-40FAN-TUNEPRO-01' }, rows }, 'L');
  assert.equal(res.available, true);
  assert.equal(res.secondEdge, true);
  assert.equal(res.fromLevel, 100);
  assert.equal(res.target, 200);
  assert.ok(res.riseMs <= 40);
  assert.ok(Math.abs(res.overshootPct - 10) < 1);
  assert.equal(res.crossings, 0);

  // Oscillating store yields high crossing count (>4)
  const store = createStore(128);
  for (let i = 0; i < 60; i++) store.push(i * 20, { setL: 150, encL: 150 + (i % 2 === 0 ? 12 : -12) });
  const cr = crossings(store, 'setL', 'encL', 0, 1200, 2);
  assert.ok(cr.count > 20);
});

test('Continuous #X RUN builder and natural quick-command parser (L/R/B/W/PID/fan/stop)', () => {
  assert.match(commands.proRun('L', 120), /^#X RUN L 120\*[0-9A-F]{4}\r\n$/);
  assert.match(commands.proRun('W', -300), /^#X RUN W -300\*[0-9A-F]{4}\r\n$/);
  assert.equal(commands.proRun('L', 500), null);
  assert.equal(commands.proRun('L', -10), null);
  assert.equal(commands.proRun('W', 700), null);

  assert.deepEqual(parseQuickCommand('L 120'), { action: 'run', kind: 'L', target: 120 });
  assert.deepEqual(parseQuickCommand('左 180'), { action: 'run', kind: 'L', target: 180 });
  assert.deepEqual(parseQuickCommand('r 150'), { action: 'run', kind: 'R', target: 150 });
  assert.deepEqual(parseQuickCommand('b 200'), { action: 'run', kind: 'B', target: 200 });
  assert.deepEqual(parseQuickCommand('w -300'), { action: 'run', kind: 'W', target: -300 });
  assert.deepEqual(parseQuickCommand('lkp 18.5'), { action: 'write', key: 'lkp', val: 1850, display: 18.5, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('lki=2.2'), { action: 'write', key: 'lki', val: 2200, display: 2.2, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('ykp 0.35'), { action: 'write', key: 'ykp', val: 3500, display: 0.35, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('kp 8.5'), { action: 'write', key: 'dirp_cur', val: 8500, display: 8.5, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('fan 40'), { action: 'write', key: 'fan', val: 4000, display: 40, kind: 'fan' });
  assert.deepEqual(parseQuickCommand('s'), { action: 'stop' });
  assert.deepEqual(parseQuickCommand('0'), { action: 'stop' });
  assert.deepEqual(parseQuickCommand('vkp 38'), { action: 'write', key: 'vkp', val: 3800, display: 38, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('yff 1.85'), { action: 'write', key: 'yff', val: 1850, display: 1.85, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('ykp 18'), { action: 'write', key: 'ykp', val: 180000, display: 18, kind: 'gain' });
  assert.deepEqual(parseQuickCommand('cap 1380'), { action: 'write', key: 'yaw_cap', val: 1380, display: 1380, kind: 'cfg' });
  assert.equal(parseQuickCommand('L 999'), null);

  // Extended 27-field @GP1 with topology & yff + derived 2S1P pwm_cm/pwm_dm ingest
  const gpExt = parseProLine(signCommand('@GP1,1,7,3,2,3,125,140,90,300,4000,0,3800,1200,3800,1200,180000,1200,80,920,8500,6000,0,6,2000,1,1850').trim());
  assert.equal(gpExt.topo, 1);
  assert.equal(gpExt.yff, 1.85);
  assert.equal(gpExt.ykp, 18);
  assert.equal(gpExt.yaw_cap, 920);
});
