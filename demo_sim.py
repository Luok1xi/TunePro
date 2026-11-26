# SPDX-License-Identifier: GPL-3.0-or-later
"""Zero-dependency offline virtual TC264 dual-topology car simulator (MOCK0).

Supports switching live between two control architectures:
  - topo = 1 : Two-Series-One-Parallel ("两串一并" 2S1P, default)
               Series  : Outer vision PD -> Target Yaw Rate -> 500 Hz Direct-PWM Gyro Inner Loop
               Parallel: 500 Hz Average-Speed FF+PI -> Base PWM mixed asymmetrically (+0.72/-1.28)
                         with steering-priority ±8500 saturation transfer
  - topo = 0 : Classic 3-Cascade ("传统三串")
               Outer vision PD -> Yaw Rate PI -> L/R Speed Diff -> Independent L/R Wheel PI

Implements the full TunePro serial protocol with a real-time 2 ms physics loop:
  - Live hot-tuning (#W/#D), continuous run & target switching (#X RUN/#X TGT),
    timed step / multi-step stair / alternating yaw (#X GO), urgent E-stop (#!),
    watchdog timeout, 50 Hz @M2 telemetry, A5 5B sliced RLE/GRAY4/GRAY8 camera
    stream (#X IMG), legacy #B/#I frame grabs, and 4 ms onboard step capture (#X CAP).
"""
from __future__ import annotations

import math
import queue
import random
import re
import struct
import threading
import time

PWM_LIMIT = 8500
IMG_W = 188
IMG_H = 120


def _build_crc16_table() -> tuple[int, ...]:
    tbl = []
    for i in range(256):
        c = i << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
        tbl.append(c)
    return tuple(tbl)


_CRC16_TABLE = _build_crc16_table()


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF; check('123456789') == 0x29B1)."""
    tbl = _CRC16_TABLE
    for ch in data:
        crc = ((crc << 8) & 0xFFFF) ^ tbl[((crc >> 8) ^ ch) & 0xFF]
    return crc


def sign_line(body: str) -> bytes:
    raw = body.encode('ascii')
    return f'{body}*{crc16(raw):04X}\r\n'.encode('ascii')


def verify_signed(line: str) -> str | None:
    star = line.rfind('*')
    if star < 1 or len(line) - star != 5:
        return None
    hex_part = line[star + 1:]
    if not re.fullmatch(r'[0-9A-Fa-f]{4}', hex_part):
        return None
    body = line[:star]
    if crc16(body.encode('ascii')) != int(hex_part, 16):
        return None
    return body


def _u8(v: int) -> int:
    return int(round(v)) & 0xFF


def _u16(v: int) -> int:
    return int(round(v)) & 0xFFFF


def _u32(v: int) -> int:
    return int(round(v)) & 0xFFFFFFFF


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def _emit_rle_run(out: bytearray, is_white: bool, count: int) -> None:
    hi = 0x80 if is_white else 0
    while count > 127:
        out.append(hi | 127)
        count -= 127
    if count > 0:
        out.append(hi | count)


def _build_track_rle_fast(error_px: float, phase_t: float) -> tuple[bytes, int]:
    """Direct row-span RLE encoder for 188x120 binary track (~180x faster than pixel-by-pixel)."""
    curve = math.sin(phase_t * 0.9) * 18.0
    out = bytearray()
    white_cnt = 0
    pending_white = False
    pending_run = 0

    def push(is_white: bool, cnt: int) -> None:
        nonlocal pending_white, pending_run
        if cnt <= 0:
            return
        if pending_run == 0:
            pending_white = is_white
            pending_run = cnt
        elif pending_white == is_white:
            pending_run += cnt
        else:
            _emit_rle_run(out, pending_white, pending_run)
            pending_white = is_white
            pending_run = cnt

    for y in range(IMG_H):
        ny = (y - 60) / 60.0
        cx = IMG_W * 0.5 + error_px + curve * (ny * ny - 0.35)
        half_w = 13.0 + (y / IMG_H) * 16.0
        x0 = max(0, int(cx - half_w))
        x1 = min(IMG_W, int(cx + half_w))
        if x1 > x0:
            push(False, x0)
            push(True, x1 - x0)
            push(False, IMG_W - x1)
            white_cnt += x1 - x0
        else:
            push(False, IMG_W)
    if pending_run:
        _emit_rle_run(out, pending_white, pending_run)
    avg_gray = int(38 + (white_cnt / (IMG_W * IMG_H)) * 175)
    return bytes(out), avg_gray


def _rle_encode_binary(pixels: bytes) -> bytes:
    out = bytearray()
    n = len(pixels)
    i = 0
    while i < n:
        cur = 1 if pixels[i] else 0
        run = 1
        i += 1
        while i < n and run < 127 and (1 if pixels[i] else 0) == cur:
            run += 1
            i += 1
        out.append((0x80 if cur else 0) | run)
    return bytes(out)


def _build_track_image(error_px: float, mode: int, phase_t: float, binary: bool) -> tuple[bytes, int]:
    """Generate a synthetic 188x120 racing track camera frame (white tape on dark floor)."""
    curve = math.sin(phase_t * 0.9) * 18.0
    if mode == 1:
        buf = bytearray(IMG_W * IMG_H)
        white_cnt = 0
        for y in range(IMG_H):
            ny = (y - 60) / 60.0
            cx = IMG_W * 0.5 + error_px + curve * (ny * ny - 0.35)
            half_w = 13.0 + (y / IMG_H) * 16.0
            x0 = max(0, int(cx - half_w))
            x1 = min(IMG_W, int(cx + half_w))
            if x1 > x0:
                row_off = y * IMG_W
                buf[row_off + x0:row_off + x1] = b'\xff' * (x1 - x0)
                white_cnt += x1 - x0
        avg_gray = int(38 + (white_cnt / (IMG_W * IMG_H)) * 175)
        return bytes(buf), avg_gray

    if mode == 2:
        w, h = 94, 60
        packed = bytearray(w * h // 2)
        total = 0
        for y in range(h):
            ny = (y - 30) / 30.0
            cx = (w * 0.5) + error_px * 0.5 + curve * 0.5 * (ny * ny - 0.35)
            half_w = 6.5 + (y / h) * 8.0
            row_off = y * (w // 2)
            for pair in range(w // 2):
                x0 = pair * 2
                x1 = x0 + 1
                g0 = 14 if abs(x0 - cx) <= half_w else 3
                g1 = 14 if abs(x1 - cx) <= half_w else 3
                packed[row_off + pair] = (g0 << 4) | g1
                total += (g0 + g1) * 17
        return bytes(packed), total // (w * h)

    if mode == 3:
        w, h = 47, 30
        out = bytearray(w * h)
        total = 0
        for y in range(h):
            ny = (y - 15) / 15.0
            cx = (w * 0.5) + error_px * 0.25 + curve * 0.25 * (ny * ny - 0.35)
            half_w = 3.2 + (y / h) * 4.0
            row_off = y * w
            for x in range(w):
                d = abs(x - cx)
                val = 218 if d <= half_w else (120 if d <= half_w + 1.0 else 44)
                out[row_off + x] = val
                total += val
        return bytes(out), total // (w * h)

    out = bytearray(IMG_W * IMG_H)
    total = 0
    for y in range(IMG_H):
        ny = (y - 60) / 60.0
        cx = IMG_W * 0.5 + error_px + curve * (ny * ny - 0.35)
        half_w = 13.0 + (y / IMG_H) * 16.0
        row_off = y * IMG_W
        for x in range(IMG_W):
            d = abs(x - cx)
            if binary:
                val = 255 if d <= half_w else 0
            else:
                val = 215 if d <= half_w else (130 if d <= half_w + 2.0 else 42)
            out[row_off + x] = val
            total += val
    return bytes(out), total // (IMG_W * IMG_H)


def _pack_slice_packets(
    fmt: int,
    payload: bytes,
    seq: int,
    stamp_ms: int,
    error_px: int,
    avg_gray: int,
    route_k: int,
    lp: int,
    rp: int,
    running: bool,
) -> bytes:
    chunk_size = 240
    total_len = len(payload)
    slice_total = max(1, (total_len + chunk_size - 1) // chunk_size)
    flags = 1 | (2 if running else 0)
    out = bytearray()
    for idx in range(slice_total):
        part = payload[idx * chunk_size:(idx + 1) * chunk_size]
        slen = len(part)
        header = struct.pack(
            '<BBBBBBHHIIhBBHH',
            0xA5, 0x5B, fmt, flags, idx, slice_total,
            slen, total_len, seq & 0xFFFFFFFF, stamp_ms & 0xFFFFFFFF,
            int(_clamp(error_px, -32768, 32767)),
            avg_gray & 0xFF, route_k & 0xFF,
            lp & 0xFFFF, rp & 0xFFFF,
        )
        body_and_payload = header[2:] + part
        pkt_crc = crc16(body_and_payload)
        out.extend(header)
        out.extend(part)
        out.extend(struct.pack('<H', pkt_crc))
    return bytes(out)


class MockCarSerial:
    """In-process virtual serial device implementing TC264 TunePro (2S1P + 3-Cascade)."""

    def __init__(self, port: str = 'MOCK0', baud: int = 115200, *, realtime: bool = True, topo: int = 1):
        self.port = port.upper()
        self.baud = int(baud)
        self._realtime = realtime
        self._rx_queue: queue.Queue[bytes] = queue.Queue(maxsize=256)
        self._lock = threading.RLock()
        self._closed = False
        self._rx_buf = bytearray()
        self._rng = random.Random(2026)

        # Control topology: 1 = Two-Series-One-Parallel (两串一并), 0 = Classic 3-Cascade (三串)
        self.topo = 1 if topo else 0

        # Firmware configuration snapshot (@GP1)
        self.rev = 1
        self.last_rid = 0
        self.mode = 3 if self.topo == 1 else 1   # default: both wheels for 2S1P, left wheel for 3-cascade
        self.wheel = 2 if self.topo == 1 else 0  # 0=L, 1=R, 2=B
        self.step_mode = 3                       # 0=single, 1=stair, 2=yaw-alt, 3=continuous
        self.race_speed = 125
        self.test_speed = 140
        self.test_low = 90
        self.test_yaw = 300
        self.fan_duty = 4000                     # 0..8000 (40%)
        self.tune_fan = 0
        self.turn_speed = 80
        self.dirp_cur = 8.5 if self.topo == 1 else 7.0
        self.dird_cur = 6.0
        self.img_mode = 0
        self.img_fps = 6
        self.dur_ms = 2000
        self._load_topology_defaults(self.topo)

        # Runtime & physical plant state
        self.running = False
        self.stop_reason = 0
        self.run_id = 0
        self.seq = 0
        self.frame_seq = 0
        self.stamp_ms = 0
        self.run_elapsed_ms = 0
        self.last_ping_s = time.monotonic()

        self.enc_l = 0.0
        self.enc_r = 0.0
        self.set_l = 0.0
        self.set_r = 0.0
        self.pwm_l = 0.0
        self.pwm_r = 0.0
        self.err_l_prev = 0.0
        self.err_r_prev = 0.0
        self.avg_int = 0.0
        self.yaw_int = 0.0

        self.gyro_dps = 0.0
        self.yaw_target_dps = 0.0
        self.yaw_out = 0.0
        self.err_y_prev = 0.0
        self.heading_deg = 0.0
        self.heading_target_deg = 0.0

        self.image_error = 0.0
        self.image_error_prev = 0.0
        self.route_k = 0
        self.vbat_mv = 12280
        self.enc_total_l = 0
        self.enc_total_r = 0
        self._active_target = 0.0
        self._phase = 0

        # Onboard 4 ms high-res capture buffer (#X CAP)
        self._capture_profile = 'step-stair'
        self._capture_side = 1
        self._capture_rows: list[tuple[int, int, int, int, int, int, int, int, int, int]] = []
        self._capture_meta: dict[str, int | str] = {}

        self._stop_ev = threading.Event()
        self._thread: threading.Thread | None = None

        self._emit(b'ID fw=TUNEPRO-01-DEMO hex=offline-sim tlm=m2 bb=0 params=pro\r\n')
        self._emit(self._format_gp1(0))
        if self._realtime:
            self._thread = threading.Thread(target=self._sim_loop, name=f'mock-{self.port}', daemon=True)
            self._thread.start()

    def _load_topology_defaults(self, topo: int) -> None:
        if topo == 1:
            # Two-Series-One-Parallel (两串一并) defaults aligned with direct-PWM yaw inner loop
            self.lkp = 38.0
            self.lki = 1.20
            self.rkp = 38.0
            self.rki = 1.20
            self.ykp = 18.0    # PWM / (deg/s)
            self.yki = 0.12    # PWM integral (spin test)
            self.yff = 1.85    # PWM / (deg/s) kinematic feedforward
            self.yaw_cap = 920
        else:
            # Classic 3-Cascade defaults
            self.lkp = 15.0
            self.lki = 1.50
            self.rkp = 15.5
            self.rki = 1.52
            self.ykp = 0.22    # speed-cnt / (deg/s)
            self.yki = 0.011
            self.yff = 0.13
            self.yaw_cap = 400

    def _emit(self, data: bytes) -> None:
        if self._closed or not data:
            return
        try:
            self._rx_queue.put_nowait(data)
        except queue.Full:
            try:
                self._rx_queue.get_nowait()
                self._rx_queue.put_nowait(data)
            except queue.Empty:
                pass

    def read(self, n: int = 8192) -> bytes:
        if self._closed:
            raise OSError('closed')
        try:
            chunk = self._rx_queue.get(timeout=0.03)
        except queue.Empty:
            return b''
        buf = bytearray(chunk)
        while len(buf) < n:
            try:
                buf.extend(self._rx_queue.get_nowait())
            except queue.Empty:
                break
        return bytes(buf)

    def write(self, data: bytes) -> int:
        if self._closed:
            raise OSError('closed')
        if not data:
            return 0
        with self._lock:
            i = 0
            raw = bytes(data)
            while i < len(raw):
                if raw[i:i + 2] == b'#!':
                    self._handle_estop(25)
                    i += 2
                    continue
                if self._rx_buf.endswith(b'#') and raw[i:i + 1] == b'!':
                    self._rx_buf.pop()
                    self._handle_estop(25)
                    i += 1
                    continue
                b = raw[i]
                i += 1
                if b in (0x0A, 0x0D):
                    if self._rx_buf:
                        line = bytes(self._rx_buf).decode('latin1', errors='ignore').strip()
                        self._rx_buf.clear()
                        if line:
                            self._handle_line(line)
                else:
                    self._rx_buf.append(b)
                    if self._rx_buf in (b'#B', b'#I'):
                        cmd = bytes(self._rx_buf).decode('ascii')
                        self._rx_buf.clear()
                        self._handle_legacy_grab(cmd)
        return len(data)

    def close(self) -> None:
        self._closed = True
        self._stop_ev.set()
        if self._thread is not None and self._thread.is_alive() and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)

    def _format_gp1(self, rid: int) -> bytes:
        fields = [
            '@GP1',
            rid, self.rev, self.mode, self.wheel, self.step_mode,
            self.race_speed, self.test_speed, self.test_low, self.test_yaw,
            self.fan_duty, self.tune_fan,
            round(self.lkp * 100), round(self.lki * 1000),
            round(self.rkp * 100), round(self.rki * 1000),
            round(self.ykp * 10000), round(self.yki * 10000),
            self.turn_speed, self.yaw_cap,
            round(self.dirp_cur * 1000), round(self.dird_cur * 1000),
            self.img_mode, self.img_fps, self.dur_ms,
            self.topo, round(self.yff * 1000),
        ]
        return sign_line(','.join(str(x) for x in fields))

    def _emit_error(self, rid: int, code: str) -> None:
        self._emit(sign_line(f'@QX1,{rid},{self.rev},{code},{self.last_rid}'))

    def _reset_controllers(self) -> None:
        self.pwm_l = 0.0
        self.pwm_r = 0.0
        self.err_l_prev = 0.0
        self.err_r_prev = 0.0
        self.avg_int = 0.0
        self.yaw_int = 0.0
        self.yaw_out = 0.0
        self.err_y_prev = 0.0
        self.set_l = 0.0
        self.set_r = 0.0
        self.yaw_target_dps = 0.0

    def _handle_estop(self, reason: int) -> None:
        self.running = False
        self.stop_reason = reason
        self._reset_controllers()
        self._emit(sign_line(f'@RA1,STOP,0,{reason},0'))
        self._emit(self._format_m2())

    def _start_capture_buffer(self) -> None:
        if self.mode == 6:
            profile = 'step-stair'
            side = self.wheel + 1
        elif self.mode == 7:
            profile = 'yaw-alt'
            side = 4
        elif self.step_mode == 3:
            profile = 'live-hold'
            side = 4 if self.mode in (4, 5) else max(1, min(3, self.mode))
        else:
            profile = 'zero-high-zero'
            side = 4 if self.mode == 4 else (5 if self.mode == 5 else max(1, min(3, self.mode)))
        self._capture_profile = profile
        self._capture_side = side
        self._capture_rows = []
        self._capture_meta = {
            'side': side,
            'fan_pct': round((self.fan_duty if (self.mode == 0 or self.tune_fan) else 0) / 100),
            'kpL': int(_clamp(round(self.lkp * 10), 0, 2000)),
            'kiL': int(_clamp(round(self.lki * 100), 0, 2000)),
            'kpR': int(_clamp(round(self.rkp * 10), 0, 2000)),
            'kiR': int(_clamp(round(self.rki * 100), 0, 2000)),
            'akp': int(_clamp(round(self.ykp * (10 if self.topo == 1 else 1000)), 0, 2000)),
            'aki': int(_clamp(round(self.yki * (100 if self.topo == 1 else 1000)), 0, 2000)),
            'profile': profile,
        }

    def _handle_legacy_grab(self, cmd: str) -> None:
        binary = cmd == '#B'
        pixels, _ = _build_track_image(self.image_error, 0, self.stamp_ms / 1000.0, binary=binary)
        self.frame_seq += 1
        kind = 'bin' if binary else 'gray'
        hdr = f'\r\nIMG {kind} {IMG_W} {IMG_H} {self.frame_seq}\r\n'.encode('ascii')
        self._emit(hdr + pixels + b'\r\nEND\r\n')

    def _handle_line(self, line: str) -> None:
        if line in ('#B', '#I'):
            self._handle_legacy_grab(line)
            return
        if not line.startswith('#'):
            return
        body = verify_signed(line)
        if body is None:
            self._emit_error(0, 'CRC')
            return

        parts = body.split()
        if not parts:
            return
        cmd = parts[0]

        if cmd == '#M' and len(parts) == 2:
            rid = int(parts[1])
            self.last_rid = rid
            self._emit(self._format_gp1(rid))
            return

        if cmd == '#W' and len(parts) == 5:
            rid, rev = int(parts[1]), int(parts[2])
            key, val = parts[3], int(parts[4])
            if rev != self.rev:
                self._emit_error(rid, 'REVISION')
                self._emit(self._format_gp1(rid))
                return
            if not self._apply_write(rid, key, val):
                return
            self.rev += 1
            self.last_rid = rid
            self._emit(sign_line(f'@WA1,{rid},{self.rev},{key},{val}'))
            self._emit(self._format_gp1(rid))
            return

        if cmd == '#D' and len(parts) == 7:
            rid, rev = int(parts[1]), int(parts[2])
            kind, val = parts[3], int(parts[6])
            if rev != self.rev:
                self._emit_error(rid, 'REVISION')
                self._emit(self._format_gp1(rid))
                return
            key = 'dirp_cur' if kind == 'p' else 'dird_cur'
            if kind == 'p':
                self.dirp_cur = _clamp(val / 1000.0, 0.0, 100.0)
            else:
                self.dird_cur = _clamp(val / 1000.0, 0.0, 500.0)
            self.rev += 1
            self.last_rid = rid
            self._emit(sign_line(f'@WA1,{rid},{self.rev},{key},{val}'))
            self._emit(self._format_gp1(rid))
            return

        if cmd == '#X' and len(parts) >= 2:
            sub = parts[1]
            if sub == 'STOP':
                self._handle_estop(25)
                return
            if sub == 'PING':
                self.last_ping_s = time.monotonic()
                return
            if sub == 'IMG' and len(parts) == 4:
                self.img_mode = int(_clamp(int(parts[2]), 0, 3))
                raw_fps = int(parts[3])
                self.img_fps = 0 if raw_fps <= 0 else int(_clamp(raw_fps, 1, 120))
                self._emit(sign_line(f'@RA1,IMG,0,{self.img_mode},{self.img_fps}'))
                return
            if sub == 'CAP':
                self._emit_capture()
                return
            if sub == 'GO' and len(parts) == 5:
                rid, rev = int(parts[2]), int(parts[3])
                if rev != self.rev:
                    self._emit_error(rid, 'REVISION')
                    self._emit(self._format_gp1(rid))
                    return
                if self.running:
                    self._emit_error(rid, 'MOVING')
                    return
                self._start_go(rid)
                return
            if sub == 'RUN' and len(parts) == 4:
                kind = parts[2].upper()
                target = int(parts[3])
                self._handle_run(kind, target)
                return
            if sub == 'TGT' and len(parts) == 3:
                target = int(parts[2])
                self._active_target = float(target)
                if self.mode in (4, 5):
                    self.test_yaw = abs(target)
                    self.mode = 5 if target < 0 else 4
                else:
                    self.test_speed = abs(target)
                self._emit(sign_line(f'@RA1,TGT,0,{target},0'))
                return

        self._emit_error(0, 'FORMAT')

    def _apply_write(self, rid: int, key: str, val: int) -> bool:
        if key == 'topo':
            if self.running:
                self._emit_error(rid, 'MOVING')
                return False
            self.topo = 1 if val else 0
            self._load_topology_defaults(self.topo)
            if self.topo == 1 and self.mode in (1, 2):
                self.mode = 3
                self.wheel = 2
            return True
        if key == 'mode':
            if self.running:
                self._emit_error(rid, 'MOVING')
                return False
            self.mode = int(_clamp(val, 0, 7))
            if self.mode == 6:
                self.step_mode = 1
            elif self.mode == 7:
                self.step_mode = 2
        elif key == 'wheel':
            self.wheel = int(_clamp(val, 0, 2))
            if self.mode in (1, 2, 3):
                self.mode = self.wheel + 1
        elif key == 'step_mode':
            if self.running:
                self._emit_error(rid, 'MOVING')
                return False
            self.step_mode = int(_clamp(val, 0, 3))
            if self.step_mode == 1:
                self.mode = 6
            elif self.step_mode == 2:
                self.mode = 7
            elif self.mode == 6:
                self.mode = self.wheel + 1
            elif self.mode == 7:
                self.mode = 4
        elif key == 'race_speed':
            self.race_speed = int(_clamp(val, 0, 400))
        elif key == 'test_speed':
            self.test_speed = int(_clamp(val, 0, 350))
            if self.running and self.mode in (1, 2, 3):
                self._active_target = float(self.test_speed)
        elif key == 'test_low':
            self.test_low = int(_clamp(val, 0, 350))
        elif key == 'test_yaw':
            self.test_yaw = int(_clamp(abs(val), 0, 600))
        elif key == 'fan':
            self.fan_duty = int(_clamp(val, 0, 8000))
        elif key == 'tune_fan':
            self.tune_fan = 1 if val else 0
        elif key == 'lkp':
            self.lkp = _clamp(val / 100.0, 0.0, 200.0)
        elif key == 'lki':
            self.lki = _clamp(val / 1000.0, 0.0, 20.0)
        elif key == 'rkp':
            self.rkp = _clamp(val / 100.0, 0.0, 200.0)
        elif key == 'rki':
            self.rki = _clamp(val / 1000.0, 0.0, 20.0)
        elif key == 'ykp':
            self.ykp = _clamp(val / 10000.0, 0.0, 60.0)
        elif key == 'yki':
            self.yki = _clamp(val / 10000.0, 0.0, 10.0)
        elif key == 'yff':
            self.yff = _clamp(val / 1000.0, 0.0, 10.0)
        elif key == 'turn_speed':
            self.turn_speed = int(_clamp(val, 0, 400))
        elif key == 'yaw_cap':
            self.yaw_cap = int(_clamp(val, 1, 1500))
        elif key == 'dirp_cur':
            self.dirp_cur = _clamp(val / 1000.0, 0.0, 100.0)
        elif key == 'dird_cur':
            self.dird_cur = _clamp(val / 1000.0, 0.0, 500.0)
        else:
            self._emit_error(rid, 'KEY')
            return False
        return True

    def _start_go(self, rid: int) -> None:
        self.run_id += 1
        self.running = True
        self.stop_reason = 0
        self.run_elapsed_ms = 0
        self.last_ping_s = time.monotonic()
        self._reset_controllers()
        if self.mode == 0:
            tgt = self.race_speed
        elif self.mode in (4, 5, 7):
            tgt = -self.test_yaw if self.mode == 5 else self.test_yaw
        else:
            tgt = self.test_speed
        self._active_target = float(tgt)
        self._start_capture_buffer()
        self._emit(sign_line(f'@RA1,GO,{rid},{self.mode},{tgt}'))

    def _handle_run(self, kind: str, target: int) -> None:
        kind_map = {'L': 1, 'R': 2, 'B': 3, 'W': 4}
        if kind not in kind_map:
            self._emit_error(0, 'FORMAT')
            return
        k_idx = kind_map[kind]
        want_yaw = kind == 'W'
        if self.running:
            in_yaw = self.mode in (4, 5, 7)
            in_spd = self.mode in (1, 2, 3, 6)
            if (want_yaw and not in_yaw) or (not want_yaw and not in_spd):
                self._emit_error(0, 'MOVING')
                return
        if want_yaw:
            self.mode = 5 if target < 0 else 4
            self.test_yaw = abs(target)
        else:
            self.wheel = k_idx - 1
            self.mode = k_idx
            self.test_speed = abs(target)
        self.step_mode = 3
        self._active_target = float(target)
        self.last_ping_s = time.monotonic()
        if not self.running and target != 0:
            self.run_id += 1
            self.running = True
            self.stop_reason = 0
            self.run_elapsed_ms = 0
            self._reset_controllers()
            self._start_capture_buffer()
        self.rev += 1
        self._emit(sign_line(f'@RA1,RUN,{k_idx},{target},{1 if self.running else 0}'))
        self._emit(self._format_gp1(0))

    def _emit_capture(self) -> None:
        if not self._capture_rows:
            self._emit_error(0, 'NO_CAPTURE')
            return
        rows = self._capture_rows[-1000:]
        m = self._capture_meta
        hdr = (
            f'[tune2] cap begin n={len(rows)} dt_ms=4 stride=1 side={m["side"]} '
            f'fan_pct={m["fan_pct"]} kpL={m["kpL"]} kiL={m["kiL"]} kpR={m["kpR"]} kiR={m["kiR"]} '
            f'akp={m["akp"]} aki={m["aki"]} profile={m["profile"]} build=TUNEPRO-01\r\n'
        )
        lines = [hdr]
        for idx, row in enumerate(rows):
            _, target, enc_l, enc_r, pwm_l, pwm_r, gyro10, set_l, set_r, phase = row
            lines.append(f'u2,{idx},{target},{enc_l},{enc_r},{pwm_l},{pwm_r},{gyro10},{set_l},{set_r},{phase}\r\n')
        lines.append('[tune2] cap end\r\n')
        self._emit(''.join(lines).encode('ascii'))

    def step_2ms(self, now_s: float | None = None) -> None:
        """Advance the simulated plant by one 2 ms control tick."""
        now = time.monotonic() if now_s is None else now_s
        with self._lock:
            self.stamp_ms += 2
            if self.running:
                if self._realtime and (now - self.last_ping_s > 1.5):
                    self._handle_estop(30)
                    return
                self.run_elapsed_ms += 2
                self._step_targets_and_control()
            else:
                self.set_l = 0.0
                self.set_r = 0.0
                self.yaw_target_dps = 0.0
                self.pwm_l = 0.0
                self.pwm_r = 0.0
                self.enc_l *= 0.90
                self.enc_r *= 0.90
                self.gyro_dps *= 0.88
                if abs(self.enc_l) < 0.2:
                    self.enc_l = 0.0
                if abs(self.enc_r) < 0.2:
                    self.enc_r = 0.0
                if abs(self.gyro_dps) < 0.2:
                    self.gyro_dps = 0.0
                self.image_error = 14.0 * math.sin(self.stamp_ms / 1400.0)

            fan_active = self.fan_duty if (self.running and (self.mode == 0 or self.tune_fan)) else 0
            load_frac = (abs(self.pwm_l) + abs(self.pwm_r)) / (2.0 * PWM_LIMIT)
            self.vbat_mv = int(12280 - fan_active * 0.045 - load_frac * 260 + self._rng.uniform(-6, 6))
            self.enc_total_l += int(round(self.enc_l))
            self.enc_total_r += int(round(self.enc_r))

            if self.running and (self.run_elapsed_ms % 4 == 0):
                tgt_int = int(round(self.yaw_target_dps if self.mode in (4, 5, 7) else self._active_target))
                row = (
                    len(self._capture_rows),
                    tgt_int,
                    int(round(self.enc_l)),
                    int(round(self.enc_r)),
                    int(round(self.pwm_l)),
                    int(round(self.pwm_r)),
                    int(round(self.gyro_dps * 10.0)),
                    int(round(self.set_l)),
                    int(round(self.set_r)),
                    self._phase,
                )
                self._capture_rows.append(row)
                if len(self._capture_rows) > 1000:
                    self._capture_rows.pop(0)

    def _step_targets_and_control(self) -> None:
        t = self.run_elapsed_ms
        base_speed = 0.0
        use_yaw = False
        single_wheel = 0  # 0=normal, 1=L-only, 2=R-only

        if self.mode == 6:
            if t < 300:
                tgt, self._phase = 0.0, 0
            elif t < 1200:
                tgt, self._phase = float(self.test_low), 1
            elif t < 2100:
                tgt, self._phase = float(self.test_speed), 2
            elif t < 2800:
                tgt, self._phase = float(self.test_low), 1
            elif t < 3200:
                tgt, self._phase = 0.0, 0
            else:
                self._handle_estop(20)
                return
            self._active_target = tgt
            base_speed = tgt
            if self.wheel == 0:
                single_wheel = 1
            elif self.wheel == 1:
                single_wheel = 2
            self.yaw_target_dps = 0.0
        elif self.mode == 7:
            if t < 300:
                w, self._phase = 0.0, 0
            elif t < 1200:
                w, self._phase = float(self.test_yaw), 1
            elif t < 2100:
                w, self._phase = -float(self.test_yaw), 2
            elif t < 2800:
                w, self._phase = float(self.test_yaw), 1
            elif t < 3200:
                w, self._phase = 0.0, 0
            else:
                self._handle_estop(20)
                return
            self._active_target = w
            self.yaw_target_dps = w
            use_yaw = True
        elif self.mode in (1, 2, 3):
            if self.step_mode == 0:
                if t < 500:
                    tgt, self._phase = 0.0, 0
                elif t < 1500:
                    tgt, self._phase = float(self.test_speed), 1
                elif t < 2000:
                    tgt, self._phase = 0.0, 2
                else:
                    self._handle_estop(20)
                    return
                self._active_target = tgt
            else:
                tgt = self._active_target
                self._phase = 1
            base_speed = tgt
            if self.mode == 1:
                single_wheel = 1
            elif self.mode == 2:
                single_wheel = 2
            self.yaw_target_dps = 0.0
        elif self.mode in (4, 5):
            if self.step_mode == 0:
                base_w = -float(self.test_yaw) if self.mode == 5 else float(self.test_yaw)
                if t < 500:
                    w, self._phase = 0.0, 0
                elif t < 1500:
                    w, self._phase = base_w, 1
                elif t < 2000:
                    w, self._phase = 0.0, 2
                else:
                    self._handle_estop(20)
                    return
                self._active_target = w
            else:
                w = self._active_target
                self._phase = 1
            self.yaw_target_dps = w
            use_yaw = True
        else:
            # Outer vision tracking mode (mode == 0)
            self._phase = 1
            curv_disturbance = 30.0 * math.sin(t / 900.0) + 14.0 * math.sin(t / 340.0)
            self.route_k = (t // 1500) % 48
            err = _clamp(curv_disturbance - self.gyro_dps * 0.055, -85.0, 85.0)
            d_err = err - self.image_error_prev
            self.image_error_prev = err
            self.image_error = err
            yaw_cmd = _clamp(self.dirp_cur * err + self.dird_cur * d_err, -self.yaw_cap, self.yaw_cap)
            self.yaw_target_dps = yaw_cmd
            base_speed = float(self.turn_speed if abs(err) > 22.0 else self.race_speed)
            self._active_target = base_speed
            use_yaw = True

        # Topology-specific inner control law
        if self.topo == 1 and single_wheel == 0:
            self._step_2s1p_control(base_speed, use_yaw)
        else:
            if use_yaw:
                self._compute_yaw_diff_cascade(base_speed)
            else:
                self.set_l = base_speed if single_wheel in (0, 1) else 0.0
                self.set_r = base_speed if single_wheel in (0, 2) else 0.0
            self._step_wheel_pi_cascade(single_wheel)

        # Electromechanical wheel & yaw plant dynamics (2 ms step)
        tau_l, tau_r = 0.031, 0.030
        dt = 0.002
        gain_l, gain_r = 0.042, 0.0408
        eff_l = math.copysign(max(0.0, abs(self.pwm_l) - 180.0), self.pwm_l) * gain_l
        eff_r = math.copysign(max(0.0, abs(self.pwm_r) - 180.0), self.pwm_r) * gain_r
        self.enc_l += (dt / tau_l) * (eff_l - self.enc_l) + self._rng.uniform(-0.35, 0.35)
        self.enc_r += (dt / tau_r) * (eff_r - self.enc_r) + self._rng.uniform(-0.35, 0.35)

        # Differential yaw rate responds faster directly to differential torque in 2S1P
        yaw_tau = 0.011 if self.topo == 1 else 0.022
        target_gyro = (self.enc_r - self.enc_l) * 3.15
        self.gyro_dps += (dt / yaw_tau) * (target_gyro - self.gyro_dps) + self._rng.uniform(-1.1, 1.1)
        self.heading_deg += self.gyro_dps * dt
        self.heading_target_deg += self.yaw_target_dps * dt

    def _step_2s1p_control(self, base_speed: float, use_yaw: bool) -> None:
        """Two-Series-One-Parallel ("两串一并") control law:
        Parallel average-speed loop -> PWM_speed
        Series direct-PWM gyro loop -> PWM_turn
        Asymmetric mixing (+0.72 outer / -1.28 inner) + steering-priority ±8500 saturation transfer.
        """
        v_avg = 0.5 * (self.enc_l + self.enc_r)
        # Turning-coupled speed dip above 360 dps
        eff_speed = base_speed
        if use_yaw and base_speed > 0 and abs(self.yaw_target_dps) > 360.0:
            dip = min(base_speed * 0.58, (abs(self.yaw_target_dps) - 360.0) * 0.20)
            eff_speed = max(0.0, base_speed - dip)

        self.set_l = eff_speed
        self.set_r = eff_speed

        # 1) Parallel average-speed positional FF + PI
        err_v = eff_speed - v_avg
        kp_v = 0.5 * (self.lkp + self.rkp)
        ki_v = 0.5 * (self.lki + self.rki)
        pwm_speed_unsat = 6.0 * eff_speed + kp_v * err_v + self.avg_int
        if abs(pwm_speed_unsat) < PWM_LIMIT or (pwm_speed_unsat * err_v < 0):
            self.avg_int = _clamp(self.avg_int + ki_v * err_v, -PWM_LIMIT, PWM_LIMIT)
        pwm_speed = _clamp(6.0 * eff_speed + kp_v * err_v + self.avg_int, -PWM_LIMIT, PWM_LIMIT)

        # 2) Series direct-PWM gyro inner loop (bypasses wheel-speed lag!)
        if use_yaw:
            err_w = self.yaw_target_dps - self.gyro_dps
            ki_w = 0.0 if self.mode == 0 else self.yki
            pwm_turn_raw = self.yff * self.yaw_target_dps + self.ykp * err_w + self.yaw_int
            if abs(pwm_turn_raw) < 8200.0 or (pwm_turn_raw * err_w < 0):
                self.yaw_int = _clamp(self.yaw_int + ki_w * err_w, -4000.0, 4000.0)
            pwm_turn = _clamp(self.yff * self.yaw_target_dps + self.ykp * err_w + self.yaw_int, -8200.0, 8200.0)
        else:
            pwm_turn = 0.0
            self.yaw_int = 0.0

        # 3) Asymmetric mixing (+0.72 outer / -1.28 inner) with steering-priority saturation transfer
        if abs(base_speed) < 1.0:
            raw_l = -pwm_turn
            raw_r = pwm_turn
        elif pwm_turn >= 0:
            raw_r = pwm_speed + 0.72 * pwm_turn
            raw_l = pwm_speed - 1.28 * pwm_turn
        else:
            raw_l = pwm_speed + 0.72 * (-pwm_turn)
            raw_r = pwm_speed - 1.28 * (-pwm_turn)

        if raw_r > PWM_LIMIT:
            ex = raw_r - PWM_LIMIT
            raw_r = PWM_LIMIT
            raw_l -= ex
        elif raw_l > PWM_LIMIT:
            ex = raw_l - PWM_LIMIT
            raw_l = PWM_LIMIT
            raw_r -= ex

        self.pwm_l = _clamp(raw_l, -PWM_LIMIT, PWM_LIMIT)
        self.pwm_r = _clamp(raw_r, -PWM_LIMIT, PWM_LIMIT)

    def _compute_yaw_diff_cascade(self, base_speed: float) -> None:
        err_y = self.yaw_target_dps - self.gyro_dps
        delta_u = self.ykp * (err_y - self.err_y_prev) + self.yki * err_y
        self.err_y_prev = err_y
        self.yaw_out = _clamp(self.yaw_out + delta_u, -240.0, 240.0)
        self.set_l = _clamp(base_speed - self.yaw_out, -350.0, 350.0)
        self.set_r = _clamp(base_speed + self.yaw_out, -350.0, 350.0)

    def _step_wheel_pi_cascade(self, single_wheel: int) -> None:
        if single_wheel in (0, 1):
            err_l = self.set_l - self.enc_l
            self.pwm_l = _clamp(self.pwm_l + self.lkp * (err_l - self.err_l_prev) + self.lki * err_l, -PWM_LIMIT, PWM_LIMIT)
            self.err_l_prev = err_l
        else:
            self.pwm_l = 0.0
            self.err_l_prev = 0.0

        if single_wheel in (0, 2):
            err_r = self.set_r - self.enc_r
            self.pwm_r = _clamp(self.pwm_r + self.rkp * (err_r - self.err_r_prev) + self.rki * err_r, -PWM_LIMIT, PWM_LIMIT)
            self.err_r_prev = err_r
        else:
            self.pwm_r = 0.0
            self.err_r_prev = 0.0

    def _format_m2(self) -> bytes:
        self.seq += 1
        flags = (
            (1 if self.running else 0)
            | ((self._phase & 1) << 1)
            | ((1 if (abs(self.pwm_l) > 1 or abs(self.pwm_r) > 1) else 0) << 2)
            | (1 << 3)
        )
        fan_now = self.fan_duty if (self.running and (self.mode == 0 or self.tune_fan)) else 0
        fields = [
            f'{_u32(self.run_id):08X}',
            f'{_u32(self.seq):08X}',
            f'{_u32(self.stamp_ms):08X}',
            f'{_u8(self.route_k):02X}',
            f'{_u8(0):02X}',
            f'{_u8(0):02X}',
            f'{_u16(flags):04X}',
            f'{_u16(self.image_error):04X}',
            f'{_u16(self.yaw_target_dps * 10):04X}',
            f'{_u16(self.gyro_dps * 10):04X}',
            f'{_u16(self.set_l * 10):04X}',
            f'{_u16(self.enc_l):04X}',
            f'{_u16(self.pwm_l):04X}',
            f'{_u16(self.set_r * 10):04X}',
            f'{_u16(self.enc_r):04X}',
            f'{_u16(self.pwm_r):04X}',
            f'{_u16(fan_now):04X}',
            f'{_u8(self.stop_reason):02X}',
            f'{_u32(self.heading_deg * 10):08X}',
            f'{_u32(self.heading_target_deg * 10):08X}',
            f'{_u16(8):04X}',
            f'{_u32(0):08X}',
            f'{_u32(self.frame_seq):08X}',
            f'{_u32(self.rev):08X}',
        ]
        return sign_line('@M2,' + ','.join(fields))

    def _format_status(self) -> bytes:
        gz_mdps = int(round(self.gyro_dps * 1000))
        return (
            f't={self.stamp_ms} vbat_mv={self.vbat_mv} tune=1 running={1 if self.running else 0} '
            f'stop={self.stop_reason} imu=1 cal=0 gz_mdps={gz_mdps} '
            f'encL={self.enc_total_l} encR={self.enc_total_r}\r\n'
        ).encode('ascii')

    def _emit_compressed_frame(self) -> None:
        if self.img_mode <= 0:
            return
        self.frame_seq += 1
        fmt_map = {1: 1, 2: 3, 3: 4}
        fmt = fmt_map.get(self.img_mode, 1)
        if fmt == 1:
            payload, avg_gray = _build_track_rle_fast(self.image_error, self.stamp_ms / 1000.0)
        else:
            payload, avg_gray = _build_track_image(self.image_error, self.img_mode, self.stamp_ms / 1000.0, binary=False)
        pkt = _pack_slice_packets(
            fmt=fmt,
            payload=payload,
            seq=self.frame_seq,
            stamp_ms=self.stamp_ms,
            error_px=int(round(self.image_error)),
            avg_gray=avg_gray,
            route_k=self.route_k,
            lp=110,
            rp=110,
            running=self.running,
        )
        self._emit(pkt)

    def _sim_loop(self) -> None:
        next_tick = time.monotonic()
        last_status = next_tick
        last_img = next_tick
        sub_tick = 0
        while not self._stop_ev.is_set():
            now = time.monotonic()
            if now < next_tick:
                time.sleep(min(0.0025, next_tick - now))
                continue
            # 10 ms outer tick (5x 2ms physics): keeps @M2 at 50 Hz while letting unlocked camera reach 100 Hz!
            for _ in range(5):
                self.step_2ms(now)
            sub_tick ^= 1
            with self._lock:
                if sub_tick == 0:
                    self._emit(self._format_m2())
                if now - last_status >= 0.25:
                    last_status = now
                    self._emit(self._format_status())
                if self.img_mode > 0:
                    interval = 0.0 if self.img_fps <= 0 else (1.0 / min(120, self.img_fps))
                    if now - last_img >= interval:
                        last_img = now
                        self._emit_compressed_frame()
            next_tick += 0.010
            if next_tick < now - 0.1:
                next_tick = now + 0.010
