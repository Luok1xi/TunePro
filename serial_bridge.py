# SPDX-License-Identifier: GPL-3.0-or-later
"""Zero-dependency local serial-to-SSE bridge for TC264 remote tuning.

Owns one serial port cleanly, fans out raw serial bytes to multiple browser tabs via
Server-Sent Events (/api/serial/stream), supports priority emergency-stop writes,
auto-reconnect on dongle replug, optional @M2 run auto-saving to disk, and an
offline virtual car port (MOCK0) for zero-hardware testing and demos.
"""
from __future__ import annotations

import base64
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path
import queue
import re
import threading
import time

if os.name == 'nt':
    import winreg
else:
    winreg = None

import demo_sim

M2_RE = re.compile(rb'@M2((?:,[0-9A-F]+){24})\*([0-9A-F]{4})')
M2_FIELDS = [
    'run', 'seq', 'stamp', 'route', 'action', 'state', 'flags', 'error', 'yaw', 'gyro',
    'set_l', 'enc_l', 'pwm_l', 'set_r', 'enc_r', 'pwm_r', 'fan', 'stop',
    'heading', 'heading_target', 'age', 'drop', 'frame_seq', 'cfg_rev',
]
DEFAULT_LOG_ROOT = Path(os.environ.get('TUNEPRO_LOG_DIR') or (Path(__file__).resolve().parent / 'runs'))
ALLOWED_BAUDS = {9600, 115200, 230400, 460800, 921600, 2000000, 3000000}


def _crc16(b: bytes) -> int:
    c = 0xFFFF
    for ch in b:
        c ^= ch << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def _s8(v: int) -> int:
    return v - 256 if v >= 128 else v


def _s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def _s32(v: int) -> int:
    return v - 0x100000000 if v >= 0x80000000 else v


def decode_m2_match(m) -> dict | None:
    body = b'@M2' + m.group(1)
    if _crc16(body) != int(m.group(2), 16):
        return None
    vals = [int(v, 16) for v in m.group(1).split(b',')[1:]]
    d = dict(zip(M2_FIELDS, vals))
    flags = d['flags']
    return {
        'run_id': d['run'],
        't': round(d['stamp'] / 1000.0, 3),
        'seq': d['seq'],
        'route': d['route'],
        'action': _s8(d['action']),
        'state': d['state'],
        'flags': f'0x{flags:04X}',
        'launch': flags & 1,
        'phase': (flags >> 1) & 1,
        'pwm_on': (flags >> 2) & 1,
        'valid': (flags >> 3) & 1,
        'held': (flags >> 4) & 1,
        'angle': (flags >> 5) & 1,
        'ra_settle': (flags >> 6) & 1,
        'error': _s16(d['error']),
        'yaw': round(_s16(d['yaw']) / 10.0, 1),
        'gyro': round(_s16(d['gyro']) / 10.0, 1),
        'set_l': round(_s16(d['set_l']) / 10.0, 1),
        'enc_l': _s16(d['enc_l']),
        'pwm_l': _s16(d['pwm_l']),
        'set_r': round(_s16(d['set_r']) / 10.0, 1),
        'enc_r': _s16(d['enc_r']),
        'pwm_r': _s16(d['pwm_r']),
        'fan': d['fan'],
        'stop': d['stop'],
        'heading': round(_s32(d['heading']) / 10.0, 1),
        'heading_target': round(_s32(d['heading_target']) / 10.0, 1),
        'age': d['age'],
        'drop': d['drop'],
        'frame_seq': d['frame_seq'],
    }


def list_serial_ports(*, include_demo: bool = True, demo_first: bool = False) -> list[dict]:
    """Enumerate available serial ports without opening or stalling them."""
    ports: list[dict] = []
    if os.name == 'nt' and winreg is not None:
        try:
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM') as key:
                idx = 0
                while True:
                    try:
                        dev_name, com_name, _ = winreg.EnumValue(key, idx)
                    except OSError:
                        break
                    idx += 1
                    com = str(com_name).upper()
                    dev = str(dev_name)
                    low = dev.lower()
                    hint = 'USB 串口'
                    priority = 2
                    if 'bth' in low or 'bthenum' in low:
                        hint = '蓝牙虚拟串口（通常不是无线数传）'
                        priority = 9
                    elif 'das' in low or 'jds' in low:
                        hint = '下载器 / DAS 调试口'
                        priority = 3
                    elif 'usbser' in low or 'vcp' in low or 'silabser' in low or 'ch34' in low:
                        hint = 'USB 转串口（无线模块 / 调试口）'
                        priority = 1
                    num = int(m.group(1)) if (m := re.search(r'(\d+)$', com)) else 999
                    ports.append({'port': com, 'device': dev, 'hint': hint, 'priority': priority, 'num': num})
        except OSError:
            pass
        ports.sort(key=lambda item: (item['priority'], item['num']))
        result = [{k: p[k] for k in ('port', 'device', 'hint')} for p in ports]
    else:
        result = []
        try:
            import serial.tools.list_ports  # type: ignore[import-not-found]
            for info in serial.tools.list_ports.comports():
                result.append({'port': info.device, 'device': info.device, 'hint': info.description or 'Serial Port'})
        except Exception:
            pass

    if include_demo:
        demo_entry = {
            'port': 'MOCK0',
            'device': 'virtual://mock-tc264',
            'hint': '离线虚拟小车（内置三环物理仿真，免硬件体验）',
        }
        if demo_first or not result:
            result.insert(0, demo_entry)
        else:
            result.append(demo_entry)
    return result


if os.name == 'nt':
    _kernel32 = ctypes.WinDLL('kernel32', use_last_error=True)
    _INVALID_HANDLE_VALUE = wintypes.HANDLE(-1).value
    _GENERIC_READ = 0x80000000
    _GENERIC_WRITE = 0x40000000
    _OPEN_EXISTING = 3
    _PURGE_RXCLEAR = 0x0008
    _PURGE_TXCLEAR = 0x0004

    class _DCB(ctypes.Structure):
        _fields_ = [
            ('DCBlength', wintypes.DWORD),
            ('BaudRate', wintypes.DWORD),
            ('fBits', wintypes.DWORD),
            ('wReserved', wintypes.WORD),
            ('XonLim', wintypes.WORD),
            ('XoffLim', wintypes.WORD),
            ('ByteSize', wintypes.BYTE),
            ('Parity', wintypes.BYTE),
            ('StopBits', wintypes.BYTE),
            ('XonChar', ctypes.c_char),
            ('XoffChar', ctypes.c_char),
            ('ErrorChar', ctypes.c_char),
            ('EofChar', ctypes.c_char),
            ('EvtChar', ctypes.c_char),
            ('wReserved1', wintypes.WORD),
        ]

    class _COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ('ReadIntervalTimeout', wintypes.DWORD),
            ('ReadTotalTimeoutMultiplier', wintypes.DWORD),
            ('ReadTotalTimeoutConstant', wintypes.DWORD),
            ('WriteTotalTimeoutMultiplier', wintypes.DWORD),
            ('WriteTotalTimeoutConstant', wintypes.DWORD),
        ]

    _kernel32.CreateFileW.argtypes = [
        wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, wintypes.LPVOID,
        wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
    ]
    _kernel32.CreateFileW.restype = wintypes.HANDLE
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    _kernel32.CloseHandle.restype = wintypes.BOOL
    _kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.GetCommState.restype = wintypes.BOOL
    _kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.SetCommState.restype = wintypes.BOOL
    _kernel32.SetCommTimeouts.argtypes = [wintypes.HANDLE, ctypes.POINTER(_COMMTIMEOUTS)]
    _kernel32.SetCommTimeouts.restype = wintypes.BOOL
    _kernel32.SetupComm.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.DWORD]
    _kernel32.SetupComm.restype = wintypes.BOOL
    _kernel32.PurgeComm.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    _kernel32.PurgeComm.restype = wintypes.BOOL
    _kernel32.ReadFile.argtypes = [
        wintypes.HANDLE, wintypes.LPVOID, wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID,
    ]
    _kernel32.ReadFile.restype = wintypes.BOOL
    _kernel32.WriteFile.argtypes = [
        wintypes.HANDLE, wintypes.LPCVOID, wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD), wintypes.LPVOID,
    ]
    _kernel32.WriteFile.restype = wintypes.BOOL


class WinSerialPort:
    """Zero-dependency Win32 serial port wrapper."""

    def __init__(self, port: str, baud: int = 115200, timeout_ms: int = 40):
        if os.name != 'nt':
            raise OSError('WinSerialPort 仅支持 Windows；Linux/macOS 请使用 MOCK0 或安装 pyserial。')
        self.port = port.upper()
        self.baud = int(baud)
        h = _kernel32.CreateFileW(
            f'\\\\.\\{self.port}', _GENERIC_READ | _GENERIC_WRITE, 0, None, _OPEN_EXISTING, 0, None
        )
        if h == _INVALID_HANDLE_VALUE:
            err = ctypes.get_last_error()
            if err == 5:
                raise PermissionError(
                    f'{self.port} 正被其他进程占用（例如串口助手或其他后台采集脚本）；'
                    f'请先关闭占用程序后再连本机桥（本机桥自带自动落盘）。'
                )
            if err == 2:
                raise FileNotFoundError(f'找不到串口 {self.port}，请检查无线模块或下载器 USB 是否插好。')
            raise OSError(f'无法打开 {self.port}（Win32 err={err}）。')
        self.h = h
        self._wlock = threading.Lock()
        try:
            _kernel32.SetupComm(h, 65536, 8192)
            dcb = _DCB()
            dcb.DCBlength = ctypes.sizeof(_DCB)
            if not _kernel32.GetCommState(h, ctypes.byref(dcb)):
                raise OSError(f'GetCommState 失败: err={ctypes.get_last_error()}')
            dcb.BaudRate = self.baud
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            dcb.fBits = 0x00000001  # fBinary=1, no flow control
            if not _kernel32.SetCommState(h, ctypes.byref(dcb)):
                raise OSError(f'SetCommState 失败: err={ctypes.get_last_error()}')
            to = _COMMTIMEOUTS(10, 0, timeout_ms, 0, 200)
            if not _kernel32.SetCommTimeouts(h, ctypes.byref(to)):
                raise OSError(f'SetCommTimeouts 失败: err={ctypes.get_last_error()}')
            _kernel32.PurgeComm(h, _PURGE_RXCLEAR | _PURGE_TXCLEAR)
        except Exception:
            self.close()
            raise

    def read(self, n: int = 8192) -> bytes:
        buf = ctypes.create_string_buffer(n)
        got = wintypes.DWORD(0)
        ok = _kernel32.ReadFile(self.h, buf, n, ctypes.byref(got), None)
        if not ok:
            raise OSError(f'ReadFile 失败 ({self.port}): err={ctypes.get_last_error()}')
        return buf.raw[:got.value]

    def write(self, data: bytes) -> int:
        if not data:
            return 0
        with self._wlock:
            written = wintypes.DWORD(0)
            ok = _kernel32.WriteFile(self.h, data, len(data), ctypes.byref(written), None)
            if not ok:
                raise OSError(f'WriteFile 失败 ({self.port}): err={ctypes.get_last_error()}')
            return int(written.value)

    def close(self) -> None:
        if getattr(self, 'h', None) and self.h != _INVALID_HANDLE_VALUE:
            _kernel32.CloseHandle(self.h)
            self.h = _INVALID_HANDLE_VALUE


def open_serial_port(port: str, baud: int = 115200):
    """Open either the built-in MOCK virtual car or a real OS serial port."""
    if port.upper().startswith('MOCK'):
        return demo_sim.MockCarSerial(port.upper(), baud)
    if os.name == 'nt':
        return WinSerialPort(port, baud)
    try:
        import serial  # type: ignore[import-not-found]
    except ImportError as exc:
        raise OSError('非 Windows 系统连接物理串口需要安装 pyserial（pip install pyserial），或选择 MOCK0 离线演示。') from exc
    return serial.Serial(port, baud, timeout=0.04, write_timeout=0.2)


class RunAutoLogger:
    """Automatic @M2 run recorder: writes m2-frames.bin, m2-decoded.csv, and summary.json."""

    def __init__(self, base_dir: Path):
        self.base_dir = Path(base_dir)
        self.pending = b''
        self.m2buf = b''
        self.running = False
        self.stopped_since: float | None = None
        self.last_rx = 0.0
        self.last_saved = ''
        self.run_counter = 0

    def reset(self) -> None:
        self.pending = b''
        self.m2buf = b''
        self.running = False
        self.stopped_since = None

    def feed(self, chunk: bytes, now: float | None = None) -> str | None:
        now = time.monotonic() if now is None else now
        if chunk:
            self.last_rx = now
            self.pending += chunk
            if self.running or self.stopped_since is not None:
                self.m2buf += chunk
            parts = re.split(rb'[\r\n]+', self.pending)
            self.pending = parts.pop() if parts else b''
            for ln in parts:
                r_val = None
                m_run = re.search(rb'running=(\d)', ln)
                if m_run:
                    r_val = int(m_run.group(1))
                elif b'@M2,' in ln:
                    mm = M2_RE.search(ln)
                    if mm:
                        row = decode_m2_match(mm)
                        if row is not None:
                            if row['launch'] == 1 and row['phase'] == 1:
                                r_val = 1
                            elif row['launch'] == 0:
                                r_val = 0
                if r_val == 1 and not self.running:
                    self.running = True
                    self.m2buf = ln + b'\r\n'
                    self.stopped_since = None
                elif r_val == 0 and self.running:
                    self.running = False
                    self.stopped_since = now
        elif self.running and self.last_rx and (now - self.last_rx > 3.0):
            self.running = False
            self.stopped_since = now

        if (not self.running) and self.stopped_since is not None and (now - self.stopped_since > 0.8):
            saved = self._flush()
            self.m2buf = b''
            self.stopped_since = None
            return saved
        return None

    def _flush(self) -> str | None:
        if not self.m2buf:
            return None
        self.base_dir.mkdir(parents=True, exist_ok=True)
        existing = [p for p in self.base_dir.iterdir() if p.is_dir() and p.name.startswith('run')]
        self.run_counter = max(self.run_counter, len(existing)) + 1
        folder = self.base_dir / f'run{self.run_counter:02d}-{time.strftime("%H%M%S")}'
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'm2-frames.bin').write_bytes(self.m2buf)
        rows = [row for m in M2_RE.finditer(self.m2buf) if (row := decode_m2_match(m)) is not None]
        summary = {'run': folder.name, 'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S'), 'frames': len(rows), 'm2_bytes': len(self.m2buf)}
        if rows:
            cols = list(rows[0].keys())
            lines = [','.join(cols)] + [','.join(str(r[c]) for c in cols) for r in rows]
            (folder / 'm2-decoded.csv').write_text('\n'.join(lines) + '\n', encoding='utf-8')
            summary['max_route'] = max(r['route'] for r in rows)
            summary['final_stop'] = rows[-1]['stop']
            summary['duration_s'] = round(rows[-1]['t'] - rows[0]['t'], 2)
        (folder / 'summary.json').write_text(json.dumps(summary, indent=1, ensure_ascii=False), encoding='utf-8')
        self.last_saved = str(folder)
        return self.last_saved


class SerialBridge:
    """Single-owner serial bridge with multi-client SSE fan-out and priority E-stop writes."""

    def __init__(self, serial_factory=open_serial_port, log_dir: Path | None = None):
        self._factory = serial_factory
        self._lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._port_obj = None
        self._stop_ev = threading.Event()
        self._thread: threading.Thread | None = None
        self._clients: set[queue.Queue] = set()
        self.port_name = ''
        self.baud = 115200
        self.connected = False
        self.auto_reconnect = True
        self.auto_log = True
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.last_error = ''
        self.connected_at = 0.0
        resolved_dir = Path(log_dir) if log_dir is not None else DEFAULT_LOG_ROOT
        self._logger = RunAutoLogger(resolved_dir)

    def configure_log_dir(self, log_dir: Path) -> None:
        with self._lock:
            self._logger = RunAutoLogger(Path(log_dir))

    def status(self) -> dict:
        with self._lock:
            return {
                'connected': self.connected,
                'port': self.port_name,
                'baud': self.baud,
                'auto_reconnect': self.auto_reconnect,
                'auto_log': self.auto_log,
                'clients': len(self._clients),
                'rx_bytes': self.rx_bytes,
                'tx_bytes': self.tx_bytes,
                'last_error': self.last_error,
                'last_saved_run': self._logger.last_saved,
                'log_dir': str(self._logger.base_dir),
            }

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=256)
        with self._lock:
            self._clients.add(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._lock:
            self._clients.discard(q)

    def _broadcast(self, event: str, payload: dict) -> None:
        with self._lock:
            targets = list(self._clients)
        for q in targets:
            try:
                q.put_nowait((event, payload))
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait((event, payload))
                except queue.Empty:
                    pass

    def _validate_port(self, port: str) -> str:
        raw = str(port).strip()
        upper = raw.upper()
        if re.fullmatch(r'COM\d{1,3}|MOCK\d*', upper):
            return upper
        if os.name != 'nt' and re.fullmatch(r'/dev/(?:tty|cu)\.[A-Za-z0-9._-]+|/dev/tty(?:USB|ACM|S)\d+', raw):
            return raw
        raise ValueError('串口号格式无效（Windows 应为 COM1~COM255 或 MOCK0）。')

    def connect(self, port: str, baud: int = 115200, auto_reconnect: bool = True, auto_log: bool = True) -> dict:
        validated_port = self._validate_port(port)
        baud = int(baud)
        if baud not in ALLOWED_BAUDS:
            raise ValueError(f'不支持的波特率 {baud}。')
        self.disconnect()
        ser = self._factory(validated_port, baud)
        with self._lock:
            self._port_obj = ser
            self.port_name = validated_port
            self.baud = baud
            self.connected = True
            self.auto_reconnect = bool(auto_reconnect)
            self.auto_log = bool(auto_log)
            self.rx_bytes = 0
            self.tx_bytes = 0
            self.last_error = ''
            self.connected_at = time.time()
            self._logger.reset()
            self._stop_ev.clear()
            self._thread = threading.Thread(target=self._reader_loop, name=f'serial-{validated_port}', daemon=True)
            self._thread.start()
        st = self.status()
        self._broadcast('status', st)
        return st

    def disconnect(self) -> dict:
        self._stop_ev.set()
        with self._lock:
            ser = self._port_obj
            self._port_obj = None
            self.connected = False
            thread = self._thread
            self._thread = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        st = self.status()
        self._broadcast('status', st)
        return st

    def send(self, data: bytes, paced: bool = True, pace_ms: float = 4.0, urgent: bool = False) -> int:
        if not isinstance(data, (bytes, bytearray)) or not data:
            raise ValueError('发送数据为空。')
        if len(data) > 512:
            raise ValueError('单次发送不得超过 512 字节。')
        with self._lock:
            ser = self._port_obj
            if not self.connected or ser is None:
                raise ConnectionError('本机串口桥尚未连接串口。')
            baud = self.baud
        raw = bytes(data)
        if urgent or not paced or baud >= 921600:
            written = ser.write(raw)
        else:
            delay = max(0.001, min(0.020, float(pace_ms) / 1000.0))
            written = 0
            with self._write_lock:
                for i in range(0, len(raw), 4):
                    written += ser.write(raw[i:i + 4])
                    if i + 4 < len(raw):
                        time.sleep(delay)
        with self._lock:
            self.tx_bytes += written
        return written

    def _reader_loop(self) -> None:
        while not self._stop_ev.is_set():
            with self._lock:
                ser = self._port_obj
                port = self.port_name
                baud = self.baud
                auto_reconnect = self.auto_reconnect
                auto_log = self.auto_log
            if ser is None:
                if not auto_reconnect:
                    break
                time.sleep(0.5)
                try:
                    new_ser = self._factory(port, baud)
                    with self._lock:
                        if self._stop_ev.is_set():
                            new_ser.close()
                            break
                        self._port_obj = new_ser
                        self.connected = True
                        self.last_error = ''
                    self._broadcast('status', self.status())
                except Exception as exc:
                    with self._lock:
                        self.last_error = str(exc)
                continue
            try:
                chunk = ser.read(8192)
            except Exception as exc:
                if self._stop_ev.is_set():
                    break
                with self._lock:
                    if self._port_obj is ser:
                        self._port_obj = None
                    self.connected = False
                    self.last_error = f'串口断开：{exc}'
                try:
                    ser.close()
                except Exception:
                    pass
                self._broadcast('status', self.status())
                if not auto_reconnect:
                    break
                continue
            if chunk:
                with self._lock:
                    self.rx_bytes += len(chunk)
                self._broadcast('data', {'b64': base64.b64encode(chunk).decode('ascii'), 'n': len(chunk)})
            if auto_log:
                saved = self._logger.feed(chunk)
                if saved:
                    self._broadcast('saved', {'path': saved})
            if not chunk:
                time.sleep(0.008)


BRIDGE = SerialBridge()
