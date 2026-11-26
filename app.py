# SPDX-License-Identifier: GPL-3.0-or-later
"""TC264 三环远程调参台 — 零第三方依赖本地运行程序。

用法：
  双击 启动三环调参台.bat（或 启动离线演示.bat），或运行：
  python app.py [--port 18760] [--demo] [--no-window] [--log-dir ./runs]
"""
from __future__ import annotations

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.parse
import urllib.request
import webbrowser

BUNDLE_ROOT = Path(getattr(sys, '_MEIPASS', Path(__file__).resolve().parent))
APP_ROOT = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent
ROOT = BUNDLE_ROOT
WEB_DIR = BUNDLE_ROOT / 'web'
if str(BUNDLE_ROOT) not in sys.path:
    sys.path.insert(0, str(BUNDLE_ROOT))
import serial_bridge
import ai_tuner

mimetypes.add_type('application/javascript; charset=utf-8', '.js')
mimetypes.add_type('application/javascript; charset=utf-8', '.mjs')
mimetypes.add_type('text/css; charset=utf-8', '.css')
mimetypes.add_type('image/svg+xml', '.svg')
mimetypes.add_type('image/x-icon', '.ico')
mimetypes.add_type('image/png', '.png')

DEMO_MODE = False


class LifecycleTracker:
    """Tracks live browser window connections so a frozen --noconsole EXE exits cleanly on window close."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.active_clients = 0
        self.ever_connected = False
        self.last_zero_at = 0.0

    def connect(self) -> None:
        with self._lock:
            self.active_clients += 1
            self.ever_connected = True

    def disconnect(self) -> None:
        with self._lock:
            self.active_clients = max(0, self.active_clients - 1)
            if self.active_clients == 0:
                self.last_zero_at = time.monotonic()

    def should_exit(self, grace_s: float = 4.0) -> bool:
        with self._lock:
            if not self.ever_connected or self.active_clients > 0:
                return False
            return (time.monotonic() - self.last_zero_at) >= grace_s


LIFECYCLE = LifecycleTracker()
SHUTDOWN_EV = threading.Event()


class AppServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 64


class AppHandler(BaseHTTPRequestHandler):
    server_version = 'TC264TunePro/1.4'

    def setup(self):
        super().setup()
        try:
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass

    def log_message(self, fmt, *args):
        pass

    def _headers_common(self):
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Cache-Control', 'no-store')

    def _send_json(self, data: dict, status: int = 200):
        raw = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self._headers_common()
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _read_json(self) -> dict:
        size = int(self.headers.get('Content-Length', '0'))
        if size <= 0 or size > 262144:
            return {}
        return json.loads(self.rfile.read(size).decode('utf-8'))

    def _stream_sse(self, control_only: bool = False):
        sub = serial_bridge.BRIDGE.subscribe(include_data=not control_only)
        self.send_response(200)
        self._headers_common()
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            init = f"event: status\ndata: {json.dumps(serial_bridge.BRIDGE.status(), ensure_ascii=False)}\n\n".encode('utf-8')
            self.wfile.write(init)
            self.wfile.flush()
            while True:
                try:
                    ev, payload = sub.get(timeout=15.0)
                    msg = f"event: {ev}\ndata: {json.dumps(payload, ensure_ascii=False)}\n\n".encode('utf-8')
                except queue.Empty:
                    msg = b': ping\n\n'
                self.wfile.write(msg)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            serial_bridge.BRIDGE.unsubscribe(sub)

    def _stream_raw(self):
        """Zero-Base64 raw binary octet stream for sub-millisecond camera & 50Hz telemetry delivery."""
        sub = serial_bridge.BRIDGE.subscribe_raw()
        self.send_response(200)
        self._headers_common()
        self.send_header('Content-Type', 'application/octet-stream')
        self.send_header('X-Accel-Buffering', 'no')
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            self.wfile.flush()
            while not SHUTDOWN_EV.is_set():
                try:
                    chunk = sub.get(timeout=0.5)
                except queue.Empty:
                    continue
                # Drain any immediately queued chunks into a single write to minimize syscalls
                if not sub.empty():
                    buf = bytearray(chunk)
                    while len(buf) < 65536:
                        try:
                            buf.extend(sub.get_nowait())
                        except queue.Empty:
                            break
                    chunk = bytes(buf)
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            serial_bridge.BRIDGE.unsubscribe_raw(sub)

    def _stream_lifecycle(self):
        LIFECYCLE.connect()
        self.send_response(200)
        self._headers_common()
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        try:
            self.wfile.write(b'event: ready\ndata: {"ok":true}\n\n')
            self.wfile.flush()
            while not SHUTDOWN_EV.wait(5.0):
                self.wfile.write(b': keepalive\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            LIFECYCLE.disconnect()

    def do_GET(self):
        try:
            parsed = urllib.parse.urlsplit(self.path)
            path = parsed.path
            qs = urllib.parse.parse_qs(parsed.query)
            if path == '/api/health':
                return self._send_json({'ok': True, 'app': 'tc264-tunepro', 'version': '1.4', 'demo': DEMO_MODE, 'ai_tuner': True, 'ai_chat': True, 'raw_stream': True, 'local_cli': True})
            if path == '/api/serial/ports':
                return self._send_json({
                    'ports': serial_bridge.list_serial_ports(include_demo=True, demo_first=DEMO_MODE),
                    'status': serial_bridge.BRIDGE.status(),
                    'demo': DEMO_MODE,
                })
            if path == '/api/serial/status':
                return self._send_json(serial_bridge.BRIDGE.status())
            if path == '/api/serial/stream':
                control_only = ((qs.get('events') or [''])[0] == 'control')
                return self._stream_sse(control_only=control_only)
            if path == '/api/serial/raw-stream':
                return self._stream_raw()
            if path == '/api/app/lifecycle':
                return self._stream_lifecycle()
            if path == '/api/ai/local-clis':
                return self._send_json({'ok': True, 'clis': ai_tuner.detect_local_clis()})
            if path == '/api/ai/snapshot':
                loop_id = (qs.get('loop') or [None])[0]
                dry_mode = ((qs.get('dry') or ['0'])[0] in ('1', 'true', 'yes'))
                return self._send_json(ai_tuner.AI_TUNER.snapshot(loop_id=loop_id, dry_mode=dry_mode))
            if path == '/api/ai/status':
                return self._send_json(ai_tuner.AI_TUNER.status())
            rel = urllib.parse.unquote(path.lstrip('/') or 'index.html')
            target = (WEB_DIR / rel).resolve()
            if not target.is_relative_to(WEB_DIR.resolve()) or not target.is_file():
                return self._send_json({'error': '文件不存在'}, 404)
            ctype = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
            if target.suffix == '.html':
                ctype = 'text/html; charset=utf-8'
            raw = target.read_bytes()
            self.send_response(200)
            self._headers_common()
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as exc:
            self._send_json({'error': str(exc)[:300]}, 500)

    def do_POST(self):
        try:
            path = urllib.parse.urlsplit(self.path).path
            body = self._read_json()
            if path == '/api/serial/connect':
                return self._send_json(serial_bridge.BRIDGE.connect(
                    body.get('port', ''),
                    int(body.get('baud', 115200)),
                    bool(body.get('auto_reconnect', True)),
                    bool(body.get('auto_log', True)),
                ))
            if path == '/api/serial/disconnect':
                ai_tuner.AI_TUNER.stop(estop=False)
                return self._send_json(serial_bridge.BRIDGE.disconnect())
            if path == '/api/serial/send':
                if 'b64' in body:
                    raw = base64.b64decode(str(body['b64']), validate=True)
                else:
                    raw = str(body.get('text', '')).encode('latin1')
                n = serial_bridge.BRIDGE.send(
                    raw,
                    paced=bool(body.get('paced', True)),
                    pace_ms=float(body.get('pace_ms', 4.0)),
                    urgent=bool(body.get('urgent', False)),
                )
                return self._send_json({'ok': True, 'written': n})
            if path == '/api/ai/start':
                return self._send_json(ai_tuner.AI_TUNER.start_auto_tune(body))
            if path == '/api/ai/stop':
                return self._send_json(ai_tuner.AI_TUNER.stop(estop=bool(body.get('estop', True))))
            if path == '/api/ai/estop':
                return self._send_json(ai_tuner.AI_TUNER.stop(estop=True))
            if path == '/api/ai/step-test':
                return self._send_json(ai_tuner.AI_TUNER.run_one_step(
                    loop_id=str(body.get('loop') or 'yaw_2s1p'),
                    target=int(body['target']) if body.get('target') is not None else None,
                    low=int(body['low']) if body.get('low') is not None else None,
                    dry_mode=bool(body.get('dry_mode', False)),
                    allow_real_car=bool(body.get('allow_real_car', False)),
                    confirm_wheels_off_ground=bool(body.get('confirm_wheels_off_ground', False)),
                ))
            if path == '/api/ai/apply':
                return self._send_json(ai_tuner.AI_TUNER.apply_bounded(
                    loop_id=str(body.get('loop') or 'yaw_2s1p'),
                    proposed=dict(body.get('parameters') or {}),
                    dry_mode=bool(body.get('dry_mode', False)),
                    allow_real_car=bool(body.get('allow_real_car', False)),
                    confirm_wheels_off_ground=bool(body.get('confirm_wheels_off_ground', False)),
                    strict=bool(body.get('strict', True)),
                ))
            if path == '/api/ai/rollback':
                return self._send_json(ai_tuner.AI_TUNER.rollback_to_best(
                    allow_real_car=bool(body.get('allow_real_car', False)),
                    confirm_wheels_off_ground=bool(body.get('confirm_wheels_off_ground', False)),
                ))
            if path == '/api/ai/chat':
                return self._send_json(ai_tuner.AI_TUNER.chat(body))
            if path == '/api/app/shutdown':
                SHUTDOWN_EV.set()
                threading.Timer(0.15, self.server.shutdown).start()
                return self._send_json({'ok': True})
            return self._send_json({'error': '未知接口'}, 404)
        except PermissionError as exc:
            self._send_json({'error': str(exc)}, 403)
        except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self._send_json({'error': str(exc)}, 400)
        except (FileNotFoundError, ConnectionError, RuntimeError, TimeoutError, OSError) as exc:
            self._send_json({'error': str(exc)}, 409)
        except Exception as exc:
            self._send_json({'error': str(exc)[:300]}, 500)


def pick_port(host: str, preferred: int) -> int:
    for port in [preferred, *range(preferred + 1, preferred + 20)]:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind((host, port))
                return port
            except OSError:
                continue
    return 0


def open_app_window(url: str) -> None:
    """Open as a clean native app window (Edge/Chrome --app mode), fallback to browser."""
    if os.name == 'nt':
        candidates = [
            Path(r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe'),
            Path(r'C:\Program Files\Microsoft\Edge\Application\msedge.exe'),
            Path(r'C:\Program Files\Google\Chrome\Application\chrome.exe'),
            Path(r'C:\Program Files (x86)\Google\Chrome\Application\chrome.exe'),
        ]
        profile_dir = Path(tempfile.gettempdir()) / 'tc264-tunepro-window-profile'
        for exe in candidates:
            if exe.is_file():
                try:
                    subprocess.Popen(
                        [
                            str(exe),
                            f'--app={url}',
                            '--window-size=1420,900',
                            f'--user-data-dir={profile_dir}',
                        ],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    return
                except OSError:
                    pass
    webbrowser.open(url)


def _existing_instance_url(host: str, preferred: int) -> str | None:
    url = f'http://{host}:{preferred}/api/health'
    try:
        with urllib.request.urlopen(url, timeout=0.35) as resp:
            info = json.loads(resp.read().decode('utf-8'))
            if info.get('ok') and info.get('app') == 'tc264-tunepro':
                return f'http://{host}:{preferred}/'
    except Exception:
        pass
    return None


def main():
    global DEMO_MODE
    ap = argparse.ArgumentParser(description='TC264 三环远程调参台（零依赖开源本地版）')
    ap.add_argument('--host', default='127.0.0.1', help='监听地址（默认 127.0.0.1）')
    ap.add_argument('--port', type=int, default=18760, help='HTTP 端口（默认 18760）')
    ap.add_argument('--demo', action='store_true', help='离线演示模式：优先选中内置虚拟小车 MOCK0')
    ap.add_argument('--log-dir', type=Path, default=None, help='每趟 @M2 遥测自动落盘目录（默认 ./runs）')
    ap.add_argument('--no-window', action='store_true', help='仅启动后台服务，不自动弹窗口')
    args = ap.parse_args()

    DEMO_MODE = bool(args.demo)
    if args.log_dir is not None:
        serial_bridge.BRIDGE.configure_log_dir(args.log_dir.resolve())

    # If double-clicked while an instance is already running on the default port, reuse it cleanly
    if not args.no_window:
        existing = _existing_instance_url(args.host, args.port)
        if existing:
            open_app_window(existing)
            return

    port = pick_port(args.host, args.port)
    httpd = AppServer((args.host, port), AppHandler)
    actual_port = httpd.server_address[1]
    url = f'http://{args.host}:{actual_port}/'
    print(f'TC264 三环调参台已启动：{url}', flush=True)
    if DEMO_MODE:
        print('离线演示模式已启用：在顶部串口下拉选择 MOCK0 点击「连接」即可零硬件体验三环调参与实时图传。', flush=True)
    print(f'自动落盘目录：{serial_bridge.BRIDGE.status()["log_dir"]}', flush=True)
    print('按 Ctrl+C 或关闭窗口即可退出程序。', flush=True)

    if not args.no_window:
        threading.Timer(0.35, open_app_window, args=(url,)).start()
        if getattr(sys, 'frozen', False):
            def _watch_window_close():
                while not SHUTDOWN_EV.wait(1.0):
                    if LIFECYCLE.should_exit(grace_s=4.5) and not ai_tuner.AI_TUNER.status().get('active'):
                        SHUTDOWN_EV.set()
                        httpd.shutdown()
                        break
            threading.Thread(target=_watch_window_close, name='exe-lifecycle-watch', daemon=True).start()

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        SHUTDOWN_EV.set()
        ai_tuner.AI_TUNER.stop(estop=True)
        serial_bridge.BRIDGE.disconnect()
        httpd.server_close()


if __name__ == '__main__':
    main()
