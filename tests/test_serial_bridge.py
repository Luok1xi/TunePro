# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for serial_bridge.py (zero hardware required)."""
from __future__ import annotations

import base64
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import serial_bridge


class FakeSerial:
    def __init__(self, port: str, baud: int):
        self.port = port
        self.baud = baud
        self.rx: queue.Queue[bytes] = queue.Queue()
        self.tx: list[bytes] = []
        self.closed = False

    def read(self, n: int = 8192) -> bytes:
        if self.closed:
            raise OSError('closed')
        try:
            return self.rx.get(timeout=0.03)
        except queue.Empty:
            return b''

    def write(self, data: bytes) -> int:
        if self.closed:
            raise OSError('closed')
        self.tx.append(bytes(data))
        return len(data)

    def close(self) -> None:
        self.closed = True


class SerialBridgeTest(unittest.TestCase):
    def test_connect_fanout_urgent_and_autolog(self):
        created: list[FakeSerial] = []

        def factory(port: str, baud: int):
            s = FakeSerial(port, baud)
            created.append(s)
            return s

        with tempfile.TemporaryDirectory() as tmp:
            bridge = serial_bridge.SerialBridge(serial_factory=factory, log_dir=Path(tmp))
            q1 = bridge.subscribe()
            q2 = bridge.subscribe()
            try:
                st = bridge.connect('COM7', 115200, auto_reconnect=False, auto_log=True)
                self.assertTrue(st['connected'])
                self.assertEqual(st['port'], 'COM7')
                ser = created[-1]

                # Urgent emergency stop writes atomically without chunk splitting
                n = bridge.send(b'#!#X STOP*1234\r\n', paced=True, urgent=True)
                self.assertEqual(n, 16)
                self.assertEqual(ser.tx, [b'#!#X STOP*1234\r\n'])

                # Paced wireless write splits into small chunks
                ser.tx.clear()
                bridge.send(b'#M 1*ABCD\r\n', paced=True, pace_ms=1.0, urgent=False)
                self.assertGreater(len(ser.tx), 1)
                self.assertEqual(b''.join(ser.tx), b'#M 1*ABCD\r\n')

                # Broadcast raw bytes to multiple subscribers
                ser.rx.put(b't=100 tune=1 running=0\r\n')
                got = []
                for q in (q1, q2):
                    deadline = time.time() + 1.0
                    while time.time() < deadline:
                        ev, payload = q.get(timeout=0.5)
                        if ev == 'data':
                            got.append(base64.b64decode(payload['b64']))
                            break
                self.assertEqual(got, [b't=100 tune=1 running=0\r\n', b't=100 tune=1 running=0\r\n'])
            finally:
                bridge.unsubscribe(q1)
                bridge.unsubscribe(q2)
                bridge.disconnect()

    def test_invalid_port_and_baud_rejected(self):
        bridge = serial_bridge.SerialBridge(serial_factory=FakeSerial)
        with self.assertRaises(ValueError):
            bridge.connect('../etc/passwd', 115200)
        with self.assertRaises(ValueError):
            bridge.connect('COM7', 12345)

    def test_m2_autologger_saves_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            logger = serial_bridge.RunAutoLogger(Path(tmp))
            body = b'@M2,00000001,00000001,000003E8,05,00,00,000F,0002,0000,0000,007D,007C,0400,007D,007C,0400,0FA0,00,00000000,00000000,0005,00000000,00000010,00000001'
            crc = serial_bridge._crc16(body)
            m2_run = body + f'*{crc:04X}\r\n'.encode('ascii')
            stop_body = b'@M2,00000001,00000002,000004B0,05,00,00,0008,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,1A,00000000,00000000,0005,00000000,00000011,00000001'
            m2_stop = stop_body + f'*{serial_bridge._crc16(stop_body):04X}\r\n'.encode('ascii')
            self.assertIsNone(logger.feed(m2_run, now=10.0))
            self.assertTrue(logger.running)
            self.assertIsNone(logger.feed(m2_stop, now=10.5))
            self.assertFalse(logger.running)
            saved = logger.feed(b'', now=11.5)
            self.assertIsNotNone(saved)
            folder = Path(saved)
            self.assertTrue((folder / 'm2-decoded.csv').is_file())
            self.assertTrue((folder / 'summary.json').is_file())

    def test_raw_binary_stream_skips_control_sse_data(self):
        created: list[FakeSerial] = []

        def factory(port: str, baud: int):
            s = FakeSerial(port, baud)
            created.append(s)
            return s

        bridge = serial_bridge.SerialBridge(serial_factory=factory)
        ctrl_q = bridge.subscribe(include_data=False)
        raw_q = bridge.subscribe_raw()
        try:
            bridge.connect('COM7', 115200, auto_reconnect=False, auto_log=False)
            ser = created[-1]
            payload = b'\xa5\x5b\x01RAW_BINARY_FRAME\r\n'
            ser.rx.put(payload)
            got = raw_q.get(timeout=1.0)
            self.assertEqual(got, payload)
            # Control-only SSE subscriber must NOT receive Base64 data events
            while not ctrl_q.empty():
                ev, _ = ctrl_q.get_nowait()
                self.assertNotEqual(ev, 'data')
        finally:
            bridge.unsubscribe(ctrl_q)
            bridge.unsubscribe_raw(raw_q)
            bridge.disconnect()


if __name__ == '__main__':
    unittest.main()
