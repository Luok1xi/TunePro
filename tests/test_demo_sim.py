# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit tests for demo_sim.py (offline virtual TC264 three-loop car simulator)."""
from __future__ import annotations

from pathlib import Path
import struct
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import demo_sim
import serial_bridge


class DemoSimTest(unittest.TestCase):
    def _drain(self, car: demo_sim.MockCarSerial, timeout: float = 0.08) -> bytes:
        out = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = car.read(8192)
            if chunk:
                out.extend(chunk)
            else:
                if out:
                    break
        return bytes(out)

    def test_crc16_known_vector(self):
        self.assertEqual(demo_sim.crc16(b'123456789'), 0x29B1)

    def test_meta_write_revision_cas_and_continuous_run(self):
        car = demo_sim.MockCarSerial('MOCK0', 115200, realtime=False)
        try:
            boot = self._drain(car)
            self.assertIn(b'ID fw=TUNEPRO-01-DEMO', boot)
            self.assertIn(b'@GP1,0,1,', boot)

            # Query snapshot (#M)
            car.write(demo_sim.sign_line('#M 1'))
            reply = self._drain(car)
            self.assertIn(b'@GP1,1,1,', reply)

            # Revision mismatch rejected with REVISION
            car.write(demo_sim.sign_line('#W 2 99 lkp 1800'))
            err = self._drain(car)
            self.assertIn(b'@QX1,2,1,REVISION,1*', err)

            # Valid live gain write increments rev
            car.write(demo_sim.sign_line('#W 3 1 lkp 1850'))
            ack = self._drain(car)
            self.assertIn(b'@WA1,3,2,lkp,1850*', ack)
            self.assertIn(b'@GP1,3,2,', ack)
            self.assertAlmostEqual(car.lkp, 18.5, places=2)

            # Start continuous left-wheel run and step plant 250 ticks (500 ms)
            car.write(demo_sim.sign_line('#X RUN L 150'))
            run_ack = self._drain(car)
            self.assertIn(b'@RA1,RUN,1,150,1*', run_ack)
            self.assertTrue(car.running)
            for _ in range(250):
                car.step_2ms()
            self.assertGreater(car.enc_l, 120.0)
            self.assertLess(abs(car.enc_l - 150.0), 25.0)
            self.assertLess(abs(car.enc_r), 5.0)

            # Emergency stop via urgent '#!'
            car.write(b'#!')
            stop_ack = self._drain(car)
            self.assertFalse(car.running)
            self.assertIn(b'@RA1,STOP,0,25,0*', stop_ack)

            # Export 4 ms onboard capture (#X CAP)
            car.write(demo_sim.sign_line('#X CAP'))
            cap = self._drain(car)
            self.assertIn(b'[tune2] cap begin ', cap)
            self.assertIn(b'build=TUNEPRO-01', cap)
            self.assertIn(b'u2,0,', cap)
            self.assertIn(b'[tune2] cap end', cap)
        finally:
            car.close()

    def test_2s1p_direct_pwm_yaw_and_topology_switch(self):
        car = demo_sim.MockCarSerial('MOCK0', 115200, realtime=False, topo=1)
        try:
            boot = self._drain(car)
            self.assertEqual(car.topo, 1)
            self.assertAlmostEqual(car.ykp, 18.0, places=2)
            self.assertAlmostEqual(car.yff, 1.85, places=2)
            self.assertIn(b'@GP1,0,1,', boot)

            # Live write yff & direct-PWM ykp
            car.write(demo_sim.sign_line('#W 1 1 yff 2000'))
            self._drain(car)
            self.assertAlmostEqual(car.yff, 2.0, places=2)

            # Spin in place using 2S1P direct-PWM yaw loop
            car.write(demo_sim.sign_line('#X RUN W 300'))
            self._drain(car)
            for _ in range(120):
                car.step_2ms()
            # Direct-PWM yaw tracks +300 deg/s with opposite L/R PWM
            self.assertGreater(car.gyro_dps, 240.0)
            self.assertLess(car.pwm_l, -500.0)
            self.assertGreater(car.pwm_r, 500.0)

            # Stop and switch to classic 3-cascade (topo=0)
            car.write(b'#!')
            self._drain(car)
            car.write(demo_sim.sign_line(f'#W 2 {car.rev} topo 0'))
            self._drain(car)
            self.assertEqual(car.topo, 0)
            self.assertAlmostEqual(car.ykp, 0.22, places=3)
        finally:
            car.close()

    def test_compressed_camera_packets_crc_and_modes(self):
        car = demo_sim.MockCarSerial('MOCK0', 115200, realtime=False)
        try:
            self._drain(car)
            for img_mode, expected_fmt in ((1, 1), (2, 3), (3, 4)):
                car.write(demo_sim.sign_line(f'#X IMG {img_mode} 10'))
                ack = self._drain(car)
                self.assertIn(f'@RA1,IMG,0,{img_mode},10*'.encode('ascii'), ack)
                car._emit_compressed_frame()
                pkt = self._drain(car)
                self.assertTrue(pkt.startswith(b'\xa5\x5b'))
                self.assertEqual(pkt[2], expected_fmt)
                slen = struct.unpack_from('<H', pkt, 6)[0]
                expected_crc = struct.unpack_from('<H', pkt, 26 + slen)[0]
                self.assertEqual(demo_sim.crc16(pkt[2:26 + slen]), expected_crc)
        finally:
            car.close()

    def test_bridge_connects_mock0_and_streams_m2(self):
        bridge = serial_bridge.SerialBridge()
        sub = bridge.subscribe()
        try:
            st = bridge.connect('MOCK0', 115200, auto_reconnect=False, auto_log=False)
            self.assertTrue(st['connected'])
            self.assertEqual(st['port'], 'MOCK0')
            saw_m2 = False
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                ev, payload = sub.get(timeout=0.5)
                if ev == 'data':
                    import base64
                    if b'@M2,' in base64.b64decode(payload['b64']):
                        saw_m2 = True
                        break
            self.assertTrue(saw_m2, 'Expected live @M2 frames from MOCK0 bridge')
        finally:
            bridge.unsubscribe(sub)
            bridge.disconnect()


if __name__ == '__main__':
    unittest.main()
