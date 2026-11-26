# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit and closed-loop integration tests for ai_tuner.py and tc264-pid-tuner CLI."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import ai_tuner
import demo_sim
import serial_bridge


class DummyComSerial:
    """Minimal fake physical COM port to verify real-car safety interlocks."""

    def __init__(self, port: str, baud: int):
        self._inner = demo_sim.MockCarSerial('MOCK0', baud, realtime=True, topo=1)

    def read(self, n: int = 8192) -> bytes:
        return self._inner.read(n)

    def write(self, data: bytes) -> int:
        return self._inner.write(data)

    def close(self) -> None:
        self._inner.close()


class TestAITunerGuardrails(unittest.TestCase):
    def test_strict_and_clamp_guardrails(self):
        bounds_suction = ai_tuner.get_loop_bounds('yaw_2s1p', dry_mode=False)
        bounds_dry = ai_tuner.get_loop_bounds('yaw_2s1p', dry_mode=True)
        self.assertEqual(bounds_dry['ykp']['max'], 14.0)

        current = {'yff': 1.85, 'ykp': 10.0, 'yki': 0.12}
        # Strict accepts small bounded move
        ok, notes = ai_tuner.check_and_clamp_proposal(
            current, {'yff': 2.0, 'ykp': 12.0, 'yki': 0.15}, bounds_suction, strict=True
        )
        self.assertEqual(ok['ykp'], 12.0)
        self.assertEqual(notes, [])

        # Strict rejects excessive step
        with self.assertRaises(ValueError):
            ai_tuner.check_and_clamp_proposal(
                current, {'yff': 1.85, 'ykp': 25.0, 'yki': 0.12}, bounds_suction, strict=True
            )

        # Strict rejects dry ceiling violation
        with self.assertRaises(ValueError):
            ai_tuner.check_and_clamp_proposal(
                {'yff': 1.85, 'ykp': 13.0, 'yki': 0.12},
                {'yff': 1.85, 'ykp': 14.5, 'yki': 0.12},
                bounds_dry,
                strict=True,
            )

        # Strict rejects booleans and NaN
        with self.assertRaises(ValueError):
            ai_tuner.check_and_clamp_proposal(
                current, {'yff': True, 'ykp': 10.0, 'yki': 0.12}, bounds_suction, strict=True
            )
        with self.assertRaises(ValueError):
            ai_tuner.check_and_clamp_proposal(
                current, {'yff': float('nan'), 'ykp': 10.0, 'yki': 0.12}, bounds_suction, strict=True
            )

        # Non-strict clamps with guardrail notes
        clamped, clamp_notes = ai_tuner.check_and_clamp_proposal(
            current, {'yff': 9.0, 'ykp': 50.0, 'yki': -1.0}, bounds_suction, strict=False
        )
        self.assertLessEqual(clamped['ykp'], 13.5)
        self.assertGreaterEqual(clamped['yki'], 0.0)
        self.assertTrue(len(clamp_notes) >= 2)

    def test_extract_json_object(self):
        wrapped = 'Here is the next step:\n```json\n{"parameters": {"ykp": 14.5, "yff": 1.9, "yki": 0.1}, "status": "TUNING", "analysis_summary": "ok"}\n```'
        obj = ai_tuner.extract_json_object(wrapped)
        self.assertIsNotNone(obj)
        self.assertEqual(obj['parameters']['ykp'], 14.5)
        self.assertIsNone(ai_tuner.extract_json_object('no json here'))

    def test_capture_analyzer_second_edge_and_rollback(self):
        # Build synthetic yaw-alt capture: 0 -> +300 -> -300 -> +300
        rows = []
        idx = 0
        for _ in range(20):
            rows.append({'idx': idx, 'target': 0, 'enc_l': 0, 'enc_r': 0, 'pwm_l': 0, 'pwm_r': 0, 'gyro': 0.0, 'set_l': 0, 'set_r': 0, 'phase': 0})
            idx += 1
        for k in range(80):
            g = 300.0 * (1.0 - pow(2.71828, -k / 5.0))
            rows.append({'idx': idx, 'target': 300, 'enc_l': -40, 'enc_r': 40, 'pwm_l': -1800, 'pwm_r': 1800, 'gyro': g, 'set_l': 0, 'set_r': 0, 'phase': 1})
            idx += 1
        for k in range(100):
            g = 300.0 - 600.0 * (1.0 - pow(2.71828, -k / 4.0))
            rows.append({'idx': idx, 'target': -300, 'enc_l': 40, 'enc_r': -40, 'pwm_l': 1900, 'pwm_r': -1900, 'gyro': g, 'set_l': 0, 'set_r': 0, 'phase': 2})
            idx += 1

        m = ai_tuner.analyze_capture_rows({'dt_ms': 4, 'profile': 'yaw-alt', 'side': 4}, rows, 'yaw_2s1p')
        self.assertEqual(m['edge_used'], 'second_dynamic_edge')
        self.assertEqual(m['status'], 'STABLE')
        self.assertLess(m['overshoot_pct'], 2.0)
        self.assertLess(m['rise_ms'], 60.0)

        bad_metrics = dict(m, status='OSCILLATING', overshoot_pct=24.0, crossings=8)
        self.assertTrue(ai_tuner.should_rollback(bad_metrics, m, 'yaw_2s1p'))

    def test_real_car_safety_gate_and_battery_floor(self):
        bridge = serial_bridge.SerialBridge(serial_factory=DummyComSerial)
        tuner = ai_tuner.AITunerManager(bridge)
        try:
            bridge.connect('COM9', 115200, auto_reconnect=False, auto_log=False)
            time.sleep(0.15)
            # Without explicit real-car confirmation -> MUST be blocked
            with self.assertRaises(PermissionError):
                tuner.apply_bounded(loop_id='yaw_2s1p', proposed={'yff': 1.85, 'ykp': 16.0, 'yki': 0.12})
            with self.assertRaises(PermissionError):
                tuner.start_auto_tune({'loop': 'yaw_2s1p', 'provider': 'rule', 'max_rounds': 2})

            # With explicit confirmation -> bounded write succeeds with readback
            res = tuner.apply_bounded(
                loop_id='yaw_2s1p',
                proposed={'yff': 1.85, 'ykp': 16.0, 'yki': 0.12},
                allow_real_car=True,
                confirm_wheels_off_ground=True,
            )
            self.assertTrue(res['readback_confirmed'])
            self.assertAlmostEqual(res['applied']['ykp'], 16.0, places=2)

            # Battery floor interlock blocks low 3S voltage
            tuner.session.vbat_mv = 10400
            with self.assertRaises(RuntimeError):
                tuner.start_auto_tune({
                    'loop': 'yaw_2s1p',
                    'provider': 'rule',
                    'max_rounds': 2,
                    'allow_real_car': True,
                    'confirm_wheels_off_ground': True,
                })
        finally:
            tuner.stop(estop=False)
            tuner.session.stop()
            bridge.disconnect()

    def test_closed_loop_auto_tune_on_mock0(self):
        bridge = serial_bridge.SerialBridge(serial_factory=serial_bridge.open_serial_port)
        tuner = ai_tuner.AITunerManager(bridge)
        try:
            bridge.connect('MOCK0', 115200, auto_reconnect=False, auto_log=False)
            time.sleep(0.15)
            # Intentionally start from sluggish yaw gains to verify active tuning
            tuner.session.write_param('ykp', 4.5)
            tuner.session.write_param('yff', 0.8)
            tuner.session.write_param('yki', 0.02)

            st = tuner.start_auto_tune({
                'loop': 'yaw_2s1p',
                'provider': 'rule',
                'max_rounds': 3,
                'target': 300,
                'goal': 'balanced',
            })
            self.assertTrue(st['active'])

            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline:
                cur = tuner.status()
                if not cur['active']:
                    break
                time.sleep(0.15)

            final = tuner.status()
            self.assertFalse(final['active'], f'Auto-tune did not finish: {final}')
            self.assertIn(final['completed_reason'], ('converged', 'max_rounds_reached'))
            self.assertGreaterEqual(len(final['history']), 2)
            self.assertIsNotNone(final['best'])
            # Best verified round should improve or maintain score over initial sluggish baseline
            r1 = final['history'][0]
            self.assertLessEqual(final['best']['score'], r1['score'])
            # Gains actually moved upward from sluggish 4.5
            self.assertGreater(final['best']['parameters']['ykp'], 4.5)
        finally:
            tuner.stop(estop=False)
            tuner.session.stop()
            bridge.disconnect()

    def test_cli_offline_tools(self):
        cli = ROOT / 'skills' / 'tc264-pid-tuner' / 'scripts' / 'tunepro_ai_cli.py'
        with tempfile.TemporaryDirectory() as td:
            td_path = Path(td)
            prop = td_path / 'proposal.json'
            prop.write_text(json.dumps({
                'current': {'yff': 1.85, 'ykp': 12.0, 'yki': 0.1},
                'proposed': {'yff': 1.95, 'ykp': 14.0, 'yki': 0.12},
                'bounds': {
                    'yff': {'min': 0.2, 'max': 5.0, 'max_step': 0.45},
                    'ykp': {'min': 1.0, 'max': 36.0, 'max_step': 3.5},
                    'yki': {'min': 0.0, 'max': 1.5, 'max_step': 0.15},
                },
            }), encoding='utf-8')
            p_ok = subprocess.run([sys.executable, str(cli), 'check-update', '--file', str(prop)], capture_output=True, text=True, check=True)
            data = json.loads(p_ok.stdout)
            self.assertTrue(data['valid'])
            self.assertFalse(data['applied'])

            # Out-of-step proposal must exit nonzero
            bad = td_path / 'bad.json'
            bad.write_text(json.dumps({
                'current': {'ykp': 12.0},
                'proposed': {'ykp': 25.0},
                'bounds': {'ykp': {'min': 1.0, 'max': 36.0, 'max_step': 3.5}},
            }), encoding='utf-8')
            p_bad = subprocess.run([sys.executable, str(cli), 'check-update', '--file', str(bad)], capture_output=True, text=True)
            self.assertNotEqual(p_bad.returncode, 0)

            # CSV analyzer on TunePro m2-decoded format
            csv_file = td_path / 'm2-decoded.csv'
            csv_file.write_text(
                't,error,yaw,gyro,set_l,enc_l,pwm_l,set_r,enc_r,pwm_r\n'
                '0.00,0,300,120,0,-20,-1500,0,20,1500\n'
                '0.02,0,300,240,0,-40,-2100,0,40,2100\n'
                '0.04,0,300,290,0,-48,-2200,0,48,2200\n'
                '0.06,0,300,298,0,-49,-2150,0,49,2150\n'
                '0.08,0,300,301,0,-50,-2100,0,50,2100\n',
                encoding='utf-8',
            )
            p_csv = subprocess.run([sys.executable, str(cli), 'analyze-csv', '--csv', str(csv_file), '--channel', 'yaw'], capture_output=True, text=True, check=True)
            csv_res = json.loads(p_csv.stdout)
            self.assertEqual(csv_res['format'], 'tunepro_m2_csv')
            self.assertEqual(csv_res['samples'], 5)
            self.assertLess(csv_res['metrics']['tail_mae'], 5.0)

    def test_ai_chat_and_reasoning_effort(self):
        # Reasoning effort normalization & round budgets
        self.assertEqual(ai_tuner.normalize_effort('max'), 'max')
        self.assertEqual(ai_tuner.normalize_effort('HIGH'), 'high')
        self.assertEqual(ai_tuner.normalize_effort('unknown_tier'), 'medium')
        self.assertEqual(ai_tuner.REASONING_LEVELS['fast']['default_rounds'], 3)
        self.assertEqual(ai_tuner.REASONING_LEVELS['max']['default_rounds'], 10)

        bridge = serial_bridge.SerialBridge(serial_factory=serial_bridge.open_serial_port)
        tuner = ai_tuner.AITunerManager(bridge)
        try:
            bridge.connect('MOCK0', 115200, auto_reconnect=False, auto_log=False)
            time.sleep(0.15)
            # Ask AI to damp overshoot with live telemetry and high reasoning effort
            res_damp = tuner.chat({
                'message': '入弯超调太大而且有一点晃动，怎么压低？',
                'loop': 'yaw_2s1p',
                'provider': 'rule',
                'reasoning_effort': 'high',
                'live_telemetry': {'spd': 120.0, 'gyro': 280.0, 'pwmCm': 2100.0, 'pwmDm': 1950.0},
            })
            self.assertTrue(res_damp['ok'])
            self.assertEqual(res_damp['effort'], 'high')
            self.assertTrue(len(res_damp['insights']) >= 2)
            self.assertIsNotNone(res_damp['suggested_params'])
            self.assertLess(res_damp['suggested_params']['ykp'], 18.0)

            # Dry-floor reference intent must stay within bounded step
            res_dry = tuner.chat({
                'message': '切干地 0% 负压参考参数',
                'loop': 'yaw_2s1p',
                'provider': 'rule',
                'reasoning_effort': 'max',
                'dry_mode': True,
            })
            self.assertTrue(res_dry['ok'])
            self.assertIsNotNone(res_dry['suggested_params'])
            self.assertLessEqual(res_dry['suggested_params']['ykp'], 14.0)

            # Empty chat message rejected
            with self.assertRaises(ValueError):
                tuner.chat({'message': '   ', 'loop': 'yaw_2s1p'})
        finally:
            tuner.stop(estop=False)
            tuner.session.stop()
            bridge.disconnect()


if __name__ == '__main__':
    unittest.main()
