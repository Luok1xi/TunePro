# SPDX-License-Identifier: GPL-3.0-or-later
"""Unit and closed-loop integration tests for ai_tuner.py and tc264-pid-tuner CLI."""
from __future__ import annotations

import json
import math
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

    def test_detect_local_clis_and_fallback_error(self):
        clis = ai_tuner.detect_local_clis()
        self.assertEqual([c['id'] for c in clis], ['agy', 'claude', 'codex', 'gemini'])
        for item in clis:
            self.assertIn('label', item)
            self.assertIn('available', item)
            self.assertIsInstance(item['available'], bool)

        # Requesting a non-existent CLI tool raises a clear RuntimeError without side effects
        with self.assertRaises(RuntimeError):
            ai_tuner.call_local_cli_text(
                cli_tool='nonexistent_cli_tool',
                model='',
                reasoning_effort='fast',
                system_prompt='test',
                user_prompt='test',
            )

    def test_bridge_session_parses_gp1_and_wa1_amid_binary_camera_bytes(self):
        bridge = serial_bridge.SerialBridge(serial_factory=DummyComSerial)
        sess = ai_tuner.BridgeSession(bridge)
        gp1_raw = demo_sim.sign_line('@GP1,1,7,4,2,3,125,150,90,300,4000,1,3800,1200,3800,1200,180000,1200,80,920,8500,6000,1,0,2000,1850,1')
        wa1_raw = demo_sim.sign_line('@WA1,2,8,ykp,165000')
        # Binary A5 5B camera packet glued immediately before @GP1 and @WA1 without newline
        sess._feed_bytes(b'\xa5\x5b\x01\x01\x00\x01\x10\x00\x80\xff\x00\xfe' + gp1_raw)
        self.assertIsNotNone(sess.gp1)
        self.assertEqual(sess.gp1['rev'], 7)
        sess._feed_bytes(b'\xa5\x5b\x01\x01\x00\x01\x10\x00\x80\xff\x00\xfe' + wa1_raw)
        self.assertEqual(sess._last_ack, ('WA1', ['2', '8', 'ykp', '165000']))

    def test_warm_cli_model_tiering_and_stream_params_filter(self):
        self.assertEqual(ai_tuner.resolve_local_cli_model('agy', '', 'fast'), 'gemini-3.8-flash-low')
        self.assertEqual(ai_tuner.resolve_local_cli_model('agy', '', 'medium'), 'gemini-3.8-flash-medium')
        self.assertEqual(ai_tuner.resolve_local_cli_model('agy', '', 'max'), 'gemini-4-neon-high')
        self.assertEqual(ai_tuner.resolve_local_cli_model('agy', 'custom-model', 'fast'), 'custom-model')
        self.assertEqual(ai_tuner.resolve_local_cli_model('claude', '', 'low'), 'haiku')

        # Stream filter suppresses <<<PARAMS>>> even when split across tiny chunks
        emitted: list[str] = []
        filt = ai_tuner._StreamParamsFilter(emitted.append)
        full = '建议先降一点 ykp 压低超调。\n<<<PARAMS>>>\n{"ykp": 16.0}\n<<<END>>>'
        for i in range(0, len(full), 3):
            filt.feed(full[i:i + 3])
        filt.flush()
        streamed_txt = ''.join(emitted)
        self.assertIn('建议先降一点 ykp', streamed_txt)
        self.assertNotIn('<<<', streamed_txt)
        self.assertNotIn('PARAMS', streamed_txt)

        prose, ins, params = ai_tuner.split_prose_and_params(full)
        self.assertEqual(prose, '建议先降一点 ykp 压低超调。')
        self.assertEqual(params, {'ykp': 16.0})
        self.assertEqual(ins, [])

        # Legacy JSON reply fallback unwraps cleanly without raw braces
        prose2, ins2, params2 = ai_tuner.split_prose_and_params('{"reply": "先降 Kp", "insights": ["超调偏高"], "suggested_params": {"ykp": 15.5}}')
        self.assertEqual(prose2, '先降 Kp')
        self.assertEqual(ins2, ['超调偏高'])
        self.assertEqual(params2, {'ykp': 15.5})

        # chat_stream offline rule engine emits live deltas and elapsed_ms
        bridge = serial_bridge.SerialBridge(serial_factory=serial_bridge.open_serial_port)
        tuner = ai_tuner.AITunerManager(bridge)
        try:
            deltas: list[str] = []
            res = tuner.chat_stream({'message': '超调偏大怎么调？', 'loop': 'yaw_2s1p', 'provider': 'rule'}, on_delta=deltas.append)
            self.assertTrue(res['ok'])
            self.assertGreaterEqual(len(deltas), 1)
            self.assertIn('elapsed_ms', res)
        finally:
            tuner.stop(estop=False)
            tuner.session.stop()
            bridge.disconnect()

    def test_v16_waveform_sampler_dynamics_history_and_incremental_pid(self):
        # 1) Extrema-preserving waveform sampler & dynamics on synthetic underdamped yaw capture
        rows = []
        idx = 0
        for _ in range(15):
            rows.append({'idx': idx, 'target': 300, 'enc_l': -40, 'enc_r': 40, 'pwm_l': -1200, 'pwm_r': 1200, 'gyro': 300.0, 'set_l': 0, 'set_r': 0, 'phase': 1})
            idx += 1
        for k in range(140):
            # Step from +300 -> -300 with ~18% low-freq overshoot
            t = k * 0.004
            g = -300.0 + 600.0 * math.exp(-t / 0.045) * math.cos(t * 28.0)
            dm = -1100.0 - 400.0 * math.exp(-t / 0.04)
            rows.append({'idx': idx, 'target': -300, 'enc_l': 40, 'enc_r': -40, 'pwm_l': -dm, 'pwm_r': dm, 'gyro': g, 'set_l': 0, 'set_r': 0, 'phase': 2})
            idx += 1
        hdr = {'dt_ms': 4, 'profile': 'yaw-alt', 'side': 4}
        m = ai_tuner.analyze_capture_rows(hdr, rows, 'yaw_2s1p')
        wf = ai_tuner.sample_capture_waveform(hdr, rows, 'yaw_2s1p', limit=20)
        wf_lines = wf.splitlines()
        self.assertGreaterEqual(len(wf_lines), 12)
        self.assertLessEqual(len(wf_lines), 30)
        self.assertIn('t_ms,tgt,act,err,pwm_cm,pwm_dm', wf_lines[1])
        self.assertIn('dynamics', m)
        self.assertIsNotNone(m['dynamics']['tau_eff_ms'])
        self.assertIsNotNone(m['dynamics']['implied_ff_hint'])

        # 2) Acceptance gaps & tiered rollback (never roll back from OVERSHOOTING to OSCILLATING)
        ok, gaps, detail = ai_tuner.describe_acceptance_gaps(m, 'yaw_2s1p', 'balanced')
        self.assertFalse(ok)
        self.assertTrue(any('超调' in g for g in gaps))
        self.assertIn('超调', detail)
        osc_m = dict(m, status='OSCILLATING', overshoot_pct=35.0, crossings=9)
        ov_m = dict(m, status='OVERSHOOTING', overshoot_pct=16.0, crossings=2)
        self.assertFalse(ai_tuner.should_rollback(ov_m, osc_m, 'yaw_2s1p'))
        self.assertTrue(ai_tuner.should_rollback(osc_m, dict(m, status='STABLE', overshoot_pct=3.0, crossings=1), 'yaw_2s1p'))

        # 3) History table keeps best and last_rejected even beyond recent window + sensitivity hints
        hist = [
            {'round': 1, 'parameters': {'ykp': 6.0, 'yff': 1.0, 'yki': 0.0}, 'metrics': dict(m, status='SLOW_RESPONSE', rise_ms=130.0, overshoot_pct=0.0), 'score': 45.0, 'action': 'BOOST_KP_AND_FF'},
            {'round': 2, 'parameters': {'ykp': 9.5, 'yff': 1.3, 'yki': 0.0}, 'metrics': dict(m, status='STABLE', rise_ms=52.0, overshoot_pct=3.0), 'score': 12.0, 'action': 'FINE_TUNE'},
            {'round': 3, 'parameters': {'ykp': 13.0, 'yff': 1.6, 'yki': 0.2}, 'metrics': osc_m, 'score': 88.0, 'action': 'ROLLBACK'},
        ]
        tbl = ai_tuner.format_tuning_history_table(hist, best=hist[1], last_rejected=hist[2], max_recent=1)
        self.assertIn('best|2|', tbl)
        self.assertIn('last_rejected|3|', tbl)
        sens = ai_tuner.compute_sensitivity_hints(hist)
        self.assertTrue(any('R1->R2' in s and '改善' in s for s in sens))

        # 4) Multi-strategy LLM proposal parser (nested, flat, OBSERVE, plain key=val)
        y_bounds = ai_tuner.get_loop_bounds('yaw_2s1p')
        p_flat = ai_tuner.parse_llm_proposal_response('{"ykp": 14.2, "yff": 1.9, "yki": 0.05, "status": "TUNING", "analysis_summary": "flat ok"}', y_bounds)
        self.assertIsNotNone(p_flat)
        self.assertEqual(p_flat['parameters']['ykp'], 14.2)
        p_obs = ai_tuner.parse_llm_proposal_response('{"tuning_action": "OBSERVE", "status": "TUNING", "analysis_summary": "hold"}', y_bounds)
        self.assertEqual(p_obs['tuning_action'], 'OBSERVE')
        p_txt = ai_tuner.parse_llm_proposal_response('建议调整：ykp=13.5, yff=1.85, yki=0.04 TUNING', y_bounds)
        self.assertIsNotNone(p_txt)
        self.assertEqual(p_txt['parameters']['ykp'], 13.5)

        # 5) Incremental PI physics: yaw_3c severely sluggish boosts BOTH ykp AND yki
        y3_bounds = ai_tuner.get_loop_bounds('yaw_3c')
        slow_m = {
            'status': 'SLOW_RESPONSE', 'rise_ms': 220.0, 'overshoot_pct': 0.0,
            'steady_err_pct': 18.0, 'tail_mae_pct': 19.0, 'crossings': 0, 'pwm_sat_pct': 0.0,
            'dynamics': {'osc_type': 'none', 'tau_eff_ms': 40.0},
        }
        prop_y3 = ai_tuner.build_rule_proposal('yaw_3c', {'ykp': 0.04, 'yki': 0.001}, slow_m, y3_bounds, [], set())
        self.assertGreater(prop_y3['parameters']['ykp'], 0.04)
        self.assertGreater(prop_y3['parameters']['yki'], 0.001)

        # 6) Incremental PI physics: low-freq overshoot cuts Ki strongly; high-freq chatter cuts Kp strongly
        s3_bounds = ai_tuner.get_loop_bounds('speed_3c')
        lf_m = {
            'status': 'OVERSHOOTING', 'rise_ms': 45.0, 'overshoot_pct': 22.0,
            'steady_err_pct': 2.0, 'tail_mae_pct': 4.0, 'crossings': 2, 'pwm_sat_pct': 10.0,
            'dynamics': {'osc_type': 'low_freq_i_swing', 'tau_eff_ms': 30.0},
        }
        hf_m = {
            'status': 'OSCILLATING', 'rise_ms': 28.0, 'overshoot_pct': 14.0,
            'steady_err_pct': 1.5, 'tail_mae_pct': 8.0, 'crossings': 8, 'pwm_sat_pct': 40.0,
            'dynamics': {'osc_type': 'high_freq_p_chatter', 'tau_eff_ms': 24.0},
        }
        p_lf = ai_tuner.build_rule_proposal('speed_3c', {'lkp': 14.0, 'lki': 3.0, 'rkp': 14.0, 'rki': 3.0}, lf_m, s3_bounds, [], set())
        p_hf = ai_tuner.build_rule_proposal('speed_3c', {'lkp': 36.0, 'lki': 1.5, 'rkp': 36.0, 'rki': 1.5}, hf_m, s3_bounds, [], set())
        # Low-freq overshoot cuts Ki proportionally much harder than Kp
        self.assertGreater((3.0 - p_lf['parameters']['lki']) / 3.0, (14.0 - p_lf['parameters']['lkp']) / 14.0)
        # High-freq chatter cuts Kp proportionally harder than Ki
        self.assertGreater((36.0 - p_hf['parameters']['lkp']) / 36.0, (1.5 - p_hf['parameters']['lki']) / 1.5)

        # 7) speed_3c independent L/R wheel analysis & independent tuning
        asym_m = dict(lf_m, left=dict(slow_m, osc_type='none', tau_eff_ms=34.0), right=dict(hf_m, osc_type='high_freq_p_chatter', tau_eff_ms=26.0))
        p_asym = ai_tuner.build_rule_proposal('speed_3c', {'lkp': 10.0, 'lki': 0.5, 'rkp': 34.0, 'rki': 2.0}, asym_m, s3_bounds, [], set())
        self.assertGreater(p_asym['parameters']['lkp'], 10.0)
        self.assertLess(p_asym['parameters']['rkp'], 34.0)


if __name__ == '__main__':
    unittest.main()
