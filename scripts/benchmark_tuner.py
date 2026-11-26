#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Multi-scenario virtual benchmark harness for TC264 TunePro AI auto-tuning.

Runs deterministic closed-loop tuning experiments against the full 2 ms TC264
dual-topology plant simulator (demo_sim.MockCarSerial, realtime=False) across
degraded initial gains for both 2S1P and Classic 3-Cascade incremental PI loops,
comparing:
  - baseline (un-tuned initial gains)
  - built-in evidence-based rule engine (zero API key)
  - live local official AI CLI (e.g. agy / claude) or external HTTP LLM API

Usage:
  python scripts/benchmark_tuner.py --providers rule
  python scripts/benchmark_tuner.py --providers rule,local_cli --cli-tool agy --effort medium
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import ai_tuner
import demo_sim


SCENARIOS: list[dict[str, Any]] = [
    {
        'id': 'yaw_2s1p_sluggish',
        'name': '2S1P 角速度环 · 严重迟钝/欠调',
        'loop': 'yaw_2s1p',
        'topo': 1,
        'target': 300,
        'low': 0,
        'init': {'yff': 0.65, 'ykp': 3.5, 'yki': 0.0},
        'goal': 'balanced',
        'max_rounds': 6,
    },
    {
        'id': 'yaw_2s1p_oscillating',
        'name': '2S1P 角速度环 · 严重过冲/积分振荡',
        'loop': 'yaw_2s1p',
        'topo': 1,
        'target': 300,
        'low': 0,
        'init': {'yff': 3.4, 'ykp': 32.0, 'yki': 1.25},
        'goal': 'low_overshoot',
        'max_rounds': 6,
    },
    {
        'id': 'speed_2s1p_sluggish',
        'name': '2S1P 均速并联环 · 严重迟钝',
        'loop': 'speed_2s1p',
        'topo': 1,
        'target': 150,
        'low': 90,
        'init': {'vkp': 6.0, 'vki': 0.10},
        'goal': 'balanced',
        'max_rounds': 6,
    },
    {
        'id': 'speed_3c_inc_sluggish',
        'name': '三串增量式双轮速度环 · 严重欠调 + 左右失衡',
        'loop': 'speed_3c',
        'topo': 0,
        'target': 140,
        'low': 85,
        'init': {'lkp': 4.0, 'lki': 0.12, 'rkp': 34.0, 'rki': 4.8},
        'goal': 'balanced',
        'max_rounds': 6,
    },
    {
        'id': 'speed_3c_inc_i_overshoot',
        'name': '三串增量式双轮速度环 · Ki 过大低频超调',
        'loop': 'speed_3c',
        'topo': 0,
        'target': 140,
        'low': 85,
        'init': {'lkp': 6.5, 'lki': 4.8, 'rkp': 6.5, 'rki': 4.8},
        'goal': 'low_overshoot',
        'max_rounds': 6,
    },
    {
        'id': 'yaw_3c_inc_sluggish',
        'name': '三串增量式角速度中环 · 严重迟钝',
        'loop': 'yaw_3c',
        'topo': 0,
        'target': 260,
        'low': 0,
        'init': {'ykp': 0.04, 'yki': 0.001},
        'goal': 'balanced',
        'max_rounds': 6,
    },
    {
        'id': 'yaw_3c_inc_oscillating',
        'name': '三串增量式角速度中环 · 过高增益振荡',
        'loop': 'yaw_3c',
        'topo': 0,
        'target': 260,
        'low': 0,
        'init': {'ykp': 1.15, 'yki': 0.12},
        'goal': 'balanced',
        'max_rounds': 6,
    },
]


class FastMockHarness:
    """Runs exact MockCarSerial protocol & 2 ms physics at CPU speed (no wall-clock sleep)."""

    def __init__(self, topo: int, seed: int = 2026):
        self.sim = demo_sim.MockCarSerial('MOCK0', 115200, realtime=False, topo=topo)
        self.sim._rng.seed(seed)
        self._rid = 100
        self._drain()

    def _drain(self) -> list[str]:
        out = bytearray()
        while True:
            try:
                out.extend(self.sim._rx_queue.get_nowait())
            except Exception:
                break
        return out.decode('latin1', errors='ignore').splitlines()

    def _send(self, body: str) -> list[str]:
        self.sim.write(demo_sim.sign_line(body))
        return self._drain()

    def gp1(self) -> dict[str, Any]:
        self._rid += 1
        lines = self._send(f'#M {self._rid}')
        for ln in reversed(lines):
            body = demo_sim.verify_signed(ln.strip())
            if body and body.startswith('@GP1,'):
                parsed = ai_tuner.parse_gp1_line(body)
                if parsed:
                    return parsed
        raise RuntimeError('Failed to read @GP1 from fast mock')

    def write_param(self, key: str, value: float) -> dict[str, Any]:
        gp = self.gp1()
        self._rid += 1
        ival = int(round(float(value) * ai_tuner.PARAM_SCALES[key]))
        self._send(f'#W {self._rid} {gp["rev"]} {key} {ival}')
        return self.gp1()

    def apply_loop_params(self, loop_id: str, params: dict[str, float]) -> dict[str, Any]:
        gp = self.gp1()
        for key, val in ai_tuner.expand_loop_writes(loop_id, params):
            gp = self.write_param(key, val)
        return gp

    def run_step(self, loop_id: str, target: int, low: int) -> dict[str, Any]:
        profile = ai_tuner.LOOP_PROFILES[loop_id]
        if profile['kind'] == 'speed':
            self.write_param('wheel', profile['wheel'])
            self.write_param('mode', profile['mode'])
            self.write_param('test_low', low)
            gp = self.write_param('test_speed', target)
        else:
            self.write_param('mode', profile['mode'])
            gp = self.write_param('test_yaw', abs(target))

        self._rid += 1
        self._send(f'#X GO {self._rid} {gp["rev"]} 0')
        # Step 2 ms physics until timed profile finishes (max 4.0 s = 2000 ticks)
        ticks = 0
        while self.sim.running and ticks < 2100:
            self.sim.step_2ms(0.0)
            ticks += 1
        # Let plant coast down 120 ms between rounds like real car
        for _ in range(60):
            self.sim.step_2ms(0.0)
        self._drain()

        # Pull onboard 4 ms capture (#X CAP)
        cap_lines = self._send('#X CAP')
        hdr: dict[str, Any] = {}
        rows: list[dict[str, float]] = []
        for ln in cap_lines:
            s = ln.strip()
            if s.startswith('[tune2] cap begin'):
                for tok in s.split()[3:]:
                    if '=' in tok:
                        k, v = tok.split('=', 1)
                        try:
                            hdr[k] = int(v)
                        except ValueError:
                            hdr[k] = v
            elif s.startswith('u2,'):
                parts = s.split(',')
                if len(parts) >= 11:
                    rows.append({
                        'idx': float(parts[1]),
                        'target': float(parts[2]),
                        'enc_l': float(parts[3]),
                        'enc_r': float(parts[4]),
                        'pwm_l': float(parts[5]),
                        'pwm_r': float(parts[6]),
                        'gyro': float(parts[7]) / 10.0,
                        'set_l': float(parts[8]),
                        'set_r': float(parts[9]),
                        'phase': float(parts[10]),
                    })
        metrics = ai_tuner.analyze_capture_rows(hdr, rows, loop_id)
        wf_csv = ai_tuner.sample_capture_waveform(hdr, rows, loop_id, limit=20)
        return {'gp1': self.gp1(), 'metrics': metrics, 'waveform_csv': wf_csv, 'rows': len(rows)}


def run_scenario(
    scenario: dict[str, Any],
    *,
    provider: str,
    cli_tool: str = 'agy',
    model: str = '',
    effort: str = 'medium',
    base_url: str = '',
    api_key: str = '',
    max_rounds_override: int | None = None,
) -> dict[str, Any]:
    t0 = time.perf_counter()
    loop_id = scenario['loop']
    max_rounds = max_rounds_override or int(scenario['max_rounds'])
    bounds = ai_tuner.get_loop_bounds(loop_id, dry_mode=False)

    harness = FastMockHarness(topo=scenario['topo'], seed=2026)
    harness.apply_loop_params(loop_id, scenario['init'])

    history: list[dict[str, Any]] = []
    rejected: set[tuple[tuple[str, float], ...]] = set()
    best: dict[str, Any] | None = None
    last_rejected: dict[str, Any] | None = None
    best_wf = ''
    stable_streak = 0
    rollbacks = 0
    llm_calls = 0
    fallbacks = 0
    reason = 'max_rounds_reached'

    for r_idx in range(1, max_rounds + 1):
        exp = harness.run_step(loop_id, scenario['target'], scenario['low'])
        params = ai_tuner.extract_params_for_loop(exp['gp1'], loop_id)
        metrics = exp['metrics']
        wf_csv = exp['waveform_csv']
        score = ai_tuner.score_metrics(metrics, loop_id, scenario['goal'])
        ok, gaps, gap_detail = ai_tuner.describe_acceptance_gaps(metrics, loop_id, scenario['goal'])

        rec: dict[str, Any] = {
            'round': r_idx,
            'parameters': params,
            'metrics': metrics,
            'score': score,
            'acceptable': ok,
            'acceptance_detail': gap_detail,
            'action': 'BASELINE' if r_idx == 1 else 'EVALUATED',
            'summary': '',
        }

        cur_rank = ai_tuner._STATUS_RANK.get(str(metrics.get('status', '')).upper(), 5)
        best_rank = ai_tuner._STATUS_RANK.get(str(best['metrics'].get('status', '')).upper(), 5) if best is not None else 99
        if best is None or cur_rank < best_rank or (cur_rank == best_rank and score < best['score']):
            best = deepcopy(rec)
            best_wf = wf_csv

        stable_streak = (stable_streak + 1) if ok else 0

        rolled_back = False
        if (
            r_idx > 1
            and best is not None
            and ai_tuner._param_signature(params) != ai_tuner._param_signature(best['parameters'])
            and ai_tuner.should_rollback(metrics, best['metrics'], loop_id, scenario['goal'])
        ):
            rejected.add(ai_tuner._param_signature(params))
            harness.apply_loop_params(loop_id, best['parameters'])
            rec['action'] = 'ROLLBACK'
            last_rejected = deepcopy(rec)
            rollbacks += 1
            rolled_back = True
            params = dict(best['parameters'])

        history.append(rec)

        if stable_streak >= 2:
            rec['action'] = 'DONE'
            reason = 'converged'
            break

        if r_idx == max_rounds:
            if best is not None and ai_tuner._param_signature(params) != ai_tuner._param_signature(best['parameters']):
                harness.apply_loop_params(loop_id, best['parameters'])
            break

        if ok:
            rec['action'] = 'HOLD_VERIFY'
            continue

        ref_params = best['parameters'] if (rolled_back and best) else params
        ref_metrics = best['metrics'] if (rolled_back and best) else metrics
        ref_wf = best_wf if (rolled_back and best_wf) else wf_csv

        if provider == 'rule':
            prop = ai_tuner.build_rule_proposal(loop_id, ref_params, ref_metrics, bounds, history, rejected, scenario['goal'])
        else:
            llm_calls += 1
            try:
                prop = ai_tuner.call_llm_proposal(
                    provider=provider,
                    base_url=base_url,
                    model=model,
                    api_key=api_key,
                    loop_id=loop_id,
                    current=ref_params,
                    metrics=ref_metrics,
                    bounds=bounds,
                    history=history,
                    best_record=best,
                    goal=scenario['goal'],
                    user_note='',
                    reasoning_effort=effort,
                    cli_tool=cli_tool,
                    waveform_csv=ref_wf,
                    last_rejected=last_rejected,
                    rejected_sigs=rejected,
                )
                if prop.get('tuning_action') != 'OBSERVE':
                    sig = ai_tuner._param_signature(prop['parameters'])
                    if sig in rejected or sig == ai_tuner._param_signature(ref_params):
                        fallbacks += 1
                        prop = ai_tuner.build_rule_proposal(loop_id, ref_params, ref_metrics, bounds, history, rejected, scenario['goal'])
            except Exception as exc:
                fallbacks += 1
                prop = ai_tuner.build_rule_proposal(loop_id, ref_params, ref_metrics, bounds, history, rejected, scenario['goal'])
                prop['analysis_summary'] = f'[fallback: {str(exc)[:60]}] {prop["analysis_summary"]}'

        rec['summary'] = prop.get('analysis_summary', '')
        if not rolled_back:
            rec['action'] = prop.get('tuning_action', 'ADJUST_PARAMETERS')
        if prop.get('tuning_action') != 'OBSERVE':
            harness.apply_loop_params(loop_id, prop['parameters'])

    r1 = history[0]
    assert best is not None
    elapsed_s = round(time.perf_counter() - t0, 2)
    return {
        'scenario': scenario['id'],
        'name': scenario['name'],
        'loop': loop_id,
        'provider': provider,
        'elapsed_s': elapsed_s,
        'rounds': len(history),
        'reason': reason,
        'converged': reason == 'converged',
        'rollbacks': rollbacks,
        'llm_calls': llm_calls,
        'fallbacks': fallbacks,
        'initial_params': r1['parameters'],
        'best_params': best['parameters'],
        'initial_score': r1['score'],
        'best_score': best['score'],
        'score_drop_pct': round((r1['score'] - best['score']) / max(0.01, r1['score']) * 100.0, 1),
        'initial_metrics': {
            'status': r1['metrics']['status'],
            'rise_ms': r1['metrics']['rise_ms'],
            'ov': r1['metrics']['overshoot_pct'],
            'steady': r1['metrics']['steady_err_pct'],
            'cross': r1['metrics']['crossings'],
        },
        'best_metrics': {
            'status': best['metrics']['status'],
            'rise_ms': best['metrics']['rise_ms'],
            'ov': best['metrics']['overshoot_pct'],
            'steady': best['metrics']['steady_err_pct'],
            'cross': best['metrics']['crossings'],
            'acceptable': best['acceptable'],
        },
        'trace': [
            {
                'r': h['round'],
                'p': h['parameters'],
                'rise': h['metrics']['rise_ms'],
                'ov': h['metrics']['overshoot_pct'],
                'st_err': h['metrics']['steady_err_pct'],
                'cross': h['metrics']['crossings'],
                'status': h['metrics']['status'],
                'score': h['score'],
                'action': h['action'],
                'summary': h['summary'][:120],
            }
            for h in history
        ],
    }


def main() -> int:
    ap = argparse.ArgumentParser(description='TunePro virtual multi-scenario PID auto-tuning benchmark')
    ap.add_argument('--providers', default='rule', help='Comma-separated providers: rule,local_cli,openai,ollama,anthropic')
    ap.add_argument('--scenarios', default='', help='Comma-separated scenario IDs (default: all)')
    ap.add_argument('--cli-tool', default='agy')
    ap.add_argument('--model', default='')
    ap.add_argument('--effort', default='medium')
    ap.add_argument('--base-url', default='')
    ap.add_argument('--api-key', default='')
    ap.add_argument('--max-rounds', type=int, default=0)
    ap.add_argument('--json-out', default='')
    args = ap.parse_args()

    wanted_ids = {s.strip() for s in args.scenarios.split(',') if s.strip()}
    chosen = [sc for sc in SCENARIOS if not wanted_ids or sc['id'] in wanted_ids]
    providers = [p.strip() for p in args.providers.split(',') if p.strip()]

    results: list[dict[str, Any]] = []
    for prov in providers:
        print(f'\n=== Provider: {prov} (cli={args.cli_tool}, effort={args.effort}) ===', flush=True)
        for sc in chosen:
            res = run_scenario(
                sc,
                provider=prov,
                cli_tool=args.cli_tool,
                model=args.model,
                effort=args.effort,
                base_url=args.base_url,
                api_key=args.api_key,
                max_rounds_override=args.max_rounds or None,
            )
            results.append(res)
            im = res['initial_metrics']
            bm = res['best_metrics']
            print(
                f"[{sc['id']:25s}] {res['elapsed_s']:5.2f}s | R={res['rounds']} | "
                f"conv={str(res['converged']):5s} | rb={res['rollbacks']} fb={res['fallbacks']} | "
                f"score {res['initial_score']:5.1f} -> {res['best_score']:5.1f} (-{res['score_drop_pct']:4.1f}%) | "
                f"{im['status']}({im['rise_ms']}ms,{im['ov']}%,{im['steady']}%) -> "
                f"{bm['status']}({bm['rise_ms']}ms,{bm['ov']}%,{bm['steady']}%)",
                flush=True,
            )

    if args.json_out:
        Path(args.json_out).write_text(json.dumps(results, ensure_ascii=False, indent=2), encoding='utf-8')
    ai_tuner.LOCAL_CLI_POOL.close_all()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
