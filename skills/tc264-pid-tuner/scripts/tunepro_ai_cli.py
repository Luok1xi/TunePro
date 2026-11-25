#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Zero-dependency CLI for TC264 AI tuning skill (live TunePro HTTP + offline CSV/guardrail).

Works with Python 3.10+ standard library only.
Offline commands (no server or hardware needed):
  analyze-csv   Analyze TunePro m2-decoded.csv or generic timestamp_ms,setpoint,input CSV
  check-update  Strictly validate a bounded parameter proposal JSON

Live TunePro commands (talks to local app.py, default http://127.0.0.1:18760):
  snapshot      Fetch @GP1 parameters, topology, battery, and active bounds
  step-test     Run one watchdog-protected 4 ms step capture and return metrics
  apply         Apply a bounded parameter update with strict guardrail & readback
  rollback      Restore the last verified best stable parameters
  auto-tune     Start/poll an automated tuning session
  estop         Send immediate hardware E-stop (#! + #X STOP)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import urllib.error
import urllib.parse
import urllib.request


def _finite(val, name: str) -> float:
    if isinstance(val, bool) or not isinstance(val, (int, float, str)):
        raise ValueError(f'{name} must be a finite number')
    out = float(val)
    if not math.isfinite(out):
        raise ValueError(f'{name} must be a finite number')
    return out


def _http_json(base_url: str, path: str, payload: dict | None = None, timeout: float = 15.0) -> dict:
    url = base_url.rstrip('/') + path
    data = json.dumps(payload, ensure_ascii=False).encode('utf-8') if payload is not None else None
    headers = {'Content-Type': 'application/json'} if data is not None else {}
    req = urllib.request.Request(url, data=data, headers=headers, method='POST' if data is not None else 'GET')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode('utf-8'))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode('utf-8', errors='replace')
        try:
            err_obj = json.loads(body)
            msg = err_obj.get('error') or body
        except Exception:
            msg = body
        raise RuntimeError(f'HTTP {exc.code}: {msg}') from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f'无法连接 TunePro 本地服务 ({url})：{exc.reason}') from exc


def cmd_check_update(file_path: Path) -> dict:
    spec = json.loads(file_path.read_text(encoding='utf-8-sig'))
    for key in ('current', 'proposed', 'bounds'):
        if key not in spec or not isinstance(spec[key], dict):
            raise ValueError(f'Proposal JSON must contain object field "{key}"')
    current, proposed, bounds = spec['current'], spec['proposed'], spec['bounds']
    if set(current) != set(proposed) or set(current) != set(bounds) or not current:
        raise ValueError('current, proposed, and bounds must declare the exact same non-empty parameter set')
    checked = {}
    for name in current:
        b = bounds[name]
        lo = _finite(b['min'], f'{name}.min')
        hi = _finite(b['max'], f'{name}.max')
        step = _finite(b['max_step'], f'{name}.max_step')
        old = _finite(current[name], f'current.{name}')
        new = _finite(proposed[name], f'proposed.{name}')
        if lo > hi or step < 0:
            raise ValueError(f'Invalid bounds for {name}')
        if not (lo <= old <= hi) or not (lo <= new <= hi):
            raise ValueError(f'{name} is outside declared bounds [{lo}, {hi}]')
        if abs(new - old) > step + 1e-12:
            raise ValueError(f'{name} change {new - old:+.4f} exceeds max_step {step}')
        checked[name] = new
    return {'valid': True, 'proposed': checked, 'applied': False}


def cmd_analyze_csv(csv_path: Path, channel: str) -> dict:
    with csv_path.open(encoding='utf-8-sig', newline='') as f:
        reader = csv.DictReader(f)
        fields = set(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError('CSV has no rows')

    # Detect TunePro m2-decoded.csv vs generic timestamp_ms,setpoint,input
    times: list[float] = []
    targets: list[float] = []
    outputs: list[float] = []
    pwms: list[float] = []

    if {'t', 'yaw', 'gyro', 'enc_l', 'enc_r'}.issubset(fields):
        source_format = 'tunepro_m2_csv'
        for r in rows:
            times.append(_finite(r['t'], 't'))
            if channel == 'yaw':
                targets.append(_finite(r['yaw'], 'yaw'))
                outputs.append(_finite(r['gyro'], 'gyro'))
                pwms.append(0.5 * (_finite(r['pwm_r'], 'pwm_r') - _finite(r['pwm_l'], 'pwm_l')))
            elif channel == 'outer':
                targets.append(0.0)
                outputs.append(_finite(r['error'], 'error'))
                pwms.append(0.5 * (_finite(r['pwm_r'], 'pwm_r') - _finite(r['pwm_l'], 'pwm_l')))
            else:
                targets.append(0.5 * (_finite(r['set_l'], 'set_l') + _finite(r['set_r'], 'set_r')))
                outputs.append(0.5 * (_finite(r['enc_l'], 'enc_l') + _finite(r['enc_r'], 'enc_r')))
                pwms.append(0.5 * (_finite(r['pwm_l'], 'pwm_l') + _finite(r['pwm_r'], 'pwm_r')))
    elif {'timestamp_ms', 'setpoint', 'input'}.issubset(fields):
        source_format = 'generic_pid_csv'
        for r in rows:
            times.append(_finite(r['timestamp_ms'], 'timestamp_ms') / 1000.0)
            targets.append(_finite(r['setpoint'], 'setpoint'))
            outputs.append(_finite(r['input'], 'input'))
            pwms.append(_finite(r['pwm'], 'pwm') if r.get('pwm') else 0.0)
    else:
        raise ValueError('Unrecognized CSV headers; expected TunePro m2-decoded.csv or timestamp_ms,setpoint,input')

    t0 = times[0]
    rel_t = [t - t0 for t in times]
    errs = [tgt - y for tgt, y in zip(targets, outputs)]
    abs_e = [abs(e) for e in errs]
    mae = sum(abs_e) / len(abs_e)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    tail_i = max(1, int(len(errs) * 0.8))
    tail_e = errs[tail_i:]
    tail_mae = sum(abs(e) for e in tail_e) / len(tail_e)
    tail_bias = sum(tail_e) / len(tail_e)
    tail_p2p = (max(outputs[tail_i:]) - min(outputs[tail_i:])) if len(outputs[tail_i:]) >= 2 else 0.0
    sat_pct = sum(1 for u in pwms if abs(u) >= 8330.0) / len(pwms) * 100.0

    return {
        'source': 'csv_observations',
        'format': source_format,
        'channel': channel,
        'samples': len(rows),
        'duration_s': round(rel_t[-1], 3),
        'metrics': {
            'mae': round(mae, 3),
            'rms_err': round(rms, 3),
            'tail_mae': round(tail_mae, 3),
            'tail_bias': round(tail_bias, 3),
            'tail_p2p': round(tail_p2p, 3),
            'pwm_peak': round(max(abs(u) for u in pwms), 1),
            'pwm_sat_pct': round(sat_pct, 1),
        },
        'warnings': ['CSV analysis alone does not confirm hardware safety or readback gains.'],
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description='TC264 TunePro AI Tuning CLI (stdlib only)')
    ap.add_argument('--url', default='http://127.0.0.1:18760', help='TunePro local HTTP URL')
    sub = ap.add_subparsers(dest='cmd', required=True)

    p_snap = sub.add_parser('snapshot', help='Read current car parameters, topology, and bounds')
    p_snap.add_argument('--loop', default='')
    p_snap.add_argument('--dry', action='store_true')

    p_csv = sub.add_parser('analyze-csv', help='Analyze an offline CSV run log')
    p_csv.add_argument('--csv', type=Path, required=True)
    p_csv.add_argument('--channel', choices=['speed', 'yaw', 'outer'], default='yaw')

    p_chk = sub.add_parser('check-update', help='Validate a bounded proposal JSON offline')
    p_chk.add_argument('--file', type=Path, required=True)

    p_step = sub.add_parser('step-test', help='Run one safe 4 ms step capture experiment')
    p_step.add_argument('--loop', required=True, choices=['speed_2s1p', 'yaw_2s1p', 'speed_3c', 'yaw_3c', 'outer'])
    p_step.add_argument('--target', type=int, default=None)
    p_step.add_argument('--low', type=int, default=None)
    p_step.add_argument('--dry', action='store_true')
    p_step.add_argument('--allow-real-car', action='store_true')
    p_step.add_argument('--wheels-off-ground', action='store_true')

    p_app = sub.add_parser('apply', help='Apply bounded parameter proposal with @GP1 readback')
    p_app.add_argument('--loop', required=True, choices=['speed_2s1p', 'yaw_2s1p', 'speed_3c', 'yaw_3c', 'outer'])
    p_app.add_argument('--params', required=True, help='JSON dict of parameters to apply')
    p_app.add_argument('--dry', action='store_true')
    p_app.add_argument('--allow-real-car', action='store_true')
    p_app.add_argument('--wheels-off-ground', action='store_true')

    p_rb = sub.add_parser('rollback', help='Restore best verified baseline parameters')
    p_rb.add_argument('--allow-real-car', action='store_true')
    p_rb.add_argument('--wheels-off-ground', action='store_true')

    p_auto = sub.add_parser('auto-tune', help='Start automated multi-round tuning on TunePro')
    p_auto.add_argument('--loop', required=True, choices=['speed_2s1p', 'yaw_2s1p', 'speed_3c', 'yaw_3c', 'outer'])
    p_auto.add_argument('--provider', choices=['rule', 'openai', 'ollama', 'anthropic'], default='rule')
    p_auto.add_argument('--rounds', type=int, default=6)
    p_auto.add_argument('--target', type=int, default=None)
    p_auto.add_argument('--goal', choices=['balanced', 'fast_response', 'low_overshoot', 'stability_first'], default='balanced')
    p_auto.add_argument('--dry', action='store_true')
    p_auto.add_argument('--allow-real-car', action='store_true')
    p_auto.add_argument('--wheels-off-ground', action='store_true')

    sub.add_parser('estop', help='Send immediate emergency stop (#! + #X STOP)')

    args = ap.parse_args(argv)
    try:
        if args.cmd == 'check-update':
            res = cmd_check_update(args.file)
        elif args.cmd == 'analyze-csv':
            res = cmd_analyze_csv(args.csv, args.channel)
        elif args.cmd == 'snapshot':
            qs = urllib.parse.urlencode({'loop': args.loop, 'dry': '1' if args.dry else '0'})
            res = _http_json(args.url, f'/api/ai/snapshot?{qs}')
        elif args.cmd == 'step-test':
            res = _http_json(args.url, '/api/ai/step-test', {
                'loop': args.loop,
                'target': args.target,
                'low': args.low,
                'dry_mode': args.dry,
                'allow_real_car': args.allow_real_car,
                'confirm_wheels_off_ground': args.wheels_off_ground,
            }, timeout=20.0)
        elif args.cmd == 'apply':
            params = json.loads(args.params)
            res = _http_json(args.url, '/api/ai/apply', {
                'loop': args.loop,
                'parameters': params,
                'dry_mode': args.dry,
                'allow_real_car': args.allow_real_car,
                'confirm_wheels_off_ground': args.wheels_off_ground,
                'strict': True,
            })
        elif args.cmd == 'rollback':
            res = _http_json(args.url, '/api/ai/rollback', {
                'allow_real_car': args.allow_real_car,
                'confirm_wheels_off_ground': args.wheels_off_ground,
            })
        elif args.cmd == 'auto-tune':
            res = _http_json(args.url, '/api/ai/start', {
                'loop': args.loop,
                'provider': args.provider,
                'max_rounds': args.rounds,
                'target': args.target,
                'goal': args.goal,
                'dry_mode': args.dry,
                'allow_real_car': args.allow_real_car,
                'confirm_wheels_off_ground': args.wheels_off_ground,
            })
        else:
            res = _http_json(args.url, '/api/ai/estop', {})
        print(json.dumps(res, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except Exception as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
