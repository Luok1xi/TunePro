# SPDX-License-Identifier: GPL-3.0-or-later
"""Zero-dependency AI auto-tuning engine and safety gate for TC264 TunePro.

Supports both:
  1. In-app automated closed-loop tuning (built-in evidence-based rule engine,
     OpenAI-compatible / DeepSeek / Qwen / Ollama / Anthropic HTTP APIs via
     stdlib urllib.request — zero third-party packages).
  2. Atomic bounded HTTP/CLI tools so external coding agents (via the bundled
     skills/tc264-pid-tuner skill) can inspect telemetry, trigger safe step
     captures, apply bounded gain updates with readback verification, and roll
     back to the last verified stable baseline.

Methodology and safety concepts (bounded proposals, measured-vs-predicted
separation, regression rollback, robust LLM JSON extraction) are adapted from
KINGSTON-115/llm-pid-tuner (MIT License, Copyright (c) 2026 Charles Liang; see
THIRD_PARTY_NOTICES.md).
"""
from __future__ import annotations

import base64
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Any, Callable
import urllib.error
import urllib.request

import demo_sim
import serial_bridge

PWM_LIMIT = 8500.0
VBAT_USB_MV = 6000
VBAT_FLOOR_FAN_MV = 11700
VBAT_FLOOR_DRY_MV = 10800

# Fixed-point integer scale factors matching docs/PROTOCOL.md & demo_sim.py
PARAM_SCALES: dict[str, int] = {
    'lkp': 100,
    'lki': 1000,
    'rkp': 100,
    'rki': 1000,
    'ykp': 10000,
    'yki': 10000,
    'yff': 1000,
    'dirp_cur': 1000,
    'dird_cur': 1000,
    'test_speed': 1,
    'test_low': 1,
    'test_yaw': 1,
    'race_speed': 1,
    'turn_speed': 1,
    'yaw_cap': 1,
    'fan': 1,
    'tune_fan': 1,
    'mode': 1,
    'wheel': 1,
    'step_mode': 1,
    'topo': 1,
}

# Loop definitions: topology, parameter bounds, step caps, and test profile
LOOP_PROFILES: dict[str, dict[str, Any]] = {
    'speed_2s1p': {
        'label': '两串一并 · 均速并联环 (vkp / vki)',
        'topology': '2s1p',
        'topo_val': 1,
        'kind': 'speed',
        'mode': 6,          # multi-step stair: 0 -> low -> high -> low -> 0
        'wheel': 2,         # both wheels
        'step_mode': 1,
        'default_target': 150,
        'default_low': 90,
        'target_bounds': [60, 280],
        'unit': 'cnt/2ms',
        'equations': 'PWM_speed = 6.0*v_des + vkp*(v_des - v_avg) + sum(vki*(v_des - v_avg)), v_avg = 0.5*(encL + encR), 500 Hz',
        'bounds': {
            'vkp': {'min': 4.0, 'max': 80.0, 'max_step': 8.0, 'max_ratio': 1.6, 'decimals': 2},
            'vki': {'min': 0.0, 'max': 6.0, 'max_step': 0.6, 'max_ratio': 1.8, 'decimals': 3},
        },
        'acceptance': {
            'max_overshoot_pct': 8.0,
            'max_rise_ms': 95.0,
            'max_steady_err_pct': 4.0,
            'max_tail_mae_pct': 5.0,
            'max_crossings': 4,
            'max_sat_pct': 35.0,
        },
    },
    'yaw_2s1p': {
        'label': '两串一并 · 角速度直出 PWM 内环 (yff / ykp / yki)',
        'topology': '2s1p',
        'topo_val': 1,
        'kind': 'yaw',
        'mode': 7,          # alternating yaw: 0 -> +w -> -w -> +w -> 0
        'wheel': 2,
        'step_mode': 2,
        'default_target': 300,
        'default_low': 0,
        'target_bounds': [120, 550],
        'unit': '°/s',
        'equations': 'PWM_turn = yff*w_des + ykp*(w_des - gyro_z) + sum(yki*(w_des - gyro_z)), 500 Hz direct PWM',
        'bounds': {
            'yff': {'min': 0.2, 'max': 5.0, 'max_step': 0.45, 'max_ratio': 1.5, 'decimals': 3},
            'ykp': {'min': 1.0, 'max': 36.0, 'max_step': 3.5, 'max_ratio': 1.6, 'decimals': 2},
            'yki': {'min': 0.0, 'max': 1.5, 'max_step': 0.15, 'max_ratio': 1.8, 'decimals': 3},
        },
        'dry_overrides': {
            'ykp': {'max': 14.0, 'max_step': 2.0},
            'yki': {'max': 0.5, 'max_step': 0.08},
        },
        'acceptance': {
            'max_overshoot_pct': 10.0,
            'max_rise_ms': 70.0,
            'max_steady_err_pct': 4.5,
            'max_tail_mae_pct': 6.0,
            'max_crossings': 4,
            'max_sat_pct': 35.0,
        },
    },
    'speed_3c': {
        'label': '传统三串 · 双轮速度内环 (lkp / lki / rkp / rki)',
        'topology': '3cascade',
        'topo_val': 0,
        'kind': 'speed',
        'mode': 6,
        'wheel': 2,
        'step_mode': 1,
        'default_target': 140,
        'default_low': 85,
        'target_bounds': [60, 260],
        'unit': 'cnt/2ms',
        'equations': 'Incremental wheel PI: dPWM = kp*(e - e_prev) + ki*e, 500 Hz independent L/R',
        'bounds': {
            'lkp': {'min': 2.0, 'max': 50.0, 'max_step': 4.5, 'max_ratio': 1.6, 'decimals': 2},
            'lki': {'min': 0.05, 'max': 6.0, 'max_step': 0.5, 'max_ratio': 1.8, 'decimals': 3},
            'rkp': {'min': 2.0, 'max': 50.0, 'max_step': 4.5, 'max_ratio': 1.6, 'decimals': 2},
            'rki': {'min': 0.05, 'max': 6.0, 'max_step': 0.5, 'max_ratio': 1.8, 'decimals': 3},
        },
        'acceptance': {
            'max_overshoot_pct': 9.0,
            'max_rise_ms': 105.0,
            'max_steady_err_pct': 4.5,
            'max_tail_mae_pct': 5.5,
            'max_crossings': 4,
            'max_sat_pct': 35.0,
        },
    },
    'yaw_3c': {
        'label': '传统三串 · 角速度中环 (ykp / yki)',
        'topology': '3cascade',
        'topo_val': 0,
        'kind': 'yaw',
        'mode': 7,
        'wheel': 2,
        'step_mode': 2,
        'default_target': 260,
        'default_low': 0,
        'target_bounds': [100, 450],
        'unit': '°/s',
        'equations': 'Incremental yaw PI -> wheel diff target: dDiff = ykp*(e - e_prev) + yki*e, 500 Hz',
        'bounds': {
            'ykp': {'min': 0.02, 'max': 1.5, 'max_step': 0.14, 'max_ratio': 1.6, 'decimals': 4},
            'yki': {'min': 0.0, 'max': 0.15, 'max_step': 0.015, 'max_ratio': 1.8, 'decimals': 4},
        },
        'acceptance': {
            'max_overshoot_pct': 12.0,
            'max_rise_ms': 110.0,
            'max_steady_err_pct': 5.0,
            'max_tail_mae_pct': 7.0,
            'max_crossings': 4,
            'max_sat_pct': 35.0,
        },
    },
    'outer': {
        'label': '视觉外环 · 巡线 PD (dirp_cur / dird_cur)',
        'topology': 'both',
        'topo_val': None,
        'kind': 'outer',
        'mode': 0,
        'wheel': 2,
        'step_mode': 3,
        'default_target': 125,
        'default_low': 80,
        'target_bounds': [60, 220],
        'unit': 'px',
        'equations': 'w_des = clip(dirp_cur * err_px + dird_cur * d_err_px, -yaw_cap, yaw_cap)',
        'bounds': {
            'dirp_cur': {'min': 1.0, 'max': 40.0, 'max_step': 2.5, 'max_ratio': 1.5, 'decimals': 2},
            'dird_cur': {'min': 0.0, 'max': 40.0, 'max_step': 3.0, 'max_ratio': 1.6, 'decimals': 2},
        },
        'acceptance': {
            'max_overshoot_pct': 15.0,
            'max_rise_ms': 180.0,
            'max_steady_err_pct': 10.0,
            'max_tail_mae_pct': 18.0,
            'max_crossings': 6,
            'max_sat_pct': 45.0,
        },
    },
}

GOAL_PRESETS: dict[str, dict[str, float]] = {
    'balanced': {'overshoot_w': 1.0, 'rise_w': 1.0, 'steady_w': 1.2, 'osc_w': 1.0, 'max_overshoot_scale': 1.0},
    'fast_response': {'overshoot_w': 0.7, 'rise_w': 1.6, 'steady_w': 1.0, 'osc_w': 0.9, 'max_overshoot_scale': 1.25},
    'low_overshoot': {'overshoot_w': 1.8, 'rise_w': 0.7, 'steady_w': 1.2, 'osc_w': 1.3, 'max_overshoot_scale': 0.65},
    'stability_first': {'overshoot_w': 1.6, 'rise_w': 0.6, 'steady_w': 1.1, 'osc_w': 1.8, 'max_overshoot_scale': 0.70},
}

REASONING_LEVELS: dict[str, dict[str, Any]] = {
    'fast':   {'label': '极速', 'en': 'Fast',   'default_rounds': 3,  'max_tokens': 320, 'temp': 0.25, 'depth': '简明直给，优先快速修正最明显问题。'},
    'low':    {'label': '轻量', 'en': 'Low',    'default_rounds': 4,  'max_tokens': 420, 'temp': 0.22, 'depth': '关注主要阶跃指标（上升时间与超调），给出稳妥微调。'},
    'medium': {'label': '均衡', 'en': 'Medium', 'default_rounds': 6,  'max_tokens': 600, 'temp': 0.20, 'depth': '综合权衡上升时间、超调、稳态静差与 PWM 饱和占比。'},
    'high':   {'label': '深度', 'en': 'High',   'default_rounds': 8,  'max_tokens': 850, 'temp': 0.16, 'depth': '深度对比历史轮次敏感度、共模/差模耦合与抗饱和裕度。'},
    'max':    {'label': '极限', 'en': 'Max',    'default_rounds': 10, 'max_tokens': 1100, 'temp': 0.12, 'depth': '逐项做精细物理诊断，分析相位滞后、静摩擦死区、转向优先饱和转移与多轮稳定裕度。'},
}


def normalize_effort(val: Any) -> str:
    key = str(val or 'medium').strip().lower()
    return key if key in REASONING_LEVELS else 'medium'


def _find_cli_binary(tool: str) -> str | None:
    """Locate an officially installed local AI CLI using PATH and standard install locations."""
    home = Path.home()
    appdata = Path(os.environ.get('APPDATA') or (home / 'AppData' / 'Roaming'))
    localapp = Path(os.environ.get('LOCALAPPDATA') or (home / 'AppData' / 'Local'))

    candidates_map: dict[str, list[str | Path]] = {
        'claude': [
            'claude',
            appdata / 'npm' / 'claude.cmd',
            home / '.local' / 'bin' / 'claude.exe',
            home / '.local' / 'bin' / 'claude',
        ],
        'codex': [
            'codex',
            appdata / 'npm' / 'codex.cmd',
            home / '.codex' / '.sandbox-bin' / 'codex.exe',
            home / '.local' / 'bin' / 'codex.exe',
            home / '.local' / 'bin' / 'codex',
        ],
        'agy': [
            'agy',
            appdata / 'npm' / 'agy.cmd',
            home / '.gemini' / 'antigravity-cli' / 'bin' / 'agy.exe',
            home / '.gemini' / 'antigravity-cli' / 'bin' / 'agy',
            localapp / 'Programs' / 'antigravity' / 'bin' / 'agy.cmd',
            home / '.local' / 'bin' / 'agy',
        ],
        'gemini': [
            'gemini',
            appdata / 'npm' / 'gemini.cmd',
            home / '.local' / 'bin' / 'gemini.exe',
            home / '.local' / 'bin' / 'gemini',
        ],
    }
    for cand in candidates_map.get(tool, []):
        if isinstance(cand, str):
            hit = shutil.which(cand)
            if hit:
                return hit
        elif cand.is_file():
            return str(cand)
    return None


def detect_local_clis() -> list[dict[str, Any]]:
    """Probe installed official local AI CLIs (uses the user's own official account quota)."""
    specs = [
        ('claude', 'Claude Code', '本机已登录 Claude 官方额度'),
        ('codex', 'Codex CLI', '本机已登录 OpenAI / Codex 官方额度'),
        ('agy', 'Antigravity (agy)', '本机已登录 Google Antigravity 官方额度'),
        ('gemini', 'Gemini CLI', '本机已登录 Google Gemini 官方额度'),
    ]
    out: list[dict[str, Any]] = []
    for tool_id, label, desc in specs:
        path = _find_cli_binary(tool_id)
        out.append({
            'id': tool_id,
            'label': label,
            'desc': desc,
            'available': bool(path),
            'path': path or '',
        })
    return out


def call_local_cli_text(
    *,
    cli_tool: str,
    model: str,
    reasoning_effort: str,
    system_prompt: str,
    user_prompt: str,
    timeout_s: float = 90.0,
) -> tuple[str, str]:
    """Invoke a locally installed, officially authenticated AI CLI non-interactively."""
    eff_key = normalize_effort(reasoning_effort)
    detected = detect_local_clis()
    avail = {item['id']: item for item in detected if item['available']}

    wanted = str(cli_tool or 'auto').strip().lower()
    if wanted in ('', 'auto'):
        chosen_id = next((t for t in ('claude', 'codex', 'agy', 'gemini') if t in avail), '')
    elif wanted in avail:
        chosen_id = wanted
    else:
        chosen_id = ''

    if not chosen_id:
        raise RuntimeError('未检测到可用的本机官方 AI CLI（请确认已安装并登录 claude / codex / agy / gemini）。')

    exe = avail[chosen_id]['path']
    full_prompt = f'{system_prompt}\n\n{user_prompt}'
    creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
    env = dict(os.environ)
    env.setdefault('NO_COLOR', '1')

    def _run_cmd(cmd: list[str], stdin_text: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            cmd,
            input=stdin_text,
            capture_output=True,
            text=True,
            encoding='utf-8',
            errors='replace',
            timeout=timeout_s,
            creationflags=creationflags,
            env=env,
        )

    if chosen_id == 'claude':
        effort_map = {'fast': 'low', 'low': 'low', 'medium': 'medium', 'high': 'high', 'max': 'max'}
        cmd = [
            exe, '-p',
            '--output-format', 'text',
            '--no-session-persistence',
            '--tools', '',
            '--effort', effort_map[eff_key],
        ]
        if model:
            cmd.extend(['--model', model])
        proc = _run_cmd(cmd, full_prompt)
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or '').strip()[:180]
            raise RuntimeError(f'Claude Code 返回错误 ({proc.returncode})：{err}')
        text = (proc.stdout or '').strip()
        return text, f'Claude Code ({model or "官方默认"})'

    if chosen_id == 'codex':
        effort_map = {'fast': 'low', 'low': 'low', 'medium': 'medium', 'high': 'high', 'max': 'xhigh'}
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(prefix='tunepro_codex_', suffix='.txt', delete=False) as tf:
                tmp_path = tf.name
            cmd = [
                exe, 'exec',
                '--ephemeral',
                '--skip-git-repo-check',
                '--sandbox', 'read-only',
                '--color', 'never',
                '-o', tmp_path,
                '-c', f'model_reasoning_effort="{effort_map[eff_key]}"',
            ]
            if model:
                cmd.extend(['-m', model])
            cmd.append('-')
            proc = _run_cmd(cmd, full_prompt)
            out_txt = ''
            if tmp_path and Path(tmp_path).is_file():
                out_txt = Path(tmp_path).read_text(encoding='utf-8', errors='replace').strip()
            if proc.returncode != 0 and not out_txt:
                err = (proc.stderr or proc.stdout or '').strip()[:180]
                raise RuntimeError(f'Codex CLI 返回错误 ({proc.returncode})：{err}')
            text = out_txt or (proc.stdout or '').strip()
            return text, f'Codex ({model or "官方默认"})'
        finally:
            if tmp_path:
                try:
                    Path(tmp_path).unlink(missing_ok=True)
                except OSError:
                    pass

    if chosen_id == 'agy':
        cmd = [exe, '-p', full_prompt]
        if model:
            cmd.extend(['--model', model])
        proc = _run_cmd(cmd, '')
        if proc.returncode != 0:
            err = (proc.stderr or proc.stdout or '').strip()[:180]
            raise RuntimeError(f'Antigravity CLI 返回错误 ({proc.returncode})：{err}')
        return (proc.stdout or '').strip(), f'Antigravity ({model or "官方默认"})'

    # gemini CLI
    cmd = [exe, '-p', '']
    if model:
        cmd.extend(['-m', model])
    proc = _run_cmd(cmd, full_prompt)
    if proc.returncode != 0:
        err = (proc.stderr or proc.stdout or '').strip()[:180]
        raise RuntimeError(f'Gemini CLI 返回错误 ({proc.returncode})：{err}')
    return (proc.stdout or '').strip(), f'Gemini CLI ({model or "官方默认"})'


def _finite_float(val: Any, name: str) -> float:
    if isinstance(val, bool) or not isinstance(val, (int, float, str)):
        raise ValueError(f'{name} 必须是有限数值')
    out = float(val)
    if not math.isfinite(out):
        raise ValueError(f'{name} 必须是有限数值')
    return out


def get_loop_bounds(loop_id: str, *, dry_mode: bool = False) -> dict[str, dict[str, float]]:
    if loop_id not in LOOP_PROFILES:
        raise ValueError(f'不支持的控制环路：{loop_id}')
    profile = LOOP_PROFILES[loop_id]
    bounds = {k: dict(v) for k, v in profile['bounds'].items()}
    if dry_mode and 'dry_overrides' in profile:
        for k, ov in profile['dry_overrides'].items():
            if k in bounds:
                bounds[k].update(ov)
    return bounds


def check_and_clamp_proposal(
    current: dict[str, float],
    proposed: dict[str, Any],
    bounds: dict[str, dict[str, float]],
    *,
    strict: bool = False,
) -> tuple[dict[str, float], list[str]]:
    """Validate and optionally clamp candidate parameters against hard bounds & step limits.

    When strict=True (used by check-update / external agent /api/ai/apply), any
    out-of-bounds or excessive step raises ValueError instead of silently clipping.
    When strict=False (used inside the in-app LLM loop), unsafe jumps are clamped
    with explicit human-readable guardrail notes.
    """
    notes: list[str] = []
    sanitized: dict[str, float] = {}
    for key, spec in bounds.items():
        if key not in current:
            raise ValueError(f'缺少当前参数 {key}')
        cur = _finite_float(current[key], f'current.{key}')
        lo = float(spec['min'])
        hi = float(spec['max'])
        max_step = float(spec['max_step'])
        max_ratio = float(spec.get('max_ratio', 0.0))
        decimals = int(spec.get('decimals', 3))

        if key not in proposed:
            if strict:
                raise ValueError(f'缺少候选参数 {key}')
            sanitized[key] = round(cur, decimals)
            continue

        raw = _finite_float(proposed[key], f'proposed.{key}')
        val = raw

        if val < lo or val > hi:
            if strict:
                raise ValueError(f'{key}={val} 超出安全范围 [{lo}, {hi}]')
            val = max(lo, min(hi, val))
            notes.append(f'{key} 超界，已限幅至 {val:.{decimals}f} (允许 [{lo}, {hi}])')

        delta = val - cur
        if abs(delta) > max_step + 1e-9:
            if strict:
                raise ValueError(f'{key} 单轮变化 {delta:+.4f} 超过最大步长 ±{max_step}')
            val = cur + math.copysign(max_step, delta)
            notes.append(f'{key} 单轮步长过大，已限步至 {val:.{decimals}f} (±{max_step})')

        if max_ratio > 1.0 and cur > 1e-6 and val > cur * max_ratio + 1e-9:
            if strict:
                raise ValueError(f'{key} 增幅超过单轮最大倍率 {max_ratio}x')
            val = min(hi, cur * max_ratio)
            notes.append(f'{key} 增幅超 {max_ratio}x，已限至 {val:.{decimals}f}')

        val = round(max(lo, min(hi, val)), decimals)
        sanitized[key] = val

    extra = set(proposed.keys()) - set(bounds.keys())
    if extra and strict:
        raise ValueError(f'包含当前环路未声明的参数：{", ".join(sorted(extra))}')

    return sanitized, notes


def extract_params_for_loop(gp1: dict[str, Any], loop_id: str) -> dict[str, float]:
    if loop_id == 'speed_2s1p':
        return {
            'vkp': round(0.5 * (float(gp1['lkp']) + float(gp1['rkp'])), 2),
            'vki': round(0.5 * (float(gp1['lki']) + float(gp1['rki'])), 3),
        }
    if loop_id == 'yaw_2s1p':
        return {
            'yff': round(float(gp1.get('yff', 1.85)), 3),
            'ykp': round(float(gp1['ykp']), 2),
            'yki': round(float(gp1['yki']), 3),
        }
    if loop_id == 'speed_3c':
        return {
            'lkp': round(float(gp1['lkp']), 2),
            'lki': round(float(gp1['lki']), 3),
            'rkp': round(float(gp1['rkp']), 2),
            'rki': round(float(gp1['rki']), 3),
        }
    if loop_id == 'yaw_3c':
        return {
            'ykp': round(float(gp1['ykp']), 4),
            'yki': round(float(gp1['yki']), 4),
        }
    if loop_id == 'outer':
        return {
            'dirp_cur': round(float(gp1['dirp_cur']), 2),
            'dird_cur': round(float(gp1['dird_cur']), 2),
        }
    raise ValueError(f'未知环路：{loop_id}')


def expand_loop_writes(loop_id: str, params: dict[str, float]) -> list[tuple[str, float]]:
    """Expand high-level loop parameters into concrete firmware #W keys."""
    if loop_id == 'speed_2s1p':
        writes: list[tuple[str, float]] = []
        if 'vkp' in params:
            writes.extend([('lkp', params['vkp']), ('rkp', params['vkp'])])
        if 'vki' in params:
            writes.extend([('lki', params['vki']), ('rki', params['vki'])])
        return writes
    return [(k, float(v)) for k, v in params.items()]


def parse_gp1_line(body: str) -> dict[str, Any] | None:
    parts = body.split(',')
    if not parts or parts[0] != '@GP1' or len(parts) < 25:
        return None
    nums = [int(x) for x in parts[1:]]
    topo = nums[24] if len(nums) >= 25 else 0
    yff = (nums[25] / 1000.0) if len(nums) >= 26 else (1.85 if topo == 1 else 0.13)
    return {
        'rid': nums[0],
        'rev': nums[1],
        'mode': nums[2],
        'wheel': nums[3],
        'step_mode': nums[4],
        'race_speed': nums[5],
        'test_speed': nums[6],
        'test_low': nums[7],
        'test_yaw': nums[8],
        'fan_duty': nums[9],
        'fan_pct': round(nums[9] / 100.0, 1),
        'tune_fan': nums[10],
        'lkp': nums[11] / 100.0,
        'lki': nums[12] / 1000.0,
        'rkp': nums[13] / 100.0,
        'rki': nums[14] / 1000.0,
        'ykp': nums[15] / 10000.0,
        'yki': nums[16] / 10000.0,
        'turn_speed': nums[17],
        'yaw_cap': nums[18],
        'dirp_cur': nums[19] / 1000.0,
        'dird_cur': nums[20] / 1000.0,
        'img_mode': nums[21],
        'img_fps': nums[22],
        'dur_ms': nums[23],
        'topo': topo,
        'topology': '2s1p' if topo == 1 else '3cascade',
        'yff': yff,
    }


def analyze_capture_rows(
    header: dict[str, Any],
    rows: list[dict[str, float]],
    loop_id: str,
) -> dict[str, Any]:
    """Compute step-response metrics from onboard [tune2] 4 ms capture rows.

    For multi-step profiles (step-stair / yaw-alt), automatically skips the 1st
    edge (static friction deadzone) and evaluates the 2nd clean dynamic edge.
    """
    if len(rows) < 8:
        raise ValueError(f'抓拍点数不足（仅 {len(rows)} 点），无法评估响应')
    dt_ms = float(header.get('dt_ms', 4.0) or 4.0)
    profile = LOOP_PROFILES[loop_id]
    kind = profile['kind']

    targets = [float(r['target']) for r in rows]
    if kind == 'yaw':
        measured = [float(r['gyro']) for r in rows]
    elif kind == 'speed':
        side = int(header.get('side', 3) or 3)
        if side == 1:
            measured = [float(r['enc_l']) for r in rows]
        elif side == 2:
            measured = [float(r['enc_r']) for r in rows]
        else:
            measured = [0.5 * (float(r['enc_l']) + float(r['enc_r'])) for r in rows]
    else:
        # Outer loop uses gyro tracking against target yaw, plus image error if available
        measured = [float(r['gyro']) for r in rows]

    pwm_l = [float(r['pwm_l']) for r in rows]
    pwm_r = [float(r['pwm_r']) for r in rows]
    pwm_cm = [0.5 * (a + b) for a, b in zip(pwm_l, pwm_r)]
    pwm_dm = [0.5 * (b - a) for a, b in zip(pwm_l, pwm_r)]

    # Detect step transitions in target
    edges: list[int] = []
    for i in range(1, len(targets)):
        if abs(targets[i] - targets[i - 1]) >= max(5.0, abs(targets[i]) * 0.15):
            edges.append(i)

    # Prefer the 2nd dynamic transition on multi-step (stair or yaw-alt) to skip static friction
    cap_profile = str(header.get('profile', '') or '')
    if len(edges) >= 2 and cap_profile in ('step-stair', 'yaw-alt'):
        seg_start = edges[1]
        seg_end = edges[2] if len(edges) >= 3 else len(rows)
        edge_used = 'second_dynamic_edge'
    elif edges:
        seg_start = edges[0]
        seg_end = edges[1] if len(edges) >= 2 else len(rows)
        edge_used = 'first_edge'
    else:
        seg_start = 0
        seg_end = len(rows)
        edge_used = 'full_window'

    # Keep at least 12 samples in the evaluated segment
    if seg_end - seg_start < 12:
        seg_start = 0
        seg_end = len(rows)
        edge_used = 'full_window'

    seg_t = [(i - seg_start) * dt_ms for i in range(seg_start, seg_end)]
    seg_tgt = targets[seg_start:seg_end]
    seg_y = measured[seg_start:seg_end]
    seg_pl = pwm_l[seg_start:seg_end]
    seg_pr = pwm_r[seg_start:seg_end]
    seg_cm = pwm_cm[seg_start:seg_end]
    seg_dm = pwm_dm[seg_start:seg_end]

    y0 = measured[max(0, seg_start - 1)]
    target_val = seg_tgt[min(2, len(seg_tgt) - 1)]
    amp = abs(target_val - y0)
    direction = 1.0 if target_val >= y0 else -1.0

    errors = [t - y for t, y in zip(seg_tgt, seg_y)]
    abs_err = [abs(e) for e in errors]
    mae = sum(abs_err) / len(abs_err)
    rms_err = math.sqrt(sum(e * e for e in errors) / len(errors))

    # Tail window = final 35% of the step plateau
    tail_idx = max(1, int(len(seg_y) * 0.65))
    tail_err = errors[tail_idx:]
    tail_y = seg_y[tail_idx:]
    tail_mae = sum(abs(e) for e in tail_err) / len(tail_err)
    steady_err = sum(tail_err) / len(tail_err)
    tail_p2p = (max(tail_y) - min(tail_y)) if len(tail_y) >= 2 else 0.0

    overshoot_pct = 0.0
    rise_ms = seg_t[-1]
    settling_ms = seg_t[-1]
    band = max(2.5 if kind == 'speed' else 10.0, amp * 0.05)

    if amp > 1e-3:
        excursion = max(direction * (y - target_val) for y in seg_y)
        overshoot_pct = max(0.0, excursion / amp * 100.0)
        progress = [direction * (y - y0) / amp for y in seg_y]
        t10 = next((t for t, p in zip(seg_t, progress) if p >= 0.10), None)
        t90 = next((t for t, p in zip(seg_t, progress) if p >= 0.90), None)
        if t10 is not None and t90 is not None and t90 >= t10:
            rise_ms = t90 - t10
        else:
            rise_ms = seg_t[-1]
        last_out = max((i for i, e in enumerate(abs_err) if e > band), default=-1)
        if last_out + 1 < len(seg_t) - 1:
            settling_ms = seg_t[last_out + 1]

    # Count target crossings after reaching 50% progress (oscillation detector)
    hyst = max(2.0 if kind == 'speed' else 8.0, amp * 0.025)
    crossings = 0
    last_sign = 0
    start_cross = max(1, int(len(seg_y) * 0.15))
    for e in errors[start_cross:]:
        if e > hyst:
            s = 1
        elif e < -hyst:
            s = -1
        else:
            continue
        if last_sign != 0 and s != last_sign:
            crossings += 1
        last_sign = s

    sat_count = sum(1 for a, b in zip(seg_pl, seg_pr) if abs(a) >= PWM_LIMIT * 0.98 or abs(b) >= PWM_LIMIT * 0.98)
    pwm_sat_pct = sat_count / len(seg_pl) * 100.0
    pwm_cm_peak = max(abs(x) for x in seg_cm)
    pwm_dm_peak = max(abs(x) for x in seg_dm)

    ref_scale = max(1.0, abs(target_val), amp)
    steady_err_pct = abs(steady_err) / ref_scale * 100.0
    tail_mae_pct = tail_mae / ref_scale * 100.0
    tail_p2p_pct = tail_p2p / ref_scale * 100.0

    # Stability classification
    acc = profile['acceptance']
    if crossings >= 6 or tail_p2p_pct > 18.0:
        status = 'OSCILLATING'
    elif overshoot_pct > max(acc['max_overshoot_pct'] * 1.4, 15.0):
        status = 'OVERSHOOTING'
    elif pwm_sat_pct > 65.0 and steady_err_pct > 8.0:
        status = 'SATURATED'
    elif rise_ms > acc['max_rise_ms'] * 1.35 or (steady_err_pct > 8.0 and overshoot_pct < 2.0):
        status = 'SLOW_RESPONSE'
    else:
        status = 'STABLE'

    return {
        'loop': loop_id,
        'edge_used': edge_used,
        'sample_count': len(seg_y),
        'dt_ms': dt_ms,
        'duration_ms': round(seg_t[-1], 1),
        'y0': round(y0, 2),
        'target': round(target_val, 2),
        'amplitude': round(amp, 2),
        'rise_ms': round(rise_ms, 1),
        'settling_ms': round(settling_ms, 1),
        'overshoot_pct': round(overshoot_pct, 2),
        'steady_err': round(steady_err, 2),
        'steady_err_pct': round(steady_err_pct, 2),
        'mae': round(mae, 2),
        'rms_err': round(rms_err, 2),
        'tail_mae': round(tail_mae, 2),
        'tail_mae_pct': round(tail_mae_pct, 2),
        'tail_p2p': round(tail_p2p, 2),
        'crossings': int(crossings),
        'pwm_sat_pct': round(pwm_sat_pct, 1),
        'pwm_cm_peak': round(pwm_cm_peak, 0),
        'pwm_dm_peak': round(pwm_dm_peak, 0),
        'status': status,
    }


def analyze_outer_m2(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Evaluate outer vision tracking loop from @M2 50 Hz frames."""
    if len(rows) < 20:
        raise ValueError(f'外环遥测样本不足（仅 {len(rows)} 帧）')
    # Skip initial 20% launch transient
    seg = rows[max(1, len(rows) // 5):]
    errs = [float(r['error']) for r in seg]
    abs_e = [abs(e) for e in errs]
    mae = sum(abs_e) / len(abs_e)
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    peak = max(abs_e)
    yaw_err = [abs(float(r['yaw']) - float(r['gyro'])) for r in seg]
    yaw_mae = sum(yaw_err) / len(yaw_err)

    crossings = 0
    last_s = 0
    for e in errs:
        if e > 4.0:
            s = 1
        elif e < -4.0:
            s = -1
        else:
            continue
        if last_s != 0 and s != last_s:
            crossings += 1
        last_s = s

    sat = sum(1 for r in seg if abs(float(r['pwm_l'])) >= PWM_LIMIT * 0.98 or abs(float(r['pwm_r'])) >= PWM_LIMIT * 0.98)
    sat_pct = sat / len(seg) * 100.0
    pwm_cm_peak = max(abs(0.5 * (float(r['pwm_l']) + float(r['pwm_r']))) for r in seg)
    pwm_dm_peak = max(abs(0.5 * (float(r['pwm_r']) - float(r['pwm_l']))) for r in seg)

    if crossings >= 10 or peak > 65.0:
        status = 'OSCILLATING'
    elif mae > 24.0:
        status = 'SLOW_RESPONSE'
    else:
        status = 'STABLE'

    return {
        'loop': 'outer',
        'edge_used': 'm2_tracking_window',
        'sample_count': len(seg),
        'dt_ms': 20.0,
        'duration_ms': round((seg[-1]['t'] - seg[0]['t']) * 1000.0, 1),
        'y0': 0.0,
        'target': 0.0,
        'amplitude': 50.0,
        'rise_ms': round(yaw_mae * 0.5, 1),
        'settling_ms': 0.0,
        'overshoot_pct': round(max(0.0, (peak - 30.0) / 30.0 * 100.0), 2),
        'steady_err': round(sum(errs) / len(errs), 2),
        'steady_err_pct': round(abs(sum(errs) / len(errs)) / 50.0 * 100.0, 2),
        'mae': round(mae, 2),
        'rms_err': round(rms, 2),
        'tail_mae': round(mae, 2),
        'tail_mae_pct': round(mae / 50.0 * 100.0, 2),
        'tail_p2p': round(max(errs) - min(errs), 2),
        'crossings': int(crossings),
        'pwm_sat_pct': round(sat_pct, 1),
        'pwm_cm_peak': round(pwm_cm_peak, 0),
        'pwm_dm_peak': round(pwm_dm_peak, 0),
        'status': status,
    }


def score_metrics(metrics: dict[str, Any], loop_id: str, goal: str = 'balanced') -> float:
    """Lower is better. Combines normalized tracking error, rise speed, overshoot, oscillation, saturation."""
    weights = GOAL_PRESETS.get(goal, GOAL_PRESETS['balanced'])
    acc = LOOP_PROFILES[loop_id]['acceptance']

    status = str(metrics.get('status', 'UNKNOWN')).upper()
    penalty = 0.0
    if status == 'OSCILLATING':
        penalty = 35.0
    elif status == 'OVERSHOOTING':
        penalty = 18.0
    elif status == 'SATURATED':
        penalty = 22.0
    elif status == 'SLOW_RESPONSE':
        penalty = 8.0
    elif status != 'STABLE':
        penalty = 40.0

    rise_norm = float(metrics.get('rise_ms', 200.0)) / max(10.0, float(acc['max_rise_ms'])) * 10.0
    ov_norm = float(metrics.get('overshoot_pct', 0.0)) * 1.1
    steady_norm = float(metrics.get('steady_err_pct', 0.0)) * 2.0 + float(metrics.get('tail_mae_pct', 0.0)) * 1.6
    osc_norm = max(0, int(metrics.get('crossings', 0)) - 2) * 3.2
    sat_norm = max(0.0, float(metrics.get('pwm_sat_pct', 0.0)) - 25.0) * 0.25

    score = (
        rise_norm * weights['rise_w']
        + ov_norm * weights['overshoot_w']
        + steady_norm * weights['steady_w']
        + osc_norm * weights['osc_w']
        + sat_norm
        + penalty
    )
    return round(score, 3)


def is_acceptable(metrics: dict[str, Any], loop_id: str, goal: str = 'balanced') -> bool:
    acc = LOOP_PROFILES[loop_id]['acceptance']
    weights = GOAL_PRESETS.get(goal, GOAL_PRESETS['balanced'])
    max_ov = acc['max_overshoot_pct'] * weights['max_overshoot_scale']
    return (
        str(metrics.get('status', '')).upper() == 'STABLE'
        and float(metrics.get('overshoot_pct', 999.0)) <= max_ov
        and float(metrics.get('rise_ms', 9999.0)) <= acc['max_rise_ms']
        and float(metrics.get('steady_err_pct', 999.0)) <= acc['max_steady_err_pct']
        and float(metrics.get('tail_mae_pct', 999.0)) <= acc['max_tail_mae_pct']
        and int(metrics.get('crossings', 99)) <= acc['max_crossings']
        and float(metrics.get('pwm_sat_pct', 100.0)) <= acc['max_sat_pct']
    )


def should_rollback(current: dict[str, Any], best: dict[str, Any], loop_id: str, goal: str = 'balanced') -> bool:
    cur_score = score_metrics(current, loop_id, goal)
    best_score = score_metrics(best, loop_id, goal)
    if cur_score <= best_score:
        return False
    best_st = str(best.get('status', '')).upper()
    cur_st = str(current.get('status', '')).upper()
    if best_st == 'STABLE' and cur_st in ('OSCILLATING', 'SATURATED'):
        return True
    if float(current.get('overshoot_pct', 0.0)) > float(best.get('overshoot_pct', 0.0)) + 9.0:
        return True
    if cur_score > max(best_score * 1.35, best_score + 8.0):
        return True
    return False


def _param_signature(params: dict[str, float]) -> tuple[tuple[str, float], ...]:
    return tuple((k, round(float(v), 4)) for k, v in sorted(params.items()))


def build_rule_proposal(
    loop_id: str,
    current: dict[str, float],
    metrics: dict[str, Any],
    bounds: dict[str, dict[str, float]],
    history: list[dict[str, Any]],
    rejected: set[tuple[tuple[str, float], ...]],
    goal: str = 'balanced',
) -> dict[str, Any]:
    """Evidence-based zero-dependency PID tuning heuristic for TC264 smart cars.

    Used both in built-in offline auto-tune mode and as the conservative fallback
    when an external LLM provider times out or returns unparseable output.
    """
    acc = LOOP_PROFILES[loop_id]['acceptance']
    weights = GOAL_PRESETS.get(goal, GOAL_PRESETS['balanced'])
    max_ov = acc['max_overshoot_pct'] * weights['max_overshoot_scale']
    prop = dict(current)

    status = str(metrics.get('status', 'UNKNOWN')).upper()
    ov = float(metrics.get('overshoot_pct', 0.0))
    rise = float(metrics.get('rise_ms', 0.0))
    st_err = float(metrics.get('steady_err_pct', 0.0))
    tail_mae = float(metrics.get('tail_mae_pct', 0.0))
    crossings = int(metrics.get('crossings', 0))
    sat = float(metrics.get('pwm_sat_pct', 0.0))

    # Detect if previous step worsened score -> damp step scale
    step_scale = 0.85 if len(history) >= 2 and history[-1].get('score', 0) > history[-2].get('score', 0) else 1.0

    def nudge(key: str, frac: float) -> None:
        if key not in prop or key not in bounds:
            return
        sp = bounds[key]
        cur_v = prop[key]
        span = sp['max_step'] * step_scale
        if cur_v <= 1e-6 and frac > 0:
            delta = span * min(0.5, frac)
        else:
            delta = cur_v * frac
            if abs(delta) > span:
                delta = math.copysign(span, delta)
        prop[key] = max(sp['min'], min(sp['max'], cur_v + delta))

    if loop_id == 'yaw_2s1p':
        if status == 'OSCILLATING' or crossings >= 5:
            nudge('ykp', -0.22)
            nudge('yki', -0.30)
            summary = f'检测到角速度振荡（穿越 {crossings} 次），保守回调 ykp 并减小 yki。'
            action = 'DAMP_OSCILLATION'
        elif ov > max_ov:
            nudge('ykp', -0.15)
            nudge('yki', -0.25)
            if sat > 40.0:
                nudge('yff', -0.08)
            summary = f'角速度超调 {ov:.1f}% 偏高，适度降低 ykp/yki 抑制过冲。'
            action = 'REDUCE_OVERSHOOT'
        elif st_err > acc['max_steady_err_pct'] and rise <= acc['max_rise_ms'] * 1.15:
            # Under-tracked steady plateau -> boost feedforward first, small Ki
            nudge('yff', 0.16)
            if current.get('yki', 0.0) < 0.25:
                nudge('yki', 0.25)
            summary = f'角速度稳态静差 {st_err:.1f}%，优先补前馈 yff 并微加 yki。'
            action = 'BOOST_FF_AND_I'
        elif rise > acc['max_rise_ms'] or status == 'SLOW_RESPONSE':
            boost = 0.45 if (status == 'SLOW_RESPONSE' and ov < 1.5 and sat < 15.0 and st_err > 12.0) else 0.24
            nudge('ykp', boost)
            if st_err > 2.5:
                nudge('yff', 0.26 if st_err > 12.0 else 0.12)
            summary = f'角速度上升偏慢（{rise:.0f} ms，静差 {st_err:.1f}%），提升直出比例增益 ykp 加快响应。'
            action = 'BOOST_KP'
        elif tail_mae > acc['max_tail_mae_pct']:
            nudge('ykp', 0.10)
            nudge('yff', 0.06)
            summary = f'尾部跟踪误差 {tail_mae:.1f}% 偏大，小幅补强 ykp 与 yff。'
            action = 'FINE_TUNE'
        else:
            nudge('ykp', 0.06 if rise > acc['max_rise_ms'] * 0.7 else -0.04)
            summary = '已接近目标窗口，做小步细调以验证最优裕度。'
            action = 'FINE_TUNE'
    elif loop_id == 'speed_2s1p':
        if status == 'OSCILLATING' or crossings >= 5:
            nudge('vkp', -0.20)
            nudge('vki', -0.25)
            summary = f'均速环出现振荡（穿越 {crossings} 次），同步下调 vkp/vki。'
            action = 'DAMP_OSCILLATION'
        elif ov > max_ov:
            nudge('vkp', -0.14)
            nudge('vki', -0.22)
            summary = f'均速超调 {ov:.1f}% 偏大，适当减小 vkp 与积分 vki。'
            action = 'REDUCE_OVERSHOOT'
        elif rise > acc['max_rise_ms'] or status == 'SLOW_RESPONSE':
            boost = 0.40 if (status == 'SLOW_RESPONSE' and ov < 1.5 and sat < 15.0 and st_err > 12.0) else 0.22
            nudge('vkp', boost)
            if st_err > acc['max_steady_err_pct']:
                nudge('vki', 0.20)
            summary = f'均速上升时间 {rise:.0f} ms 偏慢，提升均速 Kp 加快跟踪。'
            action = 'BOOST_KP'
        elif st_err > acc['max_steady_err_pct'] or tail_mae > acc['max_tail_mae_pct']:
            nudge('vki', 0.24)
            summary = f'均速存在 {st_err:.1f}% 稳态偏差，适度增加均速 Ki 消除静差。'
            action = 'BOOST_KI'
        else:
            nudge('vkp', 0.06)
            summary = '均速响应平稳，微增 vkp 测试是否能进一步缩短上升时间。'
            action = 'FINE_TUNE'
    elif loop_id == 'speed_3c':
        if status == 'OSCILLATING' or crossings >= 5:
            for k in ('lkp', 'rkp'):
                nudge(k, -0.18)
            for k in ('lki', 'rki'):
                nudge(k, -0.22)
            summary = f'三串轮速振荡（穿越 {crossings} 次），保守回调左右轮 Kp/Ki。'
            action = 'DAMP_OSCILLATION'
        elif ov > max_ov:
            for k in ('lkp', 'rkp'):
                nudge(k, -0.12)
            for k in ('lki', 'rki'):
                nudge(k, -0.20)
            summary = f'轮速超调 {ov:.1f}%，减小左右轮 Kp/Ki 抑制超调。'
            action = 'REDUCE_OVERSHOOT'
        elif rise > acc['max_rise_ms'] or status == 'SLOW_RESPONSE':
            for k in ('lkp', 'rkp'):
                nudge(k, 0.22)
            if st_err > acc['max_steady_err_pct']:
                for k in ('lki', 'rki'):
                    nudge(k, 0.16)
            summary = f'轮速上升 {rise:.0f} ms 偏慢，提升左右轮 Kp。'
            action = 'BOOST_KP'
        elif st_err > acc['max_steady_err_pct']:
            for k in ('lki', 'rki'):
                nudge(k, 0.22)
            summary = f'轮速稳态静差 {st_err:.1f}%，增加左右轮 Ki。'
            action = 'BOOST_KI'
        else:
            for k in ('lkp', 'rkp'):
                nudge(k, 0.06)
            summary = '进入细调，微升轮速 Kp 检查响应裕度。'
            action = 'FINE_TUNE'
    elif loop_id == 'yaw_3c':
        if status == 'OSCILLATING' or crossings >= 5:
            nudge('ykp', -0.20)
            nudge('yki', -0.25)
            summary = f'三串角速度中环振荡（穿越 {crossings} 次），回调 ykp/yki。'
            action = 'DAMP_OSCILLATION'
        elif ov > max_ov:
            nudge('ykp', -0.14)
            nudge('yki', -0.20)
            summary = f'角速度中环超调 {ov:.1f}%，回调 ykp/yki。'
            action = 'REDUCE_OVERSHOOT'
        elif rise > acc['max_rise_ms'] or status == 'SLOW_RESPONSE':
            nudge('ykp', 0.22)
            summary = f'角速度中环偏慢（{rise:.0f} ms），提升 ykp。'
            action = 'BOOST_KP'
        elif st_err > acc['max_steady_err_pct']:
            nudge('yki', 0.22)
            summary = f'角速度中环存在 {st_err:.1f}% 稳态偏差，微增 yki。'
            action = 'BOOST_KI'
        else:
            nudge('ykp', 0.06)
            summary = '角速度中环进入细调阶段。'
            action = 'FINE_TUNE'
    else:  # outer
        if status == 'OSCILLATING' or crossings >= 8:
            nudge('dirp_cur', -0.15)
            nudge('dird_cur', 0.18)
            summary = f'视觉循迹摆动偏大（穿越 {crossings} 次），降外环 Kp 并补 Kd 阻尼。'
            action = 'DAMP_OUTER'
        elif float(metrics.get('mae', 0.0)) > 16.0:
            nudge('dirp_cur', 0.16)
            nudge('dird_cur', 0.10)
            summary = f'视觉偏差 MAE={metrics["mae"]:.1f} px 偏大，提升外环 Kp/Kd。'
            action = 'BOOST_OUTER'
        else:
            nudge('dird_cur', 0.08)
            summary = '外环跟踪平稳，微调 Kd 抑制过弯摆尾。'
            action = 'FINE_TUNE'

    safe, notes = check_and_clamp_proposal(current, prop, bounds, strict=False)
    # Avoid repeating an already-rejected parameter point
    if _param_signature(safe) in rejected or _param_signature(safe) == _param_signature(current):
        for key in bounds:
            sp = bounds[key]
            mid = round(max(sp['min'], min(sp['max'], current[key] + sp['max_step'] * 0.25)), int(sp.get('decimals', 3)))
            alt = dict(current)
            alt[key] = mid
            if _param_signature(alt) not in rejected and _param_signature(alt) != _param_signature(current):
                safe = alt
                notes.append(f'避开已拒绝参数点，对 {key} 做 1/4 步长试探')
                break

    return {
        'parameters': safe,
        'tuning_action': action,
        'status': 'TUNING',
        'analysis_summary': summary,
        'guardrail_notes': notes,
        'source': 'rule',
    }


def extract_json_object(text: str) -> dict[str, Any] | None:
    """Robustly extract the first valid JSON object from an LLM response."""
    stripped = (text or '').strip()
    if not stripped:
        return None
    candidates = [stripped]
    depth = 0
    start = 0
    quoted = False
    escaped = False
    for idx, ch in enumerate(text):
        if depth and quoted:
            if escaped:
                escaped = False
            elif ch == '\\':
                escaped = True
            elif ch == '"':
                quoted = False
        elif depth and ch == '"':
            quoted = True
        elif ch == '{':
            if depth == 0:
                start = idx
            depth += 1
        elif ch == '}' and depth:
            depth -= 1
            if depth == 0:
                cand = text[start:idx + 1]
                if cand != stripped:
                    candidates.append(cand)
    for cand in candidates:
        try:
            obj = json.loads(cand)
            if isinstance(obj, dict):
                return obj
        except (ValueError, RecursionError):
            continue
    return None


def call_llm_proposal(
    *,
    provider: str,
    base_url: str,
    model: str,
    api_key: str,
    loop_id: str,
    current: dict[str, float],
    metrics: dict[str, Any],
    bounds: dict[str, dict[str, float]],
    history: list[dict[str, Any]],
    best_record: dict[str, Any] | None,
    goal: str,
    user_note: str,
    reasoning_effort: str = 'medium',
    cli_tool: str = 'auto',
    timeout_s: float = 25.0,
) -> dict[str, Any]:
    """Request next bounded parameter proposal from OpenAI-compatible, Ollama, or Anthropic API."""
    profile = LOOP_PROFILES[loop_id]
    eff_key = normalize_effort(reasoning_effort)
    eff_cfg = REASONING_LEVELS[eff_key]
    system_prompt = (
        '你是 TC264 智能车（差速视觉车，支持两串一并 2S1P 与传统三串）控制整定工程师。'
        '根据实测阶跃指标和历史轮次，给出下一轮有界参数建议。\n'
        f'当前推理强度：{eff_cfg["label"]} ({eff_cfg["en"]}) — {eff_cfg["depth"]}\n'
        '硬性规则：\n'
        '1. 只调整当前环路声明的参数，绝对不能超出 min/max 和单轮 max_step。\n'
        '2. 优先保留已验证有效的参数，不要无依据清零积分或前馈；已经验证变差的方向不要重复试。\n'
        '3. 如果指标已经稳定达标，保持参数并把 status 设为 "DONE"。\n'
        '4. 严格输出单个合法 JSON 对象，不含 Markdown 代码块或多余文字：\n'
        '{"parameters": {<声明的参数名>: <数值>}, "tuning_action": "ADJUST_PARAMETERS"|"OBSERVE", '
        '"status": "TUNING"|"DONE", "analysis_summary": "一句话中文分析与调参依据"}'
    )
    compact_history = [
        {
            'round': h['round'],
            'action': h.get('action', ''),
            'parameters': h['parameters'],
            'metrics': {
                k: h['metrics'].get(k)
                for k in ('status', 'rise_ms', 'overshoot_pct', 'steady_err_pct', 'tail_mae_pct', 'crossings', 'pwm_sat_pct')
            },
            'score': h.get('score'),
        }
        for h in history[-6:]
    ]
    user_payload = {
        'loop': loop_id,
        'loop_label': profile['label'],
        'equations': profile['equations'],
        'goal_priority': goal,
        'user_note': (user_note or '').strip()[:300],
        'acceptance_limits': profile['acceptance'],
        'parameter_bounds': bounds,
        'current_parameters': current,
        'current_metrics': metrics,
        'best_verified': (
            {'round': best_record['round'], 'parameters': best_record['parameters'], 'score': best_record['score']}
            if best_record else None
        ),
        'recent_history': compact_history,
    }
    user_prompt = '当前实测数据如下，请输出下一轮 JSON 调参建议：\n' + json.dumps(user_payload, ensure_ascii=False)

    prov = provider.strip().lower()
    if prov == 'local_cli':
        text, _used_lbl = call_local_cli_text(
            cli_tool=cli_tool,
            model=model,
            reasoning_effort=eff_key,
            system_prompt=system_prompt,
            user_prompt=user_prompt,
            timeout_s=max(timeout_s, 75.0),
        )
    else:
        url = base_url.strip().rstrip('/')
        headers = {'Content-Type': 'application/json', 'User-Agent': 'TunePro-AI/1.4'}

        if prov == 'anthropic':
            endpoint = url if url.endswith('/messages') else f'{url or "https://api.anthropic.com/v1"}/messages'
            headers['x-api-key'] = api_key
            headers['anthropic-version'] = '2023-06-01'
            body = {
                'model': model or 'claude-3-5-sonnet-latest',
                'max_tokens': eff_cfg['max_tokens'],
                'temperature': eff_cfg['temp'],
                'system': system_prompt,
                'messages': [{'role': 'user', 'content': user_prompt}],
            }
        else:
            # OpenAI-compatible (OpenAI, DeepSeek, Qwen, OpenRouter, vLLM, Ollama /v1)
            if prov == 'ollama' and not url:
                url = 'http://127.0.0.1:11434/v1'
            if not url:
                url = 'https://api.openai.com/v1'
            endpoint = url if url.endswith('/chat/completions') else f'{url}/chat/completions'
            if api_key:
                headers['Authorization'] = f'Bearer {api_key}'
            body = {
                'model': model or ('qwen2.5:7b' if prov == 'ollama' else 'gpt-4o-mini'),
                'temperature': eff_cfg['temp'],
                'max_tokens': eff_cfg['max_tokens'],
                'messages': [
                    {'role': 'system', 'content': system_prompt},
                    {'role': 'user', 'content': user_prompt},
                ],
            }

        req = urllib.request.Request(
            endpoint,
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers=headers,
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            raw_resp = resp.read().decode('utf-8', errors='replace')
        parsed_resp = json.loads(raw_resp)

        if prov == 'anthropic':
            blocks = parsed_resp.get('content', [])
            text = '\n'.join(b.get('text', '') for b in blocks if isinstance(b, dict) and b.get('type') == 'text')
        else:
            choices = parsed_resp.get('choices') or []
            msg = choices[0].get('message', {}) if choices else {}
            text = str(msg.get('content') or '')

    obj = extract_json_object(text)
    if not obj or not isinstance(obj.get('parameters'), dict):
        raise ValueError(f'模型返回格式无法解析为参数 JSON：{text[:160]}')

    safe_params, notes = check_and_clamp_proposal(current, obj['parameters'], bounds, strict=False)
    return {
        'parameters': safe_params,
        'tuning_action': str(obj.get('tuning_action') or 'ADJUST_PARAMETERS').upper(),
        'status': 'DONE' if str(obj.get('status', '')).upper() == 'DONE' else 'TUNING',
        'analysis_summary': str(obj.get('analysis_summary') or '模型已给出下一轮参数建议。')[:280],
        'guardrail_notes': notes,
        'source': prov,
    }


def _build_offline_chat_reply(
    *,
    message: str,
    loop_id: str,
    gp1: dict[str, Any] | None,
    vbat_mv: int | None,
    live: dict[str, Any],
    history: list[dict[str, Any]],
    best: dict[str, Any] | None,
    bounds: dict[str, dict[str, float]],
    effort: str,
    dry_mode: bool,
) -> dict[str, Any]:
    """Zero-API-key local smart-car tuning advisor with live telemetry & bounded gain proposals."""
    profile = LOOP_PROFILES[loop_id]
    eff_cfg = REASONING_LEVELS[effort]
    text = (message or '').strip()
    low_txt = text.lower()
    cur = extract_params_for_loop(gp1, loop_id) if gp1 else None
    last_rec = history[-1] if history else None
    last_m = (last_rec or {}).get('metrics') or {}

    insights: list[str] = []
    suggested: dict[str, float] | None = None
    notes: list[str] = []

    if vbat_mv is not None and vbat_mv >= VBAT_USB_MV:
        v = vbat_mv / 1000.0
        if v < 11.70:
            insights.append(f'电池提示：当前 3S 电压 {v:.2f} V 偏低，开负压或大角速度阶跃可能触发限幅或掉速，建议换满电。')
        elif effort in ('high', 'max'):
            insights.append(f'供电正常：3S 电池 {v:.2f} V，动力裕度充足。')

    if cur:
        p_str = ' / '.join(f'{k}={v}' for k, v in cur.items())
        insights.append(f'当前环路「{profile["label"]}」实读参数：{p_str}（{("干地限幅" if dry_mode else "标准范围")}）。')
    else:
        insights.append('尚未同步到车端 @GP1 参数，建议先连接 MOCK0 或实车串口后再做实测整定。')

    # Live telemetry commentary
    pwm_cm = live.get('pwmCm')
    pwm_dm = live.get('pwmDm')
    gyro = live.get('gyro')
    spd = live.get('spd')
    if isinstance(pwm_cm, (int, float)) and isinstance(pwm_dm, (int, float)) and (abs(pwm_cm) > 10 or abs(pwm_dm) > 10):
        sat_peak = max(abs(pwm_cm + pwm_dm), abs(pwm_cm - pwm_dm))
        insights.append(f'实时母线分配：纵向共模 PWM≈{pwm_cm:.0f}，转向差模 PWM≈{pwm_dm:.0f}（单轮峰值≈{sat_peak:.0f}/8500）。')

    if last_m:
        insights.append(
            f'最近实测（第 {last_rec["round"]} 轮，{last_m.get("status", "—")}）：'
            f'上升 {last_m.get("rise_ms", "—")} ms · 超调 {last_m.get("overshoot_pct", "—")}% · '
            f'静差 {last_m.get("steady_err_pct", "—")}% · 饱和 {last_m.get("pwm_sat_pct", "—")}%。'
        )

    # Intent-aware engineering advice + bounded candidate proposal
    want_damp = any(w in low_txt for w in ('超调', '震荡', '振荡', '晃', '抖', '摆尾', '抽风', '稳', '柔', 'overshoot', 'oscillat'))
    want_fast = any(w in low_txt for w in ('慢', '滞后', '响应', '激进', '快', '跟不上', '拉高', '提速', 'faster', 'slow'))
    want_steady = any(w in low_txt for w in ('静差', '偏低', '差一点', '不到目标', '稳态', '前馈', 'steady'))
    want_dry = any(w in low_txt for w in ('干地', '无负压', '打滑', 'dry'))
    want_suction = any(w in low_txt for w in ('负压', '吸盘', '抓地', 'suction'))

    reply_lines: list[str] = []
    if want_dry and loop_id == 'yaw_2s1p' and cur:
        target_p = {'yff': cur['yff'], 'ykp': min(6.5, cur['ykp']), 'yki': 0.0}
        suggested, notes = check_and_clamp_proposal(cur, target_p, bounds, strict=False)
        reply_lines.append('两串一并干地（0% 负压）抓地力明显低于开负压状态，直出角速度 Kp 过高会引发原地抽振；建议先往 Kp≈6.5、Ki=0 靠拢，并保留 Kff≈1.85 维持转向起手响应。')
    elif want_suction and loop_id == 'yaw_2s1p' and cur:
        target_p = {'yff': 1.85, 'ykp': min(18.0, cur['ykp'] + bounds['ykp']['max_step']), 'yki': 0.0}
        suggested, notes = check_and_clamp_proposal(cur, target_p, bounds, strict=False)
        reply_lines.append('开负压后轮胎侧向抓地显著增强，可把直出角速度 Kp 逐步推向 15~18、Kff 维持 1.85（循迹建议 Ki=0 避免入弯积分过冲）。单轮仍受护栏步长保护，可分步逼近。')
    elif want_damp and cur:
        prop = dict(cur)
        for k in prop:
            if 'kp' in k or 'dirp' in k:
                prop[k] *= 0.84
            elif 'ki' in k:
                prop[k] *= 0.75
            elif 'dird' in k:
                prop[k] = prop[k] * 1.15 + 0.5
        suggested, notes = check_and_clamp_proposal(cur, prop, bounds, strict=False)
        reply_lines.append('抑制超调与摆动优先回调比例增益 15% 左右并压低积分累积（若为视觉外环则顺带补一点 Kd 阻尼）。下面已按当前环路安全步长生成一组保守收敛参数，你可以一键安全写入后立刻跑一趟 4 ms 阶跃复测。')
    elif want_fast and cur:
        prop = dict(cur)
        for k in prop:
            if 'kp' in k or 'dirp' in k:
                prop[k] *= 1.22
            elif k == 'yff':
                prop[k] *= 1.10
        suggested, notes = check_and_clamp_proposal(cur, prop, bounds, strict=False)
        reply_lines.append('想加快响应先在 PWM 未顶满的前提下小步提升比例增益（两串一并角速度环可同步微补前馈 Kff 缩短相位滞后）。下方候选已通过步长护栏检查，写入后重点盯「共模/差模 PWM」是否提前触顶 8500。')
    elif want_steady and cur:
        prop = dict(cur)
        if 'yff' in prop:
            prop['yff'] *= 1.14
        for k in prop:
            if 'ki' in k:
                prop[k] = prop[k] * 1.22 if prop[k] > 0.01 else bounds[k]['max_step'] * 0.4
        suggested, notes = check_and_clamp_proposal(cur, prop, bounds, strict=False)
        reply_lines.append('稳态静差优先区分原因：两串一并角速度环优先微调前馈 Kff（无相位滞后），均速/轮速环则适度增加 Ki；若 PWM 已接近 8500 饱和则加积分只会引发 windup 超调。')
    elif cur and last_m:
        rule_prop = build_rule_proposal(loop_id, cur, last_m, bounds, history, set(), 'balanced')
        suggested = rule_prop['parameters']
        notes = rule_prop.get('guardrail_notes', [])
        reply_lines.append(f'结合最近一次 4 ms 阶跃实测：{rule_prop["analysis_summary"]}')
        if effort in ('high', 'max'):
            reply_lines.append(f'（{eff_cfg["label"]}诊断模式）公式参考：{profile["equations"]}。建议改参后立刻做同目标复测，确认第 2 阶跃沿的上升时间与超调同步改善再保留。')
    else:
        reply_lines.append(f'当前处于「{profile["label"]}」· {eff_cfg["label"]}推理档。你可以直接点「单步测基线」或「开始自动调参」采集 4 ms 阶跃，也可以直接告诉我车体体感（例如："入弯超调太大"、"响应偏慢"、"切干地限幅"），我会结合实时遥测给出安全步长内的参数建议。')
        if isinstance(spd, (int, float)) and isinstance(gyro, (int, float)):
            reply_lines.append(f'当前实时读数：均速 {spd:.1f} cnt/2ms，陀螺 {gyro:.1f} °/s。')

    if best and effort in ('medium', 'high', 'max'):
        b_str = ' / '.join(f'{k}={v}' for k, v in best['parameters'].items())
        insights.append(f'历史最佳稳定点：第 {best["round"]} 轮（评分 {best["score"]} · {b_str}）。')

    return {
        'ok': True,
        'provider': 'rule',
        'model': f'内置循证诊断引擎 · {eff_cfg["label"]}',
        'effort': effort,
        'reply': '\n'.join(reply_lines),
        'insights': insights,
        'suggested_params': suggested,
        'guardrail_notes': notes,
        'loop': loop_id,
    }


def call_llm_chat(
    *,
    provider: str,
    base_url: str,
    model: str,
    api_key: str,
    reasoning_effort: str,
    message: str,
    chat_history: list[dict[str, str]],
    loop_id: str,
    gp1: dict[str, Any] | None,
    vbat_mv: int | None,
    live: dict[str, Any],
    tune_history: list[dict[str, Any]],
    best: dict[str, Any] | None,
    bounds: dict[str, dict[str, float]],
    dry_mode: bool,
    cli_tool: str = 'auto',
    timeout_s: float = 28.0,
) -> dict[str, Any]:
    """Conversational AI tuning copilot via stdlib urllib.request (OpenAI/Ollama/Anthropic)."""
    profile = LOOP_PROFILES[loop_id]
    eff_key = normalize_effort(reasoning_effort)
    eff_cfg = REASONING_LEVELS[eff_key]
    cur = extract_params_for_loop(gp1, loop_id) if gp1 else None

    system_prompt = (
        '你是嵌入在 TC264 智能车无线调参工作台（TunePro）里的资深控制工程搭档。'
        '用户正一边盯着实时波形与摄像头画面，一边和你交流调参。\n'
        f'当前推理强度：{eff_cfg["label"]} ({eff_cfg["en"]}) — {eff_cfg["depth"]}\n'
        '请基于实时遥测、当前参数和最近阶跃数据直接给出工程见解（简体中文，条理清晰、拒绝废话）。\n'
        '严格输出单个合法 JSON 对象：\n'
        '{"reply": "你的主要中文回复与工程分析", "insights": ["最多3条精炼实时诊断要点"], '
        '"suggested_params": {<可选：若建议改参，仅填当前环路声明参数名和数值，否则为 null>}}'
    )
    context_obj = {
        'loop': loop_id,
        'loop_label': profile['label'],
        'equations': profile['equations'],
        'dry_mode': dry_mode,
        'vbat_v': round(vbat_mv / 1000.0, 2) if (vbat_mv and vbat_mv >= VBAT_USB_MV) else None,
        'current_parameters': cur,
        'bounds': bounds,
        'live_telemetry': live,
        'recent_rounds': tune_history[-4:],
        'best_verified': ({'round': best['round'], 'parameters': best['parameters'], 'score': best['score']} if best else None),
    }

    msgs: list[dict[str, str]] = []
    for item in (chat_history or [])[-6:]:
        role = 'assistant' if item.get('role') == 'assistant' else 'user'
        txt = str(item.get('text') or '').strip()[:600]
        if txt:
            msgs.append({'role': role, 'content': txt})
    msgs.append({
        'role': 'user',
        'content': f'【实时车端上下文】{json.dumps(context_obj, ensure_ascii=False)}\n\n【用户消息】{message}',
    })

    prov = provider.strip().lower()
    if prov == 'local_cli':
        hist_lines = []
        for item in msgs[:-1]:
            tag = '助手' if item['role'] == 'assistant' else '用户'
            hist_lines.append(f'{tag}: {item["content"]}')
        convo_prefix = ('【历史对话】\n' + '\n'.join(hist_lines) + '\n\n') if hist_lines else ''
        text, used_model = call_local_cli_text(
            cli_tool=cli_tool,
            model=model,
            reasoning_effort=eff_key,
            system_prompt=system_prompt,
            user_prompt=convo_prefix + msgs[-1]['content'],
            timeout_s=max(timeout_s, 90.0),
        )
    else:
        url = base_url.strip().rstrip('/')
        headers = {'Content-Type': 'application/json', 'User-Agent': 'TunePro-AI/1.4'}

        if prov == 'anthropic':
            endpoint = url if url.endswith('/messages') else f'{url or "https://api.anthropic.com/v1"}/messages'
            headers['x-api-key'] = api_key
            headers['anthropic-version'] = '2023-06-01'
            used_model = model or 'claude-3-5-sonnet-latest'
            body: dict[str, Any] = {
                'model': used_model,
                'max_tokens': eff_cfg['max_tokens'],
                'temperature': eff_cfg['temp'],
                'system': system_prompt,
                'messages': msgs,
            }
        else:
            if prov == 'ollama' and not url:
                url = 'http://127.0.0.1:11434/v1'
            if not url:
                url = 'https://api.openai.com/v1'
            endpoint = url if url.endswith('/chat/completions') else f'{url}/chat/completions'
            if api_key:
                headers['Authorization'] = f'Bearer {api_key}'
            used_model = model or ('qwen2.5:7b' if prov == 'ollama' else 'gpt-4o-mini')
            body = {
                'model': used_model,
                'temperature': eff_cfg['temp'],
                'max_tokens': eff_cfg['max_tokens'],
                'messages': [{'role': 'system', 'content': system_prompt}, *msgs],
            }
            low_m = used_model.lower()
            if prov == 'openai' and any(tok in low_m for tok in ('o1', 'o3', 'o4', 'gpt-5', 'gpt-6', 'reasoner')):
                body['reasoning_effort'] = 'high' if eff_key == 'max' else ('low' if eff_key == 'fast' else eff_key)

        req = urllib.request.Request(
            endpoint,
            data=json.dumps(body, ensure_ascii=False).encode('utf-8'),
            headers=headers,
            method='POST',
        )
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            raw = resp.read().decode('utf-8', errors='replace')
        parsed = json.loads(raw)

        if prov == 'anthropic':
            blocks = parsed.get('content', [])
            text = '\n'.join(b.get('text', '') for b in blocks if isinstance(b, dict) and b.get('type') == 'text')
        else:
            choices = parsed.get('choices') or []
            msg_obj = choices[0].get('message', {}) if choices else {}
            text = str(msg_obj.get('content') or '')

    obj = extract_json_object(text)
    if obj and isinstance(obj.get('reply'), str):
        reply_txt = obj['reply'].strip()
        raw_ins = obj.get('insights') if isinstance(obj.get('insights'), list) else []
        insights = [str(x).strip()[:200] for x in raw_ins if str(x).strip()][:4]
        raw_prop = obj.get('suggested_params') if isinstance(obj.get('suggested_params'), dict) else None
    else:
        reply_txt = text.strip() or '模型未返回有效文本。'
        insights = []
        raw_prop = None

    suggested = None
    notes: list[str] = []
    if raw_prop and cur:
        try:
            suggested, notes = check_and_clamp_proposal(cur, raw_prop, bounds, strict=False)
        except Exception:
            suggested = None

    return {
        'ok': True,
        'provider': prov,
        'model': used_model,
        'effort': eff_key,
        'reply': reply_txt,
        'insights': insights,
        'suggested_params': suggested,
        'guardrail_notes': notes,
        'loop': loop_id,
    }


_PRO_REPLY_RE = re.compile(rb'@(?:GP1|WA1|QX1|RA1),[^\r\n@*]+\*[0-9A-Fa-f]{4}')


class BridgeSession:
    """Protocol-level client attached to serial_bridge.SerialBridge via pub/sub.

    Reads @GP1, @WA1, @QX1, @RA1, @M2, status lines, and [tune2] captures without
    interfering with browser SSE subscribers.
    """

    def __init__(self, bridge: serial_bridge.SerialBridge):
        self.bridge = bridge
        self._lock = threading.RLock()
        self._sub_q: queue.Queue | None = None
        self._stop_ev = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._line_buf = bytearray()

        self._rid = 100
        self.gp1: dict[str, Any] | None = None
        self.vbat_mv: int | None = None
        self.running: bool = False
        self.stop_reason: int = 0
        self.last_error: str = ''

        self._gp1_ev = threading.Event()
        self._ack_ev = threading.Event()
        self._last_ack: tuple[str, list[str]] | None = None
        self._stop_seen_ev = threading.Event()

        self._cap_active = False
        self._cap_header: dict[str, Any] = {}
        self._cap_rows: list[dict[str, float]] = []
        self._cap_ev = threading.Event()
        self._last_capture: dict[str, Any] | None = None

        self._m2_recording = False
        self._m2_rows: list[dict[str, Any]] = []

    def start(self) -> None:
        with self._lock:
            if self._sub_q is not None:
                return
            self._sub_q = self.bridge.subscribe_raw()
            self._ctl_q = self.bridge.subscribe(include_data=False)
            self._stop_ev.clear()
            self._reader_thread = threading.Thread(target=self._drain_loop, name='ai-bridge-tap', daemon=True)
            self._reader_thread.start()

    def stop(self) -> None:
        self._stop_ev.set()
        with self._lock:
            sub = self._sub_q
            ctl = getattr(self, '_ctl_q', None)
            self._sub_q = None
            self._ctl_q = None
            thr = self._reader_thread
            self._reader_thread = None
        if sub is not None:
            self.bridge.unsubscribe_raw(sub)
        if ctl is not None:
            self.bridge.unsubscribe(ctl)
        if thr is not None and thr.is_alive() and thr is not threading.current_thread():
            thr.join(timeout=1.0)

    def _next_rid(self) -> int:
        with self._lock:
            self._rid = (self._rid % 9000) + 100
            return self._rid

    def _drain_loop(self) -> None:
        while not self._stop_ev.is_set():
            sub = self._sub_q
            ctl = getattr(self, '_ctl_q', None)
            if sub is None:
                break
            if ctl is not None:
                while True:
                    try:
                        ev, payload = ctl.get_nowait()
                        if ev == 'status' and not payload.get('connected'):
                            with self._lock:
                                self.running = False
                                self._stop_seen_ev.set()
                    except queue.Empty:
                        break
            try:
                chunk = sub.get(timeout=0.05)
            except queue.Empty:
                continue
            if chunk:
                self._feed_bytes(chunk)

    def _feed_bytes(self, data: bytes) -> None:
        # Fast-path C-level split by newline; preserves trailing partial ASCII replies even under 100+ FPS binary camera streams
        if b'\n' not in data and b'\r' not in data:
            self._line_buf.extend(data)
            if len(self._line_buf) > 4096:
                del self._line_buf[:-512]
            return
        buf = bytes(self._line_buf) + data
        self._line_buf.clear()
        parts = buf.replace(b'\r', b'\n').split(b'\n')
        tail = parts.pop()
        if tail:
            self._line_buf.extend(tail[-512:] if len(tail) > 4096 else tail)
        for raw_line in parts:
            if raw_line:
                self._handle_line(raw_line)

    def _handle_line(self, raw_line: bytes) -> None:
        # Fast-path @M2 binary-adjacent ASCII frame
        if b'@M2,' in raw_line:
            m = serial_bridge.M2_RE.search(raw_line)
            if m:
                row = serial_bridge.decode_m2_match(m)
                if row is not None:
                    with self._lock:
                        self.running = bool(row['launch'] == 1)
                        if self._m2_recording:
                            self._m2_rows.append(row)

        # Signed protocol replies (@GP1, @WA1, @QX1, @RA1) — extract cleanly even if preceded by binary A5 5B camera slices
        if b'@' in raw_line and b'*' in raw_line:
            matched_pro = False
            for pm in _PRO_REPLY_RE.finditer(raw_line):
                matched_pro = True
                body = demo_sim.verify_signed(pm.group(0).decode('ascii', errors='ignore'))
                if body is None:
                    continue
                if body.startswith('@GP1,'):
                    gp1 = parse_gp1_line(body)
                    if gp1 is not None:
                        with self._lock:
                            self.gp1 = gp1
                            self._gp1_ev.set()
                elif body.startswith('@WA1,'):
                    parts = body.split(',')
                    with self._lock:
                        self._last_ack = ('WA1', parts[1:])
                        self._ack_ev.set()
                elif body.startswith('@QX1,'):
                    parts = body.split(',')
                    with self._lock:
                        self._last_ack = ('QX1', parts[1:])
                        self.last_error = parts[3] if len(parts) > 3 else 'ERR'
                        self._ack_ev.set()
                elif body.startswith('@RA1,'):
                    parts = body.split(',')
                    sub = parts[1] if len(parts) > 1 else ''
                    with self._lock:
                        if sub == 'STOP':
                            self.running = False
                            self.stop_reason = int(parts[3]) if len(parts) > 3 and parts[3].isdigit() else 0
                            self._stop_seen_ev.set()
                        elif sub in ('GO', 'RUN'):
                            self.running = True
                            self._last_ack = ('RA1', parts[1:])
                            self._ack_ev.set()
            if matched_pro and b'[tune2]' not in raw_line and b'vbat_mv=' not in raw_line:
                return

        text = raw_line.decode('latin1', errors='ignore').strip()
        if not text:
            return

        # [tune2] onboard 4 ms capture parser (tolerant of leading binary bytes)
        cap_b = text.find('[tune2] cap begin')
        if cap_b >= 0:
            hdr: dict[str, Any] = {}
            for tok in text[cap_b:].split()[3:]:
                if '=' in tok:
                    k, v = tok.split('=', 1)
                    try:
                        hdr[k] = int(v)
                    except ValueError:
                        hdr[k] = v
            with self._lock:
                self._cap_active = True
                self._cap_header = hdr
                self._cap_rows = []
            return
        if self._cap_active:
            u2_i = text.find('u2,')
            if u2_i >= 0:
                parts = text[u2_i:].split(',')
                if len(parts) >= 11:
                    try:
                        row = {
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
                        }
                        with self._lock:
                            self._cap_rows.append(row)
                    except ValueError:
                        pass
                return
            if '[tune2] cap end' in text:
                with self._lock:
                    self._cap_active = False
                    self._last_capture = {
                        'header': dict(self._cap_header),
                        'rows': list(self._cap_rows),
                    }
                    self._cap_ev.set()
                return

        # Status line (4 Hz): t=... vbat_mv=... running=... stop=...
        if 'vbat_mv=' in text:
            m_vb = re.search(r'vbat_mv=(\d+)', text)
            m_rn = re.search(r'running=(\d)', text)
            with self._lock:
                if m_vb:
                    self.vbat_mv = int(m_vb.group(1))
                if m_rn:
                    is_run = m_rn.group(1) == '1'
                    if self.running and not is_run:
                        self._stop_seen_ev.set()
                    self.running = is_run

    def send_signed(self, body: str, *, urgent: bool = False) -> None:
        raw = demo_sim.sign_line(body)
        self.bridge.send(raw, paced=False, urgent=urgent)

    def send_estop(self) -> None:
        try:
            self.bridge.send(b'#!', paced=False, urgent=True)
            self.send_signed('#X STOP', urgent=True)
        except Exception:
            pass

    def sync_gp1(self, timeout_s: float = 2.0) -> dict[str, Any]:
        self.start()
        rid = self._next_rid()
        self._gp1_ev.clear()
        self.send_signed(f'#M {rid}')
        if not self._gp1_ev.wait(timeout_s):
            raise TimeoutError('等待车端 @GP1 参数快照超时，请确认串口已连接并支持 TunePro 协议。')
        with self._lock:
            assert self.gp1 is not None
            return dict(self.gp1)

    def write_param(self, key: str, value: float, *, timeout_s: float = 1.5, retries: int = 2) -> dict[str, Any]:
        if key not in PARAM_SCALES:
            raise ValueError(f'未知协议参数键：{key}')
        ival = int(round(float(value) * PARAM_SCALES[key]))
        last_err = ''
        for _ in range(retries + 1):
            gp1 = self.gp1 or self.sync_gp1()
            rev = int(gp1['rev'])
            rid = self._next_rid()
            self._ack_ev.clear()
            self._gp1_ev.clear()
            self.send_signed(f'#W {rid} {rev} {key} {ival}')
            if not self._ack_ev.wait(timeout_s):
                last_err = f'写入 {key}={value} 超时'
                self.sync_gp1()
                continue
            with self._lock:
                ack = self._last_ack
            if ack and ack[0] == 'WA1':
                self._gp1_ev.wait(0.5)
                with self._lock:
                    if self.gp1 is not None:
                        return dict(self.gp1)
                return self.sync_gp1()
            if ack and ack[0] == 'QX1':
                code = ack[1][2] if len(ack[1]) > 2 else 'ERR'
                last_err = f'车端拒绝写入 {key}：{code}'
                if code == 'REVISION':
                    self._gp1_ev.wait(0.5)
                    self.sync_gp1()
                    continue
                raise RuntimeError(last_err)
        raise RuntimeError(last_err or f'写入 {key} 失败')

    def run_step_experiment(
        self,
        *,
        loop_id: str,
        target: int,
        low: int = 90,
        abort_ev: threading.Event | None = None,
        duration_outer_s: float = 2.8,
    ) -> dict[str, Any]:
        """Run one safe, watchdog-protected step experiment and collect 4 ms capture metrics."""
        profile = LOOP_PROFILES[loop_id]
        gp1 = self.sync_gp1()

        # Ensure matching topology if required
        want_topo = profile['topo_val']
        if want_topo is not None and int(gp1['topo']) != int(want_topo):
            gp1 = self.write_param('topo', want_topo)

        # Configure test mode & target before GO
        if profile['kind'] == 'speed':
            gp1 = self.write_param('wheel', profile['wheel'])
            gp1 = self.write_param('mode', profile['mode'])
            gp1 = self.write_param('test_low', low)
            gp1 = self.write_param('test_speed', target)
        elif profile['kind'] == 'yaw':
            gp1 = self.write_param('mode', profile['mode'])
            gp1 = self.write_param('test_yaw', abs(target))
        else:  # outer
            gp1 = self.write_param('mode', 0)
            gp1 = self.write_param('race_speed', target)
            gp1 = self.write_param('turn_speed', low)

        # Clear state & trigger #X GO
        self._stop_seen_ev.clear()
        self._ack_ev.clear()
        with self._lock:
            self._m2_rows = []
            self._m2_recording = (profile['kind'] == 'outer')

        rev = int(gp1['rev'])
        rid = self._next_rid()
        self.send_signed(f'#X GO {rid} {rev} 0')
        if not self._ack_ev.wait(1.5):
            self.send_estop()
            raise TimeoutError('等待车端发车确认 (@RA1,GO) 超时')
        with self._lock:
            ack = self._last_ack
        if ack and ack[0] == 'QX1':
            code = ack[1][2] if len(ack[1]) > 2 else 'REFUSED'
            raise RuntimeError(f'车端拒绝发车：{code}')

        # Keep 350 ms watchdog alive until the timed step completes or abort fires
        start_s = time.monotonic()
        max_wait = duration_outer_s if profile['kind'] == 'outer' else 5.0
        next_ping = start_s + 0.30
        try:
            while True:
                now = time.monotonic()
                if abort_ev is not None and abort_ev.is_set():
                    self.send_estop()
                    raise InterruptedError('已手动中止自动调参并触发急停')
                # Continuous 3S battery floor check while running
                vb = self.vbat_mv
                if vb is not None and vb >= VBAT_USB_MV:
                    floor = VBAT_FLOOR_FAN_MV if int(gp1.get('tune_fan', 0)) else VBAT_FLOOR_DRY_MV
                    if vb < floor:
                        self.send_estop()
                        raise RuntimeError(f'3S 电池压降触发安全红线 ({vb / 1000:.2f} V < {floor / 1000:.2f} V)，已紧急停车')
                if profile['kind'] == 'outer':
                    if now - start_s >= duration_outer_s:
                        self.send_estop()
                        break
                else:
                    if self._stop_seen_ev.is_set():
                        break
                    if now - start_s >= max_wait:
                        self.send_estop()
                        break
                if now >= next_ping:
                    self.send_signed('#X PING')
                    next_ping = now + 0.32
                time.sleep(0.03)
        finally:
            with self._lock:
                self._m2_recording = False

        # Give car 120 ms to settle after stop
        time.sleep(0.12)

        if profile['kind'] == 'outer':
            with self._lock:
                m2_copy = list(self._m2_rows)
            metrics = analyze_outer_m2(m2_copy)
            return {
                'gp1': self.sync_gp1(),
                'metrics': metrics,
                'capture_rows': len(m2_copy),
            }

        # Pull onboard 4 ms capture (#X CAP)
        self._cap_ev.clear()
        with self._lock:
            self._last_capture = None
        self.send_signed('#X CAP')
        if not self._cap_ev.wait(3.0):
            raise TimeoutError('等待车端 4 ms 板载阶跃波形 (#X CAP) 超时')
        with self._lock:
            cap = self._last_capture
        if not cap or not cap['rows']:
            raise RuntimeError('车端未返回有效阶跃抓拍数据')

        metrics = analyze_capture_rows(cap['header'], cap['rows'], loop_id)
        return {
            'gp1': self.sync_gp1(),
            'metrics': metrics,
            'capture_header': cap['header'],
            'capture_rows': len(cap['rows']),
        }


class AITunerManager:
    """Thread-safe manager for both one-shot agent API calls and background auto-tune sessions."""

    def __init__(self, bridge: serial_bridge.SerialBridge):
        self.bridge = bridge
        self.session = BridgeSession(bridge)
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._abort_ev = threading.Event()
        self._state: dict[str, Any] = {
            'active': False,
            'phase': 'idle',
            'loop': 'yaw_2s1p',
            'provider': 'rule',
            'round': 0,
            'max_rounds': 6,
            'goal': 'balanced',
            'dry_mode': False,
            'message': '就绪：连接 MOCK0 或实车后即可开始 AI 自动调参。',
            'best': None,
            'history': [],
            'rejected_count': 0,
            'completed_reason': '',
            'started_at': None,
            'finished_at': None,
        }

    def _is_mock_port(self) -> bool:
        st = self.bridge.status()
        return bool(st.get('connected')) and str(st.get('port', '')).upper().startswith('MOCK')

    def _check_safety_gate(self, *, allow_real_car: bool, confirm_wheels_off_ground: bool, dry_mode: bool = False) -> dict[str, Any]:
        st = self.bridge.status()
        if not st.get('connected'):
            raise ConnectionError('请先在顶栏连接串口（或选择 MOCK0 离线虚拟车）。')
        is_mock = str(st.get('port', '')).upper().startswith('MOCK')
        if not is_mock:
            if not (allow_real_car and confirm_wheels_off_ground):
                raise PermissionError(
                    '当前连接的是真实物理串口，安全联锁已拦截：请先架空车轮并明确勾选「实车安全确认」后再允许 AI 动轮或写参。'
                )
        gp1 = self.session.sync_gp1()
        vb = self.session.vbat_mv
        if vb is not None and vb >= VBAT_USB_MV:
            floor = VBAT_FLOOR_FAN_MV if (int(gp1.get('tune_fan', 0)) and not dry_mode) else VBAT_FLOOR_DRY_MV
            if vb < floor:
                raise RuntimeError(f'3S 电池电压偏低 ({vb / 1000:.2f} V < {floor / 1000:.2f} V)，拒绝启动自动调参。')
        return gp1

    def snapshot(self, loop_id: str | None = None, dry_mode: bool = False) -> dict[str, Any]:
        st = self.bridge.status()
        gp1 = None
        if st.get('connected'):
            try:
                gp1 = self.session.sync_gp1(timeout_s=1.2)
            except Exception:
                gp1 = self.session.gp1
        topo = (gp1 or {}).get('topology', '2s1p')
        chosen = loop_id or ('yaw_2s1p' if topo == '2s1p' else 'yaw_3c')
        if chosen not in LOOP_PROFILES:
            chosen = 'yaw_2s1p'
        bounds = get_loop_bounds(chosen, dry_mode=dry_mode)
        cur_params = extract_params_for_loop(gp1, chosen) if gp1 else None
        with self._lock:
            state_copy = deepcopy(self._state)
        return {
            'connected': bool(st.get('connected')),
            'port': st.get('port', ''),
            'is_mock': self._is_mock_port(),
            'vbat_mv': self.session.vbat_mv,
            'running': self.session.running,
            'gp1': gp1,
            'loop': chosen,
            'loop_profile': {
                'id': chosen,
                'label': LOOP_PROFILES[chosen]['label'],
                'topology': LOOP_PROFILES[chosen]['topology'],
                'equations': LOOP_PROFILES[chosen]['equations'],
                'default_target': LOOP_PROFILES[chosen]['default_target'],
                'default_low': LOOP_PROFILES[chosen]['default_low'],
                'unit': LOOP_PROFILES[chosen]['unit'],
                'acceptance': LOOP_PROFILES[chosen]['acceptance'],
            },
            'available_loops': [
                {'id': k, 'label': v['label'], 'topology': v['topology'], 'default_target': v['default_target'], 'default_low': v['default_low'], 'unit': v['unit']}
                for k, v in LOOP_PROFILES.items()
            ],
            'current_parameters': cur_params,
            'bounds': bounds,
            'local_clis': detect_local_clis(),
            'tuner': state_copy,
        }

    def status(self) -> dict[str, Any]:
        with self._lock:
            out = deepcopy(self._state)
        out['connected'] = bool(self.bridge.status().get('connected'))
        out['port'] = self.bridge.status().get('port', '')
        out['is_mock'] = self._is_mock_port()
        out['vbat_mv'] = self.session.vbat_mv
        return out

    def apply_bounded(
        self,
        *,
        loop_id: str,
        proposed: dict[str, Any],
        dry_mode: bool = False,
        allow_real_car: bool = False,
        confirm_wheels_off_ground: bool = False,
        strict: bool = True,
    ) -> dict[str, Any]:
        gp1 = self._check_safety_gate(
            allow_real_car=allow_real_car,
            confirm_wheels_off_ground=confirm_wheels_off_ground,
            dry_mode=dry_mode,
        )
        bounds = get_loop_bounds(loop_id, dry_mode=dry_mode)
        current = extract_params_for_loop(gp1, loop_id)
        safe, notes = check_and_clamp_proposal(current, proposed, bounds, strict=strict)
        for key, val in expand_loop_writes(loop_id, safe):
            gp1 = self.session.write_param(key, val)
        verified = extract_params_for_loop(gp1, loop_id)
        return {
            'ok': True,
            'loop': loop_id,
            'previous': current,
            'applied': verified,
            'readback_confirmed': True,
            'guardrail_notes': notes,
            'rev': gp1['rev'],
        }

    def run_one_step(
        self,
        *,
        loop_id: str,
        target: int | None = None,
        low: int | None = None,
        dry_mode: bool = False,
        allow_real_car: bool = False,
        confirm_wheels_off_ground: bool = False,
    ) -> dict[str, Any]:
        if loop_id not in LOOP_PROFILES:
            raise ValueError(f'未知控制环路：{loop_id}')
        self._check_safety_gate(
            allow_real_car=allow_real_car,
            confirm_wheels_off_ground=confirm_wheels_off_ground,
            dry_mode=dry_mode,
        )
        profile = LOOP_PROFILES[loop_id]
        t_lo, t_hi = profile['target_bounds']
        tgt = int(max(t_lo, min(t_hi, target if target is not None else profile['default_target'])))
        low_v = int(max(0, min(tgt - 10, low if low is not None else profile['default_low']))) if profile['kind'] == 'speed' else int(low or profile['default_low'])

        res = self.session.run_step_experiment(loop_id=loop_id, target=tgt, low=low_v)
        gp1 = res['gp1']
        params = extract_params_for_loop(gp1, loop_id)
        score = score_metrics(res['metrics'], loop_id)
        return {
            'ok': True,
            'loop': loop_id,
            'target': tgt,
            'parameters': params,
            'metrics': res['metrics'],
            'score': score,
            'acceptable': is_acceptable(res['metrics'], loop_id),
            'capture_rows': res['capture_rows'],
            'source': 'mock_simulation' if self._is_mock_port() else 'measured_hardware',
        }

    def rollback_to_best(self, *, allow_real_car: bool = False, confirm_wheels_off_ground: bool = False) -> dict[str, Any]:
        with self._lock:
            best = deepcopy(self._state.get('best'))
            loop_id = str(self._state.get('loop', 'yaw_2s1p'))
        if not best:
            raise ValueError('当前会话尚无已验证的历史最佳参数可回滚。')
        gp1 = self._check_safety_gate(
            allow_real_car=allow_real_car,
            confirm_wheels_off_ground=confirm_wheels_off_ground,
        )
        for key, val in expand_loop_writes(loop_id, best['parameters']):
            gp1 = self.session.write_param(key, val)
        verified = extract_params_for_loop(gp1, loop_id)
        with self._lock:
            self._state['message'] = f'已回滚并回读确认第 {best["round"]} 轮最佳稳定参数。'
        return {
            'ok': True,
            'loop': loop_id,
            'restored_round': best['round'],
            'parameters': verified,
            'readback_confirmed': True,
        }

    def stop(self, *, estop: bool = True) -> dict[str, Any]:
        self._abort_ev.set()
        if estop:
            self.session.send_estop()
        with self._lock:
            if self._state['active']:
                self._state['phase'] = 'stopping'
                self._state['message'] = '正在中止自动调参并执行急停…'
        thr = self._thread
        if thr is not None and thr.is_alive() and thr is not threading.current_thread():
            thr.join(timeout=2.0)
        return self.status()

    def start_auto_tune(self, cfg: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            if self._state['active']:
                raise RuntimeError('AI 自动调参正在运行中，请先停止当前任务。')

        loop_id = str(cfg.get('loop') or 'yaw_2s1p')
        if loop_id not in LOOP_PROFILES:
            raise ValueError(f'不支持的整定环路：{loop_id}')
        provider = str(cfg.get('provider') or 'rule').strip().lower()
        if provider not in ('rule', 'local_cli', 'openai', 'ollama', 'anthropic'):
            raise ValueError(f'不支持的模型通道：{provider}')

        dry_mode = bool(cfg.get('dry_mode', False))
        allow_real = bool(cfg.get('allow_real_car', False))
        wheels_ok = bool(cfg.get('confirm_wheels_off_ground', False))
        self._check_safety_gate(
            allow_real_car=allow_real,
            confirm_wheels_off_ground=wheels_ok,
            dry_mode=dry_mode,
        )

        profile = LOOP_PROFILES[loop_id]
        t_lo, t_hi = profile['target_bounds']
        target = int(max(t_lo, min(t_hi, int(cfg.get('target') or profile['default_target']))))
        low = int(cfg.get('low') if cfg.get('low') is not None else profile['default_low'])
        effort = normalize_effort(cfg.get('reasoning_effort'))
        eff_cfg = REASONING_LEVELS[effort]
        max_rounds = int(max(2, min(12, int(cfg.get('max_rounds') or eff_cfg['default_rounds']))))
        goal = str(cfg.get('goal') or 'balanced')
        if goal not in GOAL_PRESETS:
            goal = 'balanced'

        api_key = str(cfg.get('api_key') or '').strip()
        if not api_key:
            if provider == 'anthropic':
                api_key = os.environ.get('TUNEPRO_AI_API_KEY') or os.environ.get('ANTHROPIC_API_KEY') or ''
            elif provider == 'openai':
                api_key = os.environ.get('TUNEPRO_AI_API_KEY') or os.environ.get('OPENAI_API_KEY') or ''

        run_cfg = {
            'loop': loop_id,
            'provider': provider,
            'cli_tool': str(cfg.get('cli_tool') or 'auto').strip().lower(),
            'base_url': str(cfg.get('base_url') or '').strip(),
            'model': str(cfg.get('model') or '').strip(),
            'api_key': api_key.strip(),
            'reasoning_effort': effort,
            'target': target,
            'low': low,
            'max_rounds': max_rounds,
            'goal': goal,
            'dry_mode': dry_mode,
            'user_note': str(cfg.get('user_note') or '').strip()[:300],
        }

        self._abort_ev.clear()
        with self._lock:
            self._state = {
                'active': True,
                'phase': 'starting',
                'loop': loop_id,
                'provider': provider,
                'round': 0,
                'max_rounds': max_rounds,
                'goal': goal,
                'dry_mode': dry_mode,
                'target': target,
                'message': f'启动 AI 自动调参：{profile["label"]}（最多 {max_rounds} 轮）…',
                'best': None,
                'history': [],
                'rejected_count': 0,
                'completed_reason': '',
                'started_at': time.strftime('%H:%M:%S'),
                'finished_at': None,
            }
            self._thread = threading.Thread(
                target=self._run_loop,
                args=(run_cfg,),
                name='tunepro-ai-worker',
                daemon=True,
            )
            self._thread.start()
        return self.status()

    def _update_state(self, **kwargs: Any) -> None:
        with self._lock:
            self._state.update(kwargs)

    def _run_loop(self, cfg: dict[str, Any]) -> None:
        loop_id = cfg['loop']
        bounds = get_loop_bounds(loop_id, dry_mode=cfg['dry_mode'])
        rejected: set[tuple[tuple[str, float], ...]] = set()
        history: list[dict[str, Any]] = []
        best: dict[str, Any] | None = None
        stable_streak = 0
        reason = 'max_rounds_reached'
        fallback_note = ''

        try:
            for round_idx in range(1, cfg['max_rounds'] + 1):
                if self._abort_ev.is_set():
                    reason = 'stopped_by_user'
                    break

                self._update_state(
                    round=round_idx,
                    phase='measuring',
                    message=f'第 {round_idx}/{cfg["max_rounds"]} 轮：执行受控阶跃并采集 4 ms 板载波形…',
                )
                exp = self.session.run_step_experiment(
                    loop_id=loop_id,
                    target=cfg['target'],
                    low=cfg['low'],
                    abort_ev=self._abort_ev,
                )
                gp1 = exp['gp1']
                params = extract_params_for_loop(gp1, loop_id)
                metrics = exp['metrics']
                score = score_metrics(metrics, loop_id, cfg['goal'])
                ok = is_acceptable(metrics, loop_id, cfg['goal'])

                record: dict[str, Any] = {
                    'round': round_idx,
                    'source': 'mock_simulation' if self._is_mock_port() else 'measured_hardware',
                    'parameters': params,
                    'metrics': metrics,
                    'score': score,
                    'acceptable': ok,
                    'action': 'BASELINE' if round_idx == 1 else 'EVALUATED',
                    'summary': '',
                    'guardrail_notes': [],
                }

                # Update best verified result (STABLE always beats non-STABLE; never promote OSCILLATING/SATURATED)
                cur_st = metrics['status']
                best_st = best['metrics']['status'] if best is not None else ''
                promote = False
                if cur_st == 'STABLE':
                    promote = (best is None) or (best_st != 'STABLE') or (score < best['score'])
                elif cur_st == 'SLOW_RESPONSE' and (best is None or (best_st != 'STABLE' and score < best['score'])):
                    promote = True
                elif best is None and round_idx == 1:
                    promote = True
                if promote:
                    for prev_r in history:
                        prev_r.pop('is_best', None)
                    record['is_best'] = True
                    best = deepcopy(record)

                if ok:
                    stable_streak += 1
                else:
                    stable_streak = 0

                # Check severe regression -> auto-rollback to best verified baseline
                rolled_back = False
                if (
                    round_idx > 1
                    and best is not None
                    and _param_signature(params) != _param_signature(best['parameters'])
                    and should_rollback(metrics, best['metrics'], loop_id, cfg['goal'])
                ):
                    rejected.add(_param_signature(params))
                    self._update_state(
                        phase='rollback',
                        message=f'第 {round_idx} 轮指标明显劣化（{metrics["status"]}），自动回滚到第 {best["round"]} 轮最佳参数…',
                    )
                    for key, val in expand_loop_writes(loop_id, best['parameters']):
                        gp1 = self.session.write_param(key, val)
                    verified_back = extract_params_for_loop(gp1, loop_id)
                    record['action'] = 'ROLLBACK'
                    record['summary'] = f'响应劣化（状态 {metrics["status"]}，超调 {metrics["overshoot_pct"]}%），已自动回滚并核验第 {best["round"]} 轮参数。'
                    rolled_back = True
                    params = verified_back

                history.append(record)
                self._update_state(
                    best=deepcopy(best),
                    history=deepcopy(history),
                    rejected_count=len(rejected),
                )

                if stable_streak >= 2:
                    record['action'] = 'DONE'
                    record['summary'] = record['summary'] or f'连续 2 轮稳定达标（上升 {metrics["rise_ms"]} ms，超调 {metrics["overshoot_pct"]}%），自动收尾。'
                    reason = 'converged'
                    break

                if round_idx == cfg['max_rounds']:
                    # Restore best verified params before finishing if final round wasn't best
                    if best is not None and _param_signature(params) != _param_signature(best['parameters']):
                        for key, val in expand_loop_writes(loop_id, best['parameters']):
                            self.session.write_param(key, val)
                        record['summary'] = (record['summary'] + ' ' if record['summary'] else '') + f'已达轮次上限，最终恢复第 {best["round"]} 轮最佳参数。'
                    reason = 'max_rounds_reached'
                    break

                # Propose next bounded update
                self._update_state(
                    phase='analyzing',
                    message=f'第 {round_idx}/{cfg["max_rounds"]} 轮：分析响应指标并生成下一轮有界参数…',
                )
                ref_params = best['parameters'] if rolled_back and best is not None else params
                ref_metrics = best['metrics'] if rolled_back and best is not None else metrics

                if ok:
                    # Hold verified acceptable gains for confirmation window (from llm-pid-tuner skill!)
                    record['action'] = 'HOLD_VERIFY'
                    record['summary'] = f'本轮已达标（上升 {metrics["rise_ms"]} ms，超调 {metrics["overshoot_pct"]}%），保持参数复测一轮确认稳定性。'
                    self._update_state(history=deepcopy(history))
                    continue

                suggestion: dict[str, Any]
                if cfg['provider'] == 'rule':
                    suggestion = build_rule_proposal(loop_id, ref_params, ref_metrics, bounds, history, rejected, cfg['goal'])
                else:
                    try:
                        suggestion = call_llm_proposal(
                            provider=cfg['provider'],
                            base_url=cfg['base_url'],
                            model=cfg['model'],
                            api_key=cfg['api_key'],
                            loop_id=loop_id,
                            current=ref_params,
                            metrics=ref_metrics,
                            bounds=bounds,
                            history=history,
                            best_record=best,
                            goal=cfg['goal'],
                            user_note=cfg['user_note'],
                            reasoning_effort=cfg.get('reasoning_effort', 'medium'),
                            cli_tool=cfg.get('cli_tool', 'auto'),
                        )
                    except Exception as exc:
                        fallback_note = f'LLM 调用失败（{str(exc)[:80]}），已切保守规则兜底。'
                        suggestion = build_rule_proposal(loop_id, ref_params, ref_metrics, bounds, history, rejected, cfg['goal'])
                        suggestion['guardrail_notes'] = [fallback_note, *suggestion.get('guardrail_notes', [])]

                record['summary'] = (record['summary'] + ' ' if record['summary'] else '') + suggestion['analysis_summary']
                record['guardrail_notes'] = list(suggestion.get('guardrail_notes', []))
                if not rolled_back:
                    record['action'] = suggestion.get('tuning_action', 'ADJUST_PARAMETERS')

                next_params = suggestion['parameters']
                if suggestion.get('tuning_action') == 'OBSERVE':
                    self._update_state(history=deepcopy(history))
                    continue

                self._update_state(
                    phase='applying',
                    message=f'第 {round_idx}/{cfg["max_rounds"]} 轮：写入并回读核验下一组安全参数…',
                    history=deepcopy(history),
                )
                for key, val in expand_loop_writes(loop_id, next_params):
                    self.session.write_param(key, val)

        except InterruptedError as exc:
            reason = 'stopped_by_user'
            self._update_state(message=str(exc))
        except Exception as exc:
            reason = 'error'
            self.session.send_estop()
            self._update_state(message=f'自动调参异常中止并已急停：{exc}')
        finally:
            self.session.send_estop()
            final_msg = {
                'converged': f'自动调参已收敛！最佳结果来自第 {best["round"]} 轮（评分 {best["score"]}）。' if best else '自动调参已收敛。',
                'max_rounds_reached': f'已完成 {len(history)} 轮实验，已保留第 {best["round"]} 轮最佳参数（评分 {best["score"]}）。' if best else '已完成全部实验轮次。',
                'stopped_by_user': '已停止自动调参并确保停车。',
            }.get(reason, self._state.get('message', '已结束'))
            self._update_state(
                active=False,
                phase='done' if reason in ('converged', 'max_rounds_reached') else reason,
                completed_reason=reason,
                best=deepcopy(best),
                history=deepcopy(history),
                rejected_count=len(rejected),
                finished_at=time.strftime('%H:%M:%S'),
                message=final_msg,
            )


    def chat(self, cfg: dict[str, Any]) -> dict[str, Any]:
        message = str(cfg.get('message') or '').strip()
        if not message:
            raise ValueError('请输入要和 AI 讨论的内容。')
        loop_id = str(cfg.get('loop') or self._state.get('loop') or 'yaw_2s1p')
        if loop_id not in LOOP_PROFILES:
            loop_id = 'yaw_2s1p'
        provider = str(cfg.get('provider') or 'rule').strip().lower()
        if provider not in ('rule', 'local_cli', 'openai', 'ollama', 'anthropic'):
            provider = 'rule'
        cli_tool = str(cfg.get('cli_tool') or 'auto').strip().lower()
        effort = normalize_effort(cfg.get('reasoning_effort'))
        dry_mode = bool(cfg.get('dry_mode', False))
        bounds = get_loop_bounds(loop_id, dry_mode=dry_mode)

        gp1 = None
        if self.bridge.status().get('connected'):
            try:
                gp1 = self.session.gp1 or self.session.sync_gp1(timeout_s=1.0)
            except Exception:
                gp1 = self.session.gp1

        live = cfg.get('live_telemetry') if isinstance(cfg.get('live_telemetry'), dict) else {}
        with self._lock:
            tune_history = deepcopy(self._state.get('history', []))
            best = deepcopy(self._state.get('best'))

        if provider == 'rule':
            return _build_offline_chat_reply(
                message=message,
                loop_id=loop_id,
                gp1=gp1,
                vbat_mv=self.session.vbat_mv,
                live=live,
                history=tune_history,
                best=best,
                bounds=bounds,
                effort=effort,
                dry_mode=dry_mode,
            )

        api_key = str(cfg.get('api_key') or '').strip()
        if not api_key:
            if provider == 'anthropic':
                api_key = os.environ.get('TUNEPRO_AI_API_KEY') or os.environ.get('ANTHROPIC_API_KEY') or ''
            elif provider == 'openai':
                api_key = os.environ.get('TUNEPRO_AI_API_KEY') or os.environ.get('OPENAI_API_KEY') or ''

        try:
            return call_llm_chat(
                provider=provider,
                base_url=str(cfg.get('base_url') or '').strip(),
                model=str(cfg.get('model') or '').strip(),
                api_key=api_key.strip(),
                reasoning_effort=effort,
                message=message,
                chat_history=cfg.get('history') if isinstance(cfg.get('history'), list) else [],
                loop_id=loop_id,
                gp1=gp1,
                vbat_mv=self.session.vbat_mv,
                live=live,
                tune_history=tune_history,
                best=best,
                bounds=bounds,
                dry_mode=dry_mode,
                cli_tool=cli_tool,
            )
        except Exception as exc:
            fallback = _build_offline_chat_reply(
                message=message,
                loop_id=loop_id,
                gp1=gp1,
                vbat_mv=self.session.vbat_mv,
                live=live,
                history=tune_history,
                best=best,
                bounds=bounds,
                effort=effort,
                dry_mode=dry_mode,
            )
            fallback['fallback_used'] = True
            fallback['insights'] = [f'外部模型未响应（{str(exc)[:72]}），已自动切内置诊断引擎回复。', *fallback['insights']]
            return fallback


AI_TUNER = AITunerManager(serial_bridge.BRIDGE)
