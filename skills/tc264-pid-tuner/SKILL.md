---
name: tc264-pid-tuner
description: Evidence-based PID auto-tuning and telemetry diagnosis for TC264 smart cars (Two-Series-One-Parallel 2S1P and Classic 3-Cascade) connected to TunePro or MOCK0. Use for step-response analysis, bounded gain updates, multi-loop tuning order, overshoot/oscillation diagnosis, and rollback-safe closed-loop experiments.
metadata:
  version: "1.2.0"
---

# TC264 PID Tuner Skill

Tune a TC264 differential vision car through attributable, bounded experiments:
inspect topology & current gains, measure a 4 ms baseline step, propose one
bounded change, verify readback and measured response, and keep or roll back.

This skill runs on Python 3.10+ standard library only (zero external packages).
Its experiment-loop discipline is adapted from `KINGSTON-115/llm-pid-tuner` (MIT)
and tailored to TC264 **2S1P (两串一并)** and **3-Cascade (传统三串)** cars.

## 1. Confirm topology, loop order, and safety first

Read [references/tc264-loops.md](references/tc264-loops.md) before changing any
parameter. Always follow the smart-car multi-loop order:

1. **Speed / Average-speed inner loop first** (`speed_2s1p` or `speed_3c`):
   verify encoder polarity, clean step rise, and no steady PWM saturation.
2. **Yaw-rate inner loop second** (`yaw_2s1p` direct-PWM or `yaw_3c` cascade):
   keep dry (`dry_mode=true`, `ykp <= 14`) and suction (`ykp <= 36`) ceilings
   separate. Never copy 3-Cascade gains (`0.22`) into 2S1P (`6.5~18.0`) or vice versa.
3. **Outer vision PD last** (`outer`: `dirp_cur / dird_cur`), only after inner
   loops track cleanly.

### Hard safety interlocks (never bypass)
- **Offline first**: on `MOCK0`, all experiments run safely without moving real wheels.
- **Physical COM port**: never trigger motion (`step-test`, `auto-tune`) or write
  live gains (`apply`) unless the user has explicitly authorized real-car tuning
  AND confirmed wheels are lifted off the ground (`--allow-real-car --wheels-off-ground`).
- **3S battery floor**: stop immediately if battery drops below `11.70 V` under fan
  load (or `10.80 V` dry).
- **Emergency stop**: on any oscillation blow-up, lost telemetry, or unexpected motion,
  immediately invoke `estop`.

## 2. Measure before proposing

Use `scripts/tunepro_ai_cli.py` against a running TunePro instance (`http://127.0.0.1:18760`
by default) or against offline CSV logs in `./runs/*/m2-decoded.csv`:

```sh
# Read current @GP1 parameters, topology, battery, and safety bounds
python <skill-dir>/scripts/tunepro_ai_cli.py snapshot --loop yaw_2s1p

# Analyze an offline run log (TunePro m2-decoded.csv or generic timestamp_ms,setpoint,input CSV)
python <skill-dir>/scripts/tunepro_ai_cli.py analyze-csv --csv runs/run01/m2-decoded.csv --channel yaw

# Validate a bounded proposal JSON without writing to the car
python <skill-dir>/scripts/tunepro_ai_cli.py check-update --file proposal.json
```

Rules for measurement:
- Never compare metrics across different topologies, targets, or fan states.
- Multi-step captures (`step-stair`, `yaw-alt`) evaluate the **second dynamic edge**
  to skip static friction deadzone; note which edge was measured.
- Always distinguish `mock_simulation` from `measured_hardware` in your report.

## 3. Execute bounded experiment loop

```sh
# 1) Measure baseline 4 ms step capture
python <skill-dir>/scripts/tunepro_ai_cli.py step-test --loop yaw_2s1p --target 300

# 2) Apply a single bounded change (strictly checked against min/max and max_step, with @GP1 readback)
python <skill-dir>/scripts/tunepro_ai_cli.py apply --loop yaw_2s1p --params '{"yff": 1.85, "ykp": 14.0, "yki": 0.12}'

# 3) Re-measure under the exact same target/conditions
python <skill-dir>/scripts/tunepro_ai_cli.py step-test --loop yaw_2s1p --target 300

# 4) If metrics regress or oscillate, restore last verified baseline immediately
python <skill-dir>/scripts/tunepro_ai_cli.py rollback

# Emergency stop at any time
python <skill-dir>/scripts/tunepro_ai_cli.py estop
```

Decision guidelines:
- Preserve working gains; do not reset `yki` or `yff` to zero without evidence.
- Change the gain with strongest physical evidence first:
  - **Oscillation / crossings >= 5**: reduce `ykp`/`vkp` by 15–25% and trim `ki`.
  - **Overshoot above limit**: reduce `kp` and `ki`; check `pwm_sat_pct` for windup.
  - **Slow rise with PWM headroom**: increase `kp` within `max_step`.
  - **Steady-state bias with acceptable rise**: in `yaw_2s1p`, tune `yff` first before raising `yki`; in speed loops, raise `ki` cautiously.
- When acceptance limits are met, **hold gains and repeat one verification trial**
  before declaring convergence.
- Keep a compact trial summary (round, source, confirmed readback gains, rise/overshoot/steady_err/sat%, decision, best verified baseline).
