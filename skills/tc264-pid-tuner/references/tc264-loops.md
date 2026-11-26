# TC264 Control Loops & Protocol Reference

## 1. Two-Series-One-Parallel (`topo=1`, 两串一并 2S1P)

Two independent inner branches mixed directly at PWM (`±8500` ceiling, steering-priority saturation transfer):

1. **Parallel average-speed loop (`speed_2s1p`, 500 Hz)**:
   - Feedback: `v_avg = 0.5 * (enc_l + enc_r)` (`cnt/2ms`)
   - Control law: `PWM_speed = 6.0 * v_des + vkp * (v_des - v_avg) + sum(vki * (v_des - v_avg))`
   - Firmware write keys: `vkp` writes both `lkp` and `rkp` (`scale * 100`); `vki` writes both `lki` and `rki` (`scale * 1000`).
   - Safe bounds: `vkp ∈ [4.0, 80.0]` (`max_step=8.0`), `vki ∈ [0.0, 6.0]` (`max_step=0.6`).
   - Step test: `mode=6` (`step-stair`, `0 -> test_low -> test_speed -> test_low -> 0`, 3.2 s).

2. **Series direct-PWM gyro inner loop (`yaw_2s1p`, 500 Hz)**:
   - Feedback: `gyro_z` (`°/s`), bypassing wheel-speed loop lag.
   - Control law: `PWM_turn = yff * w_des + ykp * (w_des - gyro_z) + sum(yki * (w_des - gyro_z))`
   - Firmware write keys: `yff` (`* 1000`), `ykp` (`* 10000`), `yki` (`* 10000`).
   - Safe bounds:
     - Suction mode: `yff ∈ [0.2, 5.0]` (`±0.45`), `ykp ∈ [1.0, 36.0]` (`±3.5`), `yki ∈ [0.0, 1.5]` (`±0.15`).
     - Dry mode (`--dry`): `ykp <= 14.0` (`±2.0`), `yki <= 0.5` (`±0.08`).
   - Reference starting points: suction `ykp=18.0, yff=1.85`; dry `ykp=6.5, yff=1.85`.
   - Step test: `mode=7` (`yaw-alt`, `0 -> +w -> -w -> +w -> 0`, 3.2 s). Evaluates 2nd edge (`+w -> -w`).

3. **Asymmetric mixer & diagnostics**:
   - Outer wheel `+0.72 * PWM_turn`, inner wheel `-1.28 * PWM_turn`.
   - Common-mode `pwm_cm = (pwm_l + pwm_r) / 2` shows longitudinal effort; differential `pwm_dm = (pwm_r - pwm_l) / 2` shows steering effort.

## 2. Classic 3-Cascade (`topo=0`, 传统三串)

1. **Independent wheel PI (`speed_3c`, 500 Hz)**:
   - Incremental PI: `dPWM = kp * (e - e_prev) + ki * e`
   - Keys: `lkp, rkp ∈ [2.0, 50.0]` (`±4.5`), `lki, rki ∈ [0.05, 6.0]` (`±0.5`).
2. **Yaw-rate middle loop (`yaw_3c`, 500 Hz)**:
   - Output is wheel speed difference (`cnt/2ms`), NOT direct PWM!
   - Keys: `ykp ∈ [0.02, 1.5]` (`±0.14`), `yki ∈ [0.0, 0.15]` (`±0.015`).

## 3. Outer Vision PD (`outer`)

- Control law: `w_des = clip(dirp_cur * err_px + dird_cur * d_err_px, -yaw_cap, yaw_cap)`.
- Keys: `dirp_cur ∈ [1.0, 40.0]` (`±2.5`, `* 1000`), `dird_cur ∈ [0.0, 40.0]` (`±3.0`, `* 1000`).
- Tune only after inner speed and yaw loops are verified stable.

## 4. Incremental PI/PID vs Positional PID/PD Tuning Semantics

Understanding controller form prevents tuning gains in the wrong direction:

1. **Incremental PI (`speed_3c`, `yaw_3c`: `u[k] = u[k-1] + Kp*(e[k]-e[k-1]) + Ki*e[k]`, `Kd = 0`)**:
   - **`Kp` (multiplies error delta `Δe`)**: Provides the initial kick on a setpoint jump, and acts as **braking / damping** while `e` shrinks (`Δe < 0`) during approach!
     - **`Kp` too small**: Sluggish initial kick; if `Ki` is moderate/large (`Kp/Ki` too small), braking is too weak and the curve overshoots & swings slowly.
     - **`Kp` too large**: Initial PWM spike / saturation (`±8500`), high-frequency sawtooth buzzing/chatter near setpoint (`crossings >= 6`, short period), or if `Ki` is tiny (`Kp/Ki` too huge) an initial jump followed by a painfully slow tail crawl.
   - **`Ki` (multiplies current error `e[k]`)**: Drives sustained climbing force and steady-state convergence.
     - **`Ki` too small**: Long slow approach tail or persistent steady-state droop even with decent `Kp`.
     - **`Ki` too large**: Severe **low-frequency overshoot** and wide slow sinusoidal swinging around target (`osc_period > 60 ms`).
   - **`Kd` (multiplies second difference `e[k] - 2e[k-1] + e[k-2]`)**:
     - Left at **`0`** in 500 Hz wheel-speed and gyro inner loops because second differencing amplifies encoder/IMU quantization noise by **4× per tick**, causing harsh PWM buzz and motor heating.
     - If used on a smooth inertial loop: too small fails to curb inertia peak; too large causes violent high-frequency actuator chatter.
   - **Balance rule of thumb**: For a first-order plant with time constant `τ` and period `Ts`, pole-zero cancellation suggests `Kp / Ki ≈ τ / Ts` (e.g. `τ ≈ 30 ms`, `Ts = 2 ms` -> `Kp/Ki ≈ 15`). When severely sluggish, raise `Kp` and `Ki` together instead of starving `Ki`.
   - **Independent L/R wheels (`speed_3c`)**: Left and right motors/gearboxes differ; inspect per-wheel metrics and tune `(lkp, lki)` independently from `(rkp, rki)`.

2. **Positional FF + PI (`yaw_2s1p`, `speed_2s1p`)**:
   - `yff` gives zero-phase-lag feedforward: if rise is acceptable but steady plateau is short/high, tune `yff` first before raising `yki`.
   - High-frequency buzz (`osc_type=high_freq_p_chatter`) -> cut `Kp` strongly (`-20%` to `-28%`).
   - Slow wide overshoot (`osc_type=low_freq_i_swing`) -> cut `Ki` strongly (`-30%` to `-40%`), trim `Kp` mildly.

3. **Positional PD (`outer`: `dirp_cur * e + dird_cur * Δe`)**:
   - `dirp_cur` too small -> lazy corner entry / large lateral error; too large -> overshooting corners & fishtailing.
   - `dird_cur` too small -> insufficient damping / repeated fishtailing; too large -> amplifies camera quantization noise into jittery steering.

## 5. Safety Interlocks

- All `#W` writes use `rev` CAS + CRC-16/CCITT-FALSE and require `@WA1` + `@GP1` readback confirmation.
- Remote motion sends `#X PING` every ~320 ms (firmware watchdog stops car if link drops).
- Physical serial ports refuse motion/writes unless explicit real-car + wheels-off-ground flags are set.
- 3S battery floor: `< 11.70 V` with fan on (`< 10.80 V` dry) triggers immediate `#!` E-stop.
