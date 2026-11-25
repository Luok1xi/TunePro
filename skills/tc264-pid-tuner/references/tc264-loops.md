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

## 4. Safety Interlocks

- All `#W` writes use `rev` CAS + CRC-16/CCITT-FALSE and require `@WA1` + `@GP1` readback confirmation.
- Remote motion sends `#X PING` every ~320 ms (firmware watchdog stops car if link drops).
- Physical serial ports refuse motion/writes unless explicit real-car + wheels-off-ground flags are set.
- 3S battery floor: `< 11.70 V` with fan on (`< 10.80 V` dry) triggers immediate `#!` E-stop.
