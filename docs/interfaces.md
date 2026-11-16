# FOC interface contract

The exported `FOC Motion Controller` has one `single`-precision step every **50 µs**. The current loop runs on each step. The position P and speed PI loops commit their state once every 20 steps, at **1 kHz**. MATLAB checks the name, order, type and width of the root ports before ERT generates code. The C adapter checks the ABI version, rates and structure sizes at compile time.

## ABI v2

| Direction | Port | Width | Value |
| --- | --- | ---: | --- |
| Input | `i_phase_bc_a` | 2 | Phase currents `[Ib, Ic]` in A |
| Input | `theta_e_rad` | 1 | Electrical rotor angle in rad |
| Input | `i_override_dq_a` | 2 | Direct `[Id_ref, Iq_ref]` in A during alignment |
| Input | `vbus_v` | 1 | Measured DC-bus voltage in V |
| Input | `vmax_phase_v` | 1 | Limit on the phase-voltage vector in V |
| Input | `gain_pi` | 4 | `[Kp_d, Ki_d, Kp_q, Ki_q]` in V/A and V/(A·s) |
| Input | `mechanical_feedback` | 2 | Axis position (rad) and speed (rad/s) |
| Input | `motion_command` | 2 | Mode and target. 0 alignment, 1 torque (N·m), 2 speed (rad/s), 3 position (rad) |
| Input | `motion_gains` | 3 | Speed Kp and Ki, then position Kp |
| Input | `motion_limits` | 5 | Iq (A), speed (rad/s), acceleration (rad/s²), Iq slew (A/s), Kt (N·m/A) |
| Input | `previous_state` | 8 | Filtered Id and Iq (A), d/q integrators (V), speed integrator (A), speed reference (rad/s), Iq reference (A), divider phase |
| Output | `next_state` | 8 | Updated state in the same order and units |
| Output | `v_dq_v` | 2 | Limited command `[Vd, Vq]` in V |
| Output | `duty_abc` | 3 | Phase PWM duty fractions `[A, B, C]` |

## Firmware call sequence

1. Firmware initializes the controller before enabling the ADC interrupt.
2. The ADC interrupt converts the active axis's current samples into amperes and supplies its electrical angle, mechanical feedback, command and saved state.
3. `foc_motion_step()` writes those inputs and calls `odrive_foc_core_step()`. Torque mode uses Kt to convert torque to Iq. Speed mode uses the speed PI. Position mode feeds the speed PI from the position P loop.
4. The adapter checks finite values, operating bounds and duty before the firmware updates that axis's state and PWM. An invalid step raises a numeric fault.

The ADC interrupt owns the generated I/O buffers. Firmware retains state independently for each axis and resets it on a mode change. The Simulink bench uses an explicit unit delay to close the same state path. Generate a fresh export when upgrading from the original current-loop ABI.

[Model contract](../simulink/odrive_foc_contract.m) · [C interface](../include/foc.h) · [Adapter](../simulink/firmware/foc_simulink.c) · [ADC control code](../src/app.c) · [中文接口说明](three_loop.md)

## Reference hardware

The reference profile targets STM32F405RGT6 with two DRV8301 channels and two AS5600 devices. M0 uses software I²C on PB4/PB5 and M1 uses I²C1 on PB6/PB7. For each axis the first pin is SCL and the second is SDA.

The reference bench profile uses an **8.0–12.2 V** bus window, a **0.20 A** current command limit and a **0.60 A** current trip threshold. Consult the configuration and board pin assignments when adapting the firmware to another driver.

[Control and protection settings](../include/config.h) · [Board pin assignments](../src/board.c) · [Encoder transport](../src/encoder.c)
