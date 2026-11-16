# FOC interface contract

The model exports a single-rate, `single`-precision current loop at **20 kHz**, or **50 µs per call**. MATLAB checks the name, order, type and width of the root ports before ERT generates code. The C adapter checks the ABI version, clock rate and structure sizes at compile time.

| Direction | Port | Width | Value |
| --- | --- | ---: | --- |
| Input | `i_phase_bc_a` | 2 | Phase currents `[Ib, Ic]` in A |
| Input | `theta_e_rad` | 1 | Electrical rotor angle in rad |
| Input | `i_ref_dq_a` | 2 | Current commands `[Id_ref, Iq_ref]` in A |
| Input | `vbus_v` | 1 | Measured DC-bus voltage in V |
| Input | `vmax_phase_v` | 1 | Limit on the phase-voltage vector in V |
| Input | `gain_pi` | 4 | `[Kp_d, Ki_d, Kp_q, Ki_q]` in V/A and V/(A·s) |
| Input | `previous_state` | 4 | Filtered `[Id, Iq]` in A, then d/q PI integrators in V |
| Output | `next_state` | 4 | Updated state in the same order and units |
| Output | `v_dq_v` | 2 | Limited command `[Vd, Vq]` in V |
| Output | `duty_abc` | 3 | Phase PWM duty fractions `[A, B, C]` |

## Firmware call sequence

1. Firmware initializes the controller before enabling the ADC interrupt.
2. The ADC interrupt reads the active axis, converts samples into amperes and calculates the electrical angle. The outer loops supply the current command at 1 kHz.
3. `foc_current_step()` writes this axis's full input and previous state, then calls `odrive_foc_core_step()`.
4. The adapter checks finite values, duty bounds and the phase-voltage limit. A valid step updates the selected axis's state and PWM. An invalid step raises the firmware numeric fault.

The generated I/O buffers are owned by the ADC interrupt. Each axis has its own persistent filter and PI state. The Simulink bench uses an explicit unit delay to close the same state path.

[Model contract](../simulink/odrive_foc_contract.m) · [C interface](../include/foc.h) · [Adapter](../simulink/firmware/foc_simulink.c) · [ADC control code](../src/app.c)

## Reference hardware

The reference profile targets STM32F405RGT6 with two DRV8301 channels and two AS5600 devices. M0 uses software I²C on PB4/PB5 and M1 uses I²C1 on PB6/PB7. For each axis the first pin is SCL and the second is SDA.

The reference bench profile uses an **8.0–12.2 V** bus window, a **0.20 A** current command limit and a **0.60 A** current trip threshold. Consult the configuration and board pin assignments when adapting the firmware to another driver.

[Control and protection settings](../include/config.h) · [Board pin assignments](../src/board.c) · [Encoder transport](../src/encoder.c)
