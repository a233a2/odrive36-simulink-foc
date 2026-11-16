function contract = odrive_foc_contract()
% Root-port order and vector widths used by the STM32 adapter and tests.
contract = struct;
contract.model = 'odrive_foc';
contract.subsystem = 'FOC Motion Controller';
contract.codegen_model = 'odrive_foc_core';
contract.revision = 2;
contract.sample_hz = 20000;
contract.motion_hz = 1000;
contract.current_inputs = {
    'i_phase_bc_a', 2;
    'theta_e_rad', 1;
    'i_ref_dq_a', 2;
    'vbus_v', 1;
    'vmax_phase_v', 1;
    'gain_pi', 4;
    'previous_state', 4;
};
contract.current_outputs = {
    'next_state', 4;
    'v_dq_v', 2;
    'duty_abc', 3;
};
contract.inputs = {
    'i_phase_bc_a', 2;
    'theta_e_rad', 1;
    'i_override_dq_a', 2;
    'vbus_v', 1;
    'vmax_phase_v', 1;
    'gain_pi', 4;
    'mechanical_feedback', 2;  % position [rad], speed [rad/s]
    'motion_command', 2;       % mode, target [Nm / rad/s / rad]
    'motion_gains', 3;         % Kp_speed, Ki_speed, Kp_position
    'motion_limits', 5;        % Iq, speed, acceleration, Iq slew, Kt
    'previous_state', 8;
};
contract.outputs = {'next_state',8; 'v_dq_v',2; 'duty_abc',3};
end
