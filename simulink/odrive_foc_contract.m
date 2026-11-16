function contract = odrive_foc_contract()
% Root-port order and vector widths used by the STM32 adapter and tests.
contract = struct;
contract.model = 'odrive_foc';
contract.subsystem = 'FOC Current Loop';
contract.codegen_model = 'odrive_foc_core';
contract.revision = 1;
contract.sample_hz = 20000;
contract.inputs = {
    'i_phase_bc_a', 2;
    'theta_e_rad', 1;
    'i_ref_dq_a', 2;
    'vbus_v', 1;
    'vmax_phase_v', 1;
    'gain_pi', 4;
    'previous_state', 4;
};
contract.outputs = {
    'next_state', 4;
    'v_dq_v', 2;
    'duty_abc', 3;
};
end
