function create_odrive_foc_model(output_folder)
% Create an editable PMSM bench around the one deployable three-loop subsystem.
if nargin == 0
    output_folder = fileparts(mfilename('fullpath'));
end
if ~isfolder(output_folder)
    mkdir(output_folder);
end
contract = odrive_foc_contract();
model = contract.model;
model_file = fullfile(output_folder,[model '.slx']);
assert(~isfile(model_file) && ~bdIsLoaded(model), ...
    'Choose an empty output folder and close %s before recreating the model.',model);
load_system('simulink');
new_system(model);
close_model = onCleanup(@() close_system(model,0));
ts = sprintf('%.16g',1/contract.sample_hz);
set_param(model,'Description', ...
    'ODrive 3.6 three-loop bench. Export the FOC Motion Controller subsystem to the STM32.', ...
    'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',ts, ...
    'StartTime','0','StopTime','4','EnableMultiTasking','off', ...
    'AlgebraicLoopMsg','error','SignalLogging','on','SignalLoggingName','logsout');

% Demo parameters reside in the SLX workspace. Kt and mechanics are illustrative.
workspace = get_param(model,'ModelWorkspace');
assignin(workspace,'motor',struct('R_ohm',8,'L_h',0.00425, ...
    'pole_pairs',7,'flux_wb',0.003,'inertia_kgm2',3e-5,'damping_nms',3e-4));
assignin(workspace,'demo_mode',2);
assignin(workspace,'demo_target',2);
assignin(workspace,'demo_load_nm',0.0014);

controller_name = contract.subsystem;
controller = [model '/' controller_name];
add_block('built-in/SubSystem',controller,'Position',[429 71 660 620], ...
    'TreatAsAtomicUnit','on','BackgroundColor','[0.82, 0.91, 0.98]', ...
    'Description','Position P and speed PI at 1 kHz, current FOC and PWM at 20 kHz.');
create_odrive_foc_controller(controller,contract);
set_param(controller,'ContentPreviewEnabled','off');
at_input(model,'built-in/From','Phase current feedback',[controller_name '/1'],163, ...
    'GotoTag','phase_bc','ShowName','off');
at_input(model,'built-in/From','Electrical angle feedback',[controller_name '/2'],163, ...
    'GotoTag','theta_e','ShowName','off');
at_input(model,'built-in/Constant','Alignment override',[controller_name '/3'],163, ...
    'Value','single([0 0])');
at_input(model,'built-in/Constant','DC bus V',[controller_name '/4'],163,'Value','single(12)');
at_input(model,'built-in/Constant','Phase limit V',[controller_name '/5'],163,'Value','single(2)');
at_input(model,'built-in/Constant','Current PI gains',[controller_name '/6'],163, ...
    'Value','single(2*pi*300*[motor.L_h motor.R_ohm motor.L_h motor.R_ohm])');

at_input(model,'built-in/From','Mechanical feedback',[controller_name '/7'],163, ...
    'GotoTag','mech_fb','ShowName','off');
command_y = port_y(controller,'Inport',8);
command = stage(model,'Motion command',[140 command_y-20 279 command_y+20], ...
    {},{'mode_target'},'[0.94, 0.94, 0.94]');
demo_command(command,ts);
wire(model,{'Motion command/1',[controller_name '/8']});
log_line(model,'Motion command/1',[controller_name '/8'],'motion_command');
at_input(model,'built-in/Constant','Speed and position gains',[controller_name '/9'],163, ...
    'Value','single([0.045 0.20 3.0])');
at_input(model,'built-in/Constant','Motion limits',[controller_name '/10'],163, ...
    'Value','single([0.20 3 12 2 1.5*motor.pole_pairs*motor.flux_wb])');
at_input(model,'built-in/From','Controller prior state',[controller_name '/11'],163, ...
    'GotoTag','axis_state','ShowName','off');
send(model,'bus','DC bus V/1',276,port_y(controller,'Inport',4)+17,'Bus for inverter');

memory = at_output(model,'built-in/UnitDelay','Axis state z^-1',[controller_name '/1'],739, ...
    'SampleTime',ts,'InitialCondition','single(zeros(1,8))');
send(model,'axis_state','Axis state z^-1/1',860,port_y(memory,'Outport',1));
ref_y = port_y(controller,'Outport',1)+65;
gain(model,'Dq reference for scopes',[753 ref_y-18 860 ref_y+18], ...
    'single([zeros(1,8); 0 0 0 0 0 0 1 0])','Matrix(K*u) (u vector)');
wire(model,{[controller_name '/1'],'Dq reference for scopes/1'});
tag = send(model,'idq_ref','Dq reference for scopes/1',900,ref_y);
log_line(model,'Dq reference for scopes/1',[tag '/1'],'idq_ref_a');
send(model,'voltage_dq',[controller_name '/2'],722,port_y(controller,'Outport',2));

duty_y = port_y(controller,'Outport',3);
inverter = stage(model,'Averaged inverter',[873 duty_y-20 1019 duty_y+60], ...
    {'duty_abc','vbus_v'},{'phase_voltage_v'},'[0.96, 0.93, 0.85]');
averaged_inverter(inverter);
at_input(model,'built-in/From','Inverter DC bus','Averaged inverter/2',753, ...
    'GotoTag','bus','ShowName','off');
wire(model,{[controller_name '/3'],'Averaged inverter/1'});
log_line(model,[controller_name '/3'],'Averaged inverter/1','duty_abc');
send(model,'duty',[controller_name '/3'],717,duty_y+76,'Duty for scopes');

inverter_y = port_y(inverter,'Outport',1);
plant = stage(model,'PMSM with load input',[1148 inverter_y-57 1374 inverter_y+167], ...
    {'phase_voltage_v','load_torque_nm'}, ...
    {'current_bc_a','theta_e_rad','current_dq_a','speed_rad_s','torque_nm','position_rad'}, ...
    '[0.86, 0.94, 0.87]');
pmsm(plant,ts);
wire(model,{'Averaged inverter/1','PMSM with load input/1'});
at_input(model,'built-in/Step','Load torque step','PMSM with load input/2',930, ...
    'Time','0.65','Before','single(0)','After','single(demo_load_nm)', ...
    'SampleTime',ts,'OutDataTypeStr','single');
log_line(model,'Load torque step/1','PMSM with load input/2','load_torque_nm');
send(model,'load','Load torque step/1',1044,inverter_y+151,'Load for scopes');
for entry = {
    'phase_bc',1,''; 'theta_e',2,''; ...
    'idq_actual',3,'idq_motor_a'; 'speed',4,'speed_rad_s'; ...
    'torque',5,'torque_nm'; 'position',6,'position_rad'
    }.'
    source = sprintf('PMSM with load input/%d',entry{2});
    tag = send(model,entry{1},source,1444,port_y(plant,'Outport',entry{2}));
    if ~isempty(entry{3}), log_line(model,source,[tag '/1'],entry{3}); end
end

put(model,'built-in/Mux','Position and speed',[1258 765 1268 829],'Inputs','2','DisplayOption','bar');
at_input(model,'built-in/From','Position for controller','Position and speed/1',1117, ...
    'GotoTag','position','ShowName','off');
at_input(model,'built-in/From','Speed for controller','Position and speed/2',1117, ...
    'GotoTag','speed','ShowName','off');
send(model,'mech_fb','Position and speed/1',1339,798);
scope(model,'Current Scope',[330 733 410 813], ...
    {'idq_ref','idq_actual'},{'dq ref (A)','dq motor (A)'});
scope(model,'Motion Scope',[680 754 760 834], ...
    {'position','speed','torque','load'},{'Position (rad)','Speed (rad/s)','Te (N m)','Load (N m)'});
scope(model,'Voltage and PWM Scope',[971 736 1051 816], ...
    {'voltage_dq','duty'},{'Vdq (V)','Duty abc'});

set_param(model,'SimulationCommand','update');
save_system(model,model_file);
fprintf('Saved %s\n',model_file);
end

function demo_command(s,ts)
% Mode 1 torque [Nm], 2 speed [rad/s], 3 position [rad].
constant(s,'Mode 1 torque 2 speed 3 position',[67 55 160 84],'single(demo_mode)');
put(s,'built-in/Step','Target step',[67 129 160 158], ...
    'Time','0.04','Before','single(0)','After','single(demo_target)', ...
    'SampleTime',ts,'OutDataTypeStr','single');
put(s,'built-in/Mux','Mode and target',[246 65 255 152],'Inputs','2','DisplayOption','bar');
place_port(s,'mode_target',339,109);
wire(s,{'Mode 1 torque 2 speed 3 position/1','Mode and target/1'; ...
    'Target step/1','Mode and target/2'; 'Mode and target/1','mode_target/1'});
end


function averaged_inverter(s)
% Average each bridge leg over one PWM period, then remove neutral voltage.
gain(s, 'Remove common mode', [158 62 270 105], ...
    'single(eye(3) - ones(3)/3)', 'Matrix(K*u) (u vector)');
put(s, 'built-in/Product', 'Duty times DC bus', [319 71 374 126], 'Inputs', '**');
wire(s, {'duty_abc/1','Remove common mode/1'; ...
    'Remove common mode/1','Duty times DC bus/1'; 'vbus_v/1','Duty times DC bus/2'; ...
    'Duty times DC bus/1','phase_voltage_v/1'});
end

function pmsm(s, ts)
% Ld = Lq. Stationary-frame electrical dynamics give an independent plant
% for testing the rotating-frame controller.
electrical = stage(s, 'Electrical dynamics', [246 82 402 242], ...
    {'phase_voltage_v','back_emf_ab_v'}, {'current_bc_a','current_ab_a'}, '[0.86, 0.94, 0.87]');
electrical_dynamics(electrical, ts);
rotor = stage(s, 'Rotor current and EMF', [559 172 714 341], ...
    {'current_ab','theta_e','speed'}, {'idq_a','back_emf_ab','torque_nm'}, '[0.86, 0.94, 0.87]');
rotor_signals(rotor);
mechanical = stage(s, 'Mechanical dynamics', [559 390 714 492], ...
    {'electromagnetic_torque','load_torque'}, {'speed','theta_e','position'}, '[0.86, 0.94, 0.87]');
mechanical_dynamics(mechanical, ts);
place_port(s, 'phase_voltage_v', 33, port_y(electrical, 'Inport', 1));
place_port(s, 'load_torque_nm', 33, port_y(mechanical, 'Inport', 2));
place_port(s, 'current_bc_a', 977, 74);
place_port(s, 'theta_e_rad', 977, 125);
place_port(s, 'current_dq_a', 977, port_y(rotor, 'Outport', 1));
place_port(s, 'speed_rad_s', 977, 364);
place_port(s, 'torque_nm', 977, 541);
place_port(s, 'position_rad', 977, 593);
wire(s, {'phase_voltage_v/1','Electrical dynamics/1'; ...
    'Electrical dynamics/2','Rotor current and EMF/1'; ...
    'load_torque_nm/1','Mechanical dynamics/2'; 'Rotor current and EMF/1','current_dq_a/1'});
send(s, 'phase_bc', 'Electrical dynamics/1', 442, port_y(electrical, 'Outport', 1));
send(s, 'emf_ab', 'Rotor current and EMF/2', 753, port_y(rotor, 'Outport', 2));
send(s, 'torque', 'Rotor current and EMF/3', 753, port_y(rotor, 'Outport', 3));
send(s, 'speed', 'Mechanical dynamics/1', 753, port_y(mechanical, 'Outport', 1));
send(s, 'theta', 'Mechanical dynamics/2', 753, port_y(mechanical, 'Outport', 2));
send(s, 'position', 'Mechanical dynamics/3', 753, port_y(mechanical, 'Outport', 3));
at_input(s, 'built-in/From', 'Back EMF feedback', 'Electrical dynamics/2', 127, ...
    'GotoTag', 'emf_ab', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Rotor angle', 'Rotor current and EMF/2', 430, ...
    'GotoTag', 'theta', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Rotor speed', 'Rotor current and EMF/3', 430, ...
    'GotoTag', 'speed', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Torque for mechanics', 'Mechanical dynamics/1', 429, ...
    'GotoTag', 'torque', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Phase current output', 'current_bc_a/1', 861, ...
    'GotoTag', 'phase_bc', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Angle output', 'theta_e_rad/1', 861, 'GotoTag', 'theta', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Speed output', 'speed_rad_s/1', 861, 'GotoTag', 'speed', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Torque output', 'torque_nm/1', 861, 'GotoTag', 'torque', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Position output', 'position_rad/1', 861, 'GotoTag', 'position', 'ShowName', 'off');
end

function electrical_dynamics(s, ts)
% L * di_alpha_beta/dt = v_alpha_beta - R*i_alpha_beta - e_alpha_beta.
gain(s, 'Clarke voltage', [128 41 229 78], ...
    'single([2/3 -1/3 -1/3; 0 1/sqrt(3) -1/sqrt(3)])', 'Matrix(K*u) (u vector)');
sum_block(s, 'Winding voltage', [285 52 331 138], '+--');
gain(s, 'Winding resistance', [170 171 255 209], 'single(motor.R_ohm)');
integrate(s, 'Winding current', [394 77 482 113], '1/motor.L_h', ts);
gain(s, 'Inverse Clarke B C', [579 154 707 193], ...
    'single([-0.5 sqrt(3)/2; -0.5 -sqrt(3)/2])', 'Matrix(K*u) (u vector)');
place_port(s, 'current_ab_a', 786, 95);
place_port(s, 'current_bc_a', 786, 174);
send(s, 'current_ab', 'Winding current/1', 550, 222);
at_input(s, 'built-in/From', 'Resistance current', 'Winding resistance/1', 46, ...
    'GotoTag', 'current_ab', 'ShowName', 'off');
wire(s, {'phase_voltage_v/1','Clarke voltage/1'; 'Clarke voltage/1','Winding voltage/1'; ...
    'Winding resistance/1','Winding voltage/2'; 'back_emf_ab_v/1','Winding voltage/3'; ...
    'Winding voltage/1','Winding current/1'; 'Winding current/1','current_ab_a/1'; ...
    'Winding current/1','Inverse Clarke B C/1'; 'Inverse Clarke B C/1','current_bc_a/1'});
end

function rotor_signals(s)
% Radial [cos theta, sin theta] and tangent [-sin theta, cos theta].
put(s, 'built-in/Trigonometry', 'Cos theta', [134 92 202 122], 'Operator', 'cos');
put(s, 'built-in/Trigonometry', 'Sin theta', [134 154 202 184], 'Operator', 'sin');
put(s, 'built-in/Mux', 'Radial unit vector', [257 104 264 172], 'Inputs', '2', 'DisplayOption', 'bar');
gain(s, 'Tangent unit vector', [317 212 409 252], ...
    'single([0 -1; 1 0])', 'Matrix(K*u) (u vector)');
put(s, 'simulink/Math Operations/Dot Product', 'Id', [543 61 601 111]);
put(s, 'simulink/Math Operations/Dot Product', 'Iq', [543 161 601 211]);
put(s, 'built-in/Mux', 'Motor d q currents', [660 81 667 191], 'Inputs', '2', 'DisplayOption', 'bar');
gain(s, 'Torque per amp', [663 213 764 250], ...
    'single(1.5*motor.pole_pairs*motor.flux_wb)');
gain(s, 'Back EMF per speed', [325 308 432 346], ...
    'single(motor.pole_pairs*motor.flux_wb)');
put(s, 'built-in/Product', 'Back EMF alpha beta', [546 272 599 321], 'Inputs', '**');
send(s, 'i_ab', 'current_ab/1', 144, 53);
at_input(s, 'built-in/From', 'Current for Id', 'Id/1', 423, 'GotoTag', 'i_ab', 'ShowName', 'off');
at_input(s, 'built-in/From', 'Current for Iq', 'Iq/1', 423, 'GotoTag', 'i_ab', 'ShowName', 'off');
place_port(s, 'speed', 39, 327);
place_port(s, 'idq_a', 838, 136);
place_port(s, 'back_emf_ab', 838, 297);
place_port(s, 'torque_nm', 838, 232);
wire(s, {'theta_e/1','Cos theta/1'; 'theta_e/1','Sin theta/1'; ...
    'Cos theta/1','Radial unit vector/1'; 'Sin theta/1','Radial unit vector/2'; ...
    'Radial unit vector/1','Id/2'; 'Radial unit vector/1','Tangent unit vector/1'; ...
    'Tangent unit vector/1','Iq/2'; 'Id/1','Motor d q currents/1'; ...
    'Iq/1','Motor d q currents/2'; 'Motor d q currents/1','idq_a/1'; ...
    'Iq/1','Torque per amp/1'; 'Torque per amp/1','torque_nm/1'; ...
    'speed/1','Back EMF per speed/1'; 'Tangent unit vector/1','Back EMF alpha beta/1'; ...
    'Back EMF per speed/1','Back EMF alpha beta/2'; 'Back EMF alpha beta/1','back_emf_ab/1'});
end

function mechanical_dynamics(s, ts)
% J*d(omega_m)/dt = T_e - T_load - B*omega_m.
sum_block(s, 'Net torque', [273 54 315 142], '+--');
integrate(s, 'Mechanical speed', [369 78 465 116], '1/motor.inertia_kgm2', ts);
gain(s, 'Viscous damping', [164 176 254 212], 'single(motor.damping_nms)');
integrate(s, 'Rotor electrical angle', [536 136 642 175], 'motor.pole_pairs', ts);
constant(s, 'Two pi', [559 220 616 246], 'single(2*pi)');
put(s, 'built-in/Math', 'Wrap electrical angle', [696 147 768 202], 'Operator', 'mod');
gain(s, 'Mechanical angle radians', [689 274 788 309], 'single(1/motor.pole_pairs)');
send(s, 'omega_m', 'Mechanical speed/1', 508, 53);
at_input(s, 'built-in/From', 'Mechanical speed feedback', 'Viscous damping/1', 47, ...
    'GotoTag', 'omega_m', 'ShowName', 'off');
place_port(s, 'speed', 840, 97);
place_port(s, 'theta_e', 840, 175);
place_port(s, 'position', 840, 291);
wire(s, {'electromagnetic_torque/1','Net torque/1'; 'load_torque/1','Net torque/2'; ...
    'Viscous damping/1','Net torque/3'; 'Net torque/1','Mechanical speed/1'; ...
    'Mechanical speed/1','speed/1'; 'Mechanical speed/1','Rotor electrical angle/1'; ...
    'Rotor electrical angle/1','Wrap electrical angle/1'; 'Two pi/1','Wrap electrical angle/2'; ...
    'Rotor electrical angle/1','Mechanical angle radians/1'; ...
    'Mechanical angle radians/1','position/1'; 'Wrap electrical angle/1','theta_e/1'});
end

function scope(s, name, position, tags, labels)
put(s, 'simulink/Sinks/Scope', name, position, 'NumInputPorts', num2str(numel(tags)));
config = get_param([s '/' name], 'ScopeConfiguration');
config.ShowLegend = true;
if numel(tags) > 1
    config.LayoutDimensions = [numel(tags) 1];
end
for k = 1:numel(tags)
    dest = sprintf('%s/%d', name, k);
    h = at_input(s, 'built-in/From', [name ' input ' num2str(k)], dest, position(1)-171, ...
        'GotoTag', tags{k}, 'ShowName', 'off');
    ph = get_param(h, 'PortHandles');
    set_param(get_param(ph.Outport, 'Line'), 'Name', labels{k});
end
end

function integrate(s, name, pos, scale, ts)
put(s, 'built-in/DiscreteIntegrator', name, pos, ...
    'IntegratorMethod', 'Integration: Forward Euler', 'gainval', scale, ...
    'SampleTime', ts, 'InitialCondition', 'single(0)');
end

function p = stage(s, name, pos, in_names, out_names, color)
p = [s '/' name];
put(s, 'built-in/SubSystem', name, pos, 'BackgroundColor', color);
for k = 1:numel(in_names)
    put(p, 'built-in/Inport', in_names{k}, [26 53+54*(k-1)-7 56 53+54*(k-1)+7], 'Port', num2str(k));
end
for k = 1:numel(out_names)
    put(p, 'built-in/Outport', out_names{k}, [442 96+42*(k-1)-7 472 96+42*(k-1)+7], 'Port', num2str(k));
end
set_param(p, 'ContentPreviewEnabled', 'off');
end

function place_port(s, name, x, y)
set_param([s '/' name], 'Position', [x y-7 x+30 y+7]);
end

function gain(s, name, pos, value, multiplication)
if nargin == 4
    multiplication = 'Element-wise(K.*u)';
end
put(s, 'built-in/Gain', name, pos, 'Gain', value, 'Multiplication', multiplication);
end

function sum_block(s, name, pos, operations)
put(s, 'built-in/Sum', name, pos, 'Inputs', operations, 'IconShape', 'rectangular');
end

function constant(s, name, pos, value)
put(s, 'built-in/Constant', name, pos, 'Value', value, 'SampleTime', 'inf');
end

function h = at_input(s, library, name, destination, x, varargin)
y = target_y(s, destination, 'Inport');
h = put(s, library, name, [x y-9 x+82 y+9], varargin{:});
wire(s, {[name '/1'],destination});
end

function p = at_output(s, library, name, source, x, varargin)
y = target_y(s, source, 'Outport');
put(s, library, name, [x y-15 x+94 y+15], varargin{:});
p = [s '/' name];
wire(s, {source,[name '/1']});
end

function name = send(s, tag, source, x, y, name)
if nargin < 6
    name = ['Store ' tag];
end
put(s, 'built-in/Goto', name, [x y-9 x+90 y+9], 'GotoTag', tag, ...
    'TagVisibility', 'local', 'ShowName', 'off');
wire(s, {source,[name '/1']});
end

function y = target_y(s, endpoint, direction)
separator = find(endpoint == '/', 1, 'last');
y = port_y([s '/' endpoint(1:separator-1)], direction, str2double(endpoint(separator+1:end)));
end

function y = port_y(block, direction, index)
ports = get_param(block, 'PortHandles');
coordinates = get_param(ports.(direction)(index), 'Position');
y = coordinates(2);
end

function log_line(s, source, destination, name)
separator = find(destination == '/', 1, 'last');
ports = get_param([s '/' destination(1:separator-1)], 'PortHandles');
line = get_param(ports.Inport(str2double(destination(separator+1:end))), 'Line');
assert(line ~= -1, 'Missing line from %s to %s.', source, destination);
separator = find(source == '/', 1, 'last');
ports = get_param([s '/' source(1:separator-1)], 'PortHandles');
port = ports.Outport(str2double(source(separator+1:end)));
set_param(get_param(port, 'Line'), 'Name', name);
set_param(port, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end

function h = put(s, library, name, position, varargin)
h = add_block(library, [s '/' name], 'Position', position, varargin{:});
end

function wire(s, connections)
for k = 1:size(connections, 1)
    add_line(s, connections{k, 1}, connections{k, 2}, 'autorouting', 'on');
end
end
