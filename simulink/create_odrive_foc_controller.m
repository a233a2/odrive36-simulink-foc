function create_odrive_foc_controller(s, contract)
% Assemble one stateless three-loop package with explicit per-axis state.
ts = sprintf('%.16g', 1 / contract.sample_hz);
input_tags = {'i_bc','theta_e','align_dq','vbus','v_limit','current_pi', ...
    'mech_fb','command','gains','limits'};
for k = 1:size(contract.inputs,1)
    name = contract.inputs{k,1};
    y = 60 + 54*(k-1);
    input(s,name,k,29,y,'OutDataTypeStr','single', ...
        'PortDimensions',num2str(contract.inputs{k,2}),'SampleTime',ts,'Interpolate','off');
    if k <= numel(input_tags), send(s,input_tags{k},[name '/1'],166,y); end
end
put(s,'built-in/Demux','Split axis state',[174 580 184 640],'Outputs','[4 4]');
wire(s,{'previous_state/1','Split axis state/1'});
send(s,'state_i','Split axis state/1',223,595);
send(s,'state_m','Split axis state/2',223,625);

outer = stage(s,'Motion loops',[426 267 597 497], ...
    {'command','mechanical','gains','limits','prior','align_i_dq'}, {'i_ref_dq','next_motion'});
create_motion_loops(outer,contract);
drive(s,'Motion loops',{'command','mech_fb','gains','limits','state_m','align_dq'},302);
send(s,'next_m','Motion loops/2',620,port_y(outer,'Outport',2));

inner = [s '/FOC Current Loop'];
put(s,'built-in/SubSystem','FOC Current Loop',[895 84 1080 431], ...
    'BackgroundColor','[0.84, 0.91, 0.98]');
create_current_controller(inner,ts,contract.current_inputs,contract.current_outputs);
set_param(inner,'ContentPreviewEnabled','off');
send(s,'i_ref_dq','Motion loops/1',620,port_y(outer,'Outport',1));
drive(s,'FOC Current Loop',{'i_bc','theta_e','i_ref_dq','vbus', ...
    'v_limit','current_pi','state_i'},764);
join_state = [s '/Join axis state'];
put(s,'built-in/Mux','Join axis state',[1280 106 1288 169],'Inputs','2','DisplayOption','bar');
align_input(join_state,1,inner,1);
for k = 1:size(contract.outputs,1)
    name = contract.outputs{k,1};
    y = port_y(inner,'Outport',k);
    if k == 1, y = port_y(join_state,'Outport',1); end
    output(s,name,k,1408,y,'OutDataTypeStr','single','PortDimensions',num2str(contract.outputs{k,2}));
end
wire(s,{'FOC Current Loop/1','Join axis state/1'});
read(s,'next_m','Join axis state/2',1148);
wire(s,{'Join axis state/1','next_state/1'; ...
    'FOC Current Loop/2','v_dq_v/1'; 'FOC Current Loop/3','duty_abc/1'});
end

function create_current_controller(s, ts, inputs, outputs)
% Current-loop layout is shared by calibration and all three run modes.
input_y = [206 103 424 475 545 605 664];
output_y = [91 146 404];
for k = 1:size(inputs, 1)
    input(s, inputs{k, 1}, k, 30, input_y(k), ...
        'OutDataTypeStr', 'single', 'PortDimensions', num2str(inputs{k, 2}), ...
        'SampleTime', ts, 'Interpolate', 'off');
end
for k = 1:size(outputs, 1)
    output(s, outputs{k, 1}, k, 1464, output_y(k), ...
        'OutDataTypeStr', 'single', 'PortDimensions', num2str(outputs{k, 2}));
end

trig = stage(s, 'Sin and cos', [145 79 265 127], {'theta'}, {'cos_sin'});
put(trig, 'built-in/Trigonometry', 'Cosine', [145 51 202 82], 'Operator', 'cos');
put(trig, 'built-in/Trigonometry', 'Sine', [145 110 202 141], 'Operator', 'sin');
put(trig, 'built-in/Mux', 'Cos sin', [275 64 282 127], 'Inputs', '2', 'DisplayOption', 'bar');
wire(trig, {'theta/1','Cosine/1'; 'theta/1','Sine/1'; ...
    'Cosine/1','Cos sin/1'; 'Sine/1','Cos sin/2'; 'Cos sin/1','cos_sin/1'});
send(s, 'cos_sin', 'Sin and cos/1', 306, 103);
wire(s, {'theta_e_rad/1','Sin and cos/1'});

clarke = stage(s, 'Clarke and Park', [193 176 329 261], ...
    {'ib_ic', 'cos_sin'}, {'id_iq'});
gain(clarke, 'Clarke B C to alpha beta', [130 46 249 84], ...
    'single([-1 -1; 0.57735026919 -0.57735026919])', 'Matrix(K*u) (u vector)');
park = stage(clarke, 'Park rotation', [307 53 419 130], {'stationary','cos_sin'}, {'rotating'});
rotation(park, false, 'stationary', 'rotating');
wire(clarke, {'ib_ic/1','Clarke B C to alpha beta/1'; ...
    'Clarke B C to alpha beta/1','Park rotation/1'; ...
    'cos_sin/1','Park rotation/2'; 'Park rotation/1','id_iq/1'});
read(s, 'cos_sin', 'Clarke and Park/2', 78);
wire(s, {'i_phase_bc_a/1','Clarke and Park/1'});

filter = stage(s, 'Current filter', [449 174 568 262], ...
    {'id_iq','previous'}, {'filtered'});
sum_block(filter, 'Current delta', [142 45 173 83], '+-');
gain(filter, 'Filter coefficient', [222 46 289 83], 'single(0.15)');
sum_block(filter, 'Update filter', [388 44 419 146], '++');
set_param([filter '/filtered'], 'Position', [470 88 500 102]);
send(filter, 'prior', 'previous/1', 115, 119);
read(filter, 'prior', 'Current delta/2', 49);
read(filter, 'prior', 'Update filter/2', 278, 'Previous filtered current');
wire(filter, {'id_iq/1','Current delta/1'; 'Current delta/1','Filter coefficient/1'; ...
    'Filter coefficient/1','Update filter/1'; 'Update filter/1','filtered/1'});
align_input(filter, 1, clarke, 1);
read(s, 'idq_prev', 'Current filter/2', 347);
wire(s, {'Clarke and Park/1','Current filter/1'});
send(s, 'idq_filt', 'Current filter/1', 590, 138);

pi_stage = stage(s, 'Current PI', [716 174 852 338], ...
    {'measured','reference','gains','integral','maximum'}, {'pi_value','i_candidate'});
build_pi(pi_stage, ts, 2);
align_input(pi_stage, 1, filter, 1);
wire(s, {'Current filter/1','Current PI/1'});
read(s, 'reference', 'Current PI/2', 607);
read(s, 'gain_pi', 'Current PI/3', 607);
read(s, 'integral_prev', 'Current PI/4', 607);
read(s, 'voltage_limit', 'Current PI/5', 607);

voltage_spacing = port_y(pi_stage, 'Outport', 2) - port_y(pi_stage, 'Outport', 1);
vector = stage(s, 'Voltage vector limit', [983 173 1111 173+3*voltage_spacing], ...
    {'v_raw','i_candidate','maximum'}, {'v_limited','i_updated'});
voltage_vector(vector);
align_input(vector, 1, pi_stage, 1);
wire(s, {'Current PI/1','Voltage vector limit/1'; ...
    'Current PI/2','Voltage vector limit/2'});
read(s, 'voltage_limit', 'Voltage vector limit/3', 872);
send(s, 'v_dq', 'Voltage vector limit/1', 1150, port_y(vector, 'Outport', 1)-42);
send(s, 'integral_next', 'Voltage vector limit/2', 1150, port_y(vector, 'Outport', 2)+10);

pwm = stage(s, 'Inverse Park and SVPWM', [1248 337 1395 470], ...
    {'v_dq','cos_sin','bus'}, {'duty'});
modulation(pwm);
align_input(pwm, 1, vector, 1);
wire(s, {'Voltage vector limit/1','Inverse Park and SVPWM/1'});
read(s, 'cos_sin', 'Inverse Park and SVPWM/2', 1145);
read(s, 'bus_v', 'Inverse Park and SVPWM/3', 1145);
set_param([s '/duty_abc'], 'Position', port_rect(1464, port_y(pwm, 'Outport', 1)));
wire(s, {'Inverse Park and SVPWM/1','duty_abc/1'});
read(s, 'v_dq', 'v_dq_v/1', 1335, 'Voltage monitor');

put(s, 'built-in/Mux', 'Pack state', [1377 60 1384 122], 'Inputs', '2', 'DisplayOption', 'bar');
read(s, 'idq_filt', 'Pack state/1', 1246);
read(s, 'integral_next', 'Pack state/2', 1246);
wire(s, {'Pack state/1','next_state/1'});

% Root ports stay scalar/vector single. Local tags keep the secondary
% inputs close to the blocks that consume them.
send(s, 'reference', 'i_ref_dq_a/1', 121, 424);
send(s, 'gain_pi', 'gain_pi/1', 121, 605);
put(s, 'built-in/Demux', 'Unpack state', [146 638 153 689], 'Outputs', '[2 2]');
wire(s, {'previous_state/1','Unpack state/1'});
send(s, 'idq_prev', 'Unpack state/1', 248, 651);
send(s, 'integral_prev', 'Unpack state/2', 248, 676);
gain(s, 'Bus headroom', [149 458 251 492], 'single(0.30)');
put(s, 'built-in/MinMax', 'Phase voltage limit', [304 457 364 561], 'Function', 'min', 'Inputs', '2');
wire(s, {'vbus_v/1','Bus headroom/1'; 'Bus headroom/1','Phase voltage limit/1'; ...
    'vmax_phase_v/1','Phase voltage limit/2'});
send(s, 'bus_v', 'vbus_v/1', 125, 518);
send(s, 'voltage_limit', 'Phase voltage limit/1', 409, 509);
end

function create_motion_loops(s, contract)
% Position P supplies the speed command. Speed PI or torque mode supplies Iq.
split_port(s,'command',{'mode','target'},[1 1],90);
split_port(s,'mechanical',{'position','speed'},[1 1],205);
split_port(s,'gains',{'speed_gains','position_kp'},[2 1],320);
split_port(s,'limits',{'iq_limit','speed_limit','accel_limit','iq_rate','kt'},ones(1,5),480);
split_port(s,'prior',{'speed_i','prior_speed','prior_iq','prior_tick'},ones(1,4),694);
set_param([s '/align_i_dq'],'Position',port_rect(23,836));
send(s,'align_ref','align_i_dq/1',225,836);

position = stage(s,'Position P',[487 80 631 178],{'target','measured','kp'},{'speed_request'});
sum_block(position,'Position error',[151 47 181 114],'+-');
product(position,'P gain',[248 56 293 133],'**');
wire(position,{'target/1','Position error/1'; 'measured/1','Position error/2'; ...
    'Position error/1','P gain/1'; 'kp/1','P gain/2'; 'P gain/1','speed_request/1'});
drive(s,'Position P',{'target','position','position_kp'},380);
send(s,'pos_request','Position P/1',649,129);
mode_compare(s,'Position mode',3,[490 220 540 272]);
send(s,'pos_mode','Position mode/1',572,246);
put(s,'built-in/Switch','Select speed request',[855 83 913 187],'Criteria','u2 ~= 0');
drive(s,'Select speed request',{'pos_request','pos_mode','target'},751);
send(s,'speed_req','Select speed request/1',939,135);

dt = sprintf('single(1/%d)',contract.motion_hz);
ramp(s,'Speed and acceleration limits',[1156 65 1306 209],dt);
drive(s,'Speed and acceleration limits',{'speed_req','speed_limit','prior_speed','accel_limit'},1048);
send(s,'speed_ref','Speed and acceleration limits/1',1332,137);
speed_pi = stage(s,'Speed PI',[1551 65 1689 221], ...
    {'measured','reference','gains','integral','maximum'},{'pi_value','i_candidate'});
build_pi(speed_pi,sprintf('1/%d',contract.motion_hz),1);
drive(s,'Speed PI',{'speed','speed_ref','speed_gains','speed_i','iq_limit'},1443);
send(s,'iq_speed','Speed PI/1',1724,port_y(speed_pi,'Outport',1));
send(s,'next_speed_i','Speed PI/2',1724,port_y(speed_pi,'Outport',2));

product(s,'Torque divided by Kt',[491 347 553 412],'*/');
drive(s,'Torque divided by Kt',{'target','kt'},380);
send(s,'iq_request','Torque divided by Kt/1',590,port_y([s '/Torque divided by Kt'],'Outport',1));
ramp(s,'Torque current and slew limits',[858 315 1032 468],dt);
drive(s,'Torque current and slew limits',{'iq_request','iq_limit','prior_iq','iq_rate'},751);
send(s,'iq_torque','Torque current and slew limits/1',1071,port_y([s '/Torque current and slew limits'],'Outport',1));

divider = stage(s,'1 kHz tick',[492 560 632 641],{'phase'},{'update','next_phase'});
build_divider(divider,contract.sample_hz/contract.motion_hz);
read(s,'prior_tick','1 kHz tick/1',380);
send(s,'outer_update','1 kHz tick/1',673,port_y(divider,'Outport',1));
send(s,'outer_tick','1 kHz tick/2',673,port_y(divider,'Outport',2));
send(s,'prior_motion','prior/1',225,782);

commit = stage(s,'Mode and state commit',[1958 128 2142 566], ...
    {'mode','align','speed_iq','speed_i','speed_ref','torque_iq','prior','update','phase'}, ...
    {'current_ref','state_next'});
build_motion_commit(commit);
drive(s,'Mode and state commit',{'mode','align_ref','iq_speed','next_speed_i', ...
    'speed_ref','iq_torque','prior_motion','outer_update','outer_tick'},1845);
place_out(s,'i_ref_dq',2275,port_y(commit,'Outport',1));
place_out(s,'next_motion',2275,port_y(commit,'Outport',2));
wire(s,{'Mode and state commit/1','i_ref_dq/1'; 'Mode and state commit/2','next_motion/1'});
end

function ramp(s, name, pos, dt)
p = stage(s,name,pos,{'target','limit','previous','rate'},{'reference'});
clamp(p,'Limit setpoint',[164 32 265 105]);
sum_block(p,'Step error',[353 45 390 116],'+-');
gain(p,'Step time',[273 183 344 219],dt);
clamp(p,'Limit rate',[472 43 574 134]);
sum_block(p,'Apply step',[648 52 680 123],'++');
send(p,'previous_step','previous/1',100,163);
read(p,'previous_step','Step error/2',244);
read(p,'previous_step','Apply step/2',545);
place_out(p,'reference',776,port_y([p '/Apply step'],'Outport',1));
wire(p,{'target/1','Limit setpoint/1'; 'limit/1','Limit setpoint/2'; ...
    'Limit setpoint/1','Step error/1'; 'Step error/1','Limit rate/1'; ...
    'rate/1','Step time/1'; 'Step time/1','Limit rate/2'; ...
    'Limit rate/1','Apply step/1'; 'Apply step/1','reference/1'});
end

function build_divider(s, ratio)
constant(s,'Base tick',[35 98 103 122],'single(1)');
sum_block(s,'Increment tick',[155 47 187 110],'++');
constant(s,'Ticks per update',[203 126 277 149],sprintf('single(%d)',ratio));
put(s,'built-in/RelationalOperator','Outer update',[320 50 376 118],'Operator','>=');
constant(s,'First tick',[232 182 278 206],'single(0)');
send(s,'increment','Increment tick/1',205,77);
send(s,'update_signal','Outer update/1',410,85);
put(s,'built-in/Switch','Wrap phase',[545 161 598 246],'Criteria','u2 ~= 0');
read(s,'update_signal','Wrap phase/2',425);
read(s,'increment','Wrap phase/3',425);
place_out(s,'update',663,85);
place_out(s,'next_phase',663,204);
wire(s,{'phase/1','Increment tick/1'; 'Base tick/1','Increment tick/2'; ...
    'Increment tick/1','Outer update/1'; 'Ticks per update/1','Outer update/2'; ...
    'First tick/1','Wrap phase/1'; 'Outer update/1','update/1'; 'Wrap phase/1','next_phase/1'});
end

function build_motion_commit(s)
% Hold outer-loop references for 19 inner ticks. Alignment clears outer state.
names = {'mode','align','speed_iq','speed_i','speed_ref','torque_iq','prior','update','phase'};
for k = 1:numel(names)
    send(s,names{k},[names{k} '/1'],116,53+54*(k-1));
end
constant(s,'Zero',[325 248 376 272],'single(0)');
send(s,'zero','Zero/1',407,260);
mode_compare(s,'Torque mode',1,[332 58 387 112]);
mode_compare(s,'Alignment mode',0,[332 145 387 199]);
send(s,'is_torque','Torque mode/1',433,85);
send(s,'is_align','Alignment mode/1',433,172);
put(s,'built-in/Mux','Speed candidate',[563 312 573 425],'Inputs','3','DisplayOption','bar');
put(s,'built-in/Mux','Torque candidate',[563 467 573 580],'Inputs','3','DisplayOption','bar');
drive(s,'Speed candidate',{'speed_i','speed_ref','speed_iq'},435);
drive(s,'Torque candidate',{'zero','zero','torque_iq'},435);
send(s,'candidate_speed','Speed candidate/1',609,368);
send(s,'candidate_torque','Torque candidate/1',609,523);
put(s,'built-in/Switch','Select active loop',[774 329 832 435],'Criteria','u2 ~= 0');
drive(s,'Select active loop',{'candidate_torque','is_torque','candidate_speed'},665);
send(s,'active_candidate','Select active loop/1',871,383);
select(s,'Previous references',[791 530 891 559],'4','[1 2 3]');
read(s,'prior','Previous references/1',669);
send(s,'held_references','Previous references/1',926,545);
put(s,'built-in/Switch','Commit on 1 kHz tick',[1064 344 1124 454],'Criteria','u2 ~= 0');
drive(s,'Commit on 1 kHz tick',{'active_candidate','update','held_references'},948);
put(s,'built-in/Mux','State with phase',[1243 368 1253 453],'Inputs','2','DisplayOption','bar');
wire(s,{'Commit on 1 kHz tick/1','State with phase/1'});
read(s,'phase','State with phase/2',1139);
send(s,'running_state','State with phase/1',1292,411);
constant(s,'Clear outer state',[1225 231 1309 259],'single(zeros(1,4))');
send(s,'clear_motion','Clear outer state/1',1347,245);
put(s,'built-in/Switch','Reset while aligning',[1524 341 1586 447],'Criteria','u2 ~= 0');
drive(s,'Reset while aligning',{'clear_motion','is_align','running_state'},1412);
place_out(s,'state_next',1736,394);
wire(s,{'Reset while aligning/1','state_next/1'});
select(s,'Iq reference',[1672 477 1784 512],'4','3');
wire(s,{'Reset while aligning/1','Iq reference/1'});
put(s,'built-in/Mux','Id zero and Iq',[1900 451 1910 523],'Inputs','2','DisplayOption','bar');
read(s,'zero','Id zero and Iq/1',1801);
wire(s,{'Iq reference/1','Id zero and Iq/2'});
send(s,'running_i_dq','Id zero and Iq/1',1958,486);
put(s,'built-in/Switch','Select current reference',[2181 421 2252 530],'Criteria','u2 ~= 0');
drive(s,'Select current reference',{'align','is_align','running_i_dq'},2059);
place_out(s,'current_ref',2341,475);
wire(s,{'Select current reference/1','current_ref/1'});
end

function split_port(s, name, tags, widths, y)
set_param([s '/' name],'Position',port_rect(23,y));
block = ['Unpack ' name];
height = 28*numel(tags);
put(s,'built-in/Demux',block,[157 y-height/2 167 y+height/2], ...
    'Outputs',mat2str(widths));
wire(s,{[name '/1'],[block '/1']});
for k = 1:numel(tags)
    send(s,tags{k},sprintf('%s/%d',block,k),225,port_y([s '/' block],'Outport',k));
end
end

function mode_compare(s, name, value, pos)
put(s,'built-in/RelationalOperator',name,pos,'Operator','==');
read(s,'mode',[name '/1'],pos(1)-110);
y=port_y([s '/' name],'Inport',2);
constant(s,[name ' value'],[pos(1)-71 y-10 pos(1)-25 y+10],sprintf('single(%d)',value));
set_param([s '/' name ' value'],'ShowName','off');
wire(s,{[name ' value/1'],[name '/2']});
end

function drive(s, name, tags, x)
for k = 1:numel(tags)
    read(s,tags{k},sprintf('%s/%d',name,k),x, ...
        sprintf('Read %s for %s %d',strrep(tags{k},'_',' '),name,k));
end
end

function place_out(s, name, x, y)
set_param([s '/' name],'Position',port_rect(x,y));
end

function build_pi(s, ts, axes)
% Identical conditional-integration PI law for speed and each current lane.
set_param([s '/integral'], 'Position', port_rect(23, 363));
set_param([s '/maximum'], 'Position', port_rect(23, 421));
sum_block(s, 'Error', [151 76 180 128], '-+');
select(s, 'Kp lanes', [146 156 235 191], num2str(2*axes), mat2str(1:2:2*axes));
select(s, 'Ki lanes', [146 221 235 256], num2str(2*axes), mat2str(2:2:2*axes));
send(s, 'error_dq', 'Error/1', 227, 102);
send(s, 'old_i', 'integral/1', 151, 363);
send(s, 'maximum', 'maximum/1', 151, 421);
wire(s, {'measured/1','Error/1'; 'reference/1','Error/2'; ...
    'gains/1','Kp lanes/1'; 'gains/1','Ki lanes/1'});
product(s, 'Proportional', [387 122 432 174], '**');
product(s, 'Integral rate', [387 221 432 273], '**');
send(s, 'p_term', 'Proportional/1', 474, 148);
read(s, 'error_dq', 'Proportional/1', 263);
read(s, 'error_dq', 'Integral rate/2', 263, 'Error for integral');
wire(s, {'Kp lanes/1','Proportional/2'; 'Ki lanes/1','Integral rate/1'});
gain(s, 'Ts', [480 226 565 267], ['single(' ts ')']);
sum_block(s, 'Integrate', [641 228 673 300], '++');
read(s, 'old_i', 'Integrate/2', 545);
wire(s, {'Integral rate/1','Ts/1'; 'Ts/1','Integrate/1'});
clamp(s, 'Clamp candidate', [788 239 887 340]);
wire(s, {'Integrate/1','Clamp candidate/1'});
read(s, 'maximum', 'Clamp candidate/2', 683, 'Candidate limit');

conditional = stage(s, 'Conditional integration', [1031 222 1186 447], ...
    {'p','i_candidate','i_previous','error','maximum'}, {'i_next'});
conditional_integral(conditional);
align_input(conditional, 2, [s '/Clamp candidate'], 1);
wire(s, {'Clamp candidate/1','Conditional integration/2'});
read(s, 'p_term', 'Conditional integration/1', 916);
read(s, 'old_i', 'Conditional integration/3', 916, 'Previous integral');
read(s, 'error_dq', 'Conditional integration/4', 916, 'Error for gate');
read(s, 'maximum', 'Conditional integration/5', 916, 'Conditional limit');
sum_block(s, 'Add P and I', [1322 273 1354 355], '++');
align_input([s '/Add P and I'], 2, conditional, 1);
read(s, 'p_term', 'Add P and I/1', 1212);
clamp(s, 'Clamp PI output', [1471 288 1575 404]);
align_input([s '/Clamp PI output'], 1, [s '/Add P and I'], 1);
read(s, 'maximum', 'Clamp PI output/2', 1363, 'PI output limit');
set_param([s '/pi_value'], 'Position', port_rect(1634, port_y([s '/Clamp PI output'], 'Outport', 1)));
set_param([s '/i_candidate'], 'Position', port_rect(1634, 440));
send(s, 'new_i', 'Conditional integration/1', 1226, 428);
read(s, 'new_i', 'i_candidate/1', 1517);
wire(s, {'Conditional integration/1','Add P and I/2'; ...
    'Add P and I/1','Clamp PI output/1'; 'Clamp PI output/1','pi_value/1'});
end

function conditional_integral(s)
sum_block(s, 'Candidate output', [146 58 179 102], '++');
put(s, 'built-in/Abs', 'Magnitude', [239 55 289 85]);
put(s, 'built-in/RelationalOperator', 'Within limit', [403 45 453 100], 'Operator', '<=');
product(s, 'Output times error', [254 186 290 230], '**');
put(s, 'built-in/RelationalOperator', 'Error unwinds', [403 186 453 230], 'Operator', '<');
constant(s, 'Zero', [321 241 357 266], 'single(0)');
put(s, 'built-in/Logic', 'Allow integral update', [522 105 573 159], 'Operator', 'OR', 'Inputs', '2');
put(s, 'built-in/Switch', 'Select integral', [665 108 713 189], 'Criteria', 'u2 ~= 0');
set_param([s '/i_next'], 'Position', [777 142 806 156]);
wire(s, {'p/1','Candidate output/1'; 'i_candidate/1','Candidate output/2'; ...
    'Candidate output/1','Magnitude/1'; 'Magnitude/1','Within limit/1'; ...
    'maximum/1','Within limit/2'; 'Candidate output/1','Output times error/1'; ...
    'error/1','Output times error/2'; 'Output times error/1','Error unwinds/1'; ...
    'Zero/1','Error unwinds/2'; 'Within limit/1','Allow integral update/1'; ...
    'Error unwinds/1','Allow integral update/2'; 'i_candidate/1','Select integral/1'; ...
    'Allow integral update/1','Select integral/2'; 'i_previous/1','Select integral/3'; ...
    'Select integral/1','i_next/1'});
set_param([s '/maximum'], 'Position', [298 93 327 107]);
set_param([s '/error'], 'Position', [120 223 149 237]);
set_param([s '/i_previous'], 'Position', [522 202 551 216]);
end

function voltage_vector(s)
scale = stage(s, 'Compute voltage scale', [152 46 300 127], {'voltage','maximum'}, {'scale','saturated'});
voltage_scale(scale);
product(s, 'Scale d q voltage', [396 57 442 111], '**');
sum_block(s, 'Integral back calculation', [512 188 552 254], '++-');
clamp(s, 'Clamp new integral', [626 193 754 300]);
put(s, 'built-in/Switch', 'Commit if saturated', [916 183 968 312], 'Criteria', 'u2 ~= 0');
send(s, 'raw_voltage', 'v_raw/1', 146, 153);
send(s, 'old_integral', 'i_candidate/1', 147, 206);
send(s, 'maximum', 'maximum/1', 147, 254);
read(s, 'raw_voltage', 'Scale d q voltage/1', 304);
read(s, 'old_integral', 'Integral back calculation/1', 400);
read(s, 'raw_voltage', 'Integral back calculation/3', 400, 'Original voltage');
read(s, 'maximum', 'Clamp new integral/2', 523);
read(s, 'old_integral', 'Commit if saturated/3', 800, 'Unchanged integral');
set_param([s '/v_raw'], 'Position', [27 60 57 74]);
set_param([s '/i_candidate'], 'Position', [27 199 57 213]);
set_param([s '/maximum'], 'Position', [27 247 57 261]);
set_param([s '/v_limited'], 'Position', port_rect(1030, 84));
set_param([s '/i_updated'], 'Position', port_rect(1030, 248));
wire(s, {'v_raw/1','Compute voltage scale/1'; 'maximum/1','Compute voltage scale/2'; ...
    'Compute voltage scale/1','Scale d q voltage/2'; 'Scale d q voltage/1','v_limited/1'; ...
    'Scale d q voltage/1','Integral back calculation/2'; ...
    'Integral back calculation/1','Clamp new integral/1'; 'Clamp new integral/1','Commit if saturated/1'; ...
    'Compute voltage scale/2','Commit if saturated/2'; 'Commit if saturated/1','i_updated/1'});
end

function voltage_scale(s)
put(s, 'simulink/Math Operations/Dot Product', 'Squared magnitude', [140 49 209 106]);
put(s, 'built-in/Math', 'Squared limit', [140 141 209 177], 'Operator', 'square');
put(s, 'built-in/RelationalOperator', 'Over voltage limit', [285 110 343 162], 'Operator', '>');
constant(s, 'One', [271 205 314 231], 'single(1)');
put(s, 'built-in/Switch', 'Safe squared divisor', [403 44 451 122], 'Criteria', 'u2 ~= 0');
put(s, 'simulink/Math Operations/Sqrt', 'Magnitude', [504 67 553 99]);
product(s, 'Limit over magnitude', [610 42 654 94], '*/');
put(s, 'built-in/Switch', 'Voltage scale', [819 58 868 172], 'Criteria', 'u2 ~= 0');
send(s, 'saturated', 'Over voltage limit/1', 386, 179);
read(s, 'saturated', 'Voltage scale/2', 706);
set_param([s '/scale'], 'Position', port_rect(937, 115));
set_param([s '/saturated'], 'Position', port_rect(937, 207));
read(s, 'saturated', 'saturated/1', 818, 'Saturation flag');
wire(s, {'voltage/1','Squared magnitude/1'; 'voltage/1','Squared magnitude/2'; ...
    'maximum/1','Squared limit/1'; 'Squared magnitude/1','Over voltage limit/1'; ...
    'Squared limit/1','Over voltage limit/2'; 'Squared magnitude/1','Safe squared divisor/1'; ...
    'Over voltage limit/1','Safe squared divisor/2'; 'One/1','Safe squared divisor/3'; ...
    'Safe squared divisor/1','Magnitude/1'; 'maximum/1','Limit over magnitude/1'; ...
    'Magnitude/1','Limit over magnitude/2'; 'Limit over magnitude/1','Voltage scale/1'; ...
    'One/1','Voltage scale/3'; 'Voltage scale/1','scale/1'});
end

function modulation(s)
rot = stage(s, 'Inverse Park rotation', [150 47 275 128], {'rotating','cos_sin'}, {'stationary'});
rotation(rot, true, 'rotating', 'stationary');
gain(s, 'Phase voltages', [322 67 409 105], ...
    'single([1 0; -0.5 0.86602540378; -0.5 -0.86602540378])', 'Matrix(K*u) (u vector)');
common = stage(s, 'Common mode', [459 147 555 208], {'phase_volts'}, {'common_voltage'});
put(common, 'built-in/MinMax', 'Peak', [131 48 185 83], 'Inputs', '1', 'Function', 'max');
put(common, 'built-in/MinMax', 'Valley', [131 114 185 149], 'Inputs', '1', 'Function', 'min');
sum_block(common, 'Peak plus valley', [238 72 269 119], '++');
gain(common, 'Average', [312 80 363 113], 'single(0.5)');
set_param([common '/common_voltage'], 'Position', [409 89 438 103]);
wire(common, {'phase_volts/1','Peak/1'; 'phase_volts/1','Valley/1'; 'Peak/1','Peak plus valley/1'; ...
    'Valley/1','Peak plus valley/2'; 'Peak plus valley/1','Average/1'; 'Average/1','common_voltage/1'});
sum_block(s, 'Remove common mode', [619 69 654 121], '+-');
constant(s, 'Minimum bus', [278 204 362 229], 'single(1)');
put(s, 'built-in/MinMax', 'Safe bus', [423 227 474 276], 'Inputs', '2', 'Function', 'max');
product(s, 'Normalize by bus', [706 77 751 126], '*/');
put(s, 'built-in/Bias', 'Midpoint', [811 84 862 120], 'Bias', 'single(0.5)');
put(s, 'built-in/Saturate', 'PWM duty bounds', [924 83 1008 120], ...
    'LowerLimit', 'single(0.10)', 'UpperLimit', 'single(0.80)');
set_param([s '/duty'], 'Position', [1063 94 1092 108]);
wire(s, {'v_dq/1','Inverse Park rotation/1'; 'cos_sin/1','Inverse Park rotation/2'; ...
    'Inverse Park rotation/1','Phase voltages/1'; 'Phase voltages/1','Common mode/1'; ...
    'Phase voltages/1','Remove common mode/1'; 'Common mode/1','Remove common mode/2'; ...
    'Minimum bus/1','Safe bus/1'; 'bus/1','Safe bus/2'; ...
    'Remove common mode/1','Normalize by bus/1'; 'Safe bus/1','Normalize by bus/2'; ...
    'Normalize by bus/1','Midpoint/1'; 'Midpoint/1','PWM duty bounds/1'; ...
    'PWM duty bounds/1','duty/1'});
end

function rotation(s, inverse, in_name, out_name)
% With c=cos(theta), s=sin(theta), each output is a dot product of
% the input vector and one signed row of the rotation matrix.
if inverse
    first = 'single([1 0; 0 -1])';
    second = 'single([0 1; 1 0])';
else
    first = 'single(eye(2))';
    second = 'single([0 -1; 1 0])';
end
gain(s, 'Rotation row 1', [138 122 231 160], first, 'Matrix(K*u) (u vector)');
gain(s, 'Rotation row 2', [138 211 231 249], second, 'Matrix(K*u) (u vector)');
put(s, 'simulink/Math Operations/Dot Product', 'Component 1', [349 101 414 154]);
put(s, 'simulink/Math Operations/Dot Product', 'Component 2', [349 190 414 243]);
put(s, 'built-in/Mux', 'Rotate vector', [489 122 496 222], 'Inputs', '2', 'DisplayOption', 'bar');
send(s, 'components_in', [in_name '/1'], 139, 54);
read(s, 'components_in', 'Component 1/1', 247);
read(s, 'components_in', 'Component 2/1', 247, 'Input vector');
set_param([s '/' out_name], 'Position', [551 165 580 179]);
wire(s, {'cos_sin/1','Rotation row 1/1'; 'cos_sin/1','Rotation row 2/1'; ...
    'Rotation row 1/1','Component 1/2'; 'Rotation row 2/1','Component 2/2'; ...
    'Component 1/1','Rotate vector/1'; 'Component 2/1','Rotate vector/2'; ...
    'Rotate vector/1',[out_name '/1']});
end

function clamp(s, name, position)
p = stage(s, name, position, {'u','maximum'}, {'limited'});
gain(p, 'Negative limit', [133 128 224 159], 'single(-1)');
put(p, 'simulink/Discontinuities/Saturation Dynamic', 'Clamp u', [266 53 332 150]);
set_param([p '/limited'], 'Position', [395 94 424 108]);
wire(p, {'maximum/1','Clamp u/1'; 'u/1','Clamp u/2'; ...
    'maximum/1','Negative limit/1'; 'Negative limit/1','Clamp u/3'; 'Clamp u/1','limited/1'});
end

function p = stage(s, name, position, in_names, out_names)
p = [s '/' name];
add_block('built-in/SubSystem', p, 'Position', position, 'BackgroundColor', '[0.86, 0.92, 0.97]');
for k = 1:numel(in_names)
    input(p, in_names{k}, k, 23, 53 + 54*(k-1));
end
for k = 1:numel(out_names)
    output(p, out_names{k}, k, 469, 81 + 68*(k-1));
end
set_param(p, 'ContentPreviewEnabled', 'off');
end

function input(s, name, number, x, y, varargin)
put(s, 'built-in/Inport', name, port_rect(x, y), 'Port', num2str(number), varargin{:});
end

function output(s, name, number, x, y, varargin)
put(s, 'built-in/Outport', name, port_rect(x, y), 'Port', num2str(number), varargin{:});
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

function product(s, name, pos, operations)
put(s, 'built-in/Product', name, pos, 'Inputs', operations);
end

function constant(s, name, pos, value)
put(s, 'built-in/Constant', name, pos, 'Value', value, 'SampleTime', 'inf');
end

function select(s, name, pos, width, indices)
put(s, 'simulink/Signal Routing/Selector', name, pos, ...
    'InputPortWidth', width, 'IndexParamArray', {indices});
end

function put(s, library, name, position, varargin)
add_block(library, [s '/' name], 'Position', position, varargin{:});
end

function send(s, tag, source, x, y)
name = ['Store ' strrep(tag, '_', ' ')];
put(s, 'built-in/Goto', name, [x y-9 x+85 y+9], 'GotoTag', tag, ...
    'TagVisibility', 'local', 'ShowName', 'off');
wire(s, {source,[name '/1']});
end

function read(s, tag, destination, x, name)
separator = find(destination == '/', 1, 'last');
block = destination(1:separator-1);
if nargin < 5
    name = ['Read ' strrep(tag, '_', ' ') ' for ' block];
end
y = port_y([s '/' block], 'Inport', str2double(destination(separator+1:end)));
put(s, 'built-in/From', name, [x y-9 x+85 y+9], 'GotoTag', tag, 'ShowName', 'off');
wire(s, {[name '/1'],destination});
end

function align_input(block, index, source, source_index)
offset = port_y(source, 'Outport', source_index) - port_y(block, 'Inport', index);
set_param(block, 'Position', get_param(block, 'Position') + [0 offset 0 offset]);
end

function y = port_y(block, direction, index)
handles = get_param(block, 'PortHandles');
xy = get_param(handles.(direction)(index), 'Position');
y = xy(2);
end

function rect = port_rect(x, y)
rect = [x y-7 x+30 y+7];
end

function wire(s, connections)
for k = 1:size(connections, 1)
    add_line(s, connections{k, 1}, connections{k, 2}, 'autorouting', 'on');
end
end
