function report = verify_odrive_foc(model)
% Normal-mode simulation of independent three-loop ticks for ARM replay.
contract = odrive_foc_contract();
count = 192;
time = (0:count-1)' / contract.sample_hz;
k = (1:count)';
phase = 2*pi*k/count;
currents = single([0.17*sin(phase), 0.13*cos(phase)]);
theta = single(phase);
reference = single([0.14*cos(1.5*phase), -0.18*sin(1.2*phase)]);
bus = single(10 + 2*sin(phase));
maximum = single(2*ones(count, 1));
gain = single(repmat([8, 15000, 6, 12000], count, 1));
previous = single([0.09*cos(phase),0.05*sin(phase),0.7*sin(3*phase),0.8*cos(2*phase), ...
    0.08*sin(phase),0.6*cos(phase),0.10*sin(2*phase),mod(k-1,20)]);
mechanical = single([0.5*sin(phase),0.8*cos(phase)]);
command = zeros(count,2,'single');
command(97:128,:) = single([ones(32,1),0.003*sin(phase(97:128))]);
command(129:160,:) = single([2*ones(32,1),1.3*sin(phase(129:160))]);
command(161:192,:) = single([3*ones(32,1),0.9*sin(phase(161:192))]);
motion_gains = single(repmat([0.015 0.005 2],count,1));
motion_limits = single(repmat([0.2 3 3 0.5 0.0315],count,1));

% Known neutral, positive Iq, two-axis saturation, B/C sign and bus scaling.
currents(1:3, :) = 0;
theta(1:3) = 0;
reference(1:3, :) = single([0 0; 0 0.05; 0.2 -0.2]);
gain(3, :) = single([20 15000 25 12000]);
previous(1:5, :) = 0;
currents(4, :) = single([0.2 -0.1]);
theta(4) = single(pi/2);
currents(5, :) = 0;
theta(5) = 0;
reference(5, :) = single([0.2 0.2]);
gain(5, :) = gain(3, :);
bus(5) = single(4);  % Model boundary test. Board undervoltage protection blocks this input.

% Check signed torque conversion, held references, speed PI, and the position cascade.
previous([97 129 160 161],:) = 0;
previous([97 129 160 161],8) = 19;
command(97,:) = single([1, -0.00315]);
previous(98,5:8) = single([0 0 0.03 18]);
mechanical([129 160 161],:) = single([0 0; 0 -50; 1 0]);
command(129,:) = single([2 1]);
command(160,:) = single([2 1]);
command(161,:) = single([3 0.5]);

channels = {currents,theta,reference,bus,maximum,gain,mechanical,command, ...
    motion_gains,motion_limits,previous};
% Give each tick a hold interval on both sides. At the final tick a rounding
% difference between solver time and the last data knot must not zero the input.
dt = 1/contract.sample_hz;
knots = [0; time(2:end)-dt/4; time(end)+dt];
dataset = Simulink.SimulationData.Dataset;
for i = 1:numel(channels)
    value = channels{i};
    signal = setinterpmethod(timeseries([value;value(end,:)],knots),'zoh');
    dataset = dataset.addElement(signal, contract.inputs{i, 1});
end
simulation = Simulink.SimulationInput(model);
simulation = simulation.setExternalInput(dataset);
simulation = simulation.setModelParameter('SimulationMode', 'normal', ...
    'StopTime', sprintf('%.16g', time(end)), 'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', 'SaveFormat', 'Dataset', 'SaveTime', 'off', ...
    'ReturnWorkspaceOutputs', 'on');
out = sim(simulation);
dataset = out.get('yout');
responses = cell(1, size(contract.outputs, 1));
for i = 1:numel(responses)
    element = dataset.getElement(i);
    value = element.Values;
    width = contract.outputs{i, 2};
    assert(numel(value.Time) == count && isa(value.Data, 'single'), ...
        'Output %s sample count or type changed.', contract.outputs{i, 1});
    if value.IsTimeFirst
        responses{i} = reshape(value.Data, count, width);
    else
        responses{i} = reshape(value.Data, width, count).';
    end
end

state = responses{1};
voltage = responses{2};
duty = responses{3};
magnitude = sqrt(sum(double(voltage).^2, 2));
assert(all(isfinite([state, voltage, duty]), 'all'), 'Nonfinite model output.');
assert(all(duty >= single(0.10) & duty <= single(0.80), 'all'), 'PWM window violation.');
assert(all(magnitude <= min(double(maximum), 0.30*double(bus)) + 2e-6), ...
    'Voltage vector limit violation.');
assert(all(state(1, :) == 0) && all(voltage(1, :) == 0) && all(duty(1, :) == single(0.5)), ...
    'Zero commands and zero state must give midpoint duty.');
assert(abs(voltage(2, 2) - (6 + 12000/contract.sample_hz)*0.05) < 2e-6 ...
    && duty(2, 2) > single(0.5) && duty(2, 3) < single(0.5), 'Positive Iq sign is wrong.');
assert(abs(magnitude(3) - 2) < 2e-6 && abs(magnitude(5) - 1.2) < 2e-6, ...
    'Expected both the phase-voltage and bus-voltage limits to activate.');
assert(all(abs(state(4, 1:2) - [0.15*0.3/sqrt(3), 0.15*0.1]) < 2e-6), ...
    'B/C measurement phase mapping is wrong.');
assert(abs(state(97,7)+0.0005)<2e-7 && state(97,8)==0, ...
    'Torque mode must convert Nm to signed Iq and use the Iq slew rate.');
assert(state(98,7)==previous(98,7) && state(98,8)==19, ...
    'Outer references must hold between 1 kHz ticks.');
assert(abs(state(129,6)-0.003)<2e-7 && state(129,7)>0 && state(129,8)==0, ...
    'Speed mode must ramp its speed target and update the PI.');
assert(abs(state(160,7)-0.2)<2e-7 && state(160,5)==0, ...
    'Saturated speed PI must hold the integrator when error drives deeper into saturation.');
assert(state(161,6)<0 && state(161,7)<0, ...
    'Position mode must command speed from position error.');
assert(all(state(1:96,5:8)==0,'all'), 'Alignment must clear motion-loop state.');

report = struct('count', count, 'sample_hz', contract.sample_hz, ...
    'abi_revision',contract.revision,'motion_hz',contract.motion_hz, ...
    'duty_min', double(min(duty, [], 'all')), 'duty_max', double(max(duty, [], 'all')), ...
    'max_voltage_norm', max(magnitude), 'time_s', time, ...
    'inputs', double([channels{:}]), 'outputs', double([responses{:}]));
fprintf('Normal-mode simulation passed (%d ticks, duty %.4f to %.4f).\n', ...
    count, report.duty_min, report.duty_max);
end
