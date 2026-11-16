function report = verify_odrive_foc(model)
% Normal-mode simulation of independent current-loop ticks, for ARM replay.
contract = odrive_foc_contract();
count = 96;
time = (0:count-1)' / contract.sample_hz;
k = (1:count)';
phase = 2*pi*k/count;
currents = single([0.17*sin(phase), 0.13*cos(phase)]);
theta = single(phase);
reference = single([0.14*cos(1.5*phase), -0.18*sin(1.2*phase)]);
bus = single(10 + 2*sin(phase));
maximum = single(2*ones(count, 1));
gain = single(repmat([8, 15000, 6, 12000], count, 1));
previous = single([0.09*cos(phase), 0.05*sin(phase), 0.7*sin(3*phase), 0.8*cos(2*phase)]);

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

channels = {currents, theta, reference, bus, maximum, gain, previous};
dataset = Simulink.SimulationData.Dataset;
for i = 1:numel(channels)
    signal = timeseries(channels{i}, time);
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

report = struct('count', count, 'sample_hz', contract.sample_hz, ...
    'duty_min', double(min(duty, [], 'all')), 'duty_max', double(max(duty, [], 'all')), ...
    'max_voltage_norm', max(magnitude), 'time_s', time, ...
    'inputs', double([channels{:}]), 'outputs', double([responses{:}]));
fprintf('Normal-mode simulation passed (%d ticks, duty %.4f to %.4f).\n', ...
    count, report.duty_min, report.duty_max);
end
