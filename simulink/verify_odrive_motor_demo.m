function result = verify_odrive_motor_demo(model_file, output_folder)
% Run the supplied current/torque-step scenario and save its measured traces.
if nargin < 1 || isempty(model_file)
    model_file = fullfile(fileparts(mfilename('fullpath')), 'odrive_foc.slx');
end
[~, model] = fileparts(model_file);
was_loaded = bdIsLoaded(model);
if ~was_loaded
    load_system(model_file);
    close_model = onCleanup(@() close_system(model, 0));
end
simulation = Simulink.SimulationInput(model);
simulation = simulation.setModelParameter('SimulationMode', 'normal', 'ReturnWorkspaceOutputs', 'on');
out = sim(simulation);
data = out.logsout;
[t, measured] = signal(data, 'idq_motor_a', 2);
[t_ref, reference] = signal(data, 'idq_ref_a', 2);
[t_speed, speed] = signal(data, 'speed_rad_s', 1);
[t_torque, torque] = signal(data, 'torque_nm', 1);
[t_load, load] = signal(data, 'load_torque_nm', 1);
[~, duty] = signal(data, 'duty_abc', 3);
assert(isequal(t, t_ref, t_speed, t_torque, t_load), 'Expected a common sample grid in the supplied demo.');
assert(all(isfinite([measured, reference, speed, torque, load, duty]), 'all'), ...
    'The motor demo produced a nonfinite output.');
assert(all(duty >= single(0.10) & duty <= single(0.80), 'all'), ...
    'The motor demo exceeded the PWM duty envelope.');

command_index = find(reference(:,2) ~= reference(1,2), 1);
load_index = find(load ~= load(1), 1);
assert(~isempty(command_index) && ~isempty(load_index) && load_index > command_index, ...
    'The supplied scenario uses a current step followed by an opposing load step.');
command_time = t(command_index);
load_time = t(load_index);
before_load = t >= load_time-0.01 & t < load_time;
at_end = t >= t(end)-0.02;
tracking_error = abs(double(measured(:,2) - reference(:,2)));
band = 0.02 * abs(double(reference(command_index,2)));
outside_band = find((t >= command_time & t < load_time) & tracking_error > band, 1, 'last');
settled = command_index;
if ~isempty(outside_band)
    settled = outside_band+1;
end
result = struct( ...
    'sample_hz', 1 / (t(2)-t(1)), 'stop_s', t(end), 'current_step_s', command_time, ...
    'iq_reference_a', double(reference(command_index,2)), ...
    'load_step_s', load_time, 'load_nm', double(load(load_index)), ...
    'iq_settling_2pct_ms', 1000*(t(settled)-command_time), ...
    'iq_before_load_a', mean(double(measured(before_load,2))), ...
    'iq_at_end_a', mean(double(measured(at_end,2))), ...
    'peak_iq_error_after_load_a', max(tracking_error(t >= load_time)), ...
    'max_abs_id_a', max(abs(double(measured(:,1)))), ...
    'speed_before_load_rad_s', mean(double(speed(before_load))), ...
    'speed_at_end_rad_s', mean(double(speed(at_end))), ...
    'duty_min', double(min(duty, [], 'all')), 'duty_max', double(max(duty, [], 'all')));
assert(result.iq_settling_2pct_ms < 20 && abs(result.iq_at_end_a-result.iq_reference_a) < 0.003, ...
    'The default current step did not track within its test limits.');
assert(result.max_abs_id_a < 0.005 && result.peak_iq_error_after_load_a < 0.005, ...
    'Unexpected d-axis current or Iq deviation during the default load step.');
assert(result.speed_before_load_rad_s > 0 && ...
    result.speed_at_end_rad_s < result.speed_before_load_rad_s-0.5, ...
    'The opposing torque did not reduce the speed in the default current-control scenario.');
disp(result);

if nargin >= 2
    if ~isfolder(output_folder)
        mkdir(output_folder);
    end
    fid = fopen(fullfile(output_folder, 'motor_demo.json'), 'w');
    assert(fid >= 0, 'Cannot write the motor demo result.');
    close_file = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', jsonencode(result, 'PrettyPrint', true));
    plot_demo(output_folder, t, measured, reference, speed, torque, load, load_time);
end
end

function [time, values] = signal(data, name, width)
entry = data.getElement(name);
series = entry.Values;
time = series.Time(:);
if series.IsTimeFirst
    values = reshape(series.Data, numel(time), width);
else
    values = reshape(series.Data, width, numel(time)).';
end
end

function plot_demo(folder, time, measured, reference, speed, torque, load, load_time)
f = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 860 610]);
close_figure = onCleanup(@() close(f));
layout = tiledlayout(f, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'ODrive FOC current step and mechanical load');

a = nexttile(layout);
plot(a, time, measured(:,2), 'LineWidth', 1.3);
hold(a, 'on');
stairs(a, time, reference(:,2), '--', 'LineWidth', 1.1);
plot(a, time, measured(:,1), 'LineWidth', 1.0);
ylabel(a, 'Current (A)');
legend(a, 'Motor I_q', 'Reference I_q', 'Motor I_d', 'Location', 'southeast');

b = nexttile(layout);
plot(b, time, speed, 'LineWidth', 1.3);
ylabel(b, 'Speed (rad/s)');

c = nexttile(layout);
plot(c, time, torque*1000, 'LineWidth', 1.3);
hold(c, 'on');
stairs(c, time, load*1000, '--', 'LineWidth', 1.1);
ylabel(c, 'Torque (mN m)');
xlabel(c, 'Time (s)');
legend(c, 'Electromagnetic torque', 'Opposing load', 'Location', 'southeast');
for axes_handle = [a b c]
    hold(axes_handle, 'on');
    xline(axes_handle, load_time, ':k', 'HandleVisibility', 'off');
    grid(axes_handle, 'on');
    xlim(axes_handle, [time(1) time(end)]);
end
exportgraphics(f, fullfile(folder, 'motor_demo.png'), 'Resolution', 150);
end
