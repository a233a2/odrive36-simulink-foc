function build_odrive_foc(model_file, output_folder)
% Extract the controller, replay its interface vectors, and generate target C.
assert(strcmp(version('-release'), '2024b'), 'This build expects MATLAB R2024b.');
contract = odrive_foc_contract();
source = contract.model;
model = contract.codegen_model;
[~, filename, extension] = fileparts(model_file);
assert(strcmp(filename, source) && strcmpi(extension, '.slx'), ...
    'Use the saved model %s.slx.', source);
assert(isfile(model_file) && ~bdIsLoaded(source) && ~bdIsLoaded(model), ...
    'The saved simulation and export model must be closed.');
if ~isfolder(output_folder)
    mkdir(output_folder);
end

environment = probe_environment();
write_json(fullfile(output_folder, 'matlab_environment.json'), environment);
for k = 1:numel(environment.products)
    item = environment.products(k);
    if item.required
        assert(item.installed && item.license_available, ...
            'Required product unavailable: %s (feature %s).', item.name, item.feature);
    end
end

old = Simulink.fileGenControl('getConfig');
restore = onCleanup(@() Simulink.fileGenControl('setConfig', 'config', old));
cache = fullfile(output_folder, 'cache');
code = fullfile(output_folder, 'code');
Simulink.fileGenControl('set', 'CacheFolder', cache, 'CodeGenFolder', code, 'createDir', true);
load_system(model_file);
close_source = onCleanup(@() close_system(source, 0));
subsystem = [source '/' contract.subsystem];
assert(getSimulinkBlockHandle(subsystem) ~= -1 && strcmp(get_param(subsystem, 'BlockType'), 'SubSystem'), ...
    'Missing controller subsystem %s.', subsystem);
new_system(model);
close_model = onCleanup(@() close_system(model, 0));
Simulink.SubSystem.copyContentsToBlockDiagram(subsystem, model);
set_controller_target(model, contract);
save_system(model, fullfile(output_folder, [model '.slx']));
fprintf('Exported %s to %s.slx.\n', subsystem, model);
check_model(model, contract);
replay = verify_odrive_foc(model);
write_json(fullfile(output_folder, 'simulation.json'), replay);

slbuild(model);
location = RTW.getBuildDir(model);
archive = fullfile(output_folder, [source '_c.zip']);
packNGo(fullfile(location.BuildDirectory, 'buildInfo.mat'), ...
    'packType', 'flat', 'fileName', archive, 'minimalHeaders', true);
generated = fullfile(output_folder, 'generated');
unzip(archive, generated);
public_header = fileread(fullfile(generated, [model '.h']));
assert(~contains(public_header, ['DW_' model '_T']) && ~contains(public_header, ['RT_MODEL_' model '_T']), ...
    'Use one rate and keep model state in the previous_state and next_state ports.');
header = fopen(fullfile(generated, 'odrive_foc_contract.h'), 'wt');
assert(header >= 0, 'Cannot write the generated interface metadata.');
close_header = onCleanup(@() fclose(header));
fprintf(header, ['#ifndef ODRIVE_FOC_CONTRACT_H\n#define ODRIVE_FOC_CONTRACT_H\n' ...
    '#define ODRIVE_FOC_ABI_REVISION %du\n#define ODRIVE_FOC_SAMPLE_HZ %du\n' ...
    '#define ODRIVE_FOC_MOTION_HZ %du\n#endif\n'], ...
    contract.revision, contract.sample_hz, contract.motion_hz);
fprintf('C export: %s\n', generated);
end

function set_controller_target(model, contract)
set_param(model, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', sprintf('%.16g', 1 / contract.sample_hz), 'StartTime', '0', ...
    'StopTime', '0.01', 'EnableMultiTasking', 'off', 'SystemTargetFile', 'ert.tlc', ...
    'TargetLang', 'C', 'GenCodeOnly', 'on', 'GenerateSampleERTMain', 'off', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'CodeInterfacePackaging', 'Nonreusable function', 'SupportNonFinite', 'off', ...
    'SuppressErrorStatus', 'on', 'MatFileLogging', 'off', 'DefaultParameterBehavior', 'Inlined');
end

function check_model(model, contract)
period = 1 / contract.sample_hz;
check_ports(model, 'Inport', contract.inputs, period);
check_ports(model, 'Outport', contract.outputs, period);
end

function check_ports(model, type, spec, period)
ports = find_system(model, 'SearchDepth', 1, 'BlockType', type);
assert(numel(ports) == size(spec, 1), 'Unexpected number of root %s blocks.', type);
for k = 1:size(spec, 1)
    name = spec{k, 1};
    path = [model '/' name];
    assert(ismember(path, ports) && str2double(get_param(path, 'Port')) == k ...
        && str2double(get_param(path, 'PortDimensions')) == spec{k, 2} ...
        && strcmp(get_param(path, 'OutDataTypeStr'), 'single'), ...
        'Root %s port %d must be %s, single, width %d.', type, k, name, spec{k, 2});
    if strcmp(type, 'Inport')
        assert(str2double(get_param(path, 'SampleTime')) == period, ...
            'Sample time changed at %s.', path);
    end
end
end

function report = probe_environment()
products = {
    'MATLAB', 'MATLAB', true;
    'Simulink', 'Simulink', true;
    'MATLAB Coder', 'MATLAB_Coder', true;
    'Simulink Coder', 'Real-Time_Workshop', true;
    'Embedded Coder', 'RTW_Embedded_Coder', true;
    'Motor Control Blockset', 'Motor_Control_Blockset', false;
    'Fixed-Point Designer', 'Fixed_Point_Toolbox', false;
};
installed = ver;
report.release = version('-release');
report.version = version;
report.matlabroot = matlabroot;
addons = matlab.addons.installedAddons();
report.addons = table2struct(addons(:, {'Name', 'Version', 'Enabled'}));
host_compilers = mex.getCompilerConfigurations('C', 'Selected');
report.host_c_compilers = {host_compilers.Name};
report.fi_available = ~isempty(which('fi'));
for k = 1:size(products, 1)
    name = products{k, 1};
    feature = products{k, 2};
    index = find(strcmp({installed.Name}, name), 1);
    item = struct('name', name, 'feature', feature, 'required', products{k, 3}, ...
        'installed', ~isempty(index), 'version', '', 'license_available', logical(license('test', feature)));
    if ~isempty(index)
        item.version = installed(index).Version;
    end
    report.products(k) = item;
    fprintf('%s: installed=%d, license_test=%d\n', name, item.installed, item.license_available);
end
end

function write_json(path, value)
file = fopen(path, 'wt', 'n', 'UTF-8');
assert(file >= 0, 'Cannot write %s.', path);
close_file = onCleanup(@() fclose(file));
fprintf(file, '%s\n', jsonencode(value, 'PrettyPrint', true));
end
