function report = verify_odrive_motor_demo(model_file, output_folder)
% Exercise each selectable mode through the PMSM and a common load step.
if nargin < 1 || isempty(model_file)
    model_file=fullfile(fileparts(mfilename('fullpath')),'odrive_foc.slx');
end
[~,model]=fileparts(model_file);
if ~bdIsLoaded(model)
    load_system(model_file);
    close_model=onCleanup(@() close_system(model,0));
end
names={'torque','speed','position'};
targets=[0.00378 2.0 0.55];
keys={'torque_nm','speed_rad_s','position_rad'};
tolerances=[0.00008 0.08 0.035];
traces=cell(1,numel(names));
for mode=1:numel(names)
    in=Simulink.SimulationInput(model);
    in=in.setVariable('demo_mode',mode,'Workspace',model);
    in=in.setVariable('demo_target',targets(mode),'Workspace',model);
    in=in.setModelParameter('SimulationMode','normal','ReturnWorkspaceOutputs','on');
    out=sim(in);
    [t,y]=signal(out.logsout,keys{mode},1);
    [load_time,load]=signal(out.logsout,'load_torque_nm',1);
    [~,duty]=signal(out.logsout,'duty_abc',3);
    [~,dq]=signal(out.logsout,'idq_motor_a',2);
    assert(all(isfinite([y;duty(:);dq(:)])),'Nonfinite response in %s mode.',names{mode});
    assert(all(duty>=single(0.10) & duty<=single(0.80),'all'),'PWM window violation in %s mode.',names{mode});
    load_step=load_time(find(load~=load(1),1));
    assert(~isempty(load_step),'Missing load step.');
    last=t>=t(end)-0.25;
    final=mean(double(y(last)));
    assert(abs(final-targets(mode))<=tolerances(mode), ...
        '%s final %.6f differs from target %.6f.',names{mode},final,targets(mode));
    report.(names{mode})=struct('target',targets(mode),'final',final,'error',final-targets(mode), ...
        'load_step_s',load_step,'load_nm',double(load(end)), ...
        'max_iq_a',double(max(abs(dq(:,2)))),'duty_min',double(min(duty,[],'all')), ...
        'duty_max',double(max(duty,[],'all')));
    traces{mode}=struct('time',t,'value',y,'load_step',load_step);
    fprintf('%s target %.6f, final %.6f, maximum Iq %.4f A\n', ...
        names{mode},targets(mode),final,report.(names{mode}).max_iq_a);
end

if nargin>1 && ~isempty(output_folder)
    if ~isfolder(output_folder), mkdir(output_folder); end
    save_plot(output_folder,names,targets,traces);
    file=fopen(fullfile(output_folder,'motion_modes.json'),'wt','n','UTF-8');
    assert(file>=0,'Could not write simulation report.');
    close_file=onCleanup(@() fclose(file));
    fprintf(file,'%s\n',jsonencode(report,'PrettyPrint',true));
end
end

function [t,data]=signal(dataset,name,width)
value=dataset.getElement(name).Values;
t=value.Time;
if value.IsTimeFirst
    data=reshape(value.Data,numel(t),width);
else
    data=reshape(value.Data,width,numel(t)).';
end
end

function save_plot(folder,names,targets,traces)
fig=figure('Visible','off','Color','white','Position',[100 100 1220 910]);
close_figure=onCleanup(@() close(fig));
layout=tiledlayout(fig,3,1,'TileSpacing','compact','Padding','compact');
title(layout,'ODrive FOC closed-loop simulation | three run modes');
units={'Torque (mN m)','Speed (rad/s)','Position (rad)'};
scales=[1000 1 1];
for i=1:3
    ax=nexttile(layout);
    sample=traces{i};
    idx=1:10:numel(sample.time);
    plot(ax,sample.time(idx),scales(i)*double(sample.value(idx)),'Color',[0.035 0.41 0.38],'LineWidth',1.6);
    hold(ax,'on');
    plot(ax,[0 0.04 0.04 sample.time(end)],[0 0 targets(i) targets(i)]*scales(i), ...
        '--','Color',[0.23 0.36 0.53],'LineWidth',1);
    xline(ax,sample.load_step,':','Load step','Color',[0.55 0.35 0.11],'HandleVisibility','off');
    ylabel(ax,units{i});
    title(ax,[upper(names{i}(1)) names{i}(2:end) ' mode']);
    grid(ax,'on'); xlim(ax,[0 sample.time(end)]);
    if i==1, legend(ax,{'Simulated response','Command'},'Location','southeast'); end
end
xlabel(ax,'Time (s)');
exportgraphics(fig,fullfile(folder,'motion_modes.png'),'Resolution',150);
end
