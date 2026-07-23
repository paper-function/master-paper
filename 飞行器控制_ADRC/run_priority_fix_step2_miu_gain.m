%% Priority fix step 2: tune bank-angle outer-loop gain K_miu
% Requires FlowAngle_Controller.m support for K_miu_override.

clear; clc;
rootDir = fileparts(mfilename('fullpath'));
cd(rootDir);
addpath(rootDir);

baseModel = 'Ctrl_For_HRV';
harness = 'Ctrl_For_HRV_step1_miu_td_harness';
outDir = fullfile(rootDir, 'terminal_adrc_results', 'priority_fixes', 'step2_miu_gain');
figDir = fullfile(outDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end

bdclose('all');
Parameter;
buildHarness(baseModel, harness);
configureLogging(harness);
xTemplate = getInitialStateTemplate(harness);

rMiuValue = 0.24;
kMiuValues = [4 6 8 10];
cases = defineCases();
rows = [];
results = struct([]);

for i = 1:numel(kMiuValues)
    for j = 1:numel(cases)
        fprintf('K_miu=%0.3f case=%s\n', kMiuValues(i), cases(j).name);
        r = runCase(harness, cases(j), xTemplate, rMiuValue, kMiuValues(i));
        results(end+1).r = r; %#ok<SAGROW>
        if r.ok
            m = r.metrics;
            rows = [rows; table(kMiuValues(i), string(cases(j).name), true, m.stable, ...
                m.downpress, m.dH, m.avgHdot, m.minThetaDeg, m.maxAlphaErrDeg, ...
                m.maxMiuErrDeg, m.tailMiuRmsDeg, m.maxDeltaDeg, m.satFrac, ...
                'VariableNames', {'K_miu','case','ok','stable','downpress','dH','avgHdot', ...
                'minThetaDeg','maxAlphaErrDeg','maxMiuErrDeg','tailMiuRmsDeg','maxDeltaDeg','satFrac'})]; %#ok<AGROW>
        else
            rows = [rows; table(kMiuValues(i), string(cases(j).name), false, false, ...
                false, NaN, NaN, NaN, NaN, NaN, NaN, NaN, NaN, ...
                'VariableNames', {'K_miu','case','ok','stable','downpress','dH','avgHdot', ...
                'minThetaDeg','maxAlphaErrDeg','maxMiuErrDeg','tailMiuRmsDeg','maxDeltaDeg','satFrac'})]; %#ok<AGROW>
        end
    end
end

writetable(rows, fullfile(outDir, 'step2_miu_gain_metrics.csv'));
save(fullfile(outDir, 'step2_miu_gain_results.mat'), 'rows', 'results', 'cases', 'rMiuValue', 'kMiuValues');
plotSummary(rows, figDir);
writeReport(fullfile(outDir, 'step2_miu_gain_report.md'), rows);
fprintf('Done: %s\n', outDir);

function buildHarness(baseModel, harness)
load_system(baseModel);
if exist([harness '.slx'], 'file'), delete([harness '.slx']); end
save_system(baseModel, harness);
close_system(baseModel, 0);
load_system(harness);
replaceConstantWithFromWorkspace(harness, 'Constant', 'alpha_cmd_deg');
replaceConstantWithFromWorkspace(harness, 'Constant1', 'beta_cmd_deg');
replaceConstantWithFromWorkspace(harness, 'Constant2', 'miu_cmd_deg');
set_param(harness, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'SaveTime', 'on', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveState', 'on', 'StateSaveName', 'xout', 'SaveFormat', 'StructureWithTime');
save_system(harness);
end

function replaceConstantWithFromWorkspace(model, blockName, variableName)
oldPath = [model '/' blockName];
pos = get_param(oldPath, 'Position');
ph = get_param(oldPath, 'PortHandles');
line = get_param(ph.Outport, 'Line');
dst = get_param(line, 'DstPortHandle');
delete_line(line);
delete_block(oldPath);
add_block('simulink/Sources/From Workspace', oldPath, 'Position', pos, ...
    'VariableName', variableName, 'SampleTime', '0', 'Interpolate', 'on');
newPh = get_param(oldPath, 'PortHandles');
add_line(model, newPh.Outport, dst, 'autorouting', 'on');
end

function configureLogging(model)
blocks = {'HRV_Model','FlowAngle_Controller','AngleRate_Controller','actuator','actuator1','actuator2'};
names = {{'lon','lat','X','Y','H','V','psi','theta','alpha','beta','miu','p','q','r'}, ...
    {'p_d','q_d','r_d'}, {'delta_a_d','delta_e_d','delta_r_d'}, {'delta_a'}, {'delta_e'}, {'delta_r'}};
for b = 1:numel(blocks)
    ph = get_param([model '/' blocks{b}], 'PortHandles');
    for k = 1:min(numel(ph.Outport), numel(names{b}))
        set_param(ph.Outport(k), 'DataLogging', 'on', ...
            'DataLoggingNameMode', 'Custom', 'DataLoggingName', names{b}{k});
    end
end
end

function xTemplate = getInitialStateTemplate(model)
simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('alpha_cmd_deg', timeseries(8, 0));
simIn = simIn.setVariable('beta_cmd_deg', timeseries(0, 0));
simIn = simIn.setVariable('miu_cmd_deg', timeseries(0, 0));
simIn = simIn.setModelParameter('StopTime', '0', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveState', 'on', 'StateSaveName', 'xInit', 'SaveFormat', 'StructureWithTime');
simOut = sim(simIn);
xTemplate = simOut.xInit;
end

function cases = defineCases()
idx = 0;
idx = idx + 1; cases(idx) = makeCase('bank90_alpha8', 45000, 4200, [0 10 28 45], [8 8 8 8], [0 0 0 0], [0 -90 -90 0]);
idx = idx + 1; cases(idx) = makeCase('bank120_alpha8', 45000, 4200, [0 12 30 50], [8 8 8 8], [0 0 0 0], [0 -120 -120 0]);
end

function c = makeCase(name, H0, V0, t, alphaDeg, betaDeg, miuDeg)
c.name = name;
c.H0 = H0;
c.V0 = V0;
c.t = t(:);
c.alphaDeg = alphaDeg(:);
c.betaDeg = betaDeg(:);
c.miuDeg = miuDeg(:);
c.stopTime = max(t);
end

function r = runCase(model, c, xTemplate, rMiu, kMiu)
r = struct('ok', false, 'caseDef', c, 'r_miu', rMiu, 'K_miu', kMiu, 'error', '', 'metrics', [], 'data', []);
xInit = xTemplate;
hrvIdx = find(contains({xInit.signals.blockName}, '/HRV_Model/S-Function'), 1);
xInit.signals(hrvIdx).values(1, 5) = c.H0;
xInit.signals(hrvIdx).values(1, 6) = c.V0;

simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('alpha_cmd_deg', timeseries(c.alphaDeg, c.t));
simIn = simIn.setVariable('beta_cmd_deg', timeseries(c.betaDeg, c.t));
simIn = simIn.setVariable('miu_cmd_deg', timeseries(c.miuDeg, c.t));
simIn = simIn.setVariable('xInit', xInit);
simIn = simIn.setVariable('r_miu', rMiu);
simIn = simIn.setVariable('K_miu_override', kMiu);
simIn = simIn.setModelParameter('StopTime', num2str(c.stopTime), ...
    'LoadInitialState', 'on', 'InitialState', 'xInit', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveState', 'on', ...
    'StateSaveName', 'xout', 'SaveFormat', 'StructureWithTime');
try
    simOut = sim(simIn);
    r = computeMetrics(r, simOut);
catch ME
    r.error = ME.message;
end
end

function r = computeMetrics(r, simOut)
logs = simOut.logsout;
get = @(name) logs.get(name).Values;
t = get('H').Time;
H = get('H').Data;
V = get('V').Data;
theta = get('theta').Data * 180/pi;
alpha = get('alpha').Data * 180/pi;
beta = get('beta').Data * 180/pi;
miu = get('miu').Data * 180/pi;
deltaA = get('delta_a').Data;
deltaE = get('delta_e').Data;
deltaR = get('delta_r').Data;
c = r.caseDef;
cmdAlpha = interp1(c.t, c.alphaDeg, t, 'linear', 'extrap');
cmdMiu = interp1(c.t, c.miuDeg, t, 'linear', 'extrap');
eMiu = miu - cmdMiu;
tail = t > 0.75 * t(end);
r.ok = true;
r.data = struct('t', t, 'H', H, 'V', V, 'theta', theta, 'alpha', alpha, 'beta', beta, ...
    'miu', miu, 'cmdAlpha', cmdAlpha, 'cmdMiu', cmdMiu, ...
    'deltaA', deltaA, 'deltaE', deltaE, 'deltaR', deltaR);
r.metrics = struct('stable', all(isfinite([H; V; theta; alpha; beta; miu])) && min(H) > 20000 && min(V) > 1500, ...
    'downpress', min(theta) < -1 || (H(end)-H(1))/t(end) < -20, ...
    'dH', H(end)-H(1), 'avgHdot', (H(end)-H(1))/t(end), 'minThetaDeg', min(theta), ...
    'maxAlphaErrDeg', max(abs(alpha - cmdAlpha)), 'maxMiuErrDeg', max(abs(eMiu)), ...
    'tailMiuRmsDeg', sqrt(mean(eMiu(tail).^2)), 'maxDeltaDeg', max(abs([deltaA(:); deltaE(:); deltaR(:)])), ...
    'satFrac', mean(abs([deltaA(:); deltaE(:); deltaR(:)]) > 29.4));
end

function plotSummary(rows, figDir)
fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1000 700]);
tiledlayout(2, 2);
cases = unique(rows.case, 'stable');
for i = 1:numel(cases)
    sub = rows(rows.case == cases(i), :);
    nexttile(1); hold on; plot(sub.K_miu, sub.maxMiuErrDeg, '-o', 'DisplayName', cases(i));
    nexttile(2); hold on; plot(sub.K_miu, sub.tailMiuRmsDeg, '-o', 'DisplayName', cases(i));
    nexttile(3); hold on; plot(sub.K_miu, sub.maxDeltaDeg, '-o', 'DisplayName', cases(i));
    nexttile(4); hold on; plot(sub.K_miu, sub.avgHdot, '-o', 'DisplayName', cases(i));
end
nexttile(1); grid on; ylabel('max miu error deg'); xlabel('K_miu'); legend('Location','best');
nexttile(2); grid on; ylabel('tail miu RMS deg'); xlabel('K_miu');
nexttile(3); grid on; ylabel('max surface deg'); xlabel('K_miu');
nexttile(4); grid on; ylabel('avg Hdot m/s'); xlabel('K_miu');
exportgraphics(fig, fullfile(figDir, 'step2_miu_gain_summary.png'));
close(fig);
end

function writeReport(file, rows)
fid = fopen(file, 'w'); c = onCleanup(@() fclose(fid));
fprintf(fid, '# Step 2: bank outer-loop K_miu tuning\n\n');
fprintf(fid, '| K_miu | case | ok | stable | downpress | max miu err | tail miu RMS | max surface | avg Hdot |\n');
fprintf(fid, '|---:|---|---:|---:|---:|---:|---:|---:|---:|\n');
for k = 1:height(rows)
    fprintf(fid, '| %.3f | %s | %d | %d | %d | %.2f | %.2f | %.2f | %.1f |\n', ...
        rows.K_miu(k), rows.case(k), rows.ok(k), rows.stable(k), rows.downpress(k), ...
        rows.maxMiuErrDeg(k), rows.tailMiuRmsDeg(k), rows.maxDeltaDeg(k), rows.avgHdot(k));
end
end
