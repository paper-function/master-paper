%% Alpha + bank-reversal down-press feasibility test
% Uses the same temporary harness approach as run_terminal_adrc_campaign.m.
% The original ADRC controller, ESO, actuator and plant structure are not changed.

clear; clc;
rootDir = fileparts(mfilename('fullpath'));
cd(rootDir);
addpath(rootDir);

baseModel = 'Ctrl_For_HRV';
harness = 'Ctrl_For_HRV_alpha_reversal_harness';
outDir = fullfile(rootDir, 'terminal_adrc_results', 'alpha_reversal_downpress');
figDir = fullfile(outDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end

bdclose('all');
Parameter;
buildHarness(baseModel, harness);
xTemplate = getInitialStateTemplate(harness);
configureLogging(harness);

cases = defineCases();
results = repmat(emptyResult(), numel(cases), 1);
for k = 1:numel(cases)
    fprintf('%2d/%2d %s\n', k, numel(cases), cases(k).name);
    results(k) = runCase(harness, cases(k), xTemplate);
end

save(fullfile(outDir, 'alpha_reversal_results.mat'), 'cases', 'results');
writeCsv(fullfile(outDir, 'alpha_reversal_metrics.csv'), results);
makePlots(figDir, results);
writeReport(fullfile(outDir, 'alpha_reversal_report.md'), results);
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
    'SaveTime', 'on', 'SaveState', 'on', 'StateSaveName', 'xout', ...
    'SaveFormat', 'StructureWithTime', 'ReturnWorkspaceOutputs', 'on');
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
    for i = 1:min(numel(ph.Outport), numel(names{b}))
        set_param(ph.Outport(i), 'DataLogging', 'on', ...
            'DataLoggingNameMode', 'Custom', 'DataLoggingName', names{b}{i});
    end
end
end

function xTemplate = getInitialStateTemplate(model)
simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('alpha_cmd_deg', timeseries(9, 0));
simIn = simIn.setVariable('beta_cmd_deg', timeseries(0, 0));
simIn = simIn.setVariable('miu_cmd_deg', timeseries(-15, 0));
simIn = simIn.setModelParameter('StopTime', '0', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveState', 'on', 'StateSaveName', 'xInit', 'SaveFormat', 'StructureWithTime');
simOut = sim(simIn);
xTemplate = simOut.xInit;
end

function cases = defineCases()
idx = 0;
idx = idx + 1; cases(idx) = makeCase('bank60_alpha8', 45000, 4200, [0 8 24 40], [8 8 8 8], [0 0 0 0], [0 -60 -60 0]);
idx = idx + 1; cases(idx) = makeCase('bank90_alpha8', 45000, 4200, [0 10 28 45], [8 8 8 8], [0 0 0 0], [0 -90 -90 0]);
idx = idx + 1; cases(idx) = makeCase('bank120_alpha8', 45000, 4200, [0 12 30 50], [8 8 8 8], [0 0 0 0], [0 -120 -120 0]);
idx = idx + 1; cases(idx) = makeCase('near_inverted_alpha8', 45000, 4200, [0 15 35 55], [8 8 8 8], [0 0 0 0], [0 -165 -165 0]);
idx = idx + 1; cases(idx) = makeCase('bank90_alpha12', 45000, 4200, [0 10 28 45], [10 12 12 9], [0 0 0 0], [0 -90 -90 0]);
idx = idx + 1; cases(idx) = makeCase('bank90_lowerQ', 55690, 3475, [0 10 28 45], [8 8 8 8], [0 0 0 0], [0 -90 -90 0]);
idx = idx + 1; cases(idx) = makeCase('bank60_lowerQ', 55690, 3475, [0 10 28 45], [8 8 8 8], [0 0 0 0], [0 -60 -60 0]);
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

function result = runCase(model, c, xTemplate)
result = emptyResult();
result.caseDef = c;
xInit = xTemplate;
hrvIdx = find(contains({xInit.signals.blockName}, '/HRV_Model/S-Function'), 1);
xInit.signals(hrvIdx).values(1, 5) = c.H0;
xInit.signals(hrvIdx).values(1, 6) = c.V0;

simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('alpha_cmd_deg', timeseries(c.alphaDeg, c.t));
simIn = simIn.setVariable('beta_cmd_deg', timeseries(c.betaDeg, c.t));
simIn = simIn.setVariable('miu_cmd_deg', timeseries(c.miuDeg, c.t));
simIn = simIn.setVariable('xInit', xInit);
simIn = simIn.setModelParameter('StopTime', num2str(c.stopTime), ...
    'LoadInitialState', 'on', 'InitialState', 'xInit', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveState', 'on', ...
    'StateSaveName', 'xout', 'SaveFormat', 'StructureWithTime');
try
    simOut = sim(simIn);
    result = computeMetrics(c, simOut);
catch ME
    result.ok = false;
    result.error = ME.message;
end
end

function result = computeMetrics(c, simOut)
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
[Ma, Q] = arrayfun(@M_Q, H, V);
cmdAlpha = interp1(c.t, c.alphaDeg, t, 'linear', 'extrap');
cmdMiu = interp1(c.t, c.miuDeg, t, 'linear', 'extrap');
dt = [0; diff(t)];
dHdtAvg = (H(end) - H(1)) / max(t(end) - t(1), eps);
downpress = min(theta) < -1 || dHdtAvg < -20;

result = emptyResult();
result.caseDef = c;
result.ok = true;
result.t = t;
result.data = struct('H', H, 'V', V, 'theta', theta, 'alpha', alpha, 'beta', beta, 'miu', miu, ...
    'cmdAlpha', cmdAlpha, 'cmdMiu', cmdMiu, 'deltaA', deltaA, 'deltaE', deltaE, 'deltaR', deltaR, ...
    'Ma', Ma(:), 'Q', Q(:));
result.metrics = struct('stable', all(isfinite([H; V; theta; alpha; beta; miu])) && min(H) > 20000 && min(V) > 1500, ...
    'downpress', downpress, 'dH', H(end) - H(1), 'avgHdot', dHdtAvg, ...
    'minThetaDeg', min(theta), 'maxThetaDeg', max(theta), ...
    'maxAlphaErrDeg', max(abs(alpha - cmdAlpha)), 'maxMiuErrDeg', max(abs(miu - cmdMiu)), ...
    'maxBetaDeg', max(abs(beta)), 'maxDeltaDeg', max(abs([deltaA(:); deltaE(:); deltaR(:)])), ...
    'satFrac', mean(abs([deltaA(:); deltaE(:); deltaR(:)]) > 29.4), ...
    'minQ', min(Q), 'maxQ', max(Q), 'minMa', min(Ma), 'maxMa', max(Ma), ...
    'energyLoss', V(1) - V(end));
end

function result = emptyResult()
result = struct('caseDef', [], 'ok', false, 'error', '', 't', [], 'data', [], 'metrics', []);
end

function writeCsv(file, results)
fid = fopen(file, 'w'); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'case,ok,stable,downpress,dH,avgHdot,minThetaDeg,maxAlphaErrDeg,maxMiuErrDeg,maxBetaDeg,maxDeltaDeg,satFrac,minQ,maxQ,minMa,maxMa,energyLoss,error\n');
for k = 1:numel(results)
    r = results(k);
    if r.ok
        m = r.metrics;
        fprintf(fid, '%s,1,%d,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,\n', ...
            r.caseDef.name, m.stable, m.downpress, m.dH, m.avgHdot, m.minThetaDeg, ...
            m.maxAlphaErrDeg, m.maxMiuErrDeg, m.maxBetaDeg, m.maxDeltaDeg, m.satFrac, ...
            m.minQ, m.maxQ, m.minMa, m.maxMa, m.energyLoss);
    else
        fprintf(fid, '%s,0,0,0,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,%s\n', ...
            r.caseDef.name, strrep(r.error, ',', ';'));
    end
end
end

function makePlots(figDir, results)
for k = 1:numel(results)
    r = results(k);
    if ~r.ok, continue; end
    d = r.data; t = r.t;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1100 750]);
    tiledlayout(3, 2);
    nexttile; plot(t, d.alpha, t, d.cmdAlpha, '--'); grid on; ylabel('alpha deg'); legend('actual','cmd');
    nexttile; plot(t, d.miu, t, d.cmdMiu, '--'); grid on; ylabel('miu deg'); legend('actual','cmd');
    nexttile; plot(t, d.theta); grid on; ylabel('theta deg');
    nexttile; plot(t, d.H/1000); grid on; ylabel('H km');
    nexttile; plot(t, [d.deltaA d.deltaE d.deltaR]); yline(30, ':'); yline(-30, ':'); grid on; ylabel('surface deg'); legend('da','de','dr');
    nexttile; plot(t, d.Q/1000); grid on; ylabel('Q kPa'); xlabel('s');
    exportgraphics(fig, fullfile(figDir, [r.caseDef.name '.png']));
    close(fig);
end
end

function writeReport(file, results)
fid = fopen(file, 'w'); cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# Alpha + bank-reversal down-press test\n\n');
fprintf(fid, 'This campaign keeps positive alpha and commands large bank reversal to turn lift downward. Original ADRC structure is unchanged.\n\n');
fprintf(fid, '| Case | OK | Stable | Downpress | dH m | avg Hdot m/s | min theta deg | max alpha err | max miu err | max surface | sat frac |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for k = 1:numel(results)
    r = results(k);
    if r.ok
        m = r.metrics;
        fprintf(fid, '| %s | 1 | %d | %d | %.1f | %.1f | %.2f | %.2f | %.2f | %.1f | %.2f |\n', ...
            r.caseDef.name, m.stable, m.downpress, m.dH, m.avgHdot, m.minThetaDeg, ...
            m.maxAlphaErrDeg, m.maxMiuErrDeg, m.maxDeltaDeg, m.satFrac);
    else
        fprintf(fid, '| %s | 0 | 0 | 0 | fail | fail | fail | fail | fail | fail | fail |\n', r.caseDef.name);
    end
end
fprintf(fid, '\nInterpretation: if large bank commands fail or produce huge miu error, the limiting factor is bank-reversal authority/tracking, not the alpha channel alone.\n');
end
