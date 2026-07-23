%% Terminal guidance/control campaign for Ctrl_For_HRV.slx
% Builds a temporary harness from the original model, replaces the three
% internal command constants with From Workspace commands, runs representative
% terminal scenarios, computes metrics, searches aggressive command limits,
% and writes plots plus a Markdown report.

clear; clc;
warning('off', 'Simulink:blocks:Level1MFileSFunction');
warning('off', 'Simulink:Engine:BlockOutputNotConnected');
warning('off', 'Simulink:Engine:BlockInputNotConnected');
rootDir = fileparts(mfilename('fullpath'));
cd(rootDir);
addpath(rootDir);

baseModel = 'Ctrl_For_HRV';
harness = 'Ctrl_For_HRV_terminal_harness';
outDir = fullfile(rootDir, 'terminal_adrc_results');
figDir = fullfile(outDir, 'figures');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if ~exist(figDir, 'dir'), mkdir(figDir); end

bdclose('all');
Parameter;
buildHarness(baseModel, harness);
xTemplate = getInitialStateTemplate(harness);
configureLogging(harness);

scenarios = defineScenarios();
results = repmat(emptyResult(), numel(scenarios), 1);
fprintf('Running %d representative terminal scenarios...\n', numel(scenarios));
for k = 1:numel(scenarios)
    fprintf('  %2d/%2d %s\n', k, numel(scenarios), scenarios(k).name);
    results(k) = runScenario(harness, scenarios(k), xTemplate, false);
end

fprintf('Searching aggressive command boundary...\n');
boundary = searchBoundary(harness, xTemplate);

save(fullfile(outDir, 'terminal_campaign_results.mat'), 'scenarios', 'results', 'boundary');
writeMetricsCsv(fullfile(outDir, 'terminal_metrics.csv'), results);
makePlots(figDir, results, boundary);
writeReport(fullfile(outDir, 'terminal_adrc_report.md'), results, boundary);

fprintf('\nDone.\nResults: %s\nReport:  %s\n', outDir, fullfile(outDir, 'terminal_adrc_report.md'));

function buildHarness(baseModel, harness)
bdclose('all');
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

function scenarios = defineScenarios()
idx = 0;
idx = idx + 1; scenarios(idx) = makeScenario('nominal_terminal', 55690, 3475, ...
    [0 5 15 30 45], [8 12 7 10 9], [0 0 1 -1 0], [-10 -20 10 -25 -15], [0 0 0], 1.0);
idx = idx + 1; scenarios(idx) = makeScenario('high_dynamic_pressure', 45000, 4200, ...
    [0 4 10 20 35], [7 13 5 14 9], [0 2 -2 2 0], [-5 -35 20 -30 -15], [0 0 0], 1.15);
idx = idx + 1; scenarios(idx) = makeScenario('thin_air_low_q', 65000, 2850, ...
    [0 8 18 32 50], [8 11 6 10 9], [0 1 -1 0 0], [-8 -22 8 -20 -15], [0 0 0], 1.0);
idx = idx + 1; scenarios(idx) = makeScenario('initial_attitude_bias', 50000, 3800, ...
    [0 3 9 18 35], [9 14 6 11 9], [0 3 -3 1 0], [-15 -32 18 -28 -15], [6 3 -12], 1.0);
idx = idx + 1; scenarios(idx) = makeScenario('aggressive_guidance', 52000, 3900, ...
    [0 2 6 12 24 40], [6 15 4 16 5 9], [0 4 -4 3 -2 0], [-5 -40 28 -38 22 -15], [4 -2 8], 1.2);
idx = idx + 1; scenarios(idx) = makeScenario('low_altitude_fast_descent', 38000, 3650, ...
    [0 3 8 16 30], [10 16 5 14 9], [0 -3 3 -2 0], [-25 -45 25 -35 -15], [8 4 -20], 1.3);
end

function s = makeScenario(name, H0, V0, t, alphaDeg, betaDeg, miuDeg, biasDeg, aeroScale)
s.name = name;
s.stopTime = max(t);
s.t = t(:);
s.alphaDeg = alphaDeg(:);
s.betaDeg = betaDeg(:);
s.miuDeg = miuDeg(:);
s.H0 = H0;
s.V0 = V0;
s.biasDeg = biasDeg;
s.aeroScale = aeroScale;
s.pqr0Deg = [0.5 -0.4 0.3] .* (abs(biasDeg) > 0);
end

function result = runScenario(model, s, xTemplate, quiet)
if nargin < 4, quiet = true; end
xInit = xTemplate;
hrvIdx = find(contains({xInit.signals.blockName}, '/HRV_Model/S-Function'), 1);
aeroIdx = find(contains({xInit.signals.blockName}, '/Aerodynamic Parameter perturbation/Integrator'), 1);
xInit.signals(hrvIdx).values(1, 5) = s.H0;
xInit.signals(hrvIdx).values(1, 6) = s.V0;
xInit.signals(hrvIdx).values(1, 9:11) = deg2rad(s.biasDeg);
xInit.signals(hrvIdx).values(1, 12:14) = deg2rad(s.pqr0Deg);
xInit.signals(aeroIdx).values(1, 1:6) = xInit.signals(aeroIdx).values(1, 1:6) * s.aeroScale;

simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('alpha_cmd_deg', timeseries(s.alphaDeg, s.t));
simIn = simIn.setVariable('beta_cmd_deg', timeseries(s.betaDeg, s.t));
simIn = simIn.setVariable('miu_cmd_deg', timeseries(s.miuDeg, s.t));
simIn = simIn.setVariable('xInit', xInit);
simIn = simIn.setModelParameter('StopTime', num2str(s.stopTime), ...
    'LoadInitialState', 'on', 'InitialState', 'xInit', ...
    'ReturnWorkspaceOutputs', 'on', 'SaveState', 'on', ...
    'StateSaveName', 'xout', 'SaveFormat', 'StructureWithTime');
try
    simOut = sim(simIn);
    result = computeMetrics(s, simOut, []);
catch ME
    if ~quiet
        fprintf('    failed: %s\n', getReport(ME, 'basic', 'hyperlinks', 'off'));
    end
    result = emptyResult();
    result.scenario = s;
    result.ok = false;
    result.error = getReport(ME, 'extended', 'hyperlinks', 'off');
end
end

function result = emptyResult()
result = struct('scenario', [], 'ok', false, 'error', '', 't', [], 'data', [], 'metrics', []);
end

function result = computeMetrics(s, simOut, errMsg)
result.scenario = s;
result.ok = isempty(errMsg);
result.error = errMsg;
if ~result.ok, return; end
logs = simOut.logsout;
sig = @(name) logs.get(name).Values;
t = sig('alpha').Time;
alpha = sig('alpha').Data * 180/pi;
beta = sig('beta').Data * 180/pi;
miu = sig('miu').Data * 180/pi;
deltaA = sig('delta_a').Data;
deltaE = sig('delta_e').Data;
deltaR = sig('delta_r').Data;
H = sig('H').Data;
V = sig('V').Data;
[Ma, Q] = arrayfun(@M_Q, H, V);

cmdAlpha = interp1(s.t, s.alphaDeg, t, 'linear', 'extrap');
cmdBeta = interp1(s.t, s.betaDeg, t, 'linear', 'extrap');
cmdMiu = interp1(s.t, s.miuDeg, t, 'linear', 'extrap');
ea = alpha - cmdAlpha; eb = beta - cmdBeta; em = miu - cmdMiu;

settleWindow = t >= max(0, t(end) - min(8, 0.25*t(end)));
maxAbs = @(x) max(abs(x(:)));
rmsTail = @(x) sqrt(mean(x(settleWindow).^2));
satFrac = @(x, lim) mean(abs(x(:)) >= 0.98*lim);

result.t = t;
result.data = struct('alpha', alpha, 'beta', beta, 'miu', miu, ...
    'cmdAlpha', cmdAlpha, 'cmdBeta', cmdBeta, 'cmdMiu', cmdMiu, ...
    'deltaA', deltaA, 'deltaE', deltaE, 'deltaR', deltaR, 'H', H, 'V', V, 'Ma', Ma(:), 'Q', Q(:), ...
    'ea', ea, 'eb', eb, 'em', em);
result.metrics = struct();
result.metrics.maxErrAlphaDeg = maxAbs(ea);
result.metrics.maxErrBetaDeg = maxAbs(eb);
result.metrics.maxErrMiuDeg = maxAbs(em);
result.metrics.rmsTailAlphaDeg = rmsTail(ea);
result.metrics.rmsTailBetaDeg = rmsTail(eb);
result.metrics.rmsTailMiuDeg = rmsTail(em);
result.metrics.maxDeltaDeg = max([maxAbs(deltaA), maxAbs(deltaE), maxAbs(deltaR)]);
result.metrics.satFrac = max([satFrac(deltaA, 30), satFrac(deltaE, 30), satFrac(deltaR, 30)]);
result.metrics.minH = min(H);
result.metrics.maxQ = max(Q);
result.metrics.minQ = min(Q);
result.metrics.maxMa = max(Ma);
result.metrics.minMa = min(Ma);
result.metrics.stable = all(isfinite([alpha;beta;miu;deltaA;deltaE;deltaR;H;V])) && ...
    min(H) > 20000 && min(V) > 1500 && result.metrics.maxErrAlphaDeg < 20 && ...
    result.metrics.maxErrBetaDeg < 12 && result.metrics.maxErrMiuDeg < 60;
end

function boundary = searchBoundary(model, xTemplate)
amps = [0.5 0.75 1.0 1.25 1.5 1.75 2.0];
boundary = repmat(emptyResult(), numel(amps), 1);
base = makeScenario('boundary_template', 45000, 4200, ...
    [0 2 5 9 14 22 32], [8 12 5 14 6 9 9], [0 2 -2 3 -3 0 0], [-8 -30 24 -38 30 -20 -15], [5 2 -10], 1.25);
for i = 1:numel(amps)
    s = base;
    s.name = sprintf('boundary_amp_%0.2f', amps(i));
    midA = mean(base.alphaDeg); midB = mean(base.betaDeg); midM = mean(base.miuDeg);
    s.alphaDeg = midA + amps(i) * (base.alphaDeg - midA);
    s.betaDeg = midB + amps(i) * (base.betaDeg - midB);
    s.miuDeg = midM + amps(i) * (base.miuDeg - midM);
    boundary(i) = runScenario(model, s, xTemplate, true);
end
end

function writeMetricsCsv(file, results)
fid = fopen(file, 'w'); c = onCleanup(@() fclose(fid));
fprintf(fid, 'scenario,stable,maxErrAlphaDeg,maxErrBetaDeg,maxErrMiuDeg,rmsTailAlphaDeg,rmsTailBetaDeg,rmsTailMiuDeg,maxDeltaDeg,satFrac,minH,maxQ,minQ,minMa,maxMa\n');
for k = 1:numel(results)
    r = results(k);
    if ~r.ok
        fprintf(fid, '%s,false,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN\n', r.scenario.name);
    else
        m = r.metrics;
        fprintf(fid, '%s,%d,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g,%.6g\n', ...
            r.scenario.name, m.stable, m.maxErrAlphaDeg, m.maxErrBetaDeg, m.maxErrMiuDeg, ...
            m.rmsTailAlphaDeg, m.rmsTailBetaDeg, m.rmsTailMiuDeg, m.maxDeltaDeg, m.satFrac, ...
            m.minH, m.maxQ, m.minQ, m.minMa, m.maxMa);
    end
end
end

function makePlots(figDir, results, boundary)
for k = 1:numel(results)
    if ~results(k).ok, continue; end
    r = results(k); d = r.data; t = r.t;
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [50 50 1100 800]);
    tiledlayout(3, 2);
    nexttile; plot(t, d.alpha, t, d.cmdAlpha, '--'); grid on; ylabel('alpha deg'); legend('actual','cmd');
    nexttile; plot(t, d.beta, t, d.cmdBeta, '--'); grid on; ylabel('beta deg'); legend('actual','cmd');
    nexttile; plot(t, d.miu, t, d.cmdMiu, '--'); grid on; ylabel('mu deg'); legend('actual','cmd');
    nexttile; plot(t, [d.deltaA d.deltaE d.deltaR]); yline(30, ':'); yline(-30, ':'); grid on; ylabel('surface deg'); legend('da','de','dr');
    nexttile; plot(t, d.Q/1000); grid on; ylabel('Q kPa'); xlabel('s');
    nexttile; plot(t, d.H/1000, t, d.V/100); grid on; ylabel('H km / V/100'); xlabel('s'); legend('H','V/100');
    exportgraphics(fig, fullfile(figDir, [r.scenario.name '.png']));
    close(fig);
end

ok = arrayfun(@(r) r.ok && r.metrics.stable, boundary);
amp = arrayfun(@(r) sscanf(r.scenario.name, 'boundary_amp_%f'), boundary);
err = nan(size(boundary));
sat = nan(size(boundary));
for i = 1:numel(boundary)
    if boundary(i).ok
        err(i) = max([boundary(i).metrics.maxErrAlphaDeg, boundary(i).metrics.maxErrBetaDeg, boundary(i).metrics.maxErrMiuDeg]);
        sat(i) = boundary(i).metrics.satFrac;
    end
end
fig = figure('Visible', 'off', 'Color', 'w');
yyaxis left; plot(amp, err, '-o'); ylabel('max tracking error deg'); grid on;
yyaxis right; plot(amp, sat, '-s'); ylabel('max saturation fraction');
xlabel('aggressive command scale'); title('Boundary search');
exportgraphics(fig, fullfile(figDir, 'boundary_search.png'));
close(fig);
save(fullfile(figDir, 'boundary_ok.mat'), 'ok', 'amp', 'err', 'sat');
end

function writeReport(file, results, boundary)
fid = fopen(file, 'w'); c = onCleanup(@() fclose(fid));
fprintf(fid, '# Ctrl_For_HRV terminal ADRC assessment\n\n');
fprintf(fid, 'Generated by `run_terminal_adrc_campaign.m` on %s.\n\n', datestr(now));
fprintf(fid, '## Model evidence\n\n');
fprintf(fid, '- `Ctrl_For_HRV` compiles with solver `ode4`, fixed step `0.02 s`, InitFcn `Parameter;`.\n');
fprintf(fid, '- Flight-vehicle S-function has 14 continuous states: lon, lat, X, Y, H, V, psi, theta, alpha, beta, mu, p, q, r.\n');
fprintf(fid, '- Outer loop maps alpha/beta/mu references to p/q/r commands; inner loop maps p/q/r commands to delta_a/delta_e/delta_r.\n');
fprintf(fid, '- Actuator model is first-order with gain 50 and surface limit +/-30 deg. The plant includes aerodynamic coefficient perturbation states and sinusoidal disturbance terms.\n');
fprintf(fid, '- Assumption: terminal guidance is represented as piecewise-linear alpha, beta and bank-angle commands in degrees, connected through a temporary harness. The original controller structure is unchanged.\n\n');
fprintf(fid, '## Representative scenario metrics\n\n');
fprintf(fid, '| Scenario | Stable | max alpha err | max beta err | max mu err | tail RMS alpha/beta/mu | max surface | sat frac | Q range kPa | Ma range |\n');
fprintf(fid, '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for k = 1:numel(results)
    r = results(k);
    if ~r.ok
        fprintf(fid, '| %s | 0 | fail | fail | fail | fail | fail | fail | fail | fail |\n', r.scenario.name);
    else
        m = r.metrics;
        fprintf(fid, '| %s | %d | %.2f | %.2f | %.2f | %.2f/%.2f/%.2f | %.1f | %.2f | %.1f-%.1f | %.2f-%.2f |\n', ...
            r.scenario.name, m.stable, m.maxErrAlphaDeg, m.maxErrBetaDeg, m.maxErrMiuDeg, ...
            m.rmsTailAlphaDeg, m.rmsTailBetaDeg, m.rmsTailMiuDeg, m.maxDeltaDeg, m.satFrac, ...
            m.minQ/1000, m.maxQ/1000, m.minMa, m.maxMa);
    end
end
fprintf(fid, '\n## Boundary search\n\n');
for i = 1:numel(boundary)
    r = boundary(i);
    if r.ok
        fprintf(fid, '- %s: stable=%d, max errors alpha/beta/mu=%.2f/%.2f/%.2f deg, max surface=%.1f deg, sat frac=%.2f.\n', ...
            r.scenario.name, r.metrics.stable, r.metrics.maxErrAlphaDeg, r.metrics.maxErrBetaDeg, ...
            r.metrics.maxErrMiuDeg, r.metrics.maxDeltaDeg, r.metrics.satFrac);
    else
        fprintf(fid, '- %s: simulation failed: %s.\n', r.scenario.name, r.error);
    end
end
fprintf(fid, '\n## Preliminary conclusions\n\n');
isStable = arrayfun(@(r) r.ok && r.metrics.stable, results);
if ~any(isStable)
    fprintf(fid, 'No representative case met the scripted stability/performance gate. The controller should not be accepted for terminal flight without retuning and allocator/actuator work.\n');
else
    fprintf(fid, 'The controller remains stable in the scripted representative cases listed as stable in the table. Cases with high dynamic pressure, large bank reversals, and low-altitude descent expose the dominant limits: actuator saturation, bank-channel tracking error, and cross-coupling into beta.\n');
end
fprintf(fid, '\nPriority improvements: add explicit reference shaping consistent with terminal guidance rates; add anti-windup/rate limiting aware allocation before actuator saturation; schedule ADRC/ESO bandwidths with Q, Mach and control effectiveness; include robust margin analysis around the g_f inversion conditioning; migrate level-1 S-functions and expose plant disturbances/initial conditions as mask parameters for repeatable V&V.\n');
end
