function results = run_hrv_guidance_cases()
%RUN_HRV_GUIDANCE_CASES Batch validation for HRV terminal guidance.

mdl = 'Ctrl_For_HRV_20260625';
cases = defineCases();
results = repmat(emptyResult(), numel(cases), 1);

for k = 1:numel(cases)
    bdclose(mdl);
    clear functions;
    assignin('base', 'HRV_INIT', cases(k).init);
    load_system(mdl);
    set_param([mdl '/target/Integrator'], 'InitialCondition', mat2str(cases(k).target(:)));
    set_param(mdl, 'FixedStep', '0.02');
    try
        simOut = sim(mdl, 'StopTime', '34', 'ReturnWorkspaceOutputs', 'on');
        result = evaluateCase(simOut, cases(k));
    catch ME
        result = emptyResult();
        result.name = cases(k).name;
        result.target = cases(k).target;
        result.error = ME.message;
        fprintf('%02d %-16s FAILED: %s\n', k, result.name, ME.message);
    end
    results(k) = result;
    if isempty(result.error)
        fprintf('%02d %-16s min=%8.3f m sample=%8.3f m t*=%.3f end=%.3f vec=[%7.3f %7.3f %7.3f]\n', ...
            k, result.name, result.segmentMin, result.sampleMin, result.tClosest, result.tEnd, ...
            result.errX, result.errY, result.errH);
    end
end

assignin('base', 'HRV_CASE_RESULTS', results);
valid = [results.valid];
fprintf('Summary: valid %d/%d, pass<=1m %d/%d, max=%.3f m, mean=%.3f m\n', ...
    nnz(valid), numel(results), nnz([results(valid).segmentMin] <= 1.0), nnz(valid), ...
    max([results(valid).segmentMin]), mean([results(valid).segmentMin]));
end

function cases = defineCases()
baseInit = struct('H0', 55690, 'V0', 5000, ...
    'psi0', 16.7*pi/180, 'theta0', -29.48*pi/180);

defs = {
    'nominal',      [100000 30000 0], baseInit
    'target_x_low', [ 99800 30000 0], baseInit
    'target_x_high',[100200 30000 0], baseInit
    'target_y_low', [100000 29800 0], baseInit
    'target_y_high',[100000 30200 0], baseInit
    'target_diag_1',[ 99900 29900 0], baseInit
    'target_diag_2',[100100 30100 0], baseInit
    'init_h_low',   [100000 30000 0], withField(baseInit, 'H0', 55590)
    'init_h_high',  [100000 30000 0], withField(baseInit, 'H0', 55790)
    'init_v_low',   [100000 30000 0], withField(baseInit, 'V0', 4975)
    'init_v_high',  [100000 30000 0], withField(baseInit, 'V0', 5025)
    'init_ang_mix', [100000 30000 0], withAngles(baseInit, 16.8, -29.58)
    };

cases = repmat(struct('name', '', 'target', [], 'init', baseInit), size(defs, 1), 1);
for i = 1:size(defs, 1)
    cases(i).name = defs{i, 1};
    cases(i).target = defs{i, 2};
    cases(i).init = defs{i, 3};
end
end

function s = withField(s, fieldName, value)
s.(fieldName) = value;
end

function s = withAngles(s, psiDeg, thetaDeg)
s.psi0 = psiDeg*pi/180;
s.theta0 = thetaDeg*pi/180;
end

function result = evaluateCase(simOut, caseDef)
t = simOut.Scope_X(:, 1);
pos = [simOut.Scope_X(:, 2), simOut.Scope_Y(:, 2), simOut.Scope_H(:, 2)];
target = simOut.Scope_position_target1(1, 2:4);
dist = sqrt(sum((pos - target).^2, 2));
[sampleMin, sampleIdx] = min(dist);

segmentMin = inf;
tClosest = t(sampleIdx);
closest = pos(sampleIdx, :);
for i = 1:size(pos, 1)-1
    step = pos(i+1, :) - pos(i, :);
    den = dot(step, step);
    if den > 0
        u = max(0, min(1, dot(target - pos(i, :), step)/den));
    else
        u = 0;
    end
    candidate = pos(i, :) + u*step;
    candidateDist = norm(candidate - target);
    if candidateDist < segmentMin
        segmentMin = candidateDist;
        tClosest = t(i) + u*(t(i+1) - t(i));
        closest = candidate;
    end
end

err = closest - target;
result = emptyResult();
result.name = caseDef.name;
result.target = caseDef.target;
result.segmentMin = segmentMin;
result.sampleMin = sampleMin;
result.tClosest = tClosest;
result.tEnd = t(end);
result.errX = err(1);
result.errY = err(2);
result.errH = err(3);
result.valid = true;
end

function result = emptyResult()
result = struct('name', '', 'target', [], 'segmentMin', NaN, 'sampleMin', NaN, ...
    'tClosest', NaN, 'tEnd', NaN, 'errX', NaN, 'errY', NaN, 'errH', NaN, ...
    'valid', false, 'error', '');
end
