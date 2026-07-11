function metrics = evaluateMultiVehicleCooperativeMetrics()
%EVALUATEMULTIVEHICLECOOPERATIVEMETRICS Run and evaluate the 5-HRV model.

model = 'Ctrl_For_HRV_20260625';
thetaDesired = [-53 -49 -45 -41 -38];
psiDesired = [17 17 17 17 17];

load_system(model);
warningState = warning('off', 'all');
cleanup = onCleanup(@() warning(warningState));
evalin('base', sprintf('sim(''%s'');', model));

target = evalin('base', 'Scope_position_target1');
t = target(:, 1);
targetX = target(:, 2);
targetY = target(:, 3);
targetH = target(:, 4);

miss = zeros(1, 5);
arrivalTime = zeros(1, 5);
thetaTerminal = zeros(1, 5);
psiTerminal = zeros(1, 5);
initialPosition = zeros(5, 3);
missVector = zeros(5, 3);

for i = 1:5
    x = evalin('base', sprintf('ScopeData%d_v%d', 10 + 30*(i-1), i));
    y = evalin('base', sprintf('ScopeData%d_v%d', 11 + 30*(i-1), i));
    h = evalin('base', sprintf('ScopeData%d_v%d', 8 + 30*(i-1), i));
    theta = evalin('base', sprintf('ScopeData%d_v%d', 30 + 30*(i-1), i));
    psi = evalin('base', sprintf('ScopeData%d_v%d', 27 + 30*(i-1), i));
    initialPosition(i, :) = [x(1, 2), y(1, 2), h(1, 2)];

    rel = [x(:, 2) - targetX, y(:, 2) - targetY, h(:, 2) - targetH];
    [miss(i), arrivalTime(i), idx, frac, missVector(i, :)] = closestLoggedApproach(t, rel);
    thetaTerminal(i) = interpSample(theta(:, 2), idx, frac);
    psiTerminal(i) = interpSample(psi(:, 2), idx, frac);
end

metrics = struct();
metrics.miss = miss;
metrics.missVector = missVector;
metrics.initialPosition = initialPosition;
metrics.maxInitialSeparation = maxPairwiseDistance(initialPosition);
metrics.initialSeparationRequirementMet = metrics.maxInitialSeparation <= 5000.0;
metrics.arrivalTime = arrivalTime;
metrics.arrivalSpread = max(arrivalTime) - min(arrivalTime);
metrics.thetaTerminal = thetaTerminal;
metrics.psiTerminal = psiTerminal;
metrics.thetaDesired = thetaDesired;
metrics.psiDesired = psiDesired;
metrics.thetaError = thetaTerminal - thetaDesired;
metrics.psiError = psiTerminal - psiDesired;
metrics.maxAbsThetaError = max(abs(metrics.thetaError));
metrics.maxAbsPsiError = max(abs(metrics.psiError));
metrics.maxMiss = max(miss);
metrics.minThetaSeparation = min(abs(diff(sort(thetaTerminal))));
metrics.minPsiSeparation = min(abs(diff(sort(psiTerminal))));
metrics.timeRequirementMet = metrics.arrivalSpread <= 5.0;
metrics.missRequirementMet = metrics.maxMiss <= 10.0;
metrics.thetaErrorRequirementMet = metrics.maxAbsThetaError <= 3.0;
metrics.thetaSeparationRequirementMet = metrics.minThetaSeparation > 2.0;
metrics.psiSeparationRequirementMet = metrics.minPsiSeparation > 3.0;
metrics.requirementsMet = metrics.missRequirementMet && ...
    metrics.timeRequirementMet && metrics.thetaErrorRequirementMet && ...
    metrics.thetaSeparationRequirementMet;
metrics.cooperativeWeight = estimateCooperativeWeightFromLogs(t, targetX, targetY, targetH);

fprintf('Vehicle  miss_m    t_s      theta_deg  theta_err  psi_deg   psi_err   dx_m      dy_m      dh_m\n');
for i = 1:5
    fprintf('%d        %.3f   %.3f   %.3f    %.3f      %.3f    %.3f   %.1f   %.1f   %.1f\n', ...
        i, miss(i), arrivalTime(i), thetaTerminal(i), metrics.thetaError(i), ...
        psiTerminal(i), metrics.psiError(i), missVector(i, 1), missVector(i, 2), ...
        missVector(i, 3));
end
fprintf('Arrival spread: %.3f s\n', metrics.arrivalSpread);
fprintf('Max miss: %.3f m, requirement=%d\n', metrics.maxMiss, ...
    metrics.missRequirementMet);
fprintf('Max initial separation: %.3f m, requirement=%d\n', ...
    metrics.maxInitialSeparation, metrics.initialSeparationRequirementMet);
fprintf('Max abs theta error: %.3f deg\n', metrics.maxAbsThetaError);
fprintf('Max abs psi error: %.3f deg\n', metrics.maxAbsPsiError);
fprintf('Min theta separation: %.3f deg\n', metrics.minThetaSeparation);
fprintf('Min psi separation: %.3f deg\n', metrics.minPsiSeparation);
fprintf('Requirements met: miss=%d time=%d thetaErr=%d thetaSep=%d all=%d\n', ...
    metrics.missRequirementMet, metrics.timeRequirementMet, ...
    metrics.thetaErrorRequirementMet, metrics.thetaSeparationRequirementMet, ...
    metrics.requirementsMet);
fprintf('Coop weight: start %.3f, mid %.3f, final %.3f, min %.3f, max %.3f\n', ...
    metrics.cooperativeWeight(1), metrics.cooperativeWeight(round(end/2)), ...
    metrics.cooperativeWeight(end), min(metrics.cooperativeWeight), ...
    max(metrics.cooperativeWeight));

save('multi_vehicle_metric_check.mat', 'metrics');
end

function [miss, tClosest, idx, frac, rBest] = closestLoggedApproach(t, rel)
miss = inf;
tClosest = t(1);
idx = 1;
frac = 0.0;
rBest = rel(1, :);
for k = 1:size(rel, 1)-1
    r0 = rel(k, :);
    r1 = rel(k+1, :);
    dr = r1 - r0;
    den = dot(dr, dr);
    if den <= eps
        s = 0.0;
    else
        s = min(max(-dot(r0, dr)/den, 0.0), 1.0);
    end
    rClosest = r0 + s*dr;
    d = norm(rClosest);
    if d < miss
        miss = d;
        tClosest = t(k) + s*(t(k+1)-t(k));
        idx = k;
        frac = s;
        rBest = rClosest;
    end
end
end

function value = interpSample(signal, idx, frac)
if idx >= numel(signal)
    value = signal(end);
else
    value = signal(idx) + frac*(signal(idx+1)-signal(idx));
end
end

function weight = estimateCooperativeWeightFromLogs(t, targetX, targetY, targetH)
weight = zeros(size(t));
for k = 1:numel(t)
    eta = zeros(1, 5);
    xi = zeros(1, 5);
    ranges = zeros(1, 5);
    for i = 1:5
        x = evalin('base', sprintf('ScopeData%d_v%d', 10 + 30*(i-1), i));
        y = evalin('base', sprintf('ScopeData%d_v%d', 11 + 30*(i-1), i));
        h = evalin('base', sprintf('ScopeData%d_v%d', 8 + 30*(i-1), i));
        speedLog = evalin('base', sprintf('ScopeData%d_v%d', 9 + 30*(i-1), i));
        theta = evalin('base', sprintf('ScopeData%d_v%d', 30 + 30*(i-1), i));
        psi = evalin('base', sprintf('ScopeData%d_v%d', 27 + 30*(i-1), i));
        r = [targetX(k) - x(k, 2); targetY(k) - y(k, 2); targetH(k) - h(k, 2)];
        ranges(i) = max(norm(r), 1.0);
        speed = max(speedLog(k, 2), 1.0);
        v = speed * [cosd(theta(k, 2))*cosd(psi(k, 2)); ...
            cosd(theta(k, 2))*sind(psi(k, 2)); sind(theta(k, 2))];
        eta(i) = ranges(i) / speed;
        xi(i) = -dot(v/max(norm(v), 1.0), r/ranges(i));
    end
    cooperativeState = mean(abs(eta - mean(eta))) + 20.0*mean(abs(xi - mean(xi)));
    weight(k) = fuzzyWeightEstimate(cooperativeState, mean(ranges));
end
end

function distance = maxPairwiseDistance(position)
distance = 0.0;
for i = 1:size(position, 1)
    for j = i+1:size(position, 1)
        distance = max(distance, norm(position(i, :) - position(j, :)));
    end
end
end

function weight = fuzzyWeightEstimate(cooperativeState, averageRange)
terminalPngRange = 10000.0;
cooperativeScale = 3.0;
fuzzyFarRange = 60000.0;
sigma = 0.45;
cooInput = 2.0*clamp(cooperativeState/cooperativeScale, 0.0, 1.0) - 1.0;
rangeInput = 2.0*clamp((averageRange - terminalPngRange)/ ...
    max(fuzzyFarRange - terminalPngRange, eps), 0.0, 1.0) - 1.0;
centers = [-1.0, 0.0, 1.0];
cooMembership = gaussianMembership(cooInput, centers, sigma);
rangeMembership = gaussianMembership(rangeInput, centers, sigma);
ruleLevels = [-1.0 -1.0 -1.0; -1.0 0.0 1.0; -1.0 0.0 1.0];
num = 0.0;
den = 0.0;
for k = 1:401
    y = -1.0 + (k-1)*(2.0/400.0);
    aggregated = 0.0;
    for row = 1:3
        for col = 1:3
            activation = min(rangeMembership(row), cooMembership(col));
            consequent = gaussianMembership(y, ruleLevels(row, col), sigma);
            aggregated = max(aggregated, min(activation, consequent));
        end
    end
    num = num + y*aggregated;
    den = den + aggregated;
end
if den <= eps
    fuzzyOutput = -1.0;
else
    fuzzyOutput = num/den;
end
weight = clamp(0.5*(fuzzyOutput + 1.0), 0.0, 1.0);
end

function y = gaussianMembership(x, centers, sigma)
y = exp(-((x - centers).^2)/(2.0*sigma*sigma));
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
