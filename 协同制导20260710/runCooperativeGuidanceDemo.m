function results = runCooperativeGuidanceDemo()
%RUNCOOPERATIVEGUIDANCEDEMO Demo for fuzzy time-cooperative guidance.
%
% The simulation uses a normal-acceleration point-mass model. It is meant
% to validate the cooperative guidance layer before wiring the same command
% into the full HRV Simulink plant.

target = [100000; 30000; 0];
vehicleCount = 4;
dt = 0.02;
maxTime = 90;
hitRadius = 10;

initialPosition = [ ...
    0,    -4000,   2500,  -2500;
    0,     5000,  -3500,   2500;
    26000, 29000, 24000,  31000];
initialSpeed = [2700, 2500, 2850, 2400];
impactThetaDeg = [-39.0, -42.0, -45.0, -48.0];
impactPsiDeg = [9.0, 13.0, 17.0, 21.0];

initialVelocity = zeros(3, vehicleCount);
impactDirection = zeros(3, vehicleCount);
for i = 1:vehicleCount
    lineOfSight = target - initialPosition(:, i);
    impactDirection(:, i) = directionFromFlightPathAngles( ...
        impactThetaDeg(i), impactPsiDeg(i));
    nominalDirection = lineOfSight / norm(lineOfSight);
    biasedDirection = 0.97 * nominalDirection + 0.03 * impactDirection(:, i);
    initialVelocity(:, i) = initialSpeed(i) * biasedDirection / norm(biasedDirection);
end

adjacency = ones(vehicleCount) - eye(vehicleCount);
params = struct( ...
    'navigationConstant', 4.2, ...
    'consensusGain', 0.005, ...
    'fovMaxRad', deg2rad(80), ...
    'maxAcceleration', 20 * 9.806, ...
    'terminalPngRange', 55000, ...
    'terminalBlendRange', 1, ...
    'fuzzyFarRange', 90000, ...
    'cooperativeScale', 10.0, ...
    'impactDirection', impactDirection, ...
    'aimpointFraction', [0, 0, 0, 0], ...
    'maxAimpointSetback', [0, 0, 0, 0], ...
    'timeCoordinationGain', 0, ...
    'timeCoordinationTurnSign', 1, ...
    'timeWaypointGain', 0, ...
    'maxTimeWaypointOffset', 0, ...
    'cooperativeAimpointGain', 0, ...
    'maxCooperativeAimpointOffset', 0);

stepCount = floor(maxTime / dt) + 1;
time = (0:stepCount-1).' * dt;
positionHistory = nan(stepCount, 3, vehicleCount);
velocityHistory = nan(stepCount, 3, vehicleCount);
rangeHistory = nan(stepCount, vehicleCount);
etaHistory = nan(stepCount, vehicleCount);
xiHistory = nan(stepCount, vehicleCount);
coopWeightHistory = nan(stepCount, 1);
pngWeightHistory = nan(stepCount, 1);
accelHistory = nan(stepCount, 3, vehicleCount);
normalLoadHistory = nan(stepCount, vehicleCount);
lateralLoadHistory = nan(stepCount, vehicleCount);
arrivalTime = nan(1, vehicleCount);
terminalThetaDeg = nan(1, vehicleCount);
terminalPsiDeg = nan(1, vehicleCount);

position = initialPosition;
velocity = initialVelocity;

for k = 1:stepCount
    positionHistory(k, :, :) = reshape(position, 1, 3, vehicleCount);
    velocityHistory(k, :, :) = reshape(velocity, 1, 3, vehicleCount);

    [aCmd, info] = cooperativeFuzzyGuidance3D(position, velocity, target, ...
        [0; 0; 0], adjacency, params);
    rangeHistory(k, :) = info.ranges;
    etaHistory(k, :) = info.eta;
    xiHistory(k, :) = info.xi;
    coopWeightHistory(k) = info.cooperativeWeight;
    pngWeightHistory(k) = info.pngWeight;
    accelHistory(k, :, :) = reshape(aCmd, 1, 3, vehicleCount);
    for i = 1:vehicleCount
        [~, eNormal, eLateral] = velocityFrameAxes(velocity(:, i));
        normalLoadHistory(k, i) = dot(aCmd(:, i), eNormal) / 9.806;
        lateralLoadHistory(k, i) = dot(aCmd(:, i), eLateral) / 9.806;
    end

    for i = 1:vehicleCount
        if isnan(arrivalTime(i)) && info.ranges(i) <= hitRadius
            arrivalTime(i) = time(k);
            terminalThetaDeg(i) = asind(velocity(3, i) / norm(velocity(:, i)));
            terminalPsiDeg(i) = atan2d(velocity(2, i), velocity(1, i));
        end
    end

    if all(~isnan(arrivalTime))
        time = time(1:k);
        positionHistory = positionHistory(1:k, :, :);
        velocityHistory = velocityHistory(1:k, :, :);
        rangeHistory = rangeHistory(1:k, :);
        etaHistory = etaHistory(1:k, :);
        xiHistory = xiHistory(1:k, :);
        coopWeightHistory = coopWeightHistory(1:k);
        pngWeightHistory = pngWeightHistory(1:k);
        accelHistory = accelHistory(1:k, :, :);
        normalLoadHistory = normalLoadHistory(1:k, :);
        lateralLoadHistory = lateralLoadHistory(1:k, :);
        break;
    end

    velocity = velocity + aCmd * dt;
    for i = 1:vehicleCount
        speed = norm(velocity(:, i));
        if speed > 1
            desiredSpeed = initialSpeed(i);
            velocity(:, i) = desiredSpeed * velocity(:, i) / speed;
        end
    end
    position = position + velocity * dt;
end

results = struct( ...
    'description', 'Fuzzy cooperative guidance demo based on Zhao et al. 2025 chapter 3', ...
    'timeS', time, ...
    'positionM', positionHistory, ...
    'velocityMps', velocityHistory, ...
    'accelerationMps2', accelHistory, ...
    'normalLoadG', normalLoadHistory, ...
    'lateralLoadG', lateralLoadHistory, ...
    'rangeM', rangeHistory, ...
    'eta', etaHistory, ...
    'xi', xiHistory, ...
    'cooperativeWeight', coopWeightHistory, ...
    'pngWeight', pngWeightHistory, ...
    'arrivalTimeS', arrivalTime, ...
    'arrivalTimeStdS', std(arrivalTime, 'omitnan'), ...
    'desiredImpactThetaDeg', impactThetaDeg, ...
    'desiredImpactPsiDeg', impactPsiDeg, ...
    'terminalThetaDeg', terminalThetaDeg, ...
    'terminalPsiDeg', terminalPsiDeg, ...
    'targetPositionM', target, ...
    'initialPositionM', initialPosition, ...
    'initialVelocityMps', initialVelocity, ...
    'initialSpeedMps', initialSpeed, ...
    'adjacency', adjacency, ...
    'params', params);

assignin('base', 'COOPERATIVE_GUIDANCE_RESULTS', results);
save('cooperative_guidance_results.mat', 'results');
plotCooperativeGuidanceResults(results);

fprintf('Arrival times [s]: %s\n', mat2str(arrivalTime, 4));
fprintf('Arrival-time standard deviation: %.4f s\n', results.arrivalTimeStdS);
end

function direction = directionFromFlightPathAngles(thetaDeg, psiDeg)
theta = deg2rad(thetaDeg);
psi = deg2rad(psiDeg);
direction = [cos(theta) * cos(psi); cos(theta) * sin(psi); sin(theta)];
direction = direction / norm(direction);
end

function plotCooperativeGuidanceResults(results)
figure('Color', 'w', 'Name', 'Cooperative guidance trajectories');
axesHandle = axes;
hold(axesHandle, 'on');
grid(axesHandle, 'on');
box(axesHandle, 'on');
vehicleCount = size(results.positionM, 3);
for i = 1:vehicleCount
    trajectory = squeeze(results.positionM(:, :, i));
    plot3(axesHandle, trajectory(:, 1), trajectory(:, 2), trajectory(:, 3), ...
        'LineWidth', 1.4, 'DisplayName', sprintf('Vehicle %d', i));
end
plot3(axesHandle, results.targetPositionM(1), results.targetPositionM(2), ...
    results.targetPositionM(3), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'k', ...
    'DisplayName', 'Target');
xlabel(axesHandle, 'X [m]');
ylabel(axesHandle, 'Y [m]');
zlabel(axesHandle, 'H [m]');
set(axesHandle, 'ZDir', 'normal');
legend(axesHandle, 'Location', 'best');
view(axesHandle, 35, 24);

figure('Color', 'w', 'Name', 'Cooperative guidance states');
tiledlayout(3, 1);
nexttile;
plot(results.timeS, results.rangeM, 'LineWidth', 1.2);
grid on;
ylabel('Range [m]');
nexttile;
plot(results.timeS, results.eta, 'LineWidth', 1.2);
grid on;
ylabel('\eta = r/V [s]');
nexttile;
plot(results.timeS, [results.cooperativeWeight, results.pngWeight], 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Weight');
legend('Cooperative', 'PNG', 'Location', 'best');

figure('Color', 'w', 'Name', 'Cooperative guidance overload commands');
tiledlayout(2, 1);
nexttile;
plot(results.timeS, results.normalLoadG, 'LineWidth', 1.2);
grid on;
ylabel('Normal load [g]');
legend(vehicleLegend(vehicleCount), 'Location', 'best');
nexttile;
plot(results.timeS, results.lateralLoadG, 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Lateral load [g]');
legend(vehicleLegend(vehicleCount), 'Location', 'best');
end

function labels = vehicleLegend(vehicleCount)
labels = strings(1, vehicleCount);
for i = 1:vehicleCount
    labels(i) = sprintf('Vehicle %d', i);
end
end

function [eForward, eNormal, eLateral] = velocityFrameAxes(v)
speed = norm(v);
if speed < 1e-9
    eForward = [1; 0; 0];
else
    eForward = v / speed;
end

inertialUp = [0; 0; 1];
eLateral = cross(inertialUp, eForward);
if norm(eLateral) < 1e-9
    inertialRight = [0; 1; 0];
    eLateral = cross(inertialRight, eForward);
end
eLateral = eLateral / norm(eLateral);
eNormal = cross(eForward, eLateral);
eNormal = eNormal / norm(eNormal);
end
