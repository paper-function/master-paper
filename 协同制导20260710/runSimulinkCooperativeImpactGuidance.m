function results = runSimulinkCooperativeImpactGuidance(options)
%RUNSIMULINKCOOPERATIVEIMPACTGUIDANCE Legacy handoff experiment for the HRV model.
%
% This file is retained only for diagnosing a single-HRV terminal model.
% It is NOT a paper-faithful Zhao et al. (2025) simulation because its
% point-mass/Simulink handoff prevents the group fuzzy blend from acting
% continuously. Use cooperativeFuzzyGuidance3D or cooperativePnOverloadCommand3D
% as the controller core when all vehicle states are available at every step.

arguments
    options.ConfigureTerminalGuidance (1, 1) logical = true
    options.RunTerminalSimulink (1, 1) logical = true
    options.ProjectHandoffToTerminalCorridor (1, 1) logical = false
    options.SaveFolder (1, 1) string = "simulink_cooperative_results"
end

if options.ConfigureTerminalGuidance
    configureSimulinkTerminalGuidance();
end
if ~isfolder(options.SaveFolder)
    mkdir(options.SaveFolder);
end
delete(fullfile(options.SaveFolder, "vehicle_*_flight_data.mat"));

target = [25000; 7500; 0];
vehicleCount = 5;
dt = 0.02;
maxCooperativeTime = 80;
handoffRange = 48000;

thetaDesiredDeg = [-39, -42, -45, -48, -51];
psiDesiredDeg = [9, 13, 17, 21, 25];

initialSpeed = [2700, 2550, 2820, 2460, 2650];
initialRange = [86000, 83000, 90000, 82000, 88000];

impactDirection = zeros(3, vehicleCount);
initialVelocity = zeros(3, vehicleCount);
for i = 1:vehicleCount
    impactDirection(:, i) = directionFromFlightPathAngles( ...
        thetaDesiredDeg(i), psiDesiredDeg(i));
end

initialPosition = zeros(3, vehicleCount);
for i = 1:vehicleCount
    initialPosition(:, i) = target - initialRange(i) * impactDirection(:, i);
    velocityDirection = directionFromFlightPathAngles( ...
        thetaDesiredDeg(i) + 5, psiDesiredDeg(i) - 3);
    initialVelocity(:, i) = initialSpeed(i) * velocityDirection;
end

adjacency = ones(vehicleCount) - eye(vehicleCount);
cooperativeParams = struct( ...
    "navigationConstant", 4.0, ...
    "consensusGain", 0.006, ...
    "fovMaxRad", deg2rad(85), ...
    "maxAcceleration", 20 * 9.806, ...
    "terminalPngRange", handoffRange, ...
    "terminalBlendRange", 6000, ...
    "fuzzyFarRange", 100000, ...
    "cooperativeScale", 8.0, ...
    "impactDirection", impactDirection, ...
    "aimpointFraction", 0.0015 * ones(1, vehicleCount), ...
    "maxAimpointSetback", 120 * ones(1, vehicleCount), ...
    "timeCoordinationGain", 0, ...
    "timeCoordinationTurnSign", 1, ...
    "timeWaypointGain", 0, ...
    "maxTimeWaypointOffset", 0, ...
    "cooperativeAimpointGain", 0, ...
    "maxCooperativeAimpointOffset", 0);

cooperative = simulateCooperativeHandoff(initialPosition, initialVelocity, ...
    target, adjacency, cooperativeParams, dt, maxCooperativeTime, handoffRange);
if options.ProjectHandoffToTerminalCorridor
    cooperative.rawHandoffState = cooperative.handoffState;
    cooperative.handoffState = projectHandoffToTerminalCorridor( ...
        cooperative.handoffState, target, thetaDesiredDeg, psiDesiredDeg);
end

terminal = repmat(emptyTerminalSummary(), vehicleCount, 1);
terminalData = cell(1, vehicleCount);
if options.RunTerminalSimulink
    for i = 1:vehicleCount
        [terminal(i), terminalData{i}] = runTerminalVehicle( ...
            i, cooperative.handoffState(i), thetaDesiredDeg(i), ...
            psiDesiredDeg(i), target, options.SaveFolder);
    end
    plotSimulinkCooperativeResults(cooperative, terminal, terminalData, ...
        options.SaveFolder);
end

if options.RunTerminalSimulink
    arrivalTime = [terminal.closestTimeS];
else
    arrivalTime = cooperative.handoffTimeS;
end
timeSpread = max(arrivalTime) - min(arrivalTime);
results = struct( ...
    "description", "Legacy point-mass/HRV handoff diagnostic; not a continuous fuzzy-blend simulation", ...
    "dtS", dt, ...
    "targetPositionM", target, ...
    "desiredThetaDeg", thetaDesiredDeg, ...
    "desiredPsiDeg", psiDesiredDeg, ...
    "cooperative", cooperative, ...
    "terminal", terminal, ...
    "terminalFlightData", {terminalData}, ...
    "arrivalOrHandoffTimeS", arrivalTime, ...
    "arrivalOrHandoffTimeSpreadS", timeSpread);

save(fullfile(options.SaveFolder, "simulink_cooperative_summary.mat"), "results");
assignin("base", "SIMULINK_COOPERATIVE_RESULTS", results);

fprintf("\nSimulink cooperative terminal results:\n");
if options.RunTerminalSimulink
    for i = 1:vehicleCount
        fprintf("Vehicle %d: t = %.3f s, miss = %.3f m, theta = %.3f deg, psi = %.3f deg, ny/nz peak = %.2f/%.2f g\n", ...
            i, terminal(i).closestTimeS, terminal(i).missDistanceM, ...
            terminal(i).terminalThetaDeg, terminal(i).terminalPsiDeg, ...
            terminal(i).maxAbsNormalLoadG, terminal(i).maxAbsLateralLoadG);
    end
else
    for i = 1:vehicleCount
        fprintf("Vehicle %d enters terminal handoff at %.3f s\n", ...
            i, cooperative.handoffTimeS(i));
    end
end
fprintf("Arrival-time spread: %.3f s\n", timeSpread);
end

function cooperative = simulateCooperativeHandoff(position, velocity, target, ...
    adjacency, params, dt, maxTime, handoffRange)
vehicleCount = size(position, 2);
stepCount = floor(maxTime/dt) + 1;
time = (0:stepCount-1).' * dt;
positionHistory = nan(stepCount, 3, vehicleCount);
velocityHistory = nan(stepCount, 3, vehicleCount);
rangeHistory = nan(stepCount, vehicleCount);
etaHistory = nan(stepCount, vehicleCount);
xiHistory = nan(stepCount, vehicleCount);
normalLoadHistory = nan(stepCount, vehicleCount);
lateralLoadHistory = nan(stepCount, vehicleCount);
handoffTime = nan(1, vehicleCount);
handoffState = repmat(emptyHandoffState(), vehicleCount, 1);

for k = 1:stepCount
    positionHistory(k, :, :) = reshape(position, 1, 3, vehicleCount);
    velocityHistory(k, :, :) = reshape(velocity, 1, 3, vehicleCount);

    [aCmd, info] = cooperativeFuzzyGuidance3D(position, velocity, target, ...
        [0; 0; 0], adjacency, params);
    rangeHistory(k, :) = info.ranges;
    etaHistory(k, :) = info.eta;
    xiHistory(k, :) = info.xi;

    for i = 1:vehicleCount
        [~, eNormal, eLateral] = velocityFrameAxes(velocity(:, i));
        normalLoadHistory(k, i) = dot(aCmd(:, i), eNormal) / 9.806;
        lateralLoadHistory(k, i) = dot(aCmd(:, i), eLateral) / 9.806;
        if isnan(handoffTime(i)) && info.ranges(i) <= handoffRange
            handoffTime(i) = time(k);
            handoffState(i) = makeHandoffState(position(:, i), velocity(:, i));
            handoffState(i).cooperativeTimeS = handoffTime(i);
        end
    end

    if all(~isnan(handoffTime))
        time = time(1:k);
        positionHistory = positionHistory(1:k, :, :);
        velocityHistory = velocityHistory(1:k, :, :);
        rangeHistory = rangeHistory(1:k, :);
        etaHistory = etaHistory(1:k, :);
        xiHistory = xiHistory(1:k, :);
        normalLoadHistory = normalLoadHistory(1:k, :);
        lateralLoadHistory = lateralLoadHistory(1:k, :);
        break;
    end

    velocity = velocity + aCmd * dt;
    for i = 1:vehicleCount
        speed = norm(velocity(:, i));
        if speed > 1
            velocity(:, i) = speed * velocity(:, i) / speed;
        end
    end
    position = position + velocity * dt;
end

for i = 1:vehicleCount
    if isnan(handoffTime(i))
        handoffTime(i) = time(end);
        handoffState(i) = makeHandoffState(position(:, i), velocity(:, i));
        handoffState(i).cooperativeTimeS = handoffTime(i);
    end
end

cooperative = struct( ...
    "timeS", time, ...
    "handoffRangeM", handoffRange, ...
    "handoffTimeS", handoffTime, ...
    "handoffState", handoffState, ...
    "positionM", positionHistory, ...
    "velocityMps", velocityHistory, ...
    "rangeM", rangeHistory, ...
    "eta", etaHistory, ...
    "xi", xiHistory, ...
    "normalLoadG", normalLoadHistory, ...
    "lateralLoadG", lateralLoadHistory, ...
    "maxAbsNormalLoadG", max(abs(normalLoadHistory), [], 1, "omitnan"), ...
    "maxAbsLateralLoadG", max(abs(lateralLoadHistory), [], 1, "omitnan"));
end

function [summary, flightData] = runTerminalVehicle(vehicleIndex, handoffState, ...
    thetaDesiredDeg, psiDesiredDeg, target, outputFolder)
modelName = "Ctrl_For_HRV_20260625";
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
clear functions;
load_system(modelName);
set_param(modelName + "/Constant3", "Value", num2str(thetaDesiredDeg, "%.6g"));
set_param(modelName + "/Constant4", "Value", num2str(psiDesiredDeg, "%.6g"));
assignin("base", "HRV_INIT", handoffState);

simOut = sim(modelName, StopTime="55", ReturnWorkspaceOutputs="on");
[summary, flightData] = evaluateTerminalSimulation(simOut, handoffState, ...
    thetaDesiredDeg, psiDesiredDeg, target);
summary.vehicleIndex = vehicleIndex;

tag = sprintf("vehicle_%02d_theta_%+05.1f_psi_%+04.1f", ...
    vehicleIndex, thetaDesiredDeg, psiDesiredDeg);
save(fullfile(outputFolder, tag + "_flight_data.mat"), "flightData");
evalin("base", "clear HRV_INIT");
end

function handoffState = projectHandoffToTerminalCorridor(handoffState, target, ...
    thetaDesiredDeg, psiDesiredDeg)
vehicleCount = numel(handoffState);
speed = [handoffState.V0];
handoffTime = [handoffState.cooperativeTimeS];
nominalRange = 30000;
nominalArrival = mean(handoffTime + nominalRange ./ speed);
for i = 1:vehicleCount
    terminalRange = (nominalArrival - handoffTime(i)) * speed(i);
    terminalRange = min(max(terminalRange, 24000), 36000);
    thetaEntryDeg = thetaDesiredDeg(i) + 8;
    psiEntryDeg = psiDesiredDeg(i) - 3;
    entryDirection = directionFromFlightPathAngles(thetaEntryDeg, psiEntryDeg);
    position = target(:) - terminalRange * entryDirection;
    handoffState(i).H0 = max(position(3), 1000);
    handoffState(i).X0 = position(1);
    handoffState(i).Y0 = position(2);
    handoffState(i).theta0 = deg2rad(thetaEntryDeg);
    handoffState(i).psi0 = deg2rad(psiEntryDeg);
end
end

function [summary, flightData] = evaluateTerminalSimulation(simOut, handoffState, ...
    thetaDesiredDeg, psiDesiredDeg, target)
time = simOut.Scope_X(:, 1);
position = [simOut.Scope_X(:, 2), simOut.Scope_Y(:, 2), simOut.Scope_H(:, 2)];
[missDistance, closestTime] = segmentMissDistance(time, position, target(:).');

thetaDeg = simOut.Scope_theta(:, 2);
psiDeg = simOut.Scope_psi(:, 2);
normalLoadSignal = simOut.logsout.get("ny_cmd_log").Values;
lateralLoadSignal = simOut.logsout.get("nz_cmd_log").Values;
terminalTheta = interp1(time, thetaDeg, closestTime, "linear", "extrap");
terminalPsi = interp1(time, psiDeg, closestTime, "linear", "extrap");

summary = emptyTerminalSummary();
summary.thetaDesiredDeg = thetaDesiredDeg;
summary.psiDesiredDeg = psiDesiredDeg;
summary.handoffTimeS = handoffState.cooperativeTimeS;
summary.handoffRangeM = norm([handoffState.X0; handoffState.Y0; handoffState.H0] - target(:));
summary.handoffSpeedMps = handoffState.V0;
summary.handoffThetaDeg = handoffState.theta0 * 180/pi;
summary.handoffPsiDeg = handoffState.psi0 * 180/pi;
summary.missDistanceM = missDistance;
summary.closestTimeS = handoffState.cooperativeTimeS + closestTime;
summary.terminalThetaDeg = terminalTheta;
summary.terminalPsiDeg = terminalPsi;
summary.thetaErrorDeg = terminalTheta - thetaDesiredDeg;
summary.psiErrorDeg = wrapTo180(terminalPsi - psiDesiredDeg);
summary.maxAbsNormalLoadG = max(abs(normalLoadSignal.Data));
summary.maxAbsLateralLoadG = max(abs(lateralLoadSignal.Data));
summary.hit = missDistance <= 10;

flightData = struct( ...
    "timeS", handoffState.cooperativeTimeS + time, ...
    "terminalLocalTimeS", time, ...
    "positionXM", position(:, 1), ...
    "positionYM", position(:, 2), ...
    "altitudeM", position(:, 3), ...
    "speedMps", simOut.Scope_V(:, 2), ...
    "flightPathThetaDeg", thetaDeg, ...
    "flightPathPsiDeg", psiDeg, ...
    "normalLoadTimeS", handoffState.cooperativeTimeS + normalLoadSignal.Time, ...
    "normalLoadCommandG", normalLoadSignal.Data, ...
    "lateralLoadTimeS", handoffState.cooperativeTimeS + lateralLoadSignal.Time, ...
    "lateralLoadCommandG", lateralLoadSignal.Data, ...
    "targetPositionM", target(:).', ...
    "handoffState", handoffState, ...
    "result", summary);
end

function plotSimulinkCooperativeResults(cooperative, terminal, terminalData, outputFolder)
vehicleCount = numel(terminal);
labels = strings(1, vehicleCount);
for i = 1:vehicleCount
    labels(i) = sprintf("Vehicle %d", i);
end

figure("Color", "w", "Name", "Cooperative and terminal trajectories");
hold on; grid on; box on;
for i = 1:vehicleCount
    mid = squeeze(cooperative.positionM(:, :, i));
    plot3(mid(:, 1), mid(:, 2), mid(:, 3), "--", "LineWidth", 1.0);
    if ~isempty(terminalData{i})
        plot3(terminalData{i}.positionXM, terminalData{i}.positionYM, ...
            terminalData{i}.altitudeM, "LineWidth", 1.4);
    end
end
xlabel("X [m]"); ylabel("Y [m]"); zlabel("H [m]");
set(gca, "ZDir", "normal");
view(35, 24);
saveas(gcf, fullfile(outputFolder, "trajectory_3d.png"));

figure("Color", "w", "Name", "Cooperative and terminal overloads");
tiledlayout(2, 1);
nexttile; hold on; grid on;
plot(cooperative.timeS, cooperative.normalLoadG, "LineWidth", 1.0);
for i = 1:vehicleCount
    if ~isempty(terminalData{i})
        plot(terminalData{i}.normalLoadTimeS, ...
            terminalData{i}.normalLoadCommandG, "LineWidth", 1.2);
    end
end
ylabel("Longitudinal/normal load [g]");
legend(labels, "Location", "best");
nexttile; hold on; grid on;
plot(cooperative.timeS, cooperative.lateralLoadG, "LineWidth", 1.0);
for i = 1:vehicleCount
    if ~isempty(terminalData{i})
        plot(terminalData{i}.lateralLoadTimeS, ...
            terminalData{i}.lateralLoadCommandG, "LineWidth", 1.2);
    end
end
xlabel("Time [s]");
ylabel("Lateral load [g]");
saveas(gcf, fullfile(outputFolder, "overload_commands.png"));

figure("Color", "w", "Name", "Arrival time and miss distance");
tiledlayout(2, 1);
nexttile;
bar([terminal.closestTimeS]);
grid on; ylabel("Arrival time [s]");
nexttile;
bar([terminal.missDistanceM]);
grid on; ylabel("Miss distance [m]");
xlabel("Vehicle index");
saveas(gcf, fullfile(outputFolder, "arrival_and_miss.png"));
end

function state = makeHandoffState(position, velocity)
speed = norm(velocity);
state = emptyHandoffState();
state.H0 = max(position(3), 1000);
state.V0 = speed;
state.X0 = position(1);
state.Y0 = position(2);
state.theta0 = asin(max(min(velocity(3)/max(speed, 1), 1), -1));
state.psi0 = atan2(velocity(2), velocity(1));
end

function state = emptyHandoffState()
state = struct("H0", NaN, "V0", NaN, "X0", NaN, "Y0", NaN, ...
    "theta0", NaN, "psi0", NaN, "cooperativeTimeS", NaN);
end

function summary = emptyTerminalSummary()
summary = struct( ...
    "vehicleIndex", NaN, ...
    "thetaDesiredDeg", NaN, "psiDesiredDeg", NaN, ...
    "handoffTimeS", NaN, "handoffRangeM", NaN, ...
    "handoffSpeedMps", NaN, "handoffThetaDeg", NaN, ...
    "handoffPsiDeg", NaN, "missDistanceM", NaN, ...
    "closestTimeS", NaN, "terminalThetaDeg", NaN, ...
    "terminalPsiDeg", NaN, "thetaErrorDeg", NaN, ...
    "psiErrorDeg", NaN, "maxAbsNormalLoadG", NaN, ...
    "maxAbsLateralLoadG", NaN, "hit", false);
end

function direction = directionFromFlightPathAngles(thetaDeg, psiDeg)
theta = deg2rad(thetaDeg);
psi = deg2rad(psiDeg);
direction = [cos(theta)*cos(psi); cos(theta)*sin(psi); sin(theta)];
direction = direction / norm(direction);
end

function [minimumDistance, closestTime] = segmentMissDistance(time, position, target)
minimumDistance = inf;
closestTime = time(1);
for sampleIndex = 1:size(position, 1)-1
    segment = position(sampleIndex+1, :) - position(sampleIndex, :);
    denominator = dot(segment, segment);
    if denominator > 0
        fraction = dot(target-position(sampleIndex, :), segment)/denominator;
        fraction = min(max(fraction, 0), 1);
    else
        fraction = 0;
    end
    candidate = position(sampleIndex, :) + fraction*segment;
    candidateDistance = norm(candidate-target);
    if candidateDistance < minimumDistance
        minimumDistance = candidateDistance;
        closestTime = time(sampleIndex) + ...
            fraction*(time(sampleIndex+1)-time(sampleIndex));
    end
end
end

function angleDeg = wrapTo180(angleDeg)
angleDeg = mod(angleDeg + 180, 360) - 180;
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
