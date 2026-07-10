function [aCmd, info] = cooperativeFuzzyGuidance3D(rM, vM, rT, vT, adjacency, params)
%COOPERATIVEFUZZYGUIDANCE3D Fuzzy cooperative 3-D guidance for a vehicle group.
%
% This implements the chapter-3 structure in Zhao et al. (2025):
% auxiliary states eta = r/V and xi = -cos(delta), distributed consensus
% on the auxiliary states, and fuzzy blending between cooperative guidance
% and proportional navigation guidance (PNG). The command has no axial
% component, so it does not actively control speed.
%
% Inputs:
%   rM, vM      3-by-n vehicle position and velocity matrices [m], [m/s]
%   rT, vT      3-by-1 or 3-by-n target position and velocity [m], [m/s]
%   adjacency   n-by-n communication adjacency matrix
%   params      optional struct, see defaultCooperativeParams below
%
% Outputs:
%   aCmd        3-by-n inertial acceleration command [m/s^2]
%   info        struct containing PNG/cooperative components, fuzzy weights,
%               auxiliary states, estimated consensus errors and ranges

arguments
    rM (3,:) double
    vM (3,:) double
    rT double
    vT double = [0; 0; 0]
    adjacency double = []
    params struct = struct()
end

vehicleCount = size(rM, 2);
if isempty(adjacency)
    adjacency = ones(vehicleCount) - eye(vehicleCount);
end
validateattributes(adjacency, {'double'}, {'size', [vehicleCount vehicleCount]});

params = fillCooperativeParams(params);
[rT, vT] = expandTargetState(rT, vT, vehicleCount);

aPng = zeros(3, vehicleCount);
aCoop = zeros(3, vehicleCount);
aEta = zeros(3, vehicleCount);
aImpact = zeros(3, vehicleCount);
aCmd = zeros(3, vehicleCount);
localCooperativeWeight = zeros(1, vehicleCount);
ranges = zeros(1, vehicleCount);
closingSpeeds = zeros(1, vehicleCount);
eta = zeros(1, vehicleCount);
xi = zeros(1, vehicleCount);
xiRef = zeros(1, vehicleCount);
uHat = zeros(1, vehicleCount);
xiDotCmd = zeros(1, vehicleCount);
delta = zeros(1, vehicleCount);
lambda = zeros(3, vehicleCount);

for i = 1:vehicleCount
    relativePosition = rT(:, i) - rM(:, i);
    range = norm(relativePosition);
    ranges(i) = range;
    speed = norm(vM(:, i));

    if range < params.minRange || speed < params.minSpeed
        lambda(:, i) = [1; 0; 0];
        eta(i) = 0;
        xi(i) = -1;
        continue;
    end

    lambda(:, i) = relativePosition / range;
    velocityUnit = vM(:, i) / speed;
    cosDelta = clamp(dot(velocityUnit, lambda(:, i)), -1, 1);
    delta(i) = acos(cosDelta);
    xi(i) = -cosDelta;
    eta(i) = range / speed;

    relativeVelocity = vT(:, i) - vM(:, i);
    closingSpeeds(i) = -dot(relativePosition, relativeVelocity) / range;
end

laplacian = diag(sum(adjacency, 2)) - adjacency;
eEta = (laplacian * eta(:)).';
eXi = (laplacian * xi(:)).';
cooperativeState = params.betaEta * mean(abs(eta - mean(eta))) + ...
    params.betaXi * mean(abs(xi - mean(xi)));
averageRange = mean(ranges);
cooperativeWeight = fuzzyCooperativeWeight(cooperativeState, averageRange, params);
pngWeight = 1 - cooperativeWeight;

for i = 1:vehicleCount
    if ranges(i) < params.minRange || norm(vM(:, i)) < params.minSpeed
        continue;
    end

    aPng(:, i) = pngAcceleration(rM(:, i), vM(:, i), rT(:, i), vT(:, i), ...
        params.navigationConstant, params.minRange);

    xiRef(i) = params.xiCenter - params.xiHalfWidth * ...
        tanh(params.consensusGain * (eEta(i) + eXi(i)));
    uHat(i) = -params.consensusGain * (xi(i) - xiRef(i));
    % In (13), uHat is the desired derivative of xi.  It must not be
    % divided by speed before the acceleration inversion.
    xiDotCmd(i) = uHat(i);
    aCoop(:, i) = cooperativeAcceleration(rM(:, i), vM(:, i), rT(:, i), ...
        vT(:, i), lambda(:, i), xiDotCmd(i), params);
    % Zhao et al. use one group-level fuzzy weight for every vehicle.
    % PNG becomes the sole law only when Dbar <= Ra (handled above).
    localCooperativeWeight(i) = cooperativeWeight;
    localPngWeight = 1 - localCooperativeWeight(i);

    aBlend = localCooperativeWeight(i) * aCoop(:, i) + localPngWeight * aPng(:, i);
    aCmd(:, i) = removeAxialComponent(aBlend, vM(:, i), params.minSpeed);
    aCmd(:, i) = limitVectorMagnitude(aCmd(:, i), params.maxAcceleration);
end

info = struct( ...
    'aPng', aPng, ...
    'aCooperative', aCoop, ...
    'aEtaCoordination', aEta, ...
    'aImpactAngle', aImpact, ...
    'cooperativeWeight', cooperativeWeight, ...
    'localCooperativeWeight', localCooperativeWeight, ...
    'pngWeight', pngWeight, ...
    'localPngWeight', 1 - localCooperativeWeight, ...
    'eta', eta, ...
    'xi', xi, ...
    'xiRef', xiRef, ...
    'uHat', uHat, ...
    'xiDotCmd', xiDotCmd, ...
    'deltaRad', delta, ...
    'ranges', ranges, ...
    'closingSpeeds', closingSpeeds, ...
    'cooperativeState', cooperativeState, ...
    'averageRange', averageRange, ...
    'eEta', eEta, ...
    'eXi', eXi);
end

function params = fillCooperativeParams(params)
defaults = defaultCooperativeParams();
fields = fieldnames(defaults);
for k = 1:numel(fields)
    fieldName = fields{k};
    if ~isfield(params, fieldName) || isempty(params.(fieldName))
        params.(fieldName) = defaults.(fieldName);
    end
end

deltaMin = params.fovMinRad;
deltaMax = params.fovMaxRad;
xiMin = -cos(deltaMin);
xiMax = -cos(deltaMax);
if xiMin > xiMax
    temp = xiMin;
    xiMin = xiMax;
    xiMax = temp;
end
params.xiMin = xiMin;
params.xiMax = xiMax;
params.xiCenter = 0.5 * (xiMin + xiMax);
params.xiHalfWidth = 0.5 * (xiMax - xiMin);
end

function params = defaultCooperativeParams()
params = struct( ...
    'navigationConstant', 4.0, ...
    'consensusGain', 0.35, ...
    'fovMinRad', deg2rad(0), ...
    'fovMaxRad', deg2rad(80), ...
    'maxAcceleration', 8 * 9.806, ...
    'terminalPngRange', 3000, ...
    'impactAngleGain', 0, ...
    'impactAngleRange', 20000, ...
    'impactThetaPsiGain', [0 0], ...
    'approachLineGain', 0, ...
    'maxApproachLineOffset', 0, ...
    'fuzzyFarRange', 30000, ...
    'cooperativeScale', 3.0, ...
    'betaEta', 1.0, ...
    'betaXi', 20.0, ...
    'gaussianSigma', 0.45, ...
    'timeCoordinationGain', 0, ...
    'timeCoordinationTurnSign', 1, ...
    'timeWaypointGain', 0, ...
    'maxTimeWaypointOffset', 0, ...
    'cooperativeAimpointGain', 0, ...
    'maxCooperativeAimpointOffset', 0, ...
    'minRange', 1.0, ...
    'minSpeed', 1.0, ...
    'impactDirection', [], ...
    'aimpointFraction', 0.02, ...
    'maxAimpointSetback', 1000);
end

function [rT, vT] = expandTargetState(rT, vT, vehicleCount)
if isvector(rT)
    rT = rT(:);
end
if isvector(vT)
    vT = vT(:);
end
if size(rT, 2) == 1
    rT = repmat(rT, 1, vehicleCount);
end
if size(vT, 2) == 1
    vT = repmat(vT, 1, vehicleCount);
end
validateattributes(rT, {'double'}, {'size', [3 vehicleCount]});
validateattributes(vT, {'double'}, {'size', [3 vehicleCount]});
end

function aPng = pngAcceleration(rM, vM, rT, vT, navigationConstant, minRange)
relativePosition = rT - rM;
relativeVelocity = vT - vM;
range = norm(relativePosition);
if range < minRange
    aPng = [0; 0; 0];
    return;
end

lambda = relativePosition / range;
closingSpeed = -dot(relativePosition, relativeVelocity) / range;
omegaLos = cross(relativePosition, relativeVelocity) / range^2;
if closingSpeed > 0
    aPng = navigationConstant * closingSpeed * cross(omegaLos, lambda);
else
    aPng = [0; 0; 0];
end
end

function aCoop = cooperativeAcceleration(rM, vM, rT, vT, lambda, uHat, params)
speed = norm(vM);
if speed < params.minSpeed
    aCoop = [0; 0; 0];
    return;
end

velocityUnit = vM / speed;
relativePosition = rT - rM;
range = norm(relativePosition);
relativeVelocity = vT - vM;
lambdaDot = (relativeVelocity - dot(relativeVelocity, lambda) * lambda) / ...
    max(range, params.minRange);

lambdaPerp = lambda - dot(lambda, velocityUnit) * velocityUnit;
denominator = dot(lambdaPerp, lambdaPerp);
if denominator < 1e-8
    aCoop = [0; 0; 0];
    return;
end

% From xi = -vHat' * lambda and Eq. (13):
% a' * lambdaPerp = -V * (uHat + vHat' * lambdaDot).
rhs = -speed * (uHat + dot(velocityUnit, lambdaDot));
aCoop = rhs * lambdaPerp / denominator;
aCoop = removeAxialComponent(aCoop, vM, params.minSpeed);
end

function aTime = etaCoordinationAcceleration(vM, lambda, etaError, params)
speed = norm(vM);
if speed < params.minSpeed || params.timeCoordinationGain <= 0
    aTime = [0; 0; 0];
    return;
end

velocityUnit = vM / speed;
lambdaPerp = lambda - dot(lambda, velocityUnit) * velocityUnit;
lambdaPerpNorm = norm(lambdaPerp);
if etaError < 0
    if lambdaPerpNorm < 1e-8
        turnAxis = stablePerpendicularAxis(velocityUnit);
    else
        turnAxis = -lambdaPerp / lambdaPerpNorm;
    end
    turnAxis = params.timeCoordinationTurnSign * turnAxis;
    aTime = params.timeCoordinationGain * abs(etaError) * turnAxis;
elseif lambdaPerpNorm >= 1e-8
    aTime = params.timeCoordinationGain * etaError * lambdaPerp / lambdaPerpNorm;
else
    aTime = [0; 0; 0];
end

aTime = removeAxialComponent(aTime, vM, params.minSpeed);
end

function aImpact = impactAngleAlignmentAcceleration(vM, range, vehicleIndex, params)
aImpact = [0; 0; 0];
if params.impactAngleGain <= 0 || isempty(params.impactDirection)
    aImpact = thetaPsiAlignmentAcceleration(vM, range, vehicleIndex, params);
    return;
end
if range >= params.impactAngleRange
    return;
end

speed = norm(vM);
if speed < params.minSpeed
    return;
end

impactDirection = params.impactDirection;
if isvector(impactDirection)
    desiredDirection = impactDirection(:);
else
    desiredDirection = impactDirection(:, vehicleIndex);
end
desiredNorm = norm(desiredDirection);
if desiredNorm < 1e-9
    return;
end

velocityUnit = vM / speed;
desiredDirection = desiredDirection / desiredNorm;
directionError = desiredDirection - dot(desiredDirection, velocityUnit) * velocityUnit;
fade = 1 - clamp(range / params.impactAngleRange, 0, 1);
fade = fade * fade * (3 - 2 * fade);
aImpact = params.impactAngleGain * speed * fade * directionError;
aImpact = removeAxialComponent(aImpact, vM, params.minSpeed);
aImpact = aImpact + thetaPsiAlignmentAcceleration(vM, range, vehicleIndex, params);
end

function aAngle = thetaPsiAlignmentAcceleration(vM, range, vehicleIndex, params)
aAngle = [0; 0; 0];
if isempty(params.impactDirection) || all(params.impactThetaPsiGain == 0)
    return;
end
if range >= params.impactAngleRange
    return;
end

speed = norm(vM);
if speed < params.minSpeed
    return;
end

impactDirection = params.impactDirection;
if isvector(impactDirection)
    desiredDirection = impactDirection(:);
else
    desiredDirection = impactDirection(:, vehicleIndex);
end
desiredDirection = desiredDirection / max(norm(desiredDirection), eps);

theta = asin(clamp(vM(3) / speed, -1, 1));
psi = atan2(vM(2), vM(1));
thetaDesired = asin(clamp(desiredDirection(3), -1, 1));
psiDesired = atan2(desiredDirection(2), desiredDirection(1));

thetaError = wrapToPiLocal(thetaDesired - theta);
psiError = wrapToPiLocal(psiDesired - psi);
gain = params.impactThetaPsiGain(:);
if numel(gain) == 1
    gain = [gain; gain];
end

[~, eNormal, eLateral] = velocityFrameAxesLocal(vM, params.minSpeed);
fade = 1 - clamp(range / params.impactAngleRange, 0, 1);
fade = fade * fade * (3 - 2 * fade);
accelerationScale = speed^2 / max(range, params.minRange);
aAngle = fade * accelerationScale * (gain(1) * thetaError * eNormal + ...
    gain(2) * cos(theta) * psiError * eLateral);
aAngle = removeAxialComponent(aAngle, vM, params.minSpeed);
end

function [eForward, eNormal, eLateral] = velocityFrameAxesLocal(v, minSpeed)
speed = norm(v);
if speed < minSpeed
    eForward = [1; 0; 0];
else
    eForward = v / speed;
end
eLateral = cross([0; 0; 1], eForward);
if norm(eLateral) < 1e-9
    eLateral = cross([0; 1; 0], eForward);
end
eLateral = eLateral / norm(eLateral);
eNormal = cross(eForward, eLateral);
eNormal = eNormal / norm(eNormal);
end

function angle = wrapToPiLocal(angle)
angle = mod(angle + pi, 2*pi) - pi;
end

function target = cooperativeAimpoint(trueTarget, vM, lambda, xiReference, range, params)
target = trueTarget;
if params.cooperativeAimpointGain <= 0 || params.maxCooperativeAimpointOffset <= 0
    return;
end

velocityUnit = vM / max(norm(vM), params.minSpeed);
desiredCosDelta = clamp(-xiReference, cos(params.fovMaxRad), cos(params.fovMinRad));
desiredDelta = acos(desiredCosDelta);
currentCosDelta = clamp(dot(velocityUnit, lambda), -1, 1);
currentDelta = acos(currentCosDelta);
deltaCommand = max(0, desiredDelta - currentDelta);
if deltaCommand <= 1e-6
    return;
end

offsetAxis = stablePerpendicularAxis(velocityUnit);
offsetMagnitude = params.cooperativeAimpointGain * range * tan(deltaCommand);
offsetMagnitude = min(offsetMagnitude, params.maxCooperativeAimpointOffset);
target = trueTarget + offsetMagnitude * offsetAxis;
end

function axis = stablePerpendicularAxis(unitVector)
reference = [0; 0; 1];
axis = cross(reference, unitVector);
if norm(axis) < 1e-8
    reference = [0; 1; 0];
    axis = cross(reference, unitVector);
end
axis = axis / norm(axis);
end

function target = terminalAimpoint(trueTarget, range, vehicleIndex, params)
target = trueTarget;
if isempty(params.impactDirection)
    return;
end

impactDirection = params.impactDirection;
if isvector(impactDirection)
    desiredDirection = impactDirection(:);
else
    desiredDirection = impactDirection(:, vehicleIndex);
end
directionNorm = norm(desiredDirection);
if directionNorm < 1e-9
    return;
end

desiredDirection = desiredDirection / directionNorm;
aimpointFraction = scalarOrIndexed(params.aimpointFraction, vehicleIndex);
maxAimpointSetback = scalarOrIndexed(params.maxAimpointSetback, vehicleIndex);
setback = min(maxAimpointSetback, aimpointFraction * range);
target = trueTarget + setback * desiredDirection;
approachLineGain = scalarOrIndexed(params.approachLineGain, vehicleIndex);
maxApproachLineOffset = scalarOrIndexed(params.maxApproachLineOffset, vehicleIndex);
if approachLineGain > 0 && maxApproachLineOffset > 0
    lineOffset = min(maxApproachLineOffset, approachLineGain * range);
    target = trueTarget - lineOffset * desiredDirection;
end
end

function value = scalarOrIndexed(value, index)
if ~isscalar(value)
    value = value(index);
end
end

function target = timeCoordinationAimpoint(target, vM, lambda, etaError, range, params)
if params.timeWaypointGain <= 0 || params.maxTimeWaypointOffset <= 0
    return;
end
if etaError >= 0 || range <= params.terminalPngRange
    return;
end

velocityUnit = vM / max(norm(vM), params.minSpeed);
offsetAxis = stablePerpendicularAxis(velocityUnit);
secondaryAxis = cross(velocityUnit, offsetAxis);
if norm(secondaryAxis) > 1e-8
    secondaryAxis = secondaryAxis / norm(secondaryAxis);
    if dot(secondaryAxis, lambda) > 0
        offsetAxis = secondaryAxis;
    end
end

offsetMagnitude = min(params.maxTimeWaypointOffset, ...
    params.timeWaypointGain * abs(etaError) * range);
target = target + offsetMagnitude * offsetAxis;
end

function weight = fuzzyCooperativeWeight(cooperativeState, averageRange, params)
if averageRange <= params.terminalPngRange
    weight = 0;
    return;
end

cooInput = 2 * clamp(cooperativeState / params.cooperativeScale, 0, 1) - 1;
rangeInput = 2 * clamp((averageRange - params.terminalPngRange) / ...
    max(params.fuzzyFarRange - params.terminalPngRange, eps), 0, 1) - 1;
centers = [-1, 0, 1];
cooMembership = gaussianMembership(cooInput, centers, params.gaussianSigma);
rangeMembership = gaussianMembership(rangeInput, centers, params.gaussianSigma);

% Table 1 in Zhao et al. has Dbar on its rows and Coo on its columns.
% The output sets map N -> PNG-dominant, ZO -> balanced, P -> cooperative-dominant.
ruleLevels = [-1 -1 -1; -1 0 1; -1 0 1];
outputUniverse = linspace(-1, 1, 401);
aggregatedOutput = zeros(size(outputUniverse));
for row = 1:3
    for col = 1:3
        activation = min(rangeMembership(row), cooMembership(col));
        consequent = gaussianMembership(outputUniverse, ruleLevels(row, col), ...
            params.gaussianSigma);
        aggregatedOutput = max(aggregatedOutput, min(activation, consequent));
    end
end

outputArea = trapz(outputUniverse, aggregatedOutput);
if outputArea <= eps
    fuzzyOutput = -1;
else
    % Mamdani inference with centroid defuzzification, as specified in Sec. 3.2.
    fuzzyOutput = trapz(outputUniverse, outputUniverse .* aggregatedOutput) / outputArea;
end
weight = clamp(0.5 * (fuzzyOutput + 1), 0, 1);
end

function y = gaussianMembership(x, centers, sigma)
y = exp(-((x - centers) .^ 2) / (2 * sigma^2));
end

function a = removeAxialComponent(a, v, minSpeed)
speed = norm(v);
if speed < minSpeed
    return;
end
velocityUnit = v / speed;
a = a - dot(a, velocityUnit) * velocityUnit;
end

function y = limitVectorMagnitude(x, maxMagnitude)
xNorm = norm(x);
if xNorm > maxMagnitude && xNorm > 0
    y = x * (maxMagnitude / xNorm);
else
    y = x;
end
end

function y = clamp(x, lowerBound, upperBound)
y = min(max(x, lowerBound), upperBound);
end
