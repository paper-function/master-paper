function [alphaCmd, bankCmd, nNormalCmd, nLateralCmd, aCmd, info] = ...
    cooperativePnOverloadCommand3D(vehicleIndex, rMAll, vMAll, rT, vT, adjacency, ...
    guidanceParams, maxLoadFactor, g0, rho, mass, refArea, CL0, CLalpha, alphaLimit)
%COOPERATIVEPNOVERLOADCOMMAND3D Cooperative guidance converted to alpha/bank.
%
% This wrapper is intended for replacing a single-vehicle PN guidance block
% when the states of all vehicles are available. The cooperative core keeps
% the command perpendicular to velocity, so no active speed-control channel
% is introduced.

arguments
    vehicleIndex (1,1) double {mustBeInteger, mustBePositive}
    rMAll (3,:) double
    vMAll (3,:) double
    rT double
    vT double = [0; 0; 0]
    adjacency double = []
    guidanceParams struct = struct()
    maxLoadFactor (1,1) double = 8
    g0 (1,1) double = 9.806
    rho (1,1) double = 0
    mass (1,1) double = 1
    refArea (1,1) double = 1
    CL0 (1,1) double = 0
    CLalpha (1,1) double = 1
    alphaLimit (1,1) double = pi / 6
end

vehicleCount = size(rMAll, 2);
if vehicleIndex > vehicleCount
    error('cooperativePnOverloadCommand3D:InvalidVehicleIndex', ...
        'vehicleIndex must not exceed the number of vehicle columns.');
end

guidanceParams.maxAcceleration = maxLoadFactor * g0;
[aAll, info] = cooperativeFuzzyGuidance3D(rMAll, vMAll, rT, vT, adjacency, guidanceParams);

rM = rMAll(:, vehicleIndex); %#ok<NASGU>
vM = vMAll(:, vehicleIndex);
aCmd = aAll(:, vehicleIndex);
[~, eNormal, eLateral] = velocityFrameAxes(vM);

nNormalCmd = dot(aCmd, eNormal) / g0;
nLateralCmd = dot(aCmd, eLateral) / g0;

[alphaCmd, bankCmd] = overloadToAlphaBank(nNormalCmd, nLateralCmd, ...
    vM, rho, mass, refArea, CL0, CLalpha, alphaLimit, g0);
end

function [alphaCmd, bankCmd] = overloadToAlphaBank(nNormalCmd, nLateralCmd, ...
    vM, rho, mass, refArea, CL0, CLalpha, alphaLimit, g0)
nLiftCmd = hypot(nNormalCmd, nLateralCmd);

if nLiftCmd < 1e-9
    bankCmd = 0;
else
    bankCmd = atan2(nLateralCmd, nNormalCmd);
end

speed = norm(vM);
qbar = 0.5 * rho * speed^2;

if qbar <= 1e-9 || refArea <= 0 || mass <= 0 || abs(CLalpha) <= 1e-9
    alphaCmd = 0;
    return;
end

CLRequired = nLiftCmd * mass * g0 / (qbar * refArea);
alphaCmd = (CLRequired - CL0) / CLalpha;
alphaCmd = min(max(alphaCmd, -alphaLimit), alphaLimit);
end

function [eForward, eNormal, eLateral] = velocityFrameAxes(vM)
speed = norm(vM);
if speed < 1e-9
    eForward = [1; 0; 0];
else
    eForward = vM / speed;
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
