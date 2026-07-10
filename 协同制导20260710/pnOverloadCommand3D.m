function [alphaCmd, bankCmd, nNormalCmd, nLateralCmd, aCmd, aCmdRaw, ...
    lambda, Vc, omegaLOS] = pnOverloadCommand3D(rM, vM, rT, vT, N, ...
    maxLoadFactor, g0, rho, mass, refArea, CL0, CLalpha, alphaLimit)
% pnOverloadCommand3D Vector PN guidance converted to alpha/bank commands.
%#codegen
%
% This function is intended for a Simulink MATLAB Function block.
%
% Inputs:
%   rM, vM          Vehicle position and velocity, 3-by-1, inertial frame
%   rT, vT          Target position and velocity, 3-by-1, inertial frame
%   N               Navigation constant
%   maxLoadFactor   Total overload limit [g]
%   g0              Standard gravity [m/s^2]
%   rho             Atmospheric density at current altitude [kg/m^3]
%   mass            Vehicle mass [kg]
%   refArea         Aerodynamic reference area [m^2]
%   CL0             Lift coefficient at alpha = 0
%   CLalpha         Lift-curve slope [1/rad]
%   alphaLimit      Absolute angle-of-attack limit [rad]
%
% Outputs:
%   alphaCmd        Angle-of-attack command [rad]
%   bankCmd         Bank-angle command [rad]
%   nNormalCmd      Signed normal overload command [g]
%   nLateralCmd     Signed lateral overload command [g]
%   aCmd            Limited acceleration command in inertial frame [m/s^2]
%   aCmdRaw         Raw PN acceleration before projection and limiting [m/s^2]
%   lambda          LOS unit vector
%   Vc              Closing velocity [m/s]
%   omegaLOS        LOS angular velocity vector [rad/s]
%
% Velocity-frame axis definition:
%   eForward        Along vehicle velocity
%   eLateral        Right side, perpendicular to forward and inertial up
%   eNormal         Positive upward in the velocity-vertical plane
%
% Command convention:
%   bankCmd = 0        Lift points along +eNormal
%   bankCmd > 0        Lift rotates from +eNormal toward +eLateral
%   alphaCmd > 0       Produces positive lift coefficient in the model

    r = rT - rM;
    v = vT - vM;
    R = norm(r);

    if R < 1e-9
        lambda = [1; 0; 0];
        Vc = 0;
        omegaLOS = [0; 0; 0];
        aCmdRaw = [0; 0; 0];
        aCmd = [0; 0; 0];
        alphaCmd = 0;
        bankCmd = 0;
        nNormalCmd = 0;
        nLateralCmd = 0;
        return;
    end

    lambda = r / R;
    Vc = -dot(r, v) / R;
    omegaLOS = cross(r, v) / R^2;

    if Vc > 0
        aCmdRaw = N * Vc * cross(omegaLOS, lambda);
    else
        aCmdRaw = [0; 0; 0];
    end

    [eForward, eNormal, eLateral] = velocityFrameAxes(vM);

    % Only the acceleration perpendicular to velocity is realizable as
    % normal/lateral overload command in this baseline model.
    aCmdNoAxial = aCmdRaw - dot(aCmdRaw, eForward) * eForward;
    aCmd = limitVectorMagnitude(aCmdNoAxial, maxLoadFactor * g0);

    nNormalCmd = dot(aCmd, eNormal) / g0;
    nLateralCmd = dot(aCmd, eLateral) / g0;

    [alphaCmd, bankCmd] = overloadToAlphaBank(nNormalCmd, nLateralCmd, ...
        vM, rho, mass, refArea, CL0, CLalpha, alphaLimit, g0);
end

function [alphaCmd, bankCmd] = overloadToAlphaBank(nNormalCmd, nLateralCmd, ...
    vM, rho, mass, refArea, CL0, CLalpha, alphaLimit, g0)
% overloadToAlphaBank Convert velocity-frame overload to alpha and bank.
%
% The lift vector is assumed perpendicular to velocity:
%
%   nNormalCmd  = nLiftCmd * cos(bankCmd)
%   nLateralCmd = nLiftCmd * sin(bankCmd)
%
% The angle of attack is obtained from the simple aerodynamic model:
%
%   L = qbar * refArea * CL
%   CL = CL0 + CLalpha * alpha
%
% Replace this function with aerodynamic-table interpolation if your
% Simulink model has Mach/altitude-dependent CL(alpha, Mach, h).

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
% velocityFrameAxes Build the velocity coordinate frame.

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

function y = limitVectorMagnitude(x, maxMagnitude)
% limitVectorMagnitude Saturate vector magnitude while preserving direction.

    xNorm = norm(x);
    if xNorm > maxMagnitude && xNorm > 0
        y = x * (maxMagnitude / xNorm);
    else
        y = x;
    end
end
