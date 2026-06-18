%% hypersonic_3d_pn_sim.m
% Three-dimensional proportional navigation guidance simulation.
%
% Scenario:
%   A hypersonic vehicle intercepts a moving target in 3-D space. The
%   guidance law uses vector proportional navigation:
%
%       a_cmd = N * Vc * cross(omega_LOS, lambda)
%
% where:
%   r          = r_t - r_m
%   v          = v_t - v_m
%   lambda     = r / norm(r)
%   Vc         = -dot(r, v) / norm(r)
%   omega_LOS  = cross(r, v) / norm(r)^2
%
% The script is intentionally written as a clear baseline model so that
% aerodynamic force, heating/load constraints, bank-to-turn/autopilot
% dynamics, atmosphere, and glide/reentry models can be added later.

clear; clc; close all;

%% Simulation parameters
dt = 0.02;                 % Integration time step [s]
tMax = 80;                 % Maximum simulation time [s]
time = 0:dt:tMax;
nSteps = numel(time);

N = 4.0;                   % Navigation constant, typical PN range: 3 to 5
g0 = 9.80665;              % Standard gravity [m/s^2]
maxLoadFactor = 30;        % Maximum commanded normal overload [g]
aMax = maxLoadFactor * g0; % Maximum acceleration magnitude [m/s^2]

useGravity = true;         % Set false to remove gravity from vehicle motion
gVec = [0; 0; -g0];        % Inertial-frame gravity vector [m/s^2]

hitRadius = 20;            % Successful intercept radius [m]

%% Initial states
% Missile / hypersonic vehicle state.
% r_m: position [m], v_m: velocity [m/s]
r_m = [0; 0; 30000];
v_m = [2100; 80; -120];

% Target state.
% This target uses constant inertial velocity in the baseline model.
r_t = [85000; 13000; 25500];
v_t = [480; -70; 10];

%% History arrays
r_m_hist = nan(3, nSteps);
v_m_hist = nan(3, nSteps);
r_t_hist = nan(3, nSteps);
v_t_hist = nan(3, nSteps);

range_hist = nan(1, nSteps);
Vc_hist = nan(1, nSteps);
a_cmd_hist = nan(3, nSteps);
a_cmd_mag_hist = nan(1, nSteps);
a_cmd_raw_mag_hist = nan(1, nSteps);
omega_los_hist = nan(3, nSteps);

hitFlag = false;
hitTime = NaN;
missDistance = NaN;
stopIndex = nSteps;

%% Main simulation loop
for k = 1:nSteps
    % Save current states before guidance update.
    r_m_hist(:, k) = r_m;
    v_m_hist(:, k) = v_m;
    r_t_hist(:, k) = r_t;
    v_t_hist(:, k) = v_t;

    % Relative motion quantities.
    r = r_t - r_m;              % Relative position from vehicle to target [m]
    v = v_t - v_m;              % Relative velocity [m/s]
    R = norm(r);                % Relative distance [m]

    if R < 1e-9
        hitFlag = true;
        hitTime = time(k);
        missDistance = 0;
        stopIndex = k;
        break;
    end

    lambda = r / R;             % LOS unit vector
    Vc = -dot(r, v) / R;        % Closing velocity; positive when range closes
    omega_LOS = cross(r, v) / R^2;

    % Vector proportional navigation guidance law.
    % If Vc is negative, the target is receding. In that case this baseline
    % model sets PN command to zero rather than commanding an unhelpful turn.
    if Vc > 0
        a_cmd_raw = N * Vc * cross(omega_LOS, lambda);
    else
        a_cmd_raw = [0; 0; 0];
    end

    % Hypersonic vehicles usually have limited maneuver authority. Saturate
    % the commanded acceleration magnitude while preserving its direction.
    a_cmd = limitVectorMagnitude(a_cmd_raw, aMax);

    % Store guidance quantities.
    range_hist(k) = R;
    Vc_hist(k) = Vc;
    omega_los_hist(:, k) = omega_LOS;
    a_cmd_hist(:, k) = a_cmd;
    a_cmd_mag_hist(k) = norm(a_cmd);
    a_cmd_raw_mag_hist(k) = norm(a_cmd_raw);

    % Intercept check at the current step.
    if R <= hitRadius
        hitFlag = true;
        hitTime = time(k);
        missDistance = R;
        stopIndex = k;
        break;
    end

    % Translational dynamics.
    % Baseline target model: constant velocity. Replace a_t with target
    % maneuver acceleration if needed.
    a_t = [0; 0; 0];

    % Baseline vehicle model: direct acceleration command plus optional
    % gravity. Replace this with aerodynamic acceleration and flight-control
    % dynamics for a high-fidelity hypersonic model.
    a_m = a_cmd;
    if useGravity
        a_m = a_m + gVec;
    end

    % Semi-implicit Euler integration. It is simple, stable enough for this
    % baseline example, and easy to replace with RK4 or ode45 later.
    v_m = v_m + a_m * dt;
    r_m = r_m + v_m * dt;

    v_t = v_t + a_t * dt;
    r_t = r_t + v_t * dt;
end

if ~hitFlag
    validRange = range_hist(~isnan(range_hist));
    missDistance = validRange(end);
end

%% Trim histories to simulated portion
idx = 1:stopIndex;
tPlot = time(idx);
r_m_plot = r_m_hist(:, idx);
r_t_plot = r_t_hist(:, idx);
range_plot = range_hist(idx);
a_cmd_mag_plot = a_cmd_mag_hist(idx);
a_cmd_raw_mag_plot = a_cmd_raw_mag_hist(idx);
Vc_plot = Vc_hist(idx);

%% Print result summary
fprintf('\n3-D proportional navigation simulation completed.\n');
fprintf('Navigation constant N          : %.2f\n', N);
fprintf('Maximum overload limit         : %.1f g\n', maxLoadFactor);
fprintf('Gravity enabled                : %d\n', useGravity);
fprintf('Final / minimum miss distance  : %.3f m\n', missDistance);

if hitFlag
    fprintf('Intercept achieved             : yes\n');
    fprintf('Hit time                       : %.3f s\n', hitTime);
else
    [minRange, minIdx] = min(range_plot);
    fprintf('Intercept achieved             : no\n');
    fprintf('Minimum range                  : %.3f m at %.3f s\n', minRange, tPlot(minIdx));
end

%% Plot trajectory and guidance quantities
fig = figure('Color', 'w', 'Name', '3-D Proportional Navigation Simulation');
tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile([2 1]);
plot3(r_m_plot(1, :) / 1000, r_m_plot(2, :) / 1000, r_m_plot(3, :) / 1000, ...
    'b-', 'LineWidth', 1.8);
hold on;
plot3(r_t_plot(1, :) / 1000, r_t_plot(2, :) / 1000, r_t_plot(3, :) / 1000, ...
    'r--', 'LineWidth', 1.8);
plot3(r_m_plot(1, 1) / 1000, r_m_plot(2, 1) / 1000, r_m_plot(3, 1) / 1000, ...
    'bo', 'MarkerFaceColor', 'b');
plot3(r_t_plot(1, 1) / 1000, r_t_plot(2, 1) / 1000, r_t_plot(3, 1) / 1000, ...
    'ro', 'MarkerFaceColor', 'r');
plot3(r_m_plot(1, end) / 1000, r_m_plot(2, end) / 1000, r_m_plot(3, end) / 1000, ...
    'bs', 'MarkerFaceColor', 'b');
plot3(r_t_plot(1, end) / 1000, r_t_plot(2, end) / 1000, r_t_plot(3, end) / 1000, ...
    'rs', 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X [km]');
ylabel('Y [km]');
zlabel('Z [km]');
title('3-D Intercept Geometry');
legend('Hypersonic vehicle', 'Target', 'Vehicle start', 'Target start', ...
    'Vehicle final', 'Target final', 'Location', 'best');
view(35, 22);

nexttile;
plot(tPlot, range_plot / 1000, 'k-', 'LineWidth', 1.6);
grid on;
xlabel('Time [s]');
ylabel('Range [km]');
title('Relative Distance');

nexttile;
plot(tPlot, a_cmd_raw_mag_plot / g0, 'Color', [0.65 0.65 0.65], 'LineWidth', 1.2);
hold on;
plot(tPlot, a_cmd_mag_plot / g0, 'm-', 'LineWidth', 1.6);
yline(maxLoadFactor, 'r--', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('Commanded acceleration [g]');
title('PN Acceleration Command');
legend('Raw PN command', 'Limited command', 'Limit', 'Location', 'best');

figPath = fullfile(pwd, 'hypersonic_3d_pn_trajectory.png');
exportgraphics(fig, figPath, 'Resolution', 180);
fprintf('Figure saved to                : %s\n', figPath);

%% Local helper function
function y = limitVectorMagnitude(x, maxMagnitude)
% limitVectorMagnitude Saturate a vector magnitude without changing direction.
    xNorm = norm(x);
    if xNorm > maxMagnitude && xNorm > 0
        y = x * (maxMagnitude / xNorm);
    else
        y = x;
    end
end
