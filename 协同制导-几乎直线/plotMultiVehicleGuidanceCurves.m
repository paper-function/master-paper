function plotMultiVehicleGuidanceCurves()
%PLOTMULTIVEHICLEGUIDANCECURVES Plot the five-vehicle guidance histories.

model = 'Ctrl_For_HRV_20260625';
outputFolder = fullfile(pwd, 'simulink_cooperative_results', ...
    datestr(now, 'yyyymmdd_HHMMSS'));
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

load_system(model);
warningState = warning('off', 'all');
cleanup = onCleanup(@() warning(warningState));
evalin('base', sprintf('sim(''%s'');', model));

data = collectGuidanceData();
save(fullfile(outputFolder, 'multi_vehicle_guidance_curves.mat'), 'data');

colors = lines(5);
vehicleNames = compose('Vehicle %d', 1:5);

plotLoadCurves(data, colors, vehicleNames, outputFolder);
plotAttitudeCurves(data, colors, vehicleNames, outputFolder);
plotFlightPathInclination(data, colors, vehicleNames, outputFolder);
plotFlightPathAzimuth(data, colors, vehicleNames, outputFolder);
plotFlightTrajectories(data, colors, vehicleNames, outputFolder);

fprintf('Saved multi-vehicle plots to: %s\n', outputFolder);
end

function data = collectGuidanceData()
data = struct();
data.target = evalin('base', 'Scope_position_target1');
for i = 1:5
    offset = 30*(i - 1);
    vehicle = struct();
    vehicle.t = readSignal(8 + offset, i, 1);
    vehicle.H = readSignal(8 + offset, i, 2);
    vehicle.V = readSignal(9 + offset, i, 2);
    vehicle.X = readSignal(10 + offset, i, 2);
    vehicle.Y = readSignal(11 + offset, i, 2);
    vehicle.alpha = readSignal(12 + offset, i, 2);
    vehicle.beta = readSignal(13 + offset, i, 2);
    vehicle.miu = readSignal(25 + offset, i, 2);
    vehicle.psi = readSignal(27 + offset, i, 2);
    vehicle.theta = readSignal(30 + offset, i, 2);
    vehicle.ny = readSignal(4 + offset, i, 2);
    vehicle.nz = readSignal(5 + offset, i, 2);
    vehicle.loadMagnitude = hypot(vehicle.ny, vehicle.nz);
    data.vehicle(i) = vehicle;
end
end

function values = readSignal(scopeIndex, vehicleIndex, column)
name = sprintf('ScopeData%d_v%d', scopeIndex, vehicleIndex);
signal = evalin('base', name);
values = signal(:, column);
end

function plotLoadCurves(data, colors, vehicleNames, outputFolder)
fig = figure('Color', 'w', 'Name', 'Load factor histories', ...
    'Position', [100 100 1100 760]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
hold on; grid on;
for i = 1:5
    plot(data.vehicle(i).t, data.vehicle(i).ny, 'LineWidth', 1.2, ...
        'Color', colors(i, :));
end
yline(20, 'k--', '20 g');
yline(-20, 'k--', '-20 g');
ylabel('n_y (g)');
title('Normal load command');
legend(vehicleNames, 'Location', 'bestoutside');

nexttile;
hold on; grid on;
for i = 1:5
    plot(data.vehicle(i).t, data.vehicle(i).nz, 'LineWidth', 1.2, ...
        'Color', colors(i, :));
end
yline(20, 'k--', '20 g');
yline(-20, 'k--', '-20 g');
ylabel('n_z (g)');
title('Lateral load command');

nexttile;
hold on; grid on;
for i = 1:5
    plot(data.vehicle(i).t, data.vehicle(i).loadMagnitude, 'LineWidth', 1.2, ...
        'Color', colors(i, :));
end
yline(20, 'k--', '20 g');
xlabel('Time (s)');
ylabel('sqrt(n_y^2+n_z^2) (g)');
title('Load magnitude');

saveFigure(fig, outputFolder, 'load_factor_curves');
end

function plotAttitudeCurves(data, colors, vehicleNames, outputFolder)
fig = figure('Color', 'w', 'Name', 'Attitude angle histories', ...
    'Position', [130 130 1100 760]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

fields = {'alpha', 'beta', 'miu'};
labels = {'Attack angle alpha (deg)', 'Sideslip angle beta (deg)', ...
    'Bank angle mu (deg)'};
for k = 1:numel(fields)
    nexttile;
    hold on; grid on;
    for i = 1:5
        plot(data.vehicle(i).t, data.vehicle(i).(fields{k}), ...
            'LineWidth', 1.2, 'Color', colors(i, :));
    end
    ylabel(labels{k});
    if k == 1
        title('Attitude angles');
        legend(vehicleNames, 'Location', 'bestoutside');
    end
    if k == numel(fields)
        xlabel('Time (s)');
    end
end

saveFigure(fig, outputFolder, 'attitude_angle_curves');
end

function plotFlightPathInclination(data, colors, vehicleNames, outputFolder)
fig = figure('Color', 'w', 'Name', 'Flight-path inclination histories', ...
    'Position', [160 160 1000 560]);
axes(fig);
hold on; grid on;
for i = 1:5
    plot(data.vehicle(i).t, data.vehicle(i).theta, 'LineWidth', 1.3, ...
        'Color', colors(i, :));
end
xlabel('Time (s)');
ylabel('Flight-path inclination theta (deg)');
title('Flight-path inclination');
legend(vehicleNames, 'Location', 'bestoutside');
saveFigure(fig, outputFolder, 'flight_path_inclination_curves');
end

function plotFlightPathAzimuth(data, colors, vehicleNames, outputFolder)
fig = figure('Color', 'w', 'Name', 'Flight-path azimuth histories', ...
    'Position', [190 190 1000 560]);
axes(fig);
hold on; grid on;
for i = 1:5
    plot(data.vehicle(i).t, data.vehicle(i).psi, 'LineWidth', 1.3, ...
        'Color', colors(i, :));
end
xlabel('Time (s)');
ylabel('Flight-path azimuth psi (deg)');
title('Flight-path azimuth');
legend(vehicleNames, 'Location', 'bestoutside');
saveFigure(fig, outputFolder, 'flight_path_azimuth_curves');
end

function plotFlightTrajectories(data, colors, vehicleNames, outputFolder)
fig = figure('Color', 'w', 'Name', 'Impact trajectories', ...
    'Position', [220 220 1000 760]);
ax = axes(fig);
hold(ax, 'on'); grid(ax, 'on');
for i = 1:5
    plot3(ax, data.vehicle(i).X/1000, data.vehicle(i).Y/1000, ...
        data.vehicle(i).H/1000, 'LineWidth', 1.5, 'Color', colors(i, :));
end
target = data.target;
plot3(ax, target(:, 2)/1000, target(:, 3)/1000, target(:, 4)/1000, ...
    'k--', 'LineWidth', 1.4);
plot3(ax, target(end, 2)/1000, target(end, 3)/1000, target(end, 4)/1000, ...
    'kp', 'MarkerFaceColor', 'y', 'MarkerSize', 12);
xlabel(ax, 'X (km)');
ylabel(ax, 'Y (km)');
zlabel(ax, 'Altitude H (km)');
title(ax, 'Five-vehicle attack trajectories');
legend([vehicleNames, "Target"], 'Location', 'bestoutside');
view(ax, 38, 24);
saveFigure(fig, outputFolder, 'impact_trajectory_3d');
end

function saveFigure(fig, outputFolder, baseName)
pngPath = fullfile(outputFolder, baseName + ".png");
figPath = fullfile(outputFolder, baseName + ".fig");
exportgraphics(fig, pngPath, 'Resolution', 220);
savefig(fig, figPath);
end
