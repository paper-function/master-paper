function plotImpactAngleComparison(summary, outputFolder)
%PLOTIMPACTANGLECOMPARISON Plot five impact-angle test histories.

caseCount = numel(summary);
colors = lines(caseCount);
thetaFigure = figure("Color", "w", "Name", "Impact inclination comparison");
thetaAxes = axes(thetaFigure);
hold(thetaAxes, "on");
psiFigure = figure("Color", "w", "Name", "Impact azimuth comparison");
psiAxes = axes(psiFigure);
hold(psiAxes, "on");
loadFigure = figure("Color", "w", "Name", "Overload comparison");
loadLayout = tiledlayout(loadFigure, 2, 1, TileSpacing="compact");
normalLoadAxes = nexttile(loadLayout);
hold(normalLoadAxes, "on");
lateralLoadAxes = nexttile(loadLayout);
hold(lateralLoadAxes, "on");

for caseIndex = 1:caseCount
    caseTag = sprintf("case_%02d_theta_%+05.1f_psi_%+04.1f", ...
        caseIndex, summary(caseIndex).thetaDesiredDeg, ...
        summary(caseIndex).psiDesiredDeg);
    loaded = load(fullfile(outputFolder, caseTag + "_flight_data.mat"), ...
        "flightData");
    data = loaded.flightData;
    label = sprintf("Case %d: %.1f deg / %.1f deg", caseIndex, ...
        data.desiredThetaDeg, data.desiredPsiDeg);

    plot(thetaAxes, data.timeS, data.flightPathThetaDeg, ...
        LineWidth=1.5, Color=colors(caseIndex, :), DisplayName=label);
    yline(thetaAxes, data.desiredThetaDeg, "--", ...
        Color=colors(caseIndex, :), HandleVisibility="off");
    plot(psiAxes, data.timeS, data.flightPathPsiDeg, ...
        LineWidth=1.5, Color=colors(caseIndex, :), DisplayName=label);
    yline(psiAxes, data.desiredPsiDeg, "--", ...
        Color=colors(caseIndex, :), HandleVisibility="off");
    plot(normalLoadAxes, data.normalLoadTimeS, data.normalLoadCommandG, ...
        LineWidth=1.3, Color=colors(caseIndex, :), DisplayName=label);
    plot(lateralLoadAxes, data.lateralLoadTimeS, data.lateralLoadCommandG, ...
        LineWidth=1.3, Color=colors(caseIndex, :), DisplayName=label);
end

formatAxes(thetaAxes, "Time (s)", "Flight-path inclination (deg)", ...
    "Terminal flight-path inclination comparison");
formatAxes(psiAxes, "Time (s)", "Flight-path azimuth (deg)", ...
    "Terminal flight-path azimuth comparison");
formatAxes(normalLoadAxes, "Time (s)", "Normal load command (g)", ...
    "Normal overload comparison");
formatAxes(lateralLoadAxes, "Time (s)", "Lateral load command (g)", ...
    "Lateral overload comparison");

exportgraphics(thetaFigure, ...
    fullfile(outputFolder, "impact_theta_comparison.png"), Resolution=200);
exportgraphics(psiFigure, ...
    fullfile(outputFolder, "impact_psi_comparison.png"), Resolution=200);
savefig(thetaFigure, fullfile(outputFolder, "impact_theta_comparison.fig"));
savefig(psiFigure, fullfile(outputFolder, "impact_psi_comparison.fig"));
exportgraphics(loadFigure, ...
    fullfile(outputFolder, "overload_comparison.png"), Resolution=200);
savefig(loadFigure, fullfile(outputFolder, "overload_comparison.fig"));
end

function formatAxes(targetAxes, xLabelText, yLabelText, titleText)
grid(targetAxes, "on");
box(targetAxes, "on");
xlabel(targetAxes, xLabelText);
ylabel(targetAxes, yLabelText);
title(targetAxes, titleText);
legend(targetAxes, Location="best");
end
