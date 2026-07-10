function configureSimulinkTerminalGuidance()
%CONFIGURESIMULINKTERMINALGUIDANCE Tune the terminal HRV guidance block.
%
% The cooperative guidance layer only supplies the terminal handoff state.
% The terminal impact-angle constrained guidance remains inside
% Ctrl_For_HRV_20260625.slx. This helper makes the terminal block explicit:
% desired theta/psi are fed back near impact and both load channels are
% limited to +/-20 g.

modelName = "Ctrl_For_HRV_20260625";
chartPath = modelName + "/MATLAB Function2";

load_system(modelName);
root = sfroot;
chart = root.find("-isa", "Stateflow.EMChart", "Path", chartPath);
if isempty(chart)
    error("configureSimulinkTerminalGuidance:ChartNotFound", ...
        "Cannot find %s.", chartPath);
end

script = chart.Script;
script = replace(script, ...
    "Ktheta = 0.8;Kpsi = 0.8;KterminalTheta = 0.0;KterminalPsi = 0.0;", ...
    "Ktheta = 0.8;Kpsi = 0.8;KterminalTheta = 1.2;KterminalPsi = 1.0;");
script = replace(script, ...
    "-KterminalTheta*V*terminalThetaErr/tgo;", ...
    "+KterminalTheta*V*terminalThetaErr/tgo;");
script = replace(script, ...
    "ny = saturate(aTheta/g+cos(theta)-(V/g)*geomTheta,-8.0,8.0);", ...
    "ny = saturate(aTheta/g+cos(theta)-(V/g)*geomTheta,-20.0,20.0);");
script = replace(script, ...
    "nz = saturate(aPsi/g-(V*cos(theta)/g)*geomPsi,-4.0,4.0);", ...
    "nz = saturate(aPsi/g-(V*cos(theta)/g)*geomPsi,-20.0,20.0);");

chart.Script = script;
save_system(modelName);
fprintf("Configured %s terminal guidance: angle feedback on, load limit +/-20 g.\n", ...
    modelName);
end
