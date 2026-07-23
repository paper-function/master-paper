%% Structural inspection for Ctrl_For_HRV.slx
% This script is read-only: it loads the model, checks dependencies,
% compiles it, and exports block/signal summaries for review.

clear; clc;
rootDir = fileparts(mfilename('fullpath'));
cd(rootDir);
addpath(rootDir);

model = 'Ctrl_For_HRV';
outDir = fullfile(rootDir, 'terminal_adrc_results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

bdclose('all');
Parameter;
load_system(model);

fid = fopen(fullfile(outDir, 'model_structure_summary.txt'), 'w');
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Model: %s\n', model);
fprintf(fid, 'File: %s\n', which([model '.slx']));
fprintf(fid, 'InitFcn: %s\n', get_param(model, 'InitFcn'));
fprintf(fid, 'Solver: %s\n', get_param(model, 'Solver'));
fprintf(fid, 'FixedStep: %s\n', get_param(model, 'FixedStep'));
fprintf(fid, 'StopTime: %s\n\n', get_param(model, 'StopTime'));

fprintf(fid, 'Dependency analysis\n');
try
    [files, products] = dependencies.fileDependencyAnalysis([model '.slx']);
    for k = 1:numel(files)
        fprintf(fid, '  FILE %s\n', files{k});
    end
    fprintf(fid, 'Products returned by MATLAB: %d\n\n', numel(products));
catch ME
    fprintf(fid, '  Dependency analysis failed: %s\n\n', ME.message);
end

set_param(model, 'SimulationCommand', 'update');
fprintf(fid, 'Compile/update: OK\n\n');

blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
fprintf(fid, 'Total blocks: %d\n\n', numel(blocks));

fprintf(fid, 'S-functions\n');
sfuncs = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'S-Function');
for k = 1:numel(sfuncs)
    fprintf(fid, '  %s | Function=%s | Params=%s\n', ...
        sfuncs{k}, get_param(sfuncs{k}, 'FunctionName'), get_param(sfuncs{k}, 'Parameters'));
end
fprintf(fid, '\n');

fprintf(fid, 'Top-level blocks and key parameters\n');
top = find_system(model, 'SearchDepth', 1, 'Type', 'Block');
for k = 1:numel(top)
    if strcmp(top{k}, model), continue; end
    bt = get_param(top{k}, 'BlockType');
    fprintf(fid, '  %-32s %-14s', get_param(top{k}, 'Name'), bt);
    switch bt
        case 'Constant'
            fprintf(fid, ' Value=%s', get_param(top{k}, 'Value'));
        case 'Gain'
            fprintf(fid, ' Gain=%s', get_param(top{k}, 'Gain'));
        case 'Goto'
            fprintf(fid, ' Tag=%s', get_param(top{k}, 'GotoTag'));
        case 'From'
            fprintf(fid, ' Tag=%s', get_param(top{k}, 'GotoTag'));
    end
    fprintf(fid, '\n');
end
fprintf(fid, '\n');

fprintf(fid, 'Actuator limits\n');
for blk = ["actuator", "actuator1", "actuator2"]
    path = model + "/" + blk;
    fprintf(fid, '  %s\n', path);
    sats = find_system(path, 'SearchDepth', 1, 'BlockType', 'Saturate');
    for k = 1:numel(sats)
        fprintf(fid, '    %s Lower=%s Upper=%s\n', ...
            get_param(sats{k}, 'Name'), get_param(sats{k}, 'LowerLimit'), get_param(sats{k}, 'UpperLimit'));
    end
    gains = find_system(path, 'SearchDepth', 1, 'BlockType', 'Gain');
    for k = 1:numel(gains)
        fprintf(fid, '    %s Gain=%s\n', get_param(gains{k}, 'Name'), get_param(gains{k}, 'Gain'));
    end
end

fprintf(fid, '\nInitial continuous states from a zero-duration simulation\n');
simIn = Simulink.SimulationInput(model);
simIn = simIn.setModelParameter('StopTime', '0', 'ReturnWorkspaceOutputs', 'on', ...
    'SaveState', 'on', 'StateSaveName', 'xInit', 'SaveFormat', 'StructureWithTime');
simOut = sim(simIn);
xInit = simOut.xInit;
for k = 1:numel(xInit.signals)
    fprintf(fid, '  %2d %-70s dim=%s values=%s\n', k, ...
        xInit.signals(k).blockName, mat2str(xInit.signals(k).dimensions), mat2str(xInit.signals(k).values(1, :), 6));
end

fprintf('Wrote %s\n', fullfile(outDir, 'model_structure_summary.txt'));
