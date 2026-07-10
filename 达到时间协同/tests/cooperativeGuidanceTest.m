classdef cooperativeGuidanceTest < matlab.unittest.TestCase
    %COOPERATIVEGUIDANCETEST Tests fuzzy cooperative guidance helpers.

    methods (TestClassSetup)
        function addProjectFolderToPath(testCase)
            projectFolder = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(projectFolder));
        end
    end

    methods (Test)
        function testCoreGuidanceReturnsFiniteCommands(testCase)
            [position, velocity, target, adjacency, params] = sampleScenario();

            [aCmd, info] = cooperativeFuzzyGuidance3D(position, velocity, ...
                target, [0; 0; 0], adjacency, params);

            testCase.verifySize(aCmd, [3 3]);
            testCase.verifyTrue(all(isfinite(aCmd), 'all'));
            testCase.verifyGreaterThanOrEqual(info.cooperativeWeight, 0);
            testCase.verifyLessThanOrEqual(info.cooperativeWeight, 1);
            testCase.verifySize(info.eta, [1 3]);
            testCase.verifySize(info.xi, [1 3]);
        end

        function testCommandHasNoAxialAcceleration(testCase)
            [position, velocity, target, adjacency, params] = sampleScenario();

            aCmd = cooperativeFuzzyGuidance3D(position, velocity, target, ...
                [0; 0; 0], adjacency, params);

            for i = 1:size(position, 2)
                speed = norm(velocity(:, i));
                axialAcceleration = dot(aCmd(:, i), velocity(:, i) / speed);
                testCase.verifyLessThan(abs(axialAcceleration), 1e-8);
            end
        end

        function testOverloadWrapperReturnsFiniteCommands(testCase)
            [position, velocity, target, adjacency, params] = sampleScenario();

            [alphaCmd, bankCmd, nNormalCmd, nLateralCmd, aCmd, info] = ...
                cooperativePnOverloadCommand3D(2, position, velocity, target, ...
                [0; 0; 0], adjacency, params, 8, 9.806, 0.02, 63504, ...
                334.73, 0.2, 3.0, deg2rad(20));

            testCase.verifyTrue(all(isfinite([alphaCmd, bankCmd, ...
                nNormalCmd, nLateralCmd])));
            testCase.verifySize(aCmd, [3 1]);
            testCase.verifyTrue(isfield(info, 'cooperativeWeight'));
        end
    end
end

function [position, velocity, target, adjacency, params] = sampleScenario()
target = [100000; 30000; 0];
position = [ ...
    0, 2000, -1500;
    0, -2500, 3500;
    26000, 28000, 24500];
speed = [2700, 2500, 2850];
velocity = zeros(3, 3);
for i = 1:3
    lineOfSight = target - position(:, i);
    velocity(:, i) = speed(i) * lineOfSight / norm(lineOfSight);
end
adjacency = ones(3) - eye(3);
params = struct( ...
    'navigationConstant', 4, ...
    'consensusGain', 0.45, ...
    'maxAcceleration', 8 * 9.806, ...
    'terminalPngRange', 3000, ...
    'fuzzyFarRange', 45000, ...
    'cooperativeScale', 4.0, ...
    'impactDirection', [cosd(45); 0; -sind(45)]);
end
