classdef impactAngleResultsTest < matlab.unittest.TestCase
    %impactAngleResultsTest Validates saved five-case simulation results.

    methods (TestClassSetup)
        function addProjectPath(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            testCase.applyFixture( ...
                matlab.unittest.fixtures.PathFixture(projectFolder));
        end
    end

    methods (Test)
        function testAllCasesHit(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "impact_angle_summary.mat"), "summary");
            testCase.verifyTrue(all([loaded.summary.hit]));
            testCase.verifyLessThanOrEqual( ...
                max([loaded.summary.missDistanceM]), 10);
        end

        function testTerminalAnglesTrackCommands(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "impact_angle_summary.mat"), "summary");
            testCase.verifyLessThanOrEqual( ...
                max(abs([loaded.summary.thetaErrorDeg])), 6.0);
            testCase.verifyLessThanOrEqual( ...
                max(abs([loaded.summary.psiErrorDeg])), 4.0);
        end

        function testFiveCasesAreSaved(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "impact_angle_summary.mat"), "summary");
            testCase.verifyNumElements(loaded.summary, 5);
        end

        function testAngleSpacing(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "impact_angle_summary.mat"), "summary");
            testCase.verifyGreaterThanOrEqual( ...
                min(abs(diff([loaded.summary.thetaDesiredDeg]))), 2.0);
            testCase.verifyGreaterThanOrEqual( ...
                min(abs(diff([loaded.summary.psiDesiredDeg]))), 1.0);
            testCase.verifyGreaterThanOrEqual( ...
                min(abs(diff([loaded.summary.terminalThetaDeg]))), 1.0);
            testCase.verifyGreaterThanOrEqual( ...
                min(abs(diff([loaded.summary.terminalPsiDeg]))), 1.0);
        end

        function testFlightPathAnglesEvolve(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "impact_angle_summary.mat"), "summary");
            testCase.verifyGreaterThanOrEqual( ...
                min(abs([loaded.summary.thetaDesiredDeg] - ...
                [loaded.summary.thetaInitialDeg])), 1.5);
            testCase.verifyGreaterThanOrEqual( ...
                min(abs([loaded.summary.psiDesiredDeg] - ...
                [loaded.summary.psiInitialDeg])), 0.5);

            dataFiles = dir(fullfile(projectFolder, ...
                "impact_angle_results", "case_*_flight_data.mat"));
            thetaExcursion = zeros(1, numel(dataFiles));
            psiExcursion = zeros(1, numel(dataFiles));
            for fileIndex = 1:numel(dataFiles)
                loadedData = load(fullfile(dataFiles(fileIndex).folder, ...
                    dataFiles(fileIndex).name), "flightData");
                thetaExcursion(fileIndex) = ...
                    max(loadedData.flightData.flightPathThetaDeg) - ...
                    min(loadedData.flightData.flightPathThetaDeg);
                psiExcursion(fileIndex) = ...
                    max(loadedData.flightData.flightPathPsiDeg) - ...
                    min(loadedData.flightData.flightPathPsiDeg);
            end
            testCase.verifyGreaterThanOrEqual(min(thetaExcursion), 1.0);
            testCase.verifyGreaterThanOrEqual(min(psiExcursion), 0.3);
            testCase.verifyGreaterThanOrEqual(mean(psiExcursion), 0.4);
        end

        function testOverloadLimits(testCase)
            projectFolder = fileparts(fileparts(mfilename("fullpath")));
            loaded = load(fullfile(projectFolder, "impact_angle_results", ...
                "case_03_theta_-45.0_psi_+17.5_flight_data.mat"), ...
                "flightData");
            testCase.verifyLessThanOrEqual( ...
                max(abs(loaded.flightData.normalLoadCommandG)), 8.0);
            testCase.verifyLessThanOrEqual( ...
                max(abs(loaded.flightData.lateralLoadCommandG)), 4.0);
        end
    end
end
