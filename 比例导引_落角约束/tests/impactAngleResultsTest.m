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
                max(abs([loaded.summary.thetaErrorDeg])), 1.0);
            testCase.verifyLessThanOrEqual( ...
                max(abs([loaded.summary.psiErrorDeg])), 1.0);
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
            testCase.verifyLessThanOrEqual( ...
                max(abs(diff([loaded.summary.thetaInitialDeg]))), 2.5+eps(2.5));
            testCase.verifyLessThanOrEqual( ...
                max(abs(diff([loaded.summary.psiInitialDeg]))), 1.5+eps(1.5));
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
