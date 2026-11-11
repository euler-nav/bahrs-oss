%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
%% Magnetometer and Pressure Calibration for BAHRS Debug Output
% Supports both binary debug logs and MAT files from BAHRS RTE debug output.
% Calibrates PortMagnetometerInput1/2/3 and the offsets of PortPressureInput1/2/3
% in one run from the same recording and stores structured results. The
% calibration parameters are also exported as defaults of an NVM map JSON.

clearvars;
close all;
warning('on', 'backtrace');

runTimer = tic();

scriptPath = mfilename('fullpath');
if isempty(scriptPath)
    scriptDir = pwd;
else
    scriptDir = fileparts(scriptPath);
end

%% Configuration

config = struct();
config.inputPath = '20261022-calib-daq-2-0.bin';
config.outputFile = fullfile(scriptDir, 'calibration_results.mat');
config.converterExecutable = '../../02_Projects/VisualStudioToolsBahrs/WorkDir/DebugOutputToMatBahrsV3.exe';

% Optional sample range (1-based, counted over valid samples). Use [] to
% start with the first / end with the last valid sample, respectively.
config.startIndex = [];
config.endIndex = [];

% Change the reference value before calibration. The Earth's magnetic field
% changes over time. A calculator can be found here:
% https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
config.referenceMagneticFieldNed = [0.210868; 0; 0.440126];

config.enableSensor = [true, true, true];
config.plotResults = true;

% Pressure offsets are computed from the same recording and are relative to the
% mean of the three barometers. The magnetometer startIndex/endIndex and
% enableSensor settings do not apply to them. If any barometer has fewer valid
% samples than minPressureSamples, all pressure offsets are set to zero.
% pressureDriftWarnPa: a warning is printed if the mean pressure of the last
% 10 % of the samples differs from the first 10 % by more than this value.
config.calibratePressure = true;
config.minPressureSamples = 100;
config.pressureDriftWarnPa = 20.0;

% Writes a copy of the NVM map whose defaults are the computed calibration
% parameters. Open it in NVMManager instead of nvm_map.json.
config.exportNvmJson = true;

config.nvmMapTemplate = fullfile(scriptDir, '..', 'NVM_driver_generator', 'nvm_map.json');
config.nvmMapOutput = fullfile(scriptDir, 'nvm_map_calibrated.json');
config.runStartTime = now();

%% Run the calibration

fprintf('========== Magnetometer Calibration (BAHRS Debug Output) ==========\n');
fprintf('Input file: %s\n', config.inputPath);
fprintf('Output file: %s\n', config.outputFile);

if isempty(config.inputPath)
    error('mag_calib:InvalidConfig', 'inputPath is required and must be non-empty.');
end

if exist(config.inputPath, 'file') ~= 2
    error('mag_calib:FileNotFound', 'Input file not found: %s', config.inputPath);
end

[~, ~, inputExtension] = fileparts(config.inputPath);
if false == strcmpi(inputExtension, '.bin') && false == strcmpi(inputExtension, '.mat')
    error('mag_calib:InvalidExtension', 'Input file must be .bin or .mat, got %s.', inputExtension);
end

if numel(config.referenceMagneticFieldNed) ~= 3
    error('mag_calib:InvalidReference', 'referenceMagneticFieldNed must contain exactly 3 elements.');
end

if false == isempty(config.startIndex) && (false == isscalar(config.startIndex) || config.startIndex <= 0)
    error('mag_calib:NegativeSkip', 'startIndex must be empty or a positive scalar.');
end

if false == isempty(config.endIndex) && (false == isscalar(config.endIndex) || config.endIndex <= 0)
    error('mag_calib:NegativeSkip', 'endIndex must be empty or a positive scalar.');
end

if false == isempty(config.startIndex) && false == isempty(config.endIndex) && config.startIndex > config.endIndex
    error('mag_calib:InvalidIndices', 'startIndex must be less than or equal to endIndex.');
end

hasFminunc = (exist('fminunc', 'file') == 2) || (exist('fminunc', 'builtin') == 5);

fprintf('\n--- Phase 1: Load and convert input data ---\n');

if strcmpi(inputExtension, '.bin')
    if isempty(config.converterExecutable)
        error('mag_calib:MissingConverter', 'converterExecutable is required for .bin input. Adapt the path or build the VS solution.');
    end

    if exist(config.converterExecutable, 'file') ~= 2
        error('mag_calib:ConverterNotFound', 'Converter executable not found: %s', config.converterExecutable);
    end

    [inputDir, inputStem, ~] = fileparts(config.inputPath);
    convertedMatFile = fullfile(inputDir, [inputStem '_converted.mat']);
    commandString = sprintf('"%s" "%s" "%s"', config.converterExecutable, config.inputPath, convertedMatFile);
    [status, result] = system(commandString);

    if ~isempty(strtrim(result))
        fprintf('%s\n', strtrim(result));
    end

    if status ~= 0
        error('mag_calib:ConversionFailed', 'Debug-output conversion failed with status %d.', status);
    end

    if exist(convertedMatFile, 'file') ~= 2
        error('mag_calib:MissingConvertedMat', 'Converter did not create MAT file %s.', convertedMatFile);
    end

    convertedMatInfo = dir(convertedMatFile);
    if convertedMatInfo.bytes <= 0
        error('mag_calib:EmptyConvertedMat', 'Converted MAT file %s is empty.', convertedMatFile);
    end

    matFilePath = convertedMatFile;
else
    convertedMatFile = '';
    matFilePath = config.inputPath;
end

fprintf('Loading MAT file: %s\n', matFilePath);
matData = load(matFilePath);

fprintf('\n--- Phase 2: Per-sensor calibration ---\n');

portNames = {'PortMagnetometerInput1', 'PortMagnetometerInput2', 'PortMagnetometerInput3'};
calibrationResults = repmat(struct( ...
    'sensorId', 0, ...
    'portName', '', ...
    'success', false, ...
    'errorId', '', ...
    'errorMessage', '', ...
    'softIron', [], ...
    'hardIron', [], ...
    'optimizerStatus', NaN, ...
    'optimizerFval', NaN, ...
    'rawSampleCount', 0, ...
    'validSampleCount', 0, ...
    'calibrationSampleCount', 0, ...
    'meanFieldNormBefore', NaN, ...
    'meanFieldNormAfter', NaN, ...
    'timestampSpanS', NaN), 1, numel(portNames));

for iSensor = 1:numel(portNames)
    portName = portNames{iSensor};
    calibrationResults(iSensor).sensorId = iSensor;
    calibrationResults(iSensor).portName = portName;

    if false == config.enableSensor(iSensor)
        calibrationResults(iSensor).errorId = 'SensorDisabled';
        calibrationResults(iSensor).errorMessage = 'Sensor disabled by configuration.';
        fprintf('  Sensor %d (%s): SKIPPED\n', iSensor, portName);
        continue;
    end

    if false == isfield(matData, portName)
        calibrationResults(iSensor).errorId = 'PortNotFound';
        calibrationResults(iSensor).errorMessage = sprintf('%s not found in MAT file.', portName);
        fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, calibrationResults(iSensor).errorMessage);
        continue;
    end

    portData = matData.(portName);

    try
        if false == isfield(portData, 'fVectorX_') || false == isfield(portData, 'fVectorY_') || false == isfield(portData, 'fVectorZ_')
            error('mag_calib:MissingField', 'Required vector field missing from %s.', portName);
        end

        vectorX = reshape(double(portData.fVectorX_), 1, []);
        vectorY = reshape(double(portData.fVectorY_), 1, []);
        vectorZ = reshape(double(portData.fVectorZ_), 1, []);
        rawSampleCount = numel(vectorX);

        if isfield(portData, 'uValid_')
            validMask = reshape(double(portData.uValid_), 1, []) ~= 0.0;
        else
            validMask = true(1, rawSampleCount);
        end

        finiteMask = isfinite(vectorX) & isfinite(vectorY) & isfinite(vectorZ);
        filteredMask = validMask & finiteMask;
        validIndices = find(filteredMask);

        calibrationResults(iSensor).rawSampleCount = rawSampleCount;
        calibrationResults(iSensor).validSampleCount = sum(double(filteredMask));

        if isempty(config.startIndex)
            effectiveStartIndex = 1;
        else
            effectiveStartIndex = config.startIndex;
        end

        if isempty(config.endIndex)
            effectiveEndIndex = numel(validIndices);
        else
            effectiveEndIndex = config.endIndex;
        end

        if effectiveStartIndex > numel(validIndices) || effectiveEndIndex > numel(validIndices) || effectiveStartIndex > effectiveEndIndex
            calibrationResults(iSensor).errorId = 'IndexOutOfBounds';
            calibrationResults(iSensor).errorMessage = sprintf('Sample range %d..%d is invalid for valid sample count %d.', ...
                effectiveStartIndex, effectiveEndIndex, numel(validIndices));
            fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, calibrationResults(iSensor).errorMessage);
            continue;
        end

        trimmedIndices = validIndices(effectiveStartIndex:effectiveEndIndex);

        if isfield(portData, 'uTimestampUs_') && numel(trimmedIndices) >= 2
            timestampsUs = reshape(double(portData.uTimestampUs_), 1, []);
            calibrationResults(iSensor).timestampSpanS = (timestampsUs(trimmedIndices(end)) - timestampsUs(trimmedIndices(1))) / 1e6;
        end

        measurements = [vectorX(trimmedIndices); vectorY(trimmedIndices); vectorZ(trimmedIndices)];
        calibrationResults(iSensor).calibrationSampleCount = size(measurements, 2);

        if size(measurements, 2) < 10
            calibrationResults(iSensor).errorId = 'InsufficientSamples';
            calibrationResults(iSensor).errorMessage = sprintf('Only %d samples available after preprocessing.', size(measurements, 2));
            fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, calibrationResults(iSensor).errorMessage);
            continue;
        end

        if false == hasFminunc
            calibrationResults(iSensor).errorId = 'MissingOptimizer';
            calibrationResults(iSensor).errorMessage = 'fminunc is not available in this Octave environment.';
            fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, calibrationResults(iSensor).errorMessage);
            continue;
        end

        objective = @(calibParams) calibFunction(calibParams, measurements, config.referenceMagneticFieldNed);
        optimizerOptions = optimset('Display', 'off');
        initialGuess = zeros(12, 1);
        fprintf('    Running optimization... ');
        [calibParams, optimizerFval, optimizerStatus] = fminunc(objective, initialGuess, optimizerOptions);
        fprintf('done.\n');

        if optimizerStatus < 1
            calibrationResults(iSensor).optimizerStatus = optimizerStatus;
            calibrationResults(iSensor).optimizerFval = optimizerFval;
            calibrationResults(iSensor).errorId = 'OptimizerFailed';
            calibrationResults(iSensor).errorMessage = sprintf('Optimizer status %d indicates unreliable results.', optimizerStatus);
            fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, calibrationResults(iSensor).errorMessage);
            continue;
        end

        softIron = eye(3) + reshape(calibParams(4:12), 3, 3);
        hardIron = calibParams(1:3);
        compensatedMeasurements = softIron * (measurements + hardIron);

        calibrationResults(iSensor).success = true;
        calibrationResults(iSensor).softIron = softIron;
        calibrationResults(iSensor).hardIron = hardIron;
        calibrationResults(iSensor).optimizerStatus = optimizerStatus;
        calibrationResults(iSensor).optimizerFval = optimizerFval;
        calibrationResults(iSensor).meanFieldNormBefore = mean(sqrt(sum(measurements .^ 2, 1)));
        calibrationResults(iSensor).meanFieldNormAfter = mean(sqrt(sum(compensatedMeasurements .^ 2, 1)));

        fprintf('  Sensor %d (%s): SUCCESS\n', iSensor, portName);
        fprintf('    Samples: raw=%d, valid=%d, used=%d\n', calibrationResults(iSensor).rawSampleCount, calibrationResults(iSensor).validSampleCount, calibrationResults(iSensor).calibrationSampleCount);
        fprintf('    Field norm: before=%.6f G, after=%.6f G\n', calibrationResults(iSensor).meanFieldNormBefore, calibrationResults(iSensor).meanFieldNormAfter);
        fprintf('    Optimizer fval: %.6e, status: %d\n', optimizerFval, optimizerStatus);

        if config.plotResults
            fieldNormBeforeCompensation = sqrt(sum(measurements .^ 2, 1));
            fieldNormAfterCompensation = sqrt(sum(compensatedMeasurements .^ 2, 1));

            figure;
            plot(fieldNormAfterCompensation);
            hold on;
            plot(fieldNormBeforeCompensation);
            title(sprintf('Magnetic field norm. Sensor %d.', iSensor));
            legend('After compensation', 'Before compensation');

            figure;
            for iAxis = 1:1:3
                subplot(3, 1, iAxis);
                plot(compensatedMeasurements(iAxis, :));
                hold on;
                plot(measurements(iAxis, :));

                if iAxis == 1
                    title(sprintf('Magnetic measurements. Sensor %d.', iSensor));
                    legend('Compensated', 'Raw');
                end
            end
        end
    catch exception
        calibrationResults(iSensor).errorId = 'CalibrationException';
        calibrationResults(iSensor).errorMessage = exception.message;
        fprintf('  Sensor %d (%s): FAILED - %s\n', iSensor, portName, exception.message);
    end
end

if config.plotResults
    fprintf('\n--- Phase 3: Plot verification ---\n');
end

fprintf('\n--- Phase 4: Pressure offsets ---\n');

if config.calibratePressure
    pressureResults = calibratePressureOffsets(matData, config.minPressureSamples, config.pressureDriftWarnPa);
    pressureOffsets = pressureResults.offsets;
else
    fprintf('  SKIPPED (config.calibratePressure is false)\n');
    pressureResults = [];
    pressureOffsets = [];
end

fprintf('\n--- Phase 5: Save results ---\n');

runSummary = struct();
runSummary.inputFile = config.inputPath;
runSummary.convertedMatFile = convertedMatFile;
runSummary.converterExecutable = config.converterExecutable;
runSummary.outputFile = config.outputFile;
runSummary.config = config;
runSummary.runEndTime = now();
runSummary.elapsedTimeS = toc(runTimer);
runSummary.successCount = sum(double([calibrationResults.success]));
runSummary.failureCount = numel(calibrationResults) - runSummary.successCount;

save('-v7', config.outputFile, 'calibrationResults', 'pressureResults', 'runSummary');

fprintf('Results saved to: %s\n', config.outputFile);
fprintf('  Successful magnetometers: %d/3\n', runSummary.successCount);
fprintf('  Failed magnetometers: %d/3\n', runSummary.failureCount);

if config.calibratePressure
    if pressureResults.success
        fprintf('  Pressure calibration: SUCCESS\n');
    else
        fprintf('  Pressure calibration: FAILED\n');
    end
end

if config.exportNvmJson
    exportNvmJson(calibrationResults, pressureOffsets, config.nvmMapTemplate, config.nvmMapOutput);
    fprintf('NVM map with calibration defaults saved to: %s\n', config.nvmMapOutput);
end
fprintf('\n========== Calibration Complete ==========\n');
fprintf('Total elapsed time: %.2f seconds\n', toc(runTimer));
