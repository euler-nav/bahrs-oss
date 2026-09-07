%% Clean-up

clear
close all
warning('on', 'backtrace')

tic

%% Configuration

testConfig.binaryFile = 'data/eulernav_log_20260827_084643.bin';
testConfig.outputBinaryFile = 'resim_eulernav_log_20260827_085843.bin';

testConfig.plotBahrsFilter1 = false;
testConfig.plotBahrsFilter2 = false;
testConfig.plotBahrsFilter3 = false;
testConfig.plotImuMonitor = true;
testConfig.plotBarometerMonitor = true;
testConfig.plotAttitudeMonitor = true;
testConfig.plotVerticalChannelMonitor = true;

%% Set up paths for the replay tool and debug output converter

addpath(genpath('func'));
testConfig.replayExecutable = 'bin\BahrsReplayToolBahrsV3.exe';
testConfig.debugOutputToMatExe = 'bin\DebugOutputToMatBahrsV3.exe';

%% Run replay

if ~exist(testConfig.replayExecutable, 'file')
    ME = MException('script_runBahrsFilterReplay:replayToolNotFound', ...
        'Executable %s not found', testConfig.replayExecutable);
    throw(ME);
end

commandString = sprintf('%s "%s" "%s"', ...
    testConfig.replayExecutable, ...
    testConfig.binaryFile, ...
    testConfig.outputBinaryFile);

[status, ~] = system(commandString);

if status ~= 0
    ME = MException('script_runBahrsFilterReplay:replayFailed', ...
        'Replay failed with error %d', status);
    throw(ME);
end

%% Convert debug output to MAT

if ~exist(testConfig.debugOutputToMatExe, 'file')
    ME = MException('script_runBahrsFilterReplay:debugOutputToMatNotFound', ...
        'Executable %s not found', testConfig.debugOutputToMatExe);
    throw(ME);
end

[~, name, ~] = fileparts(testConfig.outputBinaryFile);
outputMatFile = strcat(name, '.mat');

commandString = sprintf('%s "%s" "%s"', ...
    testConfig.debugOutputToMatExe, ...
    testConfig.outputBinaryFile, ...
    outputMatFile);

[status, ~] = system(commandString);

if status ~= 0
    ME = MException('script_runBahrsFilterReplay:matConversionFailed', ...
        'MAT conversion failed with error %d', status);
    throw(ME);
end

%% Plot BAHRS filter states from debug MAT file

fprintf('\n ### Plotting BAHRS filter outputs...\n');

matData = load(outputMatFile);

if isfield(matData, 'PortBahrsFilterOutput1') && testConfig.plotBahrsFilter1
    plotBahrsFilterState(matData.PortBahrsFilterOutput1, 'BAHRS Filter 1');
end

if isfield(matData, 'PortBahrsFilterOutput2') && testConfig.plotBahrsFilter2
    plotBahrsFilterState(matData.PortBahrsFilterOutput2, 'BAHRS Filter 2');
end

if isfield(matData, 'PortBahrsFilterOutput3') && testConfig.plotBahrsFilter3
    plotBahrsFilterState(matData.PortBahrsFilterOutput3, 'BAHRS Filter 3');
end

%%

if isfield(matData, 'PortImuDataAfterMonitor') && testConfig.plotImuMonitor
    plotImuMonitorResults(matData.PortImuDataAfterMonitor, 'IMU Monitor');
end

if testConfig.plotBarometerMonitor
    if isfield(matData, 'PortSafePressureData1') && ...
       isfield(matData, 'PortSafePressureData2') && ...
       isfield(matData, 'PortSafePressureData3')
        plotBarometerMonitorResults(matData.PortSafePressureData1, ...
                                    matData.PortSafePressureData2, ...
                                    matData.PortSafePressureData3, ...
                                    'Barometer Monitor');
    else
        fprintf('Safe pressure data ports missing, skipping barometer monitor.\n');
    end
end

if isfield(matData, 'PortSafeAttitudeData') && testConfig.plotAttitudeMonitor
    plotSafeAttitudeData(matData.PortSafeAttitudeData, 'Attitude Monitor');
end

if isfield(matData, 'PortSafeVerticalChannelData') && testConfig.plotVerticalChannelMonitor
    plotSafeVerticalChannelData(matData.PortSafeVerticalChannelData, 'Vertical Channel Monitor');
end

fprintf('\n ### Done.\n');

%% Print execution time

fprintf('\n\n');
toc;
