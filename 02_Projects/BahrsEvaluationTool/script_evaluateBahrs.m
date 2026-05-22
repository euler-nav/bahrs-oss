%% Clean-up

clear
close all
warning('on', 'backtrace')

addpath('lib')

tic

%% Configuration

testConfig.binaryFile = 'data/eulernav_log_20260101_115823.bin';
testConfig.convertBinaryFile = true;
testConfig.software_version = '<=1.7'; % Valid options: '<=1.7', '>=1.8'
testConfig.matFile = ''; % Needs to be set if convertBinaryFile is false

%% Derive parameters from configuration
if strcmp(testConfig.software_version, '<=1.7')
    testConfig.converterExecutable = '..\..\03_Firmware\BahrsTargetApp\pf_1_0\Utilities\SerialProtocolToMat.exe';
    addpath('lib/up_to_pf_1_7');
elseif strcmp(testConfig.software_version, '>=1.8')
    testConfig.converterExecutable = '..\..\03_Firmware\BahrsTargetApp\pf_1_8\Tools\SerialProtocolToMat.exe';
    addpath('lib/from_pf_1_8');
else
    error('Invalid software version provided');
end


%% Convert binary file

if testConfig.convertBinaryFile
    if exist(testConfig.binaryFile, 'file')
        if exist(testConfig.converterExecutable,'file')
            % Convert the binary file to MAT format
            [filepath, name, ext] = fileparts(testConfig.binaryFile);
            convertedMatFile = strcat(name, '.mat');
            commandString = sprintf('%s %s %s', testConfig.converterExecutable, testConfig.binaryFile, convertedMatFile);
            [status, result] = system(commandString);

            % Octave has a bug, so we do this instead of using '-echo'
            % option in the system() call.
            disp(result);

            if status ~= 0
                warning('script_evaluateBahrs:converterFailed',...
                        'Binary file conversion failed with error %d', status);
            end
        else
            warning('script_evaluateBahrs:converterNotFound',...
                    'Executable %s not found', testConfig.converterExecutable);
        end
    else
        warning('script_evaluateBahrs:noRs232Log',...
                'Failed to find the input file %s', testConfig.binaryFile);
    end
end

%% Evaluate RS232 data

canEvaluateRs232 = false;

if ~testConfig.convertBinaryFile
    if exist(testConfig.matFile, 'file')
        objBahrsOutput = load(testConfig.matFile);
        canEvaluateRs232 = true;
    else
        warning('script_evaluateBahrs:inputMatFileNotFound', 'Failed to find the file %s', testConfig.matFile);
    end
elseif exist('convertedMatFile', 'var') && exist(convertedMatFile, 'file')
    objBahrsOutput = load(convertedMatFile);
    canEvaluateRs232 = true;
end

if canEvaluateRs232
    plotBahrsData(objBahrsOutput);
end

%% Print execution time

fprintf('\n\n');
toc;
