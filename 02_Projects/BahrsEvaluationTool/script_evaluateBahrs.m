%% Clean-up

clear
close all
warning('on', 'backtrace')

addpath(genpath('lib'))

tic

%% Configuration

testConfig.binaryFile = 'sample_bahrs_log.bin';
testConfig.convertBinaryFile = true;
testConfig.converterExecutable = '..\..\03_Firmware\BahrsTargetApp\pf_1_0\Utilities\SerialProtocolToMat.exe';
testConfig.matFile = '';

%% Convert binary file

if testConfig.convertBinaryFile
    if exist(testConfig.binaryFile, 'file')
        if exist(testConfig.converterExecutable,'file')
            % Convert the binary file to MAT format
            [filepath, name, ext] = fileparts(testConfig.binaryFile);
            convertedMatFile = strcat(name, '.mat');
            commandString = sprintf('%s %s %s', testConfig.converterExecutable, testConfig.binaryFile, convertedMatFile);
            [status, result] = system(commandString, '-echo');

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
