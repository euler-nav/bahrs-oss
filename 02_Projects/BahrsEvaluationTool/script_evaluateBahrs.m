%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
%% Clean-up

clear
close all
warning('on', 'backtrace')

tic

%% Configuration

testConfig.binaryFile = 'H:\My Drive\BAHRS\06_VV\PF_2_0\candidate_7\v3\v3_pf_2_0_candidate_7_static_10x_10min\static_10x_10min_1.bin';
testConfig.convertBinaryFile = true;
testConfig.converterExecutable = '..\VisualStudioToolsBahrs\WorkDir\SerialProtocolToMat.exe';
testConfig.matFile = '';
testConfig.canTraceFile = 'Test_1.trc';

%% Convert binary file

if testConfig.convertBinaryFile
    if exist(testConfig.binaryFile, 'file')
        if exist(testConfig.converterExecutable,'file')
            % Convert the binary file to MAT format
            [filepath, name, ext] = fileparts(testConfig.binaryFile);
            convertedMatFile = strcat(name, '.mat');
            commandString = sprintf('%s "%s" %s', testConfig.converterExecutable, testConfig.binaryFile, convertedMatFile);
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

%% Load CAN trace

if exist(testConfig.canTraceFile,'file')
    parsedFile = parseTrcRxFrames(testConfig.canTraceFile);

    uniqueRxCanIds = unique(parsedFile{3});

    if ~isempty(uniqueRxCanIds)
        canRxFrames = cell(1, length(uniqueRxCanIds));
        for i=1:1:length(uniqueRxCanIds)
            mask = (parsedFile{3} == uniqueRxCanIds(i));
            frameTypeCount = sum(mask);
            floatPayload = swapbytes(typecast(reshape(uint8([parsedFile{9}(mask), parsedFile{10}(mask), parsedFile{11}(mask), parsedFile{12}(mask)])', 1, 4 * frameTypeCount), 'single'));
            canRxFrames{i} = struct('nodeId', parsedFile{5}(mask)',...
                                    'timeUs', 1000 * parsedFile{2}(mask)',...,
                                    'dataType', parsedFile{6}(mask)',...
                                    'serviceCode', parsedFile{7}(mask)',...
                                    'messageCode', parsedFile{8}(mask)',...
                                    'payload', floatPayload);
        end
    end
else
    warning('script_evaluateBahrs:noCanTrace',...
        'Failed to find the input file %s', testConfig.canTraceFile);
end

%% Evaluate RS232 date

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

%% Evaluate CAN trace

if exist('uniqueRxCanIds', 'var') && exist('canRxFrames', 'var')
    plotBahrsCanData(uniqueRxCanIds, canRxFrames);
end

%% Print execution time

fprintf('\n\n');
toc;
