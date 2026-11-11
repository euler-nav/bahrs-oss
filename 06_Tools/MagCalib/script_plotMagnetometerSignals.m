%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
%%
clear all
close all

%%
magnetometerDataLogFile = 'sample_log.txt';

%%

if ~exist(magnetometerDataLogFile, 'file')
    warning('Input log gile not found. Check correctness of the file name.');
else
    [magData, status] = importMagnetometerLog(magnetometerDataLogFile);

    if status
        plotMagnetometerData(magData);
    end
end
