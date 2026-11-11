%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function [magData, status] = importMagnetometerLog(filename)
    magData = [];
    status = true;

    fid = fopen(filename, 'r');

    if fid > 0
        while ~feof(fid)
            line = fgetl(fid);

            if strcmp(line(1:3), 'MAG')
                data = textscan(fid, 'MAG %s %d %f %f %f\r\n');
                break;
            end
        end

        minLength = min([length(data{1}), length(data{2}), length(data{3}), length(data{4}), length(data{5})]);

        magData.sensorModels = data{1}(1:minLength);
        magData.sensorInstances = data{2}(1:minLength);
        magData.vectorX = data{3}(1:minLength);
        magData.vectorY = data{4}(1:minLength);
        magData.vectorZ = data{5}(1:minLength);

        fclose(fid);
    else
        warning('importMagnetometerLog:faileToOpenFile', 'Failed to open log file %s.\n', filename);
        status = false;
    end
end
