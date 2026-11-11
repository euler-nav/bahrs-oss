%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function output = importBahrsOutputFile(filename)
    VariableNames = {"TimeUs", "FilterMode", "Height", "VelocityDown", "QuatBodyToNedW", "QuatBodyToNedX", "QuatBodyToNedY", "QuatBodyToNedZ", "AccelBiasX", "AccelBiasY", "AccelBiasZ", "GyroBiasX", "GyroBiasY", "GyroBiasZ", "HeightStd", "VelocityDownStd", "AttitudeStd1", "AttitudeStd2", "AttitudeStd3", "AccelBiasStdX", "AccelBiasStdY", "AccelBiasStdZ", "GyroBiasStdX", "GyroBiasStdY", "GyroBiasStdZ"}; %#ok<CLARRSTR>

    importedData = importdata(filename, ';', 1);

    for i=1:1:length(VariableNames)
        output.(VariableNames{i}) = importedData.data(:, i);
    end
end