%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function evaluateResults(simOutput)
%evaluateResults Summary of this function goes here
%   Detailed explanation goes here


    fieldNames = fieldnames(simOutput);
    
    for i=1:1:length(fieldNames)
        simOutputMasked.(fieldNames{i}) = simOutput.(fieldNames{i})(simOutput.FilterMode == 2, :);
    end
    
    timeMasked = 1e-6 * simOutputMasked.TimeUs;
    
    %% Plot height and vertical velocity

    figure;
    subplot(2, 1, 1);
    plot(timeMasked, simOutputMasked.Height);
    hold on;
    plot(timeMasked, simOutputMasked.Height + 3 * simOutputMasked.HeightStd, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.Height - 3 * simOutputMasked.HeightStd, 'r');
    grid on;
    ylabel('Height, [m]');
    legend('Signal', '\pm 3 \sigma');
    
    subplot(2, 1, 2);
    plot(timeMasked, simOutputMasked.VelocityDown);
    hold on;
    plot(timeMasked, simOutputMasked.VelocityDown + 3 * simOutputMasked.VelocityDownStd, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.VelocityDown - 3 * simOutputMasked.VelocityDownStd, 'r');
    grid on;
    ylabel('Vel D, [m/s]');
    xlabel('Time, [s]');
    
    %% Plot orientation
    eulerMasked = lib_quatToEulerV([simOutputMasked.QuatBodyToNedW, simOutputMasked.QuatBodyToNedX, simOutputMasked.QuatBodyToNedY, simOutputMasked.QuatBodyToNedZ]');
    
    figure;
    subplot(3, 1, 1);
    plot(timeMasked, eulerMasked(1, :));
    hold on;
    plot(timeMasked, eulerMasked(1, :) + 3 * simOutputMasked.AttitudeStd1', 'r');
    hold on;
    plot(timeMasked, eulerMasked(1, :) - 3 * simOutputMasked.AttitudeStd1', 'r');
    grid on;
    ylabel('Roll, [rad]');
    legend('Signal', '\pm 3 \sigma');

    subplot(3, 1, 2);
    plot(timeMasked, eulerMasked(2, :));
    hold on;
    plot(timeMasked, eulerMasked(2, :) + 3 * simOutputMasked.AttitudeStd2', 'r');
    hold on;
    plot(timeMasked, eulerMasked(2, :) - 3 * simOutputMasked.AttitudeStd2', 'r');
    grid on;
    ylabel('Pitch, [rad]');
    
    subplot(3, 1, 3);
    plot(timeMasked, eulerMasked(3, :));
    hold on;
    plot(timeMasked, eulerMasked(3, :) + 3 * simOutputMasked.AttitudeStd3', 'r');
    hold on;
    plot(timeMasked, eulerMasked(3, :) - 3 * simOutputMasked.AttitudeStd3', 'r');
    grid on;
    ylabel('Yaw, [rad]');
    xlabel('Time, [s]');
    
    %% Accelerometer offsets
    
    figure;
    subplot(3, 1, 1);
    plot(timeMasked, simOutputMasked.AccelBiasX);
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasX + 3 * simOutputMasked.AccelBiasStdX, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasX - 3 * simOutputMasked.AccelBiasStdX, 'r');
    grid on;
    title('Accelerometer offsets, [m/s^2]');
    legend('Estimate', '\pm 3 \sigma');
    
    subplot(3, 1, 2);
    plot(timeMasked, simOutputMasked.AccelBiasY);
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasY + 3 * simOutputMasked.AccelBiasStdY, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasY - 3 * simOutputMasked.AccelBiasStdY, 'r');
    grid on;
    
    subplot(3, 1, 3);
    plot(timeMasked, simOutputMasked.AccelBiasZ);
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasZ + 3 * simOutputMasked.AccelBiasStdZ, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.AccelBiasZ - 3 * simOutputMasked.AccelBiasStdZ, 'r');
    grid on;
    xlabel('Time, [s]');
    
    %% Plot gyroscope offsets
    
    figure;
    subplot(3, 1, 1);
    plot(timeMasked, simOutputMasked.GyroBiasX);
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasX + 3 * simOutputMasked.GyroBiasStdX, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasX - 3 * simOutputMasked.GyroBiasStdX, 'r');
    grid on;
    title('Gyroscope offsets, [rad/s]');
    legend('Estimate', '\pm 3 \sigma');
    
    subplot(3, 1, 2);
    plot(timeMasked, simOutputMasked.GyroBiasY);
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasY + 3 * simOutputMasked.GyroBiasStdY, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasY - 3 * simOutputMasked.GyroBiasStdY, 'r');
    grid on;
    
    subplot(3, 1, 3);
    plot(timeMasked, simOutputMasked.GyroBiasZ);
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasZ + 3 * simOutputMasked.GyroBiasStdZ, 'r');
    hold on;
    plot(timeMasked, simOutputMasked.GyroBiasZ - 3 * simOutputMasked.GyroBiasStdZ, 'r');
    grid on;
    xlabel('Time, [s]');

end

