function plotBahrsData(objBahrsOutputRaw)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Plot height and vertical velocity
    SNavigationData = getFloatNavigationData(objBahrsOutputRaw);
    SInertialData = getFloatInertialData(objBahrsOutputRaw);
    SAccuracyData = getFloatAccuracyData(objBahrsOutputRaw);

    figure;
    subplot(2, 1, 1);
    maskValid = SNavigationData.pressureHeightValid;
    plot(SNavigationData.time(maskValid), SNavigationData.pressureHeight(maskValid));
    title('Pressure height and v_d');
    ylabel('Height, [m]');
    
    subplot(2, 1, 2);
    maskValid = SNavigationData.velocityDownValid;
    plot(SNavigationData.time(maskValid), SNavigationData.velocityDown(maskValid));
    ylabel('Vel D, [m/s]');
    xlabel('Time, [s]');

    %% Plot orientation
    SAccuracyDataInterp = interpolateAccuracy(SNavigationData, SAccuracyData);

    figure;
    subplot(3, 1, 1);
    maskValid = SNavigationData.rollValid;
    plot(SNavigationData.time(maskValid), SNavigationData.roll(maskValid));
    
    if ~isempty(SAccuracyDataInterp.attitudeStdN)
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.roll(maskValid) + 3 * SAccuracyDataInterp.attitudeStdN, 'r');
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.roll(maskValid) - 3 * SAccuracyDataInterp.attitudeStdN, 'r');
        grid on;
    end
    
    title('Attitude');
    legend('Signal', '\pm3\sigma');
    ylabel('Roll, [rad]');

    subplot(3, 1, 2);
    maskValid = SNavigationData.pitchValid;
    plot(SNavigationData.time(maskValid), SNavigationData.pitch(maskValid));
    
    if ~isempty(SAccuracyDataInterp.attitudeStdE)
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.pitch(maskValid) + 3 * SAccuracyDataInterp.attitudeStdE, 'r');
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.pitch(maskValid) - 3 * SAccuracyDataInterp.attitudeStdE, 'r');
    end
    
    grid on;
    ylabel('Pitch, [rad]');

    subplot(3, 1, 3);
    maskValid = SNavigationData.magneticHeadingValid;
    plot(SNavigationData.time(maskValid), SNavigationData.magneticHeading(maskValid));
    
    if ~isempty(SAccuracyDataInterp.magneticHeadingStd)
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.magneticHeading(maskValid) + 3 * SAccuracyDataInterp.magneticHeadingStd, 'r');
        hold on;
        plot(SNavigationData.time(maskValid), SNavigationData.magneticHeading(maskValid) - 3 * SAccuracyDataInterp.magneticHeadingStd, 'r');
    end
    
    grid on;
    ylabel('Magnetic Heading, [rad]');
    xlabel('Time, [s]');
    
    %% Plot specific force
    figure;
    subplot(3, 1, 1);
    maskValid = SInertialData.specificForceXValid;
    plot(SInertialData.time(maskValid), SInertialData.specificForceX(maskValid));
    grid on;
    title('Specific force');
    ylabel('X, [m/s^2]');

    subplot(3, 1, 2);
    maskValid = SInertialData.specificForceYValid;
    plot(SInertialData.time(maskValid), SInertialData.specificForceY(maskValid));
    grid on;
    ylabel('Y, [m/s^2]');
    
    subplot(3, 1, 3);
    maskValid = SInertialData.specificForceZValid;
    plot(SInertialData.time(maskValid), SInertialData.specificForceZ(maskValid));
    grid on;
    ylabel('Z, [m/s^2]');
    xlabel('Time, [s]');
    
    %% Plot angular rate
    figure;
    subplot(3, 1, 1);
    maskValid = SInertialData.angularRateXValid;
    plot(SInertialData.time(maskValid), SInertialData.angularRateX(maskValid));
    grid on;
    title('Angular rate');
    ylabel('X, [rad/s]');

    subplot(3, 1, 2);
    maskValid = SInertialData.angularRateYValid;
    plot(SInertialData.time(maskValid), SInertialData.angularRateY(maskValid));
    grid on;
    ylabel('Y, [rad/s]');
    
    subplot(3, 1, 3);
    maskValid = SInertialData.angularRateZValid;
    plot(SInertialData.time(maskValid), SInertialData.angularRateZ(maskValid));
    grid on;
    ylabel('Z, [rad/s]');
    xlabel('Time, [s]');

end

