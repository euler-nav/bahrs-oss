function plotBahrsData(objBahrsOutputRaw)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Plot height and vertical velocity
    SNavigationData = getFloatNavigationData(objBahrsOutputRaw);
    SInertialData = getFloatInertialData(objBahrsOutputRaw);
    SAccuracyData = getFloatAccuracyData(objBahrsOutputRaw);

    figure;
    subplot(2, 1, 1);
    plotSafeSignal(SNavigationData.time, SNavigationData.pressureHeight, SNavigationData.pressureHeightHealth, true);
    title('Pressure height and v_d');
    ylabel('Height, [m]');
    
    subplot(2, 1, 2);
    plotSafeSignal(SNavigationData.time, SNavigationData.velocityDown, SNavigationData.velocityDownHealth, true);
    ylabel('Vel D, [m/s]');
    xlabel('Time, [s]');

    %% Plot orientation
    SAccuracyDataInterp = interpolateAccuracy(SNavigationData, SAccuracyData);

    figure;
    subplot(3, 1, 1);
    legendEntries = plotSafeSignal(SNavigationData.time, SNavigationData.roll, SNavigationData.rollHealth, true);

    if ~isempty(SAccuracyDataInterp.attitudeStdN)
        hold on;
        plot(SAccuracyDataInterp.timeAttitudeStdN, SAccuracyDataInterp.rollRef + 3 * SAccuracyDataInterp.attitudeStdN, 'r');
        hold on;
        plot(SAccuracyDataInterp.timeAttitudeStdN, SAccuracyDataInterp.rollRef - 3 * SAccuracyDataInterp.attitudeStdN, 'r');
        grid on;
        legendEntries{end+1} = 'Signal \pm 3\sigma';
        legend(legendEntries);
    end
    
    title('Attitude');
    ylabel('Roll, [rad]');

    subplot(3, 1, 2);
    legendEntries = plotSafeSignal(SNavigationData.time, SNavigationData.pitch, SNavigationData.pitchHealth, true);

    if ~isempty(SAccuracyDataInterp.attitudeStdE)
        hold on;
        plot(SAccuracyDataInterp.timeAttitudeStdE, SAccuracyDataInterp.pitchRef + 3 * SAccuracyDataInterp.attitudeStdE, 'r');
        hold on;
        plot(SAccuracyDataInterp.timeAttitudeStdE, SAccuracyDataInterp.pitchRef - 3 * SAccuracyDataInterp.attitudeStdE, 'r');
        legendEntries{end+1} = 'Signal \pm 3\sigma';
        legend(legendEntries);
    end
    
    grid on;
    ylabel('Pitch, [rad]');

    subplot(3, 1, 3);
    legendEntries = plotSafeSignal(SNavigationData.time, SNavigationData.magneticHeading, SNavigationData.magneticHeadingHealth, true);

    if ~isempty(SAccuracyDataInterp.magneticHeadingStd)
        hold on;
        plot(SAccuracyDataInterp.timeMagneticHeadingStd, SAccuracyDataInterp.magneticHeadingRef + 3 * SAccuracyDataInterp.magneticHeadingStd, 'r');
        hold on;
        plot(SAccuracyDataInterp.timeMagneticHeadingStd, SAccuracyDataInterp.magneticHeadingRef - 3 * SAccuracyDataInterp.magneticHeadingStd, 'r');
        legendEntries{end+1} = 'Signal \pm 3\sigma';
        legend(legendEntries);
    end
    
    grid on;
    ylabel('Magnetic Heading, [rad]');
    xlabel('Time, [s]');
    
    %% Plot specific force
    figure;
    subplot(3, 1, 1);
    plotSafeSignal(SInertialData.time, SInertialData.specificForceX, SInertialData.specificForceXHealth, true);

    grid on;
    title('Specific force');
    ylabel('X, [m/s^2]');

    subplot(3, 1, 2);
    plotSafeSignal(SInertialData.time, SInertialData.specificForceY, SInertialData.specificForceYHealth, true);

    grid on;
    ylabel('Y, [m/s^2]');
    
    subplot(3, 1, 3);
    plotSafeSignal(SInertialData.time, SInertialData.specificForceZ, SInertialData.specificForceZHealth, true);

    grid on;
    ylabel('Z, [m/s^2]');
    xlabel('Time, [s]');
    
    %% Plot angular rate
    figure;
    subplot(3, 1, 1);
    plotSafeSignal(SInertialData.time, SInertialData.angularRateX, SInertialData.angularRateXHealth, true);

    grid on;
    title('Angular rate');
    ylabel('X, [rad/s]');

    subplot(3, 1, 2);
    plotSafeSignal(SInertialData.time, SInertialData.angularRateY, SInertialData.angularRateYHealth, true);

    grid on;
    ylabel('Y, [rad/s]');
    
    subplot(3, 1, 3);
    plotSafeSignal(SInertialData.time, SInertialData.angularRateZ, SInertialData.angularRateZHealth, true);

    grid on;
    ylabel('Z, [rad/s]');
    xlabel('Time, [s]');

end

