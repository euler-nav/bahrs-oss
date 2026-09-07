function plotImuMonitorResults(imuData, dataName)
%PLOTIMUMONITORRESULTS Plot IMU monitor output signals with health annotations.
%   Creates one figure with a 6x2 subplot layout:
%     Left column  -- signal value with per-sample health markers
%     Right column -- detection and isolation results as discrete step plots
%
%   Signal health classification:
%     Safe      -- detection good (==2), OR fault detected (==3) with isolation
%                  succeeded (==2). Marker: blue *
%     Unavailable-- detection invalid (==0) or unavailable (==1).
%                  Marker: yellow square
%     Unsafe    -- fault detected (==3) and isolation not succeeded.
%                  Marker: red x
%
%   Enum reference (CommonMonitorTypes.h / MeasurementStructs.h):
%     EDetectionResult:  0=invalid, 1=unavailable, 2=good, 3=failure
%     EIsolationResult:  0=invalid, 1=unavailable, 2=good, 3=failed
%
%   imuData  : struct from MAT file (PortImuDataAfterMonitor)
%   dataName : string label used in figure titles

  if ~isfield(imuData, 'oSpecificForceX_')
    fprintf('No IMU monitor data available for %s, skipping.\n', dataName);
    return;
  end

  allChannels = {imuData.oSpecificForceX_, imuData.oSpecificForceY_, imuData.oSpecificForceZ_, ...
                 imuData.oAngularRateX_,   imuData.oAngularRateY_,   imuData.oAngularRateZ_};
  allLabels   = {'Accel X [m/s^2]', 'Accel Y [m/s^2]', 'Accel Z [m/s^2]', ...
                 'Gyro X [rad/s]', 'Gyro Y [rad/s]', 'Gyro Z [rad/s]'};

  figure;
  localSgtitle(sprintf('%s  --  IMU monitor output', dataName));

  for k = 1:6
    ch        = allChannels{k};
    t         = 1e-6 * double(ch.uTimestampUs_) - 1e-6 * double(ch.uTimestampUs_(1));
    sig       = double(ch.fSignal_);
    detResult = double(ch.eDetectionResults_);
    isoResult = double(ch.eIsolationResults_);

    % Health masks
    maskSafe    = (detResult == 2) | (detResult == 3 & isoResult == 2);
    maskUnavail = (detResult == 0) | (detResult == 1);
    maskUnsafe  = ~maskSafe & ~maskUnavail;

    % ----- Left column: signal with health markers -----
    subplot(6, 2, 2 * k - 1);
    hold on;
    hSignal = plot(t, sig, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
    hSafe = plot(nan, nan, 'b*', 'MarkerSize', 3);
    hUnavail = plot(nan, nan, 'ys', 'MarkerSize', 4, 'MarkerFaceColor', 'y');
    hUnsafe = plot(nan, nan, 'rx', 'MarkerSize', 6, 'LineWidth', 1.5);
    if any(maskSafe)
      plot(t(maskSafe), sig(maskSafe), 'b*', 'MarkerSize', 3, 'HandleVisibility', 'off');
    end
    if any(maskUnavail)
      plot(t(maskUnavail), sig(maskUnavail), 'ys', 'MarkerSize', 4, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    end
    if any(maskUnsafe)
      plot(t(maskUnsafe), sig(maskUnsafe), 'rx', 'MarkerSize', 6, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    grid on;
    ylabel(allLabels{k});
    xlabel('Time [s]');
    if k == 1
      legend([hSignal hSafe hUnavail hUnsafe], {'Signal', 'Safe', 'Unavailable', 'Unsafe'}, ...
             'Location', 'southoutside', 'Orientation', 'horizontal');
    end

    % ----- Right column: detection and isolation results -----
    subplot(6, 2, 2 * k);
    hold on;
    plot(t, detResult, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
    plot(t, isoResult, 'rx', 'MarkerSize', 4, 'LineWidth', 1.2);
    grid on;
    ylim([-0.5, 3.5]);
    yticks([0 1 2 3]);
    yticklabels({'0-inv', '1-unavail', '2-good', '3-fail'});
    ylabel('Health info');
    xlabel('Time [s]');
    if k == 1
      legend('Detection', 'Isolation', 'Location', 'southoutside', 'Orientation', 'horizontal');
    end
  end

end
