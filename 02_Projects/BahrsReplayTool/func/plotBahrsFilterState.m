function plotBahrsFilterState(filterData, filterName)
%PLOTBAHRSFILTERSTATE Plot the full state of one BAHRS filter with 3-sigma bands.
%   PLOTBAHRSFILTERSTATE(filterData, filterName) creates 3 figures for
%   the given filter port data struct (PortBahrsFilterOutput1/2/3) loaded from
%   the debug MAT file. Data is masked to navigation mode (eFilterMode_ == 2).
%
%   filterData : struct from MAT file (PortBahrsFilterOutput1/2/3)
%   filterName : string label used in figure titles

  if ~isfield(filterData, 'uTimestampUs_') || isempty(filterData.uTimestampUs_)
    fprintf('No data available for %s, skipping.\n', filterName);
    return;
  end

  kSigma = 3;

  % Time vector relative to first recorded sample [s]
  time = 1e-6 * double(filterData.uTimestampUs_) - 1e-6 * double(filterData.uTimestampUs_(1));

  % Mask to navigation mode (eFilterMode_ == 2)
  activeMask = (filterData.eFilterMode_ == 2);

  if ~any(activeMask)
    fprintf('No navigation-mode samples found for %s, plotting all data.\n', filterName);
    activeMask = true(size(filterData.eFilterMode_));
  end

  t = time(activeMask);

  % ---------------------------------------------------------------------------
  % Extract state fields (scalar: 1xN, vector: 3xN or 4xN)
  % ---------------------------------------------------------------------------
  height   = filterData.oState_.fHeight_(activeMask);
  velDown  = filterData.oState_.fVelocityDown_(activeMask);
  quat       = filterData.oState_.oQuaternionBodyToNed_(:, activeMask);    % 4xN [W;X;Y;Z]
  accelBias  = filterData.oState_.oAccelerometerBias_(:, activeMask);      % 3xN [x;y;z]
  gyroBias   = filterData.oState_.oGyroscopeBias_(:, activeMask);          % 3xN [x;y;z]
  accelScale = filterData.oState_.oAccelerometerScaleFactor_(:, activeMask); % 3xN
  gyroScale  = filterData.oState_.oGyroscopeScaleFactor_(:, activeMask);     % 3xN
  accelNed   = filterData.oState_.oAccelerationNed_(:, activeMask);          % 3xN [N;E;D]

  % ---------------------------------------------------------------------------
  % Extract standard deviations (1-sigma)
  % ---------------------------------------------------------------------------
  heightStd   = filterData.oStateStd_.fHeight_(activeMask);
  velDownStd  = filterData.oStateStd_.fVelocityDown_(activeMask);
  attStd        = filterData.oStateStd_.oAttitude_(:, activeMask);                    % 3xN [roll;pitch;heading]
  accelBiasStd  = filterData.oStateStd_.oAccelerometerBias_(:, activeMask);
  gyroBiasStd   = filterData.oStateStd_.oGyroscopeBias_(:, activeMask);
  accelScaleStd = filterData.oStateStd_.oAccelerometerScaleFactor_(:, activeMask);
  gyroScaleStd  = filterData.oStateStd_.oGyroscopeScaleFactor_(:, activeMask);
  accelNedStd   = filterData.oStateStd_.oAccelerationNed_(:, activeMask);

  % ---------------------------------------------------------------------------
  % Convert quaternion to Euler angles [deg]
  % lib_quatToEulerV: input 4xN [W;X;Y;Z], output 3xN [roll;pitch;heading] in rad
  % ---------------------------------------------------------------------------
  euler    = lib_quatToEulerV(quat);        % 3xN [rad]
  eulerDeg = rad2deg(euler);                % 3xN [deg]
  attStdDeg = rad2deg(attStd);              % 3xN [deg]

  % ---------------------------------------------------------------------------
  % Figure 1: Vertical channel (2 subplots) + Attitude (3 subplots) = 5 subplots
  % ---------------------------------------------------------------------------
  figure;
  localSgtitle(sprintf('%s  --  Vertical channel & Attitude  (blue: estimate, red dashed: \\pm%d\\sigma)', filterName, kSigma));

  subplot(5, 1, 1);
  hold on;
  plot(t, height, 'b');
  plot(t, height + kSigma .* heightStd, 'r--');
  plot(t, height - kSigma .* heightStd, 'r--');
  grid on;
  ylabel('Height [m]');
  xlabel('Time [s]');

  subplot(5, 1, 2);
  hold on;
  plot(t, velDown, 'b');
  plot(t, velDown + kSigma .* velDownStd, 'r--');
  plot(t, velDown - kSigma .* velDownStd, 'r--');
  grid on;
  ylabel('Vel. down [m/s]');
  xlabel('Time [s]');

  attLabels = {'Roll [deg]', 'Pitch [deg]', 'Heading [deg]'};
  for k = 1:3
    subplot(5, 1, 2 + k);
    hold on;
    plot(t, eulerDeg(k, :), 'b');
    plot(t, eulerDeg(k, :) + kSigma .* attStdDeg(k, :), 'r--');
    plot(t, eulerDeg(k, :) - kSigma .* attStdDeg(k, :), 'r--');
    grid on;
    ylabel(attLabels{k});
    xlabel('Time [s]');
  end

  % ---------------------------------------------------------------------------
  % Figure 2: Biases and scale factors — 6 rows x 2 columns
  %   Left column  (odd subplots):  acc bias X/Y/Z, gyro bias X/Y/Z
  %   Right column (even subplots): acc scale X/Y/Z, gyro scale X/Y/Z
  % ---------------------------------------------------------------------------
  figure;
  localSgtitle(sprintf('%s  --  Biases & scale factors  (blue: estimate, red dashed: \\pm%d\\sigma)', filterName, kSigma));

  axisLabels = {'X', 'Y', 'Z'};
  for k = 1:3
    subplot(6, 2, 2 * k - 1);
    hold on;
    plot(t, accelBias(k, :), 'b');
    plot(t, accelBias(k, :) + kSigma .* accelBiasStd(k, :), 'r--');
    plot(t, accelBias(k, :) - kSigma .* accelBiasStd(k, :), 'r--');
    grid on;
    ylabel(sprintf('Acc. bias %s [m/s^2]', axisLabels{k}));
    xlabel('Time [s]');

    subplot(6, 2, 2 * k);
    hold on;
    plot(t, accelScale(k, :), 'b');
    plot(t, accelScale(k, :) + kSigma .* accelScaleStd(k, :), 'r--');
    plot(t, accelScale(k, :) - kSigma .* accelScaleStd(k, :), 'r--');
    grid on;
    ylabel(sprintf('Acc. scale %s [-]', axisLabels{k}));
    xlabel('Time [s]');
  end

  for k = 1:3
    subplot(6, 2, 6 + 2 * k - 1);
    hold on;
    plot(t, gyroBias(k, :), 'b');
    plot(t, gyroBias(k, :) + kSigma .* gyroBiasStd(k, :), 'r--');
    plot(t, gyroBias(k, :) - kSigma .* gyroBiasStd(k, :), 'r--');
    grid on;
    ylabel(sprintf('Gyro bias %s [rad/s]', axisLabels{k}));
    xlabel('Time [s]');

    subplot(6, 2, 6 + 2 * k);
    hold on;
    plot(t, gyroScale(k, :), 'b');
    plot(t, gyroScale(k, :) + kSigma .* gyroScaleStd(k, :), 'r--');
    plot(t, gyroScale(k, :) - kSigma .* gyroScaleStd(k, :), 'r--');
    grid on;
    ylabel(sprintf('Gyro scale %s [-]', axisLabels{k}));
    xlabel('Time [s]');
  end

  % ---------------------------------------------------------------------------
  % Figure 3: Acceleration NED (external, non-gravitational)
  % ---------------------------------------------------------------------------
  figure;
  localSgtitle(sprintf('%s  --  Acceleration NED  (blue: estimate, red dashed: \\pm%d\\sigma)', filterName, kSigma));

  accelNedLabels = {'Acc. NED North [m/s^2]', 'Acc. NED East [m/s^2]', 'Acc. NED Down [m/s^2]'};
  for k = 1:3
    subplot(3, 1, k);
    hold on;
    plot(t, accelNed(k, :), 'b');
    plot(t, accelNed(k, :) + kSigma .* accelNedStd(k, :), 'r--');
    plot(t, accelNed(k, :) - kSigma .* accelNedStd(k, :), 'r--');
    grid on;
    ylabel(accelNedLabels{k});
    xlabel('Time [s]');
  end

end
