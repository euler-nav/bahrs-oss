function plotBarometerMonitorResults(port1, port2, port3, dataName)
%PLOTBAROMETERMONITORRESULTS Plot barometer monitor output via safe pressure ports.
%   Creates one figure with a 2x1 subplot layout:
%     Top subplot    -- pressure [Pa] for all 3 barometers, with validity indicator
%     Bottom subplot -- temperature [degC] for all 3 barometers, with validity indicator
%
%   The safe pressure ports carry the output of the barometer monitor:
%     uValid_ == 1  -- measurement is safe and passed to the filter
%     uValid_ == 0  -- measurement was excluded by the monitor
%
%   Each signal is plotted as coloured dots:
%     blue  * -- valid (safe, passed through)
%     red   x -- invalid (excluded by barometer monitor)
%
%   port1/2/3 : struct from MAT file (PortSafePressureData1/2/3)
%   dataName  : string label used in figure titles

  if ~isfield(port1, 'fPressure_')
    fprintf('No safe pressure data available for %s, skipping.\n', dataName);
    return;
  end

  ports       = {port1, port2, port3};
  portLabels  = {'Baro 1', 'Baro 2', 'Baro 3'};
  colors      = {[0 0.45 0.74], [0.85 0.33 0.10], [0.47 0.67 0.19]};  % blue, orange, green
  validStyle  = {'*', '*', '*'};
  invalidStyle = {'x', 'x', 'x'};

  figure;
  localSgtitle(sprintf('%s  --  Barometer monitor output (safe pressure ports)', dataName));

  subplot(2, 1, 1);
  hold on;
  grid on;
  ylabel('Pressure [Pa]');
  xlabel('Time [s]');

  subplot(2, 1, 2);
  hold on;
  grid on;
  ylabel('Temperature [deg C]');
  xlabel('Time [s]');

  legHandlesPress  = [];
  legLabelsPress   = {};
  legHandlesTemp   = [];
  legLabelsTemp    = {};

  for k = 1:3
    p        = ports{k};
    t        = 1e-6 * double(p.uTimestampUs_) - 1e-6 * double(p.uTimestampUs_(1));
    press    = double(p.fPressure_);
    temp     = double(p.fTemperature_);
    maskValid   = logical(p.uValid_ ~= 0);
    maskInvalid = ~maskValid;
    col      = colors{k};

    % ----- Pressure -----
    subplot(2, 1, 1);
    h = plot(nan, nan, validStyle{k}, 'Color', col, 'MarkerSize', 3);
    legHandlesPress(end + 1) = h;
    legLabelsPress{end + 1}  = sprintf('%s valid', portLabels{k});
    h = plot(nan, nan, invalidStyle{k}, 'Color', col, 'MarkerSize', 6, 'LineWidth', 1.5);
    legHandlesPress(end + 1) = h;
    legLabelsPress{end + 1}  = sprintf('%s excluded', portLabels{k});
    if any(maskValid)
      plot(t(maskValid), press(maskValid), validStyle{k}, ...
           'Color', col, 'MarkerSize', 3, 'HandleVisibility', 'off');
    end
    if any(maskInvalid)
      plot(t(maskInvalid), press(maskInvalid), invalidStyle{k}, ...
           'Color', col, 'MarkerSize', 6, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end

    % ----- Temperature -----
    subplot(2, 1, 2);
    h = plot(nan, nan, validStyle{k}, 'Color', col, 'MarkerSize', 3);
    legHandlesTemp(end + 1) = h;
    legLabelsTemp{end + 1}  = sprintf('%s valid', portLabels{k});
    h = plot(nan, nan, invalidStyle{k}, 'Color', col, 'MarkerSize', 6, 'LineWidth', 1.5);
    legHandlesTemp(end + 1) = h;
    legLabelsTemp{end + 1}  = sprintf('%s excluded', portLabels{k});
    if any(maskValid)
      plot(t(maskValid), temp(maskValid), validStyle{k}, ...
           'Color', col, 'MarkerSize', 3, 'HandleVisibility', 'off');
    end
    if any(maskInvalid)
      plot(t(maskInvalid), temp(maskInvalid), invalidStyle{k}, ...
           'Color', col, 'MarkerSize', 6, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
  end

  subplot(2, 1, 1);
  if ~isempty(legHandlesPress)
    legend(legHandlesPress, legLabelsPress, 'Location', 'northeast');
  end

  subplot(2, 1, 2);
  if ~isempty(legHandlesTemp)
    legend(legHandlesTemp, legLabelsTemp, 'Location', 'northeast');
  end

end
