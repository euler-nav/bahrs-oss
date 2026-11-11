%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function plotSafeMagneticHeadingData(hdgData, dataName, filterData1, filterData2, filterData3)
%PLOTSAFEMAGNETICHEADINGDATA Plot safe magnetic heading port with health information.
%   Creates one figure with 3 subplots:
%     1. Safe magnetic heading [deg] with health markers
%     2. Outputs of the 3 redundant magnetic heading filters [deg] (optional)
%     3. Health status (discrete, dot plot)
%
%   Health classification (ESignalHealthInfo):
%     0 = unavailable    -- yellow square marker on signal
%     1 = safe           -- blue * marker on signal
%     2 = integrity risk -- red x marker on signal
%
%   hdgData     : struct from MAT file (PortSafeMagneticHeading)
%   dataName    : string label used in figure titles
%   filterData1 : (optional) struct from MAT file (PortMagneticHeading1)
%   filterData2 : (optional) struct from MAT file (PortMagneticHeading2)
%   filterData3 : (optional) struct from MAT file (PortMagneticHeading3)

  if ~isfield(hdgData, 'fMagneticHeading_') || isempty(hdgData.fMagneticHeading_)
    fprintf('No safe magnetic heading data available for %s, skipping.\n', dataName);
    return;
  end

  t0      = 1e-6 * double(hdgData.uTimestampUs_(1));
  t       = 1e-6 * double(hdgData.uTimestampUs_) - t0;
  heading = (180 / pi) * double(hdgData.fMagneticHeading_);
  health  = double(hdgData.eHealth_);

  maskSafe    = (health == 1);
  maskUnavail = (health == 0);
  maskRisk    = (health == 2);

  hasFilters = (nargin >= 5);

  figure;
  set(gcf, 'Units', 'pixels');
  set(gcf, 'Position', [100 100 1280 720]);
  set(gcf, 'PaperPositionMode', 'auto');
  localSgtitle(sprintf('%s  --  Safe magnetic heading (magnetic heading monitor output)', dataName));

  % ----- Safe heading -----
  subplot(3, 1, 1);
  hold on;
  plot(t, heading, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
  plotHealthMarkers(t, heading, maskSafe, maskUnavail, maskRisk);
  grid on;
  set(gca, 'FontSize', 13, 'LineWidth', 1.0);
  ylabel('Magnetic heading [deg]');
  xlabel('Time [s]');
  legend('Heading', 'Safe', 'Unavailable', 'Integrity risk', ...
         'Location', 'southoutside', 'Orientation', 'horizontal');

  % ----- Redundant filter outputs -----
  subplot(3, 1, 2);
  hold on;
  filterColors = {'b', 'r', [0 0.6 0]};
  filterInputs = {};
  if hasFilters
    filterInputs = {filterData1, filterData2, filterData3};
  end

  for i = 1:3
    if numel(filterInputs) >= i && isfield(filterInputs{i}, 'fMagneticHeading_') && ~isempty(filterInputs{i}.fMagneticHeading_)
      tf = 1e-6 * double(filterInputs{i}.uTimestampUs_) - t0;
      hf = (180 / pi) * double(filterInputs{i}.fMagneticHeading_);
      validMask = (double(filterInputs{i}.uValid_) ~= 0);
      plot(tf(validMask), hf(validMask), '.', 'Color', filterColors{i}, 'MarkerSize', 3);
    else
      plot(nan, nan, '.', 'Color', filterColors{i}, 'MarkerSize', 3);
    end
  end

  grid on;
  set(gca, 'FontSize', 13, 'LineWidth', 1.0);
  ylabel('Filter heading [deg]');
  xlabel('Time [s]');
  legend('Filter 1', 'Filter 2', 'Filter 3', ...
         'Location', 'southoutside', 'Orientation', 'horizontal');

  % ----- Health status -----
  subplot(3, 1, 3);
  hold on;
  plot(t, health, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
  grid on;
  set(gca, 'FontSize', 13, 'LineWidth', 1.0);
  ylim([-0.5, 2.5]);
  yticks([0 1 2]);
  yticklabels({'0-unavail', '1-safe', '2-risk'});
  ylabel('Health status');
  xlabel('Time [s]');

end

function plotHealthMarkers(t, sig, maskSafe, maskUnavail, maskRisk)
  if any(maskSafe)
    plot(t(maskSafe),    sig(maskSafe),    'b*', 'MarkerSize', 3);
  else
    plot(nan, nan, 'b*', 'MarkerSize', 3);
  end
  if any(maskUnavail)
    plot(t(maskUnavail), sig(maskUnavail), 'ys', 'MarkerSize', 5, 'MarkerFaceColor', 'y');
  else
    plot(nan, nan, 'ys', 'MarkerSize', 5, 'MarkerFaceColor', 'y');
  end
  if any(maskRisk)
    plot(t(maskRisk),    sig(maskRisk),    'rx', 'MarkerSize', 6, 'LineWidth', 1.5);
  else
    plot(nan, nan, 'rx', 'MarkerSize', 6, 'LineWidth', 1.5);
  end
end
