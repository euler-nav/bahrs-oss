function plotSafeAttitudeData(attData, dataName)
%PLOTSAFEATTITUDEDATA Plot safe vehicle attitude port with health information.
%   Creates one figure with 3 subplots:
%     1. Roll  [deg] with +-3sigma bands
%     2. Pitch [deg] with +-3sigma bands
%     3. Health status (discrete, dot plot)
%
%   Health classification (ESignalHealthInfo):
%     0 = unavailable   -- yellow square marker on signal
%     1 = safe          -- blue * marker on signal
%     2 = integrity risk -- red x marker on signal
%
%   Standard deviations are assumed to correspond to roll (fAttitudeStd1_)
%   and pitch (fAttitudeStd2_) in that order, matching the field ordering.
%
%   attData  : struct from MAT file (PortSafeAttitudeData)
%   dataName : string label used in figure titles

  if ~isfield(attData, 'fRoll_') || isempty(attData.fRoll_)
    fprintf('No safe attitude data available for %s, skipping.\n', dataName);
    return;
  end

  kSigma = 3;

  t        = 1e-6 * double(attData.uTimestampUs_) - 1e-6 * double(attData.uTimestampUs_(1));
  rollDeg  = rad2deg(double(attData.fRoll_));
  pitchDeg = rad2deg(double(attData.fPitch_));
  std1Deg  = rad2deg(double(attData.fAttitudeStd1_));   % roll 1-sigma [deg]
  std2Deg  = rad2deg(double(attData.fAttitudeStd2_));   % pitch 1-sigma [deg]
  health   = double(attData.eHealth_);

  maskSafe    = (health == 1);
  maskUnavail = (health == 0);
  maskRisk    = (health == 2);

  figure;
  localSgtitle(sprintf('%s  --  Safe vehicle attitude (attitude monitor output)', dataName));

  % ----- Roll -----
  subplot(3, 1, 1);
  hold on;
  plot(t, rollDeg,                             'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
  plot(t, rollDeg + kSigma .* std1Deg,         'r--', 'LineWidth', 0.8);
  plot(t, rollDeg - kSigma .* std1Deg,         'r--', 'LineWidth', 0.8);
  plotHealthMarkers(t, rollDeg, maskSafe, maskUnavail, maskRisk);
  grid on;
  ylabel('Roll [deg]');
  xlabel('Time [s]');
  legend('Roll', sprintf('+/-%d\\sigma', kSigma), '', ...
         'Safe', 'Unavailable', 'Integrity risk', ...
         'Location', 'southoutside', 'Orientation', 'horizontal');

  % ----- Pitch -----
  subplot(3, 1, 2);
  hold on;
  plot(t, pitchDeg,                            'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
  plot(t, pitchDeg + kSigma .* std2Deg,        'r--', 'LineWidth', 0.8);
  plot(t, pitchDeg - kSigma .* std2Deg,        'r--', 'LineWidth', 0.8);
  plotHealthMarkers(t, pitchDeg, maskSafe, maskUnavail, maskRisk);
  grid on;
  ylabel('Pitch [deg]');
  xlabel('Time [s]');

  % ----- Health status -----
  subplot(3, 1, 3);
  hold on;
  plot(t, health, 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b');
  grid on;
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
    % Invisible placeholder to keep legend entry count consistent on subplot 1
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

