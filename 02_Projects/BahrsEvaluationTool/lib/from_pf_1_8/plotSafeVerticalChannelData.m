function plotSafeVerticalChannelData(vcData, dataName)
%PLOTSAFEVERTICACLCHANNELDATA Plot safe vertical channel port with health information.
%   Creates one figure with 3 subplots:
%     1. Height        [m]   with health markers
%     2. Velocity down [m/s] with health markers
%     3. Health status (discrete, dot plot)
%
%   Health classification (ESignalHealthInfo):
%     0 = unavailable    -- yellow square marker on signal
%     1 = safe           -- blue * marker on signal
%     2 = integrity risk -- red x marker on signal
%
%   vcData   : struct from MAT file (PortSafeVerticalChannelData)
%   dataName : string label used in figure titles

  if ~isfield(vcData, 'fHeight_') || isempty(vcData.fHeight_)
    fprintf('No safe vertical channel data available for %s, skipping.\n', dataName);
    return;
  end

  t       = 1e-6 * double(vcData.uTimestampUs_) - 1e-6 * double(vcData.uTimestampUs_(1));
  height  = double(vcData.fHeight_);
  velDown = double(vcData.fVelocityDown_);
  health  = double(vcData.eHealth_);

  maskSafe    = (health == 1);
  maskUnavail = (health == 0);
  maskRisk    = (health == 2);

  figure;
  localSgtitle(sprintf('%s  --  Safe vertical channel (vertical channel monitor output)', dataName));

  % ----- Height -----
  subplot(3, 1, 1);
  hold on;
  plot(t, height, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
  plotHealthMarkers(t, height, maskSafe, maskUnavail, maskRisk);
  grid on;
  ylabel('Height [m]');
  xlabel('Time [s]');
  legend('Height', 'Safe', 'Unavailable', 'Integrity risk', ...
         'Location', 'southoutside', 'Orientation', 'horizontal');

  % ----- Velocity down -----
  subplot(3, 1, 2);
  hold on;
  plot(t, velDown, 'Color', [0.75 0.75 0.75], 'LineWidth', 0.5);
  plotHealthMarkers(t, velDown, maskSafe, maskUnavail, maskRisk);
  grid on;
  ylabel('Velocity down [m/s]');
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

