%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function pressureResults = calibratePressureOffsets(matData, minSampleCount, driftWarnPa)
% Computes the pressure offsets of the three barometers from a recording.
%
% The offset of each sensor is the mean of all sensors' mean pressures minus
% the sensor's own mean pressure, so that the compensated pressures
% (p + offset) agree on average. Offsets are in Pa.
%
% The offsets are relative to the other sensors, hence if any sensor cannot
% be evaluated, no offset is trusted and all offsets are zero.
%
% pressureResults is a struct with the fields:
%   success         true if all three sensors could be evaluated
%   offsets         1x3 offsets in Pa
%   sensors         1x3 struct array: portName, validSampleCount, meanPa,
%                   stdPa, driftPa, errorMessage

  portNames = {'PortPressureInput1', 'PortPressureInput2', 'PortPressureInput3'};
  sensorCount = numel(portNames);
  driftFraction = 0.1;

  sensors = repmat(struct('portName', '', 'validSampleCount', 0, 'meanPa', NaN, ...
                          'stdPa', NaN, 'driftPa', NaN, 'errorMessage', ''), 1, sensorCount);

  for iSensor = 1:sensorCount
    portName = portNames{iSensor};
    sensors(iSensor).portName = portName;

    if false == isfield(matData, portName)
      sensors(iSensor).errorMessage = sprintf('%s not found in MAT file.', portName);
    elseif false == isfield(matData.(portName), 'fPressure_')
      sensors(iSensor).errorMessage = sprintf('%s has no fPressure_ field.', portName);
    else
      portData = matData.(portName);
      pressure = reshape(double(portData.fPressure_), 1, []);

      if isfield(portData, 'uValid_')
        validMask = reshape(double(portData.uValid_), 1, []) ~= 0.0;
      else
        validMask = true(1, numel(pressure));
      end

      pressure = pressure(validMask & isfinite(pressure));
      sampleCount = numel(pressure);
      sensors(iSensor).validSampleCount = sampleCount;

      if sampleCount < minSampleCount
        sensors(iSensor).errorMessage = sprintf('Only %d valid samples, at least %d required.', ...
                                                sampleCount, minSampleCount);
      else
        edgeCount = max(1, floor(driftFraction * sampleCount));
        sensors(iSensor).meanPa = mean(pressure);
        sensors(iSensor).stdPa = std(pressure);
        sensors(iSensor).driftPa = mean(pressure(end - edgeCount + 1:end)) - mean(pressure(1:edgeCount));
      end
    end
  end

  pressureResults = struct();
  pressureResults.sensors = sensors;
  pressureResults.success = all(cellfun(@isempty, {sensors.errorMessage}));
  pressureResults.offsets = zeros(1, sensorCount);

  if pressureResults.success
    meanPressures = [sensors.meanPa];
    pressureResults.offsets = mean(meanPressures) - meanPressures;
  end

  for iSensor = 1:sensorCount
    if isempty(sensors(iSensor).errorMessage)
      fprintf('  Pressure sensor %d (%s): samples=%d, mean=%.2f Pa, std=%.2f Pa, drift=%.2f Pa, offset=%.2f Pa\n', ...
              iSensor, sensors(iSensor).portName, sensors(iSensor).validSampleCount, sensors(iSensor).meanPa, ...
              sensors(iSensor).stdPa, sensors(iSensor).driftPa, pressureResults.offsets(iSensor));

      if abs(sensors(iSensor).driftPa) > driftWarnPa
        warning('calibratePressureOffsets:Drift', ...
                'Pressure sensor %d drifts by %.2f Pa (limit %.2f Pa). The recording may not be static.', ...
                iSensor, sensors(iSensor).driftPa, driftWarnPa);
      end
    else
      fprintf('  Pressure sensor %d (%s): FAILED - %s\n', iSensor, sensors(iSensor).portName, ...
              sensors(iSensor).errorMessage);
    end
  end

  if false == pressureResults.success
    warning('calibratePressureOffsets:Failed', 'Pressure calibration failed. All pressure offsets are set to zero.');
  end
end
