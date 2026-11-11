%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function exportNvmJson(calibrationResults, pressureOffsets, templatePath, outputPath)
% Writes a copy of the NVM map JSON whose default values of the magnetometer
% and pressure calibration blocks are the computed calibration parameters.
%
% Only the "defaultValue" arrays of HardIronOffset and SoftIronDistortion of
% the blocks MagnetometerCalibrationParameters1..3 and of PressureOffset of
% the block PressureCompensationParameters are replaced; the rest of the file
% is kept byte-identical. The soft-iron matrix is written row-major, as it is
% read by the firmware (afSoftIronDistortion_[row * 3 + column]).
% Sensors without a successful calibration get zero hard iron and identity
% soft iron.
%
% pressureOffsets is a 1x3 vector in Pa. If it is empty, the pressure block is
% left as in the template.

  if exist(templatePath, 'file') ~= 2
    error('exportNvmJson:TemplateNotFound', 'NVM map template not found: %s', templatePath);
  end

  jsonText = fileread(templatePath);

  for iSensor = 1:numel(calibrationResults)
    blockName = sprintf('MagnetometerCalibrationParameters%d', iSensor);

    if calibrationResults(iSensor).success
      hardIron = reshape(calibrationResults(iSensor).hardIron, 1, 3);
      softIronRowMajor = reshape(calibrationResults(iSensor).softIron.', 1, 9);
    else
      warning('exportNvmJson:SensorNotCalibrated', ...
              'Sensor %d: calibration failed or skipped (%s). Using zero hard iron and identity soft iron.', ...
              iSensor, calibrationResults(iSensor).errorId);
      hardIron = zeros(1, 3);
      softIronRowMajor = reshape(eye(3), 1, 9);
    end

    jsonText = replaceDefaultValue(jsonText, blockName, 'HardIronOffset', hardIron);
    jsonText = replaceDefaultValue(jsonText, blockName, 'SoftIronDistortion', softIronRowMajor);
  end

  if false == isempty(pressureOffsets)
    if numel(pressureOffsets) ~= 3
      error('exportNvmJson:InvalidPressureOffsets', 'pressureOffsets must contain exactly 3 elements.');
    end

    jsonText = replaceDefaultValue(jsonText, 'PressureCompensationParameters', 'PressureOffset', ...
                                   reshape(pressureOffsets, 1, 3));
  end

  fileId = fopen(outputPath, 'wb');
  if fileId < 0
    error('exportNvmJson:WriteFailed', 'Cannot open output file: %s', outputPath);
  end
  fwrite(fileId, jsonText, 'char');
  fclose(fileId);
end

function jsonText = replaceDefaultValue(jsonText, blockName, fieldName, values)
  if false == all(isfinite(values))
    error('exportNvmJson:NonFiniteValue', '%s.%s contains non-finite values.', blockName, fieldName);
  end

  [blockStart, ~] = regexp(jsonText, ['"name"\s*:\s*"' blockName '"'], 'start', 'end', 'once');
  if isempty(blockStart)
    error('exportNvmJson:BlockNotFound', 'Block %s not found in the NVM map template.', blockName);
  end

  % The CRC field is the last field of each block, so it delimits the search range.
  crcStart = regexp(jsonText(blockStart:end), '"name"\s*:\s*"Crc"', 'start', 'once');
  if isempty(crcStart)
    blockEnd = numel(jsonText);
  else
    blockEnd = blockStart + crcStart - 1;
  end

  fieldPattern = ['"name"\s*:\s*"' fieldName '"[^}]*?"defaultValue"\s*:\s*(\[[^\]]*\])'];
  [~, ~, tokenExtents] = regexp(jsonText(blockStart:blockEnd), fieldPattern, 'once');
  if isempty(tokenExtents)
    error('exportNvmJson:FieldNotFound', 'Field %s.%s with defaultValue not found in the NVM map template.', ...
          blockName, fieldName);
  end

  valueStart = blockStart - 1 + tokenExtents(1, 1);
  valueEnd = blockStart - 1 + tokenExtents(1, 2);
  jsonText = [jsonText(1:valueStart - 1), formatArray(values), jsonText(valueEnd + 1:end)];
end

function text = formatArray(values)
  parts = cell(1, numel(values));

  for iValue = 1:numel(values)
    part = sprintf('%.9g', values(iValue));
    if isempty(regexp(part, '[.eE]', 'once'))
      part = [part '.0'];
    end
    parts{iValue} = part;
  end

  text = ['[' strjoin(parts, ', ') ']'];
end
