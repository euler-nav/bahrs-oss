%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function errorSquared = calibFunction(calibParams, measurements, referenceMagneticFieldNed)
  hardIron = calibParams(1:3);
  deltaSoftIron = reshape(calibParams(4:12), 3, 3);
  softIron = eye(3) + deltaSoftIron;

  sampleCount = size(measurements, 2);
  referenceNorm = norm(referenceMagneticFieldNed);
  errorSquared = 0.0;

  for iSample = 1:sampleCount
    compensatedMeasurement = softIron * (measurements(:, iSample) + hardIron);
    normError = referenceNorm - norm(compensatedMeasurement);
    errorSquared = errorSquared + normError * normError;
  end
end
