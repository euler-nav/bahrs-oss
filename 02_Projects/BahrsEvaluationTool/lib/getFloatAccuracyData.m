function [objAccuracyData] = getFloatAccuracyData(objBahrsOutputRaw)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    SAccuracyDataMessage = objBahrsOutputRaw.SAccuracyDataMessage;

    objAccuracyData.attitudeStdN = 9.587526e-5 * double(SAccuracyDataMessage.uAttitudeStdN_);
    objAccuracyData.attitudeStdE = 9.587526e-5 * double(SAccuracyDataMessage.uAttitudeStdE_);
    objAccuracyData.magneticHeadingStd = 9.587526e-5 * double(SAccuracyDataMessage.uMagneticHeadingStd_);
    objAccuracyData.sequenceCounter = safeCounterUnwrap(SAccuracyDataMessage.uSequenceCounter_, 256);

    objAccuracyData.attitudeStdNValid = (SAccuracyDataMessage.uAttitudeStdN_ ~= 0);
    objAccuracyData.attitudeStdEValid = (SAccuracyDataMessage.uAttitudeStdE_ ~= 0);
    objAccuracyData.magneticHeadingStdValid = (SAccuracyDataMessage.uMagneticHeadingStd_ ~= 0);

    objAccuracyData.time = 1e-6 * double(SAccuracyDataMessage.uTimestampUs_);
end

