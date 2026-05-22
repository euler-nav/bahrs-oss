function [SAccuracyDataOut] = interpolateAccuracy(SNavigationData, SAccuracyData)
%INTERPOLATEACCURACY Summary of this function goes here
%   Detailed explanation goes here
    maskAvailable = (SNavigationData.rollHealth == ESignalHealth.Safe) | (SNavigationData.rollHealth == ESignalHealth.IntegrityRisk);
    maskStdValid = SAccuracyData.attitudeStdNValid;
    SAccuracyDataOut.attitudeStdN = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.attitudeStdN = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.attitudeStdN(maskStdValid), SNavigationData.time(maskAvailable), 'nearest', 'extrap');
        SAccuracyDataOut.timeAttitudeStdN = SNavigationData.time(maskAvailable);
        SAccuracyDataOut.rollRef = SNavigationData.roll(maskAvailable);
    end

    maskAvailable = (SNavigationData.pitchHealth == ESignalHealth.Safe) | (SNavigationData.pitchHealth == ESignalHealth.IntegrityRisk);
    maskStdValid = SAccuracyData.attitudeStdEValid;
    SAccuracyDataOut.attitudeStdE = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.attitudeStdE = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.attitudeStdE(maskStdValid), SNavigationData.time(maskAvailable), 'nearest', 'extrap');
        SAccuracyDataOut.timeAttitudeStdE = SNavigationData.time(maskAvailable);
        SAccuracyDataOut.pitchRef = SNavigationData.pitch(maskAvailable);
    end

    maskAvailable = (SNavigationData.magneticHeadingHealth == ESignalHealth.Safe) | (SNavigationData.magneticHeadingHealth == ESignalHealth.IntegrityRisk);
    maskStdValid = SAccuracyData.magneticHeadingStdValid;
    SAccuracyDataOut.magneticHeadingStd = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.magneticHeadingStd = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.magneticHeadingStd(maskStdValid), SNavigationData.time(maskAvailable), 'nearest', 'extrap');
        SAccuracyDataOut.timeMagneticHeadingStd = SNavigationData.time(maskAvailable);
        SAccuracyDataOut.magneticHeadingRef = SNavigationData.magneticHeading(maskAvailable);
    end
end

