function [SAccuracyDataOut] = interpolateAccuracy(SNavigationData, SAccuracyData)
%INTERPOLATEACCURACY Summary of this function goes here
%   Detailed explanation goes here
    maskValid = SNavigationData.rollValid;
    maskStdValid = SAccuracyData.attitudeStdNValid;
    SAccuracyDataOut.attitudeStdN = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.attitudeStdN = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.attitudeStdN(maskStdValid), SNavigationData.time(maskValid), 'nearest', 'extrap');
    end

    maskValid = SNavigationData.pitchValid;
    maskStdValid = SAccuracyData.attitudeStdEValid;
    SAccuracyDataOut.attitudeStdE = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.attitudeStdE = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.attitudeStdE(maskStdValid), SNavigationData.time(maskValid), 'nearest', 'extrap');
    end

    maskValid = SNavigationData.magneticHeadingValid;
    maskStdValid = SAccuracyData.magneticHeadingStdValid;
    SAccuracyDataOut.magneticHeadingStd = [];
    
    if any(maskStdValid)
        SAccuracyDataOut.magneticHeadingStd = interp1(SAccuracyData.time(maskStdValid), SAccuracyData.magneticHeadingStd(maskStdValid), SNavigationData.time(maskValid), 'nearest', 'extrap');
    end
end

