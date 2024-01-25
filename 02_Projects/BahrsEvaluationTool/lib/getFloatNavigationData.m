function [objNavigationData] = getFloatNavigationData(objBahrsOutputRaw)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    SNavigationDataMessage = objBahrsOutputRaw.SNavigationDataMessage;
    STimeOfNavigationDataMessage = objBahrsOutputRaw.STimeOfNavigationDataMessage;

    objNavigationData.pressureHeight = 0.16784924 * double(SNavigationDataMessage.uPressureHeight_) - 1000;
    objNavigationData.velocityDown = 9.155413e-3 * double(SNavigationDataMessage.iVelocityDown_);
    objNavigationData.roll = 9.587526e-5 * double(SNavigationDataMessage.iRoll_);
    objNavigationData.pitch = 9.587526e-5 * double(SNavigationDataMessage.iPitch_);
    objNavigationData.magneticHeading = 9.587526e-5 * double(SNavigationDataMessage.uMagneticHeading_);
    objNavigationData.sequenceCounter = safeCounterUnwrap(SNavigationDataMessage.uSequenceCounter_, 256);

    objNavigationData.pressureHeightValid = logical(bitget(SNavigationDataMessage.uValidity_, 1));
    objNavigationData.velocityDownValid = logical(bitget(SNavigationDataMessage.uValidity_, 2));
    objNavigationData.rollValid = logical(bitget(SNavigationDataMessage.uValidity_, 3));
    objNavigationData.pitchValid = logical(bitget(SNavigationDataMessage.uValidity_, 4));
    objNavigationData.magneticHeadingValid = logical(bitget(SNavigationDataMessage.uValidity_, 5));

    timeDataValid = STimeOfNavigationDataMessage.uTimestampUs_ ~= 0;
    
    objNavigationData.time = getTimeFromSequenceCounter(STimeOfNavigationDataMessage.uNavigationDataSequenceCounter_(timeDataValid),...
                                                        STimeOfNavigationDataMessage.uTimestampUs_(timeDataValid),...
                                                        objNavigationData.sequenceCounter);
end

