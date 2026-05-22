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

    mask = uint16(bin2dec('11'));
    objNavigationData.pressureHeightHealth = bitand(SNavigationDataMessage.uHealthInfo_, mask);
    objNavigationData.velocityDownHealth = bitshift(bitand(SNavigationDataMessage.uHealthInfo_, bitshift(mask, 2)), -2);
    objNavigationData.rollHealth = bitshift(bitand(SNavigationDataMessage.uHealthInfo_, bitshift(mask, 4)), -4);
    objNavigationData.pitchHealth = bitshift(bitand(SNavigationDataMessage.uHealthInfo_, bitshift(mask, 6)), -6);
    objNavigationData.magneticHeadingHealth = bitshift(bitand(SNavigationDataMessage.uHealthInfo_, bitshift(mask, 8)), -8);

    timeDataValid = STimeOfNavigationDataMessage.uTimestampUs_ ~= 0;
    
    objNavigationData.time = getTimeFromSequenceCounter(STimeOfNavigationDataMessage.uNavigationDataSequenceCounter_(timeDataValid),...
                                                        STimeOfNavigationDataMessage.uTimestampUs_(timeDataValid),...
                                                        objNavigationData.sequenceCounter);
end

