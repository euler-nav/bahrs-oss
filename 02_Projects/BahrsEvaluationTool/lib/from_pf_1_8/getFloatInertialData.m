function [objInertialData] = getFloatInertialData(objBahrsOutputRaw)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
    SInertialDataMessage = objBahrsOutputRaw.SInertialDataMessage;
    STimeOfInertialDataMessage = objBahrsOutputRaw.STimeOfInertialDataMessage;
    
    objInertialData.specificForceX = 1.495384e-3 * double(SInertialDataMessage.iSpecificForceX_);
    objInertialData.specificForceY = 1.495384e-3 * double(SInertialDataMessage.iSpecificForceY_);
    objInertialData.specificForceZ = 1.495384e-3 * double(SInertialDataMessage.iSpecificForceZ_);
    objInertialData.angularRateX = 1.597921e-4 * double(SInertialDataMessage.iAngularRateX_);
    objInertialData.angularRateY = 1.597921e-4 * double(SInertialDataMessage.iAngularRateY_);
    objInertialData.angularRateZ = 1.597921e-4 * double(SInertialDataMessage.iAngularRateZ_);
    objInertialData.sequenceCounter = safeCounterUnwrap(SInertialDataMessage.uSequenceCounter_, 256);
    
    mask = uint16(bin2dec('11'));
    objInertialData.specificForceXHealth = bitand(SInertialDataMessage.uHealthInfo_, mask);
    objInertialData.specificForceYHealth = bitshift(bitand(SInertialDataMessage.uHealthInfo_, bitshift(mask, 2)), -2);
    objInertialData.specificForceZHealth = bitshift(bitand(SInertialDataMessage.uHealthInfo_, bitshift(mask, 4)), -4);
    objInertialData.angularRateXHealth = bitshift(bitand(SInertialDataMessage.uHealthInfo_, bitshift(mask, 6)), -6);
    objInertialData.angularRateYHealth = bitshift(bitand(SInertialDataMessage.uHealthInfo_, bitshift(mask, 8)), -8);
    objInertialData.angularRateZHealth = bitshift(bitand(SInertialDataMessage.uHealthInfo_, bitshift(mask, 10)), -10);
    
    timeDataValid = STimeOfInertialDataMessage.uTimestampUs_ ~= 0;

    objInertialData.time = getTimeFromSequenceCounter(STimeOfInertialDataMessage.uInertialDataSequenceCounter_(timeDataValid),...
                                                      STimeOfInertialDataMessage.uTimestampUs_(timeDataValid),...
                                                      objInertialData.sequenceCounter);
end

