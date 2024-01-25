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

    objInertialData.specificForceXValid = logical(bitget(SInertialDataMessage.uValidity_, 1));
    objInertialData.specificForceYValid = logical(bitget(SInertialDataMessage.uValidity_, 2));
    objInertialData.specificForceZValid = logical(bitget(SInertialDataMessage.uValidity_, 3));
    objInertialData.angularRateXValid = logical(bitget(SInertialDataMessage.uValidity_, 4));
    objInertialData.angularRateYValid = logical(bitget(SInertialDataMessage.uValidity_, 5));
    objInertialData.angularRateZValid = logical(bitget(SInertialDataMessage.uValidity_, 6));
    
    timeDataValid = STimeOfInertialDataMessage.uTimestampUs_ ~= 0;

    objInertialData.time = getTimeFromSequenceCounter(STimeOfInertialDataMessage.uInertialDataSequenceCounter_(timeDataValid),...
                                                      STimeOfInertialDataMessage.uTimestampUs_(timeDataValid),...
                                                      objInertialData.sequenceCounter);
end

