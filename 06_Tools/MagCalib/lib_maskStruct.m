%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function structOut = lib_maskStruct(structIn, booleanMask)
%lib_maskStruct Mask a structure
%   Detailed explanation goes here

    fieldNames = fieldnames(structIn);
    
    for i=1:1:length(fieldNames)
        structOut.(fieldNames{i}) = structIn.(fieldNames{i})(booleanMask);
    end
end

