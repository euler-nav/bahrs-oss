classdef ESignalHealth
    properties (Constant)
        Unavailable = uint16(bin2dec('00'));   % Signal is unavailable
        Safe = uint16(bin2dec('01'));          % Signal is safe
        IntegrityRisk = uint16(bin2dec('10')); % Signal may be misleading
    end
end

