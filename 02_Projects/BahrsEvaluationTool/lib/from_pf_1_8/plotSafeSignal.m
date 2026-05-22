function [legendEntries] = plotSafeSignal(time, signal, signalHealth, plotLegend)
% Validate input arguments
    assert(isvector(time) && isvector(signal) && isvector(signalHealth), ...
        'time, signal, and signalHealth must be vectors.');
    assert(length(time) == length(signal) && length(signal) == length(signalHealth), ...
        'time, signal, and signalHealth must have the same length.');
    assert(islogical(plotLegend) && isscalar(plotLegend), ...
        'plotLegend must be a logical scalar.');
    assert(isa(signalHealth, 'uint16'), ...
        'signalHealth must be of type uint16.');

    blue = '#0072BD';
    red = '#D95319';
    yellow = '#EDB120';
    
    maskSafe = (signalHealth == ESignalHealth.Safe);
    plot(time(maskSafe), signal(maskSafe), '*', 'Color', blue);
    hold on;
    maskRisk = (signalHealth == ESignalHealth.IntegrityRisk);
    plot(time(maskRisk), signal(maskRisk), 'x', 'Color', red);
    hold on;
    maskUnavailable = (signalHealth == ESignalHealth.Unavailable);
    plot(time(maskUnavailable), signal(maskUnavailable), 'x', 'Color', yellow);
    
    if plotLegend
        legendEntries = {};

        if any(maskSafe)
            legendEntries{end+1} = 'Safe';
        end
        
        if any(maskRisk)
            legendEntries{end+1} = 'Integrity risk';
        end
        
        if any(maskUnavailable)
            legendEntries{end+1} = 'Unvailable';
        end
        
        legend(legendEntries);
    end
end
