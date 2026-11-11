%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function plotBahrsCanData(canIds, canRxFrames)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %% Plot height and vertical velocity
    ind = [find(canIds == 322), find(canIds == 314)];
    figureTitle = 'Standard altitude and rate';
    ylabels = {'Altitude, [m]', 'Rate, [m/s]'};
    
    plotSignal(ind, canRxFrames, figureTitle, ylabels);
    
    %% Plot static pressure
    ind = find(canIds == 326);
    figureTitle = 'Static pressure, [hPa]';
    ylabels = {''};
    
    plotSignal(ind, canRxFrames, figureTitle, ylabels);

    %% Plot orientation
    ind = [find(canIds == 312), find(canIds == 311), find(canIds == 1069)];
    figureTitle = 'Orientation, [deg]';
    ylabels = {'Roll', 'Pitch', 'Mag heading'};
    
    plotSignal(ind, canRxFrames, figureTitle, ylabels);
    
    %% Plot specific force
    ind = [find(canIds == 300), find(canIds == 301), find(canIds == 302)];
    figureTitle = 'Specific force, [g]';
    ylabels = {'X', 'Y', 'Z'};
    
    plotSignal(ind, canRxFrames, figureTitle, ylabels);
    
    %% Plot angular rate
    ind = [find(canIds == 304), find(canIds == 303), find(canIds == 305)];
    figureTitle = 'Body angular rate, [deg/s]';
    ylabels = {'X', 'Y', 'Z'};
    
    plotSignal(ind, canRxFrames, figureTitle, ylabels);
end

function plotSignal(indices, frames, figureTitle, ylabels)
    figure;
    for i=1:1:length(indices)
        subplot(length(indices), 1, i);
        plot(1e-6 * frames{indices(i)}.timeUs, frames{indices(i)}.payload);
        if i == 1
            title(figureTitle);
        end
        ylabel(ylabels(i));
    end
    xlabel('Time, [s]');
end

