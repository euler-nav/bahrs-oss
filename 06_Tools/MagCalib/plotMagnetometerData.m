%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function plotMagnetometerData(magnetometerData)
    axesArray = {};

    figure;
    axesArray{1} = subplot(3, 1, 1);
    ylabel('X, [G]');
    grid on;
    axesArray{2} = subplot(3, 1, 2);
    ylabel('Y, [G]');
    grid on;
    axesArray{3} = subplot(3, 1, 3);
    ylabel('Z, [G]');
    grid on;
    xlabel('Sample index');
    legendEntries = {};
    legendEntriesCount = 0;


    %%
    uniqueSensorModels = unique(magnetometerData.sensorModels);
    
    for i=1:1:length(uniqueSensorModels)
        sensorModelMask = cellfun(@(x) isequal(x, uniqueSensorModels{i}), magnetometerData.sensorModels);
        sensorInstances = unique(magnetometerData.sensorInstances(sensorModelMask));

        for j=1:1:length(sensorInstances)
            mask = sensorModelMask & (magnetometerData.sensorInstances == sensorInstances(j));
            vectorX = magnetometerData.vectorX(mask);
            axes(axesArray{1});
            hold on;
            plot(vectorX);

            vectorY = magnetometerData.vectorY(mask);
            axes(axesArray{2});
            hold on;
            plot(vectorY);

            vectorZ = magnetometerData.vectorZ(mask);
            axes(axesArray{3});
            hold on;
            plot(vectorZ);

            legendEntriesCount = legendEntriesCount + 1;
            legendEntries{legendEntriesCount} = sprintf('%s %d', uniqueSensorModels{i}, sensorInstances(j));
        end
    end

    axes(axesArray{1});
    legend(legendEntries);
end
