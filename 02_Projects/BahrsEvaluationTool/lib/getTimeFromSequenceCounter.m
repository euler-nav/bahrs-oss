function [time] = getTimeFromSequenceCounter(sequenceCounter, timestampUs, querySequenceCounter)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    sequenceCounter = unique(safeCounterUnwrap(sequenceCounter, 256));
    timeUs = interp1(double(sequenceCounter), double(unique(timestampUs)), double(querySequenceCounter), 'linear', 'extrap');
    time = 1e-6 * timeUs;
end

