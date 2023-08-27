function [out] = safeCounterUnwrap(in, threshold)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    tmp = diff(int64(in));
    tmp(tmp < 0) = tmp(tmp < 0) + threshold;
    out = [uint64(in(1)), uint64(in(1)) + uint64(cumsum(tmp))];
end

