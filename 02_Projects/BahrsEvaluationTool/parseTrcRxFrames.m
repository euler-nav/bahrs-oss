%% Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
function parsedFile = parseTrcRxFrames(fileName)
% PARSETRCRXFRAMES Robustly parse received (Rx) frames from a PCAN-View TRC file.
%
%   parsedFile = parseTrcRxFrames(fileName)
%
%   Lines that are not Rx data frames (comments, Tx frames, warnings/errors,
%   truncated or malformed lines) are skipped. Frames with fewer than 8 data
%   bytes are skipped as well, because the evaluation needs a full payload.
%
%   Returns a 12x1 cell array of column vectors:
%     {1} message number, {2} time offset [ms], {3} CAN ID,
%     {4} data length, {5..12} data bytes 0..7

    parsedFile = cell(12, 1);
    for i = 1:12
        parsedFile{i} = zeros(0, 1);
    end

    fileId = fopen(fileName, 'r');
    if fileId ~= -1
        text = fread(fileId, Inf, '*char')';
        fclose(fileId);

        % One vectorized pass: only complete 8-byte Rx frames match
        bytePattern = '[0-9A-Fa-f]{2}';
        pattern = ['^[ \t]*\d+\)[ \t]+[-+]?\d*\.?\d+[ \t]+Rx[ \t]+[0-9A-Fa-f]+[ \t]+8(?:[ \t]+', bytePattern, '){8}'];
        frameLines = regexp(text, pattern, 'match', 'lineanchors');

        if ~isempty(frameLines)
            frameText = regexprep(strjoin(frameLines, ' '), '\)|Rx', ' ');
            % %x is used by sscanf only; it is supported by both Matlab and Octave here
            values = sscanf(frameText, '%d %f %x %d %x %x %x %x %x %x %x %x', [12, Inf])';
            for i = 1:12
                parsedFile{i} = values(:, i);
            end
        end

        % Report numbered Rx lines that were not accepted
        numRxLines = numel(regexp(text, '^[ \t]*\d+\)[ \t]+\S+[ \t]+Rx[ \t]', 'match', 'lineanchors'));
        skipped = numRxLines - numel(frameLines);
        if skipped > 0
            warning('parseTrcRxFrames:skippedLines', ...
                    'Skipped %d malformed or incomplete Rx lines in %s', skipped, fileName);
        end
    else
        warning('parseTrcRxFrames:fileOpenError', 'Failed to open the input file %s', fileName);
    end
end