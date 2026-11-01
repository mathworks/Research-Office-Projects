function hmmLiveViewTap(rgb1, rgb2, rgb3, rgb4, stamp)
%HMMLIVEVIEWTAP Hand the four fixed-camera frames of a live-view capture to the operator console.
%
%   HMMLIVEVIEWTAP(rgb1, rgb2, rgb3, rgb4, stamp) stores the frames in root appdata
%   'hmmLiveViewTap' as struct('images', {{rgb1 rgb2 rgb3 rgb4}}, 'simTime', stamp, 'seq', n),
%   n counting up per capture, for HomeMobileManipulatorApp to draw. Called from the root
%   LiveViewTap block only while cfg.camera.liveView is on and only when a new capture has
%   landed. A DISPLAY TAP: nothing in the model reads it, and it never throws.
%
%   See also HomeMobileManipulatorApp, hmmGroundingTap, cfg.camera.liveView.

%   Copyright 2026 The MathWorks, Inc.

try
    prev = getappdata(0, 'hmmLiveViewTap');
    seq = 1;
    if isstruct(prev) && isfield(prev, 'seq')
        seq = prev.seq + 1;
    end
    setappdata(0, 'hmmLiveViewTap', struct('images', {{rgb1, rgb2, rgb3, rgb4}}, ...
        'simTime', double(stamp), 'seq', seq));catch
end
end
