function hmmRefineNotify(event, rec)
%HMMREFINENOTIFY Tell a listening operator console about a wrist-refinement transaction, NOW.
%
%   HMMREFINENOTIFY(event, rec) calls the function handle stored in root appdata
%   'hmmRefineListener' - registered by HomeMobileManipulatorApp while it runs a session - as
%   listener(event, rec), then flushes the graphics queue. event is 'request' (the wrist frame
%   is about to be sent; rec has RGB, CameraIndex, SimTime) or 'answer' (rec is the full
%   hmmWristRefineTap record). With no listener it does nothing, so offline calls, sim() runs
%   and tests are unaffected.
%
%   WHY A PUSH AND NOT THE APP'S POLL. The refinement runs inside one simulation step, as an
%   extrinsic call that blocks on the provider for up to cfg.provider.timeout. The app's timer
%   cannot fire until that step returns, so a polled tap could only ever show the frame and
%   Gemini's answer together, after the fact. Calling the console from here, with a drawnow,
%   puts the frame on screen while the request is in flight and the answer on top of it the
%   moment it arrives - the sequence an operator needs to see to follow the second look.
%
%   NOTHING HERE MAY THROW. It is called from inside the refinement, which must not fail because
%   a display did.
%
%   See also hmmWristRefine, HomeMobileManipulatorApp.

%   Copyright 2026 The MathWorks, Inc.

try
    listener = getappdata(0, 'hmmRefineListener');
    if isempty(listener) || ~isa(listener, 'function_handle')
        return
    end
    listener(char(event), rec);
    drawnow limitrate
catch
end
end
