function blkStruct = slblocks
% SLBLOCKS Registers the Event Camera library in the Simulink Library Browser.

blkStruct.Name    = 'Event Camera';
blkStruct.OpenFcn = 'eventcameralib';
blkStruct.MaskInitialization = '';

Browser.Library = 'eventcameralib';
Browser.Name    = 'Event Camera';
Browser.IsFlat  = 0;

blkStruct.Browser = Browser;
end
