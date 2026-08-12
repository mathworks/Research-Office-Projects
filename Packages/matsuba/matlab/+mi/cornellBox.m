function scene = cornellBox()
%MI.CORNELLBOX Load Mitsuba's built-in Cornell box scene.
%   SCENE = MI.CORNELLBOX() returns an mi.Scene loaded from Mitsuba's
%   built-in Cornell box definition. No XML file needed.
%
%   Example:
%       scene = mi.cornellBox();
%       img = scene.render(SamplesPerPixel=64);
%       imshow(img)

    try
        sid = py.matlab_mitsuba.bridge.load_cornell_box();
        scene = mi.Scene(double(sid), "id");
    catch ex
        error("mi:cornellBox:Failed", ...
            "Failed to load Cornell box scene.\n%s", ex.message);
    end
end
