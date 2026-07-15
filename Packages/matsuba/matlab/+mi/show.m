function v = show(scene, options)
%MI.SHOW Open an interactive progressive viewer for a Mitsuba scene.
%   V = MI.SHOW(SCENE) creates a viewer figure showing MATLAB patches for
%   instant camera interactivity, with a progressive Mitsuba render overlay
%   that improves over time.
%
%   V = MI.SHOW(SCENE, TargetSpp=128) sets the total sample count to
%   accumulate before stopping.
%
%   Rotating, zooming, or panning the figure resets the overlay and
%   re-renders from the new viewpoint.
%
%   Example:
%       scene = mi.Scene.build( ...
%           mi.shape.fromMesh(V, F, bsdf=mi.bsdf.diffuse()), ...
%           mi.emitter.constant(), ...
%           mi.sensor.perspective(fov=45));
%       v = mi.show(scene);
%
%   See also: mi.Viewer, mi.Scene
    arguments
        scene (1,1) mi.Scene
        options.TargetSpp (1,1) double {mustBePositive} = 64
    end
    v = mi.Viewer(scene, TargetSpp=options.TargetSpp);
end
