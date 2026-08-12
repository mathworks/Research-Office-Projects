function s = directional(options)
%MI.EMITTER.DIRECTIONAL Create a directional (sun/distant) light emitter.
%   S = MI.EMITTER.DIRECTIONAL(direction=[0 -1 0]) creates a distant light
%   shining in the given direction with uniform irradiance across the scene.
%
%   S = MI.EMITTER.DIRECTIONAL(Name=Value) sets additional properties:
%       direction   - 3-element vector pointing FROM the light toward scene
%       irradiance  - spectral irradiance (scalar or mi.rgb descriptor)
%       to_world    - 4x4 transform (alternative to direction)
%
%   Example:
%       sun = mi.emitter.directional(direction=[1 -1 -0.5], ...
%           irradiance=mi.rgb([5 4.5 4]));
%
%   See also: mi.emitter.point, mi.emitter.envmap
    arguments
        options.direction double = []
        options.irradiance struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("directional", "emitter", options);
end
