function s = sphere(options)
%MI.SHAPE.SPHERE Create a sphere shape descriptor.
%   S = MI.SHAPE.SPHERE() creates a unit sphere at the origin.
%   S = MI.SHAPE.SPHERE(Name=Value) sets additional properties.
%
%   Example:
%       s = mi.shape.sphere(radius=0.5, center=[0 1 0]);
    arguments
        options.center double = []
        options.radius double = []
        options.flip_normals = []
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("sphere", "shape", options);
end
