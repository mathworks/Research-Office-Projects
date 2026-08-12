function s = cube(options)
%MI.SHAPE.CUBE Create a cube shape descriptor.
%   S = MI.SHAPE.CUBE() creates a unit cube centered at the origin.
%   S = MI.SHAPE.CUBE(Name=Value) sets additional properties.
    arguments
        options.flip_normals = []
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("cube", "shape", options);
end
