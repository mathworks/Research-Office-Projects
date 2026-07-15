function s = rectangle(options)
%MI.SHAPE.RECTANGLE Create a rectangle shape descriptor.
%   S = MI.SHAPE.RECTANGLE() creates a unit rectangle in the XY plane.
%   S = MI.SHAPE.RECTANGLE(Name=Value) sets additional properties.
    arguments
        options.flip_normals = []
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("rectangle", "shape", options);
end
