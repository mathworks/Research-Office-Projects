function s = cylinder(options)
%MI.SHAPE.CYLINDER Create a cylinder shape descriptor.
%   S = MI.SHAPE.CYLINDER() creates a unit cylinder.
%   S = MI.SHAPE.CYLINDER(Name=Value) sets additional properties.
    arguments
        options.p0 double = []
        options.p1 double = []
        options.radius double = []
        options.flip_normals = []
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("cylinder", "shape", options);
end
