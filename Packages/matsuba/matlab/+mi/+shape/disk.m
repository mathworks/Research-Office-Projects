function s = disk(options)
%MI.SHAPE.DISK Create a disk shape descriptor.
%   S = MI.SHAPE.DISK() creates a unit disk in the XY plane.
%   S = MI.SHAPE.DISK(Name=Value) sets additional properties.
    arguments
        options.flip_normals = []
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("disk", "shape", options);
end
