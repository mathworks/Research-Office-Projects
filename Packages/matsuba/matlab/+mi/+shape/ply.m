function s = ply(options)
%MI.SHAPE.PLY Create a shape descriptor for a PLY mesh.
%   S = MI.SHAPE.PLY(filename="mesh.ply") loads a mesh from a PLY file.
%   S = MI.SHAPE.PLY(Name=Value) sets additional properties.
    arguments
        options.filename string = string.empty
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.face_normals logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("ply", "shape", options);
end
