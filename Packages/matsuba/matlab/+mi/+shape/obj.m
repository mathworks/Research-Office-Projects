function s = obj(options)
%MI.SHAPE.OBJ Create a shape descriptor for a Wavefront OBJ mesh.
%   S = MI.SHAPE.OBJ(filename="mesh.obj") loads a mesh from an OBJ file.
%   S = MI.SHAPE.OBJ(Name=Value) sets additional properties.
    arguments
        options.filename string = string.empty
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.face_normals logical = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("obj", "shape", options);
end
