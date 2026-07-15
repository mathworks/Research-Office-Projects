function s = fromMesh(V, F, options)
%MI.SHAPE.FROMMESH Create a shape from MATLAB vertex/face data.
%   S = MI.SHAPE.FROMMESH(V, F) creates a mesh shape from vertices V (Nx3)
%   and faces F (Mx3, 1-indexed).
%
%   Example:
%       [V,F] = isosurface(peaks, 0.5);
%       s = mi.shape.fromMesh(V, F, bsdf=mi.bsdf.plastic());
    arguments
        V (:,3) double
        F (:,3) double
        options.bsdf struct = struct([])
        options.emitter struct = struct([])
        options.to_world double = []
        options.flip_normals = []
        options.face_normals = []
        options.key_ string = string.empty
    end
    s = mi.internal.pluginStruct("obj", "shape", options);
    s.mesh_data_ = struct('vertices', V, 'faces', F);
end
