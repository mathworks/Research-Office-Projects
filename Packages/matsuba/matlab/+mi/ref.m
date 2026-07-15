function s = ref(id)
%MI.REF Create a reference to a named scene object.
%   S = MI.REF(ID) returns a struct that references another object by its key.
%
%   Example:
%       mat = mi.bsdf.diffuse(reflectance=mi.rgb([0.8 0.1 0.1]), key_="shared_mat");
%       shape = mi.shape.sphere(bsdf=mi.ref("shared_mat"));
    arguments
        id (1,1) string
    end
    s = struct("type", "ref", "id", id);
end
