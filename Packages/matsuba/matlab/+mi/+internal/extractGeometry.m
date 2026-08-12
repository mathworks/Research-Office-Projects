function geom = extractGeometry(desc)
%MI.INTERNAL.EXTRACTGEOMETRY Extract mesh geometry and colors from a scene description.
%   GEOM = MI.INTERNAL.EXTRACTGEOMETRY(DESC) walks the scene description
%   struct tree and extracts vertex/face data and approximate diffuse color
%   for each shape that has mesh_data_.
%
%   Returns a struct array with fields:
%     geom(i).V     - Vertices (Nx3 double)
%     geom(i).F     - Faces (Mx3 double, 1-indexed)
%     geom(i).color - RGB color (1x3 double)
%     geom(i).key   - Scene key name (string)
%
%   Shapes without mesh_data_ (sphere, rectangle, etc.) are skipped.

    geom = struct('V', {}, 'F', {}, 'color', {}, 'key', {});

    fields = fieldnames(desc);
    for i = 1:numel(fields)
        fname = fields{i};
        child = desc.(fname);
        if ~isstruct(child)
            continue
        end
        if ~isfield(child, "category_") || ~strcmp(child.category_, "shape")
            continue
        end
        if ~isfield(child, "mesh_data_")
            continue
        end

        V = child.mesh_data_.vertices;
        F = child.mesh_data_.faces;

        % Apply to_world transform if present
        if isfield(child, "to_world") && ~isempty(child.to_world)
            T = child.to_world;
            Vh = [V, ones(size(V,1),1)] * T';
            V = Vh(:, 1:3);
        end

        color = extractColor(child);

        geom(end+1).V = V; %#ok<AGROW>
        geom(end).F = F;
        geom(end).color = color;
        geom(end).key = string(fname);
    end
end

function color = extractColor(shapeStruct)
%EXTRACTCOLOR Extract approximate diffuse color from a shape's BSDF.
    color = [0.5 0.5 0.5]; % default gray

    if ~isfield(shapeStruct, "bsdf") || isempty(shapeStruct.bsdf)
        return
    end
    bsdf = shapeStruct.bsdf;
    if ~isstruct(bsdf)
        return
    end

    % Try known color fields in priority order
    colorFields = ["reflectance", "diffuse_reflectance", "base_color"];
    for i = 1:numel(colorFields)
        if isfield(bsdf, colorFields{i})
            val = bsdf.(colorFields{i});
            color = resolveColor(val);
            return
        end
    end
end

function color = resolveColor(val)
%RESOLVECOLOR Resolve a color value from various formats.
    if isstruct(val) && isfield(val, "type") && strcmp(val.type, "rgb") ...
            && isfield(val, "value")
        v = double(val.value);
        if isscalar(v)
            color = [v v v];
        else
            color = v(1:3);
        end
    elseif isnumeric(val)
        v = double(val);
        if isscalar(v)
            color = [v v v];
        elseif numel(v) >= 3
            color = v(1:3);
        else
            color = [0.5 0.5 0.5];
        end
    else
        color = [0.5 0.5 0.5];
    end
end
