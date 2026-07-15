function writeXML(filepath, desc)
%MI.IO.WRITEXML Write a Mitsuba 3 XML scene file from a MATLAB struct tree.
%   MI.IO.WRITEXML(FILEPATH, DESC) writes the scene description DESC to
%   an XML file at FILEPATH. DESC should be a struct tree as returned by
%   MI.IO.READXML or built manually using MATSUBA helper functions.
%
%   Example:
%       desc = mi.io.readXML("scene.xml");
%       mi.io.writeXML("scene_modified.xml", desc);
    arguments
        filepath (1,1) string
        desc (1,1) struct
    end

    fid = fopen(char(filepath), "w", "n", "UTF-8");
    if fid == -1
        error("mi:io:writeXML:FileError", ...
            "Cannot open file for writing: '%s'", filepath);
    end
    cleanObj = onCleanup(@() fclose(fid));

    fprintf(fid, '<?xml version="1.0" encoding="utf-8"?>\n');
    fprintf(fid, '\n');

    writeSceneNode(fid, desc);
end

%% ---- Local Functions ----

function writeSceneNode(fid, desc)
    fprintf(fid, '<scene version="3.0.0">\n');

    fields = fieldnames(desc);
    for i = 1:numel(fields)
        f = fields{i};
        if any(strcmp(f, ["type", "category_", "key_"]))
            continue
        end
        writeField(fid, f, desc.(f), 1);
    end

    fprintf(fid, '</scene>\n');
end

function writeField(fid, name, value, indent)
    pad = repmat('    ', 1, indent);

    if isstruct(value) && isfield(value, "type")
        writeStructField(fid, name, value, indent, pad);
    elseif isnumeric(value) && isequal(size(value), [4 4])
        writeTransform(fid, name, value, pad);
    elseif isnumeric(value) && numel(value) == 3 && ~isscalar(value)
        fprintf(fid, '%s<point name="%s" value="%s"/>\n', pad, name, vec2str(value));
    elseif islogical(value)
        if value
            fprintf(fid, '%s<boolean name="%s" value="true"/>\n', pad, name);
        else
            fprintf(fid, '%s<boolean name="%s" value="false"/>\n', pad, name);
        end
    elseif isinteger(value) && isscalar(value)
        fprintf(fid, '%s<integer name="%s" value="%d"/>\n', pad, name, value);
    elseif isnumeric(value) && isscalar(value)
        fprintf(fid, '%s<float name="%s" value="%g"/>\n', pad, name, value);
    elseif isstring(value) || ischar(value)
        fprintf(fid, '%s<string name="%s" value="%s"/>\n', pad, name, escapeXML(char(value)));
    end
end

function writeStructField(fid, name, value, indent, pad)
    t = value.type;

    if strcmp(t, "rgb")
        fprintf(fid, '%s<rgb name="%s" value="%s"/>\n', pad, name, vec2str(value.value));
    elseif strcmp(t, "spectrum")
        if isfield(value, "filename")
            fprintf(fid, '%s<spectrum name="%s" filename="%s"/>\n', ...
                pad, name, escapeXML(char(value.filename)));
        elseif isfield(value, "value")
            fprintf(fid, '%s<spectrum name="%s" value="%s"/>\n', ...
                pad, name, vec2str(value.value));
        end
    elseif strcmp(t, "ref")
        if isfield(value, "id")
            fprintf(fid, '%s<ref id="%s"/>\n', pad, char(value.id));
        end
    else
        % Nested object node
        writeObjectNode(fid, value, indent);
    end
end

function writeObjectNode(fid, s, indent)
    pad = repmat('    ', 1, indent);

    if isfield(s, "category_")
        tag = s.category_;
    else
        tag = "object";
    end

    % Build opening tag
    attrs = sprintf(' type="%s"', s.type);
    if isfield(s, "key_") && ~isempty(s.key_)
        attrs = sprintf('%s id="%s"', attrs, char(s.key_));
    end

    % Collect child fields (skip builder-only metadata)
    skipFields = ["type", "category_", "key_", "mesh_data_", "face_normals", "raymap_data_"];
    fields = fieldnames(s);
    childFields = {};
    for i = 1:numel(fields)
        f = fields{i};
        if any(strcmp(f, skipFields))
            continue
        end
        childFields{end+1} = f; %#ok<AGROW>
    end

    % If shape has mesh_data_, write an OBJ file alongside the XML
    if isfield(s, "mesh_data_") && ~any(strcmp("filename", childFields))
        V = s.mesh_data_.vertices;
        F = s.mesh_data_.faces;
        childFields{end+1} = "filename";
        s.filename = writeMeshOBJ(fid, V, F);
    end

    if isempty(childFields)
        fprintf(fid, '%s<%s%s/>\n', pad, tag, attrs);
    else
        fprintf(fid, '%s<%s%s>\n', pad, tag, attrs);
        for i = 1:numel(childFields)
            writeField(fid, childFields{i}, s.(childFields{i}), indent + 1);
        end
        fprintf(fid, '%s</%s>\n', pad, tag);
    end
end

function objPath = writeMeshOBJ(fid, V, F)
    % Write mesh data to an OBJ file next to the XML output
    xmlPath = fopen(fid);
    [xmlDir, xmlName, ~] = fileparts(xmlPath);
    meshName = sprintf("%s_mesh_%d.obj", xmlName, round(rand*1e6));
    objPath = string(fullfile(xmlDir, meshName));
    fidObj = fopen(char(objPath), "w");
    if fidObj == -1
        warning("mi:io:writeXML:MeshWriteFailed", ...
            "Could not write mesh file: %s", objPath);
        return
    end
    for i = 1:size(V, 1)
        fprintf(fidObj, "v %g %g %g\n", V(i,1), V(i,2), V(i,3));
    end
    for i = 1:size(F, 1)
        fprintf(fidObj, "f %d %d %d\n", F(i,1), F(i,2), F(i,3));
    end
    fclose(fidObj);
end

function writeTransform(fid, name, M, pad)
    fprintf(fid, '%s<transform name="%s">\n', pad, name);
    innerPad = [pad '    '];
    fprintf(fid, '%s<matrix value="%s"/>\n', innerPad, mat2strRowmajor(M));
    fprintf(fid, '%s</transform>\n', pad);
end

function str = mat2strRowmajor(M)
    % Write 4x4 matrix in row-major order as space-separated values
    vals = M';  % Transpose so linear indexing reads row-major
    strs = arrayfun(@(v) sprintf('%g', v), vals(:)', 'UniformOutput', false);
    str = strjoin(strs, ' ');
end

function str = vec2str(v)
    % Format numeric vector as "v1, v2, v3"
    strs = arrayfun(@(x) sprintf('%g', x), v, 'UniformOutput', false);
    str = strjoin(strs, ', ');
end

function str = escapeXML(str)
    str = strrep(str, "&", "&amp;");
    str = strrep(str, "<", "&lt;");
    str = strrep(str, ">", "&gt;");
    str = strrep(str, """", "&quot;");
end
