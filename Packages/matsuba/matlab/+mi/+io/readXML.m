function desc = readXML(filepath)
%MI.IO.READXML Parse a Mitsuba 3 XML scene file into a MATLAB struct tree.
%   DESC = MI.IO.READXML(FILEPATH) reads the XML file and returns a struct
%   tree representing the scene description. The struct can be passed to
%   MI.SCENE.FROMSTRUCT or modified and saved with MI.IO.WRITEXML.
%
%   Example:
%       desc = mi.io.readXML("cornell_box.xml");
%       scene = mi.Scene.fromStruct(desc);
    arguments
        filepath (1,1) string {mustBeFile}
    end

    try
        doc = xmlread(char(filepath));
    catch ex
        error("mi:io:readXML:ParseError", ...
            "Failed to parse XML file '%s'.\n%s", filepath, ex.message);
    end

    sceneNode = doc.getDocumentElement();

    % Get base directory for resolving relative paths
    [baseDir, ~, ~] = fileparts(char(java.io.File(char(filepath)).getCanonicalPath()));

    desc = parseSceneNode(sceneNode, baseDir);
end

%% ---- Local Functions ----

function desc = parseSceneNode(node, baseDir)
    desc.type = "scene";
    desc.category_ = "scene";

    counters = struct();
    children = node.getChildNodes();
    for i = 0:children.getLength()-1
        child = children.item(i);
        if child.getNodeType() ~= child.ELEMENT_NODE
            continue
        end
        tag = char(child.getTagName());

        if isObjectTag(tag)
            s = parseObjectNode(child, baseDir, struct());
            % Determine field name
            if isfield(s, "key_") && ~isempty(s.key_)
                fname = makeValidField(s.key_);
            else
                if isfield(counters, tag)
                    counters.(tag) = counters.(tag) + 1;
                else
                    counters.(tag) = 1;
                end
                fname = sprintf("%s_%d", tag, counters.(tag));
            end
            desc.(fname) = s;
        elseif strcmp(tag, "ref")
            s = parseRefNode(child);
            if isfield(counters, "ref")
                counters.ref = counters.ref + 1;
            else
                counters.ref = 1;
            end
            fname = sprintf("ref_%d", counters.ref);
            desc.(fname) = s;
        elseif strcmp(tag, "include")
            inclFile = char(child.getAttribute("filename"));
            if ~mi.internal.isAbsolutePath(inclFile)
                inclFile = fullfile(baseDir, inclFile);
            end
            included = mi.io.readXML(inclFile);
            desc = mergeStructs(desc, included);
        elseif strcmp(tag, "default")
            % Skip <default> tags (template parameters)
        else
            % Property-level tags at scene level
            [name, value] = parsePropertyNode(child, baseDir);
            if ~isempty(name)
                desc.(makeValidField(name)) = value;
            end
        end
    end
end

function s = parseObjectNode(node, baseDir, ~)
    tag = char(node.getTagName());
    typeAttr = char(node.getAttribute("type"));
    idAttr = char(node.getAttribute("id"));

    s.type = typeAttr;
    s.category_ = tag;
    if ~isempty(idAttr)
        s.key_ = idAttr;
    end

    counters = struct();
    children = node.getChildNodes();
    for i = 0:children.getLength()-1
        child = children.item(i);
        if child.getNodeType() ~= child.ELEMENT_NODE
            continue
        end
        childTag = char(child.getTagName());

        if isObjectTag(childTag)
            cs = parseObjectNode(child, baseDir, struct());
            if isfield(cs, "key_") && ~isempty(cs.key_)
                fname = makeValidField(cs.key_);
            else
                if isfield(counters, childTag)
                    counters.(childTag) = counters.(childTag) + 1;
                else
                    counters.(childTag) = 1;
                end
                fname = sprintf("%s_%d", childTag, counters.(childTag));
            end
            s.(fname) = cs;
        elseif strcmp(childTag, "transform")
            [name, value] = parsePropertyNode(child, baseDir);
            if ~isempty(name)
                s.(makeValidField(name)) = value;
            end
        elseif strcmp(childTag, "ref")
            rs = parseRefNode(child);
            nameAttr = char(child.getAttribute("name"));
            if ~isempty(nameAttr)
                s.(makeValidField(nameAttr)) = rs;
            else
                if isfield(counters, "ref")
                    counters.ref = counters.ref + 1;
                else
                    counters.ref = 1;
                end
                s.(sprintf("ref_%d", counters.ref)) = rs;
            end
        elseif strcmp(childTag, "include")
            inclFile = char(child.getAttribute("filename"));
            if ~mi.internal.isAbsolutePath(inclFile)
                inclFile = fullfile(baseDir, inclFile);
            end
            included = mi.io.readXML(inclFile);
            s = mergeStructs(s, included);
        else
            [name, value] = parsePropertyNode(child, baseDir);
            if ~isempty(name)
                s.(makeValidField(name)) = value;
            end
        end
    end
end

function [name, value] = parsePropertyNode(node, baseDir)
    tag = char(node.getTagName());
    name = char(node.getAttribute("name"));

    switch tag
        case "float"
            value = str2double(char(node.getAttribute("value")));

        case "integer"
            value = int64(str2double(char(node.getAttribute("value"))));

        case "string"
            value = string(char(node.getAttribute("value")));
            % Resolve relative filenames
            if strcmp(name, "filename") && ~isempty(char(value))
                fpath = char(value);
                if ~mi.internal.isAbsolutePath(fpath)
                    value = string(fullfile(baseDir, fpath));
                end
            end

        case "boolean"
            raw = char(node.getAttribute("value"));
            value = strcmpi(raw, "true");

        case "rgb"
            valStr = char(node.getAttribute("value"));
            value = mi.rgb(parseVector(valStr));

        case "spectrum"
            valStr = char(node.getAttribute("value"));
            fnStr = char(node.getAttribute("filename"));
            if ~isempty(fnStr)
                if ~mi.internal.isAbsolutePath(fnStr)
                    fnStr = fullfile(baseDir, fnStr);
                end
                value = mi.spectrum(filename=string(fnStr));
            elseif ~isempty(valStr)
                v = parseVector(valStr);
                value = mi.spectrum(value=v);
            else
                value = mi.spectrum(value=1.0);
            end

        case ["point", "vector"]
            valStr = char(node.getAttribute("value"));
            if ~isempty(valStr)
                value = parseVector(valStr);
            else
                value = getXYZ(node);
            end

        case "transform"
            value = parseTransformNode(node);

        otherwise
            value = [];
            name = "";
    end
end

function T = parseTransformNode(node)
    T = eye(4);
    children = node.getChildNodes();
    for i = 0:children.getLength()-1
        child = children.item(i);
        if child.getNodeType() ~= child.ELEMENT_NODE
            continue
        end
        Tchild = parseTransformChild(child);
        T = T * Tchild;
    end
end

function T = parseTransformChild(node)
    tag = char(node.getTagName());

    switch tag
        case "translate"
            valStr = char(node.getAttribute("value"));
            if ~isempty(valStr)
                v = parseVector(valStr);
            else
                v = getXYZ(node);
            end
            T = mi.Transform.translate(v);

        case "rotate"
            angle = str2double(char(node.getAttribute("angle")));
            axis = getXYZ(node);
            % If axis is zero (individual axis attribute like x="1")
            if norm(axis) < eps
                % Check individual attributes
                xv = str2double(char(node.getAttribute("x")));
                yv = str2double(char(node.getAttribute("y")));
                zv = str2double(char(node.getAttribute("z")));
                if ~isnan(xv), axis(1) = xv; end
                if ~isnan(yv), axis(2) = yv; end
                if ~isnan(zv), axis(3) = zv; end
            end
            T = axisAngleMatrix(axis, angle);

        case "scale"
            valStr = char(node.getAttribute("value"));
            if ~isempty(valStr)
                v = parseVector(valStr);
                T = mi.Transform.scale(v);
            else
                v = getXYZ(node);
                % For scale, default to 1 not 0
                if v(1) == 0, v(1) = 1; end
                if v(2) == 0, v(2) = 1; end
                if v(3) == 0, v(3) = 1; end
                T = mi.Transform.scale(v);
            end

        case "lookat"
            origin = parseVector(char(node.getAttribute("origin")));
            target = parseVector(char(node.getAttribute("target")));
            up = parseVector(char(node.getAttribute("up")));
            T = mi.Transform.lookAt(origin, target, up);

        case "matrix"
            valStr = char(node.getAttribute("value"));
            vals = sscanf(valStr, '%f');
            if numel(vals) == 16
                % Row-major in XML → reshape then transpose to column-major
                T = reshape(vals, [4 4])';
            elseif numel(vals) == 9
                M = reshape(vals, [3 3])';
                T = eye(4);
                T(1:3, 1:3) = M;
            else
                T = eye(4);
            end

        otherwise
            T = eye(4);
    end
end

function v = parseVector(str)
    % Parse "1, 2, 3" or "1 2 3" → [1 2 3]
    str = strrep(str, ",", " ");
    v = sscanf(str, '%f')';
end

function v = getXYZ(node)
    x = str2double(char(node.getAttribute("x")));
    y = str2double(char(node.getAttribute("y")));
    z = str2double(char(node.getAttribute("z")));
    if isnan(x), x = 0; end
    if isnan(y), y = 0; end
    if isnan(z), z = 0; end
    v = [x y z];
end

function s = parseRefNode(node)
    s.type = "ref";
    s.id = string(char(node.getAttribute("id")));
end

function tf = isObjectTag(tag)
    objectTags = ["scene", "shape", "bsdf", "emitter", "sensor", "integrator", ...
        "film", "sampler", "rfilter", "texture", "medium", "phase", "volume"];
    tf = any(strcmp(tag, objectTags));
end

function T = axisAngleMatrix(axis, angleDeg)
    % Rodrigues' rotation formula
    angleRad = deg2rad(angleDeg);
    axis = axis / norm(axis);
    c = cos(angleRad);
    s = sin(angleRad);
    t = 1 - c;
    x = axis(1); y = axis(2); z = axis(3);

    R = [t*x*x + c,   t*x*y - s*z, t*x*z + s*y;
         t*x*y + s*z, t*y*y + c,   t*y*z - s*x;
         t*x*z - s*y, t*y*z + s*x, t*z*z + c  ];
    T = eye(4);
    T(1:3, 1:3) = R;
end

function fname = makeValidField(name)
    % Make a string safe for use as a MATLAB struct field name
    fname = char(name);
    fname = strrep(fname, "-", "_");
    fname = strrep(fname, ".", "_");
    fname = strrep(fname, " ", "_");
    if ~isempty(fname) && (fname(1) >= '0' && fname(1) <= '9')
        fname = "x" + fname;
    end
    if isempty(fname)
        fname = "unnamed";
    end
end

function s = mergeStructs(s, s2)
    % Merge fields from s2 into s (skip type, category_, key_)
    fields = fieldnames(s2);
    for i = 1:numel(fields)
        f = fields{i};
        if any(strcmp(f, ["type", "category_"]))
            continue
        end
        s.(f) = s2.(f);
    end
end
