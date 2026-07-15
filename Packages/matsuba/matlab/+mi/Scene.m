classdef Scene < handle
%MI.SCENE Mitsuba scene wrapper for MATLAB.
%   MI.SCENE holds a scene description (struct tree) and lazily compiles it
%   into a live Mitsuba scene for rendering. Scenes can be loaded from XML
%   files, built from factory structs, or constructed from dicts.
%
%   Workflows:
%       % Load and modify an existing Mitsuba scene
%       scene = mi.Scene("cornell_box.xml");
%       scene.add(mi.shape.sphere(radius=0.3, bsdf=mi.bsdf.conductor()));
%       scene.remove("green");
%       img = scene.render(SamplesPerPixel=256);
%       scene.save("modified.xml");
%
%       % Build a scene from scratch
%       scene = mi.Scene.build( ...
%           mi.shape.sphere(radius=1, bsdf=mi.bsdf.diffuse(reflectance=mi.rgb([.8 .1 .1]))), ...
%           mi.emitter.constant(), ...
%           mi.sensor.perspective(fov=45, film=mi.film(Width=1024, Height=768)));
%       img = scene.render();
%
%       % Quick parameter tweaks on a live scene (no rebuild)
%       scene.setParam("red.reflectance.value", [0.8 0.1 0.1]);
%
%       % Differentiable rendering (gradient-based optimization)
%       mi.setVariant("llvm_ad_rgb");
%       scene = mi.Scene.build(mi.cornellBox(), mi.integrator.prb());
%       refImg = scene.render(SamplesPerPixel=256);
%       scene.setParam("red.reflectance.value", [0.01 0.2 0.9]);
%       for i = 1:50
%           [~, loss, grads] = scene.renderDiff(refImg, ...
%               ["red.reflectance.value"], SamplesPerPixel=4);
%           cur = scene.getParam("red.reflectance.value");
%           scene.setParam("red.reflectance.value", ...
%               max(min(cur - 0.05 * grads("red.reflectance.value"), 1), 0));
%       end

    properties (SetAccess = private)
        Id (1,1) double = 0     % Python-side scene registry ID (0 = not built)
        Description struct       % Root struct tree (type='scene', children as fields)
        Dirty (1,1) logical = true  % True if description changed since last build
        HasLiveChanges (1,1) logical = false  % True if setParam/setTransform called since last build
        TempFiles string = string.empty  % Temp files to clean up
        FilePath string = string.empty   % Original XML path for resource resolution
        camera                   % mi.Camera convenience object
    end

    methods
        function obj = Scene(pathOrId, mode)
            %SCENE Construct a Scene.
            %   SCENE = MI.SCENE(PATH) loads from an XML file into a
            %   description struct. The scene is compiled lazily on first
            %   render.
            %   SCENE = MI.SCENE(ID, "id") wraps an existing registry ID
            %   (internal use only — no description available).
            %   SCENE = MI.SCENE(DESC, "desc") wraps an existing struct
            %   description (internal use by build()).
            arguments
                pathOrId = ""
                mode (1,1) string = "file"
            end

            obj.Description = struct("type", "scene", "category_", "scene");

            if mode == "id"
                obj.Id = double(pathOrId);
                obj.Dirty = false;
            elseif mode == "desc"
                obj.Description = pathOrId;
                obj.Dirty = true;
            elseif mode == "file" && strlength(string(pathOrId)) > 0
                filepath = string(pathOrId);
                obj.FilePath = filepath;
                try
                    obj.Description = mi.io.readXML(filepath);
                catch ex
                    error("mi:Scene:LoadFailed", ...
                        "Failed to load scene '%s'.\n%s", filepath, ex.message);
                end
                obj.Dirty = true;
            end

            obj.camera = mi.Camera(obj);
        end

        function add(obj, pluginStruct, key)
            %ADD Add a component to the scene description.
            %   SCENE.ADD(S) adds the plugin struct S with an auto-generated key.
            %   SCENE.ADD(S, KEY) adds it with the specified key name.
            %
            %   Example:
            %       scene.add(mi.shape.sphere(radius=0.5));
            %       scene.add(mi.emitter.point(position=[0 2 0]), "mylight");
            arguments
                obj
                pluginStruct (1,1) struct
                key string = string.empty
            end

            if ~isempty(key)
                fname = char(key);
            elseif isfield(pluginStruct, "key_") && ~isempty(pluginStruct.key_)
                fname = char(pluginStruct.key_);
            else
                % Auto-generate key from category
                if isfield(pluginStruct, "category_")
                    cat = pluginStruct.category_;
                else
                    cat = "item";
                end
                fname = generateKey(obj.Description, cat);
            end

            if obj.HasLiveChanges
                warning("mi:Scene:LiveChangesDiscarded", ...
                    "Scene will be rebuilt — live parameter changes " + ...
                    "(via setParam/setTransform) will be discarded.");
            end
            obj.Description.(fname) = pluginStruct;
            obj.Dirty = true;
        end

        function remove(obj, key)
            %REMOVE Remove a component from the scene description by key.
            %   SCENE.REMOVE(KEY) removes the field with the given key name.
            %
            %   Example:
            %       scene.remove("green");
            arguments
                obj
                key (1,1) string
            end
            fname = char(key);
            if isfield(obj.Description, fname)
                if obj.HasLiveChanges
                    warning("mi:Scene:LiveChangesDiscarded", ...
                        "Scene will be rebuilt — live parameter changes " + ...
                        "(via setParam/setTransform) will be discarded.");
                end
                obj.Description = rmfield(obj.Description, fname);
                obj.Dirty = true;
            else
                warning("mi:Scene:RemoveNotFound", ...
                    "Key '%s' not found in scene description.", key);
            end
        end

        function s = find(obj, key)
            %FIND Return the sub-struct for a given key.
            %   S = SCENE.FIND(KEY) returns the struct, or [] if not found.
            arguments
                obj
                key (1,1) string
            end
            fname = char(key);
            if isfield(obj.Description, fname)
                s = obj.Description.(fname);
            else
                s = [];
            end
        end

        function k = keys(obj)
            %KEYS List all child keys in the scene description.
            %   K = SCENE.KEYS() returns a string array of field names
            %   (excluding metadata fields type, category_, key_).
            fields = fieldnames(obj.Description);
            skip = ["type", "category_", "key_"];
            mask = ~ismember(fields, skip);
            k = string(fields(mask));
        end

        function desc = description(obj)
            %DESCRIPTION Return the scene description struct tree.
            desc = obj.Description;
        end

        function names = params(obj)
            %PARAMS List editable scene parameter names.
            %   Builds the scene if needed, then queries Mitsuba.
            obj.ensureBuilt();
            try
                pyNames = py.matlab_mitsuba.bridge.list_params(int64(obj.Id));
                names = mi.internal.pyListToCellstr(pyNames);
            catch ex
                error("mi:Scene:ParamsFailed", ...
                    "Failed to list parameters.\n%s", ex.message);
            end
        end

        function setParam(obj, name, value)
            %SETPARAM Set a single scene parameter on the live scene.
            %   SCENE.SETPARAM(NAME, VALUE) modifies the compiled Mitsuba
            %   scene directly. This is fast but is NOT reflected in the
            %   description — use ADD/REMOVE for persistent changes.
            arguments
                obj
                name (1,1) string
                value
            end
            obj.ensureBuilt();
            updates = py.dict(pyargs(char(name), mi.internal.toPython(value)));
            try
                py.matlab_mitsuba.bridge.set_params(int64(obj.Id), updates);
                obj.HasLiveChanges = true;
            catch ex
                error("mi:Scene:SetParamFailed", ...
                    "Failed to set parameter '%s'.\n%s", name, ex.message);
            end
        end

        function setParams(obj, names, values)
            %SETPARAMS Set multiple scene parameters on the live scene.
            arguments
                obj
                names (:,1) string
                values (:,1) cell
            end
            assert(numel(names) == numel(values), ...
                "mi:Scene:SetParams", "NAMES and VALUES must have equal length.");
            obj.ensureBuilt();

            items = cell(1, 2*numel(names));
            for i = 1:numel(names)
                items{2*i-1} = char(names(i));
                items{2*i}   = mi.internal.toPython(values{i});
            end
            updates = py.dict(pyargs(items{:}));
            try
                py.matlab_mitsuba.bridge.set_params(int64(obj.Id), updates);
                obj.HasLiveChanges = true;
            catch ex
                error("mi:Scene:SetParamsFailed", ...
                    "Failed to set parameters.\n%s", ex.message);
            end
        end

        function setTransform(obj, paramName, T)
            %SETTRANSFORM Set a transform parameter on the live scene.
            arguments
                obj
                paramName (1,1) string
                T (4,4) double
            end
            obj.ensureBuilt();
            pyMatrix = mi.internal.toPython(T);
            try
                py.matlab_mitsuba.bridge.set_transform( ...
                    int64(obj.Id), paramName, pyMatrix);
                obj.HasLiveChanges = true;
            catch ex
                error("mi:Scene:SetTransformFailed", ...
                    "Failed to set transform '%s'.\n%s", paramName, ex.message);
            end
        end

        function img = render(obj, options)
            %RENDER Render the scene to a MATLAB numeric array.
            %   IMG = SCENE.RENDER() renders with default settings.
            %   IMG = SCENE.RENDER(SamplesPerPixel=64) sets sample count.
            %   IMG = SCENE.RENDER(..., Seed=N) uses a specific random seed.
            %   Automatically builds/rebuilds if the description has changed.
            %
            %   Note: this runs as a single blocking call into Python.
            %   Use RENDERPROGRESSIVE for long renders that need Ctrl+C support.
            %
            %   See also mi.Scene.renderProgressive
            arguments
                obj
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 16
                options.Seed (1,1) double {mustBeNonnegative, mustBeInteger} = 0
            end
            obj.ensureBuilt();
            try
                npImg = py.matlab_mitsuba.bridge.render( ...
                    int64(obj.Id), int64(options.SamplesPerPixel), ...
                    int64(options.Seed));
                img = mi.internal.pyToMatlabArray(npImg);
            catch ex
                error("mi:Scene:RenderFailed", ...
                    "Rendering failed.\n%s", ex.message);
            end
        end

        function img = renderProgressive(obj, options)
            %RENDERPROGRESSIVE Render in multiple passes with Ctrl+C support.
            %   IMG = SCENE.RENDERPROGRESSIVE(SamplesPerPixel=64) renders
            %   in several passes, checking for Ctrl+C between each pass.
            %   IMG = SCENE.RENDERPROGRESSIVE(..., PassSize=8) controls how
            %   many samples per pass (default: SamplesPerPixel/4).
            %   IMG = SCENE.RENDERPROGRESSIVE(..., Convergence=true) enables
            %   early stopping when the image has converged. Rendering stops
            %   when the 99th percentile of per-pixel luminance variance
            %   across passes falls below an automatic threshold, or when
            %   SamplesPerPixel is reached, whichever comes first.
            %
            %   Each pass renders with a different random seed and results
            %   are averaged. This produces the same quality as a single
            %   render() call but allows interruption.
            %
            %   See also mi.Scene.render
            arguments
                obj
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 16
                options.PassSize (1,1) double {mustBeNonnegative, mustBeInteger} = 0
                options.Convergence (1,1) logical = false
                options.Verbose (1,1) logical = true
            end
            obj.ensureBuilt();

            spp = options.SamplesPerPixel;
            passSize = options.PassSize;
            if passSize == 0
                passSize = max(1, round(spp / 4));
            end

            convergenceThreshold = 1e-4;

            remaining = spp;
            accumulator = [];
            % Welford's online algorithm state for per-pass variance
            welfordMean = [];
            welfordM2 = [];
            passCount = 0;
            seed = 0;
            totalDone = 0;
            while remaining > 0
                batchSpp = min(passSize, remaining);
                try
                    npImg = py.matlab_mitsuba.bridge.render( ...
                        int64(obj.Id), int64(batchSpp), int64(seed));
                    pass = mi.internal.pyToMatlabArray(npImg);
                catch ex
                    error("mi:Scene:RenderFailed", ...
                        "Rendering failed.\n%s", ex.message);
                end
                if isempty(accumulator)
                    accumulator = pass * batchSpp;
                else
                    accumulator = accumulator + pass * batchSpp;
                end
                totalDone = totalDone + batchSpp;
                remaining = remaining - batchSpp;
                seed = seed + 1;

                % Update Welford's online variance estimate across passes
                passCount = passCount + 1;
                if passCount == 1
                    welfordMean = pass;
                    welfordM2 = zeros(size(pass));
                else
                    delta = pass - welfordMean;
                    welfordMean = welfordMean + delta / passCount;
                    delta2 = pass - welfordMean;
                    welfordM2 = welfordM2 + delta .* delta2;
                end

                % Check convergence after at least 3 passes
                if options.Convergence && passCount >= 3
                    variance = welfordM2 / (passCount - 1);
                    % Per-pixel luminance variance (max across RGB channels)
                    maxChVar = max(variance, [], 3);
                    metric = prctile(maxChVar(:), 99);
                    if options.Verbose
                        fprintf("  Pass %d/%d (%d spp) — p99 variance: %.2e\n", ...
                            passCount, ceil(spp / passSize), totalDone, metric);
                    end
                    if metric < convergenceThreshold
                        if options.Verbose
                            fprintf("  Converged at %d/%d spp.\n", totalDone, spp);
                        end
                        break
                    end
                elseif options.Convergence && passCount <= 2 && options.Verbose
                    fprintf("  Pass %d/%d (%d spp) — accumulating...\n", ...
                        passCount, ceil(spp / passSize), totalDone);
                end

                drawnow limitrate;
            end
            img = accumulator / totalDone;
        end

        function results = renderAOV(obj, aovNames, options)
            %RENDERAOV Render with Arbitrary Output Variables (depth, normals, etc.).
            %   RESULTS = SCENE.RENDERAOV(AOVNAMES) renders the scene and
            %   returns a struct with the RGB image plus the requested AOV
            %   channels.
            %
            %   AOVNAMES is a string array of AOV types:
            %     "depth"       - Distance from camera (H x W)
            %     "normals"     - Shading normals (H x W x 3), alias for "sh_normal"
            %     "sh_normal"   - Shading normals (H x W x 3)
            %     "geo_normal"  - Geometry normals (H x W x 3)
            %     "position"    - World-space position (H x W x 3)
            %     "uv"          - Texture coordinates (H x W x 2)
            %
            %   The scene's existing integrator is automatically wrapped
            %   with an AOV integrator. A temporary scene is built for the
            %   AOV render and cleaned up afterwards.
            %
            %   Returns a struct with fields:
            %     results.image  - RGB image (H x W x 3)
            %     results.depth  - depth map (if requested)
            %     results.normals - normal map (if "normals" or "sh_normal" requested)
            %     etc.
            %
            %   Example:
            %       scene = mi.Scene.build(mi.cornellBox());
            %       r = scene.renderAOV(["depth", "normals"], SamplesPerPixel=64);
            %       imagesc(r.depth); colorbar; title("Depth");
            %       figure; imshow(r.normals * 0.5 + 0.5); title("Normals");
            %
            %   See also mi.integrator.aov
            arguments
                obj
                aovNames (1,:) string
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 16
            end

            % Require a description (scenes built from Python like
            % cornellBox() have no MATLAB-side description to modify)
            if numel(fieldnames(obj.Description)) <= 2
                error("mi:Scene:RenderAOVNoDesc", ...
                    "renderAOV requires a scene with a MATLAB description.\n" + ...
                    "Scenes created via mi.cornellBox() have no description.\n" + ...
                    "Use mi.Scene.build(...) to construct a scene with AOV support.");
            end

            % Build modified description with AOV integrator
            desc = obj.Description;

            % Find and extract the existing integrator
            existingInt = [];
            intKey = "";
            fields = fieldnames(desc);
            for i = 1:numel(fields)
                v = desc.(fields{i});
                if isstruct(v) && isfield(v, "category_") ...
                        && strcmp(v.category_, "integrator")
                    existingInt = v;
                    intKey = fields{i};
                    break
                end
            end
            if isempty(existingInt)
                existingInt = mi.integrator.path();
            end

            % Wrap with AOV integrator
            aovInt = mi.integrator.aov(aovNames, existingInt);

            % Replace integrator in description copy
            if ~isempty(intKey)
                desc = rmfield(desc, intKey);
            end
            desc.aov_integrator_ = aovInt;

            % Build temporary scene and render all channels
            tempScene = mi.Scene(desc, "desc");
            tempScene.ensureBuilt();
            cleanup = onCleanup(@() delete(tempScene));
            try
                bridge = py.importlib.import_module("matlab_mitsuba.bridge");
                npImg = bridge.render_all_channels( ...
                    int64(tempScene.Id), ...
                    int64(options.SamplesPerPixel), int64(0));
                raw = mi.internal.pyToMatlabArray(npImg);
            catch ex
                error("mi:Scene:RenderAOVFailed", ...
                    "AOV rendering failed.\n%s", ex.message);
            end

            % Split channels: first 3 are RGB, rest are AOVs in order
            results.image = raw(:, :, 1:3);
            ch = 4; % next channel index
            aovMeta = mi.internal.aovChannelCount();
            for i = 1:numel(aovNames)
                name = aovNames(i);
                % Resolve alias
                if name == "normals"
                    type = "sh_normal";
                else
                    type = name;
                end
                nCh = aovMeta.(type);
                if nCh == 1
                    results.(name) = raw(:, :, ch);
                else
                    results.(name) = raw(:, :, ch:ch+nCh-1);
                end
                ch = ch + nCh;
            end
        end

        function [steady, transient] = renderTransient(obj, options)
            %RENDERTRANSIENT Render a transient scene, returning steady-state and time-resolved data.
            %   [STEADY, TRANSIENT] = SCENE.RENDERTRANSIENT() renders a scene
            %   configured with a transient film and integrator.
            %
            %   STEADY is the steady-state image (H x W x 3).
            %   TRANSIENT is the time-resolved data (H x W x T x 3) where T
            %   is the number of temporal bins configured in the transient film.
            %
            %   The scene must use mi.transientFilm() and mi.integrator.transientPath().
            %   mitransient is installed automatically on first use.
            %
            %   Example:
            %       film = mi.transientFilm(Width=256, Height=256, TemporalBins=200);
            %       sensor = mi.sensor.perspective(fov=39, film=film);
            %       scene = mi.Scene.build(..., sensor, mi.integrator.transientPath());
            %       [steady, transient] = scene.renderTransient(SamplesPerPixel=64);
            %
            %   See also mi.transientFilm, mi.integrator.transientPath
            arguments
                obj
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 16
                options.Seed (1,1) double {mustBeNonnegative, mustBeInteger} = 0
            end

            % Ensure mitransient is installed and loaded
            mi.internal.ensureMitransient();

            obj.ensureBuilt();
            try
                result = py.matlab_mitsuba.bridge.render_transient( ...
                    int64(obj.Id), int64(options.SamplesPerPixel), int64(options.Seed));
                steady = mi.internal.pyToMatlabArray(result{1});
                transient = mi.internal.pyToMatlabArray(result{2});
            catch ex
                error("mi:Scene:RenderTransientFailed", ...
                    "Transient rendering failed.\n%s", ex.message);
            end
        end

        function v = getParam(obj, name)
            %GETPARAM Read a scene parameter value from the live scene.
            %   V = SCENE.GETPARAM(NAME) returns the current value of a
            %   scene parameter as a MATLAB double array.
            %
            %   Example:
            %       color = scene.getParam("red.reflectance.value");
            %       % Returns [0.5 0.02 0.02] for a red wall
            arguments
                obj
                name (1,1) string
            end
            obj.ensureBuilt();
            try
                pyVal = py.matlab_mitsuba.bridge.get_param( ...
                    int64(obj.Id), char(name));
                v = double(pyVal);
            catch ex
                error("mi:Scene:GetParamFailed", ...
                    "Failed to get parameter '%s'.\n%s", name, ex.message);
            end
        end

        function [img, loss, grads] = renderDiff(obj, refImg, paramNames, options)
            %RENDERDIFF Differentiable render with gradient computation.
            %   [IMG, LOSS, GRADS] = SCENE.RENDERDIFF(REFIMG, PARAMNAMES)
            %   renders the scene, computes the loss against REFIMG, and
            %   returns gradients for each parameter in PARAMNAMES.
            %
            %   REFIMG is a reference image (H x W x 3 double array).
            %   PARAMNAMES is a string array of parameter names to
            %   differentiate (e.g. ["red.reflectance.value"]).
            %
            %   Returns:
            %     IMG   - rendered image (H x W x 3 double)
            %     LOSS  - scalar loss value (double)
            %     GRADS - containers.Map mapping parameter names to their
            %             gradient vectors (double arrays)
            %
            %   Requires an AD variant (e.g. mi.setVariant("llvm_ad_rgb"))
            %   and an AD integrator (e.g. mi.integrator.prb()).
            %
            %   Example:
            %       mi.setVariant("llvm_ad_rgb");
            %       scene = mi.Scene.build( ...
            %           mi.cornellBox(), mi.integrator.prb());
            %       refImg = scene.render(SamplesPerPixel=256);
            %       scene.setParam("red.reflectance.value", [0.01 0.2 0.9]);
            %
            %       for i = 1:50
            %           [img, loss, grads] = scene.renderDiff(refImg, ...
            %               ["red.reflectance.value"], SamplesPerPixel=4);
            %           g = grads("red.reflectance.value");
            %           cur = scene.getParam("red.reflectance.value");
            %           scene.setParam("red.reflectance.value", ...
            %               max(min(cur - 0.05 * g, 1), 0));
            %       end
            %
            %   See also mi.Scene.forwardGrad, mi.integrator.prb
            arguments
                obj
                refImg (:,:,:) double
                paramNames (:,1) string
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 4
                options.Seed (1,1) double {mustBeNonnegative, mustBeInteger} = 0
                options.LossFunction (1,1) string = "mse"
            end
            obj.ensureBuilt();

            % Convert reference image to numpy
            pyRef = py.numpy.array(refImg, pyargs("dtype", "float32"));

            % Convert param names to Python list (row vector for py.list)
            pyNames = py.list(cellstr(paramNames(:)'));

            try
                result = py.matlab_mitsuba.bridge.render_diff( ...
                    int64(obj.Id), pyRef, pyNames, ...
                    char(options.LossFunction), ...
                    int64(options.SamplesPerPixel), ...
                    int64(options.Seed));

                % Unpack the Python tuple: (image, loss, gradients_dict)
                img = double(result{1});
                loss = double(result{2});

                % Convert gradients dict to containers.Map
                pyGrads = result{3};
                grads = containers.Map();
                pyKeys = py.list(pyGrads.keys());
                for i = 1:length(pyKeys)
                    k = string(pyKeys{i});
                    grads(k) = double(pyGrads{char(k)});
                end
            catch ex
                error("mi:Scene:RenderDiffFailed", ...
                    "Differentiable rendering failed.\n%s", ex.message);
            end
        end

        function gradImg = forwardGrad(obj, paramName, options)
            %FORWARDGRAD Forward-mode sensitivity visualization.
            %   GRADIMG = SCENE.FORWARDGRAD(PARAMNAME) computes how the
            %   rendered image changes with respect to PARAMNAME.
            %
            %   Returns a gradient image (H x W x 3 double) showing
            %   per-pixel sensitivity to the parameter.
            %
            %   Requires an AD variant (e.g. mi.setVariant("llvm_ad_rgb")).
            %
            %   Example:
            %       mi.setVariant("llvm_ad_rgb");
            %       scene = mi.Scene.build(mi.cornellBox());
            %       gradImg = scene.forwardGrad("green.reflectance.value", ...
            %           SamplesPerPixel=128);
            %       imagesc(gradImg(:,:,2)); colormap(coolwarm); colorbar;
            %       title("Sensitivity: green wall color (G channel)");
            %
            %   See also mi.Scene.renderDiff
            arguments
                obj
                paramName (1,1) string
                options.SamplesPerPixel (1,1) double {mustBePositive, mustBeInteger} = 128
                options.Seed (1,1) double {mustBeNonnegative, mustBeInteger} = 0
            end
            obj.ensureBuilt();
            try
                pyGrad = py.matlab_mitsuba.bridge.forward_grad( ...
                    int64(obj.Id), char(paramName), ...
                    int64(options.SamplesPerPixel), ...
                    int64(options.Seed));
                gradImg = double(pyGrad);
            catch ex
                error("mi:Scene:ForwardGradFailed", ...
                    "Forward-mode gradient failed.\n%s", ex.message);
            end
        end

        function save(obj, filepath)
            %SAVE Write the scene description to a Mitsuba XML file.
            %   SCENE.SAVE(PATH) exports the description struct to XML.
            %   Note: live parameter changes via setParam are NOT included;
            %   only the description struct is saved.
            arguments
                obj
                filepath (1,1) string
            end
            if obj.HasLiveChanges
                warning("mi:Scene:SaveLiveChanges", ...
                    "Live parameter changes (via setParam/setTransform) " + ...
                    "are NOT included in the saved file.\n" + ...
                    "Only the original description struct is written.");
            end
            mi.io.writeXML(filepath, obj.Description);
        end

        function pyEval(obj, code)
            %PYEVAL Execute arbitrary Python code with scene context.
            arguments
                obj
                code (1,1) string
            end
            obj.ensureBuilt();
            try
                py.matlab_mitsuba.bridge.py_eval(int64(obj.Id), code);
            catch ex
                error("mi:Scene:PyEvalFailed", ...
                    "Python evaluation failed.\n%s", ex.message);
            end
        end

        function delete(obj)
            %DELETE Release the Python-side scene object and temp files.
            try
                if obj.Id > 0
                    py.matlab_mitsuba.bridge.release(int64(obj.Id));
                end
            catch
                % Suppress errors during cleanup
            end
        end
    end

    methods (Access = private)
        function ensureBuilt(obj)
            %ENSUREBUILT Build/rebuild the Mitsuba scene if dirty.
            if ~obj.Dirty && obj.Id > 0
                return
            end

            % Release previous scene if any
            if obj.Id > 0
                try
                    py.matlab_mitsuba.bridge.release(int64(obj.Id));
                catch
                end
                obj.Id = 0;
            end

            % Apply defaults for missing required components
            desc = obj.Description;
            if ~hasCategory(desc, "integrator")
                desc.default_integrator = mi.integrator.path();
            end
            if ~hasCategory(desc, "sensor")
                desc.default_sensor = mi.sensor.perspective( ...
                    fov=39, ...
                    to_world=mi.Transform.lookAt([0 0 4], [0 0 0], [0 1 0]), ...
                    film=mi.film(Width=512, Height=512));
            end

            % Build from description via normalize_and_load
            try
                pyDict = mi.internal.toPython(desc);
                bridge = py.importlib.import_module("matlab_mitsuba.bridge");
                sid = bridge.normalize_and_load(pyDict);
                obj.Id = double(sid);
                obj.Dirty = false;
                obj.HasLiveChanges = false;
            catch ex
                error("mi:Scene:BuildFailed", ...
                    "Failed to build scene from description.\n%s", ex.message);
            end
        end
    end

    methods (Static)
        function obj = build(varargin)
            %BUILD Assemble a scene from plugin descriptor structs.
            %   SCENE = MI.SCENE.BUILD(S1, S2, ...) takes one or more plugin
            %   structs (from mi.shape.*, mi.bsdf.*, mi.emitter.*, etc.)
            %   and assembles them into a scene.
            %
            %   If a single struct with type='scene' is passed, it is used
            %   directly as the description.
            %
            %   Example:
            %       scene = mi.Scene.build( ...
            %           mi.shape.sphere(radius=1), ...
            %           mi.emitter.constant(), ...
            %           mi.integrator.path(max_depth=8));
            desc = struct("type", "scene", "category_", "scene");
            counters = struct();

            % Flatten cell arrays so users can pass e.g. mi.lighting.threePoint()
            flat = {};
            for k = 1:nargin
                arg = varargin{k};
                if iscell(arg)
                    flat = [flat, arg{:}]; %#ok<AGROW>
                else
                    flat{end+1} = arg; %#ok<AGROW>
                end
            end

            % If a single scene-type struct is passed, use it directly
            if numel(flat) == 1 && isstruct(flat{1}) ...
                    && isfield(flat{1}, "type") ...
                    && strcmp(flat{1}.type, "scene")
                desc = flat{1};
                if ~isfield(desc, "category_")
                    desc.category_ = "scene";
                end
            else
                for k = 1:numel(flat)
                    s = flat{k};
                    if isa(s, "mi.Scene")
                        error("mi:Scene:BuildSceneObject", ...
                            "Argument %d is an mi.Scene object, not a struct.\n" + ...
                            "mi.cornellBox() returns a Scene — use it directly, " + ...
                            "do not pass it to Scene.build().", k);
                    end
                    if ~isstruct(s)
                        error("mi:Scene:BuildBadInput", ...
                            "Argument %d must be a struct (got %s).", k, class(s));
                    end

                    % Determine field name
                    if isfield(s, "key_") && ~isempty(s.key_)
                        fname = char(s.key_);
                    elseif isfield(s, "category_")
                        cat = s.category_;
                        if isfield(counters, cat)
                            counters.(cat) = counters.(cat) + 1;
                        else
                            counters.(cat) = 1;
                        end
                        fname = sprintf("%s_%d", cat, counters.(cat));
                    else
                        if isfield(counters, "item")
                            counters.item = counters.item + 1;
                        else
                            counters.item = 1;
                        end
                        fname = sprintf("item_%d", counters.item);
                    end

                    desc.(fname) = s;
                end
            end

            obj = mi.Scene(desc, "desc");
        end

        function obj = load(filepath)
            %LOAD Load a scene directly via Mitsuba's native XML parser.
            %   SCENE = MI.SCENE.LOAD(PATH) loads a Mitsuba XML scene file
            %   using Mitsuba's built-in parser, bypassing the MATLAB struct
            %   representation. This supports all Mitsuba XML features
            %   including cross-references, includes, and defaults.
            %
            %   Use this for complex scenes (e.g., from mi.io.downloadScene)
            %   that may use features not fully handled by readXML.
            %
            %   Note: scenes loaded this way have no MATLAB-side description,
            %   so add/remove/save are not available. Use setParam, render,
            %   and camera methods as usual.
            %
            %   Example:
            %       xmlPath = mi.io.downloadScene("kitchen");
            %       scene = mi.Scene.load(xmlPath);
            %       img = scene.render(SamplesPerPixel=64);
            %
            %   See also mi.Scene, mi.io.downloadScene
            arguments
                filepath (1,1) string {mustBeFile}
            end
            sid = py.matlab_mitsuba.bridge.load_file(char(filepath));
            obj = mi.Scene(double(sid), "id");
            obj.FilePath = filepath;
        end

        function obj = fromStruct(s)
            %FROMSTRUCT Load a scene from a MATLAB struct.
            %   Kept for backward compatibility. Prefer Scene.build().
            arguments
                s (1,1) struct
            end
            if ~isfield(s, "category_")
                s.category_ = "scene";
            end
            obj = mi.Scene(s, "desc");
        end
    end
end

%% ---- Package-level helper functions ----

function fname = generateKey(desc, category)
    %GENERATEKEY Generate a unique field name like "shape_1", "shape_2", etc.
    n = 1;
    while true
        fname = sprintf("%s_%d", category, n);
        if ~isfield(desc, fname)
            return
        end
        n = n + 1;
    end
end


function tf = hasCategory(desc, category)
    %HASCATEGORY Check if the description has any child with the given category.
    tf = false;
    fields = fieldnames(desc);
    for i = 1:numel(fields)
        v = desc.(fields{i});
        if isstruct(v) && isfield(v, "category_") && strcmp(v.category_, category)
            tf = true;
            return
        end
    end
end
