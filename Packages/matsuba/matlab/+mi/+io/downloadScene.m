function scenePath = downloadScene(name, options)
%MI.IO.DOWNLOADSCENE Download a Mitsuba 3 gallery scene.
%   SCENEPATH = MI.IO.DOWNLOADSCENE(NAME) downloads the named scene from
%   the Mitsuba 3 gallery and returns the path to its main XML file.
%
%   The scene is cached locally so repeated calls do not re-download.
%
%   NAME is a scene identifier from mi.io.gallery() (e.g., "kitchen",
%   "cornell-box", "dragon").
%
%   Options:
%     CacheDir - Directory for cached scenes (default: <matsuba>/scenes)
%
%   Example:
%       xmlPath = mi.io.downloadScene("kitchen");
%       scene = mi.Scene(xmlPath);
%       img = scene.render(SamplesPerPixel=64);
%
%   See also mi.io.gallery, mi.Scene

    arguments
        name (1,1) string
        options.CacheDir (1,1) string = ""
    end

    BASE_URL = "https://d38rqfq1h7iukm.cloudfront.net/scenes/";

    % Validate name against gallery
    t = mi.io.gallery();
    if ~any(t.Name == name)
        error("mi:io:downloadScene:UnknownScene", ...
            "Unknown scene '%s'. Use mi.io.gallery() to see available scenes.", name);
    end

    % Determine cache directory
    if options.CacheDir == ""
        cacheDir = fullfile(mi.internal.matsubaRoot(), "scenes");
    else
        cacheDir = options.CacheDir;
    end

    sceneDir = fullfile(cacheDir, name);
    xmlPath = findSceneXML(sceneDir);
    if xmlPath ~= ""
        fprintf("Using cached scene: %s\n", sceneDir);
        scenePath = xmlPath;
        return
    end

    % Download zip
    zipUrl = BASE_URL + name + ".zip";
    if ~isfolder(cacheDir)
        mkdir(cacheDir);
    end
    zipPath = fullfile(cacheDir, name + ".zip");

    fprintf("Downloading %s...\n", name);
    try
        websave(zipPath, zipUrl);
    catch ex
        error("mi:io:downloadScene:DownloadFailed", ...
            "Failed to download '%s'.\n%s", zipUrl, ex.message);
    end

    % Extract
    fprintf("Extracting...\n");
    try
        unzip(zipPath, cacheDir);
    catch ex
        error("mi:io:downloadScene:ExtractFailed", ...
            "Failed to extract '%s'.\n%s", zipPath, ex.message);
    end
    delete(zipPath);

    % Find the XML file
    xmlPath = findSceneXML(sceneDir);
    if xmlPath == ""
        % Some zips extract to a differently-named subfolder — search cacheDir
        xmlPath = findSceneXMLInCache(cacheDir, name);
    end
    if xmlPath == ""
        error("mi:io:downloadScene:NoXML", ...
            "No .xml scene file found in: %s", sceneDir);
    end

    fprintf("Scene ready: %s\n", xmlPath);
    scenePath = xmlPath;
end


function xmlPath = findSceneXML(sceneDir)
    xmlPath = "";
    if ~isfolder(sceneDir)
        return
    end
    % Look for scene.xml first, then any .xml
    candidate = fullfile(sceneDir, "scene.xml");
    if isfile(candidate)
        xmlPath = string(candidate);
        return
    end
    xmlFiles = dir(fullfile(sceneDir, "*.xml"));
    if ~isempty(xmlFiles)
        xmlPath = string(fullfile(sceneDir, xmlFiles(1).name));
    end
end


function xmlPath = findSceneXMLInCache(cacheDir, name)
    % Search all immediate subdirectories for the XML file
    xmlPath = "";
    subdirs = dir(cacheDir);
    for i = 1:numel(subdirs)
        if ~subdirs(i).isdir || subdirs(i).name(1) == '.'
            continue
        end
        candidate = findSceneXML(fullfile(cacheDir, subdirs(i).name));
        if candidate ~= ""
            % Rename folder to match expected name if different
            actualDir = fullfile(cacheDir, subdirs(i).name);
            expectedDir = fullfile(cacheDir, name);
            if ~strcmp(actualDir, expectedDir) && ~isfolder(expectedDir)
                movefile(actualDir, expectedDir);
                candidate = strrep(candidate, actualDir, expectedDir);
            end
            xmlPath = candidate;
            return
        end
    end
end
