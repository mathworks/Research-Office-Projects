classdef MPyReq < handle
    % MPyReq Manage Python Requirements
    %
    % MPyReq.setInstallFolder() Sets the default installation folder,
    % preferably on an SSD, with enough space. Note each version of a
    % PyTorch+CUDA install can take multiple GB's of space. MPyReq creates
    % versioned subfolders, so there is not need to change this once set
    % per machine. This folder can be also be shared across users/machines
    % as long as the machines have the same OS version. Do note that shared
    % network drives will add to first run latency. Set value is persisted
    % across sessions into MATLAB settings.
    %
    % MPyReq.installFolder() Returns the installation location.
    %
    % MPyReq.availableVersions() List Python versions available for
    % download.
    %
    % Following calls installs and configures for runtime the corresponding
    % Python requirement. On subsequent run, its a quick NOP. All calls
    % support an optional "Name" NV which controls the name of the
    % installed subfolder. Specify optional NV InstallFcn and
    % RuntimeFcn to specify function handles to perform additional install
    % or runtime setup actions.
    %   MPyReq.python(version) version can either be "x","x.xx" or "x.xx.xx"
    %    - Downloads Python from https://github.com/astral-sh/python-build-standalone/releases
    %    - Configures pyenv with it
    %    - Defaults to OutOfProcess, use ExecutionMode PV to choose
    %      InProcess if required.
    %   MPyReq.pipPackage(packageName)
    %    - pip installs named package (Name can include ==x.xx version)
    %    - Adds path to installed location to Python in pyenv instance
    %    - Note: packageName could be a git repo link to a "Python
    %      project". Use RuntimeFcn to add any specific sub-folders to
    %      python path.
    %   MPyReq.requirementTextFile(rfileName)
    %    - Name is required
    %    - pip installs requirement file
    %    - Adds downloaded packages to installed location to Python in
    %      pyenv instance
    %   MPyReq.gitrepo(url)
    %   MPyReq.weights(url)
    %    - Specify DownloadTo=<destination folder> if required
    %   MPyReq.addToPythonPath(folder)
    %
    %   MPyReq.require(...)
    %    - PV's Name, InstallFcn, InstallConfirmationText, RuntimeFcn
    %    - Common Engine API for all API's above
    %    - PWD is changed to MPyReq.installFolder()+filesep+<name> before
    %      calling Install/RuntimeFcns.
    %
    % Requirement helper APIs:
    %   MPyReq.list()       List all installed requirements.
    %   MPyReq.pathTo(name) Path to a named requirement.
    %   MPyReq.remove(name) Remove a named requirement.
    %
    % Helper APIs:
    %   MPyReq.verbose(tf)                   Set verbosity
    %   MPyReq.log(msg)                      Log message
    %   tf = MPyReq.confirm(msg)             Request confirmation
    %   MPyReq.autoAcceptDownloadPrompts(tf) Automatically accept all
    %                                        download prompts.
    %  
    %
    % Limitations:
    %
    %   MPyReq is not aware of pyenv terminations. Explicitly call
    %   MPyReq.reset() to reset runtime cache. Note: A "clear all" will
    %   also cause a reset. Usually this just re-runs the RunTimeFcn's
    %   which is likely to be fast and ought not to mess things up.
    %
    %   No compatibility validations are done. Ensure to only add
    %   packages/repos that work with the chosen Python version (and dont
    %   have internal conflicts). Ideally, just work with 'one project' at
    %   a time (i.e pyenv is configured only with packages for one
    %   project). Call
    %       terminate(pyenv)
    %       MPyReq.reset()
    %   before switching between Python projects.
    %
    % Example: Install, configure and call Cellpose 4.0
    %
    %    MPyReq.python("3.11");
    %    MPyReq.pipPackage("cellpose");
    %    % Refer MATLAB's py.<> API's and Cellpose API's
    %    model = py.cellpose.models.CellposeModel(gpu=true)
    %
    %    im = imread("AT3_1m4_01.tif");
    %    outputs = model.eval(im);
    %    labels = uint16(outputs{1});
    %    imageshow(im,OverlayData=labels)
    %
    % See also: pyenv

    properties (Access = private)
        InstallFolder (1,1) string
        Verbose (1,1) logical = false
        IsConfiguredForRuntime (1,1) dictionary
        AutoAcceptDownloadPrompts (1,1) logical = false
    end

    methods (Access = private)
        % Private constructor to enforce singleton (via static methods)
        function obj = MPyReq()

            s = settings;
            if s.hasGroup("MPyReq") && s.MPyReq.hasSetting("InstallFolder") && hasPersonalValue(s.MPyReq.InstallFolder)
                obj.InstallFolder = s.MPyReq.InstallFolder.PersonalValue;
            else
              waitfor(msgbox("MPyReq downloads Python and Python packages to a folder." ...
                  +newline+"Press OK and specify a folder to use.",...
                  "Installation Folder Needed",...
                  "modal"));
              obj.InstallFolder = uigetdir;
            end

            if ~isfolder(obj.InstallFolder)
                mkdir(obj.InstallFolder)
            end

            obj.doReset();

            % MPyReq uses static methods, ensure its always on path so
            % static methods continue to work when cd'ed out of this folder
            addpath(fileparts(which('MPyReq.m')));
        end

        function doReset(obj)
            obj.IsConfiguredForRuntime = configureDictionary("string","logical");
        end

        function tf = isInstalled(obj, reqName)
            % Optional - consider caching this
            tf = isfolder(obj.pathTo(reqName));
        end

        function tf = isConfiguredForRunTime(obj, reqName)
            tf = obj.IsConfiguredForRuntime.isKey(reqName);
        end

        function configuredForRuntime(obj, reqName)
            obj.IsConfiguredForRuntime(reqName) = true;
        end
    end

    %% Helper API

    methods (Static)
        function instance = getInstance()
            persistent pyreqManager
            if isempty(pyreqManager)
                pyreqManager = MPyReq();
            end
            instance = pyreqManager;
        end

        function instFldr = installFolder()
            instFldr = MPyReq.getInstance().InstallFolder;
        end

        function setInstallFolder(instFldr)
            arguments
                instFldr (1,1) string
            end
            if ~isfolder(instFldr)
                mkdir(instFldr)
            end

            % Persist
            s = settings;
            % Add hidden setting
            if ~s.hasGroup("MPyReq")
                s.addGroup("MPyReq",Hidden=true);
            end
            if ~s.MPyReq.hasSetting("InstallFolder")
                s.MPyReq.addSetting("InstallFolder");
            end
            s.MPyReq.InstallFolder.PersonalValue = instFldr;

            pr = MPyReq.getInstance();
            pr.InstallFolder = instFldr;
        end

        function verbose(tf)
            arguments
                tf (1,1) logical = true
            end
            pr = MPyReq.getInstance();
            pr.Verbose = tf;
        end

        function log(msg)
            arguments
                msg (1,1) string
            end
            pr = MPyReq.getInstance();
            if pr.Verbose
                disp(msg)
            end
        end

        function autoAcceptDownloadPrompts(tf)
            arguments
                tf (1,1) logical = true
            end
            pr = MPyReq.getInstance();
            pr.AutoAcceptDownloadPrompts = tf;
        end

        function tf = confirm(msg)
            arguments
                msg (1,1) string
            end
            pr = MPyReq.getInstance();
            msg = msg+newline+"Continue Y/[N]: ";
            % In windows, the input message shown on the command line does not show file-separator, as '\' is treated as an escape sequence.
            % Here we replace single backslash with double backslash for proper display.
            msg = strrep(msg,"\","\\");
            if pr.AutoAcceptDownloadPrompts
                disp(msg)
                disp("    [Y] MPyReq.autoAcceptDownloadPrompts(true) was invoked earlier.")
                tf = true;
            else
                response = input(msg,"s");
                tf = (isstring(response)||ischar(response)) && lower(response)=="y";
            end

        end

        function availableVersions()
            uvOut = MPyReq.invokeuv("python --managed-python list");
            disp(uvOut);
        end

        function fldr = pathTo(name)
            arguments
                name (1,1) string {mustBeNonzeroLengthText}
            end
            pr = MPyReq.getInstance();
            penv = pyenv();
            pyVersion = penv.Version;
            fldr = pr.InstallFolder+filesep+pyVersion+filesep+name;
        end

        function remove(name)
            pr = MPyReq.getInstance();
            rmfldr = pr.pathTo(name);
            disp("Removing folder: "+rmfldr);
            try
                rmdir(rmfldr,'s');
            catch ALL
                disp("Removing folder: "+rmfldr+" failed. Manually remove this folder");
                rethrow(ALL);
            end
        end

        function reset()
            pr = MPyReq.getInstance();
            pr.doReset();
        end

        function list()
            disp("Python requirements storage folder: ")
            disp(MPyReq.installFolder());
            dir(MPyReq.installFolder());
        end

        function addToPythonPath(pfolder)
            arguments
                pfolder (1,1) string {mustBeFolder}
            end
            insert(py.sys.path(), int64(0), pfolder);
        end
    end

    %% "require" APIs

    methods (Static)

        function require(options)
            arguments
                options.Name (1,1) string {mustBeNonzeroLengthText}
                options.InstallFcn (1,1) function_handle = @()[]
                options.InstallConfirmationText (1,1) string {mustBeNonzeroLengthText}
                options.RuntimeFcn(1,1) function_handle = @()[]
                options.ForceInstall = false;
            end

            pr = MPyReq.getInstance();

            if ~pr.isInstalled(options.Name) || options.ForceInstall
                if ~isfield(options,"InstallConfirmationText")
                    options.InstallConfirmationText = "Installing "+options.Name+". This will download external unverified code.";
                end
                restoreWD = cdToNamedFolder(options.Name); %#ok<NASGU>
                try
                    tf = MPyReq.confirm(options.InstallConfirmationText);
                    if tf
                        options.InstallFcn();
                        clear("restoreWD");
                    else
                        clear("restoreWD");
                        error("Install aborted.")
                    end
                catch ALL
                    clear("restoreWD");  % restores WD, so win64 can delete folder
                    try
                        rmdir(pr.pathTo(options.Name),"s");
                    catch
                        % win64 will still not delete folders sometimes
                        warning("Failed to delete "+pr.pathTo(options.Name)+newline+"Manually delete before proceeding.");
                    end
                    rethrow(ALL);
                end

            end

            if (pr.isInstalled(options.Name) && ~pr.isConfiguredForRunTime(options.Name)) || options.ForceInstall
                restoreWD = cdToNamedFolder(options.Name); %#ok<NASGU>
                options.RuntimeFcn();
                pr.configuredForRuntime(options.Name);
                clear("restoreWD");
            end
        end

        % Python requirement
        function python(version,options)
            arguments
                version (1,1) string {mustBeNonzeroLengthText}
                options.ExecutionMode string {mustBeMember(options.ExecutionMode,["InProcess","OutOfProcess"])} = "OutOfProcess"
            end

            % Early return if all-set.
            penv = pyenv;
            if penv.Version == version && penv.ExecutionMode==options.ExecutionMode...
                    && contains(penv.Executable,MPyReq.installFolder)
                % Note - make sure to use Python MPyReq installed, do not
                % rely on system python (for consistency).
                return;
            end

            try
                uvOut = MPyReq.invokeuv("python --managed-python find "+version);
            catch
                tf = MPyReq.confirm("Download python "+version+" using uv?");
                if ~tf
                    return;
                end
                MPyReq.invokeuv("python --managed-python install "+version);
                uvOut = MPyReq.invokeuv("python --managed-python find "+version);
            end
            pybin = strtrim(uvOut);

            curpenv = pyenv;
            if curpenv.Executable~=pybin
                MPyReq.log("pyenv has: "+curpenv.Executable)
                MPyReq.log("Requested: "+pybin)

                if curpenv.ExecutionMode=="OutOfProcess" || curpenv.Status=="NotLoaded"
                    MPyReq.log("Terminating current instance.");
                    terminate(pyenv);
                    MPyReq.reset();
                else
                    error("pyenv (InProcess Mode) in use with a different binary than requested. MATLAB needs to be restarted to switch InProcess Mode versions");
                end
            end

            MPyReq.log("Initializing pyenv with: "+pybin);

            if penv.ExecutionMode=="OutOfProcess" && options.ExecutionMode=="InProcess"
                terminate(pyenv);
            end
            pyenv(Version=pybin,ExecutionMode=options.ExecutionMode);

        end

        function pipPackage(package, options)
            arguments
                package (1,1) string {mustBeNonzeroLengthText}
                options.Name (1,1) string
                options.InstallFcn (1,1) function_handle = @()[]
                options.RuntimeFcn(1,1) function_handle = @()[]
                options.EnvironmentVars(1,:) string = []
                options.ForceInstall = false
            end

            if ~isfield(options,"Name")
                options.Name = matlab.lang.makeValidName(package);
                MPyReq.log("Using name: "+options.Name);
            end

            function installFcn()
                penv = pyenv;
                cmd = "pip install"...
                    +" --target """+MPyReq.pathTo(options.Name)+""" "...
                    +" --python-version "+penv.Version...
                    +" " +package;

                MPyReq.invokeuv(cmd,"EnvironmentVars",options.EnvironmentVars);
                options.InstallFcn();
            end

            function runTimeFcn()
                MPyReq.addPath(MPyReq.pathTo(options.Name));
                options.RuntimeFcn();
            end

            MPyReq.require(Name=options.Name,...
                InstallFcn=@installFcn, ...
                InstallConfirmationText="Installing "+package+" using uv pip."+newline+"To: " +MPyReq.pathTo(options.Name),...
                RuntimeFcn=@runTimeFcn,...
                ForceInstall=options.ForceInstall);
        end

        function weights(url, options)
            arguments
                url (1,1) string {mustBeNonzeroLengthText}
                options.DownloadTo (1,1) string
                options.Name (1,1) string
                options.InstallFcn (1,1) function_handle = @()[]
                options.RuntimeFcn(1,1) function_handle = @()[]
            end
            if ~isfield(options,"Name")
                % "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt"
                % => sam2.1_hiera_tiny
                [~, options.Name] = fileparts(url);
            end
            if ~isfield(options,"DownloadTo")
                options.DownloadTo = MPyReq.pathTo(options.Name);
            end

            function installFcn()
                [~,fname,fext] = fileparts(url);
                if ~isfolder(options.DownloadTo)
                    mkdir(options.DownloadTo);
                end
                websave(options.DownloadTo+filesep+fname+fext,url);
                options.InstallFcn();
            end

            function runTimeFcn()
                options.RuntimeFcn();
            end

            MPyReq.require(Name=options.Name,...
                InstallFcn=@installFcn, ...
                InstallConfirmationText="Downloading: "+url+newline+ ...
                "To         : "+options.DownloadTo,...
                RuntimeFcn=@runTimeFcn);
        end

        function requirementTextFile(reqtxtFile, options)
            arguments
                reqtxtFile (1,1) string {mustBeFile}
                options.Name (1,1) string
                options.InstallFcn (1,1) function_handle = @()[]
                options.RuntimeFcn(1,1) function_handle = @()[]
                options.BuildInIsolation = true
                options.EnvironmentVars(1,:) string = []
                options.ForceInstall = false
            end
            if ~isfield(options,"Name")
                error("Name PV is required for installing requirements.txt files");
            end


            function installFcn()
                penv = pyenv;

                cmd = "pip install"...
                    +" --target """+MPyReq.pathTo(options.Name)+""" "...
                    +" --python-version "+penv.Version...
                    +" -r """+reqtxtFile+"""";

                if ~options.BuildInIsolation
                    cmd = insertBefore(cmd," -r"," --no-build-isolation");
                end
                MPyReq.invokeuv(cmd,"EnvironmentVars",options.EnvironmentVars);
                options.InstallFcn();
            end

            function runTimeFcn()
                MPyReq.addPath(MPyReq.pathTo(options.Name));
                options.RuntimeFcn();
            end

            MPyReq.require(Name=options.Name,...
                InstallFcn=@installFcn, ...
                InstallConfirmationText="Installing "+reqtxtFile+" using uv pip."+newline+"To: " +MPyReq.pathTo(options.Name),...
                RuntimeFcn=@runTimeFcn,...
                ForceInstall=options.ForceInstall);
        end

        function gitrepo(url, options)
            arguments
                url (1,1) string {mustBeNonzeroLengthText}
                options.Name (1,1) string
                options.RecurseSubmodules = false
                options.InstallFcn (1,1) function_handle = @()[]
                options.RuntimeFcn(1,1) function_handle = @()[]
            end

            if ~isfield(options,"Name")
                % e.g convert
                % url = "https://github.com/ZhengPeng7/BiRefNet.git"
                % to name = BiRefNet
                [~, options.Name] = fileparts(url);
            end

            function installFcn()
                % Limit depth to save on diskspace.
                gitclone(url, pwd, Depth=1,RecurseSubmodules = options.RecurseSubmodules);
                options.InstallFcn();
            end

            function runTimeFcn()
                MPyReq.addPath(MPyReq.pathTo(options.Name));
                options.RuntimeFcn();
            end

            MPyReq.require(Name=options.Name,...
                InstallFcn=@installFcn, ...
                InstallConfirmationText="Git cloning: "+url+newline+"To         : "+MPyReq.pathTo(options.Name),...
                RuntimeFcn=@runTimeFcn);
        end

        function addPath(ppath)
            arguments
                ppath (1,1) string {mustBeNonzeroLengthText}
            end
            if ~any(strcmp(ppath, string(py.sys.path)))
                insert(py.sys.path, int64(0), ppath);
            end
        end

    end

    %% uv env

    methods (Static)
        function uv(version)
            arguments
                version string {mustBeScalarOrEmpty} =  "0.7.17"
            end

            if isfolder(MPyReq.installFolder+filesep+"uvbin")
                sysCmdString = """"+MPyReq.installFolder+filesep+"uvbin"+filesep+"uv"+""""...
                +" --version";
                [~, v] = system(sysCmdString);
                % e.g win64 output  'uv 0.7.17 (41c218a89 2025-06-29)'
                % e.g glnxa64 output: 'uv 0.7.17'
                v = regexp(v, '\d+\.\d+\.\d+', 'match', 'once');
                if v~=version
                    % TODO is uv forward-backward compatible? Would just deleting uv
                    % and re-installing work? Conservative error for now.
                    error("Found uv version"+v+" already installed. Delete the installFolder if you want to update uv version");
                else
                    % All good, installed and version matches.
                end

            else
                % Install
                tf = MPyReq.confirm("Installing uv from https://github.com/astral-sh/uv/releases/download/");
                if ~tf
                    return;
                end

                url = "https://github.com/astral-sh/uv/releases/download/";
                switch computer("arch")
                    case "win64"
                        url = url...
                            +version+"/uv-x86_64-pc-windows-msvc.zip";
                        fileName = "uv.zip";
                    case "glnxa64"
                        url = url...
                            +version+"/uv-x86_64-unknown-linux-gnu.tar.gz";
                        fileName = "uv.tar.gz";
                    case "maca64"
                        url = url...
                            +version+"/uv-aarch64-apple-darwin.tar.gz";
                        fileName = "uv.tar.gz";
                    otherwise
                        error("Unsupported platform")

                end
                MPyReq.log("Downloading: "+url);

                cwd = pwd;
                restoreWD = onCleanup(@()cd(cwd));
                cd(MPyReq.installFolder);

                MPyReq.log("To         : "+pwd);
                websave(fileName,url);

                if strcmpi(computer("arch"),"win64")
                    unzip(fileName,"uvbin");
                    delete(fileName);
                else
                    tarFileName = gunzip(fileName);
                    delete(fileName);

                    filename = tarFileName{1};
                    untar(filename);
                    delete(filename);

                    dcont = dir('uv-*');
                    movefile(dcont.name,"uvbin");
                end
            end
        end

        function uvout = invokeuv(cmdString,options)
            arguments
                cmdString
                options.EnvironmentVars(1,:) string = []
            end

            MPyReq.uv();

            defaultVars = {...
                            "UV_CACHE_DIR",MPyReq.installFolder+filesep+"uvcache",...
                            "UV_PYTHON_BIN_DIR", MPyReq.installFolder+filesep+"uvpython_bin",...
                            "UV_PYTHON_CACHE_DIR", MPyReq.installFolder+filesep+"uvpython_cache",...
                            "UV_PYTHON_INSTALL_DIR", MPyReq.installFolder+filesep+"uvpython_install"...
                          };

            if isempty(options.EnvironmentVars)
                envVars = defaultVars;
            else
                if mod(numel(options.EnvironmentVars), 2) ~= 0
                    error("'EnvironmentVars' input must be a string array of name/value pairs (even number of elements, valid names and values).");
                end
                envVars = horzcat(defaultVars,options.EnvironmentVars);
            end

            sysCmdString = """"+MPyReq.installFolder+filesep+"uvbin"+filesep+"uv"+""""...
                +" "+cmdString;

            if MPyReq.getInstance().Verbose
                sysCmdString = sysCmdString + " --verbose";
            end

            [s, uvout] = system(sysCmdString,envVars{:});

            if MPyReq.getInstance().Verbose
                disp(uvout);
            end

            if s~=0
                error(uvout);
            end
        end
    end

end

%% Helpers

function restoreWF = cdToNamedFolder(reqName)
wd = pwd;
restoreWF = onCleanup(@()cd(wd));
reqWF = MPyReq.pathTo(reqName);
if ~isfolder(reqWF)
    mkdir(reqWF);
end
cd(reqWF);
end
