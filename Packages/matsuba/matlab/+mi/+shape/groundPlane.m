function s = groundPlane(options)
%MI.SHAPE.GROUNDPLANE Create a ground plane for product/studio rendering.
%   S = MI.SHAPE.GROUNDPLANE() creates a large diffuse grey ground plane
%   at Y=0 (suitable for Y-up scenes).
%
%   S = MI.SHAPE.GROUNDPLANE(Name=Value) customizes the plane:
%       height      - vertical position (default 0)
%       size        - half-extent of the plane (default 10)
%       up          - "Y" or "Z" for scene orientation (default "Y")
%       reflectance - surface color as [r g b] (default [0.5 0.5 0.5])
%       bsdf        - custom BSDF descriptor (overrides reflectance)
%       key_        - scene key name (default "ground")
%
%   Examples:
%       ground = mi.shape.groundPlane();
%       ground = mi.shape.groundPlane(up="Z", height=-0.01);
%       ground = mi.shape.groundPlane(reflectance=[0.8 0.8 0.8], size=20);
%       ground = mi.shape.groundPlane(bsdf=mi.bsdf.plastic(diffuse_reflectance=mi.rgb([0.3 0.3 0.35])));
%
%   See also: mi.shape.rectangle, mi.bsdf.diffuse
    arguments
        options.height (1,1) double = 0
        options.size (1,1) double = 10
        options.up (1,1) string {mustBeMember(options.up, ["Y","Z"])} = "Y"
        options.reflectance double = [0.5 0.5 0.5]
        options.bsdf struct = struct([])
        options.key_ string = "ground"
    end

    if isempty(options.bsdf)
        mat = mi.bsdf.diffuse(reflectance=mi.rgb(options.reflectance));
    else
        mat = options.bsdf;
    end

    sz = options.size;
    h = options.height;

    if options.up == "Y"
        % Plane in XZ at y=height, normal pointing +Y
        T = mi.Transform.lookAt([0 h 0], [0 h+1 0], [0 0 1]) ...
          * mi.Transform.scale([sz sz 1]);
    else
        % Plane in XY at z=height, normal pointing +Z
        T = mi.Transform.lookAt([0 0 h], [0 0 h+1], [0 1 0]) ...
          * mi.Transform.scale([sz sz 1]);
    end

    s = mi.shape.rectangle(bsdf=mat, to_world=T, key_=options.key_);
end
