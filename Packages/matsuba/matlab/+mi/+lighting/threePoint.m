function shapes = threePoint(options)
%MI.LIGHTING.THREEPOINT Create a 3-point studio lighting rig.
%   SHAPES = MI.LIGHTING.THREEPOINT(target=[0 0 0]) returns a cell array
%   of scene elements (area light rectangles + optional ambient fill) that
%   implement classic key/fill/rim studio lighting aimed at the target.
%
%   SHAPES = MI.LIGHTING.THREEPOINT(Name=Value) customizes the rig:
%       target      - point the lights aim at (default [0 0 0])
%       distance    - distance of lights from target (default 5)
%       intensity   - overall brightness multiplier (default 1)
%       temperature - warm (>1) or cool (<1) color shift (default 1)
%       up          - "Y" or "Z" for scene orientation (default "Y")
%       ambient     - ambient fill radiance (default 0.05, 0 to disable)
%       hide_emitters - if true, include a path integrator with
%                       hide_emitters=true so area lights are invisible
%                       to the camera (default false)
%
%   The returned cell array can be unpacked into mi.Scene.build:
%       lights = mi.lighting.threePoint(target=[0 0.5 0]);
%       scene = mi.Scene.build(myShape, lights{:}, camera, integrator);
%
%   Light layout (top-down view, Y-up):
%       - Key:  front-right, elevated — warm, brightest
%       - Fill: front-left, moderate height — cool, softer
%       - Rim:  behind subject, high — neutral, edge definition
%
%   Example:
%       lights = mi.lighting.threePoint(target=[0 0.5 0], intensity=1.5);
%       lights = mi.lighting.threePoint(up="Z", distance=8);
%
%   See also: mi.emitter.area, mi.emitter.directional
    arguments
        options.target (1,3) double = [0 0 0]
        options.distance (1,1) double = 5
        options.intensity (1,1) double = 1
        options.temperature (1,1) double = 1
        options.up (1,1) string {mustBeMember(options.up, ["Y","Z"])} = "Y"
        options.ambient (1,1) double = 0.05
        options.hide_emitters (1,1) logical = false
    end

    t = options.target;
    d = options.distance;
    I = options.intensity;
    temp = options.temperature;

    % Base radiance values (tuned for path tracing at typical spp)
    keyRad   = I * [150 135 110] .* [temp temp 1/temp];
    fillRad  = I * [30 35 45]    .* [1/temp 1/temp temp];
    rimRad   = I * [50 50 55];
    ambRad   = options.ambient * [1 1 1.05];

    % Light sizes (proportional to distance for consistent softness)
    keySize  = d * 0.25;
    fillSize = d * 0.18;
    rimSize  = d * 0.14;

    if options.up == "Y"
        upVec = [0 1 0];
        keyPos  = t + d * [ 0.6  0.7  0.5];
        fillPos = t + d * [-0.5  0.5  0.3];
        rimPos  = t + d * [ 0.0  0.6 -0.6];
    else
        upVec = [0 0 1];
        keyPos  = t + d * [ 0.6  0.5  0.7];
        fillPos = t + d * [-0.5  0.3  0.5];
        rimPos  = t + d * [ 0.0 -0.6  0.6];
    end

    % Key light — largest, warmest, brightest
    keyT = mi.Transform.lookAt(keyPos, t, upVec) ...
         * mi.Transform.scale([keySize keySize 1]);
    keyLight = mi.shape.rectangle( ...
        emitter=mi.emitter.area(radiance=mi.rgb(keyRad)), ...
        to_world=keyT, key_="key_light");

    % Fill light — softer, cooler, from the opposite side
    fillT = mi.Transform.lookAt(fillPos, t, upVec) ...
          * mi.Transform.scale([fillSize fillSize 1]);
    fillLight = mi.shape.rectangle( ...
        emitter=mi.emitter.area(radiance=mi.rgb(fillRad)), ...
        to_world=fillT, key_="fill_light");

    % Rim light — behind subject, provides edge highlights
    rimT = mi.Transform.lookAt(rimPos, t, upVec) ...
         * mi.Transform.scale([rimSize rimSize 1]);
    rimLight = mi.shape.rectangle( ...
        emitter=mi.emitter.area(radiance=mi.rgb(rimRad)), ...
        to_world=rimT, key_="rim_light");

    shapes = {keyLight, fillLight, rimLight};

    % Ambient fill — constant environment for shadow softening
    if options.ambient > 0
        ambLight = mi.emitter.constant( ...
            radiance=mi.rgb(ambRad), key_="ambient");
        shapes{end+1} = ambLight;
    end

    % Optionally include a path integrator with hide_emitters
    if options.hide_emitters
        shapes{end+1} = mi.integrator.path(hide_emitters=true);
    end
end
