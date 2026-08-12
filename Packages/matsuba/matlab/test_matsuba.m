%% MATSUBA Smoke Test
% Minimal integration test for the mi package.
% Requires: Python with mitsuba and numpy installed, a scene XML file.

%% Setup
fprintf("=== MATSUBA Smoke Test ===\n\n");

%% 1. Configure Python
fprintf("1. Configuring Python...\n");
mi.setup;

%% 2. Set variant
fprintf("2. Setting variant...\n");
mi.setVariant("scalar_rgb");
fprintf("   Variant set to scalar_rgb\n");

%% 3. Transform utilities (pure MATLAB, no Python needed)
fprintf("3. Testing Transform utilities...\n");

T1 = mi.Transform.translate([1 2 3]);
assert(isequal(size(T1), [4 4]), "translate should return 4x4");
assert(T1(1,4) == 1 && T1(2,4) == 2 && T1(3,4) == 3, "translate values");

T2 = mi.Transform.scale(2);
assert(T2(1,1) == 2 && T2(2,2) == 2 && T2(3,3) == 2, "uniform scale");

T3 = mi.Transform.scale([1 2 3]);
assert(T3(1,1) == 1 && T3(2,2) == 2 && T3(3,3) == 3, "per-axis scale");

T4 = mi.Transform.lookAt([0 0 5], [0 0 0], [0 1 0]);
assert(isequal(size(T4), [4 4]), "lookAt should return 4x4");

T5 = mi.Transform.translate([1 0 0]) * mi.Transform.scale(1.2);
assert(isequal(size(T5), [4 4]), "composed transform should be 4x4");

fprintf("   All transform tests passed\n");

%% 4. Load scene
fprintf("4. Loading scene...\n");
% Use Mitsuba's built-in cornell_box scene via Python dict
% (avoids needing an XML file on disk)
scene = mi.Scene.fromStruct(struct("type", "scene"));
fprintf("   Scene loaded (ID-based)\n");

%% 5. List parameters
fprintf("5. Listing parameters...\n");
p = scene.params();
fprintf("   Found %d parameters\n", numel(p));

%% 6. Render
fprintf("6. Rendering...\n");
img = scene.render(SamplesPerPixel=4);
assert(isnumeric(img), "render output must be numeric");
assert(ndims(img) >= 2, "render output must be at least 2D");
fprintf("   Rendered image: %s, class=%s\n", mat2str(size(img)), class(img));

%% 7. Cleanup
fprintf("7. Cleaning up...\n");
delete(scene);
fprintf("   Scene released\n");

%% 8. Error handling tests
fprintf("8. Testing error handling...\n");

% Invalid scene path
try
    badScene = mi.Scene("nonexistent_file_xyz.xml");
    error("Should have thrown");
catch ex
    assert(contains(ex.message, "Failed") || contains(ex.message, "not found"), ...
        "Expected meaningful error for bad path");
    fprintf("   Invalid path error: OK\n");
end

fprintf("\n=== All tests passed ===\n");
