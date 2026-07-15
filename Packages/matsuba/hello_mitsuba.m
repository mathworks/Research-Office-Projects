%% Hello Mitsuba! — MATSUBA Getting Started
% A minimal script that renders Mitsuba's built-in Cornell box scene
% from MATLAB, demonstrating setup, rendering, tonemapping, and scene editing.

%% 1. Setup — configure Python and Mitsuba
addpath("matlab");
mi.setup()
mi.setVariant("scalar_rgb");

%% 2. Load the built-in Cornell box scene
scene = mi.cornellBox();
fprintf("Scene loaded. Available parameters:\n");
disp(scene.params());

%% 3. Render and tonemap the default scene
% Mitsuba returns linear HDR images — we tonemap in MATLAB for display.
img = scene.render(SamplesPerPixel=256);
figure("Name", "Hello Mitsuba!", Position=[100 100 1200 400]);

subplot(1, 3, 1);
imshow(mi.postprocess(img));
title("Original");

%% 4. Change a material color — make the red wall blue
scene.setParam("red.reflectance.value", [0.1 0.1 0.8]);
img2 = scene.render(SamplesPerPixel=256);

subplot(1, 3, 2);
imshow(mi.postprocess(img2));
title("Blue Wall");

%% 5. Move the camera slightly to the side
scene.camera.lookAt([0.25 0 3], [0 0 0], [0 1 0]);
img3 = scene.render(SamplesPerPixel=256);

subplot(1, 3, 3);
imshow(mi.postprocess(img3));
title("Camera Moved");

%% Cleanup
delete(scene);
fprintf("Done! Mitsuba is working from MATLAB.\n");


