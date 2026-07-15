classdef test_matlab_logo_caustics < matlab.unittest.TestCase
%TEST_MATLAB_LOGO_CAUSTICS Unit tests for the MATLAB logo caustics example.
%   Validates prism mesh generation, scene construction, rendering,
%   progressive rendering, and post-processing.

    properties (TestParameter)
        GridSize = {10, 25}
    end

    methods (TestClassSetup)
        function setupMitsuba(~)
            addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
            mi.setup;
            mi.setVariant("scalar_rgb");
        end
    end

    %% --- Prism Mesh Generation Tests ---

    methods (Test)
        function testPrismHasTopBottomAndSides(testCase, GridSize)
            %TESTPRISMHASTOPBOTTOMANDSIDES Prism has more faces than a single surface.
            L = 160 * membrane(1, GridSize);
            [~, F] = testCase.logoPrismMesh(L);
            [ny, nx] = size(L);
            maxSingleSurface = 2 * (ny - 1) * (nx - 1);
            testCase.verifyGreaterThan(size(F, 1), maxSingleSurface, ...
                "Prism should have more faces than a single surface.");
        end

        function testPrismVerticesDoubled(testCase, GridSize)
            %TESTPRISMVERTICESDOUBLED Prism has top + bottom vertices.
            L = 160 * membrane(1, GridSize);
            [V, ~] = testCase.logoPrismMesh(L);
            [ny, nx] = size(L);
            testCase.verifyEqual(size(V, 1), 2 * ny * nx, ...
                "Prism should have 2x grid vertices (top + bottom).");
        end

        function testPrismFaceIndicesValid(testCase, GridSize)
            %TESTPRISMFACEINDICESVALID All face indices reference valid vertices.
            L = 160 * membrane(1, GridSize);
            [V, F] = testCase.logoPrismMesh(L);
            nVerts = size(V, 1);
            testCase.verifyGreaterThanOrEqual(min(F(:)), 1);
            testCase.verifyLessThanOrEqual(max(F(:)), nVerts);
        end

        function testPrismHeightRange(testCase)
            %TESTPRISMHEIGHTRANGE Bottom vertices at 0.3, top peaks above.
            L = 160 * membrane(1, 20);
            [V, ~] = testCase.logoPrismMesh(L);
            nVerts = size(V, 1) / 2;
            bottomY = V(nVerts+1:end, 2);
            testCase.verifyGreaterThanOrEqual(min(bottomY), 0.3 - 1e-10, ...
                "Bottom of prism should be at 0.3.");
            testCase.verifyGreaterThan(max(V(:,2)), 0.3, ...
                "Top of prism should be above 0.3.");
        end

        function testPrismIsClosed(testCase)
            %TESTPRISMISCLOSED Prism mesh should have no boundary edges.
            L = 160 * membrane(1, 10);
            [~, F] = testCase.logoPrismMesh(L);
            edges = [F(:,[1 2]); F(:,[2 3]); F(:,[3 1])];
            edgesNorm = sort(edges, 2);
            [~, ~, ic] = unique(edgesNorm, "rows");
            edgeCounts = accumarray(ic, 1);
            boundaryCount = sum(edgeCounts == 1);
            testCase.verifyEqual(boundaryCount, 0, ...
                "Closed mesh should have no boundary edges.");
        end

        function testPrismNoZeroQuadrantTopFaces(testCase)
            %TESTPRISMNOZEROQUADRANTTOPFACES No top face has all-zero heights.
            L = 160 * membrane(1, 20);
            Z = L / max(abs(L(:))) * 0.8;
            heights = Z(:);
            [~, F] = testCase.logoPrismMesh(L);
            nVerts = numel(heights);
            % Top faces use indices 1..nVerts
            for i = 1:size(F, 1)
                verts = F(i, :);
                if all(verts <= nVerts)
                    testCase.verifyTrue(any(abs(heights(verts)) > 1e-6), ...
                        sprintf("Top face %d has all-zero height vertices.", i));
                end
            end
        end
    end

    %% --- Scene Construction Tests ---

    methods (Test)
        function testSceneBuilds(testCase)
            %TESTSCENEBUILDS Scene builds without errors.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            testCase.verifyClass(scene, "mi.Scene");
        end

        function testSceneHasExpectedKeys(testCase)
            %TESTSCENEHASEXPECTEDKEYS Scene has named components.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            k = scene.keys();
            testCase.verifyTrue(ismember("logo", k));
            testCase.verifyTrue(ismember("room", k));
            testCase.verifyTrue(ismember("cyan_light", k));
            testCase.verifyTrue(ismember("warm_light", k));
        end

        function testSceneHasParams(testCase)
            %TESTSCENEHASPARAMS Built scene exposes parameters.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            p = scene.params();
            testCase.verifyNotEmpty(p);
        end

        function testDielectricBsdf(testCase)
            %TESTDIELECTRICBSDF Dielectric BSDF struct has correct type.
            bsdf = mi.bsdf.dielectric(int_ior=1.5);
            testCase.verifyEqual(bsdf.type, "dielectric");
            testCase.verifyEqual(bsdf.int_ior, 1.5);
        end
    end

    %% --- Rendering Tests ---

    methods (Test)
        function testRenderProducesImage(testCase)
            %TESTRENDERPRODUCESIMAGE Render output is a valid image.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            img = scene.render(SamplesPerPixel=4);
            testCase.verifyClass(img, "double");
            testCase.verifySize(img, [64 64 3]);
        end

        function testRenderNonBlack(testCase)
            %TESTRENDERNONBLACK Image has non-zero pixels.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            img = scene.render(SamplesPerPixel=8);
            testCase.verifyGreaterThan(max(img(:)), 0);
        end

        function testRenderProgressiveProducesImage(testCase)
            %TESTRENDERPROGRESSIVEPRODUCESIMAGE Progressive render works.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            img = scene.renderProgressive(SamplesPerPixel=8, PassSize=4);
            testCase.verifyClass(img, "double");
            testCase.verifySize(img, [64 64 3]);
            testCase.verifyGreaterThan(max(img(:)), 0);
        end

        function testRenderProgressiveDefaultPassSize(testCase)
            %TESTRENDERPROGRESSIVEDEFAULTPASSSIZE Auto pass size works.
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            img = scene.renderProgressive(SamplesPerPixel=8);
            testCase.verifySize(img, [64 64 3]);
        end
    end

    %% --- Post-processing Tests ---

    methods (Test)
        function testPostprocessOutputRange(testCase)
            %TESTPOSTPROCESSOUTPUTRANGE Output is in [0, 1].
            scene = testCase.buildSmallScene();
            cleanup = onCleanup(@() delete(scene));
            img = scene.render(SamplesPerPixel=4);
            rgb = mi.postprocess(img);
            testCase.verifyGreaterThanOrEqual(min(rgb(:)), 0);
            testCase.verifyLessThanOrEqual(max(rgb(:)), 1);
        end

        function testPostprocessClampsFireflies(testCase)
            %TESTPOSTPROCESSCLAMPSFIREFLIES Extreme values are clamped.
            hdr = zeros(16, 16, 3);
            hdr(8, 8, :) = 100;
            rgb = mi.postprocess(hdr);
            testCase.verifyLessThanOrEqual(max(rgb(:)), 1);
        end

        function testPostprocessPreservesSize(testCase)
            %TESTPOSTPROCESSPRESERVESSIZE Output size matches input.
            hdr = rand(32, 32, 3);
            rgb = mi.postprocess(hdr);
            testCase.verifySize(rgb, [32 32 3]);
        end
    end

    %% --- Helper Methods ---

    methods (Static, Access = private)
        function [V, F] = logoPrismMesh(L)
            %LOGOPRISMMESH Build solid prism mesh (matches example).
            [ny, nx] = size(L);
            [X, Y] = meshgrid(linspace(-1, 1, nx), linspace(-1, 1, ny));
            Z = L / max(abs(L(:))) * 0.8;
            V_top = [X(:), Z(:), Y(:)];
            heights = Z(:);
            nVerts = size(V_top, 1);

            allF = zeros(2 * (ny - 1) * (nx - 1), 3);
            idx = 0;
            for row = 1:(ny - 1)
                for col = 1:(nx - 1)
                    v1 = (row - 1) * nx + col;
                    v2 = v1 + 1;
                    v3 = row * nx + col;
                    v4 = v3 + 1;
                    idx = idx + 1;
                    allF(idx, :) = [v1 v2 v4];
                    idx = idx + 1;
                    allF(idx, :) = [v1 v4 v3];
                end
            end
            keep = false(idx, 1);
            for i = 1:idx
                if any(abs(heights(allF(i, :))) > 1e-6)
                    keep(i) = true;
                end
            end
            topF = allF(keep, :);

            V_bottom = V_top;
            V_bottom(:, 2) = 0;
            bottomF = topF(:, [1 3 2]) + nVerts;

            edges = [topF(:,[1 2]); topF(:,[2 3]); topF(:,[3 1])];
            edgesNorm = sort(edges, 2);
            [uniqueEdges, ~, ic] = unique(edgesNorm, "rows");
            edgeCounts = accumarray(ic, 1);
            boundaryEdges = uniqueEdges(edgeCounts == 1, :);

            nBE = size(boundaryEdges, 1);
            sideF = zeros(2 * nBE, 3);
            for i = 1:nBE
                a = boundaryEdges(i, 1);
                b = boundaryEdges(i, 2);
                c = a + nVerts;
                d = b + nVerts;
                sideF(2*i-1, :) = [a b d];
                sideF(2*i,   :) = [a d c];
            end

            V = [V_top; V_bottom];
            F = [topF; bottomF; sideF];
            V(:,2) = V(:,2) + 0.3;
        end

        function scene = buildSmallScene()
            %BUILDSMALLSCENE Build small version of the caustics scene.
            L = 160 * membrane(1, 10);
            [V, F] = test_matlab_logo_caustics.logoPrismMesh(L);

            glassBsdf = mi.bsdf.dielectric(int_ior=1.5, ...
                specular_transmittance=mi.rgb([1.0 0.6 0.3]));
            logoShape = mi.shape.fromMesh(V, F, bsdf=glassBsdf, key_="logo");

            wallMat = mi.bsdf.diffuse(reflectance=mi.rgb([0.75 0.75 0.78]));
            roomT = mi.Transform.translate([-5 0 -5]) ...
                  * mi.Transform.scale([11 8 11]);
            roomShape = mi.shape.cube(flip_normals=true, ...
                bsdf=wallMat, to_world=roomT, key_="room");

            cyanLight = mi.emitter.point(position=[-0.5 3 2], ...
                intensity=mi.rgb([0 300 300]), key_="cyan_light");
            warmLight = mi.emitter.point(position=[2 4 -1], ...
                intensity=mi.rgb([350 280 0]), key_="warm_light");
            fillLight = mi.emitter.point(position=[0 5 0], ...
                intensity=mi.spectrum(value=80), key_="fill_light");
            envLight = mi.emitter.constant( ...
                radiance=mi.rgb([0.02 0.02 0.03]), key_="env");

            camera = mi.sensor.perspective( ...
                fov=38, ...
                to_world=mi.Transform.lookAt([3.5 2.0 3.5], [0 0.3 0], [0 1 0]), ...
                film=mi.film(Width=64, Height=64), ...
                key_="sensor");
            integrator = mi.integrator.path(max_depth=8);

            scene = mi.Scene.build( ...
                logoShape, roomShape, ...
                cyanLight, warmLight, fillLight, envLight, ...
                camera, integrator);
        end
    end
end
