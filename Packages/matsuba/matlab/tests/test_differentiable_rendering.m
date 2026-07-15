classdef test_differentiable_rendering < matlab.unittest.TestCase
%TEST_DIFFERENTIABLE_RENDERING Tests for differentiable rendering wrappers.
%   Validates getParam, renderDiff, forwardGrad, and mi.integrator.prb.

    methods (TestClassSetup)
        function setupMitsuba(~)
            addpath(fullfile(fileparts(mfilename("fullpath")), ".."));
            mi.setup;
            mi.setVariant("llvm_ad_rgb");
        end
    end

    %% --- mi.integrator.prb Tests ---

    methods (Test)
        function testPrbDescriptorType(testCase)
            %TESTPRBDESCRIPTORTYPE PRB descriptor has correct type field.
            s = mi.integrator.prb();
            testCase.verifyEqual(s.type, "prb");
            testCase.verifyEqual(s.category_, "integrator");
        end

        function testPrbWithMaxDepth(testCase)
            %TESTPRBWITHMAXDEPTH PRB accepts max_depth option.
            s = mi.integrator.prb(max_depth=6);
            testCase.verifyEqual(s.max_depth, 6);
        end

        function testPrbSceneBuilds(testCase)
            %TESTPRBSCENEBUILDS Scene with PRB integrator builds OK.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            p = scene.params();
            testCase.verifyNotEmpty(p);
        end

        function testPrbRenders(testCase)
            %TESTPRBRENDERS PRB integrator can render an image.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            img = scene.render(SamplesPerPixel=4);
            testCase.verifySize(img, [64 64 3]);
            testCase.verifyGreaterThan(max(img(:)), 0);
        end
    end

    %% --- getParam Tests ---

    methods (Test)
        function testGetParamReturnsValue(testCase)
            %TESTGETPARAMRETURNSVALUE getParam returns a numeric array.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            v = scene.getParam(testCase.redParam());
            testCase.verifyClass(v, "double");
            testCase.verifyGreaterThan(numel(v), 0);
        end

        function testGetParamColor3(testCase)
            %TESTGETPARAMCOLOR3 Color parameter returns 3 elements.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            v = scene.getParam(testCase.redParam());
            testCase.verifyNumElements(v, 3);
        end

        function testGetParamMatchesSetParam(testCase)
            %TESTGETPARAMMATCHESSETPARAM setParam then getParam round-trips.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            expected = [0.1 0.2 0.3];
            scene.setParam(testCase.redParam(), expected);
            actual = scene.getParam(testCase.redParam());
            testCase.verifyEqual(actual, expected, AbsTol=1e-5);
        end

        function testGetParamInvalidName(testCase)
            %TESTGETPARAMINVALIDNAME Bad parameter name throws an error.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            testCase.verifyError( ...
                @() scene.getParam("nonexistent.param.xyz"), ...
                "mi:Scene:GetParamFailed");
        end
    end

    %% --- renderDiff Tests ---

    methods (Test)
        function testRenderDiffReturnsThreeOutputs(testCase)
            %TESTRENDERDIFFRETURNSTHREEOUTPUTS Returns image, loss, grads.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=4);

            [img, loss, grads] = scene.renderDiff(refImg, ...
                testCase.redParam(), SamplesPerPixel=4);

            testCase.verifyClass(img, "double");
            testCase.verifySize(img, [64 64 3]);
            testCase.verifyClass(loss, "double");
            testCase.verifyNumElements(loss, 1);
            testCase.verifyClass(grads, "containers.Map");
        end

        function testRenderDiffGradientForMatchingScene(testCase)
            %TESTRENDERDIFFGRADIENTFORMATCHINGSCENE Loss ~0 when scene matches ref.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=16);

            [~, loss, ~] = scene.renderDiff(refImg, ...
                testCase.redParam(), SamplesPerPixel=16);

            % Loss should be small (not exactly 0 due to MC noise)
            testCase.verifyLessThan(loss, 0.01);
        end

        function testRenderDiffGradientNonzeroWhenPerturbed(testCase)
            %TESTRENDERDIFFGRADIENTNONZEROWHENPERTURBED Gradients are nonzero
            %   when scene differs from reference.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=16);

            % Perturb the ball color
            scene.setParam(testCase.redParam(), [0.01 0.2 0.9]);

            [~, loss, grads] = scene.renderDiff(refImg, ...
                testCase.redParam(), SamplesPerPixel=4);

            testCase.verifyGreaterThan(loss, 0.001);
            g = grads(testCase.redParam());
            testCase.verifyNumElements(g, 3);
            testCase.verifyGreaterThan(norm(g), 0, ...
                "Gradient should be nonzero when scene is perturbed.");
        end

        function testRenderDiffMultipleParams(testCase)
            %TESTRENDERDIFFMULTIPLEPARAMS Can differentiate multiple params.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=4);

            scene.setParam(testCase.redParam(), [0.5 0.5 0.5]);
            scene.setParam(testCase.greenParam(), [0.5 0.5 0.5]);

            [~, ~, grads] = scene.renderDiff(refImg, ...
                [testCase.redParam(); testCase.greenParam()], ...
                SamplesPerPixel=4);

            testCase.verifyTrue(grads.isKey(testCase.redParam()));
            testCase.verifyTrue(grads.isKey(testCase.greenParam()));
            testCase.verifyNumElements(grads(testCase.redParam()), 3);
            testCase.verifyNumElements(grads(testCase.greenParam()), 3);
        end

        function testRenderDiffL1Loss(testCase)
            %TESTRENDERDIFFL1LOSS L1 loss function works.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=4);

            scene.setParam(testCase.redParam(), [0.01 0.2 0.9]);

            [~, loss, grads] = scene.renderDiff(refImg, ...
                testCase.redParam(), ...
                SamplesPerPixel=4, LossFunction="l1");

            testCase.verifyGreaterThan(loss, 0);
            testCase.verifyGreaterThan(norm(grads(testCase.redParam())), 0);
        end

        function testRenderDiffOptimizationConverges(testCase)
            %TESTRENDERDIFFOPTIMIZATIONCONVERGES A few SGD steps reduce loss.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            refImg = scene.render(SamplesPerPixel=32);

            % Perturb color
            scene.setParam(testCase.redParam(), [0.1 0.1 0.8]);

            % Record initial loss
            [~, loss0, ~] = scene.renderDiff(refImg, ...
                testCase.redParam(), SamplesPerPixel=8);

            % Run 15 SGD steps with larger lr (PRB gives strong gradients)
            lr = 0.5;
            for i = 1:15
                [~, ~, grads] = scene.renderDiff(refImg, ...
                    testCase.redParam(), ...
                    SamplesPerPixel=8, Seed=i);
                g = grads(testCase.redParam());
                cur = scene.getParam(testCase.redParam());
                scene.setParam(testCase.redParam(), ...
                    max(min(cur - lr * g, 1), 0));
            end

            % Final loss should be lower
            [~, lossEnd, ~] = scene.renderDiff(refImg, ...
                testCase.redParam(), SamplesPerPixel=8);

            testCase.verifyLessThan(lossEnd, loss0, ...
                "Loss should decrease after optimization steps.");
        end
    end

    %% --- forwardGrad Tests ---

    methods (Test)
        function testForwardGradReturnsImage(testCase)
            %TESTFORWARDGRADRETURNSIMAGE Returns gradient image.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            gradImg = scene.forwardGrad(testCase.greenParam(), ...
                SamplesPerPixel=4);
            testCase.verifyClass(gradImg, "double");
            testCase.verifySize(gradImg, [64 64 3]);
        end

        function testForwardGradNonzero(testCase)
            %TESTFORWARDGRADNONZERO Gradient image has nonzero values.
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            gradImg = scene.forwardGrad(testCase.greenParam(), ...
                SamplesPerPixel=16);
            testCase.verifyGreaterThan(max(abs(gradImg(:))), 0, ...
                "Forward gradient image should have nonzero values.");
        end

        function testForwardGradShowsSpatialSensitivity(testCase)
            %TESTFORWARDGRADSHOWSSPATIALSENSITIVITY Gradient should be
            %   spatially localized (not uniform everywhere).
            scene = testCase.buildCornellBoxPrb();
            cleanup = onCleanup(@() delete(scene));
            gradImg = scene.forwardGrad(testCase.greenParam(), ...
                SamplesPerPixel=32);
            % Check that there's spatial variation (std > 0)
            testCase.verifyGreaterThan(std(gradImg(:)), 0, ...
                "Gradient image should have spatial variation.");
        end
    end

    %% --- Helper Methods ---

    methods (Static, Access = private)
        function scene = buildCornellBoxPrb()
            %BUILDCORNELLBOXPRB Build a small scene with PRB integrator.
            %   Uses a simple sphere scene (not mi.cornellBox) so we can
            %   control the integrator. The sphere has named materials with
            %   known parameter paths: "red.reflectance.value" and
            %   "green.reflectance.value".
            redBsdf = mi.bsdf.diffuse( ...
                reflectance=mi.rgb([0.57 0.043 0.044]), key_="red");
            greenBsdf = mi.bsdf.diffuse( ...
                reflectance=mi.rgb([0.1 0.37 0.067]), key_="green");

            scene = mi.Scene.build( ...
                mi.shape.sphere(radius=0.5, ...
                    to_world=mi.Transform.translate([-0.5 0 0]), ...
                    bsdf=redBsdf, key_="red_ball"), ...
                mi.shape.sphere(radius=0.5, ...
                    to_world=mi.Transform.translate([0.5 0 0]), ...
                    bsdf=greenBsdf, key_="green_ball"), ...
                mi.emitter.constant(radiance=mi.spectrum(value=1)), ...
                mi.integrator.prb(max_depth=4), ...
                mi.sensor.perspective( ...
                    fov=45, ...
                    to_world=mi.Transform.lookAt([0 0 3], [0 0 0], [0 1 0]), ...
                    film=mi.film(Width=64, Height=64)));
        end

        function name = redParam()
            name = "red_ball.bsdf.reflectance.value";
        end

        function name = greenParam()
            name = "green_ball.bsdf.reflectance.value";
        end
    end
end
