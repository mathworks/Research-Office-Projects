function rgb = postprocess(hdr, options)
%MI.POSTPROCESS Tonemap and denoise a linear HDR render for display.
%   RGB = MI.POSTPROCESS(HDR) applies firefly removal, luminance-based
%   Reinhard tonemapping, sRGB gamma correction, and adaptive denoising to
%   a linear HDR image from Mitsuba.
%
%   RGB = MI.POSTPROCESS(HDR, FireflyClamp=T) sets the luminance percentile
%   used for firefly clamping. T is the percentile ceiling (default 99.9).
%   Use a lower value (e.g. 99) for scenes with heavy caustic fireflies.
%   Set to 100 to disable clamping.
%
%   RGB = MI.POSTPROCESS(HDR, Denoise="adaptive") uses a gated median filter
%   that only replaces outlier pixels (default). "median" applies a blanket
%   3x3 median. "none" skips denoising entirely.
%
%   RGB = MI.POSTPROCESS(HDR, Tonemap="reinhard") applies luminance-based
%   Reinhard tonemapping (default) — preserves color ratios and highlight
%   detail. "aces" applies luminance-based ACES filmic curve — punchier
%   midtone contrast, but compresses highlights more aggressively.
%   "none" skips tonemapping.
%
%   RGB = MI.POSTPROCESS(HDR, Exposure=E) applies an exposure multiplier
%   before tonemapping (default 1.0). Values > 1 brighten the image.
%
%   RGB = MI.POSTPROCESS(HDR, DenoiseStrength=K) controls the outlier
%   threshold for adaptive denoising. Higher values are more conservative
%   (fewer pixels replaced). Default is 3.
%
%   Example:
%       hdr = scene.render(SamplesPerPixel=64);
%       rgb = mi.postprocess(hdr);
%       imshow(rgb);
%
%       % Heavy caustic scene — aggressive firefly removal
%       rgb = mi.postprocess(hdr, FireflyClamp=99, DenoiseStrength=2);
%
%   See also: mi.Scene/render, mi.Scene/renderProgressive

    arguments
        hdr (:,:,3) double
        options.FireflyClamp (1,1) double {mustBeBetween(options.FireflyClamp, 0, 100)} = 99.9
        options.Tonemap (1,1) string {mustBeMember(options.Tonemap, ["reinhard","aces","none"])} = "reinhard"
        options.Exposure (1,1) double {mustBePositive} = 1.0
        options.Denoise (1,1) string {mustBeMember(options.Denoise, ["adaptive","median","none"])} = "adaptive"
        options.DenoiseStrength (1,1) double {mustBePositive} = 3
    end

    % --- Step 1: Firefly clamping (in linear HDR space) ---
    if options.FireflyClamp < 100
        luminance = 0.2126*hdr(:,:,1) + 0.7152*hdr(:,:,2) + 0.0722*hdr(:,:,3);
        threshold = prctile(luminance(:), options.FireflyClamp);
        threshold = max(threshold, 1e-6);
        scale = min(threshold ./ max(luminance, 1e-6), 1);
        hdr = hdr .* scale;
    end

    % --- Step 2: Exposure ---
    if options.Exposure ~= 1.0
        hdr = hdr * options.Exposure;
    end

    % --- Step 3: Tonemapping ---
    switch options.Tonemap
        case "aces"
            % Luminance-based ACES filmic curve (Narkowicz 2015 fit)
            % Applied to luminance to preserve highlight detail and color ratios
            lum = 0.2126*hdr(:,:,1) + 0.7152*hdr(:,:,2) + 0.0722*hdr(:,:,3);
            a = 2.51; b = 0.03; c = 2.43; d = 0.59; e = 0.14;
            lumTM = max((lum .* (a * lum + b)) ./ (lum .* (c * lum + d) + e), 0);
            scale = lumTM ./ max(lum, 1e-8);
            hdr = hdr .* scale;
        case "reinhard"
            % Luminance-based Reinhard — preserves color ratios
            lum = 0.2126*hdr(:,:,1) + 0.7152*hdr(:,:,2) + 0.0722*hdr(:,:,3);
            lumTM = lum ./ (1 + lum);
            scale = lumTM ./ max(lum, 1e-8);
            hdr = hdr .* scale;
        otherwise
            % "none" — no tonemapping
    end

    % --- Step 4: sRGB OETF ---
    linear = max(hdr, 0);
    linear = min(linear, 1);
    rgb = (linear <= 0.0031308) .* (12.92 * linear) ...
        + (linear > 0.0031308) .* (1.055 * linear.^(1/2.4) - 0.055);

    % --- Step 5: Denoising ---
    switch options.Denoise
        case "adaptive"
            rgb = adaptiveMedian(rgb, options.DenoiseStrength);
        case "median"
            for c = 1:3
                rgb(:,:,c) = medfilt2(rgb(:,:,c), [3 3]);
            end
        otherwise
            % "none" — no denoising
    end
end

function img = adaptiveMedian(img, k)
%ADAPTIVEMEDIAN Gated median filter — replaces only outlier pixels.
%   For each pixel, computes the local median and MAD (median absolute
%   deviation) in a 3x3 neighborhood. Pixels deviating from the local
%   median by more than k * MAD are replaced with the median value.
%   Non-outlier pixels are left untouched, preserving edges and detail.

    for c = 1:3
        ch = img(:,:,c);

        % Local median in 3x3 window
        med = medfilt2(ch, [3 3]);

        % Local MAD (median absolute deviation) — robust noise estimator
        % ordfilt2 with the 5th element of 9 gives median of the |deviation|
        absDev = abs(ch - med);
        localMAD = ordfilt2(absDev, 5, ones(3));

        % Identify outlier pixels: deviation > k * MAD
        % Add a small floor to MAD to avoid division issues in flat regions
        isOutlier = absDev > k * max(localMAD, 1e-4);

        % Replace only outlier pixels with the local median
        ch(isOutlier) = med(isOutlier);
        img(:,:,c) = ch;
    end
end
