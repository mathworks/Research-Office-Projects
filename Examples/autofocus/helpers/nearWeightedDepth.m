function refDepth = nearWeightedDepth(depthPixels, power)
%nearWeightedDepth Median depth of a patch, weighted towards the near content, for plotting.
%   refDepth = nearWeightedDepth(depthPixels) reduces a patch of depth pixels to one reference
%   distance: the median, with each pixel weighted by 1/depth so that nearer content counts for
%   more. It is a display and scoring reference only - the autofocus never sees it.

arguments
    depthPixels (:,1) double
    power (1,1) double = 1
end

v = depthPixels(isfinite(depthPixels) & depthPixels > 0 & depthPixels < 100);
if isempty(v)
    refDepth = NaN;
    return
end

[sorted, order] = sort(v);
cumulative = cumsum(v(order).^(-power));
refDepth = sorted(find(cumulative >= cumulative(end)/2, 1));
end
