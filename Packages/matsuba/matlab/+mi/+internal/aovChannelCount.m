function info = aovChannelCount()
%MI.INTERNAL.AOVCHANNELCOUNT Number of output channels per AOV type.
%   Single source of truth for AOV channel counts used by both
%   mi.Scene.renderAOV and mi.integrator.aov.

    persistent cached
    if isempty(cached)
        cached.depth       = 1;
        cached.position    = 3;
        cached.uv          = 2;
        cached.geo_normal  = 3;
        cached.sh_normal   = 3;
        cached.dp_du       = 3;
        cached.dp_dv       = 3;
        cached.prim_index  = 1;
        cached.shape_index = 1;
    end
    info = cached;
end
