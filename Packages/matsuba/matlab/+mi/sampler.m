function s = sampler(options)
%MI.SAMPLER Create a sampler descriptor for Mitsuba.
%   S = MI.SAMPLER() returns a default independent sampler (4 samples).
%   S = MI.SAMPLER(Count=64) specifies the sample count.
%
%   Example:
%       s = mi.sampler(Count=256, Type="stratified");
    arguments
        options.Count double = 4
        options.Type string = "independent"
        options.key_ string = string.empty
    end

    opts.sample_count = options.Count;
    opts.key_ = options.key_;

    s = mi.internal.pluginStruct(options.Type, "sampler", opts);
end
