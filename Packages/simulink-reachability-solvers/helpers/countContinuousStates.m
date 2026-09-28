function [nc, nd, ne] = countContinuousStates(mdl)
%COUNTCONTINUOUSSTATES How many CONTINUOUS states a model has, and how many discrete.
%   [nc, nd] = countContinuousStates(mdl) returns the number of SCALAR continuous
%   states and the number of scalar discrete ones.
%
%   [nc, nd, ne] = countContinuousStates(mdl) also returns the number of continuous
%   state ELEMENTS, which is a different and usually less useful number.
%
%   SCALARS, NOT ELEMENTS, AND THE DIFFERENCE IS NOT COSMETIC. The engine's initial
%   state is a list of elements, and ONE ELEMENT CAN CARRY SEVERAL SCALARS: a
%   Second-Order Integrator contributes a single element holding both position and
%   velocity. Counting elements instead reports 3 for sldemo_foucault, which has 4
%   scalar states, so the set solver would be propagating a 4-vector while the count
%   said 3. Every caller wants the length of the state vector the solver actually
%   sees: it is the dimension the reachable set lives in, the length setRadii has to
%   match, and the number an nc == 0 test is asking about. So scalars are what nc
%   means, and the element count is available separately.
%
%   WHY THIS EXISTS. A set solver here propagates the CONTINUOUS state vector and
%   nothing else, so nc is the single number that decides whether the method applies
%   at all. nc == 0 is the sharp edge: Simulink then never runs a continuous solver,
%   so it never even CONSTRUCTS the plugin solver. On a Clock -> Unit Delay model
%   neither start() nor step() is ever called, while the model simulates perfectly.
%   Selecting a set solver there is silently inert and the set log stays empty.
%   Nothing can warn from inside the solver, because there is no solver, so the
%   diagnostic has to be asked for.
%
%   WHY IT IS NOT A ONE-LINER. Simulink.BlockDiagram.getInitialState returns two
%   different TYPES depending on the model's SaveFormat, and the state label lives in
%   a differently-capitalised field in each (R2026b):
%
%       SaveFormat = 'Dataset'              -> Simulink.SimulationData.Dataset,
%                                              element property .Label
%       SaveFormat = 'Array' | 'Structure'  -> struct, field .signals.label
%
%   Both are handled here, so a caller never has to know which one it got.
%
%   Labels seen: CSTATE for continuous, DSTATE for discrete block state, DWORK for
%   discrete work vectors. Anything not CSTATE counts as discrete.
%
%   Example
%       [nc, nd] = countContinuousStates('vdp')                  % 2, 0
%       [nc, nd] = countContinuousStates('sldemo_bounce')        % 2, 0
%       [nc, nd, ne] = countContinuousStates('sldemo_foucault')  % 4, 0, 3
%
%   See also SETREACH, PLOTSETTUBE, SAMPLEMODELTRAJECTORIES.

wasLoaded = bdIsLoaded(mdl);
if ~wasLoaded
    load_system(mdl);
end
s = Simulink.BlockDiagram.getInitialState(mdl);

% WID is how many SCALARS each element carries. Both containers state it, in
% different places, and neither is a safe guess from the other: the Dataset gives the
% data itself, while the struct format carries an explicit .dimensions that is
% preferred over numel(values) because a matrix-valued state would flatten either way
% but only .dimensions says so without reshaping.
if isa(s, 'Simulink.SimulationData.Dataset')
    lbl = cell(1, s.numElements);
    wid = zeros(1, s.numElements);
    for k = 1:s.numElements
        lbl{k} = s{k}.Label;
        wid(k) = numel(s{k}.Values.Data);
    end
elseif isstruct(s) && isfield(s, 'signals')
    lbl = {s.signals.label};
    wid = zeros(1, numel(s.signals));
    for k = 1:numel(s.signals)
        if isfield(s.signals(k), 'dimensions') && ~isempty(s.signals(k).dimensions)
            wid(k) = prod(s.signals(k).dimensions);
        else
            wid(k) = numel(s.signals(k).values);
        end
    end
else
    % Do not guess a count from an unrecognised container: a silent 0 here would
    % read as "this model has no continuous states", which is exactly the
    % conclusion that must never be reached by accident.
    error('countContinuousStates:unknownFormat', ...
        ['Simulink.BlockDiagram.getInitialState returned a %s, which this ' ...
         'function does not\nknow how to read. Refusing to guess a state count.'], ...
        class(s));
end

isC = strcmp(lbl, 'CSTATE');
nc  = sum(wid(isC));
nd  = sum(wid(~isC));
ne  = nnz(isC);
end
