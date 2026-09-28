function Lq = resampleSetLog(L, tq)
%RESAMPLESETLOG Report a set-reachability log on a grid of your choosing.
%   Lq = resampleSetLog(L, tq) returns a log-shaped struct holding the centre
%   and the set at each time in tq, built from the log L that SetReach or
%   SetReachVar produced. tq need not be a subset of L.t.
%
%   WHY THIS EXISTS. A variable-step run stops wherever accuracy and zero
%   crossings send it, so its log lands on times like 2.1307 rather than 2.13.
%   Comparing such a run against ground truth then means comparing two curves
%   sampled at different times, which is not a comparison at all -- and asking
%   ode45 for output at 2.1307 just moves the arbitrariness rather than removing
%   it. Adaptive stepping is an internal accuracy decision and should not leak
%   into the display sampling. So: adapt internally, present uniformly.
%
%   NOT AN INTERPOLANT. Entry k of the log carries A and f0 for the step that
%   produced it, and step() advanced the set by expm([A f0; 0]*h). Asking for
%   tau < h evaluates the SAME map at a different time, so a resampled point is
%   what the solver would have returned had the engine stopped there. Nothing is
%   approximated that freezing A at t_{k-1} had not already approximated, and
%   tq = L.t reproduces L exactly. Compare
%   spline-interpolating the centres, which has no such property and cannot
%   produce the SET at all -- there is no meaningful interpolation between two
%   zonotopes, but there is an exact linear map from one to the other.
%
%   EXACT HITS WIN. If tq(i) coincides with a logged time, the logged entry is
%   returned verbatim rather than recomputed. That matters at a state jump: the
%   entry at a reset time carries the POST-jump centre (SetReach.correctCentre),
%   which the pre-jump linearisation cannot reproduce by construction.
%
%   Times before L.t(1) or after L.t(end) are dropped rather than extrapolated;
%   Lq.t reports what survived.
%
%   Example -- a variable-step run, reported every 2 ms:
%       set_param(mdl, 'SolverType', 'Variable-step');
%       set_param(mdl, 'Solver', 'SetReachVarZonotope');
%       sim(mdl);
%       L  = SetReach.getLog();          % non-uniform
%       Lq = resampleSetLog(L, 0:0.002:3);
%
%   See also SETREACH/DENSE, SETREACHVAR.

arguments
    L  struct
    tq (:,1) double
end

Lq = SetReach.emptyLog();
% Carry the source-model stamp across: a resampled log is the SAME run on a different
% grid, and dropping the stamp here would leave plotSetTube unable to name the model
% for exactly the variable-step runs that need resampling most.
if isfield(L, 'mdl')
    Lq.mdl = L.mdl;
end
if isempty(L.t)
    return
end
if ~isfield(L, 'f0')
    error('resampleSetLog:oldLog', ...
        ['This log has no f0 field, so its dense output cannot be rebuilt. ' ...
         'It came from a SetReach older than the one on the path.']);
end

t = L.t(:);
% Absolute rather than relative, and scaled by the span: times here are model
% times of order 1..10, and a relative test would misbehave near t = 0.
tol = 1e-9 * max(1, t(end) - t(1));

for i = 1:numel(tq)
    ti = tq(i);
    if ti < t(1) - tol || ti > t(end) + tol
        continue                            % no extrapolation, ever
    end

    % An exact hit returns the logged PAYLOAD, under the REQUESTED time. The
    % distinction is not pedantic: the caller asked for 0.74 and a solver that
    % accumulated 0.037 twenty times holds 0.74000000000000005, so returning the
    % logged time would put a stray ulp into a grid the caller intends to be
    % uniform -- and then isequal(Lq.t, tq) is false and every downstream
    % alignment test has to carry a tolerance. The two agree to within tol by
    % construction, so there is nothing to lose by reporting the round number.
    %
    % Last rather than first match, because a discarded step can leave two
    % entries at one time only if a caller has mismanaged rewind, and the later
    % one is then the surviving truth.
    k = find(abs(t - ti) < tol, 1, 'last');
    if ~isempty(k)
        Lq = append(Lq, ti, L.c{k}, L.S{k}, L.mu(k), L.A{k}, L.h(k), L.f0{k});
        continue
    end

    % Strictly inside the step that ends at entry k: re-evaluate that step's own
    % affine flow at tau instead of h.
    k   = find(t > ti, 1, 'first');
    tau = ti - t(k - 1);
    [Phi, d] = SetReach.dense(L.A{k}, L.f0{k}, tau);
    Lq = append(Lq, ti, L.c{k-1}(:) + d, L.S{k-1}.map(Phi), ...
        L.mu(k), L.A{k}, tau, L.f0{k});
end
end

function L = append(L, t, c, S, mu, A, h, f0)
L.t(end+1, 1)  = t;
L.c{end+1, 1}  = c;
L.S{end+1, 1}  = S;
L.mu(end+1, 1) = mu;
L.A{end+1, 1}  = A;
L.h(end+1, 1)  = h;
L.f0{end+1, 1} = f0;
end
