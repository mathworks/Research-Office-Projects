%% Splitting the initial set of the van der Pol oscillator
% A set solver linearises about one centre trajectory, so the wider the initial
% set, the worse it predicts the states far from that centre. Through the fast
% drop of the vdp cycle the unsplit tube is off by whole state units.
% splitSetReach bisects the initial set and re-simulates each piece from its own
% centre until the HSCC'09 error indicator of every piece is below Tol. This
% script runs it once around the cycle and measures how far each tube is from
% the true states.
%
% About 55 s. Run |setup| once in the session before this script.
%
% See also SPLITSETREACH, PLOTSPLITSETTUBE, VDPREACHTUBE.

%% Register the solvers
registerSetSolvers();

mdl   = 'vdp';
RADII = 0.15;     % half-width of the initial box about the model's own x0
T     = 8;        % one full loop of the limit cycle, both fast drops included
H     = 0.01;     % the step's own error, which no Tol can lower, is 1e-2 here
TOL   = 1;        % accept a piece when its indicator stays at or below this

% vdp is a shipping example model. Typing its name runs a stub that fetches it and
% opens it, which also changes the current folder, so pwd is restored on both paths.
% Simulink will warn that the model file is shadowed by that stub; that is how the
% model is distributed, not a problem with this script.
if ~bdIsLoaded(mdl)
    pwd0 = pwd;
    try
        vdp;
    catch ME
        cd(pwd0);
        rethrow(ME);
    end
    cd(pwd0);
end

% Set explicitly so the numbers below do not depend on the local copy of the model.
% The shipping value is 2. splitSetReach passes everything else through
% SimulationInput, so this is the only change, and it is put back at the end.
muWas = get_param([mdl '/Mu'], 'Gain');
set_param([mdl '/Mu'], 'Gain', '2');

%% Split
% Every split halves both directions of the initial box, 4 children, each a fresh
% simulation of the unmodified model. The first piece, R.pieces(1), is the
% unsplit run the refinement starts from, so it is the baseline.
R = splitSetReach(mdl, Tol = TOL, Radii = RADII, StopTime = T, FixedStep = H);
leaves = R.pieces(R.leaves);
fprintf('%d pieces cover the initial set, %d simulations in %.0f s\n', ...
    numel(leaves), R.stats.sims, R.stats.seconds);
fprintf('largest indicator %.3g against Tol %g; half-step check moved %.3g\n', ...
    max([leaves.err]), TOL, R.stepErr);

%% The truth
% Trajectories the engine computes from points of the same initial box, under
% ode45 at tight tolerances: the 4 corners, which a linear map sends to the
% extremes, and 4 interior points.
S = sampleModelTrajectories(mdl, R.t, RADII, 8);

%% Measure the misses, do not just draw them
% The distance from each true state to the union of the pieces' sets at the same
% time, zero when some piece contains it. Measure the distance, not a count:
% vdp flattens the set onto the limit cycle, and the linearised sets flatten a
% little harder than the truth, so most true states lie a hair outside the thin
% sets. The worst miss is what splitting changes.
for tube = {'unsplit', 'split'}
    if strcmp(tube{1}, 'unsplit'), P = R.pieces(1); else, P = leaves; end
    d = missDistances(P, S);
    fprintf('%-8s worst miss %.3g, median miss %.2g state units\n', ...
        tube{1}, max(d, [], 'all'), median(d(d > 0)));
end

% What an accepted piece means, narrowly: refinement stopped where splitting
% further no longer changes the answer by more than Tol. The indicator compares
% two linearisations, the parent's and the child's, so it is an estimate, not a
% bound, and the union is not a certified enclosure. A smaller Tol brings the
% worst miss down further, at the cost of more pieces.

plotSplitSetTube(R, 'Samples', S, ...
    'Title', sprintf('van der Pol, mu = 2, radii = %g, Tol = %g', RADII, TOL));

%% Leave the shipping model as it was found
set_param([mdl '/Mu'], 'Gain', muWas);
set_param(mdl, 'Dirty', 'off');

%% Local functions
function d = missDistances(P, S)
%MISSDISTANCES Distance from every sample-instant to the union of the pieces of P,
%   numel(S.t)-by-nSamples, in state units. A piece is skipped when the distance
%   to its centre less its radius already exceeds the best so far, which no point
%   of it can beat, so only the few nearby pieces are measured exactly.
[K, n, N] = size(S.X);
C = reshape([P.c], n, K, []);
rad = zeros(K, numel(P));
for j = 1:numel(P)
    rad(:, j) = squeeze(sum(vecnorm(P(j).G, 2, 1), 2));
end
d = zeros(K, N);
for k = 1:K
    c = reshape(C(:, k, :), n, []);
    for s = 1:N
        x = S.X(k, :, s)';
        [lb, order] = sort(vecnorm(x - c) - rad(k, :));
        best = inf;
        for i = 1:numel(order)
            if lb(i) >= best
                break
            end
            j = order(i);
            best = min(best, boxDistance(P(j).G(:, :, k), x - c(:, j)));
            if best <= 1e-12 * max(1, norm(x))
                best = 0;
                break
            end
        end
        d(k, s) = best;
    end
end
end

function d = boxDistance(G, y)
%BOXDISTANCE min ||G*b - y|| over ||b||_inf <= 1, exactly, for a few generators.
%   If the unconstrained solution is in the box it is the optimum, the problem
%   being convex. If not, the optimum lies on a face b_i = +-1, which is the same
%   problem with one generator fewer.
m = size(G, 2);
if m == 0
    d = norm(y);
    return
end
b = lsqminnorm(G, y);
if max(abs(b)) <= 1
    d = norm(G * b - y);
    return
end
d = inf;
for i = 1:m
    rest = [1:i-1, i+1:m];
    for sgn = [-1 1]
        d = min(d, boxDistance(G(:, rest), y - sgn * G(:, i)));
    end
end
end
