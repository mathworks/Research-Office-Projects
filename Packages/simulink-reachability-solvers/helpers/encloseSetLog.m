function E = encloseSetLog(L, opts)
%ENCLOSESETLOG Cover the gaps between a set log's samples.
%   E = encloseSetLog(L) returns one enclosure per interval of the log L that
%   SetReach or SetReachVar produced. Entry i of E covers the CLOSED interval
%   [E.tL(i), E.tR(i)] -- not just its endpoints -- so the union over i is a
%   genuine reach TUBE rather than a stack of snapshots.
%
%   WHY IT IS NEEDED. The log holds sets at times. Drawing a filled band between
%   consecutive sets silently asserts something about the times in between that
%   nothing computed, and at a coarse step that assertion can be false: between
%   two samples the trajectory bulges outside the chord joining them. Markers
%   rather than lines make the gap honest; this function closes it.
%
%   HOW. Over one step the solver integrated a frozen affine system, whose flow is
%   linear in one dimension more, so the chord between the endpoints is second
%   order and Omega = CH(endpoints) (+) alpha*B holds with alpha = O(h^2). See
%   HullRep, which derives alpha and evaluates the hull through support functions
%   so all four shapes are served by one implementation.
%
%   THE RIGHT ENDPOINT IS RECONSTRUCTED, NOT READ, and this is the subtle part. At
%   a state jump SetReach.correctCentre OVERWRITES the logged centre with the
%   engine's post-jump state, because that is what the engine carries forward and
%   what any comparison against tout must use. But the interval before the jump was
%   flowed to the PRE-jump state, and hulling to the post-jump one instead is not
%   merely loose -- it can MISS the pre-jump endpoint entirely. On sldemo_bounce
%   the velocity arrives at about -22.1 and leaves at +17.7, and -22.1 is outside
%   the hull of the arrival and departure velocities. So every interval's right
%   endpoint is rebuilt from that entry's own dense output,
%
%       [Phi, d] = SetReach.dense(L.A{k}, L.f0{k}, tau)
%
%   which is the flow endpoint whether or not a jump followed it, and needs no
%   special case. Where the two differ, E.jump(i) is true and E.gap(i) reports how
%   far apart they are -- a direct readout of the jump the solver could not see.
%
%   The post-jump set is deliberately NOT hulled in. Smearing an instantaneous jump
%   across the interval would inflate the tube for no gain in soundness: the
%   post-jump state is the LEFT endpoint of the next interval, so the union over
%   all intervals already contains it.
%
%   SOUND IS NOT THE SAME AS USEFUL. alpha is O(h^2), and a variable-step solver
%   is free to take a long step wherever the error estimate allows one: on vdp it
%   takes h = 0.4, where norm(A)*h is about 2.8 and the chord bound returns
%   alpha = 2.9 around a trajectory of amplitude 2. That enclosure is correct and
%   tells you nothing. Subdividing fixes it, because the union of sub-interval
%   enclosures covers the same closed interval while each one's bloat falls with
%   the square of the sub-step:
%
%       E = encloseSetLog(L, MaxBloat = 0.05 * max(width of the sampled sets))
%
%   splits each logged interval into as many equal parts as that bound needs, and
%   leaves the majority -- which already satisfy it -- untouched. The default is
%   Inf, meaning one enclosure per logged interval exactly as before.
%
%   Options
%     MaxBloat   split each interval until every sub-interval's alpha is at or
%                below this. Inf leaves intervals unsplit.        (default Inf)
%     MaxSplit   cap on sub-intervals per logged interval, so a stiff step
%                cannot make this unbounded.                       (default 64)
%
%   FIELDS. All numeric fields are nInt-by-1, all cells nInt-by-1. With splitting,
%   nInt exceeds numel(L.t) - 1 and several entries can share one logged step:
%       tL, tR    interval endpoints
%       c         reference centre, = the left endpoint's centre. Omega's support
%                 is measured from here, as for every SetRep.
%       Om        the HullRep covering [tL, tR]
%       alpha     its bloat radius, the whole of what the sampled sets missed
%       xFlow     the reconstructed flow endpoint centre
%       jump      true where the logged endpoint is not the flow endpoint
%       gap       norm(logged endpoint - flow endpoint), zero unless jump
%
%   Example -- is a point reachable at a time nobody sampled?
%       L = SetReach.getLog();
%       E = encloseSetLog(L);
%       i = find(E.tL <= t & t <= E.tR, 1);
%       inside = E.Om{i}.ratio(x - E.c{i}) <= 1;
%
%   See also HULLREP, SETREACH/DENSE, RESAMPLESETLOG.

arguments
    L struct
    opts.MaxBloat (1,1) double {mustBePositive} = Inf
    opts.MaxSplit (1,1) double {mustBePositive} = 64
end

E = struct('tL', [], 'tR', [], 'c', {{}}, 'Om', {{}}, 'alpha', [], ...
    'xFlow', {{}}, 'jump', logical([]), 'gap', []);

if ~isfield(L, 'f0')
    error('encloseSetLog:oldLog', ...
        ['This log has no f0 field, so a step''s flow cannot be rebuilt and the ' ...
         'between-step enclosure is not computable. It came from a SetReach ' ...
         'older than the one on the path.']);
end
if numel(L.t) < 2
    return
end

t = L.t(:);
for k = 2:numel(t)
    % The interval ACTUALLY spanned, which is what the flow must be evaluated
    % over. L.h(k) is normally the same number, but it is the h that was passed
    % to step(), and across a discarded step or a reset the surviving gap can be
    % shorter. The gap is the authority here; h is not.
    tau = t(k) - t(k - 1);
    % Zero-length intervals are dropped, and the threshold is roundoff-scaled
    % rather than exactly zero because that is how they actually turn up: the
    % engine emits a final step of 2.2e-16 to land on StopTime, so the log holds
    % t = 2 twice. Such an "interval" has alpha ~ 1e-32 and encloses nothing
    % beyond its own endpoint, but it is not harmless -- its enclosure degenerates
    % to the endpoint SET, whose boundary the true trajectories touch exactly, so
    % it reports a containment ratio of 1 and drowns out every real interval in a
    % worst-case sweep. Nothing is lost by dropping it: no time elapses, and the
    % duplicated sample is still the endpoint of the interval before it.
    if tau <= 8 * eps * max(1, abs(t(k)))
        continue
    end

    A  = L.A{k};
    f0 = L.f0{k};
    cL = L.c{k - 1}(:);
    SL = L.S{k - 1};
    n  = numel(cL);

    % THE OFFSET HANDED TO bloat IS NOT THE ONE HANDED TO dense, and the two must
    % not be confused. The log stores the frozen field in DEVIATION form -- f0 is
    % f(c), so xdot = f0 + A*(x - c) and dense() integrates zdot = A*z + f0 in
    % z = x - c, which is why the endpoint below is cL + d and not Phi*cL + d.
    % HullRep.bloat is stated for the ABSOLUTE form xdot = A*x + g0, so it needs
    % g0 = f0 - A*c. Passing the deviation offset together with the absolute
    % centre bounds neither field: it adds a spurious A^j*c to every term of the
    % series, which on vdp inflates alpha by 1.3x to 1.7x and draws the tube as a
    % chain of discs wider than the trajectory they cover. Worse, the spurious
    % term is signed, so where A^j*c opposes A^(j-1)*f0 it SHRINKS the bound, and
    % an under-estimated alpha is not an enclosure at all.
    g0 = f0 - A * cL;

    % One enclosure per sub-interval. m = 1 reproduces the single-hull behaviour
    % exactly, which is the default, so existing callers see no change. Splitting
    % is the only way to make a tube TIGHT as well as sound: alpha is O(ds^2), so
    % halving the sub-step quarters the bloat, while the union over sub-intervals
    % still covers the same closed interval. It is worth doing per interval rather
    % than globally because the need is wildly uneven -- on a variable-step vdp run
    % the median interval needs nothing and a handful of 0.4-long steps carry all
    % the looseness.
    m = 1;
    if isfinite(opts.MaxBloat)
        while m < opts.MaxSplit && ...
                max(subBloat(A, g0, f0, tau, cL, SL, m)) > opts.MaxBloat
            m = 2 * m;
        end
    end

    s = linspace(0, tau, m + 1);
    for q = 1:m
        [PhiL, dL] = SetReach.dense(A, f0, s(q));
        [PhiR, dR] = SetReach.dense(A, f0, s(q + 1));
        cq  = cL + dL;
        Sq  = SL.map(PhiL);
        cqR = cL + dR;
        SqR = SL.map(PhiR);

        % Reference centre is the sub-interval's left endpoint, so member 1 sits
        % at the origin -- the same convention as the unsplit case.
        Om = HullRep({Sq, SqR}, [zeros(n, 1), cqR - cq], ...
            HullRep.bloat(A, g0, s(q + 1) - s(q), cq, Sq));

        E.tL(end+1, 1)    = t(k - 1) + s(q);
        E.tR(end+1, 1)    = t(k - 1) + s(q + 1);
        E.c{end+1, 1}     = cq;
        E.Om{end+1, 1}    = Om;
        E.alpha(end+1, 1) = Om.Alpha;
        E.xFlow{end+1, 1} = cqR;
        % gap and jump describe the LOGGED step, so they are reported on the
        % sub-interval that ends where the step ends and are zero elsewhere:
        % nothing jumped in the middle of a step the solver took whole.
        if q == m
            E.gap(end+1, 1) = norm(L.c{k}(:) - cqR);
        else
            E.gap(end+1, 1) = 0;
        end
        % A tolerance rather than an exact test: the logged endpoint was produced
        % by the same expm, so absent a jump the two agree to roundoff, not
        % exactly -- dense() is called here with the GAP where step() used h.
        E.jump(end+1, 1)  = E.gap(end) > 1e-9 * max(1, norm(cqR));
    end
end
end

% ------------------------------------------------------------------- helpers

function a = subBloat(A, g0, f0, tau, cL, SL, m)
%SUBBLOAT The bloat each of m equal sub-intervals would need. Measured, not
%predicted from the O(ds^2) law: the law is asymptotic, the split is not, and the
%whole point of the search is to stop as soon as the real number is small enough.
s = linspace(0, tau, m + 1);
a = zeros(1, m);
for q = 1:m
    [Phi, d] = SetReach.dense(A, f0, s(q));
    a(q) = HullRep.bloat(A, g0, s(q + 1) - s(q), cL + d, SL.map(Phi));
end
end
