classdef HullRep < SetRep
    %HULLREP The between-step enclosure Omega_k: everything reached BETWEEN samples.
    %
    %   Every other representation here answers "where is the set at time t_k". That
    %   leaves a hole, and it is a hole in the CLAIM rather than in the picture: a
    %   reach tube drawn by filling between consecutive sets asserts coverage of the
    %   whole interval, which nothing computed. Plot the samples as markers instead
    %   of lines and the gap is visible -- there is simply nothing between them.
    %
    %   Omega_k closes it. Over one step the solver integrates a FROZEN affine
    %   system, and that flow is exactly a linear flow in one dimension more:
    %
    %       z = [x; 1],   zdot = Atil*z,   Atil = [A f0; 0 0],   z(t) = expm(Atil*t)*z0
    %
    %   which is precisely what SetReach.dense computes. For a linear flow the chord
    %   between the endpoints is second-order accurate, so the curve lives in the
    %   convex hull of its endpoints plus a ball -- Girard's construction, HSCC'05:
    %
    %       Omega_k = CH(X_{k-1}, X_k^flow)  (+)  alpha_k * B
    %
    %   DERIVATION of alpha, since a soundness claim should not rest on a citation.
    %   With tau = t/h in [0,1], compare the flow against the chord through the SAME
    %   z0. Both share their constant and linear terms, and (Atil*t)^j = tau^j
    %   (Atil*h)^j, so the difference telescopes exactly:
    %
    %       z(t) - [(1-tau) z0 + tau expm(Atil*h) z0] = sum_{j>=2} (tau^j - tau) (Atil h)^j / j!  z0
    %
    %   Every term vanishes at tau = 0 and tau = 1, as the whole bound must. Taking
    %   norms term by term,
    %
    %       alpha_k = sum_{j>=2} k_j * M_j * h^j / j!,
    %           k_j = max_tau (tau - tau^j),   M_j = sup_{z in Z_{k-1}} ||Atil^j z||
    %
    %   Note exp(s)-1-s ~ s^2/2, so alpha = O(h^2): halving the step QUARTERS the
    %   between-step gap.
    %
    %   TWO PLACES THE TEXTBOOK FORM GIVES AWAY MORE THAN IT NEEDS TO, both worth
    %   more than a constant factor on real models. Girard's alpha_h takes
    %   k_j <= 1 and ||Atil^j z|| <= ||Atil||^j * R, giving the closed form
    %   (exp(h||Atil||) - 1 - h||Atil||) * R, which is BLOATCRUDE here:
    %
    %     - k_2 is exactly 1/4, not 1, so the leading term is 4x smaller. (k_j does
    %       NOT stay small -- it climbs towards 1 -- but h^j/j! has already killed
    %       those terms, so the leading one is the whole of the gain.)
    %     - ||Atil^j|| <= ||Atil||^j is badly pessimistic when Atil is NILPOTENT,
    %       and a double integrator under gravity has Atil^3 = 0 exactly. Worse,
    %       ||Atil|| there is set by g = 9.81 -- the forcing term, which contributes
    %       nothing past j = 2 -- and it then gets exponentiated.
    %
    %   Evaluating ||Atil^j z|| directly instead, and splitting the set's centre from
    %   its spread so the dominant piece stays exact, is measured at 1025x tighter on
    %   sldemo_bounce at h = 0.05 and 9274x at h = 0.5 -- the gap widens with h
    %   because the crude bound exponentiates where the true remainder terminates.
    %   That is the difference between an enclosure one can plot and one that
    %   swallows the model. On the same system the per-power sum is not a bound at
    %   all but the exact remainder coefficient, (1/4)*g*h^2/2, since Atil^3 = 0
    %   ends the series.
    %
    %   WHY THIS ONE CLASS SERVES ALL FOUR SHAPES. It never touches a payload. A
    %   support function is all it needs, because support functions turn both set
    %   operations into arithmetic -- max for a convex hull, plus for a Minkowski sum:
    %
    %       rho_Omega(d) = max_i ( d'delta_i + rho_i(d) ) + alpha*||Ball'd||
    %
    %   So zonotope, ellipsoid, support function and sensitivity all get an enclosure
    %   from the same code, and it is itself a SetRep, so vertices(), ratio() and
    %   projection work on it unchanged.
    %
    %   ASYMMETRIC, unlike its siblings. A hull of two sets at different centres has
    %   no centre of symmetry, so rho(d) ~= rho(-d) and halfWidths is overridden to
    %   report max(rho(e), rho(-e)) -- the smallest SYMMETRIC box that still contains
    %   the set, so it stays an over-approximation. The inherited ratio() and
    %   vertices() are already correct for asymmetric sets: both sweep the full
    %   circle, and containment is the gauge test d'dx <= rho(d) for every d, which
    %   never assumed symmetry.
    %
    %   WHAT IT DOES NOT DO, stated plainly. alpha bounds the chord error of the
    %   frozen affine flow the solver actually integrated. It does NOT bound the
    %   difference between that flow and the true nonlinear one -- no term here
    %   covers freezing A and f0 at the left endpoint. So on a genuinely linear model
    %   Omega_k is a sound enclosure of the true trajectories, and on a nonlinear one
    %   it is a sound enclosure of what the solver computed. The missing nonlinear
    %   remainder is the linearisation error, and no choice of shape or step
    %   discipline closes it.
    %
    %   THE FREE PARAMETER, noted rather than exploited: z = [x; s] with f0/s in
    %   place of f0 gives the same x-flow for any s > 0, but a different ||Atil||
    %   and a different R, hence a different alpha. s = 1 is a convention, not an
    %   optimum, and minimising alpha over s is a cheap 1-D problem nobody has done.
    %
    %   See also SETREACH/DENSE, ENCLOSESETLOG, SETREP.

    properties (Constant)
        kind = 'enclosure'
    end

    properties
        Members     % cell of SetRep, the sets being hulled
        Offsets     % n-by-numel(Members), member centres relative to the reference
        Ball        % n-by-n map applied to the unit ball; I until mapped
        Alpha       % radius of the bloat, >= 0
    end

    methods
        function obj = HullRep(members, offsets, alpha, ball)
            %HULLREP CH(members, at their offsets) (+) alpha*ball.
            %   Offsets are measured from the REFERENCE centre that support() is
            %   relative to, which is the caller's choice; encloseSetLog uses the
            %   interval's left endpoint, so column 1 is zero.
            if nargin == 0
                return
            end
            if ~iscell(members)
                members = {members};
            end
            n = members{1}.dim();
            if nargin < 3 || isempty(alpha)
                alpha = 0;
            end
            if nargin < 4 || isempty(ball)
                ball = eye(n);
            end
            obj.Members = members;
            obj.Offsets = offsets;
            obj.Alpha   = alpha;
            obj.Ball    = ball;
        end

        function obj = map(obj, Phi)
            % Exact, term by term: a linear map commutes with a convex hull, and
            % Phi*(alpha*B) is the ellipsoid whose support is alpha*||(Phi*Ball)'d||.
            for i = 1:numel(obj.Members)
                obj.Members{i} = obj.Members{i}.map(Phi);
            end
            obj.Offsets = Phi * obj.Offsets;
            obj.Ball    = Phi * obj.Ball;
        end

        function r = support(obj, d)
            % max over members of (offset term + member support), then the bloat.
            r = -Inf;
            for i = 1:numel(obj.Members)
                r = max(r, d' * obj.Offsets(:, i) + obj.Members{i}.support(d));
            end
            if obj.Alpha > 0
                r = r + obj.Alpha * norm(obj.Ball' * d);
            end
        end

        function b = halfWidths(obj)
            % Symmetric box containing an asymmetric set: take the larger side.
            n = obj.dim();
            b = zeros(n, 1);
            for k = 1:n
                e = zeros(n, 1);
                e(k) = 1;
                b(k) = max(obj.support(e), obj.support(-e));
            end
        end

        function s = payload(obj)
            s = sprintf('hull of %d, alpha = %.3g', numel(obj.Members), obj.Alpha);
        end

        function M = payloadMatrix(obj)
            % Ball fixes the dimension and is the only always-present matrix; the
            % members' payloads have shapes that differ by representation.
            M = obj.Ball;
        end

        function obj = project(obj, idx)
            % Exact, as for every rep here: rho_{Px}(d) = rho_x(P'd), and slicing
            % Ball's ROWS is exactly that, since ||Ball'P'd|| = ||(Ball(idx,:))'d||.
            for i = 1:numel(obj.Members)
                obj.Members{i} = obj.Members{i}.project(idx);
            end
            obj.Offsets = obj.Offsets(idx, :);
            obj.Ball    = obj.Ball(idx, :);
        end
    end

    methods (Static)
        function alpha = bloat(A, f0, h, c, S)
            %BLOAT Chord bound for one frozen affine step, term by term.
            %   alpha = HullRep.bloat(A, f0, h, c, S) for the step that starts at
            %   centre c with set S and runs for h under xdot = A*x + f0.
            %
            %   Sums k_j * M_j * h^j/j! over j >= 2 with a norm-only tail, where
            %   k_j is the exact coefficient of the j-th remainder term and M_j
            %   bounds ||Atil^j z|| over the lifted set. Sound by the header's
            %   argument and strictly tighter than BLOATCRUDE, which takes k_j = 1
            %   and ||Atil^j|| <= ||Atil||^j. Measured ratio:
            %   1025x on sldemo_bounce at h = 0.05, 9274x at h = 0.5, because there
            %   Atil is nilpotent and its norm is set by gravity rather than by
            %   anything the flow actually does.
            [Atil, zc, nb, n] = HullRep.lift(A, f0, c, S);

            J     = 12;                 % past here the tail bound is already tiny
            P     = eye(n + 1);
            hj    = 1;                  % running h^j/j!
            alpha = 0;
            nilpotent = false;
            for j = 1:J
                P  = P * Atil;
                hj = hj * h / j;
                if ~any(P(:))
                    % Atil^j is exactly zero, so every remaining term vanishes and
                    % there is no tail to bound. Worth catching rather than being a
                    % curiosity: a double integrator under gravity has Atil^3 = 0,
                    % which is most of why the norm-only bound wastes so much on
                    % sldemo_bounce.
                    nilpotent = true;
                    break
                end
                if j < 2
                    continue            % the j = 0 and j = 1 terms cancel exactly
                end
                % Splitting centre from spread keeps the dominant term exact. When
                % the state is large and the set is small -- a ball at 22 m/s with
                % a 1 mm initial set -- ||Atil^j||*R would be set by the state and
                % pay a matrix norm for it; ||Atil^j*zc|| just evaluates it.
                Mj    = norm(P * zc) + norm(P(:, 1:n)) * nb;
                alpha = alpha + HullRep.chordFactor(j) * Mj * hj;
            end

            if ~nilpotent
                % Tail over j > J, falling back to ||Atil^j|| <= ||Atil||^j and
                % k_j <= 1. All terms positive, so forward summation is stable.
                R    = norm(zc) + nb;
                s    = h * norm(Atil);
                term = 1;
                for j = 1:J
                    term = term * s / j;            % s^J/J!
                end
                tail = 0;
                for j = J+1:J+200
                    term = term * s / j;
                    tail = tail + term;
                    if term < eps * max(tail, realmin)
                        break
                    end
                end
                alpha = alpha + R * tail;
            end

            if ~isfinite(alpha) || alpha < 0
                % Never round DOWN on a soundness constant. An h this large has no
                % usable enclosure, and saying so beats quietly reporting zero.
                alpha = Inf;
            end
        end

        function alpha = bloatCrude(A, f0, h, c, S)
            %BLOATCRUDE The textbook bound, kept only so the gain can be measured.
            %   (exp(h*||Atil||) - 1 - h*||Atil||) * R -- Girard's alpha_h, i.e.
            %   BLOAT with k_j = 1 and ||Atil^j|| <= ||Atil||^j. Sound, and loose
            %   whenever Atil is nilpotent or its norm is dominated by the forcing
            %   term. Nothing in the tube uses it.
            [Atil, zc, nb] = HullRep.lift(A, f0, c, S);
            R = norm(zc) + nb;
            s = h * norm(Atil);
            % exp(s)-1-s at the s ~ 1e-3 that is normal here: the direct form
            % cancels twice over, and expm1(s)-s is no better, because expm1
            % returns s + s^2/2 and subtracting s discards every digit that
            % survived -- about 4% relative error at s = 1e-8. The defining series
            % is all-positive, so sum that instead. Above s = 1 the closed form is
            % well conditioned.
            if s < 1
                e    = 0;
                term = s * s / 2;
                j    = 2;
                while term > eps * max(e, realmin) && j < 80
                    e    = e + term;
                    j    = j + 1;
                    term = term * s / j;
                end
            else
                e = exp(s) - 1 - s;
            end
            alpha = e * R;
        end

        function k = chordFactor(j)
            %CHORDFACTOR max over tau in [0,1] of (tau - tau^j), for j >= 2.
            %   The exact coefficient of the j-th remainder term, maximised at
            %   tau* = (1/j)^(1/(j-1)).
            %
            %   It is 1/4 at j = 2 and then INCREASES -- 0.385, 0.472, ... 0.811 at
            %   j = 20 -- tending to 1 as tau* tends to 1. So the gain over taking
            %   k_j = 1 is concentrated entirely in the leading term, which is where
            %   it matters: h^j/j! suppresses the rest, and at the step sizes in use
            %   the j = 2 term is essentially the whole bound. A factor of four on
            %   that term is the whole of what this buys.
            if j < 2
                k = 0;
                return
            end
            tau = (1 / j) ^ (1 / (j - 1));
            k   = tau - tau ^ j;
        end

        function [Atil, zc, nb, n] = lift(A, f0, c, S)
            %LIFT The frozen affine step as a linear flow one dimension up.
            %   z = [x; 1], zdot = Atil*z reproduces xdot = A*x + f0 exactly, which
            %   is what makes a linear-flow chord bound applicable at all. Returns
            %   the lifted centre zc and nb, a bound on ||dx||_2 over the set.
            n = size(A, 1);
            if isempty(f0)
                f0 = zeros(n, 1);
            end
            Atil = [A, f0(:); zeros(1, n + 1)];
            zc   = [c(:); 1];
            % sup||dx||_2 is itself a MAX of the support function, so it must not be
            % sampled: a sampled max under-reports, which would make alpha unsound.
            % The interval hull is exact per coordinate and |dx_i| <= b_i gives
            % ||dx|| <= ||b||. Costs n exact support evaluations, loose by at most
            % sqrt(n) on a box, and never wrong.
            nb = norm(S.halfWidths());
        end
    end
end
