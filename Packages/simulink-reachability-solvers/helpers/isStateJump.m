function [jumped, dev] = isStateJump(cLogged, x, relTol)
%ISSTATEJUMP Did a continuous STATE jump, or only its derivative?
%   jumped = isStateJump(cLogged, x) compares the state the engine hands back in
%   reset() against the centre the solver had already propagated to that time.
%   Far apart means a state jumped and the set cannot be transported. Equal means
%   the STATE is continuous and only xdot broke, in which case the set is still
%   perfectly valid and must not be reported as meaningless.
%
%   WHY THIS EXISTS. reset(obj, t, x, dx) does NOT mean "a state jumped". Measured:
%   it fires whenever the engine's integration history is invalidated, which
%   includes a DERIVATIVE discontinuity across a state that never moves. A Pulse
%   Generator driving xdot = -x + u(t) at Ts = 0.1 fires reset at every pulse edge
%   (t = 0, 0.2, 0.4, 0.6, 0.8) while x stays continuous throughout, and on that
%   model the reported set is EXACT -- the containment ratio against 21 sampled
%   trajectories is 1.000000002801, and the half-width matches the analytic
%   w*exp(-T) to 1.3e-10. Warning "the set after this point is not meaningful"
%   there is a false alarm on an exact answer, which is worse than saying nothing:
%   it trains users to ignore the one diagnostic that matters on a bouncing ball.
%
%   THE DISCRIMINATOR SEPARATES THE TWO CASES BY ORDERS OF MAGNITUDE. On
%   sldemo_bounce the impact hands back a velocity of +17.72 where the solver had
%   propagated to -22.12, a deviation of 39.87 against a state norm of ~22, so the
%   relative deviation is ~1.8. On the pulse model it is 6e-11. Any threshold in
%   between works; the default is 1e-6.
%
%   FALSE POSITIVES ARE THE SAFE DIRECTION and this can produce them: on a
%   nonlinear model the logged centre carries the solver's own linearisation error,
%   so a derivative-only reset late in a long run could read as a jump. That errs
%   towards warning, which is the right way to be wrong. It cannot produce a false
%   NEGATIVE unless a genuine jump is smaller than relTol, which would be a jump
%   the set already covers.
%
%   cLogged may be empty, meaning nothing had been logged at that time yet -- the
%   t = 0 establishing reset, or a variable-step root strictly inside a step. Then
%   there is nothing to compare and jumped is false: a caller that wants to treat
%   an unknown as a jump should test isempty(cLogged) itself.
%
%   See also SETREACH/RESET, SETREACHVAR/RESET.
%
if nargin < 3 || isempty(relTol)
    relTol = 1e-6;
end
if isempty(cLogged)
    jumped = false;
    dev    = NaN;
    return
end
x   = x(:);
c   = cLogged(:);
dev = norm(x - c);
% Relative to the state's own size, with a floor of 1 so that a state near zero is
% compared absolutely rather than amplifying roundoff into a jump.
jumped = dev > relTol * max(1, norm(x));
end
