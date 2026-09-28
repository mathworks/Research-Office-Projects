function V = zonotopeVertices(c, G)
%ZONOTOPEVERTICES Exact vertices of a 2-D zonotope, in counterclockwise order.
%
%   V = zonotopeVertices(c, G) returns a 2-by-K vertex list for
%   Z = { c + G*b : ||b||_inf <= 1 } with c 2-by-1 and G 2-by-m.
%
%   No convex hull and no LP. A 2-D zonotope with m non-parallel generators has
%   exactly 2m vertices, and they come out in order from one sweep: flip every
%   generator into the upper half plane (legal, since a zonotope is symmetric in
%   each generator), sort by angle, start at the bottom vertex c - sum(g_i), and
%   walk adding 2*g_i. That traces the lower boundary; the upper boundary is the
%   reflection through c.
%
%   Handles the degenerate cases this solver actually produces: G collapsing
%   toward rank 1 gives a sliver, and an all-zero or empty G gives the single
%   point c.
%
%   See also ZONOTOPEREP, PLOTSETTUBE.

tol = 1e-14;

if isempty(G)
    V = c;
    return
end

G = G(:, vecnorm(G) > tol);          % drop null generators
if isempty(G)
    V = c;
    return
end

% Flip into the upper half plane (ties on the x-axis go to +x).
neg = G(2,:) < -tol | (abs(G(2,:)) <= tol & G(1,:) < 0);
G(:, neg) = -G(:, neg);

[~, idx] = sort(atan2(G(2,:), G(1,:)));
G = G(:, idx);
m = size(G, 2);

% Lower boundary: m+1 vertices from the bottom-most point.
low = zeros(2, m+1);
p = c - sum(G, 2);
for i = 1:m
    low(:, i) = p;
    p = p + 2*G(:, i);
end
low(:, m+1) = p;

% Upper boundary is the reflection through c; the two endpoints are shared.
V = [low, 2*c - low(:, 2:m)];
end
