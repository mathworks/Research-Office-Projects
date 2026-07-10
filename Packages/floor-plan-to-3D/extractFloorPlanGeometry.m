function geometry = extractFloorPlanGeometry(imagePath, options)
%extractFloorPlanGeometry Extract walls, doors, and windows from a floor plan image.
%
%   geometry = extractFloorPlanGeometry(imagePath) processes a floor plan
%   image and returns a struct with wall polygons, openings, and scale.
%
%   The function uses boundary tracing, polygon simplification, and
%   gap analysis to vectorize walls and detect door/window openings.
%
%   Designed for ISO 128-compliant monochrome architectural floor plans:
%     - Walls drawn as solid black regions on a white background
%     - Doors indicated by quarter-circle arc symbols (swing direction)
%     - Windows indicated by hatching or parallel line patterns
%     - Consistent wall thickness at a given scale
%
%   See also visualizeFloorPlan3D.

arguments
    imagePath (1,1) string {mustBeFile}
    options.ExteriorWallThickness (1,1) double = 0.20 % meters
    options.MinRegionArea (1,1) double = 500 % pixels
    options.PolyTolerance (1,1) double = 0.002
    options.CollinearTolerance (1,1) double = 3 % pixels
    options.SnapTolerance (1,1) double = 5 % pixels
    options.WallHeight (1,1) double = 2.5 % meters
    options.DoorHeight (1,1) double = 2.1 % meters
    options.WindowBottom (1,1) double = 0.9 % meters
    options.WindowTop (1,1) double = 2.0 % meters
    options.ShowPlots (1,1) logical = false
end

%% Read and binarize
img = imread(imagePath);
if size(img, 3) == 3
    gray = rgb2gray(img);
else
    gray = img;
end
[imgH, imgW] = size(gray);

wallMask = gray < 128;
se = strel('disk', 5);
wallsOnly = imopen(wallMask, se);
wallsOnly = imfill(wallsOnly, 'holes');
wallsOnly = bwareaopen(wallsOnly, options.MinRegionArea);

%% Estimate scale from wall thickness
wallSkel = bwskel(wallsOnly, 'MinBranchLength', 30);
distMap = bwdist(~wallsOnly);
skelDist = distMap .* double(wallSkel);
extWallThickPx = 2 * median(skelDist(skelDist > 0));
scale = options.ExteriorWallThickness / extWallThickPx;

%% Vectorize wall regions using boundary tracing
[boundaries, ~] = bwboundaries(wallsOnly, 'noholes');

regions = {};
for k = 1:length(boundaries)
    b = boundaries{k};
    if size(b,1) < 50, continue; end

    % Convert to [x, y] format
    xy = [b(:,2), b(:,1)];

    % Simplify with Douglas-Peucker
    reduced = reducepoly(xy, options.PolyTolerance);

    % Remove duplicate closing vertex
    if norm(reduced(end,:) - reduced(1,:)) < 2
        reduced = reduced(1:end-1,:);
    end

    if size(reduced,1) >= 3
        regions{end+1} = reduced; %#ok<AGROW>
    end
end

%% Simplify collinear vertex runs (fixes staircase diagonals)
for r = 1:length(regions)
    regions{r} = simplifyCollinear(regions{r}, options.CollinearTolerance);
end

%% Axis-snap near-aligned vertices
for r = 1:length(regions)
    regions{r} = axisSnapVertices(regions{r}, options.SnapTolerance);
end

%% Inter-region vertex snapping
regions = snapBetweenRegions(regions, options.SnapTolerance);

%% Label wall-end edges in each polygon
wallEnds = labelWallEnds(regions, extWallThickPx);

%% Pair wall-ends by ray casting: shoot normal from each end, find matching end
openingsPx = pairWallEndsByRay(wallEnds, wallsOnly, wallMask);

%% Detect door arcs and swing lines for classification
[doorArcCentroids, swingLineCentroids] = detectDoorIndicators(wallMask, wallsOnly);

%% Classify openings using door evidence + exterior detection
hullMask = buildExteriorMask(wallsOnly);
openingsPx = classifyOpenings(openingsPx, doorArcCentroids, swingLineCentroids, ...
    hullMask, imgH, imgW, scale);

%% Package output
geometry.Regions = regions;
geometry.Openings = openingsPx;
geometry.Scale = scale;
geometry.ExteriorWallThickness = options.ExteriorWallThickness;
geometry.InteriorWallThickness = round(extWallThickPx/2) * scale;
geometry.WallHeight = options.WallHeight;
geometry.DoorHeight = options.DoorHeight;
geometry.WindowBottom = options.WindowBottom;
geometry.WindowTop = options.WindowTop;
geometry.FloorDimensions = [imgW * scale, imgH * scale];
geometry.ImageSize = [imgH, imgW];

%% Optional visualization
if options.ShowPlots
    visualizeFloorPlan3D(geometry);
end

end

%% === Collinear Simplification ===

function verts = simplifyCollinear(verts, tol)
%simplifyCollinear Collapse runs of near-collinear vertices into endpoints.
    n = size(verts, 1);
    if n < 4, return; end

    keep = 1;
    i = 1;
    while i < n
        bestJ = i + 1;
        for j = i+2:n
            segDir = verts(j,:) - verts(i,:);
            segLen = norm(segDir);
            if segLen < 1, continue; end
            segNorm = [-segDir(2), segDir(1)] / segLen;

            maxDev = 0;
            for m = i+1:j-1
                dev = abs(dot(verts(m,:) - verts(i,:), segNorm));
                if dev > maxDev, maxDev = dev; end
            end

            if maxDev <= tol
                bestJ = j;
            else
                break;
            end
        end
        keep = [keep, bestJ]; %#ok<AGROW>
        i = bestJ;
    end

    if norm(verts(keep(end),:) - verts(keep(1),:)) < 2
        keep = keep(1:end-1);
    end
    verts = verts(keep, :);

    % Wrap-around pass: check if last→first→second are collinear
    n2 = size(verts, 1);
    if n2 >= 3
        segDir = verts(2,:) - verts(n2,:);
        segLen = norm(segDir);
        if segLen > 1
            segNorm = [-segDir(2), segDir(1)] / segLen;
            dev = abs(dot(verts(1,:) - verts(n2,:), segNorm));
            if dev <= tol
                verts = verts(2:end, :);
            end
        end
    end
end

%% === Axis Snapping ===

function verts = axisSnapVertices(verts, tol)
%axisSnapVertices Snap near-horizontal/vertical edges to exact alignment.
    n = size(verts, 1);
    for i = 1:n
        j = mod(i, n) + 1;
        dx = abs(verts(j,1) - verts(i,1));
        dy = abs(verts(j,2) - verts(i,2));
        if dx < tol && dy > tol
            avg = round(mean([verts(i,1), verts(j,1)]));
            verts(i,1) = avg;
            verts(j,1) = avg;
        elseif dy < tol && dx > tol
            avg = round(mean([verts(i,2), verts(j,2)]));
            verts(i,2) = avg;
            verts(j,2) = avg;
        end
    end
end

%% === Inter-Region Snapping ===

function regions = snapBetweenRegions(regions, tol)
%snapBetweenRegions Snap vertices between different regions that are close.
    for r1 = 1:length(regions)
        for vi = 1:size(regions{r1}, 1)
            pt = regions{r1}(vi, :);
            for r2 = 1:length(regions)
                if r2 == r1, continue; end
                for vj = 1:size(regions{r2}, 1)
                    d = norm(regions{r2}(vj,:) - pt);
                    if d > 0 && d < tol
                        avg = round(mean([pt; regions{r2}(vj,:)]));
                        regions{r1}(vi,:) = avg;
                        regions{r2}(vj,:) = avg;
                    end
                end
            end
        end
    end
end

%% === Wall-End Labeling ===

function wallEnds = labelWallEnds(regions, wallThickPx)
%labelWallEnds Identify polygon edges that represent wall terminations.
%   A wall-end is a short edge (≤ 1.2x wall thickness) where both vertices
%   are convex (outside) corners of the polygon.
    wallEnds = struct('p1',{},'p2',{},'region',{},'edgeIdx',{},'normal',{},'length',{});
    maxEndLen = wallThickPx * 1.2;

    for r = 1:length(regions)
        v = regions{r};
        n = size(v, 1);
        if n < 4, continue; end

        % Determine polygon winding via signed area
        area = 0;
        for i = 1:n
            j = mod(i, n) + 1;
            area = area + (v(i,1)*v(j,2) - v(j,1)*v(i,2));
        end
        convexSign = sign(area);

        for i = 1:n
            j = mod(i, n) + 1;
            edgeVec = v(j,:) - v(i,:);
            edgeLen = norm(edgeVec);

            if edgeLen < 5 || edgeLen > maxEndLen, continue; end

            % Check both vertices are convex (outside corners)
            iPrev = mod(i - 2, n) + 1;
            jNext = mod(j, n) + 1;

            eIn_i = v(i,:) - v(iPrev,:);
            eOut_i = v(j,:) - v(i,:);
            cross_i = eIn_i(1)*eOut_i(2) - eIn_i(2)*eOut_i(1);

            eIn_j = v(j,:) - v(i,:);
            eOut_j = v(jNext,:) - v(j,:);
            cross_j = eIn_j(1)*eOut_j(2) - eIn_j(2)*eOut_j(1);

            if (cross_i * convexSign) <= 0, continue; end
            if (cross_j * convexSign) <= 0, continue; end

            k = length(wallEnds) + 1;
            wallEnds(k).p1 = v(i,:);
            wallEnds(k).p2 = v(j,:);
            wallEnds(k).region = r;
            wallEnds(k).edgeIdx = i;
            wallEnds(k).normal = [0 0];
            wallEnds(k).length = edgeLen;
        end

        % Merged wall-ends: two consecutive short collinear edges whose
        % combined length is ≤ maxEndLen and outer vertices are both convex
        for i = 1:n
            j = mod(i, n) + 1;
            jj = mod(j, n) + 1;
            totalVec = v(jj,:) - v(i,:);
            totalLen = norm(totalVec);
            if totalLen < 5 || totalLen > maxEndLen, continue; end

            len1 = norm(v(j,:) - v(i,:));
            len2 = norm(v(jj,:) - v(j,:));
            if len1 < 3 || len2 < 3, continue; end
            if abs(totalLen - (len1 + len2)) > 5, continue; end

            % Outer vertices (i and jj) must be convex
            iPrev = mod(i - 2, n) + 1;
            jjNext = mod(jj, n) + 1;

            eIn_i = v(i,:) - v(iPrev,:);
            eOut_i = v(j,:) - v(i,:);
            cx_i = eIn_i(1)*eOut_i(2) - eIn_i(2)*eOut_i(1);

            eIn_jj = v(jj,:) - v(j,:);
            eOut_jj = v(jjNext,:) - v(jj,:);
            cx_jj = eIn_jj(1)*eOut_jj(2) - eIn_jj(2)*eOut_jj(1);

            if (cx_i * convexSign) <= 0, continue; end
            if (cx_jj * convexSign) <= 0, continue; end

            k = length(wallEnds) + 1;
            wallEnds(k).p1 = v(i,:);
            wallEnds(k).p2 = v(jj,:);
            wallEnds(k).region = r;
            wallEnds(k).edgeIdx = i;
            wallEnds(k).normal = [0 0];
            wallEnds(k).length = totalLen;
        end
    end
end

%% === Wall-End Pairing ===

function openings = pairWallEndsByRay(wallEnds, wallsOnly, wallMask)
%pairWallEndsByRay Shoot rays normal to each wall-end and find matching ends.
%   From each wall-end midpoint, cast a ray perpendicular to the edge in both
%   directions. If the ray hits another wall-end (same angle, within tolerance),
%   that's an opening pair.
    openings = struct('p1',{},'p2',{},'width',{},'type',{},'midpoint',{}, ...
        'thickness',{},'corners',{});
    [imgH, imgW] = size(wallsOnly);
    nEnds = length(wallEnds);
    used = false(1, nEnds);
    maxRayLen = 500;
    angleTol = 0.3; % max angular deviation (radians, ~17 degrees)

    % Precompute midpoints and directions
    mids = zeros(nEnds, 2);
    dirs = zeros(nEnds, 2);
    for i = 1:nEnds
        mids(i,:) = (wallEnds(i).p1 + wallEnds(i).p2) / 2;
        dirs(i,:) = (wallEnds(i).p2 - wallEnds(i).p1) / wallEnds(i).length;
    end

    % For each wall-end, try to find its partner
    for i = 1:nEnds
        if used(i), continue; end

        e1 = wallEnds(i);
        e1dir = dirs(i,:);
        mid1 = mids(i,:);
        % Normal directions (both sides of the edge)
        n1 = [-e1dir(2), e1dir(1)];
        n2 = -n1;

        bestJ = 0;
        bestDist = inf;

        for ni = 1:2
            if ni == 1, rayDir = n1; else, rayDir = n2; end

            % Find closest wall-end hit along this ray
            for j = 1:nEnds
                if j == i || used(j), continue; end

                % Check angle: edges must be roughly parallel
                e2dir = dirs(j,:);
                if abs(dot(e1dir, e2dir)) < cos(angleTol), continue; end

                % Check ray hits near the target midpoint
                mid2 = mids(j,:);
                toTarget = mid2 - mid1;
                dist = dot(toTarget, rayDir);
                if dist < 15 || dist > maxRayLen, continue; end

                % Lateral offset: target should be in-line with ray
                lateral = abs(dot(toTarget, e1dir));
                if lateral > max(e1.length, wallEnds(j).length) * 0.7, continue; end

                % Verify clear gap along ray (no wall between them)
                nCheck = 20;
                checkX = round(linspace(mid1(1), mid2(1), nCheck));
                checkY = round(linspace(mid1(2), mid2(2), nCheck));
                hasWall = false;
                for s = 3:nCheck-2
                    cx = checkX(s); cy = checkY(s);
                    if cy>=1 && cy<=imgH && cx>=1 && cx<=imgW
                        if wallsOnly(cy, cx)
                            hasWall = true; break;
                        end
                    end
                end
                if hasWall, continue; end

                if dist < bestDist
                    bestDist = dist;
                    bestJ = j;
                end
            end
        end

        if bestJ == 0, continue; end

        e2 = wallEnds(bestJ);
        mid2 = mids(bestJ,:);
        gap = norm(mid2 - mid1);

        % Check for window hatching
        nCheck = 20;
        checkX = round(linspace(mid1(1), mid2(1), nCheck));
        checkY = round(linspace(mid1(2), mid2(2), nCheck));
        hasHatching = false;
        for s = 3:nCheck-2
            cx = checkX(s); cy = checkY(s);
            if cy>=1 && cy<=imgH && cx>=1 && cx<=imgW
                if wallMask(cy, cx)
                    hasHatching = true; break;
                end
            end
        end

        % Corners: pair vertices so the quad c1-c2-c4-c3 doesn't self-intersect.
        % Try both pairings and pick the one that forms a simple (non-crossing) quad.
        c1 = e1.p1; c2 = e1.p2;
        % Option A: c3=e2.p1, c4=e2.p2
        % Option B: c3=e2.p2, c4=e2.p1
        % Quad is c1-c2-c4-c3. Check if diagonals c1-c4 and c2-c3 intersect.
        % If they do, use the other option.
        if quadSelfIntersects(c1, c2, e2.p2, e2.p1)
            c3 = e2.p2; c4 = e2.p1;
        else
            c3 = e2.p1; c4 = e2.p2;
        end

        midpoint = (mid1 + mid2) / 2;

        k = length(openings) + 1;
        openings(k).p1 = mid1;
        openings(k).p2 = mid2;
        openings(k).width = gap;
        openings(k).midpoint = midpoint;
        openings(k).type = "";
        openings(k).thickness = max(e1.length, e2.length);
        openings(k).corners = [c1; c2; c3; c4];
        if hasHatching
            openings(k).type = "window_hint";
        end

        used(i) = true;
        used(bestJ) = true;
    end
end

%% === Door Indicator Detection ===

function [doorArcCentroids, swingLineCentroids] = detectDoorIndicators(wallMask, wallsOnly)
%detectDoorIndicators Find door arcs and swing lines in the floor plan.
    wallsDilated = imdilate(wallsOnly, strel('disk', 2));
    thinFeatures = wallMask & ~wallsDilated;
    thinFeatures = bwareaopen(thinFeatures, 50);
    cc = bwconncomp(thinFeatures);
    props = regionprops(cc, 'Area', 'BoundingBox', 'Centroid');

    doorArcCentroids = zeros(0, 2);
    swingLineCentroids = zeros(0, 2);
    for k = 1:length(props)
        bb = props(k).BoundingBox;
        ar = bb(3) / max(bb(4), 1);
        % Door arcs: roughly square, large area
        if ar > 0.7 && ar < 1.5 && props(k).Area > 100 && min(bb(3), bb(4)) > 50
            doorArcCentroids(end+1, :) = props(k).Centroid; %#ok<AGROW>
        % Swing lines: thin and long (aspect ratio very low or very high)
        elseif props(k).Area > 80 && max(bb(3), bb(4)) > 80 && min(bb(3), bb(4)) < 15
            swingLineCentroids(end+1, :) = props(k).Centroid; %#ok<AGROW>
        end
    end
end


%% === Exterior Mask ===

function hullMask = buildExteriorMask(wallsOnly)
%buildExteriorMask Create mask of exterior region using convex hull.
    [imgH, imgW] = size(wallsOnly);
    [rows, cols] = find(wallsOnly);
    hullIdx = convhull(cols, rows);
    hullMask = poly2mask(cols(hullIdx), rows(hullIdx), imgH, imgW);
end

%% === Opening Classification ===

function openings = classifyOpenings(openings, doorArcCentroids, swingLineCentroids, ...
        hullMask, imgH, imgW, scale)
%classifyOpenings Label each opening as door, window, or passage.
    keep = true(1, length(openings));

    for k = 1:length(openings)
        mid = openings(k).midpoint;
        widthM = openings(k).width * scale;

        % Check for nearby door arc
        hasDoorArc = false;
        for d = 1:size(doorArcCentroids, 1)
            if norm(doorArcCentroids(d,:) - mid) < 200
                hasDoorArc = true;
                break;
            end
        end

        % Check for nearby swing line
        hasSwingLine = false;
        for d = 1:size(swingLineCentroids, 1)
            if norm(swingLineCentroids(d,:) - mid) < 100
                hasSwingLine = true;
                break;
            end
        end

        hasDoorEvidence = hasDoorArc || hasSwingLine;

        % Check if opening is on exterior
        isExterior = false;
        mx = round(mid(1)); my = round(mid(2));
        if mx >= 1 && mx <= imgW && my >= 1 && my <= imgH
            dir = openings(k).p2 - openings(k).p1;
            dir = dir / max(norm(dir), 1);
            perp = [-dir(2), dir(1)];
            for offset = [30, 50, 80, 120, 160]
                testPt = round(mid + offset * perp);
                if testPt(1)>=1 && testPt(1)<=imgW && testPt(2)>=1 && testPt(2)<=imgH
                    if ~hullMask(testPt(2), testPt(1))
                        isExterior = true;
                        break;
                    end
                end
                testPt = round(mid - offset * perp);
                if testPt(1)>=1 && testPt(1)<=imgW && testPt(2)>=1 && testPt(2)<=imgH
                    if ~hullMask(testPt(2), testPt(1))
                        isExterior = true;
                        break;
                    end
                end
            end
        end

        % Classification logic
        if openings(k).type == "window_hint"
            openings(k).type = "window";
        elseif isExterior && widthM > 0.8
            openings(k).type = "window";
        elseif widthM > 1.0
            openings(k).type = "passage";
        elseif hasDoorEvidence
            openings(k).type = "door";
        elseif widthM < 1.0 && ~isExterior && ~hasDoorEvidence
            % Interior opening with no door evidence — likely a false positive
            keep(k) = false;
        else
            openings(k).type = "door";
        end
    end

    openings = openings(keep);
end

%% === Quad Intersection Check ===

function crosses = quadSelfIntersects(c1, c2, c4, c3)
%quadSelfIntersects Check if quad c1-c2-c4-c3 has crossing edges.
%   Tests if edge c2-c4 intersects edge c3-c1.
    d1 = c4 - c2; d2 = c1 - c3;
    denom = d1(1)*d2(2) - d1(2)*d2(1);
    if abs(denom) < 1e-10
        crosses = false;
        return;
    end
    t = ((c3(1)-c2(1))*d2(2) - (c3(2)-c2(2))*d2(1)) / denom;
    u = ((c3(1)-c2(1))*d1(2) - (c3(2)-c2(2))*d1(1)) / denom;
    crosses = (t > 0 && t < 1 && u > 0 && u < 1);
end
