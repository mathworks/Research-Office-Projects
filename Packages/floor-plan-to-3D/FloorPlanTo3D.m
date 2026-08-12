%[text] # Floor Plan to 3D Model
%[text] This example shows how to convert an architectural floor plan image into a 3D model with walls, doors, and windows. Each step of the pipeline is visualized so you can see how the algorithm progresses from raw pixels to a complete 3D scene.
%[text] The approach aims to work with **ISO 128-compliant monochrome floor plans.** Solid black walls on white, quarter-circle door arcs, and hatched window symbols.
%%
%[text] ## Read the Floor Plan
%[text] The input is a standard monochrome architectural floor plan at any resolution.
img = imread("test_floorplan.png");
imshow(img)
title("Input Floor Plan")
%%
%[text] ## Step 1: Binarize and Clean the Wall Mask
%[text] Threshold the image to isolate wall pixels, then use morphological opening to remove thin features (like door arcs and text), fill interior holes, and discard small noise regions.
gray = rgb2gray(img);
wallMask = gray < 128;
se = strel('disk', 5);
wallsOnly = imopen(wallMask, se);
wallsOnly = imfill(wallsOnly, 'holes');
wallsOnly = bwareaopen(wallsOnly, 500);

imshow(wallsOnly)
title("Cleaned Wall Mask (morphological opening + fill + area filter)")

%%
%[text] ## Step 2: Estimate the Drawing Scale
%[text] The scale is computed automatically from the wall thickness. Skeletonize the walls and measure the distance transform along the skeleton - this gives the half-thickness at every point. The median value, doubled, is the exterior wall thickness in pixels. Dividing the known physical thickness (0.20 m) by this pixel measurement gives the scale factor.

wallSkel = bwskel(wallsOnly, 'MinBranchLength', 30);
distMap = bwdist(~wallsOnly);
skelDist = distMap .* double(wallSkel);
extWallThickPx = 2 * median(skelDist(skelDist > 0));
scale = 0.20 / extWallThickPx;

imshow(wallsOnly); hold on
[sy, sx] = find(wallSkel);
plot(sx, sy, 'r.', 'MarkerSize', 1)
hold off
title(sprintf("Wall Skeleton - estimated thickness: %.0f px, scale: %.4f m/px", ...
    extWallThickPx, scale))

%%
%[text] ## Step 3: Trace and Simplify Wall Boundaries
%[text] `bwboundaries` traces the outline of each connected wall region. Each boundary can have thousands of points, so `reducepoly` (Douglas-Peucker algorithm) simplifies them down to a manageable set of vertices. A collinear simplification pass then collapses staircase patterns on diagonal walls into clean straight edges.
[boundaries, ~] = bwboundaries(wallsOnly, 'noholes');

% Simplify each boundary
polyTol = 0.002;
collinearTol = 3;
regions = {};
for k = 1:length(boundaries)
    b = boundaries{k};
    if size(b,1) < 50, continue; end
    xy = [b(:,2), b(:,1)];
    reduced = reducepoly(xy, polyTol);
    if norm(reduced(end,:) - reduced(1,:)) < 2
        reduced = reduced(1:end-1,:);
    end
    if size(reduced,1) >= 3
        regions{end+1} = reduced; %#ok<SAGROW>
    end
end

imshow(img); hold on
colors = lines(length(regions));
for r = 1:length(regions)
    v = regions{r};
    plot([v(:,1); v(1,1)], [v(:,2); v(1,2)], '-', ...
        'Color', colors(r,:), 'LineWidth', 2)
    centroid = mean(v);
    text(centroid(1), centroid(2), sprintf("R%d\n(%d verts)", r, size(v,1)), ...
        'Color', colors(r,:), 'FontWeight', 'bold', 'HorizontalAlignment', 'center')
end
hold off
title(sprintf("Vectorized Wall Regions: %d regions", length(regions)))

%%
%[text] ## Step 4: Identify Wall-End Edges
%[text] Each polygon edge is classified as a **wall face** (long, runs along the wall) or a **wall end** (short, ~one wall-thickness, both vertices are convex corners). Wall-end edges mark where a wall terminates - the natural boundary of a door, window, or passage opening.
geometry = extractFloorPlanGeometry("test_floorplan.png");

imshow(img); hold on
for r = 1:length(regions)
    v = regions{r};
    n = size(v,1);
    area = 0;
    for i = 1:n, j = mod(i,n)+1; area = area + (v(i,1)*v(j,2) - v(j,1)*v(i,2)); end
    convexSign = sign(area);
    for i = 1:n
        j = mod(i,n)+1;
        edgeVec = v(j,:)-v(i,:); edgeLen = norm(edgeVec);
        isEnd = false;
        if edgeLen <= extWallThickPx*1.2 && edgeLen >= 5
            iPrev = mod(i-2,n)+1; jNext = mod(j,n)+1;
            eIn_i = v(i,:)-v(iPrev,:); eOut_i = v(j,:)-v(i,:);
            cx_i = eIn_i(1)*eOut_i(2)-eIn_i(2)*eOut_i(1);
            eIn_j = v(j,:)-v(i,:); eOut_j = v(jNext,:)-v(j,:);
            cx_j = eIn_j(1)*eOut_j(2)-eIn_j(2)*eOut_j(1);
            isEnd = (cx_i*convexSign)>0 && (cx_j*convexSign)>0;
        end
        if isEnd
            plot([v(i,1),v(j,1)], [v(i,2),v(j,2)], 'r-', 'LineWidth', 3)
        else
            plot([v(i,1),v(j,1)], [v(i,2),v(j,2)], '-', 'Color', [0 0.6 0], 'LineWidth', 1.5)
        end
    end
end
hold off
title("Wall Edges: faces (green) and ends (red)")

%%
%[text] ## Step 5: Pair Wall-Ends into Openings
%[text] A ray is cast perpendicular to each wall-end edge. If it hits another wall-end with the same orientation (parallel, in-line, clear gap between), that pair defines an opening. The four vertices of the two wall-end edges become the lintel/sill corners directly - no bitmap re-measurement needed.
imshow(img); hold on
for k = 1:length(geometry.Openings)
    o = geometry.Openings(k);
    plot([o.p1(1), o.p2(1)], [o.p1(2), o.p2(2)], 'c-', 'LineWidth', 3)
    cr = o.corners;
    qx = [cr(1,1); cr(2,1); cr(4,1); cr(3,1); cr(1,1)];
    qy = [cr(1,2); cr(2,2); cr(4,2); cr(3,2); cr(1,2)];
    plot(qx, qy, 'm-', 'LineWidth', 1.5)
    text(o.midpoint(1)+10, o.midpoint(2), sprintf("#%d (%.2fm)", k, o.width*geometry.Scale), ...
        'Color', 'c', 'FontSize', 8, 'FontWeight', 'bold')
end
hold off
title(sprintf("Detected Openings: %d paired wall-ends", length(geometry.Openings)))

%%
%[text] ## Step 6: Classify Openings
%[text] Each opening is classified as a **door**, **window**, or **passage** using three signals:
imshow(img); hold on
for k = 1:length(geometry.Openings)
    o = geometry.Openings(k);
    if o.type == "door"
        c = [1 0.2 0.2];
    elseif o.type == "window"
        c = [0.2 0.4 1];
    else
        c = [0.2 0.8 0.2];
    end
    plot([o.p1(1), o.p2(1)], [o.p1(2), o.p2(2)], '-', ...
        'Color', c, 'LineWidth', 4)
    text(o.midpoint(1)+10, o.midpoint(2), string(o.type), ...
        'Color', c, 'FontSize', 9, 'FontWeight', 'bold')
end
hold off
title("Classified Openings: doors (red), windows (blue), passages (green)")
%%
%[text] ## Step 7: Lintel and Sill Geometry
%[text] The four corner points of each opening come directly from the paired wall-end vertices. For doors, the lintel spans from door height to ceiling. For windows, both a lintel (above) and sill (below) are rendered. This geometry is exact - no pixel re-scanning required.
imshow(img); hold on
for k = 1:length(geometry.Openings)
    o = geometry.Openings(k);
    c = o.corners;
    if all(c(:)==0), continue; end
    qx = [c(1,1); c(2,1); c(4,1); c(3,1); c(1,1)];
    qy = [c(1,2); c(2,2); c(4,2); c(3,2); c(1,2)];
    plot(qx, qy, 'm-', 'LineWidth', 2)
    plot(c(:,1), c(:,2), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm')
end
hold off
title("Lintel/Sill Corner Geometry (magenta quads)")

%%
%[text] ## Final Result: 3D Visualization
%[text] All the extracted geometry is combined into an interactive 3D model. Walls are extruded from the 2D polygons, doors get a lintel overhead, and windows get both a lintel and a sill creating a rectangular cutout.
visualizeFloorPlan3D(geometry)
%[text] ##
