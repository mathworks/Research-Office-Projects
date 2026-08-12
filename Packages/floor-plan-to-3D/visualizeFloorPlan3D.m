function visualizeFloorPlan3D(geometry)
%visualizeFloorPlan3D Render extracted floor plan geometry as a 3D model.
%
%   visualizeFloorPlan3D(geometry) takes the output of extractFloorPlanGeometry
%   and renders walls with proper door/window cutouts (lintels and sills).
%
%   See also extractFloorPlanGeometry.

arguments
    geometry (1,1) struct
end

regions = geometry.Regions;
openings = geometry.Openings;
scale = geometry.Scale;
imgH = geometry.ImageSize(1);
wallHeight = geometry.WallHeight;
doorHeight = geometry.DoorHeight;
windowBottom = geometry.WindowBottom;
windowTop = geometry.WindowTop;

wallColor = [0.85 0.85 0.82];
lintelColor = [0.78 0.78 0.75];

figure('Position', [50 50 1200 800], 'Color', 'w');
hold on; axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(30, 25);
title(sprintf('3D Floor Plan: %d wall regions, %d openings', ...
    length(regions), length(openings)));

%% Render wall regions as extruded polygons
for r = 1:length(regions)
    verts = regions{r};
    if isempty(verts), continue; end

    xm = verts(:,1) * scale;
    ym = (imgH - verts(:,2)) * scale;
    n = size(verts, 1);

    % Floor and ceiling faces
    patch('XData', xm, 'YData', ym, 'ZData', zeros(n,1), ...
        'FaceColor', wallColor, 'EdgeColor', 'none', 'FaceAlpha', 0.9);
    patch('XData', xm, 'YData', ym, 'ZData', wallHeight*ones(n,1), ...
        'FaceColor', wallColor, 'EdgeColor', 'none', 'FaceAlpha', 0.9);

    % Side faces
    for i = 1:n
        j = mod(i, n) + 1;
        fx = [xm(i), xm(j), xm(j), xm(i)];
        fy = [ym(i), ym(j), ym(j), ym(i)];
        fz = [0, 0, wallHeight, wallHeight];
        patch('XData', fx, 'YData', fy, 'ZData', fz, ...
            'FaceColor', wallColor, 'EdgeColor', [0.5 0.5 0.5], ...
            'EdgeAlpha', 0.3, 'FaceAlpha', 0.9);
    end
end

%% Render lintels and sills for openings
for k = 1:length(openings)
    op = openings(k);
    c = op.corners;
    if all(c(:) == 0), continue; end

    cx = c(:,1) * scale;
    cy = (imgH - c(:,2)) * scale;

    if op.type == "door"
        lintelZbot = doorHeight;
        lintelZtop = wallHeight;
        hasSill = false;
    elseif op.type == "window"
        lintelZbot = windowTop;
        lintelZtop = wallHeight;
        sillZbot = 0;
        sillZtop = windowBottom;
        hasSill = true;
    else % passage
        lintelZbot = doorHeight;
        lintelZtop = wallHeight;
        hasSill = false;
    end

    % Form quad: c1, c2, c4, c3
    qx = [cx(1); cx(2); cx(4); cx(3)];
    qy = [cy(1); cy(2); cy(4); cy(3)];

    % Render lintel box (6 faces)
    renderBox(qx, qy, lintelZbot, lintelZtop, lintelColor);

    % Render sill for windows
    if hasSill
        renderBox(qx, qy, sillZbot, sillZtop, lintelColor);
    end
end

%% Render opening indicators
for k = 1:length(openings)
    op = openings(k);
    mx = op.midpoint(1) * scale;
    my = (imgH - op.midpoint(2)) * scale;
    if op.type == "door"
        mz = doorHeight/2; mc = 'r';
    elseif op.type == "window"
        mz = (windowBottom + windowTop)/2; mc = 'b';
    else
        mz = doorHeight/2; mc = 'g';
    end
    plot3(mx, my, mz, 'o', 'Color', mc, 'MarkerSize', 6, 'MarkerFaceColor', mc);
end

lighting gouraud;
camlight('headlight');
hold off;

end

function renderBox(qx, qy, zBot, zTop, faceColor)
%renderBox Render a 6-face box given a quadrilateral footprint and z-range.
    edgeColor = [0.4 0.4 0.4];

    % Bottom and top faces
    patch('XData', qx, 'YData', qy, 'ZData', zBot*ones(4,1), ...
        'FaceColor', faceColor, 'EdgeColor', edgeColor, 'FaceAlpha', 0.95);
    patch('XData', qx, 'YData', qy, 'ZData', zTop*ones(4,1), ...
        'FaceColor', faceColor, 'EdgeColor', edgeColor, 'FaceAlpha', 0.95);

    % 4 side faces
    for i = 1:4
        j = mod(i, 4) + 1;
        fx = [qx(i), qx(j), qx(j), qx(i)];
        fy = [qy(i), qy(j), qy(j), qy(i)];
        fz = [zBot, zBot, zTop, zTop];
        patch('XData', fx, 'YData', fy, 'ZData', fz, ...
            'FaceColor', faceColor, 'EdgeColor', edgeColor, 'FaceAlpha', 0.95);
    end
end
