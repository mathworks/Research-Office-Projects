classdef SolarPanelApp < handle
    properties (Access = private)
        Fig
        MapAxes
        MapRef          % MapCellsReference from readBasemapImage
        AddressField
        LoadButton
        AddPanelButton
        TiltSlider
        TiltLabel
        DailyAxes
        MonthlyAxes
        TotalLabel
        PanelListBox
        DeletePanelButton
        ZoomLevel double = 20
        MapContextMenu
        PanelContextMenu
        IsPanning logical = false
        PanStart = []

        Latitude double = 42.3010
        Longitude double = -71.3744
        Panels = {}     % ROI rectangle handles
        PanelTilts = [] % per-panel tilt angles (degrees)
        SunEdges = {}   % line handles showing sun-facing edge
        SelectedPanel = 0 % index of currently selected panel
        OptimizeDropdown
        RealisticCheckBox
        ExportButton
        SunArcHandles = {}  % handles to sun path dot/label objects
        SunArcVisible logical = true
        SunArcAxes          % transparent overlay axes for sun dots
    end

    methods
        function app = SolarPanelApp()
            app.buildUI();
            app.loadMap();
        end
    end

    methods (Access = private)
        function buildUI(app)
            app.Fig = uifigure('Name', 'Solar Panel Planner', ...
                'Position', [100 50 1300 820], ...
                'Theme', 'dark');

            mainGrid = uigridlayout(app.Fig, [2 2], ...
                'RowHeight', {40, '1x'}, ...
                'ColumnWidth', {'1x', 380}, ...
                'Padding', 8, 'RowSpacing', 6, 'ColumnSpacing', 8);

            % ── Top toolbar ──
            topBar = uigridlayout(mainGrid, [1 6], ...
                'RowHeight', {32}, ...
                'ColumnWidth', {'1x', 80, 20, 110, 90, 105}, ...
                'Padding', [10 4 10 4], 'ColumnSpacing', 8);
            topBar.Layout.Row = 1;
            topBar.Layout.Column = [1 2];
            topBar.BackgroundColor = [0.14 0.14 0.16];

            app.AddressField = uieditfield(topBar, 'text', ...
                'Value', '1 Lakeside Campus Drive, Natick, MA', ...
                'Placeholder', 'Enter address...');
            app.LoadButton = uibutton(topBar, 'Text', 'Load', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.loadAddress), ...
                'BackgroundColor', [0.22 0.45 0.7], 'FontColor', 'w');
            % Spacer
            uilabel(topBar, 'Text', '');
            app.AddPanelButton = uibutton(topBar, 'Text', '+ Add Panel', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.addPanel), ...
                'BackgroundColor', [0.18 0.55 0.34], 'FontColor', 'w', ...
                'FontWeight', 'bold');
            uibutton(topBar, 'Text', 'Clear All', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.clearPanels), ...
                'BackgroundColor', [0.55 0.18 0.18], 'FontColor', 'w');
            app.ExportButton = uibutton(topBar, 'Text', 'Export PDF', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.exportReport), ...
                'BackgroundColor', [0.5 0.3 0.65], 'FontColor', 'w');

            % ── Map axes ──
            app.MapAxes = uiaxes(mainGrid);
            app.MapAxes.Layout.Row = 2;
            app.MapAxes.Layout.Column = 1;
            axis(app.MapAxes, 'off');
            app.MapAxes.Toolbar.Visible = 'on';
            disableDefaultInteractivity(app.MapAxes);

            % Overlay axes for sun sundial (fixed position, transparent)
            app.SunArcAxes = uiaxes(mainGrid);
            app.SunArcAxes.Layout.Row = 2;
            app.SunArcAxes.Layout.Column = 1;
            app.SunArcAxes.Color = 'none';
            app.SunArcAxes.XColor = 'none';
            app.SunArcAxes.YColor = 'none';
            app.SunArcAxes.Toolbar.Visible = 'off';
            app.SunArcAxes.HitTest = 'off';
            app.SunArcAxes.PickableParts = 'none';
            xlim(app.SunArcAxes, [0 1]);
            ylim(app.SunArcAxes, [0 1]);
            disableDefaultInteractivity(app.SunArcAxes);
            app.Fig.WindowScrollWheelFcn = @(~,evt) app.safeCall(@() app.onScroll(evt));
            app.Fig.WindowKeyPressFcn = @(~,evt) app.safeCall(@() app.onKeyPress(evt));
            app.Fig.WindowButtonDownFcn = @(~,~) app.safeCall(@app.onMouseDown);
            app.Fig.WindowButtonMotionFcn = @(~,~) app.safeCall(@app.onMouseMove);
            app.Fig.WindowButtonUpFcn = @(~,~) app.safeCall(@app.onMouseUp);

            % Context menu for empty map area
            app.MapContextMenu = uicontextmenu(app.Fig);
            uimenu(app.MapContextMenu, 'Text', 'Add Panel Here', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.addPanelAtClick));
            uimenu(app.MapContextMenu, 'Text', 'Reset Zoom', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.resetZoom));
            uimenu(app.MapContextMenu, 'Text', 'Toggle Sun Path', ...
                'Separator', 'on', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.toggleSunArc));
            app.MapAxes.ContextMenu = app.MapContextMenu;

            % Context menu for panels (attached per-ROI)
            app.PanelContextMenu = uicontextmenu(app.Fig);
            uimenu(app.PanelContextMenu, 'Text', 'Delete Panel', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.deleteSelectedPanel));
            uimenu(app.PanelContextMenu, 'Text', 'Duplicate Panel', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.duplicateSelectedPanel));
            uimenu(app.PanelContextMenu, 'Text', 'Optimize This Panel', ...
                'Separator', 'on', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@app.optimizeSelectedTilt));
            uimenu(app.PanelContextMenu, 'Text', 'Face South (180°)', ...
                'Separator', 'on', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@() app.setPanelAzimuth(180)));
            uimenu(app.PanelContextMenu, 'Text', 'Face East (90°)', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@() app.setPanelAzimuth(90)));
            uimenu(app.PanelContextMenu, 'Text', 'Face West (270°)', ...
                'MenuSelectedFcn', @(~,~) app.safeCall(@() app.setPanelAzimuth(270)));

            % ── Sidebar ──
            sidebar = uigridlayout(mainGrid, [3 1], ...
                'RowHeight', {120, 'fit', '1x'}, ...
                'Padding', [8 8 8 8], 'RowSpacing', 10);
            sidebar.Layout.Row = 2;
            sidebar.Layout.Column = 2;
            sidebar.BackgroundColor = [0.12 0.12 0.14];

            % ── Section 1: Panel List ──
            panelSection = uipanel(sidebar, ...
                'Title', 'Panels', ...
                'FontWeight', 'bold', ...
                'BackgroundColor', [0.15 0.15 0.17]);
            panelGrid = uigridlayout(panelSection, [2 1], ...
                'RowHeight', {'1x', 30}, ...
                'Padding', 6, 'RowSpacing', 6);

            app.PanelListBox = uilistbox(panelGrid, ...
                'Items', {}, ...
                'FontColor', [0.85 0.85 0.85], ...
                'BackgroundColor', [0.1 0.1 0.12], ...
                'ValueChangedFcn', @(src,~) app.safeCall(@() app.onListSelection(src)));

            btnRow = uigridlayout(panelGrid, [1 2], ...
                'ColumnWidth', {'1x', '1x'}, 'Padding', 0, 'RowHeight', {28});
            app.DeletePanelButton = uibutton(btnRow, 'Text', 'Delete Selected', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.deleteSelectedPanel), ...
                'BackgroundColor', [0.5 0.15 0.15], 'FontColor', 'w', ...
                'Enable', 'off');
            uibutton(btnRow, 'Text', 'Clear All', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.clearPanels), ...
                'BackgroundColor', [0.3 0.3 0.3], 'FontColor', 'w');

            % ── Section 2: Tilt Control ──
            tiltSection = uipanel(sidebar, ...
                'Title', 'Tilt Control', ...
                'FontWeight', 'bold', ...
                'BackgroundColor', [0.15 0.15 0.17]);
            tiltInner = uigridlayout(tiltSection, [2 1], ...
                'RowHeight', {45, 28}, ...
                'Padding', [10 6 10 4], 'RowSpacing', 6);

            tiltSliderRow = uigridlayout(tiltInner, [1 2], ...
                'ColumnWidth', {'1x', 45}, 'Padding', [8 0 4 0], 'RowHeight', {40});
            app.TiltSlider = uislider(tiltSliderRow, ...
                'Limits', [0 90], 'Value', 30, ...
                'ValueChangedFcn', @(~,~) app.safeCall(@app.tiltChanged), ...
                'MajorTicks', 0:30:90);
            app.TiltLabel = uilabel(tiltSliderRow, 'Text', '30°', ...
                'FontColor', [0.4 0.8 1], 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'FontSize', 14);

            optRow = uigridlayout(tiltInner, [1 2], ...
                'ColumnWidth', {'1x', 85}, 'Padding', 0, 'RowHeight', {26});
            app.OptimizeDropdown = uidropdown(optRow, ...
                'Items', {'Max total yield', 'Annual consistency', 'Consistent daily'}, ...
                'Value', 'Max total yield');
            uibutton(optRow, 'Text', 'Optimize', ...
                'ButtonPushedFcn', @(~,~) app.safeCall(@app.optimizeTilt), ...
                'BackgroundColor', [0.4 0.25 0.6], 'FontColor', 'w');

            % ── Section 3: Energy Analysis (summary + charts) ──
            chartSection = uipanel(sidebar, ...
                'Title', 'Energy Analysis', ...
                'FontWeight', 'bold', ...
                'BackgroundColor', [0.15 0.15 0.17]);
            chartGrid = uigridlayout(chartSection, [4 1], ...
                'RowHeight', {22, '1x', '1x', 22}, ...
                'Padding', [6 4 6 4], 'RowSpacing', 6);

            app.TotalLabel = uilabel(chartGrid, 'Text', '', ...
                'FontColor', [0.95 0.75 0.1], 'FontWeight', 'bold', 'FontSize', 13);

            app.DailyAxes = uiaxes(chartGrid);
            title(app.DailyAxes, 'Daily Power (Summer Solstice)');
            xlabel(app.DailyAxes, 'Hour (UTC)');
            ylabel(app.DailyAxes, 'W');
            app.DailyAxes.Color = [0.08 0.08 0.1];
            app.DailyAxes.XColor = [0.6 0.6 0.6];
            app.DailyAxes.YColor = [0.6 0.6 0.6];
            app.DailyAxes.Title.Color = [0.85 0.85 0.85];
            app.DailyAxes.FontSize = 9;

            app.MonthlyAxes = uiaxes(chartGrid);
            title(app.MonthlyAxes, 'Monthly Energy');
            xlabel(app.MonthlyAxes, 'Month');
            ylabel(app.MonthlyAxes, 'kWh');
            app.MonthlyAxes.Color = [0.08 0.08 0.1];
            app.MonthlyAxes.XColor = [0.6 0.6 0.6];
            app.MonthlyAxes.YColor = [0.6 0.6 0.6];
            app.MonthlyAxes.Title.Color = [0.85 0.85 0.85];
            app.MonthlyAxes.FontSize = 9;

            app.RealisticCheckBox = uicheckbox(chartGrid, ...
                'Text', 'Include weather, temperature & system losses', ...
                'FontColor', [0.85 0.85 0.85], ...
                'Value', true, ...
                'ValueChangedFcn', @(~,~) app.safeCall(@app.updatePlots));
        end

        function safeCall(app, fn)
            try
                fn();
            catch ex
                uialert(app.Fig, ex.getReport('basic'), 'Error');
            end
        end

        function loadAddress(app)
            addr = app.AddressField.Value;
            [lat, lon] = geocodeAddress(addr);
            app.Latitude = lat;
            app.Longitude = lon;
            app.clearPanels();
            app.loadMap();
        end

        function loadMap(app)
            drawnow;

            [A, R] = readBasemapImage('satellite', ...
                [app.Latitude, app.Longitude], app.ZoomLevel, 2048);
            app.MapRef = R;

            cla(app.MapAxes);
            mapshow(app.MapAxes, A, R);
            axis(app.MapAxes, 'tight', 'off');
            app.MapAxes.DataAspectRatio = [1 1 1];

            % Assign context menu to the map image so right-click works
            imgChildren = findobj(app.MapAxes.Children, 'Type', 'image');
            if isempty(imgChildren)
                imgChildren = findobj(app.MapAxes.Children, '-regexp', 'Type', '.*');
            end
            for i = 1:numel(imgChildren)
                if isprop(imgChildren(i), 'ContextMenu')
                    imgChildren(i).ContextMenu = app.MapContextMenu;
                end
            end

            app.drawSunArc();
        end

        function drawSunArc(app)
            % Draw sundial-style dots in a fixed overlay, unaffected by zoom/pan.
            for k = 1:numel(app.SunArcHandles)
                if isvalid(app.SunArcHandles{k})
                    delete(app.SunArcHandles{k});
                end
            end
            app.SunArcHandles = {};

            % Center and radius in normalized [0,1] overlay coordinates
            oxc = 0.5;
            oyc = 0.5;
            radius = 0.43;

            % Compute average sun position and irradiance for each hour
            hoursUTC = 0:23;
            avgAz = zeros(size(hoursUTC));
            avgEl = zeros(size(hoursUTC));
            avgIrr = zeros(size(hoursUTC));
            nValid = zeros(size(hoursUTC));

            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(hoursUTC);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                above = elm > 0;
                avgAz = avgAz + azm .* above;
                avgEl = avgEl + elm .* above;
                avgIrr = avgIrr + max(sind(elm), 0);
                nValid = nValid + above;
            end

            hasData = nValid > 0;
            avgAz(hasData) = avgAz(hasData) ./ nValid(hasData);
            avgEl(hasData) = avgEl(hasData) ./ nValid(hasData);
            avgIrr(hasData) = avgIrr(hasData) ./ 12;

            maxIrr = max(avgIrr);
            if maxIrr == 0, return; end

            hold(app.SunArcAxes, 'on');
            for h = 1:numel(hoursUTC)
                if ~hasData(h) || avgEl(h) < 2, continue; end

                az = avgAz(h);
                % Azimuth: 0=N=up, 90=E=right, 180=S=down, 270=W=left
                x = oxc + radius * sind(az);
                y = oyc + radius * cosd(az);

                normIrr = avgIrr(h) / maxIrr;
                dotSize = 5 + 20 * normIrr;
                dotColor = [1, 0.7 + 0.2*normIrr, 0.1 + 0.3*(1-normIrr)];

                hDot = plot(app.SunArcAxes, x, y, 'o', ...
                    'MarkerSize', dotSize, ...
                    'MarkerFaceColor', dotColor, ...
                    'MarkerEdgeColor', 'none');
                app.SunArcHandles{end+1} = hDot;

                % Label every 3 local hours and the peak
                localHour = mod(hoursUTC(h) - 5, 24);
                if mod(localHour, 3) == 0 || normIrr > 0.9
                    if localHour == 0
                        lbl = '12am';
                    elseif localHour < 12
                        lbl = sprintf('%da', localHour);
                    elseif localHour == 12
                        lbl = '12p';
                    else
                        lbl = sprintf('%dp', localHour - 12);
                    end
                    hTxt = text(app.SunArcAxes, x, y - 0.03, lbl, ...
                        'Color', [0.9 0.8 0.3], 'FontSize', 8, ...
                        'HorizontalAlignment', 'center', ...
                        'FontWeight', 'bold');
                    app.SunArcHandles{end+1} = hTxt;
                end
            end
            hold(app.SunArcAxes, 'off');

            if ~app.SunArcVisible
                for k = 1:numel(app.SunArcHandles)
                    if isvalid(app.SunArcHandles{k})
                        app.SunArcHandles{k}.Visible = 'off';
                    end
                end
            end
        end

        function toggleSunArc(app)
            app.SunArcVisible = ~app.SunArcVisible;
            vis = 'off';
            if app.SunArcVisible, vis = 'on'; end
            for k = 1:numel(app.SunArcHandles)
                if isvalid(app.SunArcHandles{k})
                    app.SunArcHandles{k}.Visible = vis;
                end
            end
        end

        function onScroll(app, evt)
            % Only zoom when cursor is over the map
            cp = app.MapAxes.CurrentPoint;
            xl = app.MapAxes.XLim;
            yl = app.MapAxes.YLim;
            if cp(1,1) < xl(1) || cp(1,1) > xl(2) || cp(1,2) < yl(1) || cp(1,2) > yl(2)
                return;
            end

            factor = 1.3;
            if evt.VerticalScrollCount > 0
                s = factor; % scroll down = zoom out
            else
                s = 1/factor; % scroll up = zoom in
            end

            xl = app.MapAxes.XLim;
            yl = app.MapAxes.YLim;
            cx = mean(xl);
            cy = mean(yl);
            newW = diff(xl) * s;
            newH = diff(yl) * s;

            % Clamp: don't zoom in past ~30m view or out past loaded extent
            maxW = diff(app.MapRef.XWorldLimits);
            maxH = diff(app.MapRef.YWorldLimits);
            minView = 30;

            newW = max(min(newW, maxW), minView);
            newH = max(min(newH, maxH), minView);

            % Keep view within loaded image bounds
            xLo = max(cx - newW/2, app.MapRef.XWorldLimits(1));
            xHi = min(cx + newW/2, app.MapRef.XWorldLimits(2));
            yLo = max(cy - newH/2, app.MapRef.YWorldLimits(1));
            yHi = min(cy + newH/2, app.MapRef.YWorldLimits(2));

            app.MapAxes.XLim = [xLo xHi];
            app.MapAxes.YLim = [yLo yHi];
        end

        function onMouseDown(app)
            % Only left-click pans; right-click is reserved for context menus
            if ~strcmp(app.Fig.SelectionType, 'normal')
                return;
            end

            % Only pan if click landed on the map image (not on a panel ROI)
            obj = app.Fig.CurrentObject;
            if isempty(obj), return; end

            % If the clicked object is an ROI or child of an ROI, skip panning
            objClass = class(obj);
            if contains(objClass, 'images.roi') || contains(objClass, 'ROI')
                return;
            end
            parent = obj;
            for i = 1:5
                if isprop(parent, 'Parent') && ~isempty(parent.Parent)
                    parent = parent.Parent;
                    if contains(class(parent), 'images.roi')
                        return;
                    end
                else
                    break;
                end
            end

            % Must be within the map axes bounds
            cp = app.MapAxes.CurrentPoint;
            xl = app.MapAxes.XLim;
            yl = app.MapAxes.YLim;
            if cp(1,1) < xl(1) || cp(1,1) > xl(2) || cp(1,2) < yl(1) || cp(1,2) > yl(2)
                return;
            end

            app.PanStart = [cp(1,1), cp(1,2)];
            app.IsPanning = true;
        end

        function onMouseMove(app)
            if ~app.IsPanning || isempty(app.PanStart)
                return;
            end
            cp = app.MapAxes.CurrentPoint;
            dx = app.PanStart(1) - cp(1,1);
            dy = app.PanStart(2) - cp(1,2);

            xl = app.MapAxes.XLim + dx;
            yl = app.MapAxes.YLim + dy;

            % Clamp within loaded image
            mapXL = app.MapRef.XWorldLimits;
            mapYL = app.MapRef.YWorldLimits;
            if xl(1) < mapXL(1), xl = xl - (xl(1) - mapXL(1)); end
            if xl(2) > mapXL(2), xl = xl - (xl(2) - mapXL(2)); end
            if yl(1) < mapYL(1), yl = yl - (yl(1) - mapYL(1)); end
            if yl(2) > mapYL(2), yl = yl - (yl(2) - mapYL(2)); end

            app.MapAxes.XLim = xl;
            app.MapAxes.YLim = yl;
        end

        function onMouseUp(app)
            app.IsPanning = false;
            app.PanStart = [];
        end

        function tiltChanged(app)
            tilt = app.TiltSlider.Value;
            app.TiltLabel.Text = sprintf('%.0f°', tilt);
            if app.SelectedPanel > 0 && app.SelectedPanel <= numel(app.PanelTilts)
                app.PanelTilts(app.SelectedPanel) = tilt;
            end
            app.updatePlots();
        end

        function selectPanel(app, idx)
            app.SelectedPanel = idx;
            if idx > 0 && idx <= numel(app.PanelTilts)
                app.TiltSlider.Value = app.PanelTilts(idx);
                app.TiltLabel.Text = sprintf('%.0f°', app.PanelTilts(idx));
                app.DeletePanelButton.Enable = 'on';
            else
                app.DeletePanelButton.Enable = 'off';
            end
            app.refreshPanelList();
            app.updatePlots();
        end

        function onListSelection(app, src)
            if isempty(src.Value)
                return;
            end
            items = src.Items;
            idx = find(strcmp(items, src.Value), 1);
            if ~isempty(idx)
                app.selectPanel(idx);
            end
        end

        function deleteSelectedPanel(app)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels)
                return;
            end
            if isvalid(app.Panels{idx}), delete(app.Panels{idx}); end
            if idx <= numel(app.SunEdges) && isvalid(app.SunEdges{idx})
                delete(app.SunEdges{idx});
            end
            app.Panels(idx) = [];
            app.PanelTilts(idx) = [];
            app.SunEdges(idx) = [];
            if app.SelectedPanel > numel(app.Panels)
                app.SelectedPanel = numel(app.Panels);
            end
            if app.SelectedPanel > 0
                app.selectPanel(app.SelectedPanel);
            else
                app.DeletePanelButton.Enable = 'off';
            end
            app.refreshPanelList();
            app.updatePlots();
        end

        function refreshPanelList(app)
            items = cell(1, numel(app.Panels));
            for k = 1:numel(app.Panels)
                if isvalid(app.Panels{k})
                    [~, az] = app.panelGeometry(app.Panels{k});
                    items{k} = sprintf('Panel %d  |  Az: %.0f°  |  Tilt: %.0f°', ...
                        k, az, app.PanelTilts(k));
                else
                    items{k} = sprintf('Panel %d  (invalid)', k);
                end
            end
            app.PanelListBox.Items = items;
            if app.SelectedPanel > 0 && app.SelectedPanel <= numel(items)
                app.PanelListBox.Value = items{app.SelectedPanel};
            end
        end

        function addPanel(app)
            if isempty(app.MapRef)
                return;
            end

            n = numel(app.Panels) + 1;

            % Place panel in the center of the current view
            xl = app.MapAxes.XLim;
            yl = app.MapAxes.YLim;
            cx = mean(xl);
            cy = mean(yl);

            % Size panel to ~8% of visible extent so it's easy to grab
            viewW = diff(xl);
            viewH = diff(yl);
            pw = viewW * 0.08;
            ph = viewH * 0.05;

            % Offset each panel slightly
            offset = (n - 1) * pw * 0.4;

            roi = drawrectangle(app.MapAxes, ...
                'Position', [cx - pw/2 + offset, cy - ph/2 + offset, pw, ph], ...
                'Color', [0 0.9 1], ...
                'FaceAlpha', 0.3, ...
                'LineWidth', 2, ...
                'Rotatable', true, ...
                'Label', sprintf('Panel %d', n), ...
                'LabelVisible', 'hover');

            panelIdx = n;
            addlistener(roi, 'ROIMoved', @(~,~) app.safeCall(@() app.onPanelMoved(panelIdx)));
            addlistener(roi, 'MovingROI', @(~,~) app.safeCall(@() app.onPanelMoving(panelIdx)));
            addlistener(roi, 'ROIClicked', @(~,~) app.safeCall(@() app.selectPanel(panelIdx)));
            roi.ContextMenu = app.PanelContextMenu;
            app.Panels{n} = roi;
            app.PanelTilts(n) = 30;

            % Create sun-facing edge highlight (will be updated in updatePlots)
            hold(app.MapAxes, 'on');
            app.SunEdges{n} = plot(app.MapAxes, [0 0], [0 0], '-', ...
                'Color', [1 0.8 0], 'LineWidth', 5);
            hold(app.MapAxes, 'off');

            app.selectPanel(n);
            app.refreshPanelList();
            app.updatePlots();
        end

        function onPanelMoving(app, idx)
            if idx > 0 && idx <= numel(app.Panels) && isvalid(app.Panels{idx})
                app.updateSunEdge(idx, app.Panels{idx});
                app.refreshPanelList();
                app.updatePlots();
            end
        end

        function onPanelMoved(app, idx)
            app.selectPanel(idx);
        end

        function clearPanels(app)
            for k = 1:numel(app.Panels)
                if isvalid(app.Panels{k}), delete(app.Panels{k}); end
            end
            for k = 1:numel(app.SunEdges)
                if isvalid(app.SunEdges{k}), delete(app.SunEdges{k}); end
            end
            app.Panels = {};
            app.PanelTilts = [];
            app.SunEdges = {};
            app.SelectedPanel = 0;
            app.DeletePanelButton.Enable = 'off';
            app.PanelListBox.Items = {};
            cla(app.DailyAxes);
            cla(app.MonthlyAxes);
            app.TotalLabel.Text = '';
        end

        function updatePlots(app)
            % Remove invalid panels and edge highlights
            valid = cellfun(@isvalid, app.Panels);
            for k = find(~valid)
                if k <= numel(app.SunEdges) && isvalid(app.SunEdges{k})
                    delete(app.SunEdges{k});
                end
            end
            app.Panels = app.Panels(valid);
            app.PanelTilts = app.PanelTilts(valid);
            app.SunEdges = app.SunEdges(valid);
            app.refreshPanelList();

            if isempty(app.Panels)
                cla(app.DailyAxes);
                cla(app.MonthlyAxes);
                app.TotalLabel.Text = '';
                return;
            end

            efficiency = 0.20;
            t = datetime(2024, 6, 21, 'TimeZone', 'UTC') + hours(0:23);
            [az, el] = sunPosition(app.Latitude, app.Longitude, t);
            nv = app.lossArgs(t);

            cla(app.DailyAxes);
            hold(app.DailyAxes, 'on');
            totalWatts = zeros(size(t));
            colors = lines(numel(app.Panels));

            for k = 1:numel(app.Panels)
                roi = app.Panels{k};
                [area, panelAz] = app.panelGeometry(roi);
                tilt = app.PanelTilts(k);

                % Update sun-facing edge highlight on map
                app.updateSunEdge(k, roi);

                w = solarPanelPower(efficiency, area, az, el, tilt, panelAz, nv{:});
                totalWatts = totalWatts + w;
                plot(app.DailyAxes, 0:23, w, 'Color', colors(k,:), ...
                    'LineWidth', 1.5, 'DisplayName', sprintf('P%d az=%.0f° t=%.0f°', k, panelAz, tilt));
            end

            if numel(app.Panels) > 1
                plot(app.DailyAxes, 0:23, totalWatts, '--w', ...
                    'LineWidth', 2, 'DisplayName', 'Total');
            end
            hold(app.DailyAxes, 'off');
            xlim(app.DailyAxes, [0 23]);
            legend(app.DailyAxes, 'TextColor', [0.8 0.8 0.8], 'FontSize', 8, ...
                'Location', 'northwest');

            % Monthly (per-panel tracking for coloring)
            N = numel(app.Panels);
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            monthlyTotal = zeros(1, 12);
            panelAnnualKWh = zeros(1, N);
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                for k = 1:N
                    [area, panelAz] = app.panelGeometry(app.Panels{k});
                    tilt = app.PanelTilts(k);
                    w = solarPanelPower(efficiency, area, azm, elm, tilt, panelAz, nvM{:});
                    kwh = sum(w)/1000 * daysPerMonth(m);
                    monthlyTotal(m) = monthlyTotal(m) + kwh;
                    panelAnnualKWh(k) = panelAnnualKWh(k) + kwh;
                end
            end

            cla(app.MonthlyAxes);
            bar(app.MonthlyAxes, 1:12, monthlyTotal, ...
                'FaceColor', [0.9 0.5 0.1], 'EdgeColor', 'none');
            app.MonthlyAxes.XTick = 1:12;
            app.MonthlyAxes.XTickLabel = {'J','F','M','A','M','J','J','A','S','O','N','D'};
            xlim(app.MonthlyAxes, [0.5 12.5]);

            % Color panels by absolute efficiency vs theoretical max for this latitude
            % Theoretical max: 1 m² panel at optimal tilt (=latitude) facing south
            optTilt = abs(app.Latitude);
            idealKWhPerM2 = 0;
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                w = solarPanelPower(efficiency, 1, azm, elm, optTilt, 180, nvM{:});
                idealKWhPerM2 = idealKWhPerM2 + sum(w)/1000 * daysPerMonth(m);
            end

            panelAreas = zeros(1, N);
            for k = 1:N
                [panelAreas(k), ~] = app.panelGeometry(app.Panels{k});
            end
            kwhPerM2 = panelAnnualKWh ./ max(panelAreas, 0.01);

            for k = 1:N
                roi = app.Panels{k};
                if ~isvalid(roi), continue; end

                if k == app.SelectedPanel
                    roi.Color = [0 0.9 1];
                else
                    % 0 = no output, 1 = theoretical max
                    t_norm = min(kwhPerM2(k) / max(idealKWhPerM2, 0.01), 1);
                    % Dim orange (low) to bright orange (high)
                    c = [0.9, 0.3 + 0.5*t_norm, 0.1*(1-t_norm)];
                    roi.Color = c;
                end

                if panelAnnualKWh(k) >= 1000
                    roi.Label = sprintf('%.1f MWh', panelAnnualKWh(k)/1000);
                else
                    roi.Label = sprintf('%.0f kWh', panelAnnualKWh(k));
                end
                roi.LabelVisible = 'on';
            end

            annualKWh = sum(panelAnnualKWh);
            totalArea = 0;
            for k = 1:N
                [a, ~] = app.panelGeometry(app.Panels{k});
                totalArea = totalArea + a;
            end

            if app.RealisticCheckBox.Value
                % Compute clear-sky total to derive effective loss %
                clearSkyKWh = 0;
                for m = 1:12
                    tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                    [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                    for k = 1:N
                        [area, panelAz] = app.panelGeometry(app.Panels{k});
                        tilt = app.PanelTilts(k);
                        w = solarPanelPower(efficiency, area, azm, elm, tilt, panelAz, ...
                            'ClearSkyFraction', 1, 'SystemLoss', 0);
                        clearSkyKWh = clearSkyKWh + sum(w)/1000 * daysPerMonth(m);
                    end
                end
                lossPct = (1 - annualKWh / clearSkyKWh) * 100;
                app.TotalLabel.Text = sprintf('%.1f kWh/day | %d panels | %.0f m² (%.0f%% losses)', ...
                    annualKWh / 365, N, totalArea, lossPct);
            else
                app.TotalLabel.Text = sprintf('%.1f kWh/day | %d panels | %.0f m² (no losses)', ...
                    annualKWh / 365, N, totalArea);
            end
        end

        function optimizeTilt(app)
            if isempty(app.Panels)
                uialert(app.Fig, 'Add panels first.', 'No Panels');
                return;
            end

            N = numel(app.Panels);
            areas = zeros(1, N);
            panelAzs = zeros(1, N);
            for k = 1:N
                [areas(k), panelAzs(k)] = app.panelGeometry(app.Panels{k});
            end

            objective = app.OptimizeDropdown.Value;
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            efficiency = 0.20;

            if strcmp(objective, 'Max total yield')
                fun = @(tilts) -app.jointAnnualEnergy(tilts, areas, panelAzs, daysPerMonth, efficiency);
            elseif strcmp(objective, 'Annual consistency')
                fun = @(tilts) -app.jointMinMonthEnergy(tilts, areas, panelAzs, daysPerMonth, efficiency);
            else
                fun = @(tilts) app.jointDailyVariance(tilts, areas, panelAzs, efficiency);
            end

            x0 = app.PanelTilts(:)';
            lb = zeros(1, N);
            ub = 90 * ones(1, N);

            opts = optimset('Display', 'off', 'TolX', 0.5);
            if N == 1
                optTilts = fminbnd(@(t) fun(t), 0, 90);
            else
                % Bounded Nelder-Mead via penalty
                penaltyFun = @(x) fun(min(max(x, lb), ub)) + ...
                    1e6 * sum(max(0, lb - x).^2 + max(0, x - ub).^2);
                optTilts = fminsearch(penaltyFun, x0, opts);
                optTilts = min(max(optTilts, 0), 90);
            end

            app.PanelTilts = optTilts;
            if app.SelectedPanel > 0 && app.SelectedPanel <= N
                app.TiltSlider.Value = optTilts(app.SelectedPanel);
                app.TiltLabel.Text = sprintf('%.0f°', optTilts(app.SelectedPanel));
            end
            app.updatePlots();
        end

        function energy = jointAnnualEnergy(app, tilts, areas, panelAzs, daysPerMonth, efficiency)
            energy = 0;
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                for k = 1:numel(tilts)
                    w = solarPanelPower(efficiency, areas(k), azm, elm, tilts(k), panelAzs(k), nvM{:});
                    energy = energy + sum(w)/1000 * daysPerMonth(m);
                end
            end
        end

        function minE = jointMinMonthEnergy(app, tilts, areas, panelAzs, daysPerMonth, efficiency)
            monthly = zeros(1, 12);
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                for k = 1:numel(tilts)
                    w = solarPanelPower(efficiency, areas(k), azm, elm, tilts(k), panelAzs(k), nvM{:});
                    monthly(m) = monthly(m) + sum(w)/1000 * daysPerMonth(m);
                end
            end
            minE = min(monthly);
        end

        function v = jointDailyVariance(app, tilts, areas, panelAzs, efficiency)
            % Minimize variance of combined hourly power on summer solstice
            t = datetime(2024, 6, 21, 'TimeZone', 'UTC') + hours(0:23);
            [az, el] = sunPosition(app.Latitude, app.Longitude, t);
            nv = app.lossArgs(t);
            totalW = zeros(size(t));
            for k = 1:numel(tilts)
                totalW = totalW + solarPanelPower(efficiency, areas(k), az, el, tilts(k), panelAzs(k), nv{:});
            end
            % Only penalize variance during daylight hours
            daylight = totalW > 0;
            if any(daylight)
                v = var(totalW(daylight));
            else
                v = 0;
            end
        end

        function updateSunEdge(app, k, roi)
            % Highlight the sun-facing edge (bottom edge of the rectangle).
            % This rotates naturally with the panel so there's no
            % directional confusion. Vertices order: BL, TL, TR, BR.
            verts = roi.Vertices;
            if k <= numel(app.SunEdges) && isvalid(app.SunEdges{k})
                set(app.SunEdges{k}, ...
                    'XData', [verts(1,1), verts(4,1)], ...
                    'YData', [verts(1,2), verts(4,2)]);
            end
        end

        function onKeyPress(app, evt)
            % Skip hotkeys when typing in the address field
            if isequal(app.Fig.CurrentObject, app.AddressField)
                return;
            end

            switch evt.Key
                case {'delete', 'backspace'}
                    app.deleteSelectedPanel();
                case 'escape'
                    app.deselectPanel();
                case 'a'
                    app.addPanel();
                case 'o'
                    app.optimizeTilt();
                case 'tab'
                    app.cyclePanel(evt);
                case {'leftarrow', 'rightarrow', 'uparrow', 'downarrow'}
                    app.nudgePanel(evt);
                case 'bracketleft'
                    app.rotateSelectedPanel(-5);
                case 'bracketright'
                    app.rotateSelectedPanel(5);
                case 'pageup'
                    app.adjustTilt(5);
                case 'pagedown'
                    app.adjustTilt(-5);
            end
        end

        function deselectPanel(app)
            app.SelectedPanel = 0;
            app.DeletePanelButton.Enable = 'off';
            app.refreshPanelList();
            app.updatePlots();
        end

        function cyclePanel(app, evt)
            if isempty(app.Panels), return; end
            if any(strcmp(evt.Modifier, 'shift'))
                next = app.SelectedPanel - 1;
                if next < 1, next = numel(app.Panels); end
            else
                next = app.SelectedPanel + 1;
                if next > numel(app.Panels), next = 1; end
            end
            app.selectPanel(next);
        end

        function nudgePanel(app, evt)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels), return; end
            roi = app.Panels{idx};
            if ~isvalid(roi), return; end

            % Nudge by 1% of current view
            step = diff(app.MapAxes.XLim) * 0.01;
            pos = roi.Position;
            switch evt.Key
                case 'leftarrow',  pos(1) = pos(1) - step;
                case 'rightarrow', pos(1) = pos(1) + step;
                case 'uparrow',    pos(2) = pos(2) + step;
                case 'downarrow',  pos(2) = pos(2) - step;
            end
            roi.Position = pos;
            app.updatePlots();
        end

        function rotateSelectedPanel(app, deg)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels), return; end
            roi = app.Panels{idx};
            if ~isvalid(roi), return; end
            roi.RotationAngle = roi.RotationAngle + deg;
            app.updatePlots();
        end

        function adjustTilt(app, delta)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.PanelTilts), return; end
            newTilt = max(0, min(90, app.PanelTilts(idx) + delta));
            app.PanelTilts(idx) = newTilt;
            app.TiltSlider.Value = newTilt;
            app.TiltLabel.Text = sprintf('%.0f°', newTilt);
            app.updatePlots();
        end

        function resetZoom(app)
            if isempty(app.MapRef), return; end
            app.MapAxes.XLim = app.MapRef.XWorldLimits;
            app.MapAxes.YLim = app.MapRef.YWorldLimits;
        end

        function addPanelAtClick(app)
            % Add panel at the last right-click location
            cp = app.MapAxes.CurrentPoint;
            cx = cp(1,1);
            cy = cp(1,2);

            n = numel(app.Panels) + 1;
            viewW = diff(app.MapAxes.XLim);
            viewH = diff(app.MapAxes.YLim);
            pw = viewW * 0.08;
            ph = viewH * 0.05;

            roi = drawrectangle(app.MapAxes, ...
                'Position', [cx - pw/2, cy - ph/2, pw, ph], ...
                'Color', [0 0.9 1], ...
                'FaceAlpha', 0.3, ...
                'LineWidth', 2, ...
                'Rotatable', true, ...
                'Label', sprintf('Panel %d', n), ...
                'LabelVisible', 'hover');

            panelIdx = n;
            addlistener(roi, 'ROIMoved', @(~,~) app.safeCall(@() app.onPanelMoved(panelIdx)));
            addlistener(roi, 'MovingROI', @(~,~) app.safeCall(@() app.onPanelMoving(panelIdx)));
            addlistener(roi, 'ROIClicked', @(~,~) app.safeCall(@() app.selectPanel(panelIdx)));
            roi.ContextMenu = app.PanelContextMenu;
            app.Panels{n} = roi;
            app.PanelTilts(n) = 30;

            hold(app.MapAxes, 'on');
            app.SunEdges{n} = plot(app.MapAxes, [0 0], [0 0], '-', ...
                'Color', [1 0.8 0], 'LineWidth', 5);
            hold(app.MapAxes, 'off');

            app.selectPanel(n);
            app.refreshPanelList();
            app.updatePlots();
        end

        function duplicateSelectedPanel(app)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels), return; end
            roi = app.Panels{idx};
            if ~isvalid(roi), return; end

            n = numel(app.Panels) + 1;
            pos = roi.Position;
            offset = pos(3) * 0.5;
            newPos = [pos(1) + offset, pos(2) + offset, pos(3), pos(4)];

            newRoi = drawrectangle(app.MapAxes, ...
                'Position', newPos, ...
                'Color', [0 0.9 1], ...
                'FaceAlpha', 0.3, ...
                'LineWidth', 2, ...
                'Rotatable', true, ...
                'RotationAngle', roi.RotationAngle, ...
                'Label', sprintf('Panel %d', n), ...
                'LabelVisible', 'hover');

            panelIdx = n;
            addlistener(newRoi, 'ROIMoved', @(~,~) app.safeCall(@() app.onPanelMoved(panelIdx)));
            addlistener(newRoi, 'MovingROI', @(~,~) app.safeCall(@() app.onPanelMoving(panelIdx)));
            addlistener(newRoi, 'ROIClicked', @(~,~) app.safeCall(@() app.selectPanel(panelIdx)));
            newRoi.ContextMenu = app.PanelContextMenu;
            app.Panels{n} = newRoi;
            app.PanelTilts(n) = app.PanelTilts(idx);

            hold(app.MapAxes, 'on');
            app.SunEdges{n} = plot(app.MapAxes, [0 0], [0 0], '-', ...
                'Color', [1 0.8 0], 'LineWidth', 5);
            hold(app.MapAxes, 'off');

            app.selectPanel(n);
            app.refreshPanelList();
            app.updatePlots();
        end

        function optimizeSelectedTilt(app)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels), return; end

            [area, panelAz] = app.panelGeometry(app.Panels{idx});
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            efficiency = 0.20;

            fun = @(t) -app.singlePanelAnnualEnergy(t, area, panelAz, daysPerMonth, efficiency);
            optTilt = fminbnd(fun, 0, 90);

            app.PanelTilts(idx) = optTilt;
            app.TiltSlider.Value = optTilt;
            app.TiltLabel.Text = sprintf('%.0f°', optTilt);
            app.updatePlots();
        end

        function energy = singlePanelAnnualEnergy(app, tilt, area, panelAz, daysPerMonth, efficiency)
            energy = 0;
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                w = solarPanelPower(efficiency, area, azm, elm, tilt, panelAz, nvM{:});
                energy = energy + sum(w)/1000 * daysPerMonth(m);
            end
        end

        function nvArgs = lossArgs(app, times)
            if app.RealisticCheckBox.Value
                Kt = clearnessIndex(app.Latitude, app.Longitude);
                Tamb = ambientTemperature(app.Latitude, times);
                nvArgs = {'ClearSkyFraction', Kt, 'AmbientTemp', Tamb};
            else
                nvArgs = {'ClearSkyFraction', 1, 'SystemLoss', 0};
            end
        end

        function exportReport(app)
            if isempty(app.Panels)
                uialert(app.Fig, 'Add at least one panel before exporting.', 'No Panels');
                return;
            end

            if ~exist('mlreportgen.dom.Document', 'class')
                uialert(app.Fig, 'MATLAB Report Generator toolbox is required.', 'Missing Toolbox');
                return;
            end

            [file, path] = uiputfile('*.pdf', 'Save Solar Report', ...
                fullfile(getenv('USERPROFILE'), 'Desktop', 'SolarReport.pdf'));
            if isequal(file, 0)
                return;
            end
            outputPath = fullfile(path, file);

            d = uiprogressdlg(app.Fig, 'Title', 'Generating Report', ...
                'Message', 'Preparing...', 'Indeterminate', 'on');

            try
                tmpDir = tempdir;

                d.Message = 'Capturing map...';
                mapImg = app.captureMap(tmpDir);

                d.Message = 'Generating charts...';
                [dailyImg, monthlyImg, seasonalImg, sunPathImg, optImg] = app.generateChartImages(tmpDir);

                d.Message = 'Building PDF...';
                app.buildPdfDocument(outputPath, mapImg, dailyImg, monthlyImg, seasonalImg, sunPathImg, optImg);

                delete(mapImg);
                delete(dailyImg);
                delete(monthlyImg);
                delete(seasonalImg);
                delete(sunPathImg);
                delete(optImg);

                close(d);
                rptview(outputPath);
            catch ex
                close(d);
                uialert(app.Fig, ex.message, 'Export Failed');
            end
        end

        function imgPath = captureMap(app, tmpDir)
            imgPath = fullfile(tmpDir, 'solar_map.png');
            exportgraphics(app.MapAxes, imgPath, 'Resolution', 200);
        end

        function [dailyImg, monthlyImg, seasonalImg, sunPathImg, optImg] = generateChartImages(app, tmpDir)
            efficiency = 0.20;
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            N = numel(app.Panels);

            areas = zeros(1, N);
            panelAzs = zeros(1, N);
            for k = 1:N
                [areas(k), panelAzs(k)] = app.panelGeometry(app.Panels{k});
            end

            % Daily power profile (summer solstice)
            fig1 = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 6.5 3]);
            t = datetime(2024, 6, 21, 'TimeZone', 'UTC') + hours(0:23);
            [az, el] = sunPosition(app.Latitude, app.Longitude, t);
            nv = app.lossArgs(t);
            totalW = zeros(1, 24);
            hold on;
            colors = lines(N);
            for k = 1:N
                w = solarPanelPower(efficiency, areas(k), az, el, ...
                    app.PanelTilts(k), panelAzs(k), nv{:});
                totalW = totalW + w;
                plot(0:23, w, 'Color', colors(k,:), 'LineWidth', 1.8, ...
                    'DisplayName', sprintf('Panel %d (Az=%.0f°, Tilt=%.0f°)', k, panelAzs(k), app.PanelTilts(k)));
            end
            if N > 1
                plot(0:23, totalW, '--k', 'LineWidth', 2.5, 'DisplayName', 'Total');
            end
            hold off;
            xlabel('Hour (UTC)'); ylabel('Power (W)');
            title('Daily Power Profile — Summer Solstice (June 21)');
            legend('Location', 'northwest');
            grid on; xlim([0 23]);
            dailyImg = fullfile(tmpDir, 'solar_daily.png');
            exportgraphics(fig1, dailyImg, 'Resolution', 200);
            close(fig1);

            % Monthly energy bar chart
            fig2 = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 6.5 3]);
            monthlyTotal = zeros(1, 12);
            for m = 1:12
                tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                nvM = app.lossArgs(tm);
                for k = 1:N
                    w = solarPanelPower(efficiency, areas(k), azm, elm, ...
                        app.PanelTilts(k), panelAzs(k), nvM{:});
                    monthlyTotal(m) = monthlyTotal(m) + sum(w)/1000 * daysPerMonth(m);
                end
            end
            bar(1:12, monthlyTotal, 'FaceColor', [0.9 0.5 0.1], 'EdgeColor', 'none');
            set(gca, 'XTick', 1:12, 'XTickLabel', ...
                {'Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'});
            xlabel('Month'); ylabel('Energy (kWh)');
            title('Monthly Energy Production');
            grid on; xlim([0.5 12.5]);
            monthlyImg = fullfile(tmpDir, 'solar_monthly.png');
            exportgraphics(fig2, monthlyImg, 'Resolution', 200);
            close(fig2);

            % Seasonal comparison
            fig3 = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 6.5 3]);
            seasonDates = [datetime(2024,3,20), datetime(2024,6,21), ...
                           datetime(2024,9,22), datetime(2024,12,21)];
            seasonNames = {'Spring Equinox', 'Summer Solstice', ...
                           'Autumn Equinox', 'Winter Solstice'};
            seasonColors = [0.2 0.8 0.2; 1 0.6 0; 0.8 0.2 0.2; 0.2 0.5 1];
            hold on;
            for s = 1:4
                ts = seasonDates(s);
                ts.TimeZone = 'UTC';
                tHours = ts + hours(0:23);
                [azs, els] = sunPosition(app.Latitude, app.Longitude, tHours);
                nvS = app.lossArgs(tHours);
                totalSeason = zeros(1, 24);
                for k = 1:N
                    totalSeason = totalSeason + solarPanelPower(efficiency, areas(k), ...
                        azs, els, app.PanelTilts(k), panelAzs(k), nvS{:});
                end
                plot(0:23, totalSeason, 'Color', seasonColors(s,:), ...
                    'LineWidth', 2, 'DisplayName', seasonNames{s});
            end
            hold off;
            xlabel('Hour (UTC)'); ylabel('Power (W)');
            title('Seasonal Daily Power Comparison');
            legend('Location', 'northwest');
            grid on; xlim([0 23]);
            seasonalImg = fullfile(tmpDir, 'solar_seasonal.png');
            exportgraphics(fig3, seasonalImg, 'Resolution', 200);
            close(fig3);

            % Sun path polar diagram (summer + winter)
            fig4 = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 5 5]);
            tSummer = datetime(2024, 6, 21, 'TimeZone', 'UTC') + minutes(0:10:1439);
            tWinter = datetime(2024, 12, 21, 'TimeZone', 'UTC') + minutes(0:10:1439);
            [azSu, elSu] = sunPosition(app.Latitude, app.Longitude, tSummer);
            [azWi, elWi] = sunPosition(app.Latitude, app.Longitude, tWinter);
            daySu = elSu > 0;
            dayWi = elWi > 0;
            polarplot(deg2rad(azSu(daySu)), 90 - elSu(daySu), 'Color', [0.9 0.5 0], 'LineWidth', 2);
            hold on;
            polarplot(deg2rad(azWi(dayWi)), 90 - elWi(dayWi), 'Color', [0.2 0.4 0.9], 'LineWidth', 2);
            % Show panel azimuth directions
            for k = 1:N
                polarplot(deg2rad(panelAzs(k)) * [1 1], [0 30], '--', ...
                    'Color', colors(k,:), 'LineWidth', 1.5);
            end
            hold off;
            ax = gca;
            ax.ThetaZeroLocation = 'top';
            ax.ThetaDir = 'clockwise';
            ax.RLim = [0 90];
            ax.RTickLabel = {'', '60°', '30°', '0°'};
            ax.RDir = 'normal';
            legend({'Summer solstice', 'Winter solstice', 'Panel direction'}, ...
                'Location', 'southoutside');
            title('Sun Path Diagram');
            sunPathImg = fullfile(tmpDir, 'solar_sunpath.png');
            exportgraphics(fig4, sunPathImg, 'Resolution', 200);
            close(fig4);

            % Optimization comparison: current vs optimal tilt
            fig5 = figure('Visible', 'off', 'Units', 'inches', 'Position', [0 0 6.5 3]);
            tilts = 0:5:90;
            energyByTilt = zeros(size(tilts));
            for ti = 1:numel(tilts)
                for m = 1:12
                    tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                    [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                    nvM = app.lossArgs(tm);
                    for k = 1:N
                        w = solarPanelPower(efficiency, areas(k), azm, elm, ...
                            tilts(ti), panelAzs(k), nvM{:});
                        energyByTilt(ti) = energyByTilt(ti) + sum(w)/1000 * daysPerMonth(m);
                    end
                end
            end
            plot(tilts, energyByTilt, 'b-', 'LineWidth', 2);
            hold on;
            % Mark current tilts
            for k = 1:N
                currentEnergy = 0;
                for m = 1:12
                    tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                    [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                    nvM = app.lossArgs(tm);
                    w = solarPanelPower(efficiency, areas(k), azm, elm, ...
                        app.PanelTilts(k), panelAzs(k), nvM{:});
                    currentEnergy = currentEnergy + sum(w)/1000 * daysPerMonth(m);
                end
                plot(app.PanelTilts(k), currentEnergy, 'o', 'Color', colors(k,:), ...
                    'MarkerSize', 10, 'LineWidth', 2, 'MarkerFaceColor', colors(k,:));
            end
            [~, optIdx] = max(energyByTilt);
            xline(tilts(optIdx), '--r', sprintf('Optimal: %d°', tilts(optIdx)), ...
                'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom');
            hold off;
            xlabel('Tilt Angle (°)'); ylabel('Annual Energy (kWh)');
            title('Energy vs Tilt Angle');
            grid on; xlim([0 90]);
            optImg = fullfile(tmpDir, 'solar_optimization.png');
            exportgraphics(fig5, optImg, 'Resolution', 200);
            close(fig5);
        end

        function buildPdfDocument(app, outputPath, mapImg, dailyImg, monthlyImg, seasonalImg, sunPathImg, optImg)
            import mlreportgen.dom.*;

            doc = Document(outputPath, 'pdf');
            open(doc);

            % Title
            h1 = Heading1('Solar Panel Analysis Report');
            h1.Style = {Color('#1a3a6a'), FontSize('20pt')};
            append(doc, h1);

            append(doc, Paragraph(sprintf('Address: %s', app.AddressField.Value)));
            append(doc, Paragraph(sprintf('Location: %.4f°N, %.4f°%s', ...
                abs(app.Latitude), abs(app.Longitude), ...
                char('E'*(app.Longitude>=0) + 'W'*(app.Longitude<0)))));
            append(doc, Paragraph(sprintf('Generated: %s', char(datetime('now')))));
            append(doc, PageBreak());

            % Map
            h2 = Heading2('Panel Layout');
            append(doc, h2);
            img = Image(mapImg);
            img.Width = '6.5in';
            img.Height = '6.5in';
            append(doc, img);
            append(doc, PageBreak());

            % System summary
            append(doc, Heading2('System Summary'));
            [summaryData, annualKWh, clearSkyKWh] = app.computeSummaryData();
            append(doc, app.createFormattedTable(summaryData));

            % Per-panel table
            append(doc, Heading2('Panel Configuration'));
            panelData = app.computePanelTableData();
            append(doc, app.createFormattedTable(panelData));
            append(doc, PageBreak());

            % Charts
            append(doc, Heading2('Daily Power Profile (Summer Solstice)'));
            img2 = Image(dailyImg);
            img2.Width = '6.5in';
            img2.Height = '3in';
            append(doc, img2);

            append(doc, Heading2('Monthly Energy Production'));
            img3 = Image(monthlyImg);
            img3.Width = '6.5in';
            img3.Height = '3in';
            append(doc, img3);
            append(doc, PageBreak());

            append(doc, Heading2('Seasonal Comparison'));
            img4 = Image(seasonalImg);
            img4.Width = '6.5in';
            img4.Height = '3in';
            append(doc, img4);
            append(doc, PageBreak());

            % Sun path diagram
            append(doc, Heading2('Sun Path Diagram'));
            append(doc, Paragraph('Solar trajectory for summer and winter solstice. Dashed lines show panel facing directions.'));
            img5 = Image(sunPathImg);
            img5.Width = '4.5in';
            img5.Height = '4.5in';
            append(doc, img5);

            % Optimization
            append(doc, Heading2('Tilt Optimization'));
            append(doc, Paragraph('Annual energy yield as a function of tilt angle. Dots show current panel tilts.'));
            img6 = Image(optImg);
            img6.Width = '6.5in';
            img6.Height = '3in';
            append(doc, img6);
            append(doc, PageBreak());

            % Loss analysis
            if app.RealisticCheckBox.Value && clearSkyKWh > 0
                append(doc, Heading2('Loss Analysis'));
                lossPct = (1 - annualKWh / clearSkyKWh) * 100;
                Kt = clearnessIndex(app.Latitude, app.Longitude);
                lossData = {
                    'Parameter', 'Value';
                    'Clear-sky annual yield', sprintf('%.0f kWh', clearSkyKWh);
                    'Realistic annual yield', sprintf('%.0f kWh', annualKWh);
                    'Total effective losses', sprintf('%.1f%%', lossPct);
                    'Clearness index (Kt)', sprintf('%.2f', Kt);
                    'Temperature derating', 'Included'
                };
                append(doc, app.createFormattedTable(lossData));
            end

            close(doc);
        end

        function [data, annualKWh, clearSkyKWh] = computeSummaryData(app)
            efficiency = 0.20;
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            N = numel(app.Panels);

            totalArea = 0;
            annualKWh = 0;
            clearSkyKWh = 0;

            for k = 1:N
                [area, panelAz] = app.panelGeometry(app.Panels{k});
                totalArea = totalArea + area;
                for m = 1:12
                    tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                    [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                    nvM = app.lossArgs(tm);
                    w = solarPanelPower(efficiency, area, azm, elm, app.PanelTilts(k), panelAz, nvM{:});
                    annualKWh = annualKWh + sum(w)/1000 * daysPerMonth(m);

                    wClear = solarPanelPower(efficiency, area, azm, elm, app.PanelTilts(k), panelAz, ...
                        'ClearSkyFraction', 1, 'SystemLoss', 0);
                    clearSkyKWh = clearSkyKWh + sum(wClear)/1000 * daysPerMonth(m);
                end
            end

            Kt = clearnessIndex(app.Latitude, app.Longitude);

            data = {
                'Parameter', 'Value';
                'Number of panels', sprintf('%d', N);
                'Total panel area', sprintf('%.1f m²', totalArea);
                'Panel efficiency', '20%';
                'Latitude', sprintf('%.4f°', app.Latitude);
                'Longitude', sprintf('%.4f°', app.Longitude);
                'Clearness index', sprintf('%.2f', Kt);
                'Annual energy (realistic)', sprintf('%.0f kWh', annualKWh);
                'Annual energy (clear sky)', sprintf('%.0f kWh', clearSkyKWh)
            };
        end

        function data = computePanelTableData(app)
            efficiency = 0.20;
            daysPerMonth = [31 29 31 30 31 30 31 31 30 31 30 31];
            N = numel(app.Panels);

            data = {'Panel', 'Area (m²)', 'Tilt (°)', 'Azimuth (°)', 'Annual kWh'};

            for k = 1:N
                [area, panelAz] = app.panelGeometry(app.Panels{k});
                kwhYear = 0;
                for m = 1:12
                    tm = datetime(2024, m, 15, 'TimeZone', 'UTC') + hours(0:23);
                    [azm, elm] = sunPosition(app.Latitude, app.Longitude, tm);
                    nvM = app.lossArgs(tm);
                    w = solarPanelPower(efficiency, area, azm, elm, app.PanelTilts(k), panelAz, nvM{:});
                    kwhYear = kwhYear + sum(w)/1000 * daysPerMonth(m);
                end
                data(end+1, :) = {sprintf('%d', k), sprintf('%.2f', area), ...
                    sprintf('%.0f', app.PanelTilts(k)), sprintf('%.0f', panelAz), ...
                    sprintf('%.0f', kwhYear)};
            end
        end

        function tbl = createFormattedTable(~, data)
            import mlreportgen.dom.*;
            tbl = FormalTable(data(1,:), data(2:end,:));
            tbl.Border = 'solid';
            tbl.BorderWidth = '1px';
            tbl.ColSep = 'solid';
            tbl.ColSepWidth = '1px';
            tbl.RowSep = 'solid';
            tbl.RowSepWidth = '1px';
            tbl.Header.Style = {Bold(true), BackgroundColor('#2a4a7a'), Color('white')};
            tbl.Width = '100%';
        end

        function setPanelAzimuth(app, targetAz)
            idx = app.SelectedPanel;
            if idx < 1 || idx > numel(app.Panels), return; end
            roi = app.Panels{idx};
            if ~isvalid(roi), return; end
            % panelAz = mod(180 + rot, 360), so rot = targetAz - 180
            roi.RotationAngle = targetAz - 180;
            app.updatePlots();
        end

        function [area, panelAz] = panelGeometry(app, roi)
            pos = roi.Position;
            % Convert Mercator area to real m² (Mercator inflates by 1/cos²(lat))
            area = pos(3) * pos(4) * cosd(app.Latitude)^2;

            rot = roi.RotationAngle;

            % RotationAngle is clockwise in MATLAB's drawrectangle.
            % At rot=0 the bottom edge faces south (azimuth 180°).
            % CW rotation by R degrees rotates facing direction CW:
            panelAz = mod(180 + rot, 360);
        end
    end
end
