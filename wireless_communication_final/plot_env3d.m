
%==================== FUNCTION ====================%
function hfig = plot_env3d(env, varargin)
% PLOT_ENV3D  Plot gateways, user clusters, and EDs in 3D.
% Legend shows only one entry for SGWs (square) and Users (circle).
%
% Optional name-value args:
%   'ShowLabels' (logical, default true)
%   'FaceAlpha'  (scalar, default 0.07)
%   'EdgeAlpha'  (scalar, default 0.4)
%   'GWMarkerSize' (scalar, default 100)
%   'EDMarkerSize' (scalar, default 10)
%   'ZScale'     (scalar, default 2)  % smaller -> Z looks "down"
%   'AutoFit'    (logical, default true)  % auto-fit XY to points
%   'XYPadding'  (scalar, default 100)    % padding (m) if AutoFit
%   'XLim'       ([xmin xmax], optional)  % fixed X limits (overrides AutoFit)
%   'YLim'       ([ymin ymax], optional)  % fixed Y limits (overrides AutoFit)

    p = inputParser;
    p.addParameter('ShowLabels', true, @(x)islogical(x)&&isscalar(x));
    p.addParameter('FaceAlpha', 0.07, @(x)isnumeric(x)&&isscalar(x));
    p.addParameter('EdgeAlpha', 0.4,  @(x)isnumeric(x)&&isscalar(x));
    p.addParameter('GWMarkerSize', 100, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.addParameter('EDMarkerSize', 10,  @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.addParameter('ZScale', 2, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.addParameter('AutoFit', true, @(x)islogical(x)&&isscalar(x));
    p.addParameter('XYPadding', 100, @(x)isnumeric(x)&&isscalar(x)&&x>=0);
    p.addParameter('XLim', [], @(x)isnumeric(x)&&numel(x)==2);
    p.addParameter('YLim', [], @(x)isnumeric(x)&&numel(x)==2);
    p.parse(varargin{:});
    opt = p.Results;

    PV   = env.PV;      % V x 3 (SGW positions)
    R    = env.Uv_R(:); % V x 1 (cluster radii)
    Piv  = env.Piv;     % 1 x V cell, each Ni x 3 (users/EDs)
    V    = size(PV,1);

    % Collect all XY points for autofit (SGWs + all EDs)
    allXY = PV(:,1:2);
    for v = 1:V
        if ~isempty(Piv{v})
            allXY = [allXY; Piv{v}(:,1:2)]; %#ok<AGROW>
        end
    end

    hfig = figure('Color','w'); hold on; grid on;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    view(-40, 25);

    % Set XY limits
    if ~isempty(opt.XLim)
        xlim(opt.XLim);
    elseif opt.AutoFit && ~isempty(allXY)
        xmin = min(allXY(:,1)) - opt.XYPadding;
        xmax = max(allXY(:,1)) + opt.XYPadding;
        xlim([xmin xmax]);
    else
        xlim([-300 1000]); % fallback
    end

    if ~isempty(opt.YLim)
        ylim(opt.YLim);
    elseif opt.AutoFit && ~isempty(allXY)
        ymin = min(allXY(:,2)) - opt.XYPadding;
        ymax = max(allXY(:,2)) + opt.XYPadding;
        ylim([ymin ymax]);
    else
        ylim([-300 1000]); % fallback
    end

    % Z range (initial, will be tightened later in main script)
    zlim([0 35]);

    % Make Z look less tall
    pbaspect([1 1 opt.ZScale]);

    cmap = lines(V);
    th = linspace(0, 2*pi, 256);

    for v = 1:V
        cx = PV(v,1); cy = PV(v,2); rv = R(v);

        % Cluster disk (hidden from legend)
        hPatch = patch('XData', cx + rv*cos(th), 'YData', cy + rv*sin(th), 'ZData', zeros(size(th)), ...
              'FaceColor', cmap(v,:), 'FaceAlpha', opt.FaceAlpha, ...
              'EdgeColor', cmap(v,:), 'EdgeAlpha', opt.EdgeAlpha, ...
              'LineWidth', 1.2);
        hideInLegend(hPatch);

        % Users (EDs) - hidden from legend
        if ~isempty(Piv{v})
            hEDs = scatter3(Piv{v}(:,1), Piv{v}(:,2), Piv{v}(:,3), opt.EDMarkerSize, ...
                     'o', 'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k', ...
                     'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.4);
            hideInLegend(hEDs);
        end

        % Mast (hidden)
        hMast = plot3([cx cx], [cy cy], [0 PV(v,3)], '--', 'Color', cmap(v,:), 'LineWidth', 1.0);
        hideInLegend(hMast);

        % SGW marker (hidden)
        hGW = scatter3(PV(v,1), PV(v,2), PV(v,3), opt.GWMarkerSize, ...
                 's', 'filled', 'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k');
        hideInLegend(hGW);

        if opt.ShowLabels
            text(PV(v,1), PV(v,2), PV(v,3), sprintf('  p_{%d}', v), ...
                 'FontWeight','bold', 'VerticalAlignment','bottom');
        end
    end

    % === Legend proxies: single entries ===
    % Square for SGWs
    plot3(NaN,NaN,NaN,'s', 'MarkerSize',9, 'MarkerFaceColor',[0 0 0], 'MarkerEdgeColor','k', ...
         'LineStyle','none', 'DisplayName','SGWs');
    % Circle for Users
    plot3(NaN,NaN,NaN,'o', 'MarkerSize',7, 'MarkerFaceColor',[0.5 0.5 0.5], 'MarkerEdgeColor','k', ...
         'LineStyle','none', 'DisplayName','Users');

    % Helper: hide object from legend
    function hideInLegend(h)
        set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end
