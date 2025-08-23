

function hfig = plot_env3d(env, varargin)
% PLOT_ENV3D  Plot gateways, user clusters, and EDs in 3D (with z magnified).

    p = inputParser;
    p.addParameter('ShowLabels', true, @(x)islogical(x)&&isscalar(x));
    p.addParameter('FaceAlpha', 0.07, @(x)isnumeric(x)&&isscalar(x));
    p.addParameter('EdgeAlpha', 0.4,  @(x)isnumeric(x)&&isscalar(x));
    p.addParameter('GWMarkerSize', 100, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.addParameter('EDMarkerSize', 10,  @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.addParameter('ZScale', 5, @(x)isnumeric(x)&&isscalar(x)&&x>0);
    p.parse(varargin{:});
    opt = p.Results;

    PV   = env.PV;
    R    = env.Uv_R(:);
    Piv  = env.Piv;
    V    = size(PV,1);

    hfig = figure('Color','w'); hold on; grid on;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    view(50,85);
    view(-40, 25);       
    xlim([-300 1000]); ylim([-300 1000]); zlim([0 35]);
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

        % Users (EDs) (hidden from legend)
        hEDs = scatter3(Piv{v}(:,1), Piv{v}(:,2), Piv{v}(:,3), opt.EDMarkerSize, ...
                 'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k', ...
                 'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.4);
        hideInLegend(hEDs);

        % Gateway mast (hidden)
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

    % Legend proxies: square for SGWs, dot for Users
    plot3(NaN,NaN,NaN,'s', 'MarkerSize',9, 'MarkerFaceColor',[0 0 0], 'MarkerEdgeColor','k', ...
         'LineStyle','none', 'DisplayName','SGWs');
    plot3(NaN,NaN,NaN,'o', 'MarkerSize',7, 'MarkerFaceColor',[0.5 0.5 0.5], 'MarkerEdgeColor','k', ...
         'LineStyle','none', 'DisplayName','Users');

    % Helper: hide object from legend
    function hideInLegend(h)
        set(get(get(h,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    end
end
