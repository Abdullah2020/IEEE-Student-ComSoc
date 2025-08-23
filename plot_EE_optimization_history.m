function plot_EE_optimization_history(hist)
%PLOT_EE_OPTIMIZATION_HISTORY Visualize the scan and the best-so-far evolution.

    % (A) EE vs P_UAV scan
    figure('Name','EE vs P_{UAV}');
    plot(hist.P_grid, hist.EE_grid, 'LineWidth', 1.6); grid on;
    xlabel('P_{UAV} (W)'); ylabel('EE (e.g., bits/J)');
    title('Energy Efficiency vs. P_{UAV}');
    hold on;
    plot(hist.P_grid(hist.idx_best_final), hist.EE_grid(hist.idx_best_final), 'o', 'MarkerSize', 8, 'LineWidth', 1.6);
    legend('EE curve','Best point','Location','best');

    % (B) Best-so-far EE over iterations
    figure('Name','Best-So-Far EE over Iterations');
    it = 1:numel(hist.EE_grid);
    plot(it, hist.best_EE_trace, 'LineWidth', 1.6); grid on;
    xlabel('Iteration (grid index)'); ylabel('Best-So-Far EE');
    title('Evolution of Best-So-Far EE');

    % (C) Optional: SR vs P_UAV (helpful for diagnosis)
    figure('Name','SR vs P_{UAV}');
    plot(hist.P_grid, hist.SR_grid, 'LineWidth', 1.6); grid on;
    xlabel('P_{UAV} (W)'); ylabel('SR (e.g., bits/s)');
    title('Sum Rate vs. P_{UAV}');
end
