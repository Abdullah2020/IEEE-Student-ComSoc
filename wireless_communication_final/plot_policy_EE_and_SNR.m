function plot_policy_EE_and_SNR(hist)
% ONLY the two visuals requested:
%   (a) EE of the maintained/selected power vs iteration
%   (b) All-users SNR vs iteration with threshold

    it = 1:numel(hist.P_selected_tr);

    % (a) EE of maintained selection
    figure('Name','EE of Selected Power over Iterations'); clf;
    plot(it, hist.EE_selected_tr, 'LineWidth', 1.6); grid on;
    xlabel('Iteration (P_{UAV}: high \rightarrow low)');
    ylabel('EE (bits/J)');
    title('Energy Efficiency : Optimization of UAV Transmit Power for broadcasting');

    % (b) All-users SNR vs iteration + threshold
    figure('Name','All-Users SNR over Iterations'); clf; hold on;
    if ~isempty(hist.snr_dB_all)
        for u = 1:size(hist.snr_dB_all,1)
            plot(it, hist.snr_dB_all(u,:), 'LineWidth', 1.0);
        end
    end
    yline(hist.snr_thresh_dB, '--', sprintf('Threshold = %.1f dB', hist.snr_thresh_dB), 'LineWidth', 1.4);
    grid on; hold off;
    xlabel('Iteration (P_{UAV}: high \rightarrow low)');
    ylabel('SNR (dB)');
    title('SNR of All SGWs : Optimization for efficient broadcasting');

    % Optional legend (can be many entries if V is large)
    if size(hist.snr_dB_all,1) <= 12
        legend_entries = arrayfun(@(u) sprintf('SGW %d',u), 1:size(hist.snr_dB_all,1), 'UniformOutput', false);
        legend([legend_entries, {'Threshold'}], 'Location','bestoutside');
    end
end