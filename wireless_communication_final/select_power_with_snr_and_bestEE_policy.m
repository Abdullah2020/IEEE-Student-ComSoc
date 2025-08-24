%% ====================== FUNCTIONS ======================
function [P_sel_final, EE_sel_final, hist] = select_power_with_snr_and_bestEE_policy( ...
        P_F_opt, P_MAX_UAV, params, env, N_dis_Broad, snr_thresh_dB)
% Iterate P from high to low. Select current P only if:
%   (1) all SGWs satisfy SNR_dB >= snr_thresh_dB, AND
%   (2) EE(P) > best EE recorded so far.
% Otherwise keep the previously selected power.
%
% Outputs:
%   P_sel_final, EE_sel_final : final maintained selection
%   hist : struct fields for plotting and inspection (see end)

    if nargin < 5 || isempty(N_dis_Broad), N_dis_Broad = 1000; end
    if nargin < 6 || isempty(snr_thresh_dB), snr_thresh_dB = -10; end
    if P_MAX_UAV < 0, error('P_MAX_UAV must be >= 0'); end

    % Power grid: high -> low
    P_grid = linspace(P_MAX_UAV, 0, N_dis_Broad);

    % Number of SGWs (V)
    % Assumes env.PV exists (size Vx?), else replace by your own V getter.
    V = size(env.PV, 1);

    % Prealloc
    feasible_iter   = false(1, N_dis_Broad);
    P_selected_tr   = nan(1, N_dis_Broad);
    EE_selected_tr  = zeros(1, N_dis_Broad);      % EE of maintained selection per iter
    snr_dB_all      = nan(V, N_dis_Broad);        % users x iterations

    % Bookkeeping
    have_selection  = false;
    P_selected      = NaN;
    EE_selected     = 0;                          % best EE so far (for selection)
    SR_selected     = 0;                          % optional info

    for k = 1:N_dis_Broad
        Pk = P_grid(k);

        % --- SNR of all SGWs at Pk ---
        rho_dB = compute_SNR_dB_A2G(P_F_opt, Pk * ones(V,1), params, env);
        

        is_feasible = all(rho_dB >= snr_thresh_dB);
        feasible_iter(k) = is_feasible;

        if is_feasible && (Pk > 0)
            % Compute SR/EE at current feasible P
            [SR_k, ~] = objective_static_SR_A2G(P_F_opt, Pk * ones(V,1), params, env);
            EE_k = SR_k / Pk;

            % --- New rule: adopt current P only if its EE strictly improves best EE so far
            if EE_k > EE_selected
                P_selected  = Pk;
                EE_selected = EE_k;
                SNR_selected = rho_dB;
                have_selection = true;
                
            end
        end

        % Maintain traces (selection may or may not change at this iteration)
        snr_dB_all(:, k) = SNR_selected;
        P_selected_tr(k)  = P_selected;
        EE_selected_tr(k) = EE_selected;  % 0 until the first feasible-improving is found
    end

    % Final outputs
    if ~have_selection
        P_sel_final  = 0;
        EE_sel_final = 0;
    else
        P_sel_final  = P_selected;
        EE_sel_final = EE_selected;
    end

    % History for plotting
    hist.P_grid         = P_grid;            % high -> low
    hist.snr_dB_all     = snr_dB_all;        % (V x N)
    hist.snr_thresh_dB  = snr_thresh_dB;
    hist.feasible_iter  = feasible_iter;     % logical 1xN
    hist.P_selected_tr  = P_selected_tr;     % maintained power over iterations
    hist.EE_selected_tr = EE_selected_tr;    % (a) EE of maintained selection
    hist.P_sel_final    = P_sel_final;
    hist.EE_sel_final   = EE_sel_final;
end
