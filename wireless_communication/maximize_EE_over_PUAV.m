function [P_UAV_opt, EE_opt, hist] = maximize_EE_over_PUAV(P_F_opt, P_MAX_UAV, params, env, N_dis_Broad)
%MAXIMIZE_EE_OVER_PUAV Grid search for P_UAV that maximizes energy efficiency,
% while recording history (EE and SR) over iterations and providing plotting-ready data.
%
% Returns:
%   P_UAV_opt : argmax P_UAV
%   EE_opt    : max EE(P_UAV)
%   hist      : struct with full evaluation history (for plotting)
%
% Notes:
%   - EE(0) is set to 0 to avoid division by zero.
%   - We recompute SR here to expose it in the history.

    if nargin < 5 || isempty(N_dis_Broad), N_dis_Broad = 1000; end
    if P_MAX_UAV < 0, error('P_MAX_UAV must be >= 0'); end

    % Discretize [0, P_MAX_UAV]
    P_grid = linspace(0, P_MAX_UAV, N_dis_Broad);
    EE_grid = zeros(1, N_dis_Broad);
    SR_grid = zeros(1, N_dis_Broad);

    V = size(env.PV,1);

    best_EE_so_far = -inf;
    best_P_so_far  = NaN;

    best_EE_trace = zeros(1, N_dis_Broad);
    best_P_trace  = zeros(1, N_dis_Broad);
    improved_idx  = false(1, N_dis_Broad);

    for k = 1:N_dis_Broad
        Pk = P_grid(k);

        if Pk <= 0
            SR_k = 0;
            EE_k = 0;  % define EE(0) = 0
        else
            % Compute SR as in your objective
            P_tx_W = Pk * ones(V, 1);
            [SR_k, ~] = objective_static_SR_A2G(P_F_opt, P_tx_W, params, env);
            EE_k = SR_k / Pk;
        end

        SR_grid(k) = SR_k;
        EE_grid(k) = EE_k;

        % Track best-so-far
        if EE_k > best_EE_so_far
            best_EE_so_far = EE_k;
            best_P_so_far  = Pk;
            improved_idx(k) = true;
        end
        best_EE_trace(k) = best_EE_so_far;
        best_P_trace(k)  = best_P_so_far;
    end

    [EE_opt, idx_best] = max(EE_grid);
    P_UAV_opt = P_grid(idx_best);

    % History struct for analysis/plotting
    hist.P_grid         = P_grid;
    hist.SR_grid        = SR_grid;
    hist.EE_grid        = EE_grid;
    hist.best_EE_trace  = best_EE_trace;   % best EE after each iteration
    hist.best_P_trace   = best_P_trace;    % corresponding P after each iteration
    hist.improved_idx   = improved_idx;    % flags where a new best was found
    hist.idx_best_final = idx_best;
    hist.P_step         = (N_dis_Broad > 1) * (P_MAX_UAV / (N_dis_Broad - 1));
end
