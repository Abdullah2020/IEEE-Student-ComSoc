% function PSCA_Trajectory_and_power_opt_with_plots
clc; clear; close all;

%% ==========================
%  Main Script to Setup Env
% ===========================

% ----- Gateways & clusters -----
PV_in = [250  250;
         700  700;
           0    0;
          50  800];
R = 50 * ones(size(PV_in,1));          % cluster radii (m)
N_per_cluster = 30 * ones(size(PV_in,1));
h_SG = 15;                              % gateway mast height (m)

% ====== Create environment ======
env = setup_env(PV_in, R, N_per_cluster, h_SG);

%% Trajectory settings
N_disc   = 30;             % number of waypoints
p_init   = [750, 0, 60]; % initial UAV position
p_final  = [50, 800, 60]; % final   UAV position
v_max    = 30;             % max speed (m/s)
delta_t  = 1.0;            % time step (s)
S_box    = [0 1000; 0 1000; 59  60]; % feasible box for every waypoint

% Initial path: straight-line interpolation
P_path0 = zeros(N_disc,3);
for n = 1:N_disc
    t = (n-1)/(N_disc-1);
    P_path0(n,:) = (1-t)*p_init + t*p_final;
end

%% Initial powers (per gateway)
V = size(env.PV,1);
P0_row = 0.5 * rand(1, V);
P0     = P0_row; % will be vectorized inside

%% =====  A2G parameters =====
params.alpha       = 4.88;
params.lambda      = 0.43;
params.eta_LoS_dB  = 0.1;
params.eta_NLoS_dB = 21.0;
params.f_Hz        = 868e6;
params.c           = 3e8;
sigma2_dBm         = -90;
params.sigma2_W    = 10^((sigma2_dBm-30)/10);
params.W_Hz        = 125e3 * ones(V,1);  % per-GW bandwidth (V x 1)

%% =====  PSCA options =====
opts_traj.S_box        = S_box;                 % box for each waypoint
opts_traj.Pmin         = 0.00 * ones(numel(P0),1);
opts_traj.Pmax         = 0.50 * ones(numel(P0),1);
opts_traj.rho_th_dB    = -7.5;                  % SNR threshold
opts_traj.gamma0       = 0.01;
opts_traj.gamma_decay  = 1e-6;                  % exponential decay
opts_traj.gamma_min    = 0.02;                  % floor
opts_traj.maxIter      = 1000;
opts_traj.tol          = 1e-7;
opts_traj.fd_eps       = 1e-12;
opts_traj.v_max        = v_max;
opts_traj.delta_t      = delta_t;
opts_traj.p_init       = p_init;
opts_traj.p_final      = p_final;

% fmincon (subproblem) options
opts_traj.fmopts = struct( ...
    'Algorithm','interior-point', ...
    'Display','iter', ...  % 'off'|'final'|'iter'|'iter-detailed'
    'MaxIterations', 300, ...
    'MaxFunctionEvaluations', 1000, ...
    'OptimalityTolerance', 1e-10, ...
    'StepTolerance', 1e-12, ...
    'ConstraintTolerance', 1e-9, ...
    'SpecifyObjectiveGradient', true ...
);

%% =====  Run PSCA (trajectory + power) =====
[P_path_opt, P_opt, info] = psca_traj_fmincon_ee(P_path0, P0, env, params, opts_traj);

% Compute final metrics
[SR_tot_opt, SR_n_opt] = objective_traj_SR_A2G(P_path_opt, P_opt, params, env);
EE_opt = SR_tot_opt / sum(P_opt);
fprintf('PSCA finished in %d iters. EE=%.6g (SR_total=%.6g, sumP=%.6g)\n', ...
        info.iters, EE_opt, SR_tot_opt, sum(P_opt));

%% ========== Plots ==========

% 1) Environment + optimal trajectory
hfig = plot_env3d(env, 'ZScale', 5, 'ShowLabels', true); %#ok<NASGU>
hold on;
plot3(P_path_opt(:,1), P_path_opt(:,2), P_path_opt(:,3), 'r-', 'LineWidth', 2.0, 'DisplayName','Optimal Trajectory');
scatter3(p_init(1), p_init(2), p_init(3), 120, 'ko', 'filled', 'DisplayName','Start');
scatter3(p_final(1), p_final(2), p_final(3), 120, 'g^', 'filled', 'DisplayName','End');
zl = zlim; zlim([0, max([zl(2), max(P_path_opt(:,3))*1.2, max(env.PV(:,3))*1.2])]);
legend('Location','best'); title('Environment and Optimal UAV Trajectory');
grid on; hold off;

iters = 1:info.iters;

% 2) Objective (energy efficiency) evolution
figure('Color','w');
plot(iters, info.hist.EE, 'LineWidth', 2);
xlabel('Iteration'); ylabel('Energy efficiency  [bps/W]');
title('PSCA Objective Evolution'); grid on;

% 3) Power evolution per gateway
figure('Color','w');
plot(iters, info.hist.P, 'LineWidth', 1.8);
xlabel('Iteration'); ylabel('Transmit power  [W]');
legend(arrayfun(@(v)sprintf('P_%d',v), 1:size(info.hist.P,2), 'UniformOutput', false), ...
       'Location','best');
title('Power Evolution per Gateway'); grid on;

% 4) Total rate (sum over time) + per-gateway total rate evolution
figure('Color','w');
plot(iters, info.hist.SR_total, 'k-', 'LineWidth', 2); hold on;
plot(iters, info.hist.Rv_total, '--', 'LineWidth', 1.5);
lgd = [{'Total SR (sum over time)'}, arrayfun(@(v)sprintf('Sum R_%d',v), 1:size(info.hist.Rv_total,2), 'UniformOutput', false)];
legend(lgd, 'Location','best');
xlabel('Iteration'); ylabel('Rate  [bps]');
title('Total (over time) Rate Evolution'); grid on; hold off;

% end % ===== end main =====

%% Save

save("Data_traj_opt.mat")

%% ==========================
%  PSCA with fmincon (trajectory + power)
% ==========================
function [P_path_opt, P_opt, info] = psca_traj_fmincon_ee(P_path0, P0, env, params, opts)
% Blocks:
%   positions: x_1 = p_1, ..., x_N = p_N   (p_1 fixed = p_init, p_N fixed = p_final)
%   powers   : x_{N+1} = P_1, ..., x_{N+V} = P_V (constant over time)
%
% Objective: maximize EE = (sum_n SR(p_n, P)) / sum_v P_v
% Constraints:
%   - p_n ∈ S_box (for all n)
%   - ||p_{n+1}-p_n|| <= v_max*delta_t (for n=1..N-1)
%   - P_v ∈ [Pmin, Pmax]
%   - SNR constraint for each gateway v holds for ALL waypoints:
%       P_v >= max_n rho_th * sigma2 / G(n,v)

[N_disc,~] = size(P_path0);
V = numel(P0);
P_path = P_path0;
P = P0(:);                        % Vx1 powers

% Sanity: fix endpoints
P_path(1,:)   = opts.p_init(:).';
P_path(end,:) = opts.p_final(:).';

% thresholds / defaults
if isfield(opts,'rho_th_dB')
    rho_th = 10^(opts.rho_th_dB/10);
elseif isfield(opts,'rho_th')
    rho_th = opts.rho_th;
else
    rho_th = 1; % 0 dB
end
S_box = opts.S_box;
s_max = opts.v_max * opts.delta_t;

% fmincon options
fmopts = build_fmincon_opts(opts);

% histories
histEE      = nan(opts.maxIter,1);
histSR_tot  = nan(opts.maxIter,1);
histPsum    = nan(opts.maxIter,1);
histP       = nan(opts.maxIter,V);
histRv_tot  = nan(opts.maxIter,V);   % per-gateway total rate over time

gamma = opts.gamma0;

for it = 1:opts.maxIter

    % --- evaluate model at current (path, P) ---
    [SR_total, SR_n, details] = objective_traj_SR_A2G(P_path, P, params, env); %#ok<ASGLU>
    % details.G is [N_disc x V]
    EE = SR_total / sum(P);
    Fl = -EE;   % we minimize F = -EE

    % --- power gradients (analytic, sum over time) ---
    gF_P = zeros(V,1);
    for v = 1:V
        gEE_Pv = grad_EE_Pv_traj(v, P_path, P, params, env, SR_total, details);
        gF_P(v) = -gEE_Pv;
    end

    % --- position gradients (finite diff, per waypoint except endpoints) ---
    gF_pn = zeros(N_disc,3);
    for n = 2:(N_disc-1)   % skip p1 and pN (fixed)
        gEE_pn = grad_EE_pn(n, P_path, P, params, env, opts.fd_eps);
        gF_pn(n,:) = -gEE_pn(:).';
    end

    % --- BLOCKS: positions (2..N-1) ---
    P_path_hat = P_path;
    for n = 2:(N_disc-1)
        % affine surrogate for p_n
        objn = @(p) surrogate_affine(Fl, gF_pn(n,:).', P_path(n,:).', p(:));
        % box bounds
        LB = S_box(:,1)';  UB = S_box(:,2)';
        % speed constraints vs neighbors (neighbors are constant in block)
        prev = P_path(n-1,:); next = P_path(n+1,:);
        nonlcon = @(p) speed_constr_block(p(:).', prev, next, s_max);
        % solve
        p_hat = fmincon(objn, P_path(n,:), [],[],[],[], LB, UB, nonlcon, fmopts);
        P_path_hat(n,:) = p_hat;
    end
    % ensure endpoints remain fixed
    P_path_hat(1,:)   = P_path(1,:);
    P_path_hat(end,:) = P_path(end,:);

    % --- BLOCKS: powers (per gateway, with SNR lower bound across ALL n) ---
    P_hat = P;
    % compute SNR-based lower bound over the trajectory
    G = details.G; % [N_disc x V]
    for v = 1:V
        Psnr_lb_allN = rho_th * params.sigma2_W ./ max(G(:,v), realmin);
        Psnr_lb      = max(Psnr_lb_allN);  % must satisfy at all time steps
        LBv          = min(max(opts.Pmin(v), Psnr_lb), opts.Pmax(v)); % avoid infeasible bound
        UBv          = opts.Pmax(v);
        objv         = @(pv) surrogate_affine(Fl, gF_P(v), P(v), pv);
        P_hat(v)     = fmincon(objv, P(v), [],[],[],[], LBv, UBv, [], fmopts);
    end

    % --- Relaxed update ---
    P_path_new = P_path;
    P_path_new(2:end-1,:) = P_path(2:end-1,:) + gamma * (P_path_hat(2:end-1,:) - P_path(2:end-1,:));
    P_new = P + gamma * (P_hat - P);

    % --- record & stopping ---
    [SR_total_new, SR_n_new, details_new] = objective_traj_SR_A2G(P_path_new, P_new, params, env); %#ok<ASGLU>
    EE_new = SR_total_new / sum(P_new);

    histEE(it)     = EE_new;
    histSR_tot(it) = SR_total_new;
    histPsum(it)   = sum(P_new);
    histP(it,:)    = P_new(:).';

    % per-gateway total rates over time for plotting
    Rv_tot = sum_rate_per_gateway_over_time(P_path_new, P_new, params, env); % [1 x V]
    histRv_tot(it,:) = Rv_tot;

    if it > 5 && abs(EE_new - EE)/max(1,abs(EE)) < opts.tol
        P_path = P_path_new; P = P_new; break;
    end

    P_path = P_path_new; P = P_new;
    % decay with floor
    % if isfield(opts,'gamma_min')
    %     gamma = max(opts.gamma_min, gamma * opts.gamma_decay);
    % else
    %     gamma = max(0.2, gamma * opts.gamma_decay);
    % end
    gamma =gamma*(1 - opts.gamma_decay * gamma);
end

P_path_opt = P_path;  P_opt = P;
info.iters         = it;
info.hist.EE       = histEE(1:it);
info.hist.SR_total = histSR_tot(1:it);
info.hist.Psum     = histPsum(1:it);
info.hist.P        = histP(1:it,:);
info.hist.Rv_total = histRv_tot(1:it,:);
end

% --- helper: fmincon options
function fmopts = build_fmincon_opts(opts)
fmopts = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','iter', ...
    'SpecifyObjectiveGradient',true, ...
    'MaxIterations', 60, ...
    'MaxFunctionEvaluations', 300, ...
    'OptimalityTolerance', 1e-9, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-9);
if isfield(opts,'fmopts') && isstruct(opts.fmopts)
    fns = fieldnames(opts.fmopts);
    for k = 1:numel(fns)
        name = fns{k};
        try, fmopts.(name) = opts.fmopts.(name); catch, end %#ok<CTCH>
    end
end
end

% --- affine surrogate (first-order Taylor) ---
function [f, g] = surrogate_affine(F0, gF, x0, x)
f = F0 + gF(:)'*(x(:) - x0(:));
g = gF(:)';  % constant gradient for fmincon
end

% --- nonlinear speed constraints for a single waypoint (block) ---
function [c, ceq] = speed_constr_block(p, prev, next, s_max)
% c <= 0; ceq = []
c = [];
if ~isempty(prev)
    c(end+1,1) = norm(p - prev) - s_max;
end
if ~isempty(next)
    c(end+1,1) = norm(next - p) - s_max;
end
ceq = [];
end

% --- objective over trajectory: SR_total and details (incl. G over time) ---
function [SR_total, SR_n, details_all] = objective_traj_SR_A2G(P_path, P_tx_W, params, env)
N = size(P_path,1);
V = numel(P_tx_W);
SR_n = zeros(N,1);
G_all = zeros(N,V);
rho_all = zeros(N,V);
for n = 1:N
    [SR_n(n), ~, det] = objective_static_SR_A2G(P_path(n,:), P_tx_W, params, env);
    G_all(n,:)   = det.G(:).';
    rho_all(n,:) = det.rho(:).';
end
SR_total = sum(SR_n);
details_all = struct('G', G_all, 'rho', rho_all);
end

% --- analytic grad EE wrt P_v aggregated over time ---
function dEE = grad_EE_Pv_traj(v, P_path, P, params, env, SR_total, details)
% dSR_total/dP_v = sum_n dR_{n,v}/dP_v
% where dR/dP = (W/log2) * (G/sigma2) / (1 + P_v G / sigma2)
sigma2 = params.sigma2_W;
Wv = params.W_Hz(v);
Gv_all = details.G(:,v);      % [N x 1]
rho_all = P(v)*Gv_all/sigma2;
dSR_dPv_all = (Wv/log(2)) * (Gv_all/sigma2) ./ (1 + rho_all); % [N x 1]
dSR_dPv = sum(dSR_dPv_all);
S = sum(P);
dEE = (dSR_dPv*S - SR_total) / (S^2);
end

% --- finite-difference grad EE wrt a single waypoint p_n (3D) ---
function g = grad_EE_pn(n, P_path, P, params, env, h)
[SR0, ~] = objective_traj_SR_A2G(P_path, P, params, env);
EE0 = SR0 / sum(P);
g = zeros(3,1);
for k = 1:3
    Pp = P_path;
    Pp(n,k) = Pp(n,k) + h;
    [SRp, ~] = objective_traj_SR_A2G(Pp, P, params, env);
    EEp = SRp / sum(P);
    g(k) = (EEp - EE0) / h;
end
end

% --- helper: per-gateway total rate over time (for plotting) ---
function Rv_tot = sum_rate_per_gateway_over_time(P_path, P, params, env)
N = size(P_path,1);
V = numel(P);
Rv_tot = zeros(1,V);
for n = 1:N
    [~, Rv, ~] = objective_static_SR_A2G(P_path(n,:), P, params, env);
    Rv_tot = Rv_tot + Rv(:).';
end
end

%% ==========================
%  Static single-position objective (given)
% ==========================
function [SR, R_v, details] = objective_static_SR_A2G(P_F, P_tx_W, params, env)
PV = env.PV;                        % Vx3
V  = size(PV,1);

% checks & shapes
assert(any(all(size(P_tx_W) == [V,1]) | all(size(P_tx_W) == [1,V])), ...
       'P_tx_W must be V-length');
P_tx_W = P_tx_W(:);
assert(any(all(size(params.W_Hz) == [V,1]) | all(size(params.W_Hz) == [1,V])), ...
       'params.W_Hz must be V-length');
W_Hz = params.W_Hz(:);

% geometry
d_xy   = vecnorm(PV(:,1:2) - P_F(1:2), 2, 2);
d_z    =  P_F(3) - PV(:,3);
d      = sqrt(d_xy.^2 + d_z.^2);
phiDeg = atan2d(d_z, d_xy);

% LoS probability
alpha  = params.alpha;
lambda = params.lambda;
P_LoS  = 1 ./ (1 + alpha .* exp(-lambda .* (phiDeg - alpha)));
P_NLoS = 1 - P_LoS;

% path loss
L_FS_dB  = 20*log10(4*pi*params.f_Hz.*d./params.c);
L_A2G_dB = L_FS_dB + params.eta_LoS_dB .* P_LoS + params.eta_NLoS_dB .* P_NLoS;

% channel gain & SNR
G   = 10.^(-L_A2G_dB/10);
rho = (P_tx_W .* G) ./ params.sigma2_W;

% rates
R_v = W_Hz .* log2(1 + rho);
SR  = sum(R_v);

% details
if nargout >= 3
    details = struct('d', d, 'phi_deg', phiDeg, 'P_LoS', P_LoS, ...
                     'L_FS_dB', L_FS_dB, 'L_A2G_dB', L_A2G_dB, ...
                     'G', G, 'rho', rho);
end
end

%% ==========================
%  Environment Setup
% ==========================
function env = setup_env(PV_in, R, N_per_cluster, h_SG)
rng(1); % reproducible
V = size(PV_in,1);
PV = [PV_in, h_SG*ones(V,1)];

Piv = cell(1,V);
for v = 1:V
    Ni = N_per_cluster(v);
    rv = R(v);
    % uniform in disk
    theta = 2*pi*rand(Ni,1);
    rad   = rv * sqrt(rand(Ni,1));
    x = PV_in(v,1) + rad.*cos(theta);
    y = PV_in(v,2) + rad.*sin(theta);
    z = zeros(Ni,1);
    Piv{v} = [x y z];
end

env = struct('PV', PV, 'Uv_R', R(:), 'Piv', {Piv}, 'h_SG', h_SG);
end

%% ==========================
%  Plotting (3D environment)
% ==========================

function hfig = plot_env3d(env, varargin)
% PLOT_ENV3D  Plot gateways, user clusters, and EDs in 3D (with z magnified).

p = inputParser;
p.addParameter('ShowLabels', true,  @(x)islogical(x)&&isscalar(x));
p.addParameter('FaceAlpha', 0.07,   @(x)isnumeric(x)&&isscalar(x));
p.addParameter('EdgeAlpha', 0.4,    @(x)isnumeric(x)&&isscalar(x));
p.addParameter('GWMarkerSize', 100, @(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('EDMarkerSize', 10,  @(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('ZScale', 5,         @(x)isnumeric(x)&&isscalar(x)&&x>0);  % z magnification
p.addParameter('AutoXY', true,      @(x)islogical(x)&&isscalar(x));       % auto x/y limits
p.addParameter('XYPadding', 0.12,   @(x)isnumeric(x)&&isscalar(x)&&x>=0); % 12% padding
p.addParameter('LegendClean', true, @(x)islogical(x)&&isscalar(x));       % hide env from legend
p.parse(varargin{:});
opt = p.Results;

PV   = env.PV;
R    = env.Uv_R(:);
Piv  = env.Piv;
V    = size(PV,1);

% For cleaner legends, hide environment objects if requested
hv = 'on';
if opt.LegendClean
    hv = 'off';
end

hfig = figure('Color','w'); hold on; grid on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
view(45,25);
pbaspect([1 1 opt.ZScale]); % stretch z visually

cmap = lines(V);
th = linspace(0, 2*pi, 256);

for v = 1:V
    cx = PV(v,1); cy = PV(v,2); rv = R(v);
    % Cluster disk
    xCirc = cx + rv*cos(th);
    yCirc = cy + rv*sin(th);
    zCirc = zeros(size(th));
    patch('XData', xCirc, 'YData', yCirc, 'ZData', zCirc, ...
          'FaceColor', cmap(v,:), 'FaceAlpha', opt.FaceAlpha, ...
          'EdgeColor', cmap(v,:), 'EdgeAlpha', opt.EdgeAlpha, ...
          'LineWidth', 1.2, 'HandleVisibility', hv);

    % EDs
    pts = Piv{v};
    scatter3(pts(:,1), pts(:,2), pts(:,3), opt.EDMarkerSize, ...
             'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k', ...
             'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.4, ...
             'HandleVisibility', hv);

    % Gateway mast + head
    plot3([cx cx], [cy cy], [0 PV(v,3)], '--', 'Color', cmap(v,:), ...
          'LineWidth', 1.0, 'HandleVisibility', hv);
    scatter3(PV(v,1), PV(v,2), PV(v,3), opt.GWMarkerSize, ...
             's', 'filled', 'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k', ...
             'HandleVisibility', hv);

    if opt.ShowLabels
        text(PV(v,1), PV(v,2), PV(v,3), sprintf('  p_{%d}', v), ...
             'FontWeight','bold', 'VerticalAlignment','bottom', ...
             'HandleVisibility','off');
    end
end

% Dynamic x/y limits from environment geometry
allPts = [cell2mat(Piv(:)); PV];
minxy = min(allPts(:,1:2), [], 1);
maxxy = max(allPts(:,1:2), [], 1);
span  = max(maxxy - minxy);
pad   = opt.XYPadding * max(span, 1);

if opt.AutoXY
    xlim([minxy(1)-pad, maxxy(1)+pad]);
    ylim([minxy(2)-pad, maxxy(2)+pad]);
else
    xlim([0 1000]); ylim([0 1000]);
end
zlim([0 20]); % base; caller can override after plotting trajectory

% Ground outline (hidden from legend)
plot3([minxy(1) maxxy(1) maxxy(1) minxy(1) minxy(1)], ...
      [minxy(2) minxy(2) maxxy(2) maxxy(2) minxy(2)], ...
      zeros(1,5), 'k:', 'LineWidth', 1.0, 'HandleVisibility','off');

title('Gateways p_v (z=h_{SG}), clusters \mathcal{U}_v, and EDs p_i^v');
end
