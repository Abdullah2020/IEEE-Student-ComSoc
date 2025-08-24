clc;
clear;
close all;

% SETUP_ENV  Static gateways and their ED clusters with ground-origin frame.
%
% Notation:
%   env.PV(v,:)     -> p_v        (gateway v position in R^3, z = h_SG)
%   env.Piv{v}(i,:) -> p_i^v      (ED positions in cluster U_v, z = 0)
%   env.Uv_R(v)     -> cluster radius for U_v (meters)
%   env.h_SG        -> h_SG       (gateway mast height, scalar)
%
% Inputs (all REQUIRED; no defaults):
%   PV_in         : Vx2 gateway base coordinates (x,y) in meters.
%   R             : Vx1 cluster radii (meters) for each gateway v.
%   N_per_cluster : Vx1 number of EDs in each cluster U_v.
%   h_SG          : scalar mast height (meters). Gateways are at z = h_SG.
%
% Output:
%   env struct with fields: PV (Vx3), Uv_R (Vx1), Piv (1xV cell, Ni x 3), h_SG (scalar)
  


%% ==========================
%  Main Script to Setup Env
%  ==========================

% ----- Input Parameters -----
% Gateway base coordinates (x, y) in meters
PV_in = [250  250;     % Gateway 1
         700 700];    % Gateway 2

% Cluster radii (meters) for each gateway
R = [200;
     200];

% Number of EDs in each cluster U_v
N_per_cluster = [30;
                 30];

% Gateway mast height (meters)
h_SG = 15;

% ----- Create Environment -----
% hfig = plot_env3d(env, 'ZScale',1.5);


% ====== Create environment ======
env = setup_env(PV_in, R, N_per_cluster, h_SG);

% ====== Extract variables ======

% Gateway positions (p_v) in 3D [x y z]
PV = env.PV;          % V x 3

% End device positions (p_i^v) for each cluster
% This is a 1xV cell, each containing Ni x 3 positions
Piv_all = env.Piv;

% Example: 

% ED positions in cluster 1
Piv1 = Piv_all{1};    % Ni x 3

% 10th user position in cluster 1
p10_1 = Piv_all{1}(10,:);

% 1st SGW positions
PV_1 = PV(1,:);



%%  Communication section


% ===== 2. Flying gateway position =====

% flying gateway altitude
h=60;

% Initial guess: position
P_F_0 = [400, 400, h];   % [x y z] in meters

% Initial guess: Transmit power (W) per gateway
P_tx_W_0 = 0.5 * rand(1, 2); %  [0.5; 0.5];   % 500mW=0.5W each (27dBm)


%% ===== 3. Define A2G parameters =====

params.alpha       = 4.88;         % Y. Shi, Y. Xia, and Y. Gao, "Joint gateway selection and resource allocation for cross-tier communication in space-air-ground integrated iot networks," IEEE Access, vol. 9, pp. 4303–4314, 2020.
params.lambda      = 0.43;         % Y. Shi, Y. Xia, and Y. Gao, "Joint gateway selection and resource allocation for cross-tier communication in space-air-ground integrated iot networks," IEEE Access, vol. 9, pp. 4303–4314, 2020.
params.eta_LoS_dB  = 0.1;          % Excess loss for LoS (dB)
params.eta_NLoS_dB = 21.0;         % Excess loss for NLoS (dB)
params.f_Hz        = 868e6;        % Carrier frequency (Hz)
params.c           = 3e8;          % Speed of light (m/s)
sigma2_dBm =  -90;                     % dBm
params.sigma2_W    = 10^((sigma2_dBm-30) / 10);        % Noise power (W) = -100dBm Ref: % H. E. Hammouti, A. Saoud, A. Ennahkami and E. H. Bergou, "Energy Efficient Aerial RIS: Phase Shift Optimization and Trajectory Design," 2024 IEEE 99th Vehicular Technology Conference (VTC2024-Spring), Singapore, Singapore, 2024, pp. 1-7, doi: 10.1109/VTC2024-Spring62846.2024.10683430.
params.W_Hz        = [125e3; 125e3]; % Bandwidth per GW (Hz)

%% Execute PSCA
% ----- PSCA options -----
opts.S_box = [0 1000; 0 1000; 30 70]; % only for block x1
opts.Pmin  = 0.0*ones(size(P_tx_W_0));
opts.Pmax  = 0.50*ones(size(P_tx_W_0));
opts.rho_th_dB = -7.5;     % example: 0 dB threshold
opts.gamma0 = 0.08; 
opts.gamma_decay = 1e-6;
opts.maxIter = 200; 
opts.tol = 1e-4; 
opts.fd_eps = 1e-2;

% Run PSCA (uses your objective_static_SR_A2G)
[pF_opt, P_opt, info] = psca_fmincon_ee(P_F_0, P_tx_W_0, env, params, opts);

[SR_opt, ~] = objective_static_SR_A2G(pF_opt, P_opt, params, env);
EE_opt = SR_opt / sum(P_opt);
fprintf('PSCA finished in %d iters. EE=%.6g (SR=%.6g, sumP=%.6g)\n', ...
        info.iters, EE_opt, SR_opt, sum(P_opt));

% [pF_opt, P_opt, info] = psca_fmincon_ee(pF0, P0, env, params, opts);

%% Functions
function [pF_opt, P_opt, info] = psca_fmincon_ee(pF0, P0, env, params, opts)
% PSCA for maximizing energy efficiency EE = SR / sum(P)
% Uses first-order Taylor surrogates \tilde F_i of F = -EE and fmincon.

V = numel(P0);
pF = pF0(:)';  P  = P0(:);

% ---- thresholds / defaults ----
if isfield(opts,'rho_th_dB')
    rho_th = 10^(opts.rho_th_dB/10);
elseif isfield(opts,'rho_th')
    rho_th = opts.rho_th;
else
    rho_th = 1; % 0 dB
end
if ~isfield(opts,'Pmin'); opts.Pmin = 1e-2*ones(V,1); end
if ~isfield(opts,'Pmax'); opts.Pmax = 2.0  *ones(V,1); end
if ~isfield(opts,'S_box'); opts.S_box = [0 1000; 0 1000; 50 300]; end
if ~isfield(opts,'gamma0'); opts.gamma0 = 0.7; end
if ~isfield(opts,'gamma_decay'); opts.gamma_decay = 0.995; end
if ~isfield(opts,'maxIter'); opts.maxIter = 150; end
if ~isfield(opts,'tol'); opts.tol = 1e-4; end
if ~isfield(opts,'fd_eps'); opts.fd_eps = 1e-2; end

% fmincon options (override anything you like)
fmopts = struct( ...
    'Algorithm', 'interior-point', ...   % 'sqp' or 'interior-point'
    'Display', 'iter-detailed', ...      % 'off' | 'final' | 'iter' | 'iter-detailed'
    'MaxIterations', 1000, ...             % max solver iterations per subproblem
    'MaxFunctionEvaluations', 500, ...   % cap evals (surrogate is affine so this stays low)
    'OptimalityTolerance', 1e-10, ...    % KKT stationarity tolerance
    'StepTolerance', 1e-12, ...          % step-size tolerance
    'ConstraintTolerance', 1e-10, ...    % feas. tolerance (mostly moot here since we use bounds)
    'SpecifyObjectiveGradient', true ... % we provide the gradient
    ... % 'FiniteDifferenceType','forward' % (not used since we give gradients)
    ... % 'UseParallel', true              % (only helpful if you add FD gradients)
);


histEE = zeros(opts.maxIter,1);
histSR = zeros(opts.maxIter,1);
histP  = zeros(opts.maxIter,1);
gamma  = opts.gamma0;

for it = 1:opts.maxIter
    % === evaluate objective + model terms once ===
    [SR, ~, details] = objective_static_SR_A2G(pF, P, params, env);
    EE  = SR / sum(P);
    Fl  = -EE;                                  % F(x^l) = -EE

    % grad wrt p_F via finite differences on EE
    gEE_pF = grad_EE_pF(pF, P, params, env, opts.fd_eps);  % 1x3
    gF_pF  = -gEE_pF(:);                                   % ∇F = -∇EE

    % grad wrt each P_v (analytic)
    gF_P = zeros(V,1);
    for v = 1:V
        gEE_Pv  = grad_EE_Pv(v, pF, P, params, env, SR, details);
        gF_P(v) = -gEE_Pv;
    end

    % === BLOCK 1: position (box only) ===
    obj1   = @(p) surrogate_affine(Fl, gF_pF, pF, p);  % ˜F_1(p|x^l)
    LB1    = opts.S_box(:,1)'; 
    UB1    = opts.S_box(:,2)';
    pF_hat = fmincon(obj1, pF, [],[],[],[], LB1, UB1, [], fmopts);

    % === BLOCKS 2..V+1: power (bounds + SNR for gateway v) ===
    P_hat = P;
    for v = 1:V
        Gv      = details.G(v);
        Psnr_lb = rho_th * params.sigma2_W / max(Gv, eps);   % from ρ_v ≥ ρ_th
        LBv     = max(opts.Pmin(v), Psnr_lb);
        UBv     = opts.Pmax(v);
        objv    = @(pv) surrogate_affine(Fl, gF_P(v), P(v), pv); % scalar surrogate
        P_hat(v)= fmincon(objv, P(v), [],[],[],[], LBv, UBv, [], fmopts);
    end

    % === PSCA relaxed update ===
    pF_new = pF + gamma*(pF_hat - pF);
    P_new  = P  + gamma*(P_hat  - P);

    % === termination bookkeeping ===
    [SR_new, ~] = objective_static_SR_A2G(pF_new, P_new, params, env);
    EE_new = SR_new / sum(P_new);
    histEE(it) = EE_new; histSR(it) = SR_new; histP(it) = sum(P_new);

    if it > 5 && abs(EE_new-EE)/max(1,abs(EE)) < opts.tol
        pF = pF_new; P = P_new; break;
    end
    pF = pF_new; P = P_new;
    gamma = max(0.2, gamma*opts.gamma_decay);
end

pF_opt = pF;  P_opt = P;
info.iters = it;
info.hist.EE  = histEE(1:it);
info.hist.SR  = histSR(1:it);
info.hist.Psum= histP(1:it);
end

% ---------- affine surrogate (first-order Taylor) ----------
function [f, g] = surrogate_affine(F0, gF, x0, x)
f = F0 + gF(:)'*(x(:) - x0(:));
g = gF(:)';     % constant gradient for fmincon
end

% ---------- grad EE wrt position by forward finite differences ----------
function g = grad_EE_pF(pF, P, params, env, h)
[SR0, ~] = objective_static_SR_A2G(pF, P, params, env);
EE0 = SR0 / sum(P);
g = zeros(1,3);
for k = 1:3
    pp = pF; pp(k) = pp(k) + h;
    [SRp, ~] = objective_static_SR_A2G(pp, P, params, env);
    EEp = SRp / sum(P);
    g(k) = (EEp - EE0)/h;
end
end

% ---------- exact grad EE wrt P_v (pF fixed) ----------
function dEE = grad_EE_Pv(v, pF, P, params, env, SR, details)
Gv    = details.G(v);
sigma2= params.sigma2_W;
Wv    = params.W_Hz(v);
rho_v = P(v)*Gv/sigma2;
dSR_dPv = (Wv/log(2)) * (Gv/sigma2) / (1 + rho_v); % only R_v depends on P_v
S = sum(P);
dEE = (dSR_dPv*S - SR) / (S^2);
end
