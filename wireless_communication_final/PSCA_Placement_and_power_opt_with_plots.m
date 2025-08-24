
clc; clear; close all;

%% ==========================
%  Main Script to Setup Env
%  ==========================

% ----- Input Parameters -----
PV_in = [250  250;     % Gateway 1 (x,y)
         700  700;
         0 0;
         50 800];    % Gateway x
R = 90 * ones(size(PV_in,1));         % cluster radii (m)
N_per_cluster = 30 * ones(size(PV_in,1)); %[30; 30];
h_SG = 15;             % gateway mast height (m)

% ====== Create environment ======
env = setup_env(PV_in, R, N_per_cluster, h_SG);
PV  = env.PV;          %#ok<NASGU> % (V x 3) gateway positions
Piv_all = env.Piv;     %#ok<NASGU>

%% Communication section
% ===== 1) Initial flying gateway position & power =====
%h = 60;                           % initial altitude (m)
%P_F_0 = [400, 400, h];            % initial UAV position [x y z] (m)

P_F_0(1)=0+(1000-0)*rand(1,1);
P_F_0(2)=0+(1000-0)*rand(1,1);
P_F_0(3)=30+(32-30)*rand(1,1);

P_tx_W_0 = 0.5 * rand(1, size(PV_in,1)); % initial powers (row ok) -> will be vectorized

% ===== 2) A2G parameters =====
params.alpha       = 4.88;
params.lambda      = 0.43;
params.eta_LoS_dB  = 0.1;
params.eta_NLoS_dB = 21.0;
params.f_Hz        = 868e6;
params.c           = 3e8;
sigma2_dBm         = -90;
params.sigma2_W    = 10^((sigma2_dBm-30)/10);
params.W_Hz        = 125e3 * ones(size(PV_in,1),1);  % per-GW bandwidth (Vx1)

% ===== 3) PSCA options =====
opts.S_box      = [0 1000; 0 1000; 30 70];   % block x1 (UAV position) box
opts.Pmin       = 0.00 * ones(numel(P_tx_W_0),1);
opts.Pmax       = 0.50 * ones(numel(P_tx_W_0),1);
opts.rho_th_dB  = -7.5;                      % SNR threshold (dB) -> converted inside
opts.gamma0     = 0.001;
opts.gamma_decay= 1e-6;
opts.maxIter    = 11000;
opts.tol        = 1e-6;
opts.fd_eps     = 1e-2;

% fmincon options (can tweak here)
opts.fmopts = struct( ...
    'Algorithm','interior-point', ...
    'Display','iter', ...           % 'off'|'final'|'iter'|'iter-detailed'
    'MaxIterations', 1000, ...
    'MaxFunctionEvaluations', 500, ...
    'OptimalityTolerance', 1e-12, ...
    'StepTolerance', 1e-12, ...
    'ConstraintTolerance', 1e-10, ...
    'SpecifyObjectiveGradient', true ...
);

% ===== 4) Run PSCA =====
[pF_opt, P_opt, info] = psca_fmincon_ee(P_F_0, P_tx_W_0, env, params, opts);

[SR_opt, ~] = objective_static_SR_A2G(pF_opt, P_opt, params, env);
EE_opt = SR_opt / sum(P_opt);
fprintf('PSCA finished in %d iters. EE=%.6g (SR=%.6g, sumP=%.6g)\n', ...
        info.iters, EE_opt, SR_opt, sum(P_opt));

%% ===== 5) Run Broadcasting Power Optimization (policy: keep last best-EE feasible) =====
% User params
N_dis_Broad   = 1000;
P_MAX_UAV     = 1;
SNR_THRESH_dB = -5;     % seuil SNR (dB) pour TOUS les SGW

% Run selection
[P_UAV_opt, EE_opt, hist] = select_power_with_snr_and_bestEE_policy( ...
    pF_opt, P_MAX_UAV, params, env, N_dis_Broad, SNR_THRESH_dB);

fprintf('Selected P_UAV = %.6g W | EE(selected) = %.6g (SNR thresh = %.2f dB)\n', ...
    P_UAV_opt, EE_opt, SNR_THRESH_dB);

% === Only the two requested plots ===
plot_policy_EE_and_SNR(hist);

%% ===== 5) Plots =====

%==================== MAIN SCRIPT ====================%
% Assumes: env (with fields PV, Uv_R, Piv), P_F_0, pF_opt are defined.

% Plot environment with softened Z scale and auto-fit XY

hfig = plot_env3d(env, 'ZScale', 2, 'ShowLabels', true, 'AutoFit', true, 'XYPadding', 120); %#ok<NASGU>
hold on;

% UAV markers (kept in legend)
scatter3(P_F_0(1), P_F_0(2), P_F_0(3), 120, 'ko', 'filled', 'DisplayName','UAV init');
scatter3(pF_opt(1), pF_opt(2), pF_opt(3), 140, 'r^', 'filled', 'DisplayName','UAV opt');

% (Removed the transition/update line)

% Ensure Z limit comfortably includes UAV and SGWs
zl = zlim;
zmax = max([zl(2), pF_opt(3)*1.2, max(env.PV(:,3))*1.2]);
zlim([0, zmax]);

legend('Location','best'); 
title('Environment and UAV optimal placement');
grid on; hold off;



iters = 1:info.iters;

% (b) Objective (energy efficiency) evolution
figure('Color','w'); 
plot(iters, info.hist.EE, 'LineWidth', 2);
xlabel('Iteration'); ylabel('Energy efficiency  [bps/W]');
title('PSCA Objective Evolution'); 
grid on;

% (c) Power evolution per gateway
figure('Color','w'); 
plot(iters, info.hist.P, 'LineWidth', 1.8);
xlabel('Iteration'); ylabel('Transmit power  [W]');
legend(arrayfun(@(v)sprintf('P_%d',v), 1:size(info.hist.P,2), 'UniformOutput', false), ...
       'Location','best');
title('Power Evolution per Gateway'); grid on;

% (d) Rate evolution (total + per gateway)
figure('Color','w'); 
plot(iters, info.hist.SR, 'k-', 'LineWidth', 2); hold on;
plot(iters, info.hist.Rv, '--', 'LineWidth', 1.5);
lgd = [{'Total SR'}, arrayfun(@(v)sprintf('R_%d',v), 1:size(info.hist.Rv,2), 'UniformOutput', false)];
legend(lgd, 'Location','best');
xlabel('Iteration'); ylabel('Rate  [bps]');
title('Rate Evolution'); grid on; hold off;


% (e) SNR evolution 
figure('Color','w'); 

plot(iters, info.hist.SNR, '-', 'LineWidth', 1.5);
hold on;
h = yline(opts.rho_th_dB , 'k--', 'LineWidth', 1.5);  % horizontal line with legend entry

lgd = [arrayfun(@(v)sprintf('SNR_%d',v), 1:size(info.hist.SNR,2), 'UniformOutput', false),'SNR Threshold of UAV'];

legend(lgd, 'Location','best');
xlabel('Iteration'); ylabel('SNR');
title('SNR Evolution'); grid on; hold off;

 % ===== end main =====


%% ================================
%  PSCA with fmincon (surrogates)
% =================================
function [pF_opt, P_opt, info] = psca_fmincon_ee(pF0, P0, env, params, opts)
% PSCA for maximizing energy efficiency EE = SR / sum(P)
% Blocks:
%   x1 = p_F ∈ S (box)
%   x_{v+1} = P_v ∈ [Pmin,Pmax], and ρ_v ≥ ρ_th (only for that v)
% Surrogate per block: first-order Taylor model of F = -EE, solved by fmincon.

V  = numel(P0);
pF = pF0(:)'; 
P  = P0(:);             % ensure column
assert(all(size(P,1) == V), 'P dimension mismatch');

% thresholds / defaults
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

% fmincon options
fmopts = build_fmincon_opts(opts);

% histories
histEE   = nan(opts.maxIter,1);
histSR   = nan(opts.maxIter,1);
histPsum = nan(opts.maxIter,1);
histP    = nan(opts.maxIter,V);
histRv   = nan(opts.maxIter,V);
histSNR   = nan(opts.maxIter,V);
histpF   = nan(opts.maxIter,3);

gamma = opts.gamma0;

for it = 1:opts.maxIter
    % evaluate model at x^l
    [SR, ~, details] = objective_static_SR_A2G(pF, P, params, env);
    EE = SR / sum(P);  Fl = -EE;

    % gradients
    gEE_pF = grad_EE_pF(pF, P, params, env, opts.fd_eps);  % 1x3
    gF_pF  = -gEE_pF(:);                                   
    gF_P   = zeros(V,1);
    for v = 1:V
        gEE_Pv  = grad_EE_Pv(v, pF, P, params, env, SR, details);
        gF_P(v) = -gEE_Pv;
    end

    % block x1: position (box only)
    obj1   = @(p) surrogate_affine(Fl, gF_pF, pF, p);
    LB1    = opts.S_box(:,1)'; 
    UB1    = opts.S_box(:,2)';
    pF_hat = fmincon(obj1, pF, [],[],[],[], LB1, UB1, [], fmopts);

    % blocks x_{v+1}: powers (bounds + SNR for gateway v)
    P_hat = P;
    for v = 1:V
        Gv      = details.G(v);
        Psnr_lb = rho_th * params.sigma2_W / max(Gv, realmin);   % ρ_v ≥ ρ_th
        LBv     = max(opts.Pmin(v), Psnr_lb);
        UBv     = opts.Pmax(v);
        objv    = @(pv) surrogate_affine(Fl, gF_P(v), P(v), pv);
        P_hat(v)= fmincon(objv, P(v), [],[],[],[], LBv, UBv, [], fmopts);
    end

    % relaxed update
    pF_new = pF + gamma*(pF_hat - pF);
    P_new  = P  + gamma*(P_hat  - P);

    % record & stopping
    [SR_new, Rv_new] = objective_static_SR_A2G(pF_new, P_new, params, env);
    EE_new = SR_new / sum(P_new);
    SNR_new = compute_SNR_dB_A2G(pF_new, P_new, params, env);

    histEE(it)   = EE_new;
    histSR(it)   = SR_new;
    histPsum(it) = sum(P_new);
    histP(it,:)  = P_new(:)';
    histRv(it,:) = Rv_new(:)';
    histSNR(it,:) = SNR_new(:)';
    histpF(it,:) = pF_new(:)';

    if it > 5 && abs(EE_new - EE)/max(1,abs(EE)) < opts.tol
        pF = pF_new; P = P_new; break;
    end
    pF = pF_new; P = P_new;
    % gamma = max(0.2, gamma*opts.gamma_decay);
    gamma =gamma*(1 - opts.gamma_decay * gamma);
end

pF_opt = pF;  P_opt = P;
info.iters      = it;
info.hist.EE    = histEE(1:it);
info.hist.SR    = histSR(1:it);
info.hist.SNR    = histSNR(1:it,:);
info.hist.Psum  = histPsum(1:it);
info.hist.P     = histP(1:it,:);
info.hist.Rv    = histRv(1:it,:);
info.hist.pF    = histpF(1:it,:);
end

% --- helper: build fmincon options from defaults + user overrides
function fmopts = build_fmincon_opts(opts)
% defaults
fmopts = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','off', ...
    'SpecifyObjectiveGradient',true, ...
    'MaxIterations', 50, ...
    'MaxFunctionEvaluations', 200, ...
    'OptimalityTolerance', 1e-9, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-9);
% user overrides from opts.fmopts (struct of fields)
if isfield(opts,'fmopts') && isstruct(opts.fmopts)
    fns = fieldnames(opts.fmopts);
    for k = 1:numel(fns)
        name = fns{k};
        try
            fmopts.(name) = opts.fmopts.(name);
        catch
            warning('fmincon option "%s" not recognized for this MATLAB version.', name);
        end
    end
end
end

% --- surrogate: first-order Taylor model of F at x^l (block-wise)
function [f, g] = surrogate_affine(F0, gF, x0, x)
f = F0 + gF(:)'*(x(:) - x0(:));
g = gF(:)';  % constant gradient for fmincon
end

% --- finite-difference grad of EE wrt pF (3D)
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

% --- exact grad of EE wrt P_v (pF fixed)
function dEE = grad_EE_Pv(v, pF, P, params, env, SR, details)
Gv    = details.G(v);
sigma2= params.sigma2_W;
Wv    = params.W_Hz(v);
rho_v = P(v)*Gv/sigma2;
dSR_dPv = (Wv/log(2)) * (Gv/sigma2) / (1 + rho_v); % only R_v depends on P_v
S = sum(P);
dEE = (dSR_dPv*S - SR) / (S^2);
end



