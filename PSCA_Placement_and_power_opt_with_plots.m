
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
opts.maxIter    = 10000;
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

% ===== 5) Run PSCA =====
N_dis_Broad = 1000;
P_MAX_UAV=1;
[P_UAV_opt, EE_opt, hist] = maximize_EE_over_PUAV(pF_opt, P_MAX_UAV, params, env, N_dis_Broad);

fprintf('Best P_UAV = %.6g W | Max EE = %.6g\n', P_UAV_opt, EE_opt);

% Plots
plot_EE_optimization_history(hist);


%% ===== 5) Plots =====

%=== Top-level plotting ===
hfig = plot_env3d(env, 'ZScale', 5, 'ShowLabels', true); %#ok<NASGU>
hold on;

% UAV markers
scatter3(P_F_0(1), P_F_0(2), P_F_0(3), 120, 'ko', 'filled', 'DisplayName','UAV init');
scatter3(pF_opt(1), pF_opt(2), pF_opt(3), 140, 'r^', 'filled', 'DisplayName','UAV opt');

% Removed the transition/update line:
% plot3([P_F_0(1) pF_opt(1)], [P_F_0(2) pF_opt(2)], [P_F_0(3) pF_opt(3)], ...
%       'k--', 'LineWidth', 1.2, 'DisplayName','update');

zl = zlim; 
zlim([0, max([zl(2), pF_opt(3)*1.2, max(env.PV(:,3))*1.2])]);
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

    histEE(it)   = EE_new;
    histSR(it)   = SR_new;
    histPsum(it) = sum(P_new);
    histP(it,:)  = P_new(:)';
    histRv(it,:) = Rv_new(:)';
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


%% ==========================
%  Objective (Sum Rate) Model
% ==========================
function [SR, R_v, details] = objective_static_SR_A2G(P_F, P_tx_W, params, env)
% Total A2G network sum rate for a flying GW at P_F.

PV = env.PV;                        % Vx3
V  = size(PV,1);

% checks
assert(all(size(P_tx_W) == [V,1] | size(P_tx_W) == [1,V]), 'P_tx_W must be V-length');
P_tx_W = P_tx_W(:); % Vx1
assert(all(size(params.W_Hz) == [V,1] | size(params.W_Hz) == [1,V]), 'params.W_Hz must be V-length');
W_Hz = params.W_Hz(:);

% geometry
d_xy   = vecnorm(PV(:,1:2) - P_F(1:2), 2, 2);  % horizontal distance
d_z    =  P_F(3) - PV(:,3);                    % vertical offset
d      = sqrt(d_xy.^2 + d_z.^2);               % 3D distance (m)
phiDeg = atan2d(d_z, d_xy);                    % elevation angle

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
% Returns env struct with:
%   PV  (Vx3): gateway positions (z = h_SG)
%   Uv_R(Vx1): cluster radii
%   Piv (1xV cell): ED positions (Ni x 3), z = 0
%   h_SG: mast height

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
p.addParameter('ShowLabels', true, @(x)islogical(x)&&isscalar(x));
p.addParameter('FaceAlpha', 0.07, @(x)isnumeric(x)&&isscalar(x));
p.addParameter('EdgeAlpha', 0.4,  @(x)isnumeric(x)&&isscalar(x));
p.addParameter('GWMarkerSize', 100, @(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('EDMarkerSize', 10,  @(x)isnumeric(x)&&isscalar(x)&&x>0);
p.addParameter('ZScale', 5, @(x)isnumeric(x)&&isscalar(x)&&x>0); % Magnification factor for z
p.addParameter('AutoXY', true,      @(x)islogical(x)&&isscalar(x));       % NEW
p.addParameter('XYPadding', 0.12,   @(x)isnumeric(x)&&isscalar(x)&&x>=0); % NEW (12% padding)
p.addParameter('LegendClean', true, @(x)islogical(x)&&isscalar(x));       % NEW

p.parse(varargin{:});
opt = p.Results;

PV   = env.PV;
R    = env.Uv_R(:);
Piv  = env.Piv;
V    = size(PV,1);

hfig = figure('Color','w'); hold on; grid on;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
view(45,25);
xlim([0 1000]);
ylim([0 1000]);
zlim([0 20]);  % base z-range (we will override after plotting UAV)
pbaspect([1 1 opt.ZScale]); % stretch z-axis visually

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
          'LineWidth', 1.2);

    % EDs
    pts = Piv{v};
    scatter3(pts(:,1), pts(:,2), pts(:,3), opt.EDMarkerSize, ...
             'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k', ...
             'MarkerFaceAlpha', 0.9, 'MarkerEdgeAlpha', 0.4);

    % Gateway mast
    plot3([cx cx], [cy cy], [0 PV(v,3)], '--', 'Color', cmap(v,:), 'LineWidth', 1.0);
    scatter3(PV(v,1), PV(v,2), PV(v,3), opt.GWMarkerSize, ...
             's', 'filled', 'MarkerFaceColor', cmap(v,:), 'MarkerEdgeColor', 'k');

    if opt.ShowLabels
        text(PV(v,1), PV(v,2), PV(v,3), sprintf('  p_{%d}', v), ...
             'FontWeight','bold', 'VerticalAlignment','bottom');
    end
end

% Ground plane outline
allPts = [cell2mat(Piv(:)); PV];
minxy = min(allPts(:,1:2), [], 1) - max(R);
maxxy = max(allPts(:,1:2), [], 1) + max(R);
plot3([minxy(1) maxxy(1) maxxy(1) minxy(1) minxy(1)], ...
      [minxy(2) minxy(2) maxxy(2) maxxy(2) minxy(2)], ...
      zeros(1,5), 'k:', 'LineWidth', 1.0);

title('Gateways p_v (z=h_{SG}), clusters \mathcal{U}_v, and EDs p_i^v');
end
