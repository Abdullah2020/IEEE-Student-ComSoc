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
env = setup_env(PV_in, R, N_per_cluster, h_SG);


hfig = plot_env3d(env, 'ZScale',1.5);




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
h=100;

% Example: above middle of gateways
P_F = [500, 500, h];   % [x y z] in meters

% Transmit power (W) per gateway
P_tx_W = [0.5; 0.5];   % 500mW=0.5W each (27dBm)

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

%% ===== 4. Call objective function =====
[SR, R_v] = objective_static_SR_A2G(P_F, P_tx_W, params, env);

fprintf('Total sum rate: %.3f Mbps\n', SR/1e6);

x = [P_F'; P_tx_W];
neg_EE = objective_energy_efficiency(x, params, env);

fprintf('Total EE: %.3f bits/J\n', -neg_EE);

