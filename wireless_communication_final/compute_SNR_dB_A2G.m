
%% ==========================
%  Compute Users SNR
% ==========================
function [rho] = compute_SNR_dB_A2G(P_F, P_tx_W, params, env)
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
SNR=(P_tx_W .* G) ./ params.sigma2_W;

rho = 10*log10 ( SNR);% SNR in dB


end
