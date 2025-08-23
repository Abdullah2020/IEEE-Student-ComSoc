function [SR, R_v, details] = objective_static_SR_A2G(P_F, P_tx_W, params, env)
% OBJECTIVE_STATIC_SR_A2G  Total A2G network sum rate for a flying GW at P_F.
%
% Inputs:
%   env.PV      : Vx3 static gateway positions p_v (meters), z = h_SG
%   P_F         : 1x3 flying gateway position [x y z] (meters)
%   P_tx_W      : Vx1 transmit power per SGW v (W)
%   params      : struct with REQUIRED fields:
%                   .alpha       % LoS model parameter (scalar)
%                   .lambda      % LoS model parameter (scalar)
%                   .eta_LoS_dB  % Excess loss for LoS   (dB)
%                   .eta_NLoS_dB % Excess loss for NLoS  (dB)
%                   .f_Hz        % Carrier frequency (Hz)
%                   .c           % Speed of light (m/s)
%                   .sigma2_W    % Noise power at receiver (W)
%                   .W_Hz        % Vx1 bandwidth per SGW v (Hz)
%
% Outputs:
%   SR      : scalar total sum rate (bps)
%   R_v     : Vx1 achievable rate per SGW v (bps)           [optional]
%   details : struct with fields: d, phi_deg, P_LoS, L_FS_dB,
%             L_A2G_dB, G, rho                               [optional]

    % ---------------- Input checks ----------------
    PV = env.PV;                        % Vx3
    V  = size(PV,1);

    % Required parameter fields
    req = {'alpha','lambda','eta_LoS_dB','eta_NLoS_dB','f_Hz','c','sigma2_W','W_Hz'};
    for k = 1:numel(req)
        assert(isfield(params, req{k}), 'params.%s is required', req{k});
    end

    % Check dimensions
    assert(all(size(P_tx_W) == [V,1]), 'P_tx_W must be Vx1');
    assert(all(size(params.W_Hz)   == [V,1]), 'params.W_Hz must be Vx1');
    assert(isscalar(params.sigma2_W), 'params.sigma2_W must be scalar');

    % ---------------- Geometry ----------------
    d_xy   = vecnorm(PV(:,1:2) - P_F(1:2), 2, 2);  % horizontal distance
    d_z    =  P_F(3) - PV(:,3);                    % vertical offset
    d      = sqrt(d_xy.^2 + d_z.^2);               % 3D distance (m)
    phiDeg = atan2d(d_z, d_xy);                    % elevation angle

    % ---------------- LoS probability ----------------
    alpha  = params.alpha;
    lambda = params.lambda;
    P_LoS  = 1 ./ (1 + alpha .* exp(-lambda .* (phiDeg - alpha)));
    P_NLoS = 1 - P_LoS;

    % ---------------- Path loss ----------------
    L_FS_dB  = 20*log10(4*pi*params.f_Hz.*d./params.c); % Free-space loss
    L_A2G_dB = L_FS_dB + params.eta_LoS_dB .* P_LoS + params.eta_NLoS_dB .* P_NLoS;

    % Channel gain
    G   = 10.^(-L_A2G_dB/10);

    % ---------------- SNR & Rate ----------------
    rho = (P_tx_W .* G) ./ params.sigma2_W;        % SNR per v
    R_v = params.W_Hz .* log2(1 + rho);            % Achievable rate (bps)
    SR  = sum(R_v);

    % Optional details
    if nargout >= 3
        details = struct('d', d, 'phi_deg', phiDeg, 'P_LoS', P_LoS, ...
                         'L_FS_dB', L_FS_dB, 'L_A2G_dB', L_A2G_dB, ...
                         'G', G, 'rho', rho);
    end
end
