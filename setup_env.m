function env = setup_env(PV_in, R, N_per_cluster, h_SG)
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

    % ----------- input checks -----------
    if nargin ~= 4
        error('setup_env requires 4 inputs: PV_in, R, N_per_cluster, h_SG.');
    end
    if size(PV_in,2) ~= 2
        error('PV_in must be Vx2 (x,y).');
    end
    V = size(PV_in,1);
    if ~isequal(size(R), [V,1])
        error('R must be Vx1 to match the number of gateways.');
    end
    if ~isequal(size(N_per_cluster), [V,1])
        error('N_per_cluster must be Vx1 to match the number of gateways.');
    end
    if ~isscalar(h_SG)
        error('h_SG must be a scalar (same height for all gateways).');
    end

    % ----------- construct PV in 3D (z = h_SG) -----------
    PV = [PV_in, h_SG*ones(V,1)];   % p_v

    % ----------- generate ED clusters on ground plane (z = 0) -----------
    Piv = cell(1,V);
    for v = 1:V
        Nv = N_per_cluster(v);
        rv = R(v) * sqrt(rand(Nv,1));   % uniform-in-disk
        th = 2*pi*rand(Nv,1);
        Xi = PV_in(v,1) + rv .* cos(th);
        Yi = PV_in(v,2) + rv .* sin(th);
        Zi = zeros(Nv,1);               % ground level
        Piv{v} = [Xi, Yi, Zi];          % p_i^v
    end

    % ----------- pack output -----------
    env = struct();
    env.PV   = PV;          % gateways at z = h_SG
    env.Uv_R = R(:);
    env.Piv  = Piv;         % EDs at z = 0
    env.h_SG = h_SG;        % scalar
end
