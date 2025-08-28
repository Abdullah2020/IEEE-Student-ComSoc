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


