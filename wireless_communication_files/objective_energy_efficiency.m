function f = objective_energy_efficiency(x, params, env)
% x = [P_F(1:3), P_tx_W(1:V)]'
    V = size(env.PV,1);

    % parse decision vector
    P_F    = x(1:3).';
    P_tx_W = x(4:3+V);

    [SR, ~] = objective_static_SR_A2G(P_F, P_tx_W, params, env);
    Ptot = sum(P_tx_W);
    EE = SR / max(Ptot, eps);

    % minimize negative EE
    f = -EE;
end

