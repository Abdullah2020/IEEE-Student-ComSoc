function f = objective_energy_efficiency_broadcasting(P_F_opt,P_UAV, params, env)

    V = size(env.PV,1);
    
    % parse decision vector

    P_tx_W = P_UAV*ones(V,1);


    [SR, ~] = objective_static_SR_A2G(P_F_opt, P_tx_W, params, env);

    EE = SR / P_UAV;

    % minimize negative EE
    f = EE;
end

