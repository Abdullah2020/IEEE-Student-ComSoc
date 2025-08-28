function [f_lin, f0, g] = taylor_approx_objective(x, x_k, params, env)
% TAYLOR1_SIMPLE
% First-order (linear) Taylor model of the scalar objective
%   f(x) = objective_energy_efficiency(x, params, env)
% around the expansion point x_k:
%   f(x) ≈ f(x_k) + ∇f(x_k)^T (x - x_k)
%
% -------------------------
% Decision vector structure
% -------------------------
% n = 3 + V, where V is the number of static gateways.
% x (and x_k) pack the UAV 3D position and the V gateway powers:
%   x(1)   = P_F_x    [m]   UAV x-position
%   x(2)   = P_F_y    [m]   UAV y-position
%   x(3)   = P_F_z    [m]   UAV altitude
%   x(4:3+V) = P_tx_W [W]   Transmit powers per static gateway (v = 1..V)
%
% Note: objective_energy_efficiency (as you defined it) returns the
% NEGATIVE energy efficiency (i.e., f = -EE) so that minimizers can be used.
%
% -----------
% Inputs
% -----------
% x      : Query/evaluation point where you want the linearized value f_lin.
% x_k    : Expansion point where the Taylor model is built (same length as x).
% params : Model parameters used internally by the rate function, e.g.:
%          alpha, lambda, eta_LoS_dB, eta_NLoS_dB, f_Hz, c, sigma2_W, W_Hz (Vx1), ...
% env    : Environment struct describing geometry:
%          env.PV   (V x 3)  Static gateway coordinates [m], z = h_SG
%          env.Piv{v} (N_v x 3) End-device coordinates in cluster U_v [m]
%          env.Uv_R (V x 1)  Cluster radii [m]
%          env.h_SG (scalar) Gateway mast height [m]
%
% -----------
% Outputs
% -----------
% f_lin : First-order Taylor approximation of f(x) about x_k
% f0    : Base objective value at x_k, i.e., f0 = f(x_k) = objective_energy_efficiency(x_k,...)
% g     : Finite-difference gradient ∇f(x_k) (size n x 1)
%
% -------------------------
% Implementation variables
% -------------------------
% V       : Number of static gateways = size(env.PV,1)
% n       : Length of the decision vector (n = 3 + V)
% h (n x 1): Per-coordinate finite-difference steps:
%            h(j) = 1e-6 * max(1, |x_k(j)|), then clamped to >= eps
%            (relative steps for scale-robustness across meters vs Watts)
% hj      : The step for coordinate j (scalar)
% isPower : Logical flag for power variables (indices 4 .. 3+V)
% x_p/x_m : Perturbation points for central difference
% x_f     : Perturbation point for forward difference (used near P_tx >= 0 bound)
% f_p/f_m/f_f : Objective evaluated at the perturbed points
%
% Gradient scheme:
%   - Central difference by default: g(j) = [f(x_k + h_j e_j) - f(x_k - h_j e_j)] / (2 h_j)
%   - Forward difference for power coordinates when x_k(j) - h_j < 0
%     to keep the probe feasible (P_tx_W >= 0):
%       g(j) = [f(x_k + h_j e_j) - f(x_k)] / h_j
%
% Final linear model:
%   f_lin = f0 + g.' * (x - x_k)

    % --- Shape checks & basic sizes ---
    x   = x(:);                      % ensure column
    x_k = x_k(:);                    % ensure column
    V = size(env.PV, 1);             % number of static gateways
    n = numel(x_k);                  % decision dimension = 3 + V
    assert(n == 3+V, 'x/x_k must be length 3+V');

    % --- 1) Finite-difference step sizes (tiny & relative) ---
    % Scale by variable magnitude to be robust across units (meters vs Watts)
    h = 1e-6 * max(1, abs(x_k));
    h = max(h, eps);                 % avoid zero steps

    % --- 2) Base objective value at expansion point ---
    % f0 is the NEGATIVE EE at x_k (because your objective returns -EE)
    f0 = objective_energy_efficiency(x_k, params, env);

    % --- 3) Gradient at x_k ---
    % Central difference by default; switch to forward near P_tx >= 0 boundary.
    g = zeros(n,1);
    for j = 1:n
        hj = h(j);
        isPower = (j >= 4) && (j <= 3+V);  % indices of P_tx_W entries

        if isPower && (x_k(j) - hj < 0)
            % Forward difference to keep P_tx_W >= 0 (feasible probe)
            x_f = x_k; x_f(j) = x_f(j) + hj;
            f_f = objective_energy_efficiency(x_f, params, env);
            g(j) = (f_f - f0) / hj;
        else
            % Central difference (more accurate when feasible)
            x_p = x_k; x_p(j) = x_p(j) + hj;
            x_m = x_k; x_m(j) = x_m(j) - hj;
            f_p = objective_energy_efficiency(x_p, params, env);
            f_m = objective_energy_efficiency(x_m, params, env);
            g(j) = (f_p - f_m) / (2*hj);
        end
    end

    % --- 4) First-order Taylor model at query point x ---
    f_lin = f0 + g.' * (x - x_k);
end
