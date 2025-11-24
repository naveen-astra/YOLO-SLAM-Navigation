%% Core ADMM Solver
function [X, U] = admm_solver(x0, obstacles, goal, params)
    % Implementation based on:
    % [1] "Distributed Optimization and Statistical Learning via the Alternating Direction Method of Multipliers"
    %     Boyd et al., 2011
    % [2] "ADMM-based Trajectory Optimization for Autonomous Vehicles" 
    %     Liu et al., IV 2019
    
    N = params.horizon;
    X = repmat(x0, N+1, 1);
    U = zeros(N, 2); % [δ, a]
    Z = X(:,1:2);
    Lambda = zeros(N+1, 2);
    
    for iter = 1:params.max_iter
        % Trajectory Update
        X = update_trajectory(X, U, Z, Lambda, goal,obstacles, params);
        
        % Collision Projection
        Z = project_collision_constraints(X, Lambda, obstacles, params);
        
        % Dual Update
        Lambda = Lambda + (X(:,1:2) - Z);
        
        % Convergence Check
        if norm(X(:,1:2)-Z,'fro') < params.tol
            break;
        end
    end
end