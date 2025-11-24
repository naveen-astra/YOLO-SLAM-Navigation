%% Trajectory Update (Subproblem)
function X = update_trajectory(X, U, Z, Lambda, goal, obstacles, params)
    Q = params.cost.goal * eye(2);
    R = params.cost.control * eye(2);

    for k = 1:params.horizon
        % Error to ADMM variables
        err_admm = X(k,1:2) - Z(k,:) + Lambda(k,:);
        cost_admm = err_admm * Q * err_admm';

        % Error to Goal
        err_goal = X(k,1:2) - goal;
        cost_goal = err_goal * Q * err_goal';

        % Control Cost
        cost_control = U(k,:) * R * U(k,:)';

        % Obstacle Penalty
        cost_coll = collision_penalty(X(k,1:2), obstacles, params);

        % Total Cost
        total_cost = cost_admm + cost_goal + cost_control + params.cost.collision * cost_coll;

        % Update States & Controls
        [X(k+1,:), U(k,:)] = bicycle_update(X(k,:), U(k,:), goal, obstacles, params);
    end
end

