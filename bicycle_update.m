function [x_next, u_new] = bicycle_update(x, u, goal, obstacles, params)
    % x: [x, z, theta, v]
    % u: [acceleration, steering]
    % goal: [x, z]
    % obstacles: Nx2 positions [x, z]

    dt = 0.1;
    L = deg2rad(30);
    safe_dist = 0.4;

    % Attractive Gradient towards Goal
    dx = goal(1) - x(1);  % x-direction
    dz = goal(2) - x(2);  % z-direction

    dist_to_goal = norm([dx, dz]);
    desired_theta = atan2(dz, dx);

    grad_goal_steer = -2 * wrapToPi(x(3) - desired_theta);
    grad_goal_acc   = -2 * (params.max_speed - x(4));

    % Repulsive Gradient from Obstacles
    grad_obs_steer = 0;
    grad_obs_acc   = 0;

    for i = 1:size(obstacles,1)
        obs_dx = obstacles(i,1) - x(1);
        obs_dz = obstacles(i,2) - x(2);
        obs_dist = sqrt(obs_dx^2 + obs_dz^2);

        if obs_dist < safe_dist
            repulsion = (safe_dist - obs_dist) / (obs_dist^2 + 1e-6);
            angle_to_obs = atan2(obs_dz, obs_dx);
            steer_away = wrapToPi(x(3) - angle_to_obs);

            grad_obs_steer = grad_obs_steer + repulsion * sign(steer_away);
            grad_obs_acc   = grad_obs_acc - repulsion;
        end
    end

    % Total Gradient
    grad_u = [grad_goal_acc + grad_obs_acc, grad_goal_steer + grad_obs_steer];

    % Control Update
    u_new = u + params.alpha_u * grad_u;
    u_new(1) = min(max(u_new(1), -params.max_acc), params.max_acc);
    u_new(2) = min(max(u_new(2), -params.max_steer), params.max_steer);

    % State Update using Bicycle Model
    x_next = zeros(1,4);
    x_next(1) = x(1) + x(4) * cos(x(3)) * dt;
    x_next(2) = x(2) + x(4) * sin(x(3)) * dt;
    x_next(3) = x(3) + (x(4)/L) * tan(u_new(2)) * dt;
    x_next(4) = x(4) + u_new(1) * dt;

end
