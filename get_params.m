function params = get_params()
    % Parameters for ADMM Trajectory Planner
    
    params.safe_distance = 0.5;
    params.horizon      = 8;
    params.dt           = 0.1;
    params.max_iter     = 200;
    params.rho          = 1.5;
    params.tol          = 1e-2;

    % Car parameters
    params.wheelbase  = 2.5;           % meters
    params.max_steer  = deg2rad(30);   % radians
    params.max_speed  = 5.0;           % m/s
    params.L          = 4.7;           % length
    params.W          = 1.8;           % width

    % Obstacle parameters
    params.obstacle.inflation      = 1.2;  % enlarge size for safety
    params.obstacle.safety_margin  = 0.8; 

    % Cost Weights
    params.cost.goal      = 10.0;  % goal tracking importance
    params.cost.control   = 0.1;   % control effort penalty
    params.cost.collision = 5.0;   % obstacle avoidance penalty

    % Bicycle Update parameters
    params.alpha_u     = 0.1;    % learning rate for control
    params.alpha_x     = 0.01;   % learning rate for state
    params.max_acc     = 2.0;    % max acceleration
    params.max_steer   = deg2rad(30); % max steering angle (rad)

end
