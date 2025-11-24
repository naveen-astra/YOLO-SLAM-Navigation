%% Main ADMM Path Planner (admm_planner.m)
function admm_planner()
clc; clear; close all;

params = get_params();

%% Load KITTI-ORBSLAM3 Data
data = readtable('slam_synced_output.csv');
timestamps = data.timestamp;

% Convert egocentric to global coordinates
global_poses = convert_to_global(data, params);

N = params.horizon;

% Unique poses for ego updates
[unique_t, idx] = unique(timestamps, 'stable');
unique_poses = global_poses(idx, :);

% Initial ego state
current_state = [unique_poses(1,1), unique_poses(1,3), 0, 1.5];

% Goal
goal = [unique_poses(end,1), unique_poses(end,3)];

% Trajectory storage
planned_traj = zeros(length(unique_t), 4);
actual_traj  = zeros(length(unique_t), 4);

pose_counter = 1; % to track unique ego updates

%% Main Loop
for k = 1:length(timestamps)-N

    % Ego State Update
    if pose_counter <= length(unique_t) && timestamps(k) == unique_t(pose_counter)
        current_state = [unique_poses(pose_counter,1), unique_poses(pose_counter,3), 0, 1.5];
        pose_counter = pose_counter + 1;
    end
    
    % Global pose used for obstacle transform
    global_state = [global_poses(k,1), global_poses(k,3), 0, 1.5];
    
    % Obstacle extraction and transform
    obs_data   = get_obstacles(data(data.timestamp == timestamps(k), :));
    global_obs = transform_obstacles(obs_data, global_state);

    % ADMM Optimization
    [X_opt, U_opt] = admm_solver(current_state, global_obs, goal, params);

    % Store planned trajectory
    planned_traj(k,:) = X_opt(1,:);
   

    % Update current state using the bicycle model
    current_state = bicycle_model(current_state, U_opt(1,:), params);

    % Store actual trajectory after applying control
    actual_traj(k,:) = current_state;
end

%% Plot Results
figure;
plot(planned_traj(:,1), planned_traj(:,2), 'b-', 'LineWidth', 2); hold on;
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('X [m]'); ylabel('Y [m]');
legend('Planned Path', 'Executed Path', 'Goal');
title('ADMM-Based Path Planning');
grid on;

end
