clc; clear; close all;

params = get_params();

%% Load KITTI-ORBSLAM3 Data
data = readtable('slam_synced_output.csv');
timestamps = data.timestamp;

% Convert egocentric to global coordinates
global_poses = convert_to_global(data, params); % assumes [x, y, z] format

N = params.horizon;

% Unique ego poses (removing duplicate timestamps)
[unique_t, idx] = unique(timestamps, 'stable');
unique_poses = global_poses(idx, :); % [x, y, z]

% Initial ego state -> [x, z, theta, velocity]
current_state = [unique_poses(1,1), unique_poses(1,3), 0, 1.5];

% Goal in X-Z plane
goal = [unique_poses(end,1), unique_poses(end,3)];

% Store trajectories
planned_traj = zeros(length(unique_t), 4);
actual_traj  = zeros(length(unique_t), 4);

pose_counter = 1;

%% Main ADMM Loop
for k = 1:length(timestamps)-N

    % Update ego state only when timestamp matches unique_t
    if pose_counter <= length(unique_t) && timestamps(k) == unique_t(pose_counter)
        current_state = [unique_poses(pose_counter,1), unique_poses(pose_counter,3), 0, 1.5];
        pose_counter = pose_counter + 1;
    end
    
    % Global pose for transforming obstacles
    global_state = [global_poses(k,1), global_poses(k,3), 0, 1.5];

    % Extract & Transform Obstacles from local to global
    obs_data   = get_obstacles(data(data.timestamp == timestamps(k), :)); 
    global_obs = transform_obstacles(obs_data, global_state);

    % Run ADMM Solver
    [X_opt, U_opt] = admm_solver(current_state, global_obs, goal, params);

    % Store Trajectories
    planned_traj(k,:) = X_opt(1,:);
    actual_traj(k,:)  = current_state;

    % Update State
    current_state = X_opt(2,:);
    
    % Visualization
    visualize_scene(current_state, X_opt, global_obs, actual_traj(1:k,:), params);
end


