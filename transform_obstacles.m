function global_obs = transform_obstacles(obs_data, current_state)
    % Transform obstacles from local (ego) to global frame
    % Output: [x, z, width, depth]

    global_obs = zeros(length(obs_data), 4); 
    % Rotation matrix based on ego heading
    R = [cos(current_state(3)), -sin(current_state(3));
         sin(current_state(3)),  cos(current_state(3))];

    for i = 1:length(obs_data)
        local_pos  = obs_data(i).position; % [x_local, z_local]
        size_      = obs_data(i).size;     % [length, width]

        global_pos = current_state(1:2) + (R * local_pos')';

        global_obs(i,:) = [global_pos, size_];
    end
end
