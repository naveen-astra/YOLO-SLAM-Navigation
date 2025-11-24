function obstacles = get_obstacles(data_subset)
    % Extract obstacle information from KITTI/ORB-SLAM3 data table
    % Input: data_subset - Table containing obstacle data for current timestamp
    % Output: obstacles - Array of structs with obstacle properties
    
    obstacles = struct(...
        'position', {}, ...    % [X, Z] in egocentric coordinates (KITTI frame)
        'size', {}, ...        % [Width, Depth] of obstacle
        'velocity', {} ...     % Obstacle velocity (m/s)
    );
    
    for i = 1:height(data_subset)
        obstacles(i).position = [data_subset.X_3D(i), data_subset.Z_3D(i)]; % KITTI X-Z plane
        obstacles(i).size = [data_subset.Width_3D(i), data_subset.Depth(i)];
        obstacles(i).velocity = data_subset.Velocity(i);
    end
end