function cost = collision_penalty(pos, obstacles, params)
    % pos : current [x, y] position of ego
    % obstacles : list of obstacles [x, y, w, h]
    % params : planner parameters

    cost = 0;
    
    for i = 1:size(obstacles,1)
        obs_pos = obstacles(i,1:2);
        obs_size = obstacles(i,3:4) / 2; % half-size for bounding box check
        
        % Compute distance to obstacle boundary
        dx = max(abs(pos(1) - obs_pos(1)) - obs_size(1), 0);
        dy = max(abs(pos(2) - obs_pos(2)) - obs_size(2), 0);
        
        dist = sqrt(dx^2 + dy^2);
        
        % Penalty if within safe distance
        if dist < params.safe_distance
            cost = cost + (params.safe_distance - dist)^2;
        end
    end
end
