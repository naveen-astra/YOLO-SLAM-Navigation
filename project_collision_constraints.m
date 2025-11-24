%% Collision Projection
function Z = project_collision_constraints(X, Lambda, obstacles, params)
    Z = X(:,1:2) + Lambda;  % [x, z] positions
    for k = 1:size(Z,1)
        for o = 1:size(obstacles,1)
            obs_pos = obstacles(o,1:2);  % [x, z]
            obs_size = obstacles(o,3:4) + params.obstacle.inflation;  % [width, depth]

            % Axis-aligned bounding box check
            dx = Z(k,1) - obs_pos(1);
            dz = Z(k,2) - obs_pos(2);  % <-- changed dy -> dz

            if abs(dx) < obs_size(1)/2 && abs(dz) < obs_size(2)/2
                % Push out along minimum penetration
                pen_x = obs_size(1)/2 - abs(dx);
                pen_z = obs_size(2)/2 - abs(dz);

                if pen_x < pen_z
                    Z(k,1) = obs_pos(1) + sign(dx)*(obs_size(1)/2 + params.obstacle.safety_margin);
                else
                    Z(k,2) = obs_pos(2) + sign(dz)*(obs_size(2)/2 + params.obstacle.safety_margin);
                end
            end
        end
    end
end
