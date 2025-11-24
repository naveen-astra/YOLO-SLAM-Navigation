function draw_vehicle(state, params)
    % Draw vehicle rectangle in X-Z plane
    % state = [x, z, theta, v]

    % Rotation matrix for heading (theta)
    R = [cos(state(3)), -sin(state(3));
         sin(state(3)),  cos(state(3))];
    
    % Vehicle corners (relative to center)
    corners = [-params.L/2, -params.W/2;
                params.L/2, -params.W/2;
                params.L/2,  params.W/2;
               -params.L/2,  params.W/2];
    
    % Transform to global X-Z coordinates
    global_corners = (R * corners')' + [state(1), state(2)]; % X-Z plane

    % Draw vehicle body
    fill(global_corners(:,1), global_corners(:,2), 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'k');

    % Draw heading direction line
    heading_end = [state(1) + cos(state(3)) * params.L/2, ...
                   state(2) + sin(state(3)) * params.L/2];
    line([state(1), heading_end(1)], [state(2), heading_end(2)], 'Color', 'k', 'LineWidth', 2);
end
