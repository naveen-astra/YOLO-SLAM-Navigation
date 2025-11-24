%% Visualization in X-Z Plane
function visualize_scene(state, plan, obstacles, trajectory, params)
    figure(1); clf; hold on;
    
    % Plot planned trajectory in X-Z
    plot(plan(:,1), plan(:,2), 'b--', 'LineWidth', 1.5);
    
    % Plot actual trajectory in X-Z
    plot(trajectory(:,1), trajectory(:,2), 'k-', 'LineWidth', 2);
    
    % Draw vehicle
    draw_vehicle(state, params);
    
    % Draw obstacles
    for i = 1:size(obstacles,1)
        rectangle('Position', [obstacles(i,1:2)-obstacles(i,3:4)/2, obstacles(i,3:4)], ...
                  'EdgeColor','r', 'LineWidth',1.5, 'FaceColor', [1 0 0 0.2]); % Optional transparency
    end
    
    axis equal; grid on;
    xlabel('X (m)');
    ylabel('Z (m)');   % Changed from Y to Z axis label
    legend('Planned Path', 'Actual Trajectory');
    title('ADMM-based Path Planning (X-Z Plane)');
    drawnow;
end
