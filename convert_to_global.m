function global_poses = convert_to_global(data, params)
    % Converts ORB-SLAM3 egocentric coordinates to global frame

    global_poses = zeros(height(data),3);
    
    for i = 1:height(data)
        % Rotation matrix directly from data
        R = [data.r11(i), data.r12(i), data.r13(i);
             data.r21(i), data.r22(i), data.r23(i);
             data.r31(i), data.r32(i), data.r33(i)];
         
        % Translation vector
        t = [data.t1(i); data.t2(i); data.t3(i)];
        
        % Global position = R' * t
        global_poses(i,:) = (-R' * t)'; % SLAM to global transform
    end
end
