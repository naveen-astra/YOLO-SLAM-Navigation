function next_state = bicycle_model(current_state, control, params)
% Inputs:
% current_state = [x, z, theta, v]
% control       = [acceleration, steering_angle]
% params        = parameter struct (must include dt, car.wheelbase)

x     = current_state(1);
z     = current_state(2);   % was y
theta = current_state(3);
v     = current_state(4);

a     = control(1);  % acceleration
delta = control(2);  % steering angle

dt    = params.dt;
L     = params.wheelbase;

% Clip values
a     = max(min(a, params.max_acc), -params.max_acc);
delta = max(min(delta, params.max_steer), -params.max_steer);

% Kinematic Bicycle Model Equations in x-z plane
x_next = x + v * cos(theta) * dt;
z_next = z + v * sin(theta) * dt;
theta_next = theta + (v / L) * tan(delta) * dt;
v_next = v + a * dt;

% Combine next state
next_state = [x_next, z_next, theta_next, v_next];

end
