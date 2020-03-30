function [q_opt, u_opt, exitval_opt] = bicycle_planning(start, goal, obstacles, ...
    upper_state_bounds, lower_state_bounds, upper_input_bounds, ...
    lower_input_bounds, N, dt)

%% Setup

% We define the dynamics as an anonymous function (a lambda function in
% python)
dynamics = @(q, u) discrete_bicycle_dynamics(q, u, dt);

% We create the variables we'll be optimizing over
q = sdpvar(4, N+1);
u = sdpvar(2, N);

%% Initial Condition

[q0, u0] = initial_condition(start, goal, N);

assign(q, q0); % We assign the initial values to our sdp vars
assign(u, u0);

%% Constraints

% You'll need to fill in most of the constraints in here

% Bound constraints
% FILL IN THESE CONSTRAINTS
x_bounds = [lower_state_bounds(1) <= q(1,:) <= upper_state_bounds(1)]:['X Constraints'];
y_bounds = [lower_state_bounds(2) <= q(2,:) <= upper_state_bounds(2)]:['Y Constraints'];
theta_bounds = [lower_state_bounds(3) <= q(3,:) <= upper_state_bounds(3)]:['Theta Constraints'];
phi_bounds = [lower_state_bounds(4) <= q(4,:) <= upper_state_bounds(4)]:['Steering Angle Constraints'];
v_bounds = [lower_input_bounds(1) <= u(1,:) <= upper_input_bounds(1)]:['Velocity Constraints'];
w_bounds = [lower_input_bounds(2) <= u(2,:) <= upper_input_bounds(2)]:['Steering Rate Constraints'];

bounds = [x_bounds, y_bounds, theta_bounds, phi_bounds, v_bounds, w_bounds];

% Dynamics constraints
% FILL IN THIS CONSTRAINT
dyn_cons = [];
for i = 1:N
    dyn_cons = [dyn_cons, [q(:,i+1) == dynamics(q(:,i), u(:,i))]:['Dynamics Constraint']];
end

% Obstacle Constraints
% You don't need to fill in this constraint
obs_cons = [];
for i = 1:size(obstacles, 1)
    for j = 1:(N+1)
        obs_cons = [obs_cons, ...
            [(q(1,j) - obstacles(i,1))^2 + (q(2,j) - obstacles(i,2))^2 >= ...
            obstacles(i,3)^2]:['Obstacle ', num2str(i), ' Constraint']];
    end
end

% Initial and Final Constraints
% FILL IN THESE CONSTRAINTS
init_cons = [q(:,1) == start]:['Initial Value Constraint'];
final_cons = [q(:,N+1) == goal]:['Final Value Constraint'];

constraints = [bounds, dyn_cons, obs_cons, init_cons, final_cons];

%% Objective Function
% These cost matrices are a bit arbitrary but indicate that we care more
% about theta than x/y (because the units of theta are smaller), and more
% about x/y than phi. We also want to minimize velocity over steering rate
Q = diag([1, 1, 2, 0.1]);
R = diag([4, 0.5]);
P = N*Q;

cost = cost_function(P, Q, R, q, u, goal);

%% Optimize

% While matlab comes with fmincon, a reasonably powerful nonlinear
% optimizer, it isn't good enough to solve this problem effectively.
% Instead we use IPOPT (Interior Point OPTimizer). IPOPT is an open source
% first order general purpose optimizer and works very well for for this
% and similar tasks
% First order methods like interior point are usually faster than second
% order methods like sqp or other trust region methods, but for certain
% classes of problems, second order methods can result in better, faster,
% or more accurate solutions

options = sdpsettings('solver', 'ipopt', ...
                      'verbose', 3, ...
                      'showprogress', 5, ...
                      'usex0', 1); % By setting this flag, we use the initial condition we defined above
            
options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 1e4;
options.ipopt.tol              = 1e-3; % The tolerance on constraints should be 1mm or 0.001 rad
options.ipopt.linear_solver    = 'ma57';
options.ipopt.ma57_automatic_scaling = 'yes';
options.ipopt.linear_scaling_on_demand = 'yes';
options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs';  % {bfgs}, sr1
options.ipopt.limited_memory_max_history = 10;  % {6}
options.ipopt.max_cpu_time = 1e8;


tic
    exitval_opt = optimize(constraints, cost, options);
toc

%% Solution

% We cast the sdp variables to doubles to get their current values (the
% solution)
q_opt = double(q);
u_opt = double(u);


end



