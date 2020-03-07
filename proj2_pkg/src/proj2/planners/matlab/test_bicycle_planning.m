clear

%% Add the path to the IPOPT binaries
% This has been tested on MATLAB R2018b on a mac running High Sierra as
% well as on MATLAB R2019B on a Lenovo X1 running Windows 10

% cur = fileparts(mfilename('fullpath'));
% if isempty(cur)
%     cur = pwd
% %     cur = append(pwd,'\ipopt');
%     
% end
% 
% % IPOPT
% disp('Adding IPOPT')
% addpath(genpath(fullfile(cur,'ipopt')));

%%%%%%%%%%%%%% Jonathan's path %%%%%%%%%%%%%%%%%
% cur = 'C:\Users\60545\Dropbox\eecs 106b\HW3\ipopt'
% addpath(cur)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Setup

obstacles = [5, 5, 2;
            -3, 4, 1;
            4, 2, 2];

start = [0;0;0;0];
goal = [7;3;0;0];

x_limits = [-10, 10];
y_limits = [-10, 10];

upper_state_bounds = [x_limits(2); y_limits(2); +inf; 0.3];
lower_state_bounds = [x_limits(1); y_limits(1); -inf; -0.3];

upper_input_bounds = [2, 3];
lower_input_bounds = [-2, -3];

N = 200;
dt = 0.05;

%% Optimize

[q_opt, u_opt] = bicycle_planning(start, goal, obstacles, ...
    upper_state_bounds, lower_state_bounds, upper_input_bounds, ...
    lower_input_bounds, N, dt);

%% Plot

close all

figure(1)
hold on
daspect([1 1 1]) % maintain aspect ratio
xlim(x_limits)
ylim(y_limits)
xlabel('X, m')
ylabel('Y, m')
title('Trajectory Plot')
% Plot obstacles
for i = 1:size(obstacles,1)
    th = 0:pi/50:2*pi;
    xunit = obstacles(i,3) * cos(th) + obstacles(i,1);
    yunit = obstacles(i,3) * sin(th) + obstacles(i,2);
    plot(xunit, yunit, '-k')
end

% Plot start and goal
plot(start(1), start(2), 'rd');
plot(goal(1), goal(2), 'r*');

% Plot trajectory
plot(q_opt(1,:), q_opt(2,:), 'bo-');
hold off

% Plot State
figure(2)
hold on
subplot(4,1,1)
plot(q_opt(1,:))
title('State Plot')
ylabel('X, m')
subplot(4,1,2)
plot(q_opt(2,:))
ylabel('Y, m')
subplot(4,1,3)
plot(q_opt(3,:))
ylabel('Theta, rad')
subplot(4,1,4)
plot(q_opt(4,:))
ylabel('Phi, rad')
xlabel('Timesteps')
hold off

% Plot Input
figure(3)
hold on
subplot(2,1,1)
plot(u_opt(1,:))
title('Input Plot')
ylabel('Velocity, m/s')
subplot(2,1,2)
plot(u_opt(2,:))
ylabel('Steering Rate, rad/s')
xlabel('Timesteps')
hold off