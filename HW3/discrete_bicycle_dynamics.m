function qplus = discrete_bicycle_dynamics(q, u, dt)
% discrete_bicycle_dynamics An Euler-Discretization of the dynamics of a
% bicycle modeled robot
% The input of this function is the state of the model q at time t the
% inputs to the model u at time t, and the discretization time dt
% the output of this function is qplus, the new state of the model at
% time t + dt
% You should use an Euler (first order) discretization
x = q(1);
y = q(2);
theta = q(3);
fai = q(4);
v = u(1);
w = u(2);

xp = x + v*cos(theta)*dt;
yp = y + v*sin(theta)*dt;
thetap = theta + v*tan(fai)*dt;
faip = fai + w*dt
qplus = [xp,yp,thetap,faip].'

end