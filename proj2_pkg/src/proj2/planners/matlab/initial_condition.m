function [x0, u0] = initial_condition(start,goal, N)
%initial_condition Creates an initial condition for our optimization
%   An easy way to initialize our optimization is to say that our robot
%   will move in a straight line in configuration space. Of course, this
%   isn't always possible since our dynamics are nonholonomic, but we don't
%   necessarily need our initial condition to be feasible. We just want it
%   to be closer to the final trajectory than any of the other simple
%   initialization strategies (random, all zero, etc).
%   We'll set our initial inputs to zeros to hopefully bias our solver into
%   picking low control inputs

for i = 1:4
    x(i,:) = linspace(start(i), goal(i), N+1);
end
x0 = x
u0 = zeros(2,N)

end

