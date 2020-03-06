function [cost] = cost_function(P, Q, R, q, u, goal)
%cost_function Cost Function for the bicycle planning optimization
%   The cost function is
%   J = sum_i=1:N (q_i' Q q_i + u_i' R u_i) + q_N+1' P q_N+1
[M,N]= size(q);
A = (q(:,N)-goal)' * P * (q(:,N)-goal);
for i = 1:N-1
    A = A + (q(:,i)-goal)'* Q * (q(:,i)-goal) + u(:,i)' * R * u(:,i);
end
cost = A

end

