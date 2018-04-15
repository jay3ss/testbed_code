% Start with two human drivers
%
alpha = 1; beta = 1;
v_max = 15; h_stop = 5; h_go = 25;

hv1 = HumanDriver(v_max, h_stop, h_go, alpha, beta);
hv2 = HumanDriver(v_max, h_stop, h_go, alpha, beta);

run_time = 30; % seconds
delta_t = 0.001; % 1 ms
num_its =  run_time/delta_t + 1;
X0 = [30; 0; 0];
X = zeros(3, num_its);
X_dot = X;
X(:, 1) = X0;

vel1 = 0; % velocity of hv1
vel2 = 0; % velocity of hv2

for i = 1:delta_t:run_time+1
  i
  notice = strcat('Iteration: ', num2str(i));
  vel1 = vel1 + X(2, i) * delta_t
  vel2 = vel2 + X(3, i) * delta_t
  h_dot = X(3, i) - X(2, i)
  X_dot(1, i+1) = X_dot(1, i) + h_dot * delta_t;
  X_dot(2, i+1) = X_dot(2, i) + hv1.v_dot(vel2) * delta_t;
  X_dot(3, i+1) = X_dot(2, i) + hv2.v_dot(vel1) * delta_t;
end
