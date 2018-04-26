clear all; close all;

v_max = 35; h_stop = 5; h_go = 30;

alpha1 = 0.6; beta1 = 0.9;
alpha2 = 0.9; beta2 = 0.6;
alpha3 = 0.6; beta3 = 0.9;
alpha4 = 0.9; beta4 = 0.6;
alpha5 = 0.6; beta5 = 0.9;

ab1 = -(alpha1 + beta1);
ab2 = -(alpha2 + beta2);
ab3 = -(alpha3 + beta3);
ab4 = -(alpha4 + beta4);
ab5 = -(alpha5 + beta5);

A = [
0,  -1,   0,   0,   0,   0,   0,   0,   0,   1;    % x1_dot
0,  ab1,  0,   0,   0,   0,   0,   0,   0, beta1;  % x2_dot
0,   1,   0,  -1,   0,   0,   0,   0,   0,   0;    % x3_dot
0, beta2, 0,  ab2,  0,   0,   0,   0,   0,   0;    % x4_dot
0,   0,   0,   1,   0,  -1,   0,   0,   0,   0;    % x5_dot
0,   0,   0, beta3, 0,  ab3,  0,   0,   0,   0;    % x6_dot
0,   0,   0,   0,   0,   1,   0,  -1,   0,   0;    % x7_dot
0,   0,   0,   0,   0, beta4, 0,  ab4,  0,   0;    % x8_dot
0,   0,   0,   0,   0,   0,   0,   1,   0,  -1;    % x9_dot
0,   0,   0,   0,   0,   0,   0, beta5, 0,  ab5;   % x10_dot
];

B = [
  0,   0,   0,   0,   0;
beta1, 0,   0,   0,   0;
  0,   0,   0,   0,   0;
  0, beta2, 0,   0,   0;
  0,   0,   0,   0,   0;
  0,   0, beta3, 0,   0;
  0,   0,   0,   0,   0;
  0,   0,   0, beta4, 0;
  0,   0,   0,   0,   0;
  0,   0,   0,   0, beta5;
];

t=0;%simulation starting time
dt=0.01;%step size
tsim=50.0;%finish time
n=round((tsim-t)/dt); %no. of iterations

% Set the initial conditions
%   [h1, v1, h2, v2, h3, v3, h4, v4, h5, v5]
X = [40, 50, 40, 50, 40, 20, 40, 20, 25, 20]';
prev_h1 = 0; h1 = X(1);
prev_h2 = 0; h2 = X(3);
prev_h3 = 0; h3 = X(5);
prev_h4 = 0; h4 = X(7);
prev_h5 = 0; h5 = X(9);

% Basic premise
% - find the control action (range policy)
% - update the velocities/headways of each vehicle
% - check the headway/velocity of each vehicle for collisions
% - if there's a collision then recalculate the headway & velocity
for i=1:n;

  h1 = X(1);
  h2 = X(3);
  h3 = X(5);
  h4 = X(7);
  h5 = X(9);

  u1 = range_policy(h1, h_stop, h_go, v_max);
  u2 = range_policy(h2, h_stop, h_go, v_max);
  u3 = range_policy(h3, h_stop, h_go, v_max);
  u4 = range_policy(h4, h_stop, h_go, v_max);
  u5 = range_policy(h5, h_stop, h_go, v_max);
  U = [u1; u2; u3; u4; u5];
  % u=4;%fixed input
  dx=A*X+B*U;
  X=X+dx*dt;
  % To prevent collisions between vehicles
  [h1, vel1] = validate_headway(h1, prev_h1, h_stop, X(2), dt);
  X(1) = h1; X(2) = vel1;
  [h2, vel2] = validate_headway(h2, prev_h2, h_stop, X(4), dt);
  X(3) = h2; X(4) = vel2;
  [h3, vel3] = validate_headway(h3, prev_h3, h_stop, X(6), dt);
  X(5) = h3; X(6) = vel3;
  [h4, vel4] = validate_headway(h4, prev_h4, h_stop, X(8), dt);
  X(7) = h4; X(8) = vel4;
  [h5, vel5] = validate_headway(h5, prev_h5, h_stop, X(10), dt);
  X(9) = h5; X(10) = vel5;
  % Y=C*X;
  % Y1(i,:)=[t, Y];
  X1(i,:)=[t, X'];
  U1(i,:)=[t, U'];
  t=t+dt;

  prev_h1 = X(1);
  prev_h2 = X(3);
  prev_h3 = X(5);
  prev_h4 = X(7);
  prev_h5 = X(9);
end

% Headways
H1 = [X1(:,1), X1(:,2), X1(:, 4), X1(:, 6), X1(:, 8), X1(:, 10)];
% Velocities
V1 = [X1(:,1), X1(:,3), X1(:, 5), X1(:, 7), X1(:, 9), X1(:, 11)];


subplot(3,1,1)
plot(V1(:,1),V1(:,2:6),'linewidth', 2)
% axis([0 10 -10 10])
xlabel('time')
ylabel('V(t)')
title('Velocities of vehicles')
legend('v_{1}', 'v_{2}', 'v_{3}', 'v_{4}', 'v_{5}');
subplot(3,1,2)
plot(H1(:,1),H1(:,2:6),'linewidth', 2);
% axis([0 10 -10 10])
xlabel('time')
ylabel('H(t)')
title('Headways of vehicles')
legend('h_{1}', 'h_{2}', 'h_{3}', 'h_{4}', 'h_{5}');
subplot(3,1,3)
plot(U1(:,1),U1(:,2:6),'linewidth', 2);
% axis([0 10 -10 10])
xlabel('time')
ylabel('V(h(t))')
title('Range policy')
legend('V(h_{1})', 'V(h_{2})', 'V(h_{3})', 'V(h_{4})', 'V(h_{5})');
