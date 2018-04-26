clear all; close all;
alpha1 = 0.6; beta1 = 0.6;
alpha2 = 0.6; beta2 = 0.6;
alpha3 = 0.6; beta3 = 0.9;
v_max = 35; h_stop = 5; h_go = 30;
% u_max = 15;

A = [
0, 0, 0, -1,  0,  1;
0, 0, 0,  1, -1,  0;
0, 0, 0,  0,  1, -1;
0, 0, 0, -(alpha1 + beta1),  0,  beta1;
0, 0, 0, beta2,  -(alpha2 + beta2),  0;
0, 0, 0, 0,  beta3,  -(alpha3 + beta3);
];

B = [
  0 ,  0,   0;
  0 ,  0,   0;
  0 ,  0,   0;
alpha1, 0,   0;
  0, alpha2, 0;
  0,   0, alpha3
];

% C = [1, 1, 1, 0, 0, 0];

C = [
1, 0, 0, 0, 0, 0;
0, 1, 0, 0, 0, 0;
0, 0, 1, 0, 0, 0;
];


D = [
0, 0, 0;
0, 0, 0;
];

X = [40, 10, 20, 0, 20, 0]';

t=0;%simulation starting time
dt=0.01;%step size
tsim=500.0;%finish time
n=round((tsim-t)/dt); %no. of iterations

% During each iteration, solve the system and make sure that 
% the cars don't go through each other


for i=1:n;
  u1 = range_policy(X(1), h_stop, h_go, v_max);
  u2 = range_policy(X(2), h_stop, h_go, v_max);
  u3 = range_policy(X(3), h_stop, h_go, v_max);

  U = [u1; u2; u3];
  % u=4;%fixed input
  dx=A*X+B*U;
  X=X+dx*dt;
  if X(1) < 0
    X(1) = 0;
  end
  if X(2) < 0
    X(2) = 0;
  end
  if X(3) < 0
    X(3) = 0;
  end
  Y=C*X;
  Y1(i,:)=[t, Y'];
  X1(i,:)=[t, X'];
  U1(i,:)=[t, U'];
  t=t+dt;
end

subplot(3,1,1)
plot(X1(:,1),X1(:,5:7),'linewidth', 2)
xlabel('time')
ylabel('V(t)')
title('Velocity of vehicles')
legend('v_{1}', 'v_{2}', 'v_{3}');
subplot(3,1,2)
plot(Y1(:,1),Y1(:,2:4),'linewidth', 2);
xlabel('time')
ylabel('H(t)')
title('Headway of vehicles')
legend('h_{1}', 'h_{2}', 'h_{3}');
subplot(3,1,3)
plot(U1(:,1),U1(:,2:4),'linewidth', 2);
xlabel('time')
ylabel('V(h(t))')
title('Range policy')
legend('V(h_{1})', 'V(h_{2})', 'V(h_{3})');
