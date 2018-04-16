clear all
alpha1 = 0.6; beta1 = 0.9;
alpha2 = 0.9; beta2 = 0.6;
v_max = 25; h_stop = 5; h_go = 30;

A = [
0,      -1,               1;
0, -(alpha1 + beta1),     beta1;
0,      beta2,       -(alpha2 + beta2)];

B = [
0 ,0;
beta1, 0;
0, beta2
];

C = [1, 0, 0];

D = [0, 0];

X = [27, 30, 75]';

t=0;%simulation starting time
dt=0.01;%step size
tsim=50.0;%finish time
n=round((tsim-t)/dt); %no. of iterations

for i=1:n;
  u1 = range_policy(X(1), h_stop, h_go, v_max);
  u2 = range_policy(-X(1), h_stop, h_go, v_max);
  U = [u1; u2];
  % u=4;%fixed input
  dx=A*X+B*U;
  X=X+dx*dt;
  Y=C*X;
  Y1(i,:)=[t, Y];
  X1(i,:)=[t, X'];
  U1(i,:)=[t, U'];
  t=t+dt;
end

subplot(3,1,1)
plot(X1(:,1),X1(:,2:3),'linewidth', 2)
% axis([0 10 -10 10])
xlabel('time')
ylabel('v(t)')
title('Velocity of vehicles')
legend('v_{1}', 'v_{2}');
subplot(3,1,2)
plot(Y1(:,1),Y1(:,2),'linewidth', 2);
% axis([0 10 -10 10])
xlabel('time')
ylabel('h(t)')
title('Headway of vehicles')
legend('h_{1}');
subplot(3,1,3)
plot(U1(:,1),U1(:,2:3),'linewidth', 2);
% axis([0 10 -10 10])
xlabel('time')
ylabel('V(h(t))')
title('Range policy')
legend('V(h_{1})', 'V(h_{2})');
