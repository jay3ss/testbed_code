clear all
alpha1 = 1; beta1 = 1;
alpha2 = 10; beta2 = 1;
v_max = 25; h_stop = 5; h_go = 10;

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
dt=0.001;%step size
tsim=50.0;%finish time
n=round( (tsim-t)/dt); %no. of iterations

for i=1:n;
  u1 = range_policy(X(1), h_stop, h_go, v_max);
  u2 = range_policy(-X(1), h_stop, h_go, v_max);
  u = [u1; u2];
  % u=4;%fixed input
  dx=A*X+B*u;
  X=X+dx*dt;
  Y=C*X;
  Y1(i,:)=[t, Y];
  X1(i,:)=[t, X'];
  t=t+dt;
end

subplot(2,1,1)
plot(X1(:,1),X1(:,2:3) )
% axis([0 10 -10 10])
xlabel('time')
ylabel('Position of vehicles')
title('Response of vehicles')
legend;
subplot(2,1,2)
plot(Y1(:,1),Y1(:,2) );
% axis([0 10 -10 10])
xlabel('time')
ylabel('Headway')
title('Response of headway')
