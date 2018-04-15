clear all
alpha = 1; beta = 1;
v_max = 15; h_stop = 5; h_go = 25;

A = [
0,      -1,               1;
0, -(alpha + beta),     beta;
0,      beta,       -(alpha + beta)];

B = [
0 ,0;
1, 0;
0, 1
];

C = [1, 0, 0];

D = [0, 0];

X = [10, 0, 0]';

t=0;%simulation starting time
dt=0.01;%step size
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
ylabel('state variables')
title('Response of state variables')
subplot(2,1,2)
plot(Y1(:,1),Y1(:,2) );
% axis([0 10 -10 10])
xlabel('time')
ylabel('output variable')
title('Response of output variable')
