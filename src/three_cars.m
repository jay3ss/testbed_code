clear all; close all;
alpha1 = 5; beta1 = 7;
alpha2 = 7; beta2 = 5;
alpha3 = 1; beta3 = 1;
v_max = 65; h_stop = 5; h_go = 10;
u_max = 15;

A = [
0, 0, 0, -1,  0,  1;
0, 0, 0,  1, -1,  0;
0, 0, 0,  0,  1, -1;
0, 0, 0, -(alpha1 + beta1),  0,  beta1;
0, 0, 0, 0,  -(alpha2 + beta2),  beta2;
0, 0, 0, 0,  beta3,  -(alpha3 + beta3);
];

B = [
  0 ,  0,   0;
  0 ,  0,   0;
  0 ,  0,   0;
beta1, 0,   0;
  0, beta2, 0;
  0,   0, beta3
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

X = [59, 43, 19, 21, 33, 47]';

t=0;%simulation starting time
dt=0.01;%step size
tsim=50.0;%finish time
n=round( (tsim-t)/dt); %no. of iterations

for i=1:n;
  u1 = range_policy(X(1), h_stop, h_go, v_max);
  u2 = range_policy(X(2), h_stop, h_go, v_max);
  u3 = range_policy(X(3), h_stop, h_go, v_max);
  
  if u1 > u_max
     u1 = u_max; 
  end
  if u2 > u_max
     u2 = u_max; 
  end
  if u3 > u_max
     u3 = u_max; 
  end
  U = [u1; u2; u3];
  % u=4;%fixed input
  dx=A*X+B*U;
  X=X+dx*dt;
  Y=C*X;
  Y1(i,:)=[t, Y'];
  X1(i,:)=[t, X'];
  U1(i,:)=[t, U'];
  t=t+dt;
end

subplot(3,1,1)
plot(X1(:,1),X1(:,5:7) )
xlabel('time')
ylabel('V(t)')
title('Velocity of vehicles')
legend('v_{1}', 'v_{2}', 'v_{3}');
subplot(3,1,2)
plot(Y1(:,1),Y1(:,2:4));
xlabel('time')
ylabel('H(t)')
title('Headway of vehicles')
legend('h_{1}', 'h_{2}', 'h_{3}');
subplot(3,1,3)
plot(U1(:,1),U1(:,2:4));
xlabel('time')
ylabel('U(t)')
title('Control action')
legend('u_{1}', 'u_{2}', 'u_{3}');