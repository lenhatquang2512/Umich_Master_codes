clc;clear;close all; format default;
% Define state space
m = 1;
b = 0.5;
k = 1;

A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = eye(2);

sys = ss(A,B,C,0);

% Define weighting matrices
Q1 = [100 0; 0 1];
R1 = 1;

Q2 = [10 0; 0 10];
R2 = 1;

Q3 = [10 0; 0 1];
R3 = 10;

% Compute feedback gain matrices
K1 = lqr(A,B,Q1,R1)
K2 = lqr(A,B,Q2,R2)
K3 = lqr(A,B,Q3,R3)

% Define closed-loop system
Acl1 = A-B*K1;
Acl2 = A-B*K2;
Acl3 = A-B*K3;

eig(Acl1) 
eig(Acl2)
eig(Acl3)

sys_cl1 = ss(Acl1,[0;0],eye(2),0);
sys_cl2 = ss(Acl2,[0;0],eye(2),0);
sys_cl3 = ss(Acl3,[0;0],eye(2),0);

% Simulate results
x0 = [10;0];
t = 0:0.01:5;
u = 0*t;

[x1,t] = lsim(sys_cl1,u,t,x0);
[x2,t] = lsim(sys_cl2,u,t,x0);
[x3,t] = lsim(sys_cl3,u,t,x0);

u1 = zeros(length(t),1);
u2 = zeros(length(t),1);
u3 = zeros(length(t),1);
for i=1:length(t)
    u1(i) = -K1*x1(i,:)';
    u2(i) = -K2*x2(i,:)';
    u3(i) = -K3*x3(i,:)';
end

% Plot results
subplot(3,1,1)
plot(t,u1,'LineWidth',1.5)
hold on
plot(t,u2,'LineWidth',1.5)
hold on
plot(t,u3,'LineWidth',1.5)
legend('Case 1','Case 2','Case 3')
title('Control Effort vs Time')
xlabel('Time [s] ')
ylabel('Control u*(t) [N]')

subplot(3,1,2)
plot(t,x1(:,1),'LineWidth',1.5)
hold on
plot(t,x2(:,1),'LineWidth',1.5)
hold on
plot(t,x3(:,1),'LineWidth',1.5)
legend('Case 1','Case 2','Case 3')
title('Position vs Time')
xlabel('Time [s] ')
ylabel('y[m]')

subplot(3,1,3)
plot(t,x1(:,2),'LineWidth',1.5)
hold on
plot(t,x2(:,2),'LineWidth',1.5)
hold on
plot(t,x3(:,2),'LineWidth',1.5)
legend('Case 1','Case 2','Case 3')
title('Velocity vs Time')
xlabel('Time [s] ')
ylabel('v [m/s] ')
% print ex3d_hw6_me599 -dpng;

