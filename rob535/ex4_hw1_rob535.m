clc;clear;close all;
%parameter definition
ms=400;
A=[0 -1;0 0];
B=[0;-1/ms];

t=0:1/100:10;
x_1=[0;0];

%4.1 LQR Design 1
%4.1.1 Feedback Design. use LQR to find feedback gains K_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Q1=100*diag([1,1]);
R1=0.00005;
% [K_lqr1, P_lqr1] = lqr_LTV(A,B ,Q1,R1,t);
[K_lqr1, P_lqr1] = lqr_LTV(@(t)A,@(t)B ,Q1,R1,t);


%4.1.2 Euler Simulate
sig_w = 0.0005;

% Define the initial state and time step
x1 = x_1;
dt = 0.01;

% Preallocate arrays to store the states
x_lqr1 = zeros(length(x1),length(t));
x_lqr1(:,1) = x1;

% % Euler method loop
for i = 1:length(t)-1
    x_lqr1(:,i+1) = x_lqr1(:,i) + dt*((A-B*K_lqr1{i})*x_lqr1(:,i) + [randn() * sqrt(sig_w);0]); 
end

% Plot the states
figure(1)
plot(t, x_lqr1(1,:),'r',t,x_lqr1(2,:),'g','LineWidth',2);
xlabel('Time (s)')
ylabel('State Values')
title('Lqr1');
legend("State 1","State 2");

% 
%4.2 LQR Design 2
%4.2.1 Feedback Design. use LQR to find feedback gains K_lqr
%note to use the lqr_LTV function with a constant A, B matrices, 
%enter @(i)A, @(i)B as the arguments AFun, BFun
Q2=100*diag([1,10]);
R2=0.00005;
[K_lqr2, P_lqr2] = lqr_LTV(@(t)A,@(t)B ,Q2,R2,t);


%4.2.2 Euluer Simulate
% Preallocate arrays to store the states
x_lqr2 = zeros(length(x1),length(t));
x_lqr2(:,1) = x1;

% % Euler method loop
for i = 1:length(t)-1
    x_lqr2(:,i+1) = x_lqr2(:,i) + dt*((A-B*K_lqr2{i})*x_lqr2(:,i) + [randn() * sqrt(sig_w);0]); 
end

% Plot the states
figure(2)
plot(t, x_lqr2(1,:),'r',t,x_lqr2(2,:),'g','LineWidth',2);
xlabel('Time (s)')
ylabel('State Values')
title('Lqr2');
legend("State 1","State 2");


% %4.3 Comparison 
vel= 1;
