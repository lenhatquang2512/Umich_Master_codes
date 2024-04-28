%% Problem 2 –Engine Dynamics
clear;clc;close all; format default;

% Parameters from Table 3.1 - 6 cylinder engines model

N = 600; % [RPM]
K_theta = 20.0; %(lb/hr)/deg
tp = 0.210; % [sec]
Kp = 0.776; % [lbf-h/(lbm-in 2 -sec)]
Gp = 13.370; % ft-lbf/psi
G_gamma = 10.0;
T = 0.033;
tR = 3.98;
KR = 67.2;
KN = 0.08;
Gf = 36.6;

%Build state space model

A = [0  0 0;
    0 -1/tp -Kp*KN;
    0 KR*Gp -1/tR;];

Bu = [K_theta 0 0 ;
    Kp*K_theta 0 0 ;
    0 KR*G_gamma KR*Gf];

bv = [0; 0;-KR];

C = eye(3);
D = 0;

%simulated time
t = 0:T:5;

%Control input

%Part a, unit step disturbance torque
v = ones(size(t));
sys = ss(A,bv,C,D);

[y,tout] = lsim(sys,v,t);

%Plot
figure(1);
plot(tout,y(:,3),'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit step disturbance $\Delta T_d $ response of engine model','Interpreter','latex','FontSize',15);

% print ex2a_hw1_me568_unit_disturb -dpng;

% Part b, separate unit step input for throttle angle (theta)
v = 0;
sys_b = ss(A,Bu,C,D);
u = [ones(size(t)); zeros(size(t));zeros(size(t))];
[y_b,tout_b] = lsim(sys_b,u,t);

%Plot
figure(2);
plot(tout_b,y_b(:,3),'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit step throttle angle $\Delta \theta $ response of engine model','Interpreter','latex','FontSize',15);

% print ex2b_hw1_me568_unit_throttle -dpng;

% Part b, separate unit step input for delta(spark timing)

u = [zeros(size(t)); ones(size(t));zeros(size(t))];
[y_b,tout_b] = lsim(sys_b,u,t);

%Plot
figure(3);
plot(tout_b,y_b(:,3),'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit step spark timing $\Delta \delta $ response of engine model','Interpreter','latex','FontSize',15);

% print ex2b_hw1_me568_unit_spark_timing -dpng;

% Part b, separate unit step input for delay fuel Fd

u = [zeros(size(t)); zeros(size(t));ones(size(t))];
[y_b,tout_b] = lsim(sys_b,u,t);

%Plot
figure(4);
plot(tout_b,y_b(:,3),'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit step delay fuel $\Delta F_d $ response of engine model','Interpreter','latex','FontSize',15);

% print ex2b_hw1_me568_unit_fuel_delay -dpng;


% Examine the characteristics of the poles of the linear system

fprintf("Poles of the linear system:  ")
eig(A)  %POLEs

%time constants, damping ratios,
%natural frequencies
fprintf("In part a: \n");
damp(sys)

fprintf("In part b: \n");
damp(sys_b)

figure(5)
pzmap(sys)






