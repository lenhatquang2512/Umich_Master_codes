%% Problem 1-Engine dynamics
clc; clear; close all; format default;

% Parameters from Table 3.1 - 6 cylinder engines model
N = 600; % [RPM] , no used
T = 0.033; % no used

K_theta = 20.0; %(lb/hr)/deg
tp = 0.210; % [sec]
Kp = 0.776; % [lbf-h/(lbm-in 2 -sec)]
Gp = 13.370; % ft-lbf/psi
G_delta = 10.0;
tR = 3.98;
KR = 67.2;
KN = 0.08;
Gf = 36.6;

%New control parameters
Gs = -0.005;
c = 0.000125;
N0 = 600;
P0 = 12;

% controller integral action gains (for part b)
% KI = 0.06;
% KI = 0.006; % for part c as well
KI = 0.0006; % for part c as well

%For part c only
% no coordination with spark ignition control
Gs = 0;
   
% Linearized six-cylinder engine idle-speed-control model
%Build state space model (derived in part a)

A = [0  0  0  -KI;
    K_theta 0 0 0;
     Kp*K_theta 0 -1/tp -Kp*KN;
     0  0  KR*(Gp + Gf*c*N0) (KR*G_delta*Gs -1/tR +KR * Gf*c*P0);];

bv = [0; 0; 0; -KR];

C = [0 0 0 1];
D = 0;

%simulated time
t = 0:0.01:5;

%Control input
%unit step disturbance torque
v = ones(size(t)); % delta Td
sys = ss(A,bv,C,D);

[y,tout] = lsim(sys,v,t);

%Plot
figure(1);
plot(tout,y,'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit-step $\Delta T_d $ response of engine model','Interpreter','latex','FontSize',13);

% print p1_img/ex2a_hw1_me568_unit_disturb_part_c_Gs_0_Ki_00006 -dpng;

% Examine the characteristics of the poles of the linear system
fprintf("Poles of the linear system:  ")
eig(A)  %POLEs
%time constants, damping ratios,
%natural frequencies
fprintf("In part a: \n");
damp(sys)
