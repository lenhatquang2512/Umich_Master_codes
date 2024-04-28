%% Problem 2 -Engine dynamics
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
% Gs = -0.005; % For TUNE later
% If no coordination with spark ignition control
Gs = 0;
c = 0.000125;
N0 = 600;
P0 = 12;

% controller integral action gains and proportional gain
KI = 0.001; 
K = 0.01;

% Linearized six-cylinder engine idle-speed-control model
%Build state space model (derived in part a)

A = [0  0  -K*KR*(Gp + Gf*c*N0)   (-KI - K* (KR*G_delta*Gs -1/tR +KR * Gf*c*P0));
    K_theta 0 0 0;
     Kp*K_theta 0 -1/tp -Kp*KN;
     0  0  KR*(Gp + Gf*c*N0) (KR*G_delta*Gs -1/tR +KR * Gf*c*P0);];

bv = [K*KR; 0; 0; -KR];

% C = [0 0 0 1];
C = [1 0 0 0;
    0 0 0 1]; % obtaining delta theta (throttle angle)
D = 0;

%simulated time
t = 0:0.01:5;

%Control input
%unit step disturbance torque
v = ones(size(t)); % delta Td
sys = ss(A,bv,C,D);

[y,tout] = lsim(sys,v,t);

%Plot 2a
figure(1);
plot(tout,y(:,2),'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit-step $\Delta T_d $ response of engine model','Interpreter','latex','FontSize',13);
ylim([-6 6])

% print p2_img/ex2a_hw3_me568_unit_disturb_gs_0_ki_0001_K_001_PI -dpng;

% Part 2b 
%If simulate to obtain throttle angle, then
t = 0:0.01:100;
[y,tout] = lsim(sys,ones(size(t)),t);
steady_state_delta_theta = y(end,1)
KFF = steady_state_delta_theta

out = sim('ex2_hw3_me568_simulink_2b_c.slx');
figure(2);
plot(out.time_vec,out.delta_N,'b','LineWidth',2);grid;
xlabel('$ Time(s) $','Interpreter','latex','FontSize',11);
ylabel('$ \Delta N (RPM) $','Interpreter','latex','FontSize',11);
title('Unit-step $\Delta T_d $ response of engine model','Interpreter','latex','FontSize',13);
% ylim([-6 6])

% print p2_img/ex2b_hw3_me568_unit_disturb_gs_0_ki_0001_K_001_PI_feedfw -dpng;

%% Part 2c - Tuning

K = 30;
KI = 0.001;
Gs = -0.005;






