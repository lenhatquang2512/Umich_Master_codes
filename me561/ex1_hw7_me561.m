%% Problem 1- ROAHM Rover
clc;clear;close all; format default;

% LTI continuous state space when driving in a straight line
A = [0                         0            0.0055           1;
     0     0            0.3390           -0.1020;
     0                         0            0            -0.0506;
     0  0            0            -1.64];

B = [0 0.0013;
     0 0.1072;
     0 1.4950;
     1.64 0];

C = [0 0 0 1];

D = 0;

% open-loop system, represents the effect of driving the rover forward on
% roads of different inclines
B_open = B(:,1);

% transfer function of the rover with
% respect to the commanded velocity input
syms s
H = C * inv(s*eye(length(A)) - A) * B_open + D; % We can also calculate like this
pretty(simplify(H))
% sys = ss(A,B_open,C,D);
% H_cts = tf(sys)

%% 1.6 run only when simulink runs

time = out.simout.Time;
sys_res = out.simout.Data;

figure(1)
plot(time,sys_res,'LineWidth',2.0); grid;
xlabel("Time[s]")
ylabel("Longitudinal velocity [m/s]");
axis([0 30 -0.5 2.5])
title("Response of the system")

print ex1_6_hw7_me561 -dpng;