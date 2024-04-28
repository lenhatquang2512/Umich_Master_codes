clc;clear;close all; format default;

T = 0.05;   % time step

D = tf([23 12.8],[1 0]);

% discretize the
% analog controller for T = 0.01s and T = 0.5s
T1 = 0.01;   % time step
T2 = 0.5;   % time step
Dz1 = c2d(D,T1,'zoh')
Dz2 = c2d(D,T2,'zoh')
