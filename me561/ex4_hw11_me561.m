clc;clear;close all; format default;
A = 0.9213; B = 0.0787; C = 1; K = 4; Kr = 5; L = 0.9146;
r = 2;
% Input to the system r(t) = 2*S(t)
q = 0.81;
% Initial condition for observer state
x = -8;
% Initial condition for actual system state
y_array = [];
y_hat_array = [];
% Simulate for 5s (5/T = 5/0.05 = 100)
for t = 1 : 100
    y = C*x + 0.1*randn; % Add noise for realistic measurements.
    y_hat = C*q;
    y_array = [y_array, y];
    y_hat_array = [y_hat_array, y_hat];
    x_next = A*x - B*K*q + B*Kr*r;  % Take a step for the actual system
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Enter your code here %
    % Take a step for the observer system
    q = (A-B*K-L*C)*q + L*y + B*Kr*r;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x = x_next;
end
figure, plot(y_array)
title('Observation from actual system')
print ex4_hw11_me561_y_actual -dpng;
figure, plot(y_hat_array)
title('Observation from observer system')
print ex4_hw11_me561_y_observer -dpng;