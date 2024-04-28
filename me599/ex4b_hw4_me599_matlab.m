%% Problem 4b - Vehicle trajectory optimization MPC
clc;clear;close all; format default;

%   Target and initial velocity parameters
x0 = 0;
v0 = 25;

%   Simulation parameters
end_time = 60;
T_MPC = 0.5;

%Call the simulink model
sim('ex4b_hw4_me599_simulink');

%Plot 
figure(1);
stairs(time_vec,u_opt,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Optimal control sequence u* [N]')
title('Resulting optimal control sequence u*')
% print ex4b_hw4_me599_u_control_MPC_woRoadgrade -dpng;

figure(2)
stairs(time_vec,v,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Predicted velocity [m/s]')
title('Vehicle velocity profile')
% print ex4b_hw4_me599_velocity_MPC_woRoadgrade -dpng;

figure(3)
stairs(time_vec,x,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Predicted position [m]')
title('Vehicle position profile')
% print ex4b_hw4_me599_velocity_MPC -dpng;


