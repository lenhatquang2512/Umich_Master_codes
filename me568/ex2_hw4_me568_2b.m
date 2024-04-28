%% Problem 1 - Cruise Control
clc; clear; close all; format default;

% Define the linearized model parameters
g=9.81; 
f=0.015; Theta=0;
rho=1.202; A=1; Cd=0.5; 

% Calculate the equilibrium force, Fx0:
% Fx0 = m*g*sin(Theta) + f*m*g + 0.5*rho*A*Cd*(u0+uw)^2;

%% Part b-i
uw=2;
% m=1000; 
u0=20;
m=1000:10:1250;

% The time constant and dc gain are:
Tau=(m/(rho*A*Cd*(u0+uw))); 
K=Tau./m;

figure(1)
subplot(211)
plot(m,Tau,'b','LineWidth',2); grid;
xlabel('Weight m (kg)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Time constant $\tau $','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("Time constant versus weight",'Interpreter','latex','FontSize',13,'FontWeight','bold')
subplot(212)
plot(m,K,'b','LineWidth',2); grid;
xlabel('Weight m (kg)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('DC Gain  $K $','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Time constant and DC Gain versus weight",'Interpreter','latex','FontSize',13,'FontWeight','bold')

% print p2_img/ex2_hw4_me568_part_b_i_m_weight -dpng;

%% Part b-ii
% uw=2;
m=1000; 
u0=20;
uw = -10:1:10;

% The time constant and dc gain are:
Tau=(m./(rho*A*Cd*(u0+uw))); 
K=Tau/m;

figure(2)
subplot(211)
plot(uw,Tau,'b','LineWidth',2); grid;
xlabel('Wind velocity(m/s)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Time constant $\tau $','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("Time constant versus Wind Speed",'Interpreter','latex','FontSize',13,'FontWeight','bold')
subplot(212)
plot(uw,K,'b','LineWidth',2); grid;
xlabel('Wind velocity(m/s)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('DC Gain  $K $','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Time constant and DC Gain versus Wind Speed",'Interpreter','latex','FontSize',13,'FontWeight','bold')

% print p2_img/ex2_hw4_me568_part_ii_uw_wind_speed -dpng;




%% Part b-iii
uw=2;
m=1000; 
% u0=20;
u0 = 20:1:30;

% The time constant and dc gain are:
Tau=(m./(rho*A*Cd*(u0+uw))); 
K=Tau/m;

figure(3)
subplot(211)
plot(u0,Tau,'b','LineWidth',2); grid;
xlabel('forward velocity(m/s)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Time constant $\tau $','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("Time constant versus forward velocity",'Interpreter','latex','FontSize',13,'FontWeight','bold')
subplot(212)
plot(u0,K,'b','LineWidth',2); grid;
xlabel('forward velocity(m/s)','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('DC Gain  $K $','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Time constant and DC Gain versus forward velocity",'Interpreter','latex','FontSize',13,'FontWeight','bold')

% print p2_img/ex2_hw4_me568_part_iii_uo_forward_speed -dpng;




