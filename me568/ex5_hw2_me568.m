%% Problem 5 - Powertrain control (air/fuel ratio)
clc; clear; close all; format default;

%Assume linear EGO sensor, engine is 1st-order lag with delay
T_L = 1.0 ; %[s]
tau_M = 0.5; % [s]
K_M = 1; 

%Desired value of air-fuel ratio
lambda_d = 1.0; 

%Design specifications (Close-loop)
ts = 0.46 ;% 1% settling time
Mp = 0 ;% percentage overshoot  

% Desired natural frequency wn and damping ratio zeta
%based on design specifications
zeta = (1-Mp) * 0.6;
wn = 4.6/(ts * zeta);

%Design PI Controller
Kp = (2*zeta*wn*tau_M -1)/ K_M;
Ki = ((wn^2) * tau_M)/K_M;


% Simulink
out = sim("ex5_hw2_simulink_draft.slx");

% figure();
% plot(out.time_vec,out.lambda_without_delay,'r','LineWidth',2); grid;
% xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
% ylabel('Normalized air-fuel ratio $\lambda$','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("PI Control of air-fuel ratio $\lambda$ WITHOUT delay $T_L$ ",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% % print p5_img/ex5_hw2_me568_lambda_no_delay_PI -dpng;
% 
% figure();
% plot(out.time_vec,out.lambda_with_delay,'r','LineWidth',2);grid;
% xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
% ylabel('Normalized air-fuel ratio $\lambda$','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("PI Control of air-fuel ratio $\lambda$ WITH delay $T_L$ ",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% % print p5_img/ex5_hw2_me568_lambda_WITH_delay_PI -dpng;
% 
% figure();
% plot(out.time_vec,out.lambda_with_delay_smith,'r','LineWidth',2);grid;
% xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
% ylabel('Normalized air-fuel ratio $\lambda$','Interpreter','latex','FontSize',13,'FontWeight','bold');
% title("Smith Predictor PI Control of $\lambda$ WITH delay $T_L$ ",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% % print p5_img/ex5_hw2_me568_lambda_WITH_delay_PI_Smith -dpng;

figure();
plot( out.time_vec,out.lambda_without_delay,'r', ...
    out.time_vec,out.lambda_with_delay_smith,'b','LineWidth',2);grid;
legend("Without delay","With delay using Smith Predictor",'Interpreter','latex','FontSize',11);
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Normalized air-fuel ratio $\lambda$ ','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Design of Smith Predictor PI Control of air-fuel ratio",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% print ex5ac_hw2_me568_smith_predictor_with_without_delay -dpng;