% clear all;
% close all;
% clc;

clc; clear; close all; format default;

global psi_his
global threshold_his

psi_his = [];
threshold_his = [];

global la; % vehicle half length
global dt_sim;
la = 2.2;

% T_sim = 1.0; % sim time 
% T_sim = 0.637;
T_sim = 3; % CHANGE

%% preceding vehicle
states_p = [2; 20; pi/2; 18]; % x,y,psi,v
c_p = [-5; 0.0];

% % Part d
% states_p = [2; 20; pi/2; 18]; % x,y,psi,v
% c_p = [-5; 0.0];

% Part e
% states_p = [2; 20; pi/2; 18]; % x,y,psi,v
% c_p = [-5; 0.1];

% Part f
% states_p = [0; 20; pi/2; 18]; % x,y,psi,v 
% c_p = [-5; 0.0];

% Part g
% states_p = [20; 30; pi; 18]; % x,y,psi,v 
% c_p = [-5; 0.0];

%% ego vehicle
states_e = [0; 0; pi/2; 18]; % for d and e
c_e = [0.0; 0.0];

% Part f
% states_e = [2; 0; pi/2 + 0.05; 18];
% c_e = [0.0; 0.0];

% Part g
% states_e = [0; 0; pi/2; 18];
% c_e = [0.0; 0.0];

%% simulation settings and storage initialization
dt_sim = 1e-3;
N_sim = floor(T_sim/dt_sim);
t = 0;

states_p_his = zeros(size(states_p, 1), N_sim);
states_e_his = zeros(size(states_p, 1), N_sim);
R_his = zeros(1, N_sim);
D_his = zeros(1, N_sim);
t_his = zeros(1, N_sim);
thetaR_his = zeros(3, N_sim);
thetaD_his = zeros(3, N_sim);

states_p_his(:, 1) = states_p;
states_e_his(:, 1) = states_e;
[R, D] = polar_coord(states_p, states_e);
R_his(:, 1) = R;
D_his(:, 1) = D;
t_his(:,1) = t;

%% rls for R evaluation
thetaR = [0; 0; R];
thetaR_his(:,1) = thetaR;
P_R = 1*eye(size(thetaR,1));

lambdaR = 0.95;
phi = [t^2;t;1];
[thetaR, P_R] = rls(thetaR, P_R, phi, R, lambdaR);

%% rls for D evaluation
thetaD = [0; 0; D];
thetaD_his(:,1) = thetaD;
P_D = 1*eye(size(thetaD,1));

lambdaD = 0.95;
[thetaD, P_D] = rls(thetaD, P_D,phi, D, lambdaD);
%% simulation is running
for step_idx = 2:1:N_sim
    %% simulate two vehicles
    states_p = states_p + kinematics(states_p, c_p)*dt_sim;
    states_e = states_e + kinematics(states_e, c_e)*dt_sim;
    flag = collision_detection(states_p, states_e, la);
    states_p_his(:, step_idx) = states_p;
    states_e_his(:, step_idx) = states_e;
    t = step_idx*dt_sim;
    t_his(:, step_idx) = t;

    [R, D] = polar_coord(states_p, states_e);
    R_his(:, step_idx) = R;
    D_his(:, step_idx) = D;
    thetaR_his(:,step_idx) = thetaR;
    thetaD_his(:,step_idx) = thetaD;

%% 
    angle_threshold = collision_psi_threshold(states_p, states_e);
    [col_flag, TTC] = collision_warning(t, thetaR, thetaD, angle_threshold);
    
    % rls for R
    phi = [t^2;t;1];
    [thetaR, P_R] = rls(thetaR, P_R, phi, R, lambdaR);
    % rls for D
    [thetaD, P_D] = rls(thetaD, P_D,phi, D, lambdaD);

%% Making plots
    if mod(step_idx, 100) == 3 || flag
        plot_res(step_idx, t_his, states_p_his, states_e_his, R_his, D_his, thetaR_his, thetaD_his, col_flag, TTC)
        pause(0.001)
    end

    if flag || abs(states_e(2) - states_p(2)) <= 0.1
        break;
    end

    % if col_flag
    %     break;
    % end
end
plot_res(step_idx, t_his, states_p_his, states_e_his, R_his, D_his, thetaR_his, thetaD_his, col_flag, TTC)
% print plot_img/ex1b_hw5_me568 -dpng;
% print plot_img/ex1d_hw5_me568 -dpng;
% print plot_img/ex1e_hw5_me568 -dpng;
% print plot_img/ex1f_hw5_me568 -dpng;
% print plot_img/ex1g_hw5_me568 -dpng;
