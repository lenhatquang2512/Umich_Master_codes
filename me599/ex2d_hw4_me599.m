clc;clear;close all; format default;

% Define step size and number of steps
T = 1;
N = 6;

% Find optimal control sequence
u0 = zeros(1,2*N); % Initial guess
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',120000);
u_opt = fmincon(@obj_func,u0,[],[],[],[],[],[],@constraints,options);

% Plot results
Egen = u_opt(1:6);
Ebat = u_opt(7:12);
Edem = [100 50 70 90 30 150];
Ewind = [80 130 30 90 20 20];

% Plot
% Data plot
% x = 1:6;  % x-axis values
% Plot 1 - E_gen_optim
figure;
subplot(4, 1, 1);
stairs([Egen Egen(6)], 'LineWidth', 2);
title('$E_{gen}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{gen}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

%Plot 2 - E_bat_optim
subplot(4, 1, 2);
stairs([Ebat Ebat(6)], 'LineWidth', 2);
title('$E_{bat}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{bat}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

% Plot 3 - E_dem
subplot(4, 1, 3);
stairs( [Edem Edem(6)] , 'LineWidth', 2);
title('$E_{dem}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{dem} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

% Plot 4 - E_wind
subplot(4, 1, 4);
stairs([Ewind Ewind(6)], 'LineWidth', 2);
title('$E_{wind}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{wind} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
xlabel('Time [Hour]','Interpreter', 'latex', 'FontSize', 13) ;

% Adjust subplot layout
sgtitle('Energy over a 6-hour window for each source using SQP', 'Interpreter', 'latex', 'Fontsize', 15);

% print ex2d_hw4_me599_energy_all_SQP -dpng;

% Print the total sum of generator (optimal cost)
fprintf("The total energy for generator is : \n");
disp(sum(Egen))

% Constraint function
function [g,h] = constraints(u)
    N = 6;
    T = 1; 
    Edem = [100 50 70 90 30 150]';
    Ewind = [80 130 30 90 20 20]';
    Egen = u(1:6);
    Ebat = u(7:12);
    C(1) = 50;
    
    for i=2:N+1
        C(i) = dynamics(C(i-1),Ebat(i-1));
    end
    
    % Inequality constraints
    g1 = -C(1:6)';
    g2 = C(1:6)' - 100*ones(length(C(1:6)),1);
    g3 = Edem - Ewind - Ebat' - Egen';
%     g4 = -Ebat' - 100*ones(length(Ebat),1);
%     g5 = Ebat' - 100*ones(length(Ebat),1);
    g6 = -Egen';
%     g = [g1;g2;g3];
%     g = [g1;g2;g3;g4;g5;g6];
     g = [g1;g2;g3;g6];
    
    % Equality constraints
    h = [];

end

% Objective functional function
function J = obj_func(u)
    
    J = sum(u(1:6));

end

% Dynamics function
function C_curr = dynamics(C_prev,Ebat)    

    C_curr = C_prev - Ebat - (0.0003*((C_prev - 50)^2)*((Ebat)^2)); 

end