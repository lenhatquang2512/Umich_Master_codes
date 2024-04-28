%% Problem 1 - Total energy expended by the generator is minimized - Linear programming
clc;clear;close all; format default;

% total energy demand and
% total energy supplied by the wind turbine
E_dem = [100 50 70 90 30 150].';
E_wind = [80 130 30 90 20 20].';

% J = k' u
k= [ones(1,6) -1 0 0 0 0 1].';

%Construct A1 and b1 so that A1 u <= b1
A1_block = [0 0 0 0 0 0;
            1 -1 0 0 0 0;
            0 1 -1 0 0 0;
            0 0 1 -1 0 0;
            0 0 0 1 -1 0;
            0 0 0 0 1 -1];

A1 = [-eye(6) , zeros(6);
     zeros(6), eye(6);
     zeros(6), -eye(6);
     -eye(6),A1_block];

b1 = [-E_dem' 100*ones(1,6) zeros(1,6) -E_wind'].';

% Calling linprog
options = optimoptions('linprog','Algorithm','dual-simplex');
[u_opt, J_opt] = linprog(k,A1,b1,[],[],[],[],options);

% Control sequences
E_gen_optim = zeros(1,6).';
E_bat_optim = zeros(1,6).';
for i = 1:6
    if(i == 1)
        E_bat_optim(i) = 0;
        E_gen_optim(i) = u_opt(i) - E_wind(i) -  E_bat_optim(i); % battery is initally empty

    else
        E_bat_optim(i) = u_opt(i+5) - u_opt(i+6);
        E_gen_optim(i) = u_opt(i) - E_wind(i) - E_bat_optim(i);
    end
end

% E_gen_total_optimal = J_opt - sum(E_wind);
% fprintf("Total generator energy over 6 hour period = ");
% disp(E_gen_total_optimal)
fprintf("E_gen_optim = \n");
disp(E_gen_optim)
fprintf("E_bat_optim = \n");
disp(E_bat_optim)

% Curtailment
E_net_curtailment = E_gen_optim + E_bat_optim + E_wind - E_dem;
fprintf("Energy Net Curtailment = ");
disp(sum(E_net_curtailment))


% Data plot
x = 1:6;  % x-axis values
% Plot 1 - E_gen_optim
figure;
subplot(4, 1, 1);
stairs(x, E_gen_optim, 'LineWidth', 2);
title('$E_{gen}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{gen}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
%Plot 2 - E_bat_optim
subplot(4, 1, 2);
stairs(x, E_bat_optim, 'LineWidth', 2);
title('$E_{bat}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{bat}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
% Plot 3 - E_dem
subplot(4, 1, 3);
stairs(x, E_dem, 'LineWidth', 2);
title('$E_{dem}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{dem} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
% Plot 4 - E_wind
subplot(4, 1, 4);
stairs(x, E_wind, 'LineWidth', 2);
title('$E_{wind}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{wind} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
xlabel('Time [Hour]','Interpreter', 'latex', 'FontSize', 13) ;

% Adjust subplot layout
sgtitle('Energy over a 6-hour window for each source', 'Interpreter', 'latex', 'Fontsize', 15);

print ex1_hw3_me599 -dpng;


