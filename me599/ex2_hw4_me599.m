%% Problem 2 - Energy demand dynamic programming
clc;clear;close all; format default;

%Constant parameters
Edem=[100 50 70 90 30 150];
Ew=[80 130 30 90 20 20];
N=6;

% Min and max allowable states and inputs values
%Define quantizations
quantC=0:10:100;
quantEb=-100:10:100;

% quantC=0:1:100;
% quantEb=-100:1:100;

% Initial conditions
C0=50;

%   Populate a table of stage costs and destination states for quick reference
J_stage = zeros(length(quantC),length(quantEb),length(Edem));
C_dest = zeros(length(quantC),length(quantEb));
for i = 1:length(quantC)
    for j = 1:length(quantEb)
        C_dest(i,j)=dynamics(quantC(i), quantEb(j));
        for k = 1:length(Edem)
            if C_dest(i,j)>=0 && C_dest(i,j)<=100
                J_stage(i, j, k)=max(0,Edem(k)-Ew(k)-quantEb(j));
            else
                J_stage(i, j, k)=NaN;
            end
        end
    end
end

% Backward Recursion
%   Compute the vector of optimal costs to go from step N-1, along with the
%   corresponding control values
J_opt_togo = zeros(length(quantC),1);  %   Optimal cost to go from each state
u_opt_togo_matrix = zeros(length(quantC),N);    %   Optimal control sequence from each originating state, at each stage
for i=1:length(quantC)
    [J_opt_togo(i),index] = min(J_stage(i,:,N));
    u_opt_togo_matrix(i,N) = quantEb(index);
end

%   Begin the backward recursion through the rest of the stages
stage = N-2;
while stage >= 0
    J_opt_togo_prev = J_opt_togo;
    u_opt_togo_matrix_prev = u_opt_togo_matrix;
    for i=1:length(quantC)
        for j=1:length(quantEb)
            %   Calculate the intermediate (successor) state resulting from
            %   application of each candidate control input
            C_inter = dynamics(quantC(i), quantEb(j));
            %   In general, the successor state calculated above will NOT
            %   be one of the quantized state values, so we will need to
            %   interpolate
            if C_inter>=0 && C_inter<=100
                J_opt_togo_current = max(0,interp1(quantC,J_opt_togo_prev,C_inter));
                %   Calculate the optimal cost to go through the above
                %   successor state
                J_togo(j) = J_stage(i,j,stage+1) + J_opt_togo_current;
            else
                J_togo(j) = NaN;
            end
        end
        % J_togo
        [J_opt_togo(i),control_index] = min(J_togo);
        u_opt_togo_matrix(i,stage+1) = quantEb(control_index);
        %   Need to append the optimal control sequence with the optimal
        %   control sequence to go from the the intermediate state
        %   calculated above. Because this intermediate state will, in
        %   general, not correspond to one of the quantized states, we must
        %   interpolate the control sequence as well
        C_inter = dynamics(quantC(i), quantEb(control_index));
        for k=stage+2:N
            u_opt_togo_matrix(i,k) = interp1(quantC,u_opt_togo_matrix_prev(:,k),C_inter);
        end
    end
    stage = stage - 1;
end
%   Note - We have computed the optimal control input sequence for all
%   possible initial conditions. The example asks specifically about x0 =
%   1, so we will plot the optimal control sequence for that one in
%   particular
relevant_IC = find(quantC == C0);
relevant_IC
u_opt_vector = u_opt_togo_matrix(relevant_IC,:);

%Preparing Data for plotting
E_bat_optim = u_opt_vector;
E_gen_optim = max(0,(Edem - Ew - E_bat_optim));

% figure(1)
% stairs(u_opt_vector);
% xlabel('Time step','fontsize',12);
% ylabel('u','fontsize',12);

%   Report the optimal cost for the relevant IC
J_best_DP = J_opt_togo(relevant_IC)

%   Although the plots in figures 3 and 4 are given for x0 = 1, we have in
%   fact computed the optimal control sequences for all candidate initial
%   conditions. These are given in the matrix
%   u_opt_togo_matrix
u_opt_matrix = u_opt_togo_matrix

% Plot
% Data plot
% x = 1:6;  % x-axis values
% Plot 1 - E_gen_optim
figure;
subplot(4, 1, 1);
stairs([E_gen_optim E_gen_optim(6)], 'LineWidth', 2);
title('$E_{gen}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{gen}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

%Plot 2 - E_bat_optim
subplot(4, 1, 2);
stairs([E_bat_optim E_bat_optim(6)], 'LineWidth', 2);
title('$E_{bat}^*$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{bat}^* [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

% Plot 3 - E_dem
subplot(4, 1, 3);
stairs( [Edem Edem(6)] , 'LineWidth', 2);
title('$E_{dem}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{dem} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;

% Plot 4 - E_wind
subplot(4, 1, 4);
stairs([Ew Ew(6)], 'LineWidth', 2);
title('$E_{wind}$', 'Interpreter', 'latex', 'Fontsize', 13);
ylabel('$E_{wind} [kWh] $','Interpreter', 'latex', 'FontSize', 11) ;
xlabel('Time [Hour]','Interpreter', 'latex', 'FontSize', 13) ;

% Adjust subplot layout
sgtitle('Energy over a 6-hour window for each source with $\Delta_E = 1$', 'Interpreter', 'latex', 'Fontsize', 15);

% print ex2b_hw4_me599_energy_all -dpng;
% print ex2c_hw4_me599_energy_all -dpng;

function Cnew=dynamics(Cprev, Ebat)
    Cnew=Cprev-Ebat-(0.0003*((Cprev - 50)^2)*((Ebat)^2));
end
