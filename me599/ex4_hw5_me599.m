%% Problem 4c - Quadprog direct shooting
clc;clear;close all; format default;

%Constant parameters
a = 2;
T = 1;
x0 = 1;

%state space model
A = -a;
B = 1;

% Time step 
delta_t_vals = [0.01 0.1];

%loop to check 2 cases of time step
u_opt_dis_cell = cell(1,2);
time_vec_cell = cell(1,2);
for iter = 1:length(delta_t_vals)
    delta_t = delta_t_vals(iter);
    N = T/ delta_t;

    %ZOH
    ad = expm(A*delta_t);
    bd = integral(@(h) expm(A*h),0,delta_t,'ArrayValued',true) * B;

    % Weight matrices ,cost function parameters
    R = zeros(N,1);
    Q = eye(N);

    %Constraints for x and u 
    %Formulate the new model equality constraint (Direct shooting)
    Aeq = zeros(1,N);
    beq = -ad^N * x0;
    for i = 1:N
        Aeq(i) = ad^(N-i)*bd;
    end

    % Calling quadprog
%     options = optimoptions('quadprog','Algorithm','active-set'); %using active set
    [u_opt_discrete,J_opt,exitflag,output] = quadprog(2*Q,R,[],[],Aeq,beq,[],[]);

    %Store u_opt in cell due to different size
    u_opt_dis_cell{iter} = u_opt_discrete;
    time_vec_cell{iter} = 0:delta_t:T; %Note this this has size + 1 compared to u_opt
end

%Compute u_cts using eqn obtained in problem 3
u_opt_continuous = @(t) -2*a*x0*exp(-a * (T-t))/ (exp(a*T)- exp(-a*T));

%Plot
figure(1)
u_opt_discrete_1 = u_opt_dis_cell{1};
stairs(time_vec_cell{1},[u_opt_discrete_1;u_opt_discrete_1(end)],'LineWidth', 1.5);
hold on 
u_opt_discrete_2 = u_opt_dis_cell{2};
stairs(time_vec_cell{2},[u_opt_discrete_2;u_opt_discrete_2(end)],'LineWidth', 2);
fplot(u_opt_continuous,[0 T],'LineWidth', 1);
legend("$u^* discrete \Delta_t = 0.01$","$u^* discrete \Delta_t = 0.1$", "$u^* continuous $",'Interpreter', 'latex', 'Fontsize', 10);
ylabel(' Control input $u^* $','Interpreter', 'latex', 'FontSize', 12);
xlabel('$ Time [s] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal control input u* versus time")
hold off

% print ex4c_hw5_me599 -dpng;

