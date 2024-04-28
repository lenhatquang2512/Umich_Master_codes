%% Problem 3 - Vehicle trajectory optimization -Direct transcription approach
clc;clear;close all; format default;

%Constant parameters 
m = 1000;
rho = 1.2;
Cd = 0.4;
Aref = 5;
Crr = 0.01;
g = 9.8;

% Initial condtions for lightweight, highly
% aerodynamic solar-powered car
x0 = 0;
v0 = 25;

%Time step in discrete
T = 0.1;
N = (60-0)/T; % steps

%Trajectory optimization (Direct transcription)
options = optimoptions('fmincon','Display','iter','Algorithm','sqp', ...
    'MaxFunctionEvaluations',3*N*4,'TolFun',1e-6 * 100);   %   Last entry sets the algorithm to SQP
w0 = zeros(1,2*N + 1 *N); %Initial guess
% w0(2) = v0; w0(2*N+1) = 2887; 
% w0(4:2:(2*N)) = 26.75 * ones(1,N-1);
% w0(3:2:(2*N)) = (8/3) * (3:2:(2*N));
% w0(2*N+2:end) = 1000*ones(1,N-1); w0(2*N-1) = 1600;

%Loading initial guess for fmincon from previous run
% load w_optimal_best_27_converge.mat
% w0 = w_opt;

%Decision variable [x1 v1 x2 v2 ... xN vN u0 ... uN-1] -> size 3xN
%Maybe using lb and ub make it faster
lb = [repmat([-Inf ;-Inf], N, 1); zeros(N,1)];
lb(2*N) = v0;
ub = [repmat([Inf ;Inf], N, 1); 3000* ones(N,1)];

obj_fun = @problem3_objective;     %   Name of objective function
nl_constraint = @problem3_constraint;      %   Name of constraint function
% w_opt = fmincon(obj_fun,w0,[],[],[],[],[],[],nl_constraint,options); %optimal decision variable
w_opt = fmincon(obj_fun,w0,[],[],[],[],lb,ub,nl_constraint,options); %optimal decision variable

%Plot 
t = linspace(T,N*T,N);
u_optimal = w_opt((2*N+1):end); % resulting optimal control sequence
X_sequence = reshape(w_opt(1:2*N),2,N);
v_sequence = X_sequence(2,:); % corresponding vehicle velocity profile
x_sequence = X_sequence(1,:);

figure(1);
stairs(u_optimal,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Optimal control sequence u* [N]')
title('Resulting optimal control sequence u*')

figure(2)
stairs(t,v_sequence,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Predicted velocity [m/s]')
title('Vehicle velocity profile')

function J = problem3_objective(w)
       T = 0.1;
       N = length(w)/(2+1); % change if necessary
       %Unpack parameters
       w_states = w(1:(2*N));
       w_input = w((2*N+1):end);
       % x = w_states(1:2:length(w_states));
       v = w_states(2:2:length(w_states));
       u = w_input(1:length(w_input));
       %   Just compute J based on the formula given
       J = dot(v,u) * T; % xT ? 
end

function [g,h] = problem3_constraint(w)
    f = @problem3_dynamics;
    N = 600;
    T = 0.1;
    x0 = 0;
    v0 = 25;
    %Unpack parameters
    w_states = w(1:(2*N));
    w_input = w((2*N+1):end);
    x = w_states(1:2:length(w_states));
    v = w_states(2:2:length(w_states));
    u = w_input(1:length(w_input));
    
    % Construct g (Inequality) and h (Equality) Constraints
%     g = zeros(2*N+1,1);
    g = [];
    h = zeros(2*N+1,1);
    for i = 1:N
        %Build g with 0 <= u <= 3000
%         g(2*i-1) = u(i) - 3000;
%         g(2*i) = -u(i);
    
        %Using RK4 to build h (Non-linear dynamic model constraint)
        if(i == 1)
            % Perform first step of RK4 approximation
            k1 = f([x0,v0]', u(1));  % u(1) is u(0) in literature
            k2 = f([x0,v0]' + 0.5 * T*  k1, u(1));
            k3 = f([x0,v0]' + 0.5 * T * k2, u(1));
            k4 = f([x0,v0]' + k3*T, u(1));
            K_weighted = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
    
            h(1) = x(1) - x0 - K_weighted(1)*T;
            h(2) = v(1) - v0 - K_weighted(2)*T;
        elseif(i >=2)
            % Perform RK4 approximation for the rest states and inputs
            k1 = f([x(i-1),v(i-1)]', u(i));
            k2 = f([x(i-1),v(i-1)]' + 0.5 * T*  k1, u(i));
            k3 = f([x(i-1),v(i-1)]' + 0.5 * T * k2, u(i));
            k4 = f([x(i-1),v(i-1)]' + k3*T, u(i));
            K_weighted = (1/6)*(k1 + 2*k2 + 2*k3 + k4);

            h(2*i-1) = x(i) - x(i-1) - K_weighted(1)*T;
            h(2*i) = v(i) - v(i-1) - K_weighted(2)*T;

        end

    end
%     g(2*N+1) = -v(N) + v0; % Velocity terminal inequality constraint 
    h(2*N+1) = x(N) - 1600; % State position x terminal equality constraint
end

% Define system dynamics function f(X, u) Note: can be f(t,X,u) in LTV
function dXdt = problem3_dynamics(X, u)
    x = X(1);
    v = X(2);
    
    m = 1000;
    rho = 1.2;
    Cd = 0.4;
    Aref = 5;
    Crr = 0.01;
    g = 9.8;

    dXdt = zeros(2,1);
   
    dXdt(1) = v;
    dXdt(2) = (1/m)*(u - (0.5*rho*Cd*Aref*v^2) - Crr*m*g); 
end







