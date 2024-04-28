%% Problem 4 - Vehicle trajectory optimization 
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
delta_T = 0.2;
N = 300; % steps

%Trajectory optimization (Direct shooting)
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',120000); 
% u0 = zeros(1,N); %Initial guess
u0 =ones(1,N); %Initial guess

obj_fun = @problem4a_objective;     %   Name of objective function
nl_constraint = @problem4a_constraint;      %   Name of constraint function
u_opt_veh_dyn = fmincon(obj_fun,u0,[],[],[],[],[],[],nl_constraint,options);

%   Compute the velocity and position profiles under the optimized force
%   profile - compare to the result from the initial guess
x(1) = 0;
v(1) = 25;
state = [x(1) v(1)]';
f = @problem4a_dynamics_withRoadgrade;
% f = @problem4a_dynamics_woRoadgrade;

for i=2:N+1
    K1 = f(state,u_opt_veh_dyn(i-1));
    K2 = f(state+K1*delta_T/2,u_opt_veh_dyn(i-1));
    K3 = f(state+K2*delta_T/2,u_opt_veh_dyn(i-1));
    K4 = f(state+K3*delta_T,u_opt_veh_dyn(i-1));
    K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);

    x(i) = x(i-1) + K_weighted(1)*delta_T;
    v(i) = v(i-1) + K_weighted(2)*delta_T;
    state = [x(i) v(i)]';

end


%Plot 
t = linspace(delta_T,N*delta_T,N);

figure(1);
stairs(t,u_opt_veh_dyn,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Optimal control sequence u* [N]')
title('Resulting optimal control sequence u*')
% print ex4a_hw4_me599_u_control -dpng;

figure(2)
stairs(linspace(0,N*delta_T,N+1),v,'LineWidth',2.5), grid;
xlabel('Time [s]')
ylabel('Predicted velocity [m/s]')
title('Vehicle velocity profile')
% print ex4a_hw4_me599_velocity -dpng;

function J = problem4a_objective(u)
    
    %   Parameters
%     m = 1000;       % kg
%     g = 9.8;        % m/s^2
%     Crr = 0.01;     % dimensionless
%     rho = 1.2;      % kg/m^3
%     Cd = .4;       % dimensionless
%     A_ref = 5;      % m^2
    
    N = 300;         % horizon length
    delta_T = 0.2;    % time step
    v_final_des = 25;       % m/s (desired final velocity)

    f = @problem4a_dynamics_woRoadgrade;
    v = zeros(1,N+1); x = zeros(1,N+1);
    v(1) = 25;     %   Initial velocity
    x(1) = 0;     %   Initial position
    state = [x(1) v(1)]';
    
    %   Step through the horizon to calculate the objective function value
    J = 0;
    for i=2:N+1
        K1 = f(state,u(i-1));
        K2 = f(state+K1*delta_T/2,u(i-1));
        K3 = f(state+K2*delta_T/2,u(i-1));
        K4 = f(state+K3*delta_T,u(i-1));
        K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);
    
        x(i) = x(i-1) + K_weighted(1)*delta_T;
        v(i) = v(i-1) + K_weighted(2)*delta_T;
        state = [x(i) v(i)]';
    
        J = J + (v(i) - v_final_des)^2  * delta_T;
    end
end


function [c_ineq,c_eq] = problem4a_constraint(u)
    
    %   Parameters
%     m = 1000;       % kg
%     g = 9.8;        % m/s^2
%     Crr = 0.01;     % dimensionless
%     rho = 1.2;      % kg/m^3
%     Cd = .4;       % dimensionless
%     A_ref = 5;      % m^2
    
    N = 300;         % horizon length
    delta_T = 0.2;    % time step
    v_final_des = 25;       % m/s (desired final velocity)

    f = @problem4a_dynamics_woRoadgrade;
    v = zeros(1,N+1);
    v(1) = 25;     %   Initial velocity
    x(1) = 0;     %   Initial position
    state = [x(1) v(1)]';

    c_eq = [];
    c_ineq = zeros(2 *N,1);
    
    %   Step through the horizon to calculate the final position
    for i=2:N+1
        K1 = f(state,u(i-1));
        K2 = f(state+K1*delta_T/2,u(i-1));
        K3 = f(state+K2*delta_T/2,u(i-1));
        K4 = f(state+K3*delta_T,u(i-1));
        K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);
    
        x(i) = x(i-1) + K_weighted(1)*delta_T;
        v(i) = v(i-1) + K_weighted(2)*delta_T;
        state = [x(i) v(i)]';
        
        c_ineq(2*i-3) = -v(i) + 20;
        c_ineq(2*i-2) = v(i) - 30;
    end
   
  %not essential, not sure
%   c_eq = v(end) - v_final_des;
end



% Define system dynamics function f(X, u) Note: can be f(t,X,u) in LTV
function dXdt = problem4a_dynamics_woRoadgrade(X, u)
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

function dXdt = problem4a_dynamics_withRoadgrade(X, u)
    x = X(1);
    v = X(2);
    
    m = 1000;
    rho = 1.2;
    Cd = 0.4;
    Aref = 5;
    Crr = 0.01;
    g = 9.8;

    dXdt = zeros(2,1);

    if 0 <= x && x <= 400
        theta = 0;   % deg
    elseif 400 < x && x <= 800
        theta = 5;
    elseif 800 < x && x <= 1200
        theta =  -5;
    elseif x > 1200
        theta = 0;
    end

    dXdt(1) = v;
    dXdt(2) = (1/m)*(u - (0.5*rho*Cd*Aref*v^2) - Crr*m*g*cos(deg2rad(theta)) - m*g * sin(deg2rad(theta))); 
end
