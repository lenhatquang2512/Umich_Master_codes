%% Problem 1 RK4 with 1-Dimension car model application
clc;clear;close all; format default;

% Initial condtions for lightweight, highly
% aerodynamic solar-powered car
x0 = 10; % v(0)
T = 0.1;
N = 10000;
u = 100*ones(N,1); %constant input force over the entire horizon

% calling Function RK4 
x_pred = RK4(x0, T, u, N, @(x,u) dynamics1D(x,u));

%Plot
t = 0:T:(N-1)*T;
figure(1);
stairs(t,x_pred), grid;
xlabel('Time [s]')
ylabel('Predicted velocity [m/s]')
title('Velocity versus Time for solar-powered car')
% print ex1_hw2_me599 -dpng;

% four-term Runge-Kutta approximation, assuming that u remains constant within
% each time step
function x_seq = RK4(x0,T,u_seq,N,odeFun)
    %T is discrete time step 
    %N is number of steps (length of time span 0:T:(N-1)*T)
    % u_seq is (N-1) step input sequence , size Nxp (p: number of control inputs)
    % x0 = x(0) is initial state vector condition , size mx1, (m : number of states)
    % odeFun is f(x,u) = x_dot , same dimention as x

    x_seq = zeros(N,length(x0)); %Initialization
    x_seq(1,:) = x0';

    for i=1:N-1
        
        % Calculate the Runge-Kutta update for each state component
        % Note: In LTV, should be odeFun(i,x,u) but we will keep it simple here
        k1 = odeFun(x_seq(i,:)',u_seq(i,:))';
        k2 = odeFun((x_seq(i,:)'+0.5*T*k1),u_seq(i,:))';
        k3 = odeFun((x_seq(i,:)'+0.5*T*k2),u_seq(i,:))';
        k4 = odeFun((x_seq(i,:)'+T*k3),u_seq(i,:))';
        
        % Update the state using the weighted sum of k1, k2, k3, and k4
        x_seq(i+1,:) = x_seq(i,:) + (T/6)*(k1+2*k2+2*k3+k4);
    end
end

% Define system dynamics function f(x, u) Note: can be f(t,x,u) in LTV
function dxdt = dynamics1D(x,u)
    m = 300;
    rho = 1.2;
    Cd = 0.15;
    A = 0.7;
    Crr = 0.01;
    g = 9.8;
   
    dxdt = (1/m)*(u - (0.5*rho*Cd*A*x^2) - Crr*m*g); 
end
