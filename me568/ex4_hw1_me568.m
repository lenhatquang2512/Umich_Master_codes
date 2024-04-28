%% Problem 4  – Vehicle Longitudinal Dynamics
clear;clc;close all; format default;

% Vehicle Parameters
m = 2000;  %[kg]
g = 9.8;
Iw = 0.003;   % [kg/m^2]
rw = 0.3; % [m]
Fz = m * g;
Tb = 7000; %[Nm]

W=m*g;
Fx=-2000; 
Theta=0.0; 
f=0.02;
rho=1.202; 
Cd=0.4;
A=2; 
uw=0.0;

% Initial time, final time (seconds)
ti = 0.0; tf = 5.0;

% Initial speed
ui = 12;

%Coefficient of friction
mu1 = 0.7; % 1st car with ABS
mu2 = 0.5; % 2nd car may have wheel lock-up shortly

% Perform integration and store results in x
[t1,u1] = ode45(@(t,u)long_dyn(t,u,mu1),[ti , tf],ui);
[t2,u2] = ode45(@(t,u)long_dyn(t,u,mu2),[ti , tf],ui);

% Plot the results
figure();
plot(t1, u1,'r', t2,u2, 'g','LineWidth',2)
legend('1st car ABS','2nd car No ABS')
title('Vehicle forward speed');
xlabel('Time [sec]');
ylabel('u [m/s]');
grid;

% Compute stopping distance and times that can be achieved by 2 vehicles
stopDis1 = trapz(t1,u1)
stopDis2 = trapz(t2,u2)

function udot = long_dyn(t,u,mu)
    % Equations of longitudinal motion
    % for a vehicle.
    % The parameters are:
    % u - vehicle forward velocity
    % m - vehicle mass
    % Fx - tractive or braking force
    % W - vehicle weight, W = m*g
    % Theta - road grade angle in radians
    % f - rolling friction coefficient
    % rho - density of air
    % Cd - aerodynamic drag coefficient
    % A - cross sectional area of vehicle
    % uw - wind velocity
    %
    % Parameter values):
    m=2000; g=9.8; W=m*g;
    Fx=-2000; Theta=0.0; f=0.02;
    rho=1.202; Cd=0.4; A=2; uw=0.0;

  
    Iw = 0.003;   % [kg/m^2]
    rw = 0.3; % [m]
    Fz = m * g;
    Tb = 7000; %[Nm]

    B = Tb/rw;
    C = Iw/(rw^2);
    D = mu * Fz;
    %
    if u > 0
        udot = (1/(m+C))*(-W*sin(Theta) - f*W*cos(Theta)-0.5*rho*Cd*A*(u+uw)^2 - B);

        Fb = B + C * udot;
        if Fb < D
            udot = udot;
        elseif Fb>= D
            udot = (1/m)*(-W*sin(Theta) - f*W*cos(Theta)-0.5*rho*Cd*A*(u+uw)^2 - D);
        end

    else
        udot=0;
    end
end