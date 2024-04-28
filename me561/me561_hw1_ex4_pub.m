%% 4.2 Plot x(t) for the system (no sampling)
clc;clear;close all; format default;

% Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
b   =  1.4;            % distance c.g. to rear axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =   2420.0;	% yaw moment of inertia (kg-m^2)
g   =  9.81;
u0  =  10;              %constant forward speed [m/s]
Caf = 44000*2   ;       %cornering stiffness of front wheels [N/rad]
Car = 47000*2   ;       %cornering stiffness of rear wheels [N/rad]

% Timespan for all simulations
Tspan = 0:0.1:10;   %be careful the step

%Dynamics model for lateral vehicle
A = [0   ,                      1,                  0,                               0 ;
     0   , -(Caf+Car)/(m*u0)     ,       (Caf+Car)/m ,             (b*Car-a*Caf)/(m*u0);
     0   ,                      0,                  0,                               1;
     0   ,  (b*Car-a*Caf)/(Iz*u0), (a*Caf -b*Car)/Iz , -((a^2)*Caf+(b^2)*Car)/(Iz*u0)];

B = [0   ; Caf/m;     0;  a*Caf/Iz];

% Output (Just assumptions) 
C = eye(4);
D = zeros(4, 1);

%Initial conditions of state variables
% x = [y y_dot psi-psi_d r].'
% y : lateral displacement
% y_dot : lateral velocity
% psi : yaw angle of vehicle
% psi_d : yaw angle of road
% r: yaw rate/velocity = psi_dot
x_init = zeros(4,1);

%Control input
delta_input = 0.1 * sin(Tspan);   % steer angle displacement [rad] 

%Simulated trajectory
sys_dyn_lateral = ss(A,B,C,D); %state space object
[X,T_sim] = lsim(sys_dyn_lateral,delta_input,Tspan,x_init); %simulated output of the system


%Plot the state x of the system
figure(1);
% title('System Response X(t) for lateral vehicle');
subplot(221)
plot(T_sim, X(:,1), 'r','LineWidth',2); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$ y(m)$','Interpreter','latex','FontSize',15);

subplot(222)
plot(T_sim, X(:,2), 'g','LineWidth',2); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\dot{y} (m/s)$','Interpreter','latex','FontSize',15);

subplot(223)
plot(T_sim, X(:,3), 'b','LineWidth',2); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\psi - \psi_d (rad)$ ','Interpreter','latex','FontSize',15);

subplot(224)
plot(T_sim, X(:,4), 'k','LineWidth',2); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\dot{\psi} = r (rad/s)$','Interpreter','latex','FontSize',15);

% print ex4_hw1_me561_part4_2 -dpng;

%% 4.3 - Solve for x(kT ) using the discrete state equation derived in part 4.1
T = 0.1;    % time discretization
tspan= 0:T:10;  %time span from 0 to 10 sec with T sampling rate
u=@(t) 0.1 * sin(t);    % steer angle displacement function in time [rad]

%Solve for X(kT)
X_k = zeros(4, length(tspan));
X_k(:, 1) = x_init;

% Loop through the time span
for k = 1:length(tspan)
    % Define the integration limits for this time step
    t_start = tspan(k);
    t_end = tspan(k+1);
    
    % Calculate the integral using Riemann integration
    delta_t = T/100;
    integral_term = zeros(size(A, 1), 1);
    
    % Perform Riemann integration
    for t = t_start:delta_t:t_end % Discretize the integral
        integral_term = integral_term + expm(A*((k-1)*T+T-t))*B*delta_t;
    end
    
    % Update x(kT) using the equation
    X_k(:, k+1) = expm(A*T) * X_k(:, k) + integral_term * u((k-1)*T);
end

% Plot x(kT) versus time
figure(2);
subplot(221)
stem(tspan, X_k(1,:), 'r','LineWidth',1); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$ y(m)$','Interpreter','latex','FontSize',15);

subplot(222)
stem(tspan, X_k(2,:), 'g','LineWidth',1); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\dot{y} (m/s)$','Interpreter','latex','FontSize',15);

subplot(223)
stem(tspan, X_k(3,:), 'b','LineWidth',1); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\psi - \psi_d (rad)$ ','Interpreter','latex','FontSize',15);

subplot(224)
stem(tspan, X_k(4,:), 'k','LineWidth',1); grid;
xlabel('$ t(s) $','Interpreter','latex','FontSize',15);
ylabel('$\dot{\psi} = r (rad/s)$','Interpreter','latex','FontSize',15);

% print ex4_hw1_me561_part4_3 -dpng;