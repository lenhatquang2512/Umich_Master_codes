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

%% 4.3 - Riemann integration, solve discrete state equations (raw way)
T = 0.1;    % time discretization
tspan= 0:T:10;  %time span from 0 to 10 sec with T sampling rate
u=@(t) 0.1 * sin(t);    % steer angle displacement function in time [rad]

%Solve for X(kT)
X_k = zeros(4, length(tspan));
X_k(:, 1) = x_init;

% Loop through the time span
for k = 1:length(tspan)-1
    % Define the integration limits for this time step
    t_start = tspan(k);
    t_end = tspan(k+1);
    
    % Calculate the integral using Riemann integration
    delta_t = T/100;
    integral_term = zeros(size(A, 1), 1);
    
    % Perform Riemann integration
    for t = t_start:delta_t:t_end % Discretize the integral
        integral_term = integral_term + expm(A*((k-1)*T+T-t))*B*u(t)*delta_t;
    end
    
    % Update x(kT) using the equation
    X_k(:, k+1) = expm(A*T) * X_k(:, k) + integral_term;
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

%% Riemann integration, using Bd from 0 to T
T = 0.1;    % time discretization
tspan= 0:T:10;  %time span from 0 to 10 sec with T sampling rate
delta_fun=@(t) 0.1 * sin(t);    % steer angle displacement function in time [rad]

% Compute discrete Ad and Bd matrices using left Riemann integration
Ad = expm(A*T);
delta_t = T/100;
Bd = zeros(4, 1);  %Bd initialization
for t = 0:delta_t:(T-delta_t) % Discretize the integral
    Bd = Bd + expm(A *t) * delta_t * B;  % Perform Riemann integration for Bd
end

%Solve for X(kT)
X_k = zeros(4, length(tspan));
X_k(:, 1) = x_init;

for k = 1:length(tspan)-1
    % Update x(kT) using the equation
    X_k(:, k+1) = Ad * X_k(:, k) + Bd * delta_fun((k-1)*T);
end

figure(3);
subplot(221)
plot(tspan, X_k(1,:), 'r'); grid;
xlabel('Time (s)');
ylabel('Lateral displacement (m)');

subplot(222)
plot(tspan, X_k(2,:), 'g'); grid;
xlabel('Time (s)');
ylabel('Lateral velocity (m/s)');

subplot(223)
plot(tspan, X_k(3,:), 'b'); grid;
xlabel('Time (s)');
ylabel('Yaw angle(vehicle - road) (rad) ');

subplot(224)
plot(tspan, X_k(4,:), 'k'); grid;T = 0.1;    % time discretization
delta_fun=@(t) 0.1 * sin(t);   
Ad = expm(A*T);
Bd = integral(@(h) expm(A*h),0,T,'ArrayValued',true) * B;
xlabel('Time (s)');
ylabel('Yaw rate (rad/s)');

%% Compute Ad and Bd not using Riemann 


% Solve the state recursively using discrete state space
tspan= 0:T:10;
X_k = zeros(4, length(tspan));
X_k(:,1) = x_init;
for i=1:length(tspan)-1
    X_k(:,i+1) = Ad * X_k(:,i) + Bd * delta_fun((i-1)*T);
end

figure(4);
subplot(221)
plot(tspan, X_k(1,:), 'r'); grid;
xlabel('Time (s)');
ylabel('Lateral displacement (m)');

subplot(222)
plot(tspan, X_k(2,:), 'g'); grid;
xlabel('Time (s)');
ylabel('Lateral velocity (m/s)');

subplot(223)
plot(tspan, X_k(3,:) *180/pi, 'b'); grid;
xlabel('Time (s)');
ylabel('Yaw angle of vehicle - road (deg) ');

subplot(224)
plot(tspan, X_k(4,:) *180/pi, 'k'); grid;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');

%% Discrete-time A and B matrices using forward Euler
A_tilde = A * T + eye(4);
B_tilde = B * T;

% Solve the state recursively using discrete state space
tspan= 0:T:10;
X_k = zeros(4, length(tspan));
X_k(:,1) = x_init;
for i=1:length(tspan)-1
    X_k(:,i+1) = A_tilde * X_k(:,i) + B_tilde * delta_fun((i-1)*T);
   
end

figure(5);
subplot(221)
plot(tspan, X_k(1,:), 'r'); grid;
xlabel('Time (s)');
ylabel('Lateral displacement (m)');

subplot(222)
plot(tspan, X_k(2,:), 'g'); grid;
xlabel('Time (s)');
ylabel('Lateral velocity (m/s)');

subplot(223)
plot(tspan, X_k(3,:) *180/pi, 'b'); grid;
xlabel('Time (s)');
ylabel('Yaw angle of vehicle - road (deg) ');

subplot(224)
plot(tspan, X_k(4,:) *180/pi, 'k'); grid;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');

%% Verify test using ode45 

% Add paths
% addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Code/LQR/Utilities')

f_R=@(t,X_k) (A * X_k + B * delta_fun(t));
[tt,X_R]=ode45(f_R,tspan,x_init);

figure(6);
subplot(221)
plot(tt, X_R(:,1), 'r'); grid;
xlabel('Time (s)');
ylabel('Lateral displacement (m)');

subplot(222)
plot(tt, X_R(:,2), 'g'); grid;
xlabel('Time (s)');
ylabel('Lateral velocity (m/s)');

subplot(223)
plot(tt, X_R(:,3) *180/pi, 'b'); grid;
xlabel('Time (s)');
ylabel('Yaw angle of vehicle - road (deg) ');

subplot(224)
plot(tt, X_R(:,4) *180/pi, 'k'); grid;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');


% -------- Functions implementations-----------------------%
%Naive left riemann function
function r=myleftsum(f,a,b,n)
    dx=(b-a)/n;
    % initialize r to f(a) (the left-end point of the first sub-interval
    % [a,a+dx])
    r=f(a);
    % need only consider the n-1 remaining sub-intervals
    for k=1:n-1
      c=a+k*dx;
      r=r+f(c);
    end 
    r=dx*r;
 end
