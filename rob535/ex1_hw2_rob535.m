clc;clear; format default;
close all;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Code/LQR/Utilities')

% Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =  2420.0;	        % yaw moment of inertia (kg-m^2)
b=L-a;                  % distance of c.g to rear axel (m) 
g=9.81;
u_0=20;

% Tire forces
B=10;
C=1.3;
D=1;
E=0.97;

% Timespan for all simulations
T=0:0.01:1;
% T = 0:0.01:10;

% vertical tire forces at the front and rear tire
Fz_f =  (b/L) * m *g;
Fz_r = (a/L) * m *g;

%% 1.1 Compute the front and rear cornerning stiffness
Ca_r= Fz_r * B * C;
Ca_f= Fz_f * B * C;

%% 1.2.1 Generate the equilibrium trajectory using Euler integration and linear tire forces
delta_fun=@(t) pi/180*sin(2*pi*t)-0.00175;

%initial conditions
% Zeq_init = zeros(5,1);
Zeq_init = [0;0;0.1;0;0];

% The full linear dynamics of the system , small angle assumptions
% Fy_f = @(Z,delta) Ca_f * (delta - ((Z(4) + a * Z(5))/ u_0));
% Fy_r = @(Z,delta) Ca_r * (- ((Z(4) - b * Z(5))/ u_0));
VehDyn_LN = @(Z,delta) [u_0 * cos(Z(3)) - Z(4) * sin(Z(3));
                    Z(4) * cos(Z(3)) + u_0 * sin(Z(3));
                    Z(5);
                    (1/m) * (Ca_r * (- ((Z(4) - b * Z(5))/ u_0)) + Ca_f * (delta - ((Z(4) + a * Z(5))/ u_0)) - m * u_0 * Z(5));
                    (1/Iz) * (-b * Ca_r * (- ((Z(4) - b * Z(5))/ u_0)) + a * Ca_f * (delta - ((Z(4) + a * Z(5))/ u_0))) ] ;

Z_eq = ode1(@(i,Z) VehDyn_LN(Z,delta_fun((i-1)/100)), T, Zeq_init);

Z_eq = Z_eq';

%% 1.2.2 Linearize bicycle model for feedback gains with linear tire forces

phi_eq = Z_eq(3,:);
v_eq = Z_eq(4,:);

% Linearizing about the trajectory yields the LTV system:
AFun = @(i) [0, 0, -u_0 * sin(phi_eq(i)) - v_eq(i) * cos(phi_eq(i)),  -sin(phi_eq(i)),    0;
             0, 0, -v_eq(i) * sin(phi_eq(i)) + u_0 * cos(phi_eq(i)),   cos(phi_eq(i)),    0;
             0, 0,                      0                          ,          0,          1;
             0, 0,                      0                          , -(Ca_f+Ca_r)/(m*u_0), ((b*Ca_r - a * Ca_f)/(m* u_0)) - u_0;
             0, 0,                      0                          , (b * Ca_r - a * Ca_f)/(Iz * u_0) , -(a^2 * Ca_f + b^2 * Ca_r)/(Iz * u_0)];

BFun = @(i) [0;
             0;
             0;
             Ca_f/m;
             Ca_f*a/Iz];


%use provided lqr_ltv function, remember the input to the A,B matrices is a discrete time index
Q= eye(5);
R= 0.5;

[K, ~] = lqr_LTV(AFun,BFun,Q,R,T);

%% 1.2.3 Plot linear vs nonlinear tire forces and find max % difference

alpha_f_li = zeros(1,length(T));
% Linear assumptions
for i = 1:length(T)
    alpha_f_li(i) = delta_fun((i-1)/100) - ((v_eq(i) + a * Z_eq(5,i))/ u_0);
end
alpha_r_li = - ((v_eq - b * Z_eq(5,:))/ u_0);
Fy_f_li =  Ca_f * alpha_f_li ;
Fy_r_li =  Ca_r * alpha_r_li;

% Pacejka model (non linear)

alpha_f_nl = zeros(1,length(T));
for i = 1:length(T)
    alpha_f_nl(i) = (delta_fun((i-1)/100) - atan((v_eq(i) + a * Z_eq(5,i))/ u_0));
end
alpha_r_nl = (- atan((v_eq - b * Z_eq(5,:))/ u_0));

Fy_f_nl =  Fz_f * D * sin(C * atan(B * (1-E)* alpha_f_nl + E * atan(B*alpha_f_nl))) ;
Fy_r_nl =  Fz_r * D * sin(C * atan(B * (1-E)* alpha_r_nl + E * atan(B*alpha_r_nl))) ;

%compute errors
e_front = max(abs(Fy_f_li - Fy_f_nl)./abs(Fy_f_nl));
e_rear = max(abs(Fy_r_li - Fy_r_nl)./abs(Fy_r_nl));

tireforce_percent_error= double(100 * max(e_front,e_rear));
fprintf("tireforce_percent_error = ");
disp(tireforce_percent_error);

% % {
% % ---Plot template---
% % uncomment template and replace <'__'> with relevant variable wherever suitable.
% % Note: these plots can be made even better and is left up to the student to improve (e.g. xlabel, ylabel...).
% 
figure()
subplot(2,1,1)
title('Mild: Tire Forces')
hold on
plot(alpha_f_li, Fy_f_li)
plot(alpha_f_nl, Fy_f_nl)
legend('linear tire forces','Nonlinear (actual) tire forces')
subplot(2,1,2)
hold on
plot(alpha_r_li, Fy_r_li)
plot(alpha_r_nl, Fy_r_nl)
legend('linear tire forces','Nonlinear (actual) tire forces')
% % }

%% 1.2.4 Euler intergrate bicycle model with nonlinear tire dynamics

% Simulate the full nonlinear dynamics of the system with the optimal
Z_eq_nl = ode1(@(i,Z) f_NL_tire(Z,K{i}*(Z_eq(:,i) -Z)+delta_fun((i-1)/100)), T, Zeq_init);

Z_eq_nl = Z_eq_nl';

% max distance error
max_distance_error= max(sqrt((Z_eq(1,:) - Z_eq_nl(1,:)).^2 + (Z_eq(2,:) - Z_eq_nl(2,:)).^2 ));
fprintf("max distance error = :");
disp(max_distance_error);

% {
% ---Plot template---
% uncomment template and replace <'__'> with relevant variable wherever suitable.
% Note: these plots can be made even better and is left up to the student to improve.

% figure(2)
% subplot(2,1,1)
% plot(<'equilibrium trajectory y-position vector'>, <'equilibrium trajectory x-position vector'>)
% hold on
% plot(<'nonlinear trajectory y-position vector'>, <'nonlinear trajectory x-position vector'>)
% xlabel('Y')
% ylabel('X')
% title('Mild: Trajectory')
% subplot(2,1,2)
% plot(<'time vector'>, <'equilibrium trajectory heading angle (psi) vector'>)
% hold on
% plot(<'time vector'>, <'nonlinear trajectory heading angle (psi) vector'>)
% xlabel('time (s)')
% legend('Reference trajectory','Actual Trajectory')
%}

figure()
subplot(2,1,1)
plot(Z_eq(2,:),Z_eq(1,:))
hold on
plot(Z_eq_nl(2,:), Z_eq_nl(1,:))
xlabel('Y')
ylabel('X')
title('Mild: Trajectory')
subplot(2,1,2)
plot(T, Z_eq(3,:))
hold on
plot(T, Z_eq_nl(3,:))
xlabel('time (s)')
legend('Reference trajectory','Actual Trajectory')


function dz = f_NL_tire(Z,delta)
    %States = z = [x,y,phi,v,r]

    % Vehicle Parameters
    a   =  1.14;		% distance c.g. to front axle (m) 
    L   =  2.54;		% wheel base (m)
    m   =  1500;		% mass (kg)
    Iz  =  2420.0;	        % yaw moment of inertia (kg-m^2)
    b=L-a;                  % distance of c.g to rear axel (m) 
    g=9.81;
    u_0=20;
    
    % Tire forces
    B=10;
    C=1.3;
    D=1;
    E=0.97;

    % vertical tire forces at the front and rear tire
    Fz_f =  (b/L) * m *g;
    Fz_r = (a/L) * m *g;

    % Set the max steering angle (must be less than pi/2 to avoid locking the
    % steering)
    phiMax = (pi*45)/180; 

    phi = Z(3);
    v = Z(4);
    r = Z(5);

    delta_sat = sat(delta,phiMax);

    alpha_f = delta_sat - atan((v + a * r)/ u_0);
    alpha_r = - atan((v - b * r)/ u_0);

    Fy_f = Fz_f * D * sin(C * atan(B * (1-E)* alpha_f + E * atan(B*alpha_f))) ;
    Fy_r = Fz_r * D * sin(C * atan(B * (1-E)* alpha_r + E * atan(B*alpha_r))) ;

    dz = zeros(5,1);
    dz(1) = u_0 * cos(phi) - v * sin(phi);
    dz(2) = v * cos(phi) + u_0 * sin(phi);
    dz(3) =  r;
    dz(4) = (1/m) * (Fy_r + Fy_f *cos(delta_sat) - m * u_0 * r);
    dz(5) = (1/Iz) * (-b * Fy_r + a * Fy_f * cos(delta_sat));

end

