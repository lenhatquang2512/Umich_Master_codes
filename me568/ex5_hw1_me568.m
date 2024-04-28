%% Problem 5  – Vehicle Lateral Dynamics
clear;clc;format default;
close all; 

% Parameters:
a = 1.14; % distance from CG to front axle (m)
L = 2.54; % wheel base (m)
m = 1000; % mass (kg)
Iz = 1200; % yaw moment of inertia (kg-mˆ2)
Caf = 2400 * 180/pi; % cornering stiffness, front axle (N/rad)
Car = 2056 * 180/pi ; % cornering stiffness, rear axle (N/rad)
g = 9.81; % acceleration of gravity
u0 = 20;
% forward speed (m/s)
b = L-a;

%Understeer coefficient
Kus = m*b/(L*Caf) - m*a/(L*Car) % rad/(m/sˆ2)
uchar = sqrt(L/Kus)
% u = uchar;

% Model beta and r
A = [-(Caf+Car)/(m*u0), -(a*Caf - b*Car)/(m*u0^2)-1;
-(a*Caf - b*Car)/(Iz), -(a^2*Caf+b^2*Car)/(Iz*u0)];

B = [Caf/(m*u0); a*Caf/Iz];
C = eye(2);
D = 0;

%Model v and r
% A = [-(Caf+Car)/(m*u0)    , (b*Car-a*Caf)/(m*u0)-u0 ;
%      (b*Car-a*Caf)/(Iz*u0), -(a^2*Caf+b^2*Car)/(Iz*u0)];
% B = [Caf/m; a*Caf/Iz];
% 
% Clat = [1 0]; Dlat = 0; %lateral speed
% Cyaw = [0 1]; Dyaw = 0; % yaw rate
% Cacc = A(1,:)+u0*[0, 1]; % lateral acceleration
% Dacc = B(1);
% C = [Clat; Cyaw; Cacc];
% D = [Dlat; Dyaw; Dacc];

%Simulate the system
tend = 3;
t = 0:0.001:tend;
U = deg2rad(1.0)*ones(size(t));
sys = ss(A,B,C,D);
[Y,t,X] = lsim(sys,U,t);

%Plot
figure()
% subplot(221)
% % plot(t, Y(:,1), 'r','LineWidth',2); grid; %if using v and r model
% plot(t, u0* tan(Y(:,1)) , 'r','LineWidth',2); grid; % if using beta and r model
% xlabel('Time (s)');
% ylabel('Lateral speed (m/s)');

% subplot(222)
subplot(211)
plot(t, Y(:,2)*180/pi, 'r','LineWidth',2); grid;
xlabel('Time (s)');
ylabel('Yaw rate (deg/s)');
title("Yaw rate versus time ")

% subplot(223)
subplot(212)
% plot(t, Y(:,3), 'r','LineWidth',2); grid; %if using v and r model
plot(t, Y(:,2)*u0 + u0 * [A(1,:) * Y' + B(1) * U]', 'r','LineWidth',2); grid; % if using beta and r model
xlabel('Time (s)');
ylabel('Lateral accel (m/sˆ2)');
title("Lateral acceleration versus time ")
axis([0 tend 1.6 2.8])


% subplot(224)
% plot(t, U*180/pi, 'r','LineWidth',2); grid;
% xlabel('Time (s)');
% ylabel('Steering (deg)');

print ex5_hw1_me568_lateral_acc_yaw_rate_2 -dpng;










