%% Problem 3 - Vehicle lateral control , using Simulink (1st method)
clc; clear; close all; format default;

% Parameters:
a = 1.14; % distance from CG to front axle (m)
L = 2.54; % wheel base (m)
m = 1000; % mass (kg)
Iz = 1200; % yaw moment of inertia (kg-mˆ2)
Caf = 2400 * 180/pi; % cornering stiffness, front axle (N/rad)
Car = 2056 * 180/pi ; % cornering stiffness, rear axle (N/rad)
g = 9.81; % acceleration of gravity
u0 = 20; % forward speed (m/s)
b = L-a;

t=0:0.01:5;

%Dynamics model for lateral vehicle
A = [0   ,                      1,                  u0,                               0 ;
     0   ,      -(Caf+Car)/(m*u0)     ,            0 ,             ((b*Car-a*Caf)/(m*u0))-u0;
     0   ,                      0,                  0,                               1;
     0   ,    (b*Car-a*Caf)/(Iz*u0),        0 ,           -((a^2)*Caf+(b^2)*Car)/(Iz*u0) ];
 
B = [0   ; Caf/m;     0;  a*Caf/Iz];
C = [1 0 0 0];
D = 0;

%Simulink
%% Case 1
clc;close all;
Kd = 1;
T = 0.001;
out = sim("ex3_hw2_simulink_draft.slx");
figure();
plot(out.time_vec,out.lateral_displacement_y,'r','LineWidth',2); grid;

figure();
plot(out.time_vec,out.lateral_acceleration_a,'r','LineWidth',2);grid;

%% Case 2
clc;close all;
Kd = 5;
T = 0.001;
out = sim("ex3_hw2_simulink_draft.slx");
figure();
plot(out.time_vec,out.lateral_displacement_y,'r','LineWidth',2); grid;


figure();
plot(out.time_vec,out.lateral_acceleration_a,'r','LineWidth',2);grid;


%% Case 3
clc;close all;
Kd = 5;
T = 0.015;
out = sim("ex3_hw2_simulink_draft.slx");
figure();
plot(out.time_vec,out.lateral_displacement_y,'r','LineWidth',2); grid;
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Lateral displacement $y [m]$','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Lateral displacement $y$ versus time",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% print p3_img/ex3_hw2_me568_disp_Kd_5_T_0015 -dpng;

figure();
plot(out.time_vec,out.lateral_acceleration_a,'r','LineWidth',2);grid;
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Lateral acceleration $\ddot{y} [m/s^2]$ ','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Lateral acceleration $\ddot{y}$  versus time",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% print p3_img/ex3_hw2_me568_acc_Kd_5_T_0015 -dpng;



%% 2nd way just using step and lsim , not Simulink (for checking)

%Getting transfer function
[Gvnum,Gvden]=ss2tf(A,B,C,D)
% Gvnum=num(3:5);


Gdnum=conv(Kd,[-T/2 1]);
% Gdden=conv([1e-6 1], [T/2 1]);
Gdden= [0 T/2 1 ];  % Padding

Gcnum=conv(Gdnum,Gvnum);
Gcden=conv(Gdden,Gvden) + [0 Gcnum];

Gc_tf = tf(Gcnum,Gcden);

% step(Gc_tf,5)
yd = ones(1,length(t));
y=lsim(Gcnum,Gcden,yd,t);
e=lsim(Gcden-[0 Gcnum],Gcden,yd,t);
e = yd'-y;

figure()
plot(t,yd,'b',t,y,'r'); grid
title('step response')
xlabel('time (sec)')
ylabel('lat. disp. (m)'),
