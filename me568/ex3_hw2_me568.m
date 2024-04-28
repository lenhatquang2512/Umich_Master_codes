clc; clear; close all; format default;

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


% Ex5.4.m
a=1.14; l=2.54; b=l-a;
g=9.81; u0=20.0; m=1500; Iz=2420.0;
Caf=2050.0*57.2958; Car=1675*57.2958;
% Define the coefficients
Yb=-(Caf+Car); Yr=(Car*b/u0)-(Caf*a/u0); Yd= Caf;
Nb=b*Car-a*Caf; Nd= a*Caf; Nr=-(a^2/u0)*Caf - (b^2/u0)*Car;
A=[0 1 u0 0;
0 Yb/m/u0 0 Yr/m-u0;
0 0 0 1;
0 Nb/Iz/u0 0 Nr/Iz];
B=[0;Yd/m;0;Nd/Iz];
C=[1 0 0 0]; D=0;
[num,Gvden]=ss2tf(A,B,C,D,1);
Gvnum=num(3:5);
Kd = 0.05; T = 0.1; Tp=0.5;
Gvnum_pv=conv(Gvnum, [Tp^2/2 Tp 1]);
Gdnum=Kd*[-T/2 1];
Gdden=[T/2 1];
Gcnum=conv(Gdnum,Gvnum_pv);
Gcden=conv(Gdden,Gvden) + Gcnum;
t=0:0.01:5;
yd=[zeros(1,51), ones(1,300), zeros(1,150)];
y_p=lsim(Gcnum,Gcden,yd,t);
e=yd-y_p';
% calculate the true lateral displacement
steer=lsim(Gdnum,Gdden,e,t);
y=lsim(Gvnum, Gvden, steer,t);
plot(t,yd, '-g', t,y_p, 'r', t-Tp, y,'-.b'); grid
xlabel('time (sec)')
title('Lat. disp. (m)')
legend('Yd', 'Y_p', 'Y')