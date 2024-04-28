%% Problem 5- simplified quarter car model
clc;clear;close all; format default;

%%

%spring-mass-damper system parameters
m = 300;%kg
k = 18000;  %N/m
b = 1200;   %N.s/m

%%
% syms k m b s

%state space model
A = [0 1; -k/m -b/m];
B = [-1 b/m]';
C = [0 1];
D = 0;

T = 1;   % sampling time step

%transfer function
syms s
H = C * inv(s*eye(length(A)) - A) * B + D; % We can also calculate like this
pretty(simplify(H))

% zero order approximation ZOH
Ad = expm(A*T);
Bd = integral(@(h) expm(A*h),0,T,'ArrayValued',true) * B;
Cd = C;
Dd = D;

% z = tf('z',-1);
syms z
H = Cd * inv(z*eye(2) - Ad) * Bd + Dd;

%output this and then check all coefficients one by one to verify
fprintf("H(z) = \n");
pretty(simplify(vpa(H)))

[numH,denH] = ss2tf(Ad,Bd,Cd,Dd);  %only to verify

% Create the transfer function H(z)
Hz = tf(numH, denH,-1,'Variable','z');

%%
sys = ss(A,B,C,D);
H_tf = tf(sys)
% pretty(simplify(H))

%convert tf type to sym type
syms s
Numerator = poly2sym(H_tf.Numerator{1,1},s);
Denominator = poly2sym(H_tf.Denominator{1,1},s);
Hsym = Numerator/Denominator;

% convert to matlab function
H_func = matlabFunction(Hsym,'Vars',s);

%% for testing

a = exp(-2)*cos(sqrt(56))

b = exp(-2)*sin(sqrt(56))

c = exp(-4)

d = (1/sqrt(14))

syms z 
syms a b c d

Goz = 1 - ((z-1)*(z-a))/(z^2 - 2*z*a +c) + d * ((z-1)*b)/(z^2 - 2*z*a +c);

pretty(simplify(Goz))

