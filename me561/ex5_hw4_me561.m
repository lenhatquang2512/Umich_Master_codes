%% Problem 5- Discretize the system and plot the step response
clc;clear;close all; format default;

M = 0.75;
m = 0.2;
l = 0.4;
I = 0.0085;
g = 9.81;

%state space
J_denom = (I+m*l^2)*(M+m) - m^2*l^2;
A = [0                         1            0            0;
     (m*g*l*(M+m))/J_denom     0            0            0;
     0                         0            0            1;
     (-(m^2)*g*(l^2))/J_denom  0            0            0];

B = [0;
     -m*l/J_denom
     0;
     (I+m*l^2)/J_denom];

C = [1 0 0 0];

D = 0;

%transfer function
% syms s
% H = C * inv(s*eye(length(A)) - A) * B + D; % We can also calculate like
% this
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

T = 0.1;   % sampling time step

% Discretize the system
sys_zo = c2d(sys,T,'zoh');

% Ad , Bd , and Cd matrices if the system
% is sampled at T s = 0.1 seconds
[Ad,Bd,Cd,Dd] = ssdata(sys_zo) % get the discrete matrices back

% plot the step response
figure(1)
step(sys_zo)
set(findall(gcf,'type','line'),'linewidth',2.0);

%If we want to replot the step response using stem
% % Calculate the step response
% [Y, T] = step(sys_zo);
% 
% % Create a plot and customize the linewidth
% stem(T, Y, 'LineWidth', 1); % Change the '2' to your desired linewidth

