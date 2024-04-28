clc;clear;close all; format default;
tend = 2.5;
a = 0.1;
%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

t = 0:0.01:tend;
e_t = (1 - exp(-(t-a))).*heaviside(t-a);
figure(1);
plot(t,e_t,'g','LineWidth',2); grid;
xlabel("Time [s]");
ylabel("e(t)");
ylim([0 1])

syms timeS 
et = (1 - exp(-(timeS-a))).*heaviside(timeS-a)

et_func = matlabFunction(et,'Vars',timeS);

fprintf("et(0) =  %f \n",et_func(0));
fprintf("et(0.5) =  %f \n",et_func(0.5));
fprintf("et(1) =  %f \n",et_func(1));
fprintf("et(1.5) = %f \n",et_func(1.5));
fprintf("et(2) = %f \n",et_func(2));
fprintf("et(2,5) = %f \n",et_func(2.5));

%%
clc;clear;close all; format default;
syms t k T z;

% %T is sampling rate 
% assume(T>0)

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

% Define the continuous-time function e(t)
et_continuous = (1- exp(-(t - 0.1)))* heaviside(t - 0.1);

% Discretize e(t) by replacing t with kT
et_discrete = subs(et_continuous, t, k*0.5);

% Use the ztrans function to find the Z-transform of the discretized function
Z_et = ztrans(et_discrete, k, z);

% Display the result
pretty(simplify(Z_et))

%%
% clc;
clear;close all; format default;
syms t k T z;

% %T is sampling rate 
% assume(T>0)

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

% Define the continuous-time function e(t)
et_continuous = (1- exp(-(t - 0.0)));

% Discretize e(t) by replacing t with kT
et_discrete = subs(et_continuous, t, 0.4+ k*0.5);

% Use the ztrans function to find the Z-transform of the discretized function
Z_et = ztrans(et_discrete, k, z);

% Display the result
pretty(simplify(Z_et))
