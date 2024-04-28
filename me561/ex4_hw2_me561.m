clc;clear;close all; format default;

syms t k T z;

%choose coef
a = 5; % or 1 or 5 or 0.5

%T is sampling rate 
assume(T>0)

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

% Define the continuous-time function e(t)
et_continuous = exp(-(t - a*T)) * heaviside(t - a*T);

% Discretize e(t) by replacing t with kT
et_discrete = subs(et_continuous, t, k*T);

% Use the ztrans function to find the Z-transform of the discretized function
Z_et = ztrans(et_discrete, k, z);

% Display the result
pretty(simplify(Z_et))
