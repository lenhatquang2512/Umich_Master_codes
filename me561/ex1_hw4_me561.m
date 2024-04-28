clc;clear;close all; format default;
num = [6];
den = [1 2 10];

H = tf(num,den)

%convert tf type to sym type
syms s
Numerator = poly2sym(H.Numerator{1,1},s);
Denominator = poly2sym(H.Denominator{1,1},s);
Hsym = Numerator/Denominator;

% To convert a transfer function in the s-domain to the time domain using MATLAB
H_tf_t = ilaplace(Hsym)


%%

clc;clear;close all; format default;

syms t k T z;

% %choose coef
% a = 5; % or 1 or 5 or 0.5

%T is sampling rate 
assume(T>0)

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

% Define the continuous-time function e(t)
et_continuous = 2* exp(-t) * sin(3*t);

% Discretize e(t) by replacing t with kT
et_discrete = subs(et_continuous, t, k*T);

% Use the ztrans function to find the Z-transform of the discretized function
Z_et = ztrans(et_discrete, k, z);

% Display the result
pretty(simplify(Z_et))

%%

clc;clear;close all; format default;

syms t k T z;

%T is sampling rate 
assume(T>0)

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

% Define the continuous-time function e(t)
et_continuous = t* exp(-2*t) ;

% Discretize e(t) by replacing t with kT
et_discrete = subs(et_continuous, t, k*T);

% Use the ztrans function to find the Z-transform of the discretized function
Z_et = ztrans(et_discrete, k, z);

% Display the result
pretty(simplify(Z_et))