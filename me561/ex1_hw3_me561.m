%% Problem 1 
clc;clear;close all; format default;

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

syms k z

H1 = (z*(z-8))/((z-1)*(z-0.7));
H2 = (0.75)/((z-1)*(z-0.7));
H3 = (0.75)/((z-1)*(z-0.7)^2);

h1 = iztrans(H1,z,k)  ;
h2 = iztrans(H2,z,k)  ;
h3 = iztrans(H3,z,k)  ;

% Display the result
fprintf("For H1: \n");
pretty(simplify(h1))
fprintf("For H2: \n");
pretty(simplify(h2))
fprintf("For H3: \n");
pretty(simplify(h3))

%% Problem 1 :check with k = 0,1,2

% convert symbolic h to function h(k) and evaluate it at k = 0,1,2
h1_func = matlabFunction(h1,'Vars',k);
h2_func = matlabFunction(h2,'Vars',k);
h3_func = matlabFunction(h3,'Vars',k);

fprintf("h1(0) =  %f \n",h1_func(0));
fprintf("h1(1) =  %f \n",h1_func(1));
fprintf("h1(2) =  %f \n",h1_func(2));

fprintf("-----------------------\n");

fprintf("h2(0) =  %f \n",h2_func(0));
fprintf("h2(1) =  %f \n",h2_func(1));
fprintf("h2(2) =  %f \n",h2_func(2));

fprintf("-----------------------\n");

fprintf("h3(0) =  %f \n",h3_func(0));
fprintf("h3(1) =  %f \n",h3_func(1));
fprintf("h3(2) =  %f \n",h3_func(2));