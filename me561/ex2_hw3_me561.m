%%  Problem 2  - 2.4
clc;clear;close all; format default;

%Change Value of Heaviside Function at Origin
oldparam = sympref('HeavisideAtOrigin',1); 

%just for checking 2.1
z = tf('z',-1);
H = (z-10)/(0.25*z - 1.25);
numH = H.Numerator{1,1};
denH = H.Denominator{1,1};
[Ac,Bc,Cc,Dc] = tf2ss(numH,denH);

%just for checking 2.3
syms k z
assume(k>=0)
u = heaviside(k);
U = ztrans(u,k,z);
%convert tf type to sym type
Numerator = poly2sym(numH,z);
Denominator = poly2sym(denH,z);
Hsym = Numerator/Denominator;
y = iztrans(Hsym*U,z,k);
fprintf("y(k) = \n");
pretty(simplify(y))

%% convert symbolic y to function y(k) and evaluate it at k = 1,2,5 and 10
y_func = matlabFunction(y,'Vars',k);

fprintf("y(1) =  %d \n",y_func(1));
fprintf("y(2) =  %d \n",y_func(2));
fprintf("y(5) =  %d \n",y_func(5));
fprintf("y(10) = %d \n",y_func(10));
