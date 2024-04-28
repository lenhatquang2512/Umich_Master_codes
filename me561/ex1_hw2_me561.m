clc;clear;close all; format default;

A1 = [1 -1;1 -1];
A2 = [5 -6;4 -5];
A3 = [-1 0 1;0 -2 -3;0 0 -1];

syms s 
%compute e^At
STM_1 = ilaplace(inv(s*eye(2)-A1)) 
STM_2 = ilaplace(inv(s*eye(2)-A2)) 
STM_3 = ilaplace(inv(s*eye(3)-A3)) 

% clc;
% STM_1 = simplify(STM_1)
% STM_2 = simplify(STM_2)
% STM_3 = simplify(STM_3)

syms t
expm(A1*t)
expm(A2*t)
expm(A3*t)

%sol for ZIR

%check e-values
fprintf("For A1: \n");
eig(A1)
fprintf("For A2: \n");
eig(A2)
fprintf("For A3: \n");
eig(A3)
