clc;clear;close all; format default;

A = [1 1;exp(2) exp(-2)];
b = [1;-3];

x = inv(A) * b

%%

A2 = [ 0 1 0 0;
    0 0 1 0;
    0 0 0 1;
    1 0 0 0];

eig(A2)
% b2 = []

%%

A3 = [1 1 0 1;
    exp(pi/2)  exp(-pi/2) 1 0;
    1 1 0 -1;
     exp(pi/2)  exp(-pi/2) -1 0];
b3 = [0 1 0 1].';
x = inv(A3)*b3