clc;clear;close all;format default;

%Design 
syms s 

% input transfer function here
G = 27.5 / (s^2 + 6*s+  2.75);

% convert to tf type
[numG, denG] = numden(G);
numG = sym2poly(numG);
denG = sym2poly(denG);
G_tf = tf(numG, denG);
% 
% % state space model
% sys = ss(A_bar,B_bar,C_bar,0);
% 
% % transfer function
% syms s
% 
% H_normal= C * inv(s*eye(length(A)) - A) * B

A = [-4 -1.5;4 0 ];
B = [2 0].';
C = [1.5 0.625];
D = 0;

sys = ss(A,B,C,D);


%transfer
H_tf = tf(sys)
[numH,denH] = ss2tf(A,B,C,D)
H_hand= C * inv(s*eye(length(A)) - A) * B
% [numH,denH] = ss2tf(A_,B_,C_,D_)

%convert back
% [A_,B_,C_,D_] = tf2ss(numH,denH)

% [V,D] = eig(A)