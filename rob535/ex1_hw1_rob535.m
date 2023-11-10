%%
clc;close all;clear;
A=[-3 0 0;0 -1 -3;0 1 -2];

[V,D] = eig(A);

% fprintf("Singular values of A are : \n");
diagonal = diag(D);
% sig_2 = diagonal(3,1)
% sig_1 = diagonal(4,1)
% S = [sig_1 0; 0 sig_2];

%1.1
l1= diagonal(1,1);
l2= diagonal(2,1);
l3= diagonal(3,1);
% 
%List eigenvectors such that v1 is the eigenvector for eigenvalue l1.
%Eigenvectors should be in column vector format.
v1= V(:,1);
v2= V(:,2);
v3= V(:,3);

%1.2
%Write out the close form equation using the symbolic variable t as time
syms t
x0 = [5 0 0].';
x = expm(A*t) * x0;

%1.3
% options= odeset('Reltol',0.001,'Stats','on');
f=@(t,x) A*x;
tspan= 0:0.1:10;
x0= [0 1 2].';

[T,Y]=ode45(f,tspan,x0);

plot(T,Y(:,1),'-r',T,Y(:,2),'og',T,Y(:,3),'.b','LineWidth',2);
grid on;
title('Solution ZIR with ODE45');
xlabel('Time t');
ylabel('Solution y');
legend('y_1','y_2','y_3')

%1.4 

v= [1 0 0].';
t = 0:0.1:10;
f = Y*v;
sol14= trapz(t,f);
