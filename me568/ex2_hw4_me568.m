%% Problem 2 - Cruise Control-adaptive control
clc; clear; format default; close all; 

% Define the linearized model parameters
g=9.81; 
f=0.015; Theta=0;
rho=1.202; A=1; Cd=0.5; 

%Initial case
% uw=2;
% m=1000; 
% u0=20;
K = 0.0756; Tau = 75.632; % for INITIAL CASE

%True case
% uw=-10;
% m=1250; 
% u0=20;
% K =  0.1664; Tau = 207.9867; % for 2nd case

% DC gain and time constant with linearised model (1st order)
% Tau=(m/(rho*A*Cd*(u0+uw))); 
% K=Tau./m;

A=-(1/Tau); B=(K/Tau); %put into state space form using eqn 12.3
T=1.0; %Sampling time 

% Convert to discrete
[F,G]=c2d(A,B,T);
C=1; D=0; 

%Construct discrete transfer function
[num,den]=ss2tf(F,G,C,D,1);
b=num(2)/den(1); a=den(2)/den(1); %OK this seems ambiguous but please simply use tf

% initialization: (guess) for RLS estimator
kmax=60; n=2; lambda=0.995;
theta = [-0.5;0.5]; % coef guess for a and b

% P = 1000*eye(n);
P = 10*eye(n); %for discussion in 2c

u_old = 0.0; y_old = 0.0; uc_old = 1.0;
phi = [-y_old; u_old]; %see RLS page 230 to understand annotation here
uc = 1.0+0.01*randn(1);

% storing u,y, and coeff
Y = zeros(kmax+1,n+3);
Y(1,:)=[u_old uc y_old theta'];

% System with a ‘PI’ controller
% designed for given z, wn
% and parameter estimation using RLS:
z=0.95; wn=0.1;
% Controller Ru = Tuc - Sy
for k=1:kmax

    uc = 1.0 + 0.02*randn(1);
    y = - a*y_old + b*u_old;
    [theta,P] = rls(y,theta,phi,P,lambda);
    a1 = theta(1); b1=theta(2);
    % we use estimated values
    % (a1, b1) of the
    % process for the controller
    % gains and control: (WHAT ?)
    s0 = (exp(-2*z*wn*T)+a1)/b1;
    s1 = (1-a1-2*cos(wn*T*(1-z^2)^0.5)*exp(-z*wn*T))/b1;
    t0 = s0 + s1;
    u = u_old + t0*uc_old -s0*y_old -s1*y;
    % update the measurement vector phi
    % for next iteration:
    phi = [-y;u];
    u_old = u; y_old = y; uc_old = uc;
    Y(k+1,:)=[u uc y a1 b1];
end

% Display the results:
k=[0:kmax]'; 
% u_uc_y_ai_bi = Y;
figure()
subplot(211)
plot(k,Y(:,3),'r','LineWidth',2); grid; title('Response');
xlabel('Sampling Interval, k'); ylabel('y'); 

% figure(2) %only show b
subplot(212)
plot(k,Y(:,n+3),'b','LineWidth',2); grid; title('Parameter b Estimate');
xlabel('Sampling Interval, k'); ylabel('Estimated b hat');

% print p2_img/ex2_hw4_me568_part_c_initial_case_P_1000 -dpng;

% self written RLS
function [theta, P] = rls(y, theta,phi, P, lambda)
    K=P*phi*inv(lambda+phi'*P*phi);
    theta=theta+K*(y-phi'*theta);
    P=(eye(size(P))-K*phi')*P/lambda;
end











