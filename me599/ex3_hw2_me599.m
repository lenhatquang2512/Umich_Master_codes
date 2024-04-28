%% Problem 3a,b,c with N = 10,100 and 1000
clear;clc;close all; format default;

%spring-mass-damper system parameters
m = 1;
k = 1;
b = 0.5;

%state space model
Ac = [0 1; -k/m -b/m];
Bc = [0 1/m]';
Cc = [1 0];
Dc = 0;

T = 0.05;   % time step

% zero order approximation
A = expm(Ac*T);
B = integral(@(h) expm(Ac*h),0,T,'ArrayValued',true) * Bc;

% Weight matrices ,cost function parameters
R = 10.0;
Q = [1 0; 0 0.1];

% Define time step and number of steps
T = 0.05;   % time step
N = 1000;

% Define the feedback control law
K = [0.0462, 0.098]; %-Kx

% Define initial state
x = zeros(2, N); %Might need to transpose it, right now every x(indes) is a row vector
x(:,1) = [10 0]';

% Define lifted system representation,
% compact expression for J (u; x(0)) that is only a function of
% A, B, Q, R, hi,i = 1 ...N , u(i),i = 0 ...N − 1, and x(0)
H = zeros(2*N,N);
Qbar = zeros(N,N);
Rbar = zeros(1,N);

%Initialize control sequence (reference linear feedback uRef)
uRef = zeros(N,1); % (reference linear feedback uRef)
u_optimal = zeros(N,1); %  optimal control sequence u*

% Finding H(A,B)
for i = 1:2:2*N
    for j = 1:(i+1)/2
            H(i:i+1,j) = A^(((i+1)/2)-j)*B;
    end
end

% Finding Qbar
for i = 1:N
    Qbar = Qbar + H(2*i-1:2*i,:)'*Q*H(2*i-1:2*i,:);
end
Qbar = Qbar + R*eye(N,N);

% Finding Rbar
for i = 1:N
    Rbar = Rbar + x(:,1)'*(A^(i))'*Q*H(2*i-1:2*i,:);
end
Rbar = Rbar * 2;

% Finding Kbar
%No need to find Kbar because Kbar is constant, so gradient J just cancel Kbar
% for i = 0:N
%     Kbar = x(1)'*A^(i+1)*Q*A^(i+1)*x(1)';
% end

% Taking gradient of J and set it to 0
%After taking the gradient of the quadratic form wrt u, we get this
%expression -> derivative of uT.Q.u = uT(Q+QT) - when Q is not symmetric
% for this case - ustarT(Qbar + Qbar.Transpose) + Rbar = 0
u_optimal = (-Rbar*inv(Qbar+Qbar'))'; 

% Finding discrete state x using linear feedback given K , x(i+1) = (A-BK)x(i)
for i = 1:N-1
    x(:,i+1) = (A - B*K)*x(:,i);
end

% Control sequence that results from the feedback control law
for i = 1:N
    uRef(i) = -K*(x(:,i));
end

%Plot to compare
figure(1);
% tspan = 0:T:(N-1)*T;
plot(1:N,uRef,1:N,u_optimal);grid;
% xlabel('Time [s]')
xlabel('Time steps N')
xlim([1,N])
ylabel('Control sequence u [m/s]')
title('Optimal control sequence versus time when N = 1000')
legend('Reference linear u','Obtained optimal u*')
% print ex3_hw2_me599_N_1000 -dpng;
max(norm(uRef-u_optimal))



