%% Problem 2 - Constrained optimization Mass Spring Damper - Quadratic programming
clc;clear;close all; format default;

%spring-mass-damper system parameters
m = 1;
k = 1;
b = 0.5;

%state space model
Ac = [0 1; -k/m -b/m];
Bc = [0 1/m]';
Cc = [1 0];
Dc = 0;

% Define time step and number of steps
T = 0.05;   % time step
N = 300;

% zero order approximation ZOH
A = expm(Ac*T);
B = integral(@(h) expm(Ac*h),0,T,'ArrayValued',true) * Bc;

% Weight matrices ,cost function parameters
R = 10.0;
Q = [1 0; 0 0.1];

%Constraints for x and u, change for part a,b and c
% umax = 5;
% umin = -umax;
% x1max = 10;
% x1min = -x1max;
% x2max = 10;
% x2min = -x2max;

% %Min and Max values of u and x (part b)
x1max = 10;
x2max = 10;
x1min  = -10;
x2min  = -10;
umax = 0.25;
umin = -0.25;

%Min and Max values of u and x (part c)
% x1max = 10;
% x2max = 10;
% x1min  = 0;
% x2min  = -10;
% umax = 5;
% umin = -5;

% Define initial state
n = size(A,1);
m = size(B,2);
% n = size(Q,1);
% m = size(R,1);
x_constrained = zeros(n, N); 
x0 = [10 0]';
x_constrained(:,1) = x0;

%   Formulate the new model inequality constraint
A1 = [eye(n*N) , zeros(n*N,m*N);
      -eye(n*N) , zeros(n*N,m*N);
      zeros(m*N,n*N), eye(m*N,m*N);
      zeros(m*N,n*N),-eye(m*N,m*N)];

b1 = [repmat([x1max ;x2max], N, 1);
       repmat([-x1min ;-x2min], N, 1);
       repmat(umax, N, 1);
       repmat(-umin, N, 1)];

%   Form the new H matrix for direct transcription
H = zeros(N*(n+m));
for i=1:N
    H(n*(i-1)+1:i*n,n*(i-1)+1:i*n) = Q;
end
for i=1:N
    H(N*n+m*(i-1)+1:N*n+i*m,N*n+m*(i-1)+1:N*n+i*m) = R;
end


%  Formulate the new model equality constraint
A2 = zeros(n*N,(n+m)*N);
b2 = [A*x0; zeros(n*(N-1),1)];
for i=1:N
    if i==1
        A2(n*(i-1)+1:n*i,n*(i-1)+1:n*i) = eye(n);
        A2(n*(i-1)+1:n*i,N*n+m*(i-1)+1:N*n+m*i) = -B;
    else
        A2(n*(i-1)+1:n*i,n*(i-1)+1:n*i) = eye(n);
        A2(n*(i-1)+1:n*i,N*n+m*(i-1)+1:N*n+m*i) = -B;
        A2(n*(i-1)+1:n*i,n*(i-2)+1:n*(i-1)) = -A;
    end
end

%   Use quadprog to determine the optimal control sequence
% options = optimoptions('quadprog','Algorithm','active-set');
% u0 = zeros((n+m)*N,1);
[u_opt_aug,J_opt,exitflag,output] = quadprog(2*H,zeros(N*(m+n),1),A1,b1,A2,b2);
u_opt = u_opt_aug(N*n+1:N*(n+m));

% Compute x1 and x2 from applying this optimized control sequence
for i = 1:N-1
    x_constrained(:, i + 1) = A * x_constrained(:, i) + B * u_opt(i);
end

% Results obtained from unconstrained case (previous homework 2 problem 3)
K = [0.0462, 0.098]; % Define the feedback control law
u_opt_unconstrained = zeros(N,1); 
% Finding discrete state x using linear feedback given K , x(i+1) = (A-BK)x(i)
x_unconstrained = zeros(length(x0), N);
x_unconstrained(:, 1) = x0;
for i = 1:N-1
    x_unconstrained(:,i+1) = (A - B*K)*x_unconstrained(:,i);
end

% Control sequence that results from the feedback control law
for i = 1:N
    u_opt_unconstrained(i) = -K*(x_unconstrained(:,i));
end


%Plot
figure(1)
subplot(3, 1, 1);
stairs(u_opt,'r','LineWidth', 2);
hold on 
stairs(u_opt_unconstrained,'b','LineWidth', 1.5)
legend("$u^*$ constrained","$u^*$ unconstrained",'Interpreter', 'latex', 'Fontsize', 10);
ylabel('$u^* [N] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal u* constrained vs u* unconstrained")
hold off

% figure(2)
subplot(3, 1, 2);
stairs(x_constrained(1,:),'r','LineWidth', 2)
hold on
stairs(x_unconstrained(1,:),'b','LineWidth', 1.5)
legend("$x_1$ constrained","$x_1$ unconstrained",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_1 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x1 constrained vs x1 unconstrained")
hold off

% figure(3)
subplot(3, 1, 3);
stairs(x_constrained(2,:),'r','LineWidth', 2)
hold on
stairs(x_unconstrained(2,:),'b','LineWidth', 1.5)
legend("$x_2$ constrained","$x_2$ unconstrained",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_2 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x2 constrained vs x2 unconstrained")
hold off

xlabel('Discrete Time Steps','fontsize',12,'Interpreter', 'latex', 'FontSize', 12);


% print ex2_hw3_me599_part_a -dpng;

