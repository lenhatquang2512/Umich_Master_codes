%% Problem 3
clear;clc;close all; format default;

%% part a
%spring-mass-damper system parameters
m = 1;
k = 1;
b = 0.5;

%state space model
Ac = [0 1; -k/m -b/m];
Bc = [0 1/m;
    0 1/m]'; %for both u and d 
Cc = [1 0];
Dc = 0;

T = 0.05;   % time step
 
cts_sys = ss(Ac,Bc,Cc,Dc);

% zero order approximation
sys_zoh = c2d(cts_sys,T,'zoh');
[Aaug,Baug,Caug,Daug] = ssdata(sys_zoh) ;% get the discrete matrices back;

%ZOH discrete time space ss representation

A = Aaug
Bc = Baug(:,1)
Bd = Baug(:,2)

%% part b - NO disturbance

B = Bc; % NO disturbance
% Define time step and number of steps
T = 0.05;   % time step
N = 100;

% Define initial state
x = zeros(2, N+1); 
x0 = [10 0]';
x(:,1) = x0;

% Weight matrices ,cost function parameters
R = 1;
Q = [100 0; 0 1];

% Offline optimization solution (option 1)
% %   Compute the entries to the lifted system matrix
H = zeros(2*N,N);  %   Initialize H
for i=1:N
    for j=1:i
        H(2*i-1:2*i,j) = A^(i-j)*B;
    end
end

Q_bar = R*eye(N);
r = zeros(1,N);
for i=1:N
    Q_bar = Q_bar + H(2*i-1:2*i,:)'*Q*H(2*i-1:2*i,:);
    r = r + 2*x0'*(A^i)'*Q*H(2*i-1:2*i,:);
end
u_opt_offline = -0.5*inv(Q_bar)*r';

% Compute x1 and x2 from applying this optimized control sequence
x_offline = x;
for i = 1:N
    x_offline(:, i + 1) = A * x_offline(:, i) + B * u_opt_offline(i);
end

%Feedback control solution (option 2) N is infinite

K = dlqr(A,B,Q,R);  % N = Inf

%   For comparison purposes, calculate the control trajectory under the
%   linear control law specified in the problem
u_opt_feedback = zeros(N,1); 

% Finding discrete state x using linear feedback given K , x(i+1) = (A-BK)x(i)
x_feedback = zeros(length(x0), N+1);
x_feedback(:, 1) = x0;
for i = 1:N
    x_feedback(:,i+1) = (A - B*K)*x_feedback(:,i);
end

% Control sequence that results from the feedback control law
for i = 1:N
    u_opt_feedback(i) = -K*(x_feedback(:,i));
end


%Plot
time_sequence = 0 : T : N* T;
figure(1)
subplot(3, 1, 1);
stairs(time_sequence(1:100),u_opt_offline,'r','LineWidth', 2);
hold on 
stairs(time_sequence(1:100) ,u_opt_feedback,'b','LineWidth', 1.5)
legend("$u^*$ offline","$u^*$ feedback",'Interpreter', 'latex', 'Fontsize', 10);
ylabel('$u^* [N] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal u* offline vs u* feedback")
hold off

% figure(2)
subplot(3, 1, 2);
stairs(time_sequence ,x_offline(1,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,x_feedback(1,:),'b','LineWidth', 1.5)
legend("$x_1$ offline","$x_1$ feedback",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_1 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x1 offline vs x1 feedback")
hold off

% figure(3)
subplot(3, 1, 3);
stairs(time_sequence ,x_offline(2,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,x_feedback(2,:),'b','LineWidth', 1.5)
legend("$x_2$ offline","$x_2$ feedback",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_2 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x2 offline vs x2 feedback")
hold off

xlabel('Time [s]','fontsize',12,'Interpreter', 'latex', 'FontSize', 12);

% print ex3b_hw4_me599_no_disturbance -dpng;

%% part c - HAVE disturbances, repeat part b

d = zeros(1,N);
for i =  1 : N
    d(i) = -10;
end

% Compute x1 and x2 from applying this optimized control sequence
x_offline = x;
for i = 1:N
    x_offline(:, i + 1) = A * x_offline(:, i) + Bc * u_opt_offline(i) + Bd * d(i);
end

%   For comparison purposes, calculate the control trajectory under the
%   linear control law specified in the problem
u_opt_feedback = zeros(N,1); 

% Finding discrete state x using linear feedback given K , x(i+1) = (A-BK)x(i)
x_feedback = zeros(length(x0), N+1);
x_feedback(:, 1) = x0;
for i = 1:N
    x_feedback(:,i+1) = (A - Bc*K)*x_feedback(:,i) + Bd * d(i);
end

% Control sequence that results from the feedback control law
for i = 1:N
    u_opt_feedback(i) = -K*(x_feedback(:,i));
end

% true cost function value for offline
J_offline = 0;
for i = 1:N
    J_offline =  J_offline + x_offline(:,i+1).'* Q * x_offline(:,i+1) + R * u_opt_offline(i)^2;
end
J_offline

% true cost function value for feedback
J_feedback = 0;
for i = 1:N
    J_feedback =  J_feedback + x_feedback(:,i+1).'* Q * x_feedback(:,i+1) + R * u_opt_feedback(i)^2;
end
J_feedback

%Plot
time_sequence = 0 : T : N* T;
figure(2)
subplot(3, 1, 1);
stairs(time_sequence(1:100) ,u_opt_offline,'r','LineWidth', 2);
hold on 
stairs(time_sequence(1:100) ,u_opt_feedback,'b','LineWidth', 1.5)
legend("$u^*$ offline","$u^*$ feedback",'Interpreter', 'latex', 'Fontsize', 10);
ylabel('$u^* [N] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal u* offline vs u* feedback")
hold off

% figure(2)
subplot(3, 1, 2);
stairs(time_sequence ,x_offline(1,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,x_feedback(1,:),'b','LineWidth', 1.5)
legend("$x_1$ offline","$x_1$ feedback",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_1 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x1 offline vs x1 feedback")
hold off

% figure(3)
subplot(3, 1, 3);
stairs(time_sequence ,x_offline(2,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,x_feedback(2,:),'b','LineWidth', 1.5)
legend("$x_2$ offline","$x_2$ feedback",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_2 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x2 offline vs x2 feedback")
hold off

xlabel('Time [s]','fontsize',12,'Interpreter', 'latex', 'FontSize', 12);

% print ex3c_hw4_me599_HAVE_disturbance -dpng;
%% part d

% Min and Max values of u and x 
x1max = 10;
x2max = 30;
x1min = -x1max;
x2min = -x2max;
umax = 40;
umin = -umax;

% Define initial state
n = size(A,1);
m = size(B,2);

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
        A2(n*(i-1)+1:n*i,N*n+m*(i-1)+1:N*n+m*i) = -Bc;
    else
        A2(n*(i-1)+1:n*i,n*(i-1)+1:n*i) = eye(n);
        A2(n*(i-1)+1:n*i,N*n+m*(i-1)+1:N*n+m*i) = -Bc;
        A2(n*(i-1)+1:n*i,n*(i-2)+1:n*(i-1)) = -A;
    end
end

%   Use quadprog to determine the optimal control sequence
% options = optimoptions('quadprog','Algorithm','active-set');
% u0 = zeros((n+m)*N,1);
[w_opt_QP,J_opt,exitflag,output] = quadprog(2*H,zeros(N*(m+n),1),A1,b1,A2,b2);
u_opt_QP = w_opt_QP(N*n+1:N*(n+m));

% Compute x1 and x2 from applying this optimized control sequence
x_constrained_QP = zeros(n, N+1); 
x_constrained_QP(:,1) = x0;
for i = 1:N
    x_constrained_QP(:, i + 1) = A * x_constrained_QP(:, i) + Bc * u_opt_QP(i) + Bd * d(i);
end

%Plot
time_sequence = 0 : T : N* T;
figure(3)
subplot(3, 1, 1);
stairs(time_sequence(1:100) ,u_opt_QP,'r','LineWidth', 2);
ylabel('$u^* [N] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal u* constrained using QP")
hold off

% figure(2)
subplot(3, 1, 2);
stairs(time_sequence ,x_constrained_QP(1,:),'r','LineWidth', 2)
ylabel('$x_1 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x1 constrained ")
hold off

% figure(3)
subplot(3, 1, 3);
stairs(time_sequence ,x_constrained_QP(2,:),'r','LineWidth', 2)
ylabel('$x_2 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x2 constrained")
hold off

xlabel('Time [s]','fontsize',12,'Interpreter', 'latex', 'FontSize', 12);
% print ex3d_hw4_me599_constrained_QP -dpng;

%% part e MPC

%Only run this part when Simulink already run
load ex3e_mpc_hw4_me599_mat_Data.mat

%Plot
time_sequence = 0 : T : N* T;
figure(4)
subplot(3, 1, 1);
stairs(time_sequence(1:100) ,u_opt_QP,'r','LineWidth', 2);
hold on 
stairs(time_sequence ,out.u_opt_mpc.Data,'b','LineWidth', 1.5)
legend("$u^*$ QP","$u^*$ MPC",'Interpreter', 'latex', 'Fontsize', 10);
ylabel('$u^* [N] $','Interpreter', 'latex', 'FontSize', 12);
title("Optimal u* QP vs u* MPC")
hold off

% figure(2)
subplot(3, 1, 2);
stairs(time_sequence ,x_constrained_QP(1,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,out.x1_mpc.Data,'b','LineWidth', 1.5)
legend("$x_1$ QP","$x_1$ MPC",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_1 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x1 QP vs x1 MPC")
hold off

% figure(3)
subplot(3, 1, 3);
stairs(time_sequence ,x_constrained_QP(2,:),'r','LineWidth', 2)
hold on
stairs(time_sequence ,out.x2_mpc.Data,'b','LineWidth', 1.5)
legend("$x_2$ QP","$x_2$ MPC",'Interpreter', 'latex', 'Fontsize', 10)
ylabel('$x_2 [m] $','Interpreter', 'latex', 'FontSize', 12);
title("x2 QP vs x2 MPC")
hold off

xlabel('Time [s]','fontsize',12,'Interpreter', 'latex', 'FontSize', 12);

% print ex3e_hw4_me599_MPC -dpng;

























 





































