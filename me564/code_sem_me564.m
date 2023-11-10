
%% hw1

clear;
clc;
v1 = [-1;-9;0];
v2 = [1;3;0];
v3 = [2;-2;1];
A = [v1 v2 v3];
rk = rank(A);
A_rref = rref(A);
fprintf("Rank for matrix A is %f\n",rk);
disp(A_rref);
fprintf("Det for matrix A is %f\n",det(A));

clear;
clc;
B1 = [1 1;1 -1];
B2 = [ 2 1;1 -2];
v = [20;12];
vb1 = mldivide(B1,v);
vb2 = mldivide(B2,v);
P = [1.5 -0.5;0.5 1.5];
% disp(vb1);
disp(vb2);
disp(P*vb2 - vb1);
%% hw2

%% hw3

clear;
clc;
A = [1 2;-2 4];
A_T = transpose(A);
M = mtimes(A_T,A);
[V,D] = eig(M);
disp(D);
d = eigs(M,1);
spectralNorm_A = sqrt(d);
disp(spectralNorm_A);

%%
A = [1 2;-2 4];
%Infinity norm
tup = -1:0.01:1;
tdn = 1:-0.01:-1;
x = [tup ones(size(tup)) tdn -1*ones(size(tdn))];
y = [-1*ones(size(tup)) tup ones(size(tdn)) tdn];
v = [x;y];
Av = A*v;
figure(1);
plot(v(1,:),v(2,:),'r',Av(1,:),Av(2,:),'g')
axis equal;
title('Infinity-norm')


%%
A = [1 2;-2 4];
% 1-norm
tup1 = -1:0.01:0;
tup2 = 0:0.01:1;
tdn1 = 0:-0.01:-1;
tdn2 = 1:-0.01:0;
x = [tup1 tup2 tdn2 tdn1];
y = [-(1-abs(tup1)) -(1-abs(tup2)) 1-abs(tdn2) 1-abs(tdn1) ];
v = [x;y];
Av = A*v;
figure(2);
plot(v(1,:),v(2,:),'r',Av(1,:),Av(2,:),'g')
axis equal;
title('1-norm')


%%
A = [1 2;-2 4];
%2-norm

t = 0:0.01:2*pi;
x = cos(t);
y = sin(t);
v = [x;y];
Av = A*v;
figure(3);
plot(v(1,:),v(2,:),'r',Av(1,:),Av(2,:),'g')
axis equal;
title('2-norm')

%%
clear;
clc;
format rat;
A = [1 -2;1 -2;-2 1;1 -3];
b = [-2;5;1;-3];
A_T = transpose(A);
M = mtimes(A_T,A);
disp(M);

if det(M) ~= 0
    fprintf("Sols for least square exists and unique with det A^T A = %f \n",det(M));
    X = inv(M) * A_T * b;
    disp(X);
else 
    fprintf("Do nothing \n");
end
%% hw4


function dx=robot(t,x)
    %states are:%x=[theta1,theta1dot,d2,d2dot]
    theta1=x(1);
    theta1dot=x(2);
    d2=x(3);
    d2dot=x(4);
    %inputs are:
    tau1=0.01*sin(t);
    tau2=0.01*sin(t);
    %computing the derivative
    dx=zeros(4,1);
    dx(1)=theta1dot;
    dx(2)=(-2*d2*theta1dot*d2dot+tau1)/(3+d2^2);
    dx(3)=d2dot;
    dx(4)=d2*theta1dot^2+tau2;
end


%run only after Simulink is run

h_nl = out.h_NonLinear;
p_nl = out.p_nonLinear;
h_ln = out.h_linear;
p_ln = out.p_linear;

plot(h_nl,p_nl,h_ln,p_ln);
title("Quang Nhat Le");
xlabel("Number of predator");
ylabel("Number of prey");
legend("Non-Linear","Linear");

%%
clear;
clc;
%sample codes
A=[0 1 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
B=[0 0;1/12 0;0 0;0 1];
C=[1 0 0 0;0 0 1 0];
D=[0 0;0 0];
linrobot=ss(A,B,C,D);
%note: use zero initial condition
%because the linear variables are the
%variation from the nominal
dx0=[0 0 0 0];
t=0:0.01:20;
u1=0.01*sin(t);
u2=0.01*sin(t);
u=[u1;u2];
[ylin,t]=lsim(linrobot,u,t,dx0);
%add 3 to d2 to get absolute value
theta1lin=ylin(:,1);
d2lin=ylin(:,2)+3;
x0=[0 0 3 0];
[tn,xnl]=ode45('robot',[0 20],x0);
theta1nl=xnl(:,1);
d2nl=xnl(:,3);
subplot(2,1,1);
plot(tn,theta1nl,'-',t,theta1lin,'-.')
xlabel('time');ylabel('theta1')
title('input=0.01*sin(t)')
subplot(2,1,2);
plot(tn,d2nl,'-',t,d2lin,'-.')
xlabel('time');ylabel('d2')

%% hw5
clc;
clear;
close;
format default;

 A = [2 1 1;0 3 1;0 -1 1];
[V,D] = eig(A);
v = null(A - 2 * eye(3),'r');
B = A - 2*eye(3);
%choose v2
v2 = [0;1;0];
v1 = B * v2;

M = [v1,v2,v(:,1)];

% 2 subblocks, 1 Jordan block
% J = 2 1 0
%     0 2 0
%     0 0 2
J = [2 1 0;0 2 0; 0 0 2];

%should be A again
A_new = M * J * inv(M);

disp(A_new);
disp(A);


%% all the time
close all;
clear;
clc;
format default;

%set of unit vectors in R2 anchored 
% at ther origin 
t = 0:0.01:(2*pi);
x = cos(t);
y = sin(t);
v = [x;y];

%matrix A
A = [2 3;3 1];
B = [cos(pi/8) -sin(pi/8);sin(pi/8) cos(pi/8)];
C = [1 -1;2 -1];

%operate
Av = A*v;
Bv = B*v;
Cv = C*v;

%plot
visualise_matrix(1,A,Av,v,'A = [2 3;3 1]');
print ex1_a -dpng;
visualise_matrix(2,B,Bv,v,'A = [cos(pi/8) -sin(pi/8);sin(pi/8) cos(pi/8)]');
print ex1_b -dpng;
visualise_matrix(3,C,Cv,v,'A = [1 -1;2 -1]');
print ex1_c -dpng;


function visualise_matrix(i,m,M,v,tit)
    figure(i);
    plot(v(1,:), v(2,:),'g',M(1,:),M(2,:),'r');
    title(tit);
    grid on;
    axis equal;
    [V,D] = eig(m);
    fprintf("Eigenvectors of A are columns of this matrix:  \n");
    disp(V);
    fprintf("Eigenvalues of A are entry diagonal of this matrix: \n");
    disp(D);
    fprintf("\n");
end

%% hw6
% ex1
clear;
clc;
format default;

A = [0.3536 0 0.25 -0.25;0 -1.4142 -1 -1;0.6124 0 0.433 -0.433];
disp(A);

[U,S,V] = svd(A); % not use S in here

%part a
V1 = V(:, 1:2);
fprintf("Orthonormal basis V1 is : \n");
disp(V1);

%part b
V2 = V(:,3:4);
fprintf("Orthonormal basis V2 is : \n");
disp(V2);

%part c
[V,D] = eig(A.' * A);
fprintf("Singular values of A are : \n");
diagonal = diag(sqrtm(D));
sig_2 = diagonal(3,1)
sig_1 = diagonal(4,1)
S = [sig_1 0; 0 sig_2];
fprintf("Verify that: \n");
fprintf("Right hand side is :\n");
svd_econ = [V1 V2]*[S^2 zeros(2,2);zeros(2,2) zeros(2,2)]*[V1.';V2.'];
disp(svd_econ);
fprintf("A.' * A is :\n");
disp(A.' * A);

%part d
U1 = A * V1 * inv(S);
fprintf("U1 is :\n");
disp(U1);

fprintf("Verify orthonormal: \n");
U11 = U1(:,1);
U12 = U1(:,2);
fprintf("Inner product of each column: \n");
dot(U11,U11)
dot(U11,U12)
dot(U12,U12)

%part e
fprintf("U is: \n ");
disp(U);
fprintf("Choose U2 is: \n");
U2 = U(:,3);
disp(U2);

%final part
b = [1;1;1];
%compute least square
fprintf("Since one has A = U1*S*V1.' therefore: \n");
x = V1 * S^(-1) * U1.' * b;
x

%% 
close all;
clear;
clc;
format default;

A = [0.0974 -0.1178 0.7876 -0.1168 0.0178;
     0.1291 -0.1174 1.2850 0.0302 0.0971;
     0.0528 0.1119 0.1325 0.7668 0.0637;
     0.0424 0.2647 0.2806 1.7644 0.1195];

[U,S,V] = svd(A);
U_r = U(:,1:2)
S_r = S(1:2,1:2)
V_r = V(:,1:2)
fprintf("Therefore the reconstruction matrix is : \n");
A_r = U_r * S_r * V_r.'

%frobenius norm
fprintf("Frobenius norm of the error between the original A matrix " + ...
    "and a reconstruction of A is : \n");
err = norm(A- A_r,'fro')

%% 

close all;
clear;
clc;
format default;

%state space matrices
A = [-2.6   0.25  -38    0;
    -0.075 -0.27  4.4    0;
     0.078 -0.99 -0.23 0.052;
     1     0.078  0      0 ];
B = [17     7;
     0.82 -3.2;
     0    0.046;
     0      0];

%simulated time
t = 0:0.01:30;

%% part a
%output 
C = eye(4);
D = zeros(4, 2);

%initial condition
x0 = zeros(4,1);

%state space object
sys = ss(A,B,C,D);

%for (i)
% u1 = (t>=0)-(t>1); % step input 
% u2 = zeros(size(t));
% u = [u1;u2];

%for (ii)
u1 = zeros(size(t));
u2 = (t>=0)-(t>1);
u = [u1;u2];

%simulated output of the system
[y,t] = lsim(sys,u,t,x0);

%plot
figure(1); 
plot(t, y,'LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Outputs X(x1,x2,x3,x4)');
legend('x_1', 'x_2', 'x_3', 'x_4');
% title('System Response Prob 1. a. i.');
title('System Response Prob 1. a. ii.');
% print ex3a_i -dpng;
% print ex3a_ii -dpng;

%% part b

%rudder position u2
%sideslip angle x3
%fed-back controller state space form
Ac = [0 1;-7.29 -4.05];
Bc = [0;1];
Cc = [10 0];
Dc = 0;

%construct new augmented dynamic system
A_aug = [A B(:,2)*Cc;
         Bc * [0 0 1 0],Ac];
B_aug = [B(:,1);0;0];
C_aug = eye(6);
D_aug = zeros(6,1);

%initial condition
x0 = zeros(6,1);

%state space object
sys = ss(A_aug,B_aug,C_aug,D_aug);

% one-second step input in
% aileron position
u1 = (t>=0)-(t>1);

%simulated output of the system
[y,t] = lsim(sys,u1,t,x0);

%plot
figure(2); 
plot(t, y,'LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Outputs X(x1,x2,x3,x4)');
legend('x_1', 'x_2', 'x_3', 'x_4','xc_1','xc_2');
title('System Response Prob 1. b.');
% print ex3b -dpng;

%% 

close all;
clear;
clc;
format default;

%parameters
M = 0.5;
m =0.2;
b =0.1;
I =0.006;
g =9.8;
l =0.3;

%state space
J_denom = I*(M+m) + M*m*(l^2);
A = [0          1                0                          0;
     0 -((I+m*(l^2))*b)/J_denom  ((m^2)*g*(l^2))/J_denom    0;
     0          0                0                          1;
     0     (-m*l*b)/J_denom       (m*g*l*(M+m))/J_denom     0];

B = [0;
     (I+m*l^2)/J_denom;
     0;
     m*l/J_denom];

%part a
%A and B as above
fprintf("A is: \n");
disp(A);
fprintf("B is: \n");
disp(B);

%part b
[Va,Da] = eig(A);
fprintf("Eigenvalues of A are: \n");
disp(diag(Da));

%part c
fprintf("Singular values of B are: \n");
disp(svd(B));

%part d
%Notice that this is LTI system so the
% state transition matrix is exponential of At
%solution is equal to ZIR (homogeneous) solution plus
% ZSR(particular) solution
fprintf("State transition matrix (STM) is: \n");
fprintf("In the case of t = 0.1: \n");
fprintf("e^(A*(t-0)) = \n");
disp(expm(A*0.1));

fprintf("In the case of t = 1: \n");
fprintf("e^(A*(t-0)) = \n");
disp(expm(A*1));

%part e
%simulated time
t = 0:0.01:3;

%output matrices
%since state vector is X = [x x_dot phi phi_dot]
%we need x and phi
C = [1 0 0 0;
    0 0 1 0];
D = zeros(2,1);

%creates a continuous-time state-space model object
sys = ss(A,B,C,D);

%step input
y = step(sys,t);

%plot
figure(1); 
plot(t, y,'LineWidth',2);
grid on;
xlabel('Time (s)');
ylabel('Outputs x and \phi');
legend('x', '\phi');
axis([0 3 0 40]);
title('Simulated system Response with unit step input Prob 5e');
print ex5e_plot -dpng;

%% 

clear;
clc;
close;
format default;

syms lamda delta_t;

%by inspection
m1 = [1;-3]; %lamda = 1
m2 = [2;1]; %lamda2 (unstable, negative)

M = [m1 m2];

%choose 2 random coordinates 
% in the same trajectory
x_t1 = [-8;2];
x_t2 = [-6;4];

J = [1 0;0 lamda];
e_Jt = expm(J*delta_t);
eqn = M* e_Jt * inv(M) * x_t1 == x_t2; 
S = solve(eqn)

J_new = subs(J,lamda,-2.932); %S.lamda

%matrix A
A = M * J_new * inv(M);

fprintf("Matrix A is: \n");
disp(A);

%% 

close all;
clear;
clc;
format default;

%start here
A = [0    1.0000         0         0;
     0   -0.1818    2.6727         0;
     0         0         0    1.0000;
     0   -0.4545   31.1818         0];

%part a
[Va,Da] = eig(A);
fprintf("Eigenvalues of A are: \n");
disp(diag(Da));


%part b
fprintf("Eigenvectors of A are: \n");
disp(Va);

%part c
fprintf("Transformation matrix P is: \n");
disp(Va);




%% hw7
%% hw8

% ex1
clear; close; clc;format default;

% S1
fprintf("----S1----- \n");
A = [1 2 1;0 4 3;0 0 2];
B = [1 0 1].';
C = [1 0 0];

% S2
% fprintf("----S2----- \n");
% A = [2 1 1;5 3 6;-5 -1 -4];
% B = [1 0 0].';
% C = [1 1 2];

% Controllable test
% Kalman rank test
Co_hand = [B A*B A^2*B];
Co = ctrb(A,B);
% disp(Co);
unco = length(A) - rank(Co);
% disp(unco);

% Observable test
Ob_hand = [C;C*A;C*A^2];
Ob = obsv(A,C);
unobsv = length(A) - rank(Ob);

% Jordan form
[V,J] = jordan(A);

% Canonical form
A_bar = J
B_bar = inv(V) * B;
C_bar = C * V;

% state space model
sys = ss(A_bar,B_bar,C_bar,0);

% transfer function
syms s

H_normal= C * inv(s*eye(length(A)) - A) * B
H_jor = C_bar * inv(s*eye(3) - A_bar) * B_bar
H_tf = tf(sys)

% Pole of dynamic system
pole(H_tf)
% pole(sys)

%% 

% ex2
clear; close; clc;format default;

M = 10;
m = 1;
l1 = 1;
l2 = 1;
g = 1;

A = [0 0 1 0;
    0 0 0 1;
    (M+m)*g/(M*l1) m*g/(M*l1) 0 0;
    m*g/(M*l2) (M+m)*g/(M*l2) 0 0];

B = [0;0;-1/(M*l1);-1/(M*l2)];

C = [1 0 0 0];

% Controllable test
% Kalman rank test
Co_hand = [B A*B A^2*B (A^3)*B];
Co = ctrb(A,B);
fprintf("Controllability matrix: \n");
disp(Co);
unco = length(A) - rank(Co);
disp(unco);
fprintf("Rank controllability matrix : \n");
disp(rank(Co));
% det(Co)

% Observable test
fprintf("Observability matrix: \n");
Ob_hand = [C;C*A;C*A^2;C*A^3]
Ob = obsv(A,C)
unobsv = length(A) - rank(Ob)
det(Ob)

%% hw9


%ME564 HW10 P1
clc
clear all
close all

T=1; 
t=0; 
Wr=[T^3/3 T^2/2; T^2/2 T];
expAT=[1 T; 0 1];
B=[0; 1];

x0=[1 1]'; x1=[-1 -1]';

%calculation for original system
for i=1:101
 expAt=[1 T-t; 0 1];  % computes expAt as a function of t (j)
 u(i)=(expAt*B)'*inv(Wr)*(x1 - expAT*x0);
 u2(i)=[T-t, 1]*[12/T^3 -6/T^2; -6/T^2 4/T]*(x1-[1 T; 0 1]*x0); 
 u3(i)=24*t-14; 
 xc(i,1:2) = [4*t^3-4*t^2+t+1; 12*t^2-8*t+1];
 tvec(i)=t; 
 t=t+0.01;
end

% adjoint system
Wr_adj = -[T T^2/2; T^2/2 T^3/3];
expAT_adj = [1 0; T 1];
C=[1 0]; %=B';
t=0;
for j=1:101
    expAt = [1 t; 0 1];
    v(j) = -C*expAt*inv(Wr_adj)*(x1 - expAT_adj*x0);
    v2(j) = -[1 t]*[-4 6;6 -12]*(x1 - expAT_adj*x0);
    v3(j) = 10- 24*t; 
    pc(j,1:2) = [-15*t^2+14*t+2; 5*t^3-7*t^2-2*t+5];
    tvec(j)=t;
    t=t+0.01;  
end

A=[0 1; 0 0]; B=[0; 1]; C=[1 0]; D=[0]; 
sys=ss(A,B,C,D); 
[y,t,x]=lsim(sys,u,tvec,x0);  % replaces t above to be tvec. 

sys_adj = ss(-A',-C',B',D);
[z,t,p]=lsim(sys_adj,v,tvec,x1);

% figure(1);
% plot(t,x,t,xc,'o'); figure(gcf);
% title('state over time for original system')
% xlabel('t'); ylabel('state'); legend('x','xc')
% 
% figure(2);
% plot(t,p,t,pc,'o');
% title('state over time for adjoint system')
% xlabel('t'); ylabel('state'); legend('p','pc')
% 
% figure(3); 
% plot(t,u,t,u3,'o',t,v,t,v3,'o'); %,t,dot(B*u,p')+dot(x',C'*v)); 
% legend('u','u3','v','v3');
% title('the open-loop control inputs'); 
% 
% figure(4); plot(t,x,t,xc,'o',t,p,t,pc,'o'); 
% figure(gcf); title('all the states'); 

figure(1);
plot(t,x)
title('state over time for original system')
xlabel('t'); ylabel('state'); 
print -dpng ex1b_plot_hw9_me564

figure(2);
plot(t,p);
title('state over time for adjoint system')
xlabel('t'); ylabel('state'); 
print -dpng ex1d_plot_hw9_me564

figure(3); plot(t,x,t,p); 
title('all the states'); xlabel('t'); ylabel('state'); 
print -dpng ex1d_and_b_plot_hw9_me564

%% 

clear; close; clc;format default;

A = [3 3 0 2;
 0 87 0 60;
 6 3 -3 2;
 0 -126 0 -87];
B = [0;3;-1;4];

% Controllable test
% Kalman rank test
Co_hand = [B A*B A^2*B (A^3)*B];
Co = ctrb(A,B);
fprintf("Controllability matrix: \n");
disp(Co);
unco = length(A) - rank(Co);
disp(unco);
fprintf("Rank controllability matrix : \n");
disp(rank(Co));
% det(Co)


%%

v4 = null(Co,'r')

V = [Co(:,1:3) v4];
% V = [B A*B A*A*B [1 0 0 0]']

AK = inv(V)*A*V
BK = inv(V)*B;
A1K = AK(1:3,1:3);
B1K = BK(1:3);
P = [-3 -2 -2];
Kk = acker(A1K,B1K,P)
% Kk = place(A1K,B1K,P)

K = [Kk 0]*inv(V)

%% 

clear; close; clc;format default;

A = [0 1;0 -5]
B = [0;-50]

eta = 0.707;
wn = 10;

coef = [1 2*eta*wn wn^2]
r = roots(coef)

P =r.'

K = place(A,B,P)

clear; close; clc;format default;

% parameters
A = [ 0   1   0
     980  0  -2.8
      0   0  -100 ];

B = [ 0
      0
      100 ];

C = [ 1 0 0 ];

%state space model
sys = ss(A,B,C,0);

%check stability
poles = eig(A);

% check Controllability and Observability
rank(ctrb(A,B))
rank(obsv(A,C))

% design pole placement
p1 = -20 + 20i;
p2 = -20 - 20i;
p3 = -100;

K = place(A,B,[p1 p2 p3]);

% scale factor
Nbar = rscale(sys,K);

% fast observer design
op1 = -100;
op2 = -101;
op3 = -102;

L = place(A',C',[op1 op2 op3])';

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B*Nbar
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

%new state space with observer
sys = ss(At,Bt,Ct,0);

% response
t = 0:0.01:2;
x0 = [0.01 0 0];

figure();
hold on;

% [y,t,x] = lsim(sys,zeros(size(t)),t,[x0 x0]);
lsim(sys,zeros(size(t)),t,[x0 x0]);


% n = 3;
% e = x(:,n+1:end);
% x = x(:,1:n);
% x_est = x - e;
% 
% % Save state variables explicitly to aid in plotting
% h = x(:,1); 
% h_est = x_est(:,1); 
% 
% plot(t,h,'-r',t,h_est,':r')
% grid on
% title('Linear Simulation Results (with observer)')
% legend('h','h_{est}')
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')

%slow observer design
op1 = -20;
op2 = -21;
op3 = -22;

L = place(A',C',[op1 op2 op3])';

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B*Nbar
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys = ss(At,Bt,Ct,0);

lsim(sys,zeros(size(t)),t,[x0 x0]);

title('Linear Simulation Results (with observer)')
xlabel('Time (sec)')
ylabel('Ball Position (m)')
legend('fast','slow')
print -dpng ex7_plot_fast_vs_slow_observer_poles_hw9_me564

%% 

clear; close; clc;format default;

% state space tutorial
% everything can be found here
% https://ctms.engin.umich.edu/CTMS/?example=Introduction&section=ControlStateSpace

% parameters
A = [ 0   1   0
     980  0  -2.8
      0   0  -100 ];

B = [ 0
      0
      100 ];

C = [ 1 0 0 ];

%check stability
poles = eig(A)

%test unstable poles when has inittial condition
t = 0:0.01:2;
u = zeros(size(t));
x0 = [0.01 0 0];

sys = ss(A,B,C,0);
% 
% [y,t,x] = lsim(sys,u,t,x0);
% plot(t,y)
% title('Open-Loop Response to Non-Zero Initial Condition')
% xlabel('Time (sec)')
% ylabel('Ball Position (m)')

% check Controllability and Observability
rank(ctrb(A,B))
rank(obsv(A,C))

% design pole placement
% p1 = -10 + 10i;
% p2 = -10 - 10i;
% p3 = -50;

p1 = -20 + 20i;
p2 = -20 - 20i;
p3 = -100;

K = place(A,B,[p1 p2 p3]);
sys_cl = ss(A-B*K,B,C,0);

lsim(sys_cl,u,t,x0);
xlabel('Time (sec)')
ylabel('Ball Position (m)')


% 
% Note: If you want to place two or more poles at the same position, place will not work. You can use a function called acker which achieves the same goal (but can be less numerically well-conditioned):
% 
% K = acker(A,B,[p1 p2 p3])

% step input
u = 0.001*ones(size(t));

sys_cl = ss(A-B*K,B,C,0);

lsim(sys_cl,u,t);
xlabel('Time (sec)')
ylabel('Ball Position (m)')
axis([0 2 -4E-6 0])

% scale factor
Nbar = rscale(sys,K)


lsim(sys_cl,Nbar*u,t)
title('Linear Simulation Results (with Nbar)')
xlabel('Time (sec)')
ylabel('Ball Position (m)')
axis([0 2 0 1.2*10^-3])

% observer design
op1 = -100;
op2 = -101;
op3 = -102;

L = place(A',C',[op1 op2 op3])';

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B*Nbar
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

sys = ss(At,Bt,Ct,0);
lsim(sys,zeros(size(t)),t,[x0 x0]);

title('Linear Simulation Results (with observer)')
xlabel('Time (sec)')
ylabel('Ball Position (m)')

t = 0:1E-6:0.1;
x0 = [0.01 0.5 -5];
[y,t,x] = lsim(sys,zeros(size(t)),t,[x0 x0]);

n = 3;
e = x(:,n+1:end);
x = x(:,1:n);
x_est = x - e;

% Save state variables explicitly to aid in plotting
h = x(:,1); h_dot = x(:,2); i = x(:,3);
h_est = x_est(:,1); h_dot_est = x_est(:,2); i_est = x_est(:,3);

plot(t,h,'-r',t,h_est,':r',t,h_dot,'-b',t,h_dot_est,':b',t,i,'-g',t,i_est,':g')
legend('h','h_{est}','hdot','hdot_{est}','i','i_{est}')
xlabel('Time (sec)')




