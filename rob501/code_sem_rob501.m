%% hw1
clc;
clear;
loop = 10 ;
 n = 4;
 m = 3;
 A = cell(n, m) ;
 for k = 1 : loop

     %form 10 matrices%
     A = rand(n,m);
     fprintf("Matrix %d: \n",k);
%      disp(A);
     
     %part i%
     A_trans = transpose(A);
     M = mtimes(A_trans,A);
     disp(M);
     
     %part ii%
     [V,D] = eig(M);

     fprintf("Eigenvalue of matrix %d: \n",k);
     disp(D);

     fprintf("Eigenvector of matrix %d: \n",k);
     disp(V);

     %part iii%
     V_trans = transpose(V);
     Inner_Product_Matrix = mtimes(V_trans,V);

     fprintf("Inner product matrix for matrix %d: \n",k);
     disp(Inner_Product_Matrix);

     %part iv%
     sum_e_values = sum(diag(D(:,1:end)));
     trace_value = trace(M);
     eva = ismembertol(sum_e_values,trace_value,0.01);

     fprintf("Compare result is %d\n",eva);

     %part v%
     prod_e_values = prod(diag(D(:,1:end)));
     det_value = det(M);
     eva_2 = ismembertol(prod_e_values,det_value,0.01);

     fprintf("Compare result is %d\n",eva_2);
     
 end

mu1 = 0;
sigma1 = 1;
mu2 = 0;
sigma2 = 3;
x = -10:.1:10;
y1= pdf('Normal',x,mu1,sigma1);
y2= pdf('Normal',x,mu2,sigma2);
plot(x,y1,'g',x,y2,'r')


clc;
mu = 2;
sigma = 5;
fun = @(x) 1/(sqrt(2*pi)*sigma)*exp(-(x-mu).^2/(2*sigma^2));

q1 = integral(fun,4,Inf);
q2 = integral(fun,-2,4);
q3 = integral(fun,8,100);

fprintf("Answer part i is %f \n",q1);
fprintf("Answer part ii is %f \n",q2);
fprintf("Answer part iii is %f \n",q3+q2);

clc;
clear;
syms x y u_x u_y 
a = -3/2;
b = sqrt(5);
% u_x = 1;
% u_y = 2;

pdf = exp(-(x-u_x)^2 +a *(y-u_y)^2 + b*(x-u_x)*(y-u_y)); % not multiply with (1/(2*pi)) yet

f_X = int(pdf,y,-Inf,Inf);
f_Y = int(pdf,x,-Inf,Inf);
% disp(f_X);
disp(f_Y);

clear;
sigma_xly = 1/sqrt(2);
mu_x1y = 1+4*sqrt(5);
x_plot = -20:.1:20;
f_x1y = pdf('Normal',x_plot,mu_x1y,sigma_xly);
plot(x_plot,f_x1y,'g')

%% hw2
%% 3
%% 4
%% hw5

clear;
clc;
A3 = [3 10 0;0 2 4;0 0 1];
[V3,D3] = eig(A3);
disp(V3);
disp(D3);
disp(rref(V3));
disp(det(V3));

fprintf("For A4: \n");

A4 = [5 1 1;0 5 3;0 0 2];
[V4,D4] = eig(A4);
disp(V4);
disp(D4);
disp(rref(V4));
disp(det(V4));


clear;
clc;
close;

load HW05_Prob5_Data;
format default;
[m,n] = size(t);
A = [ones(m,1),t,t.^2,t.^3,t.^4,t.^5,t.^6,t.^7,t.^8];
alpha_ = inv(A.'*A)*A.'*f;
f_fit = A*alpha_;
fprintf("Coefficients are as follow: \n");
% disp(alpha);
[row,col] = size(A);
for i = 1:col
    fprintf("alpha(%d) = %f\n",i,alpha_(i));
end
plot(t,f,'r',t,f_fit,'g',"LineWidth",2.5);
grid on 
xlabel('t','FontSize',10);
ylabel('f','FontSize',10);
legend('Data','Fit curve');
title('Least square fit of polynomials up to order 8 to data');
print Prob5poly -dpng;

df_dt = get_poly_deriv(alpha_,0.3);
fprintf("The derivative of f(t) at t = 0.3 is :\n");
disp(df_dt);
function df = get_poly_deriv(a,val)
    p = flip(a).';
    k = polyder(p);
    len = length(a)-2;
    coef = flip(k);
    df = 0;
    for i = 0:len
        df =df + coef(i+1) * val^i;
    end
end
%% hw6
clear;close all
clc;
%% Normal Equaitons
syms t real
% y1 = 1; y2 = t; y3 = 1/2*(3*t^2-1);
% x = exp(t);
y1 = [1 0; 2 0] ; y2 = [1 1; 1 1];
x = [0 -1; 2 0];
int_a = -1; int_b = 1;

%% Find alpha
y11 = trace(y1'*y1)
y12 = trace(y1'*y2)
y22 = trace(y2'*y2)
xy1 = trace(x'*y1)
xy2 = trace(x'*y2)

G = [y11 y12; y12 y22]
beta = [xy1; xy2]
alpha = (G')^-1*beta

xhat = alpha(1)*y1+alpha(2)*y2

%%
clc;
clear;
close all;
format rat;
load DataHW06_Prob2.mat;

%% initial
% This makes the plot
% plot(t,y,'r','linewidth',3);
% hold on;
plot(t,dy,'r','linewidth',3);
hold on;
grid on 
% xlabel('t','FontSize',10);
% ylabel('y','FontSize',10);
% print Prob2hw6_initialWave -dpng;

%% 2a
%naive estimate
del_T = t(2) - t(1);
dy_naive = zeros(length(t),1);
for i = 1:length(t)
    if i == 1
        dy_naive(i) = y(i)/ del_T;
    else
        dy_naive(i) = (y(i) - y(i-1))/ del_T;
    end
end

plot(t,dy_naive,'g','linewidth',3);
legend ({'$\dot y (t)$','$\dot y _{Naive}(t)$'},'Interpreter','latex')
xlabel('t','FontSize',10);
ylabel('dy/dt','FontSize',10);
print Prob2hw6a_naive -dpng;

%% 2b
%regression estimate
del_T = t(2) - t(1);
%choose M and N, N is polynomial fitting
M = 4; N = 2;
mov_win = -M+1 : (-M+M);
A = zeros(M,N+1);

%compute matrix A (constant)
for i = 1:length(mov_win)
    row_A = zeros(1,N+1);
    for j = 0:N
        row_A(j+1) = ((-M+i)*del_T)^j;
    end
    A(i,:) = row_A;
end

%define beta is the coef (ATA)-1 AT
beta = inv(A.'*A)*A.';

%define B
B = zeros(1,N+1);
B(2) = 1;

%compute dy/dt
dy_regress = zeros(length(y),1);
for k = 0:length(y)
    tk = k*del_T;
    if k > (M-1)
            alpha_k = beta * y(k-M+1:k);
            dy_regress(k) = B * alpha_k;
    end
end

plot(t,dy_regress,'g','linewidth',3);
legend ({'$\dot y (t)$','$\dot y _{Regression}(t)$'},'Interpreter','latex')
xlabel('t','FontSize',10);
ylabel('dy/dt','FontSize',10);
print Prob2hw6b_regression -dpng;

%%
clc;
clear;
close all;
format default;
load DataHW06_Prob3.mat;

% This makes the plot
% plot(t,y,'r','linewidth',3);
% hold on;
figure(1);
plot(t,dy,'r','linewidth',3);
hold on;
grid on 
% xlabel('t','FontSize',10);
% ylabel('y','FontSize',10);
% print Prob3hw6_initialWave -dpng;

%naive estimate
del_T = t(2) - t(1);
dy_naive = zeros(length(t),1);
for i = 1:length(t)
    if i == 1
        dy_naive(i) = y(i)/ del_T;
    else
        dy_naive(i) = (y(i) - y(i-1))/ del_T;
    end
end

plot(t,dy_naive,'g','linewidth',2);
legend ({'$\dot y (t)$','$\dot y _{Naive}(t)$'},'Interpreter','latex')
xlabel('t','FontSize',10);
ylabel('dy/dt','FontSize',10);
title("Derivative of y naive estimate");
print Prob3hw6a_naive -dpng;
hold off;

% calculate the RSME
rmse_naive = norm(dy - dy_naive,2)/(length(dy));
fprintf("RMSE for naive estimate is : ");
disp(rmse_naive);

%regression estimate
del_T = t(2) - t(1);
%choose M and N, N is polynomial fitting
M = 12; N = 2;
mov_win = -M+1 : (-M+M);
A = zeros(M,N+1);

%compute matrix A (constant)
for i = 1:length(mov_win)
    row_A = zeros(1,N+1);
    for j = 0:N
        row_A(j+1) = ((-M+i)*del_T)^j;
    end
    A(i,:) = row_A;
end

%define beta is the coef (ATA)-1 AT
beta = inv(A.'*A)*A.';

%define B
B = zeros(1,N+1);
B(2) = 1;

%compute dy/dt
dy_regress = zeros(length(y),1);
for k = 1:M:length(y)
    tk = k*del_T;
    if k > (M-1)
            alpha_k = beta * y(k-M+1:k);
            dy_regress(k) = B * alpha_k;
    end
end

figure(2);
plot(t,dy,'r','linewidth',3);
grid on;
hold on;
plot(t,dy_regress,'g','linewidth',2);
legend ({'$\dot y (t)$','$\dot y _{Regression}(t)$'},'Interpreter','latex')
xlabel('t','FontSize',10);
ylabel('dy/dt','FontSize',10);
title("Derivative of y regression estimate");
print Prob3hw6b_regression -dpng;

% calculate the RSME
rmse_reg = norm(dy - dy_regress,2)/(length(dy));
fprintf("RMSE for regression estimate is : ");
disp(rmse_reg);

% as usual
clear;
clc;
close;
format rat;

%test by using data from ex8 hw5
A = diag([0.5, 1, 1, 0.5, 1]);
B = [3 0 2 0 1].';
C = 0.25;
D = B.';

H = MIL(inv(A),B,C,D);
fprintf("The result matrix is : \n");
disp(H);

%function to implement Matrix Inversion Lemma (MIL)
function res = MIL(A_inv,B,C,D)
    %check C is invertible or not
    if det(C) == 0
        fprintf("C is not invertible\n");
        res = 0;
    else
        res = A_inv - A_inv * B * inv(inv(C) + D* A_inv *B) * D * A_inv;
    end
end

%% hw7

%%
clc;
clear;
close all;
format rat;

a = sqrt(2);
A = [1 0 a;0 2 0; a 0 0];

%part a
v1 = [0;1;0];
% disp(A*v1 - 2*v1);

%part b
v2 = [1;0;0];
v3 = [0;0;1];
V = [v1,v2,v3];

%part c
B = V.' * A * V;
% disp(B);

A2 = B(2:end,2:end);
% disp(A2);

% part d
[U2,D2] = eig(A2);
% disp(U2.' * A2 * U2);

%part e
U = eye(3);
U(2:end,2:end) = U2;
% disp(U * U.');

% part f
O = V * U;
% disp(O * O.');

%part g
lamda = O.' * A * O;
fprintf("The final result is : \n");
disp(lamda);
fprintf("This is a diagonal matrix \n");

%% all the time
close all;
clear;
clc;
format default;
load DataHW07_Prob4.mat;

% m = length(y{1});
[m,n] = size(C{1});

%% part a
%initialise
A{1} = C{1};
Y{1} = y{1};
for k = 1:N
%     S{k} = eye(m);
    R{k} = eye(m*k);
    if k > 1
        A{k} = [A{k-1};C{k}];
        Y{k} = [Y{k-1};y{k}];
        if rank(A{k}) == 100
            n_a = k;
            break;
        end

    end
end

fprintf("n s.t A_k is full rank is %d \n",n_a);

%% part b
tic;
for k = n_a : N
    R{k} = eye(m*k);
    A{k} = [A{k-1};C{k}];
    Y{k} = [Y{k-1};y{k}];
    x_batch{k} = inv(A{k}.' * R{k} * A{k}) * A{k}.' * R{k} * Y{k};
    E_b(k-n_a+1) = norm(x_batch{k} - x_actual{k});
end

t = n_a:N;
plot(t,E_b,'g','linewidth',2);
title("Norm error in x-hat using Batch Process (2.040413 seconds)");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
print Prob4b_hw7 -dpng
toc;


%% part c
%using RLS Algorithm
% -----------initialise step-------------
%compute Qk and Pk , xk at k = n_a
for k = 1:n_a
    S{k} = eye(m);
    if k == 1
        Q{k} = C{k}.' * S{k} * C{k};
        P{k} = C{k}.' * S{k} * y{k};
    else
        Q{k} = Q{k-1} + C{k}.' * S{k} * C{k};
        P{k} = P{k-1} + C{k}.' * S{k} * y{k};
    end
end

x_rls ={};
x_rls{n_a} = inv(Q{n_a}) * P{n_a};

% -----------recursion step -------------
tic;
for k = n_a : N-1
    S{k+1} = S{k};
    Q{k+1} = Q{k} + C{k+1}.' * S{k+1} * C{k+1};
    %Kalman gain
    K{k+1} = inv(Q{k+1}) * C{k+1}.' * S{k+1};

    %update new state with Kalam gain and innovation
    x_rls{k+1} = x_rls{k} + K{k+1} * (y{k+1} - C{k+1} * x_rls{k});
    E_c(k-n_a+1) = norm(x_actual{k} - x_rls{k});

end

%------plot----------------
t = n_a:N-1;
plot(t,E_c,'g','linewidth',2);
title("Norm error in x-hat using RLS without MIL (0.453542 seconds)");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
print Prob4c_hw7 -dpng
toc;





%% part d

%exactly as in part c
%using RLS Algorithm WITH MIL
% -----------initialise step-------------
%compute Qk and Pk , xk at k = n_a
for k = 1:n_a
    S{k} = eye(m);
    if k == 1
        Q{k} = C{k}.' * S{k} * C{k};
        P{k} = C{k}.' * S{k} * y{k};
    else
        Q{k} = Q{k-1} + C{k}.' * S{k} * C{k};
        P{k} = P{k-1} + C{k}.' * S{k} * y{k};
    end
end

x_rls_mil ={};
Q_inv{n_a} = inv(Q{n_a});
x_rls_mil{n_a} =  Q_inv{n_a} * P{n_a};

% -----------recursion step -------------
tic;
for k = n_a : N-1
    S{k+1} = S{k};
    %compute inverse using MIL
    Q_inv{k+1} = MIL(Q_inv{k},C{k+1}.',S{k+1},C{k+1});
    %Kalman gain
    K{k+1} = Q_inv{k+1} * C{k+1}.' * S{k+1};

    %update new state with Kalam gain and innovation
    x_rls_mil{k+1} = x_rls_mil{k} + K{k+1} * (y{k+1} - C{k+1} * x_rls_mil{k});
    E_d(k-n_a+1) = norm(x_actual{k} - x_rls_mil{k});

end
toc;
%------plot----------------
t = n_a+1:N;
plot(t,E_d,'g','linewidth',2);
title("Norm error in x-hat using RLS with MIL (0.050298 seconds)");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
print Prob4d_hw7 -dpng

%% all the time
close all;
clear;
clc;
format default;
load DataHW07_Prob5.mat;

[m,n] = size(C{1});
%% part a
%initialise
A{1} = C{1};
Y{1} = y{1};
for k = 1:N
%     R{k} = eye(m*k);
    if k > 1
        A{k} = [A{k-1};C{k}];
        Y{k} = [Y{k-1};y{k}];
        if rank(A{k}) == 20
            n_a = k;
            break;
        end

    end
end

fprintf("n s.t A_k is full rank is %d \n",n_a);

%% part b - using Batch Process
for k = n_a : N
    R{k} = eye(m*k);
    A{k} = [A{k-1};C{k}];
    Y{k} = [Y{k-1};y{k}];
    x_batch{k} = inv(A{k}.' * R{k} * A{k}) * A{k}.' * R{k} * Y{k};
    E_b(k-n_a+1) = norm(x_batch{k} - x_actual{k});
end

t = n_a:N;
figure(1);
plot(t,E_b,'g','linewidth',2);
title("Norm error in x-hat using Batch Process");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
% print Prob5b_hw7 -dpng


%% part c - Forgeting factor
% Warning : This part took long time to run 
lamda = 0.98;
%find Rk at initial
R={};
for k = 1:n_a
    S{k} = lamda^(n_a - k) * eye(m);
    if k == 1
        R{k} = S{k};
    else
        R{k} = diag([diag(R{k-1});diag(S{k})]);
    end

end
%using Batch method again
for k = n_a : N
    bl_a = lamda * R{k-1};
    bl_b = zeros(m*(k-1),m);
    bl_c = zeros(m,m*(k-1));
    bl_d = eye(m);
    row_a_b_R_k = [bl_a, bl_b];
    row_c_d_R_k = [bl_c,bl_d];
    R{k} = [row_a_b_R_k;row_c_d_R_k];
    A{k} = [A{k-1};C{k}];
    Y{k} = [Y{k-1};y{k}];
    x_batch{k} = inv(A{k}.' * R{k} * A{k}) * A{k}.' * R{k} * Y{k};
    E_b(k-n_a+1) = norm(x_batch{k} - x_actual{k});
end

t = n_a:N;
figure(2);
plot(t,E_b,'g','linewidth',2);
title("Norm error in x-hat using Batch Process with Forgetting factor");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
% print Prob5c_hw7 -dpng

%% part d 
lamda = 0.98;
%using RLS Algorithm + Forgetting factor
% -----------initialise step-------------
%compute Qk and Pk , xk at k = n_a
for k = 1:n_a
%     S{k} = eye(m);
    if k == 1
        Q{k} = C{k}.' * lamda^(n_a - k) * C{k};
        P{k} = C{k}.' * lamda^(n_a - k) * y{k};
    else
        Q{k} = Q{k-1} + C{k}.' * lamda^(n_a - k) * C{k};
        P{k} = P{k-1} + C{k}.' * lamda^(n_a - k) * y{k};
    end
end

x_rls ={};
x_rls{n_a} = inv(Q{n_a}) * P{n_a};

% -----------recursion step -------------
tic;
for k = n_a : N-1
%     S{k+1} = S{k};
    Q{k+1} = lamda * Q{k} + C{k+1}.' * C{k+1};
    %Kalman gain
    K{k+1} = inv(Q{k+1}) * C{k+1}.';

    %update new state with Kalam gain and innovation
    x_rls{k+1} = x_rls{k} + K{k+1} * (y{k+1} - C{k+1} * x_rls{k});
    E_c(k-n_a+1) = norm(x_actual{k} - x_rls{k});

end

%------plot----------------
t = n_a:N-1;
figure(3);
plot(t,E_c,'g','linewidth',2);
title("Norm error in x-hat using RLS without MIL + Forgetting factor");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
% print Prob5d_hw7 -dpng
toc;

%% part d but with MIL
lamda = 0.98;
%exactly as in part c
%using RLS Algorithm WITH MIL + Forgetting factor
% -----------initialise step-------------
%compute Qk and Pk , xk at k = n_a
for k = 1:n_a
%     S{k} = eye(m);
    if k == 1
        Q{k} = C{k}.' * lamda^(n_a - k) * C{k};
        P{k} = C{k}.' * lamda^(n_a - k) * y{k};
    else
        Q{k} = Q{k-1} + C{k}.' * lamda^(n_a - k) * C{k};
        P{k} = P{k-1} + C{k}.' * lamda^(n_a - k) * y{k};
    end
end

x_rls_mil ={};
Q_inv{n_a} = inv(Q{n_a});
x_rls_mil{n_a} =  Q_inv{n_a} * P{n_a};

% -----------recursion step -------------
for k = n_a : N-1
%     S{k+1} = S{k};
    %compute inverse using MIL
    Q_inv{k+1} = (1/lamda) * MIL(Q_inv{k},C{k+1}.',1/lamda,C{k+1},m);
    %Kalman gain
    K{k+1} = Q_inv{k+1} * C{k+1}.';

    %update new state with Kalam gain and innovation
    x_rls_mil{k+1} = x_rls_mil{k} + K{k+1} * (y{k+1} - C{k+1} * x_rls_mil{k});
    E_d(k-n_a+1) = norm(x_actual{k} - x_rls_mil{k});

end
%------plot----------------
t = n_a+1:N;
figure(4);
plot(t,E_d,'g','linewidth',2);
title("Norm error in x-hat using RLS with MIL + forgetting factor");
xlabel('k','FontSize',10);
ylabel('E_k','FontSize',10);
grid on;
% print Prob5d_MIL_hw7 -dpng

close;
clc;
clear;
format default;

% A = [1 3;3 9];
% A = [6 10 11;10 19 19;11 19 21];
A = [2 6 10;6 10 14;10 14 18];
[V,D] = eig(A);
% disp(V);
disp(D);
% N = sqrt(D)*V';
% disp(V * D * V.')

N = (V * D^(1/2)) * V.';
disp(N);
% disp(N.' *  N);

fprintf("Another way : \n");

N2 = sqrtm(A);
disp(N2);

close;
clc;
clear;
format default;

A = [1 3 2;3 8 4];
b = [1;2];
M = [5 1 9;1 2 1;9 1 17];
% x_hat = min_norm_underdeter_case(A,b);
x_hat = weighted_min_norm_underdeter_case(A,b,M);
disp(x_hat);

function sol = min_norm_underdeter_case(A,b)
    sol = A.' * inv(A * A.') * b;
end

function sol = weighted_min_norm_underdeter_case(A,b,M)
    sol = inv(M) * A.' * inv(A * inv(M) * A.') * b;
end
%% 8

close;
clc;
clear;
format default;

A = [1 2;
    3 4;
    5 0;
    0 6];
y = [1.5377
    3.6948;
    -7.7193;
    7.3621];
Q = [1.00 0.50 0.50 0.25; 
    0.50 2.00 0.25 1.00;
    0.50 0.25 2.00 1.00;
    0.25 1.00 1.00 4.00];

for i = 2:4
    [sol,cov] = blue_func(y(1:i),Q(1:i,1:i),A(1:i,:));
    fprintf("Case %d (first %d values of y) \n",i,i);
    fprintf("x_hat = \n");
    disp(sol);
    fprintf("Covariance of the estimate is :\n");
    disp(cov);
    fprintf("\n");
end


function [sol,cov] = blue_func(y,Q,C)
    %assume that x is determnistic
    % mean error = 0, cov error = Q
    %Q is PD, diagonal, mean x_hat (sol) = x

    K_hat = inv(C.' * inv(Q) * C) * C.' * inv(Q);
    x_hat = K_hat* y;
    sol = x_hat;
    cov = inv(C.' * inv(Q) * C);
end

%%

close;
clc;
clear;
format default;

C = [1 2;3 4;5 0;0 6];

y = [1.5377;3.6948;-7.7193;7.3621];

Q = [1.00 0.50 0.50 0.25;
0.50 2.00 0.25 1.00;
0.50 0.25 2.00 1.00;
0.25 1.00 1.00 4.00];

P = [0.5 0.25;
0.25 0.5];

for i = 1:4
    [sol,cov] = MVE_func(y(1:i),C(1:i,:),Q(1:i,1:i),P);
    fprintf("Case %d (first %d value(s) of y) \n",i,i);
    fprintf("x_hat = \n");
    disp(sol);
    fprintf("Covariance of the estimate is :\n");
    disp(cov);
    fprintf("\n");
end

function [sol,cov] = MVE_func(y,C,Q,P)
    %model : y = Cx + e
    %assumption: mean x = 0, mean error =0
    % cov x = P, cov error = Q
    % cov ex.T = 0, uncorelated

    K_hat = P * C.' * inv(C * P * C.' + Q);
    x_hat = K_hat * y;
    sol = x_hat;
    cov = P - P * C.' * inv(C * P * C.' + Q) * C *P;
end

%% 
close;
clc;
clear;
format default;

%model y =  Cx + e
C = [1 2;3 4;5 0;0 6];
y = [1.5377;3.6948;-7.7193;7.3621];

%part a - normal LS
fprintf("Part a: \n");
x_hat_ls = inv(C.' * C) * C.' * y;
fprintf("Standard least square: \n");
fprintf("x_hat = \n");
disp(x_hat_ls);

fprintf("----------------------------- \n");
% part b - BLUE
fprintf("Part b: \n");
Q = eye(4);
[x_hat_blue,cov] = BLUE_func(y,Q,C);
fprintf("BLUE: \n");
fprintf("x_hat = \n");
disp(x_hat_blue);

fprintf("----------------------------- \n");
%part c - MVE
fprintf("Part c: \n");
Q = eye(4);
P = 100 * eye(2);
[x_hat_mve,cov] = MVE_func(y,C,Q,P);
fprintf("MVE: \n");
fprintf("When P = 100I: \n");
fprintf("x_hat = \n");
disp(x_hat_mve);

P = 10^6 * eye(2);
[x_hat_mve,cov] = MVE_func(y,C,Q,P);
fprintf("When P = 10^6 I: \n");
fprintf("x_hat = \n");
disp(x_hat_mve);

fprintf("----------------------------- \n");
%part d - compares
fprintf("Part d: \n");
fprintf("All results are the same. \n");



function [sol,cov] = BLUE_func(y,Q,C)
    %assume that x is determnistic
    % mean error = 0, cov error = Q
    %Q is PD, diagonal, mean x_hat (sol) = x

    K_hat = inv(C.' * inv(Q) * C) * C.' * inv(Q);
    x_hat = K_hat* y;
    sol = x_hat;
    cov = inv(C.' * inv(Q) * C);
end

function [sol,cov] = MVE_func(y,C,Q,P)
    %model : y = Cx + e
    %assumption: mean x = 0, mean error =0
    % cov x = P, cov error = Q
    % cov ex.T = 0, uncorelated

    K_hat = P * C.' * inv(C * P * C.' + Q);
    x_hat = K_hat * y;
    sol = x_hat;
    cov = P - P * C.' * inv(C * P * C.' + Q) * C *P;
end

%%
close;
clc;
clear;
format default;

%parameters
C = [1 2;3 4;5 0;0 6];

y = [1.5377;3.6948;-7.7193;7.3621];

Q = [1.00 0.50 0.50 0.25;
0.50 2.00 0.25 1.00;
0.50 0.25 2.00 1.00;
0.25 1.00 1.00 4.00];

P = [0.5 0.25;
0.25 0.5];

x_mean = [1;-1];
e_mean = 0;

% compute mean and variance
y_mean = C * x_mean + e_mean;
delta_y = y - y_mean;

%call function and print solution
[sol,cov] = MVE_func(delta_y,C,Q,P);
fprintf("MVE for x: \n");
x_hat_mve = x_mean + sol;
fprintf("x_hat = \n");
disp(x_hat_mve);

function [sol,cov] = MVE_func(y,C,Q,P)
    %model : y = Cx + e
    %assumption: mean x = 0, mean error =0
    % cov x = P, cov error = Q
    % cov ex.T = 0, uncorelated

    K_hat = P * C.' * inv(C * P * C.' + Q);
    x_hat = K_hat * y;
    sol = x_hat;
    cov = P - P * C.' * inv(C * P * C.' + Q) * C *P;
end

%% 9


clear;
clc;
close;

syms t pi
% f=t^2;
% g=exp(pi*t);
% a=0; b=2;
% G11=int(f*f,a,b);
% G11=simple(G11)

G = [8/3 -2/pi;-2/pi 1];
c = [2 ; pi];
b = inv(G) * c;
f = b(1) * t + b(2) * sin (pi*t);
s = simplify(f);
disp(s);
disp(simplify(b));

clear;
close;
clc;
format default;

% QR factorization from scratch
A = [1 2;3 4;5 6];
v1 = A(:,1);
q1 = (1/norm(v1)) * v1;
v2 = A(:,2) - dot(A(:,2),q1)*q1;
q2 = (1/norm(v2))*v2;

%build Q
Q = [q1,q2];

%build R
r11 = dot(A(:,1),q1);
r12 = dot(A(:,2),q1);
r22 = dot(A(:,2),q2);
R = [r11 r12;0 r22];

%see econ QR
disp(Q);
disp(R);

fprintf("Compare with MATLAB built in function: \n");
%compare with MATLAB qr built in function
[Q_mat, R_mat] = qr(A,0);
disp(Q_mat);
disp(R_mat);

%% 10


clear;close;clc;format default;
%1a
x = sym('x',[3 1]);
f = 3*x(1)*(2*x(2) - x(3)^3) + (x(2)^4)/3;

% compute using definition
grad_f = sym(zeros(length(x),1));
for i = 1:length(x)
    grad_f(i) = diff(f,x(i));
end

fprintf("The Jacobian of f is : \n");
% pretty(simplify(grad_f))
disp(simplify(grad_f.'));
x_s = [1 3 -1].';
fprintf("Evaluate Jacobian of f at xs is : \n");
disp(subs(grad_f,x,x_s).');

% compute using Jacobian built in func
% Jacobian of a scalar function is the 
% transpose of its gradient
grad_f_jaco = jacobian(f,x).';
subs(grad_f_jaco,x,x_s);

% compute using gradient func
grad_f_gradi = gradient(f,x);
subs(grad_f_gradi,x,x_s);

%% 1b
clc;
format short;
sigma = 0.001;
e = eye(3);
df_numer = zeros(3,1);
%  symmetric difference
for i = 1:length(x)
    f_p = subs(f,x,x_s + sigma * e(:,i));
    f_m = subs(f,x,x_s - sigma * e(:,i));
    df_numer(i) = vpa((f_p - f_m)/ (2*sigma));
end

fprintf("The Jacobian of f at xs is : \n");
disp(df_numer.');

%% 1c 
clc;
sigma = 0.0001;
e = eye(5);
xs = ones(1,5);
df_numer = zeros(1,5);
for i = 1:5
    f_p = funcPartC(xs + sigma * e(i,:));
    f_m = funcPartC(xs - sigma * e(i,:));
    df_numer(i) = (f_p - f_m)/ (2*sigma);
end

fprintf("The Jacobian of funcPartC at xs is : \n");
disp(df_numer);

%%
clear;
close;
clc;
format default;

load SegwayData4KF.mat;
% whos;

% Kalman filter
% Initial condition
P = P0;
xhat = zeros(4,N);
xhat(:,1) = x0;
Karr = [];
% loop
for k = 1:N
    K = (P*C')*inv(C*P*C' + Q);
    Karr = [Karr K];
    xhat(:,k+1) = A*xhat(:,k) + B*u(k) + A*K*(y(k)-C*xhat(:,k));
    P = A*(P-K*C*P)*A'+G*R*G';
end

% Segway_anim(t,xhat(1,1:end-1),xhat(2,1:end-1),Ts);

% just plot
figure(1)
plot(t,xhat(1,1:end-1),'r',t,xhat(2,1:end-1),'g','LineWidth',2); grid on
legend('$\hat{\varphi}(t)$','$\hat{\theta}(t)$','Interpreter','latex')
xlabel('t [s]')
ylabel('Angles [rad]')
print -dpng ex2b_plot_theta_phi_vs_t

figure(2)
plot(t,xhat(3,1:end-1),'r',t,xhat(4,1:end-1),'g','LineWidth',2); grid on
legend('$\hat{\dot{\varphi}}(t)$','$\hat{\dot{\theta}}(t)$','Interpreter','latex')
xlabel('t [s]')
ylabel(' Angular velocities [rad/s]')
print -dpng ex2b_plot_theta_dot_phi_dot_vs_t

figure(3)
plot(t,Karr,'LineWidth',2); grid on
legend({'$K_{\theta}$', '$K_{\varphi}$', '$K_{\dot{\theta}}$', ...
'$K_{\dot{\varphi}}$' },'interpreter','latex')
xlabel('t [s]')
ylabel('Kalman Gains')
print -dpng ex2b_plot_Kalman_gain_vs_t

% compare Kalman gains
fprintf("Steady states values of K are: \n");
disp(K);
[Kss,Pss] = dlqe(A,G,C,R,Q);
fprintf("Kss is: \n");
disp(Kss);
cmp_res = ismembertol(K,Kss,0.001);
fprintf("Comparison result is: \n");
if cmp_res
    fprintf("Kss =  K  \n");
else
    fprintf("Kss !=  K \n");
end


%%
clear; close; clc;format default;

A = 1;
% time step 
delta_t = 0.1;
B = delta_t;
R = 16; %action cov of noise (to state)
c = 3 * 10^8;
C = -2/c;
Q = 10^(-18); %measurement cov of noise
% initial condition
xhat = 1;
P = 0.25;
uhat = 10;

% At time t = 0.1;
z1 = 2.2 * 10^(-8);

% Prediction step
xhat = A * xhat + B * uhat;
P = A * P * A.' + B*R*B.';

% Measurement Update Step
K = (P*C')*inv(C*P*C' + Q);
xhat = xhat + K*(z1-(C*xhat+ (10/c)));
P = P - K*C*P;

%show result
fprintf("Mean of position x1 of robot at t = 0.1 :\n");
disp(xhat);
fprintf("Uncertainty in its position is : \n");
disp(P);

clear;
close;
clc;
format default;

A = [ 4.041   7.046    3.014;
     10.045    17.032    7.027;
     16.006    27.005    11.048];

[U,S,V] = svd(A);

D = zeros(3,3);
D(1:2,1:2)= S(1:2,1:2);
Ahat = U*D*V.';

fprintf(" Rank 2 approximation of A is :\n");
disp(Ahat);

% check rank and norm
fprintf("Rank of Ahat is :\n");
fprintf("Ahat = :\n");
disp(rank(Ahat));
% this is equivalent to
% max e-values of del_A = Ahat - A
% E= Ahat - A;
% err = max(sqrt(eig(E'*E)));
err = norm(Ahat - A,2); 
fprintf("Smallest norm is \n");
disp(err);



%% 11

clc;close all; clear; format default;
% parameters symbolic 
x = sym('x',[2 1]);
y = 0;

%function F
F = [3;4] + [4 3;2 1] * x - x*x.'*[1;2];

% convert
F_func = matlabFunction(F,'Vars',{x});

%Newton method
xk = x - inv(jacobian(F,x)) * (F-y);
xk_func = matlabFunction(xk,'Vars',{x}); 

%intial guess
% x0 = [1 1].';
% x0 = [0 0].';
x0 = [8 8].';

%tolerance for stopping loop
tol = 1 * 10^(-30);
iter_max = 100000;
% iter_max = 100;

%intiial assign
x = x0;
for i = 2:iter_max

    %Newton
    x = xk_func(x);

    if  abs(F_func(x) - y) < tol
        break;
    end

end   

fprintf("Initial guess: \n");
disp(x0);
fprintf("The solution x is : \n");
disp(x);
fprintf("Function F =  : \n");
disp(F_func(x));

clc;close all; clear; format default;

Aeq = [ 1 1 1 1];
beq = 3;

Ain = [1 2 3 4;
    5 6 7 8];
bin = [5 10].';

[X,fval,exitflag,output] = quadprog(eye(4),[0;0;0;0],Ain,bin,Aeq,beq);

disp(X)


%% Part b

clc;clear;
Q = [ 4 1 0 0;
    1 2 1 0;
    0 1 6 1;
    0 0 1 8]
xo = [1 2 3 4].'

Ain = [1 2 3 4;
    5 6 7 8]

bin = [5 10].'

Aeq = [];
beq = [];
lb = [];
ub = [];
x0 = [];

H = 2*Q;
f = -2*xo.'*Q;

x = quadprog(H,f,Ain,bin,Aeq,beq,lb,ub,x0)


clc;close all; clear; format default;
% 1-norm
fprintf('1- norm cost function : \n');
A = [2 1;
    -3 7;
    5 4];

b = [3 2 12].';

[m,n] = size(A);

f = [zeros(1,n),ones(1,m)];

Ain = [A,-eye(m);-A,-eye(m)];

bin = [b;-b];


[X,fval] = linprog(f,Ain,bin);

% extract real x from X, rest is slack variable
xval = X(1:n)
obj_func_val = norm(A*xval-b,1)

% infinity norm
fprintf('Infinity norm cost function : \n');
f = [zeros(1,n),1];
Ain = [A,-ones(m,1);-A,-ones(m,1)];
bin = [b;-b];


[X,fval] = linprog(f,Ain,bin);

% this time slack variable s belongs to R
xval = X(1:n)
obj_func_val = norm(A*xval-b,'inf')