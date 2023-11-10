clear;clc;close all; format default;
%parameter definition
m = 1500; % vehicle mass [kg]
n = 4.8; % gear ratio (gearbox + final drive)
r = 0.4; % wheel radius [m]
g = 9.81; % acceleration of gravity [m/s^2]
C_r = 0.01; % rolling resistance
rho=1.3;  %density of air (p) [kg/m^3]
C_d=0.32; % vehicle’s drag coefficient
a = 2.4; %vehicle s frontal area [m^2]

theta_e =  2*pi/180; % road grade [rad]
v_e = 20; % longitudinal velocity [m/s]

%you may enter equations, but do not change the names of the variables given. 
%The auto-grader uses these variable names to check your answers!

%Part 2.1 Determine equilibrium engine torque
f_roll_res = m*g*C_r*sign(v_e);
f_drag = (1/2)*rho * C_d * a * (v_e)^2;
f_road_incli = m*g*sin(theta_e);
%since f_eng_torque = n*u/r = total 3 forces above: [N.m]
u_e= (f_roll_res + f_drag + f_road_incli)*r/n; 

%Part 2.2.1 Determine the A,B,F matrices for the linearized system
% v_dot = au - b sgn(v) - cv^2 - d sin(theta) linear ->
aa_ = n/(m*r);
b_ = g*C_r;
c_ = (rho * C_d * a)/ (2*m);
d_ = g;

% this is by differentiating df/dx df/du
A= - 2 * c_ *v_e;
B= aa_;
F= -d_ * cos(theta_e);

%Part 2.2.2 Determine feedback gains of linear system to place pole at -1
% Ackerman formula K = -e_n.' * inv(B AB ... A^n-1 B) char_poly(A) where
% char_poly is det(A - lamda*I)
k=  1* (1/B) * (A+1);  %usually is A+ I but this is 1D case

%Part 2.2.3 Determine the steady state error after 10 seconds
theta_tilde = 3*pi/180;
f=@(t,v) (A-B*k)*v + F* theta_tilde;
tspan= 0:0.1:10;
v_tilde_0= -2;

[t,v_tilde]=ode45(f,tspan,v_tilde_0);
sse= v_tilde(end);

figure(1);
plot(t,v_tilde,'g','LineWidth',2);
grid on;
title('Solution linearised system with ODE45');
xlabel('Time t');
ylabel('Solution v_tilde');
axis([0 10 -2 0]);



%Part 2.3.1 Determine A_I,B_I,F_I matrices of the linear system with integral action
%the subscript I is just used to indicate these matrices and vectors apply 

% padding this one
A_I= [A  0;1 0];
B_I= [B;0];
F_I= [F;0];

% %Part 2.3.2 Place poles of the system with integral action at -1,-2
%  The desired characteristic polynomial is s^2 + 3s +2 therefore:
char_des_A_I = A_I ^ 2 + 3 * A_I + 2 * eye(2);
control_matrix_Co = [B_I A_I*B_I];
k_I= [0 1] * inv(control_matrix_Co) * char_des_A_I;

%Part 2.3.3 with integral action determine the steady state error after 10
%seconds
% X = [v_tilde z].';
f_I=@(t,X_I) (A_I-B_I*k_I)*X_I+ F_I* theta_tilde;
tspan= 0:0.1:10;
X_0 = [-2 0].';

[t,X_I]=ode45(f_I,tspan,X_0);
sse_with_integral_action= X_I(end,1);

figure(2);
plot(t,X_I(:,1),'r','LineWidth',2);
grid on;
title(' Integral action  + Solution linearised system with ODE45');
xlabel('Time t');
ylabel('Solution v_tilde with integral action');
axis([0 10 -2 0.2]);



% %Part 2.3.4 simulate response of nonlinear system with actuator limits

tspan= 0:0.1:10;
Xnl_0 = [-2+v_e 0].';

[t,Xnl]=ode45(@(t,Xnl) f_Xnl(t,Xnl,k_I,v_e,u_e,theta_tilde,theta_e,aa_,b_,c_,d_),tspan,Xnl_0);

% sse_with_integral_action= X_I(end,1);

figure(3);
plot(t,X_I(:,1),'r',t,Xnl(:,1) - v_e * ones(length(Xnl(:,1)),1),'g','LineWidth',2);
grid on;
title(' Response of both linear and non-linear system with ODE45');
xlabel('Time t');
ylabel('Solution v_tilde');
legend("Linear","Non-Linear");
axis([0 10 -2 1.5]);

max_overshoot_of_nonlinear_system = max(Xnl(:,1) - v_e * ones(length(Xnl(:,1)),1));


function dXnl = f_Xnl(t,Xnl,k_I,v_e,u_e,theta_tilde,theta_e,aa_,b_,c_,d_)
    % States = Xnl = [v,z]
    v = Xnl(1);
    z = Xnl(2);
    u = -k_I * [v- v_e;z] + u_e;
    if u > 200
        u = 200;
    elseif u < 0
        u = 0;
    end

    theta_ = theta_tilde + theta_e;

    dXnl = zeros(2,1);

    dXnl(1) = aa_ * u - b_ * sign(v) - c_ * v^2 - d_ * sin(theta_);
    dXnl(2) = v - v_e;

end




















