clc;clear;close all;format default;
%vehicle parameters
b=1.5;
L=3;

%trajectory (nominal) inputs
delta_f=0.3;
u=5;

%note that the solutions Y refer to the full state vector i.e. (Y(t) is 3x1)
%3.1 Simulate the non-linear system to generate the circular trajectory (nominal trajectory)

tspan= 0:0.01:5;
Xnl_0 = [0 0 0].';

%use ode45 to simulate the dynamics with delta_f, and u as inputs. 
%T should be 501x1. Y, the nominal trajectory, should be 501x3
[T,Y]= ode45(@(t,Xnl) f_Xnl(t,Xnl,u,delta_f,b,L),tspan,Xnl_0);

figure(1);
plot(T,Y(:,1),'r',T,Y(:,2),'g',T,Y(:,3),'b','LineWidth',2);
grid on;
title(' Kinematic bicycle model non-linear system with ODE45');
xlabel('Time t');
ylabel('Solution x,y,phi');
legend("x","y","\psi");
% axis([0 10 -2 1.5]);

%3.2 Find the linearized time varying A and B matrices
% A=@(t) 
%     
% B=@(t) 
% 

syms t X1(t) X2(t) X3(t) U1(t) U2(t)
X = [X1;X2;X3];
U = [U1;U2];
f = sym('f',[3 1]);

f(1) = U1*cos(X3) - U1* (b/L) * tan(U2) * sin(X3);
f(2) = U1*sin(X3) + U1* (b/L) * tan(U2) * cos(X3);
f(3) = (1/L) *U1 * tan(U2);

A = jacobian(f,X);
B = jacobian(f,U);

f_e = subs(f,[U1,U2],[5*t*(1/t),0.3]);
X_e = int(f_e,t);
X_e = subs(X_e,X3,X_e(3));

A_ = subs(A,[X3,U1,U2],[X_e(3),5,0.3]);
A = matlabFunction(A_,'Vars',{t});
B_ = subs(B,[X3,U1,U2],[X_e(3),5,0.3]);
B = matlabFunction(B_,'Vars',{t});


% %3.3 find the optimal feedback gains
%use provided lqr_ltv function, remember the input to the A,B matrices is a discrete time index
Q= eye(3);
R= eye(2);
A_ = subs(A_,t,(t-1)/100);
Afunc = matlabFunction(A_,'Vars',{t});
B_ = subs(B_,t,(t-1)/100);
Bfunc = matlabFunction(B_,'Vars',{t});

[K,P]= lqr_LTV(Afunc,Bfunc,Q,R,tspan);

% Afunc =  matlabFunction(eye(3) + A_ *0.01);
% Bfunc =  matlabFunction(B_ * 0.01);


%3.4 Simulate the LQR controller
x0= [0.1 0.8 0.01].';

%use ode 1 to simulate the linear system i.e (A-BK)X; 
%then add nominal trajectory  it
%store the final trajectory in a 501x3 matrix named YL

% beta = (1/L) * 5 * tan(0.3);
% bb_ = beta * b;
% a = 5;
% 
% X_e_func = sym('X_e_func',[3 1]);
% X_e_func(1) = (a/beta) * sin(beta*t) + (b/beta) * cos(beta * t);
% X_e_func(2) = (-a/beta) *cos(beta * t) + (b/beta) * sin (beta*t);
% X_e_func(3) = beta * t;
% X_e_func = matlabFunction(X_e_func,'Vars',{t});

x_lqr_ = ode1(@(t,x) (Afunc(t) - Bfunc(t) *K{t})*x , tspan, x0);

% f_e_new = subs(f,[X3,U1,U2],[beta*t,5,0.3]);
% f_e_func = matlabFunction(f_e_new);
% X_e_func = matlabFunction(int(f_e_func,t),'Vars',{t});

% for i = 1:length(tspan)
%     YL(i,:) = x_lqr_(i,:) + Y(i,:);
% end

YL= x_lqr_ + Y;

figure(2);
plot(T,Y(:,1),'r',T,Y(:,2),'r-',T,Y(:,3),'r.','LineWidth',2);
hold on;
plot(T,YL(:,1),'g',T,YL(:,2),'g-',T,YL(:,3),'g.','LineWidth',2);
grid on;
title(' LQR response bicycle model with ODE1');
xlabel('Time t');
ylabel('Solution x,y,phi');
legend("x","y","\psi","x_lqr","y_lqr","\psi _lqr");
hold off;
% axis([0 10 -2 1.5]);

figure(3)
plot(Y(:,1),Y(:,2),'r-','LineWidth',2);
hold on;
plot(YL(:,1),YL(:,2),'g-','LineWidth',2);
grid on;
title(' LQR response bicycle model with ODE1 - XY Plane');
xlabel('Time t');
ylabel('Solution x,y');
legend("Linearised trajectory","lqr");

%3.5 Simulate the Nonlinear system (the grader only checks that the final trajectory YNL is correct, a sketch of a for loop is provided below if you want to use it)
YNL=NaN(size(YL));

%set initial condition 
YNL(1,:)= x0.' + Y(1,:);

for i=1:length(T)-1
    
   %calculate error between YNL and Y at time step i (state of linearized system)
   x= YNL(i,:).' - Y(i,:).' ;
    
   %compute input to nonlinear system (u_in(t)=-K{i}x(t)+nominal_input)
   u_in= -K{i} * x + [5;0.3];

   %use ode45 to simulate nonlinear system, f, forward 1 timestep 
   %(where f is the kinematic bicycle model, equation 6 in the pdf)
   %and T is the time vector given in the pdf
   [~,ytemp]=ode45(@(t,z) f_NL_lqr(t,z,u_in,b,L),[T(i),T(i+1)],YNL(i,:));
   
   %set next state  
   YNL(i+1,:)=ytemp(end,:);
   
end

figure(4);
plot(T,YL(:,1),'r',T,YL(:,2),'r-',T,YL(:,3),'r.','LineWidth',2);
hold on;
plot(T,YNL(:,1),'g',T,YNL(:,2),'g-',T,YNL(:,3),'g.','LineWidth',2);
grid on;
title(' LQR response bicycle model with ODE45 LN and non-LN');
xlabel('Time t');
ylabel('Solution x,y,phi');
legend("xL","yL","\psi_{L}","xNL","yNL","\psi_{NL}");



function dz = f_NL_lqr(t,z,u_in,b,L)
    %States = z = [x,y,phi]

    phi = z(3);

    dz = zeros(3,1);
    
    dz(1) = u_in(1)*cos(phi) - u_in(1)* (b/L) * tan(u_in(2)) * sin(phi);
    dz(2) = u_in(1)*sin(phi) + u_in(1)* (b/L) * tan(u_in(2)) * cos(phi); 
    dz(3) = (1/L) *u_in(1) * tan(u_in(2));

end


function dXnl = f_Xnl(t,Xnl,u,delta_f,b,L)
    % States = Xnl = [x,y,phi]

    phi = Xnl(3);

    dXnl = zeros(3,1);

    dXnl(1) = u*cos(phi) - u* (b/L) * tan(delta_f) * sin(phi); 
    dXnl(2) = u*sin(phi) + u* (b/L) * tan(delta_f) * cos(phi); 
    dXnl(3) = (1/L) * u * tan(delta_f);

end

function [K, P] = lqr_LTV(AFun,BFun,Q,R,tSpan)
    nSteps = length(tSpan);

    P{nSteps} = zeros(size(Q));
    K{nSteps} = zeros(length(R),length(Q));
    
    for i = nSteps-1:-1:1
        A_ = AFun(i+1);
        B_ = BFun(i+1);
        P_ = P{i+1};
        
        P{i} = P_ + (tSpan(i+1)-tSpan(i)) * ( P_*A_ + A_'*P_ - P_*B_*(R\(B_'*P_)) + Q);
        K{i} = R\(B_'*P_);
    end
end

function x = ode1(odeFun, tSpan, x0)
    x = zeros(length(tSpan),length(x0));

    x(1,:) = x0';
    
    for i = 1:length(tSpan)-1
        x(i+1,:) = x(i,:) + (tSpan(i+1) - tSpan(i))*odeFun(i,x(i,:)')';
    end
end



