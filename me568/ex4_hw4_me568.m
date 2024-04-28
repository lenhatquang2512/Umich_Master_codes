%% Problem 4 - Traction Control ABS
clc; clear; close all; format default;

% Initialize Torque (at Te=0, Tb=800 for braking and
% Te=1800, Tb=0 for accelerating)
global Torque;
Te = 0.0; Tb = 800.0; Torque = Te - Tb;
% Define the mu-lambda polynomial coefficients:
c = [-68.5937, 238.2160, -324.8197, 219.2837, ...
-75.5800, 12.0878, -0.0068];

% Perform the closed-loop simulation:
ti=0.0; tf=1.0; t1 = ti;
xi=[20.0;20.0];
% Set the rule-based control parameters:
alph1 = 0.1;
alph2=0.1; 
lmin=0.13; 
lmax=0.17;

%LOGGING
x_log(1,:)=xi';
t_log(1)=ti;
Torque_log(1) = Torque;
OPTIONS=odeset('MaxStep',0.001,'RelTol',1.0e-3);

lambda_prev = 0; % ADD

% Set the new rule-based control parameters:
Torque_Limit_High = -700; % upper limit of torque command
Torque_Limit_Low = -1000; % lower limit of torque command
desired_slip = -0.142; % desired value of slip for optimal traction

for i=1:175
%     i
    t2=ti+i*(tf-ti)/200;
    tspan = t1:0.001:t2;
    [t,x] = ode23(@(t,x)ex13_5a(t,x),tspan,xi,OPTIONS);
    x_log(i+1,:)=x(size(x,1),:); % data log
    xi=x(size(x,1),:)';
    t1 = t(size(t,1));
    t_log(i+1)=t(size(t,1));


    % Define wheel slip lambda:
    if xi(1) >= xi(2)
        lambda=(xi(2)-xi(1))/xi(1);
    else
        lambda=(xi(2)-xi(1))/xi(2);
    end
    lambda = max(min(lambda, 0), -1);
    
%     dLambda = (lambda - lambda_prev) / (t2 - t1); %ADD
 % Rule-based logic adjustment:
    slip_error = desired_slip - lambda;

    % Calculate the coefficient mu:
    al = abs(lambda);
    % if al > 1.0 al=1.0; end;
    % mu = -sign(lambda)*c*[alˆ6;alˆ5;alˆ4;alˆ3;alˆ2;al;1];

    % Sigmoid function for smooth transition:
    sigmoid = 1 / (1 + exp(-100 * (al - (lmin + lmax) / 2)));

    % Define the change in torque delT:
    if al<lmin
%         delT = sign(lambda)*alph1*abs(Torque)* (1 - sigmoid);
         delT = sign(lambda)*abs(Torque)*abs(slip_error);
    elseif al>lmax
%         delT = -sign(lambda)*alph2*abs(Torque) * sigmoid;
         delT = -sign(lambda)*abs(Torque)* abs(slip_error);
    else
        delT = 0.0;
    end

%     Torque = Torque + delT;
    Torque = max(min(Torque + delT, Torque_Limit_High), Torque_Limit_Low);
    Torque_log(i+1) = Torque;

%         lambda_prev =lambda; % ADD
end

% Plot the results:
% subplot(311), plot(t_log,x_log(:,1),'b',...
%     t_log, x_log(:,2),'r','LineWidth',1.5);
subplot(311), plot(t_log,[x_log(:,1) x_log(:,2)]);
title('Vehicle Simulation with ABS');
xlabel('Time (sec)'); ylabel('x');
legend('Vehicle speed', 'wheel speed'); grid;
% axis([0 1 -10 20])

%lambda=(x_log(:,2)-x_log(:,1))./x_log(:,2);% TCS
lambda=(x_log(:,2)-x_log(:,1))./x_log(:,1); % ABS
subplot(312)
% plot(t_log,lambda,'r');
id =find(x_log(:,2)>0,1,'last');
plot(t_log(1:id),lambda(1:id),'r')
xlabel('Time (sec)'); ylabel('Lambda'); grid;
% axis([0 1 -1 0])

subplot(313), plot(t_log,Torque_log,'r');
xlabel('Time (sec)'); ylabel('Torque (N-m)'); grid
% axis([0 1 -900 -600])

% print p4_img/ex4_hw4_me568_rule_based_new_ABS -dpng;

%Ex13_5a.m
function xdot = ex13_5a(t,x)
    global Torque;
    % Define the simulation parameters:
    m=1400; rho=1.202; Cd=0.5; A=1.95; g=9.81;
    Theta=0.0; bw=0.0; f=0.01; uw=0.0; fw=0.0;
    Iw=0.65; rw=0.31; Fz=3560.0;
    % For braking: Ie=0, Nw=4; and for accel: Ie=0.429, Nw=2.
    rg=9.5285; Ie=0.0; Nw=4;
    Jw = Iw + (rg^2)*Ie;
    % Define the mu-lambda polynomial coefficients:
    c = [-68.5937, 238.2160, -324.8197, 219.2837, ...
    -75.5800, 12.0878, -0.0068];
    % Define wheel slip lambda:
    if x(1) >= x(2)
        lambda=(x(2)-x(1))/x(1);
    else
        lambda=(x(2)-x(1))/x(2);
    end
    % Calculate the coefficient mu:
    al = abs(lambda);
    if al > 1.0 al=1.0; end
    mu = sign(lambda)*c*[al^6;al^5;al^4;al^3;al^2;al;1];
    % Define the state equations:
    xdot=[(-(0.5*rho*Cd*A)*(uw+rw*x(1))^2+Nw*Fz*mu- ...
        f*m*g*cos(Theta)-m*g*sin(Theta))/(m*rw);
    (fw*Fz-bw*x(2)-Fz*rw*mu+Torque)/Jw];
    if x(1) <= 0.0, x(1)=0.0; xdot(1)=0.0; end
    if x(2) <= 0.0, x(2)=0.0; xdot(2)=0.0; end
end



