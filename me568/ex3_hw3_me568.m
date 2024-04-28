%% Problem 3 - IDM Simulation
clc; clear; close all; format default;

% Vehicle Parameters
a = 0.73; % acceleration
b = 1.67; % deceleration
delta = 4; % acc exponent
vo = 30; %desired speed [m/s]
so = 2 ; %minimum gap wrt leading vehicle [m]
% so = 5; % for repear LATER
T = 1.5; % time gap [s]

%simulated time
tspan = 0:0.1:100;

% Initial speed and distance
v0 = 20;
s0 = 36;

%velocity profiles for its leading vehicle
vL = @(t) (t <= 20) .* (20 - t) + (t > 20) .* 0;
% vL = @(t) 20 + sin(t);

% Perform integration and store results in state X
[t,X] = ode45(@(t,X)IDM_Dynamics(t,X,vL(t),a,b,delta,vo,so,T),tspan,[v0;s0]);

% Plot the results
figure();
subplot(211)
plot(t,X(:,1),'b','LineWidth',1.5);grid;
% xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Velocity $v [m/s]$ ','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Ego vehicle forward Speed",'Interpreter',['la' ...
    'tex'],'FontSize',13,'FontWeight','bold')

subplot(212)
plot(t,X(:,2),'b','LineWidth',1.5);grid;
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Distance $s [m]$ ','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Ego vehicle Distance gap",'Interpreter',['la' ...
    'tex'],'FontSize',13,'FontWeight','bold')

% print p3_img/ex3_hw3_me568_part_b_so_5 -dpng;

function Xdot = IDM_Dynamics(t,X,vL,a,b,delta,vo,so,T)
    v = X(1);
    s = X(2);
    %
    if v > 0 
        s_star = so + max(0,v*T + ((v*(v-vL))/(2*sqrt(a*b))));
        vdot = a*(1 - (v/vo)^delta - (s_star/s)^2);
    else
        vdot=0;
    end
    sdot = vL-v;
    %
    Xdot = zeros(2,1);
    Xdot(1) = vdot;
    Xdot(2) = sdot;
end