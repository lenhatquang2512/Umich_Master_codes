%% Problem 1 - Cruise Control
clc; clear; close all; format default;

% Define the linearized model parameters
u0=30; g=9.81; m=1250; f=0.015; Theta=0;
rho=1.202; A=1; Cd=0.5; uw=5;

% Calculate the equilibrium force, Fx0:
Fx0 = m*g*sin(Theta) + f*m*g + 0.5*rho*A*Cd*(u0+uw)^2;

% The time constant and dc gain are:
Tau=(m/(rho*A*Cd*(u0+uw))); 
K=Tau/m;

% Fix the Ti gain (Ki=Kp/Ti) for the PI control:
Ti=186.86/10.0;

% The loop transfer function for PI control is
% Kp*(num/den) where num/dec = (1+1/Ti*s)*K/(1+Tau*s)
num_o=K*[Ti 1]; den_o=[Ti*Tau Ti 0];

% Calculate open loop zeros and poles:
z=roots(num_o);
p=roots(den_o);

% Obtain the root locus for Kp:
K_root= 0:10:300;
R=rlocus(num_o,den_o,K_root);
% figure(1);
% plot(R,'*'); title('RootLocus Plot');
% axis([-0.2 0 -0.05 0.05]);
% 
% [K_root, POLES]=rlocfind(num_o,den_o)
% Simulate the reference unit step response
% with and w/o control:
t= 0:0.5:100;
sys = tf([K],[Tau 1]); % Open loop
y_new_2a =step(sys,t);

Kp=186.86; KI = Kp/Ti;
numc=K*[Kp KI]; denc=[Tau K*Kp+1 K*KI];
sys_c = tf(numc, denc); % Closed loop
yc_new_2a =step(sys_c, t);

load ex1_hw4_me568_unit_step_ref_input.mat
figure(2);
% plot(t,y,'-r',t,yc,'--b'); grid;
plot(t,yc,'r',t,yc_new_2a,'b','LineWidth',2); grid;
xlabel('Time (sec)')
title('Unit Step Reference Response');
legend('$m = 1000, u_w = 2,u_o = 20$', '$m = 1250, u_w = 5,u_o = 30$','Interpreter','latex','FontSize',10,'FontWeight','bold');
% print p2_img/ex2_hw4_me568_part_a__unit_step_ref_new_system -dpng;

% Simulate the disturbance unit step response
% with and w/o control:
% yd=step(sys,t);
sys_disturbance = tf([K 0], denc);
% ycd=step(sys_disturbance,t);
% figure(3);
% plot(t,yd,'-r',t,ycd,'--b'); xlabel('Time (sec)'),grid;
% title('Unit Step Disturbance Response');
% legend('Open-loop', 'Closed-loop');



