%% Problem 3 - Autonomous Cruise Control
clc; clear; close all; format default;

g=9.81; uw=0.0; u0=30.0; rho =1.202;
Theta=0.0; ThetaPrime=0.0;

% Controlled vehicle parameters:
mc=1000.0; Cdc=0.5; Arc=1.5; fc=0.015;
Kc=(1/(rho*Cdc*Arc*(u0+uw)));Tc=mc*Kc;
wc0=mc*g*(fc*sin(Theta)-cos(Theta))*ThetaPrime; Fc=(u0/Kc);

% Lead vehicle parameters (typical):
ml=1500.0;Cdl=0.6;Arl=1.95;fl=0.015;
Kl=(1/(rho*Cdl*Arl*(u0+uw)));Tl=ml*Kl;
Fl=(u0/Kl);
wl0=(ml*g*(fl*sin(Theta)-cos(Theta))*ThetaPrime);

t= (0:0.1:20)';
U0=Fc*ones(size(t)); % Nominal control force
wl= -800*(1+0.01*t); % Ramp function
wc= wc0*ones(size(t));

% disturbance=[U0 wc wl];
% 4-state system for controller-design:
Aa=[0 -1 0 0;
0 -1/Tc 0 0;
1 0 0 0;
0 0 1 0];
Ba=[0;Kc/Tc;0;0];

% % Controller design: LQR
% % K=place(Aa,Ba,pc);
% Q = 100000 * eye(4);
% % Q = 10e5 * eye(3);  % for comparison
% R = 1;
% K = lqr(Aa,Ba,Q,R);

%For part 3b, simpler control law
Q = 10e5 * eye(3);
R =1;
Aa3=[0 -1 0 ; % 3-state system for controller-design:
0 -1/Tc 0 ;
1 0 0 ;];
Ba3=[0;Kc/Tc;0];
K = lqr(Aa3,Ba3,Q,R);

% Closed-loop simulation (4 states, keep track of vl (x5), get rid of x4):
Ac=[0 -1 0  1;
-K(1)*Kc/Tc -(1+K(2)*Kc)/Tc -K(3)*Kc/Tc  0;
1 0 0  0;
0 0 0  -1/Tl];
Bc=[0 0 0;
0 Kc/Tc 0;
-1 0 0;
0 0 Kl/Tl];

% outputs: x1 (range), vc and vl
Cc=[1 0 0 0;0 1 0 0 ; 0 0 0 1]; Dc=zeros(3,3);
r=30.0*ones(size(t));
disturbance=[r U0+wc wl]; 

%Now only 4 states without x4
xc0=[30 u0 0 u0];

%Simulate
[yc,xc]=lsim(Ac,Bc,Cc,Dc,disturbance,t,xc0);

figure(1)
subplot(311), plot(t,yc(:,1),'b','LineWidth',1.5); title('Range');
xlabel('Time (sec)'); grid;
subplot(312), plot(t,yc(:,3), 'r',t,yc(:,2),'-.b','LineWidth',1.5);
title('Vehicle speed (m/sec)');
xlabel('Time (sec)'); grid;
legend('Vl', 'Vc'); 

% print p3_img/ex3_hw4_me568_part_a_lqr_range_speed -dpng;

% figure(2)
% clf, 
subplot(313)
u= U0-K(1)*xc(:,1)-K(2)*xc(:,2)-K(3)*xc(:,3);
plot(t, u','b','LineWidth',1.5); title('Control Force (N)');
xlabel('Time (sec)'); grid

% print p3_img/ex3_hw4_me568_part_a_lqr_input_traction_force -dpng;

% print p3_img/ex3_hw4_me568_part_b_all_lqr_combined_3_states_fb -dpng;




