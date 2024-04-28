%% Problem 2 - Vehicle dynamics (vertical & ride)
clc; clear; close all; format default;

% Ex4_9.m
%
% x(1) = tire deflection (zus-z0)
% x(2) = speed of unsprung mass
% x(3) = suspension stroke (zs-zus)
% x(4) = velocity of sprung mass
%
% cs = suspension damping
% ks = suspension stiffness
% ms = sprung mass
% mus = unsprung mass
% cus = damping coefficient of the tire
% kus = tire stiffness
% road displacement input = z0(t)
% road velocity input = d(z0)/dt
% Specify the parameter values here:
% w1 = 20*pi; % w1 = sqrt(kus/mus) -------------- uncomment for part a
w2 = 2.0*pi; % w2 = sqrt(ks/ms)
z1 = 0.02; % z1 = cus/(2*mus*w1)
z2 = 0.3; % z2 = cs/(2*ms*w2)
rho = 12.; % rho = ms/mus %----------uncomment for part b
Amp=1.65E-5; Vel=80;

% Loop 3 times for rho (part a) and w1 (part b)
rho_vec = [8 10 12]; %when w1 = 20pi
w1_vec = [16*pi 20*pi 24*pi]; %when rho = 12

% Input velocity characteristics
% for iter = 1:length(rho_vec)
 for iter = 1:length(w1_vec)
%     rho = rho_vec(iter);
    w1 = w1_vec(iter);
    for i=1:20
         w(i,iter) = i*(2.0*pi)/10.0;
         w2= w(i,iter);
         % Open loop system equations:
         A = [0 1 0 0
         -w1^2 -2*(z2*w2*rho+z1*w1) rho*w2^2 2*z2*w2*rho
             0 -1 0 1
             0 2*z2*w2 -w2^2 -2*z2*w2];
         B = [0 rho 0 -1]';
         G = [-1 2*z1*w1 0 0]';
         %
         % calculate the rms response to
         % a unit variance white
         r1=5.0e4; r2=5.0E3;
          % Weights for typical (T) ride case
         Xss=lyap(A,G*G');
         x3_rms=sqrt(Xss(3,3));
         x1_rms=sqrt(Xss(1,1));
         x4dot_rms=sqrt(A(4,:)*Xss*A(4,:)'+ G(4)*G(4)');
         J(i,iter)=(2.*pi*Amp*Vel)*(x4dot_rms^2+r1*(x1_rms^2)+r2*(x3_rms^2));
    end
end

% figure();
% clf;
% [J_min_vec,index_vec] = min(J)
% plot(w(:,1)/(2*pi),J(:,1),'r',w(:,2)/(2*pi),J(:,2),'g', ...
%     w(:,3)/(2*pi),J(:,3),'b','LineWidth',2);
% hold on;
% plot(w(index_vec(1),1)/(2*pi), J(index_vec(1),1),'ro', ...
%     w(index_vec(2),2)/(2*pi), J(index_vec(2),2),'go', ...
%     w(index_vec(3),3)/(2*pi), J(index_vec(3),3),'bo','MarkerSize',8);
% legend("$ \rho = 8 $","$ \rho = 10 $","$ \rho = 12 $",'Interpreter','latex','FontSize',11);
% xlabel('$ \omega_2 $ (Hz)','Interpreter','latex','FontSize',13);
% ylabel('Cost function J','Interpreter','latex','FontSize',13);grid;
% title("Plot of J versus $ \omega_2 $",'Interpreter','latex','FontSize',13)
% 
% print ex2_hw2_me568_rho_8_10_12 -dpng;

figure();
[J_min_vec,index_vec] = min(J)
clf;
plot(w(:,1)/(2*pi),J(:,1),'r',w(:,2)/(2*pi),J(:,2),'g', ...
    w(:,3)/(2*pi),J(:,3),'b','LineWidth',2);
hold on;
plot(w(index_vec(1),1)/(2*pi), J(index_vec(1),1),'ro', ...
    w(index_vec(2),2)/(2*pi), J(index_vec(2),2),'go', ...
    w(index_vec(3),3)/(2*pi), J(index_vec(3),3),'bo','MarkerSize',10);
legend("$ \omega_1 = 16 \pi $","$ \omega_1 = 20 \pi $","$ \omega_1 = 24 \pi $",'Interpreter','latex','FontSize',11);
xlabel('$ \omega_2 $ (Hz)','Interpreter','latex','FontSize',13);
ylabel('Cost function J','Interpreter','latex','FontSize',13);grid;
title("Plot of J versus $ \omega_2 $",'Interpreter','latex','FontSize',13)
% 
% print ex2_hw2_me568_omega_1_16_20_24 -dpng;