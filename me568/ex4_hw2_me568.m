%% Problem 4 - Vehicle lateral control , using lsim (1st method)
% clc; 
clear; close all; format default;

Gvnum = [138 2400 34287];
Gvden = [1 29.832 224.523 0 0];
Kd = 5; T = 0.015; Tp=0.015;
Gvnum_pv=conv(Gvnum, [Tp^2/2 Tp 1]);
Gdnum=Kd*[-T/2 1];
Gdden=[T/2 1];
Gcnum=conv(Gdnum,Gvnum_pv);
Gcden=conv(Gdden,Gvden) + Gcnum;
Gc_tf = tf(conv(Gdnum,Gvnum),conv(Gdden,Gvden) + [0 0 conv(Gdnum,Gvnum)]);
t=0:0.01:5;
yd = ones(1,length(t));
y_p=lsim(Gcnum,Gcden,yd,t);

y = lsim(conv(Gdnum,Gvnum),conv(Gdden,Gvden) + [0 0 conv(Gdnum,Gvnum)],yd,t);
% e=yd-y_p';
% % calculate the true lateral displacement
% steer=lsim(Gdnum,Gdden,e,t);
% y=lsim(Gvnum, Gvden, steer,t);

%Plot
plot(t,yd, '-g', t,y_p, 'r', t-Tp, y,'-.b' ,'LineWidth',2); grid

% plot(t,yd, '-g', t,y_p, 'r','LineWidth',2)
xlabel('time (sec)')
title('Lat. disp. (m)')
legend('Yd', 'Y_p', 'Y')


%% 2nd method: Simulink 
clc; clear; close all; format default;
out = sim('ex4_hw2_simulink_draft.slx');
end_time = 5;
Kd = 5; T = 0.015; Tp=0.015;

yd = ones(1,length(out.tout));
t = out.tout;
y_pv = out.y_with_pv;
y_no_pv = out.y_without_pv;
% plot(t,yd, '-g', t,y_pv, 'r', t-Tp, y_no_pv,'-.b' ,'LineWidth',2); grid
figure();
plot(t,y_no_pv,'b','LineWidth',2);grid;
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Lateral displacement $y [m]$','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Lateral displacement $y$ (NO preview) vs time",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% print p4_img/ex4_hw2_me568_disp_no_preview -dpng;

figure();
plot(t,y_pv,'b','LineWidth',2); grid;
xlabel('Time $t$ [s] ','Interpreter','latex','FontSize',13,'FontWeight','bold');
ylabel('Lateral displacement $y [m]$','Interpreter','latex','FontSize',13,'FontWeight','bold');
title("Lateral displacement $y$ (WITH preview) versus time",'Interpreter','latex','FontSize',13,'FontWeight','bold')
% print p4_img/ex4_hw2_me568_disp_with_preview -dpng;
