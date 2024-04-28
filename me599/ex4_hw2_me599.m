%%   Set up an adaptive gradient descent optimization, using a line search, for the same problem
clc;clear;close all; format default;

u(:,1) = [1 1 1]';
J_iter(1) = u(1,1)^2 + u(1,1)^4*u(2,1)^2 + u(3,1)^2;
delta_u_convergence = .001;
delta_u = 1;
alpha_min = 0;
alpha_max = 0.5;
alpha_vec = linspace(alpha_min,alpha_max,100);
k = 1;
counter = 0;

%   Perform the gradient descent iterations
while delta_u >= delta_u_convergence && counter <= 100
    gradient(1) = 2*u(1,k) + 4*u(1,k)^3*u(2,k)^2;
    gradient(2) = 2*u(1,k)^4*u(2,k);
    gradient(3) = 2*u(3,k);
    J_best = J_iter(k);
    u_best = u(:,k);
    for i=1:length(alpha_vec)
        u_prelim = u(:,k) - alpha_vec(i)*gradient';
        J_prelim =  u_prelim(1)^2 + u_prelim(1)^4*u_prelim(2)^2 + u_prelim(3)^2;
        if J_prelim < J_best
            u_best = u_prelim;
            J_best = J_prelim;
        end
    end
    u(:,k+1) = u_best;
    J_iter(k+1) =  u(1,k+1)^2 + u(1,k+1)^4*u(2,k+1)^2 + u(3,k+1)^2;
    delta_u = norm(u(:,k+1)-u(:,k));
    counter = counter + 1;
    k = k + 1;
end

%   Plot the convergence of the elements of the control signal
figure(1)
plot(u(1,:),'r','LineWidth',3);
hold on
plot(u(2,:),'g','LineWidth',3);
hold on
plot(u(3,:),'b','LineWidth',3);
xlabel('Iteration','fontsize',12);
ylabel('Control signals','fontsize',12);
legend('x1','x2','x3');
title('Modified gradient descent+line search x1 x2 x3','fontsize',12);
% print ex4_hw2_me599_GD_x -dpng;

%   Plot the evolution of the cost function
figure(2)
plot(J_iter,'LineWidth',3);
xlabel('Iteration','fontsize',12);
ylabel('J','fontsize',12);
title('Modified gradient descent with line search J ','fontsize',12);
% print ex4_hw2_me599_GD_J -dpng;
%%   Set up a modified Netwon's method optimization with line search
clc;clear; format default;
% close all;
u(:,1) = [1 1]'; % We set x2 =1
%x(3,1) is actually x(2,1)
J_iter(1) = u(1,1)^2 + u(1,1)^4 + u(2,1)^2;
delta_u_convergence = .001;
delta_u = 1;
alpha_min = 0;
alpha_max = 1;
k = 1;
counter = 0;

alpha_vec = linspace(alpha_min,alpha_max,100);

%   Perform the Newton iterations
while delta_u >= delta_u_convergence && counter <= 100
    gradient = [2*u(1,k) + 4*u(1,k)^3, 2*u(2,k)];
    hessian = [2+12*u(1,k)^2, 0;
              0, 2];
    newton_direction = -inv(hessian)*gradient';
    u_best = u(:,k);
    J_best = J_iter(k);
    for i= 1:length(alpha_vec)
        u_prelim = u(:,k) + alpha_vec(i)*newton_direction;
        J_prelim = u_prelim(1)^2 + u_prelim(1)^4 + u_prelim(2)^2;
        if J_prelim < J_best
            u_best = u_prelim;
            J_best = J_prelim;
        end
    end
    u(:,k+1) = u_best;
    J_iter(k+1) =  u(1,k+1)^2 + u(1,k+1)^4 + u(2,k+1)^2;
    delta_u = norm(u(:,k+1)-u(:,k));
    counter = counter + 1;
    k = k + 1;
end
counter

%   Plot the convergence of the elements of the control signal
figure(3)
plot(u(1,:),'r','LineWidth',3);
hold on
plot(ones(length(u(1,:)),1),'g','LineWidth',3);
hold on
plot(u(2,:),'b','LineWidth',3);
ylim([0 1.1])
xlabel('Iteration','fontsize',12);
ylabel('Control signals','fontsize',12);
legend('x1','x2','x3');
title('Modified Newton+line search:x1 x2 x3','fontsize',12);
% print ex4_hw2_me599_Newton_x -dpng;

%   Plot the evolution of the cost function
figure(4)
plot(J_iter,'LineWidth',3);
xlabel('Iteration','fontsize',12);
ylabel('J','fontsize',12);
title('Modified Newton+line search: J','fontsize',12);
% print ex4_hw2_me599_Newton_J -dpng;


