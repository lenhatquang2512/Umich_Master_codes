clc;clear;close all; format default;

b = 1.5 ; 
L = 3 ;
dt=0.05;
nsteps= 121;

%remember the format for z is as follows:
%z=[x0 y0 th0 x1 y1 th1 ... xn yn thn u0 d0 ... u(n-1) d(n-1)]';
    
%3.1
ub_pose = repmat([8;3;pi/2], 121, 1);
ub_input = repmat([1;0.5], 120, 1);
ub = [ub_pose;ub_input];

lb_pose = repmat([-1;-3;-pi/2], 121, 1);
lb_input = repmat([0;-0.5], 120, 1);
lb = [lb_pose;lb_input];

% 
% %3.4
% %%%%%%%%%%%%%%%% no need to change these lines  %%%%%%%%%%%%%%%%%%%%%%
options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
                       'SpecifyObjectiveGradient',true,'Display', 'off') ;
x0=zeros(1,3*nsteps + 2 * (nsteps-1));
cf=@costfun;
nc=@nonlcon;
z=fmincon(cf,x0,[],[],[],[],lb',ub',nc,options);

Y0=reshape(z(1:3*nsteps),3,nsteps)';
U=reshape(z(3*nsteps+1:end),2,nsteps-1);
u=@(t) [interp1(0:dt:119*dt,U(1,:),t,'previous','extrap');...
        interp1(0:dt:119*dt,U(2,:),t,'previous','extrap')];
[T1,Y1]=ode45(@(t,x) odefun(x,u(t)),0:dt:120*dt,[0 0 0]);
[T2,Y2]=ode45(@(t,x) odefun(x,u(t)),0:dt:120*dt,[0 0 -0.01]);
plot(Y0(:,1),Y0(:,2),Y1(:,1),Y1(:,2),Y2(:,1),Y2(:,2))
% plot(Y0(:,1),Y0(:,2))
theta = 0:0.01:2*pi;
hold on
plot((0.7*cos(theta)+3.5),(0.7*sin(theta)-0.5))
hold on
plot(0,0,'x');
legend('fmincon trajectory','ode45 trajectory using x0 = [0;0;0]',...
    'ode45 trajectory using x0 = [0;0;-0.01]','Buffered Obstacle','Start');
ylim([-2,2]);
xlim([-1,8]);
xlabel('x');
ylabel('y');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%3.2
function [g,h,dg,dh]=nonlcon(z)

    [x,y,th,u,delta,b,L,dt] = get_states_and_inputs(z);

    % size of g must be 121 x 1 (no.of time steps);
    g = zeros(length(x),1);

    for i = 1:length(x)
        g(i) = 0.7^2 - (x(i) - 3.5)^2 - (y(i) + 0.5 )^2;
    end

    % size of dg must be 603 x 121 = Transpose(no. of time steps x no. of elements in 'z');
    dg_dx= zeros(length(x), length(z) - 2*(length(x)-1));

     % Compute the partial derivatives of g with respect to x, y, and theta
    for i = 1:length(x)

        % Insert the non-zero elements into the Jacobian matrix
        dg_dx(i, (i-1)*3+1) =  -2*(x(i) - 3.5);  %dgdx
        dg_dx(i, (i-1)*3+2) =  -2*(y(i) + 0.5);
        dg_dx(i, (i-1)*3+3) = 0; % partial derivative with respect to theta is always zero
    end
    
    % Create a block of zeros for the partial derivatives with respect to u and delta
    dg_du = zeros(length(x), 2*(length(x)-1));

    % Stack the blocks vertically to form the complete Jacobian matrix
    dg = [dg_dx, dg_du];
    dg = dg.';

    % size of h must be 363 x 1 ((no. of time steps * no. of states) x 1)
    % initialize the h vector
    h = zeros(3*length(x), 1);

    h(1) = x(1);
    h(2) = y(1);
    h(3) = th(1);

    % use a for loop to fill in the h vector
    for i = 2:length(x)

        %find x,y,th dot
%         x_dot = u(i-1)*cos(th(i-1)) - (b/L)*u(i-1)*tan(delta(i-1))*sin(th(i-1));
%         y_dot = u(i-1)*sin(th(i-1)) + (b/L)*u(i-1)*tan(delta(i-1))*cos(th(i-1));
%         th_dot = (u(i-1)/L) * tan(delta(i-1));

        dX = odefun([x(i-1);y(i-1);th(i-1)],[u(i-1);delta(i-1)]);

%         h(3*i-2) = x(i) - x(i-1) - 0.05* x_dot;
%         h(3*i-1) = y(i) - y(i-1) - 0.05* y_dot;
%         h(3*i) = th(i) - th(i-1) - 0.05* th_dot;


        h(3*i-2) = x(i) - x(i-1) - dt* dX(1);
        h(3*i-1) = y(i) - y(i-1) - dt* dX(2);
        h(3*i) = th(i) - th(i-1) - dt* dX(3);
         
    end

    % size of dh must be 603 x 363 = Transpose((no. of time steps * no. of states) x no. of elements in 'z') ;
%     dh = jacobian(h,z);

    dh = zeros(3 * length(x), 3 * length(x) + 2 * length(u));

    dh(1:3, 1:3) = eye(3);

    for i = 1:( length(x) - 1 )
        dh((i*3+1):((i + 1)*3 ), ((i - 1) * 3 + 1 ):(i * 3)) =  -eye(3) ;
        dh((i*3+1):((i+1)* 3), (i*3+1):((i+1)*3)) = eye(3)  ;
        dh((i*3+1 ):((i+1)*3 ), ...
            ((i-1) * 2 + length(x) *3 + 1):(i * 2 + length(x)*3)) ...
        = -0.05 *  [cos(th(i)) - (tan(delta(i))*sin(th(i)))*(b/L), -(sin(th(i))* u(i)*(tan(delta(i))^2 + 1))*(b/L);
           sin(th(i)) + (cos(th(i))*tan(delta(i)))*(b/L),  (cos(th(i))*u(i)*(tan(delta(i))^2 + 1))*(b/L);
                     tan(delta(i))*(1/L)             ,             (u(i)*(tan(delta(i))^2 + 1))*(1/L)  ];
    end


    dh = dh.';


end

%3.3
function [J, dJ] = costfun(z)
    
    [x,y,th,u,delta,b,L,dt] = get_states_and_inputs(z);

    % size of J must be 1 x 1
    J = sum((x-7).^2 + y.^2 + th.^2) + sum(u.^2 + delta.^2);
    % size of dJ must be 1 x 603 (1 x no. of elements in 'z')

%     dJ = jacobian(J,z);
     dJ_dx= zeros(3*length(x) ,1)*b/b *dt/dt;  % I just do want to see warning no use of b and L
     dJ_du = zeros(2*length(u),1)*L/L;
   
    for i = 1:length(x)
        dJ_dx(3*i-2) =  2*(x(i) - 7);  
        dJ_dx(3*i-1) =   2* y(i) ;  
        dJ_dx(3*i) =   2 * th(i);  
    end

    for i = 1:length(u)
        dJ_du(2*i -1) =  2 * u(i)  ;  
        dJ_du(2*i) = 2 * delta(i) ; 
    end

    dJ = [dJ_dx;dJ_du];
    dJ = dJ.';
end

function [dx] = odefun(x,u)
    b = 1.5 ; 
    L = 3 ;
    dx = [u(1)*cos(x(3))-(b/L)*u(1)*tan(u(2))*sin(x(3)) ;...
          u(1)*sin(x(3))+(b/L)*u(1)*tan(u(2))*cos(x(3)) ;...
          u(1)*tan(u(2))/L] ;
end


function [x,y,th,u,delta,b,L,dt] = get_states_and_inputs(z)

    %parameters
    b = 1.5 ; 
    L = 3 ;
    dt=0.05;

    z_states = z(1:(121*3));
    z_input = z((121*3+1):end);

    id_x = 1:3:length(z_states);
    id_y = 2:3:length(z_states);
    id_th = 3:3:length(z_states);

    id_u = 1:2:length(z_input);
    id_delta = 2:2:length(z_input);

    x = z_states(id_x);
    y = z_states(id_y);
    th = z_states(id_th);
    
    u = z_input(id_u);
    delta = z_input(id_delta);
    
end