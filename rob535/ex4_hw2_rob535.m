clc;clear;close all; format default;

%% Parameters
L=3;            % wheelbase
b=1.5;          % distance from rear wheel to center of mass

dt=0.01;        % time discretization
T=0:dt:6;       % time span

%% load reference trajectory
load('part1_traj_05_timestep.mat');

% the reference trajectory Y_ref is given as a 3x601 double
% the reference input U_ref is given as a 2x601 double

U_ref=interp1(0:0.05:6,[U,U(:,end)]',T)';
Y_ref=interp1(0:0.05:6,Y,T)';

%% 4.1 Discrete-time A and B matrices
%these are the system linearized in discrete time about the reference 
%trajectory i.e. x(i+1)=A_i*x_i+B_i*u_i

syms t X1(t) X2(t) X3(t) U1(t) U2(t)
X = [X1;X2;X3];
U = [U1;U2];
f = sym('f',[3 1]);

f(1) = U1*cos(X3) - U1* (b/L) * tan(U2) * sin(X3);
f(2) = U1*sin(X3) + U1* (b/L) * tan(U2) * cos(X3);
f(3) = (1/L) *U1 * tan(U2);

%How to obtain A and B and just compute them at reference traj and input
% A = jacobian(f,X);
% B = jacobian(f,U);

A = @(i) eye(3) + dt*[0, 0, - sin(Y_ref(3,i))*U_ref(1,i) - (cos(Y_ref(3,i))*tan(U_ref(2,i))*U_ref(1,i))/2;
          0, 0,   cos(Y_ref(3,i))*U_ref(1,i) - (tan(U_ref(2,i))*sin(Y_ref(3,i))*U_ref(1,i))/2;
          0, 0,                                                    0];

B= @(i) dt*[cos(Y_ref(3,i)) - (tan(U_ref(2,i))*sin(Y_ref(3,i)))/2, -(sin(Y_ref(3,i))*U_ref(1,i)*(tan(U_ref(2,i))^2 + 1))/2;
        sin(Y_ref(3,i)) + (cos(Y_ref(3,i))*tan(U_ref(2,i)))/2,  (cos(Y_ref(3,i))*U_ref(1,i)*(tan(U_ref(2,i))^2 + 1))/2;
                                   tan(U_ref(2,i))/3,             (U_ref(1,i)*(tan(U_ref(2,i))^2 + 1))/3];

%% 4.2 Number of decision variables for colocation method
npred=10;
n_states = 3;
n_inputs=2;

Ndec= (n_states * (npred+1)) + (n_inputs * npred);
%% 4.3 Write and test function to construct Aeq beq (equality constraints
%enforce x(i+1)=A_i*x_i+B_i*u_i for the prediction horizon) 
%check the values of Aeq and beq timestep 1 (t=0)
%with the given initial condition of Y(0)=[0.25;-0.25;-0.1]

Y0 = [0.25; -0.25; -0.1];
[Aeq_test1, beq_test1] = eq_cons(1, A, B,Y0,npred);

%% 4.4 Write and test function to generate limits on inputs 
%check the value at timestep 1 (t=0)

input_range = [0 1; -0.5 0.5];
[Lb_test1, Ub_test1] =   bound_cons(1,U_ref,input_range,npred);


%% 4.5 Simulate controller working from initial condition [0.25;-0.25;-0.1]
%use ode45 to between inputs

Q = eye(3); Q(3,3) = 0.5;
R = eye(2) * 0.1; R(2,2) = 0.01;

% Q = 2*eye(3); Q(3,3) = 0.8;
% R = eye(2) * 0.1; R(2,2) = 0.01;

% Constants
n = 3;
m = 2;
PredHorizon=npred+1;

% we begin by defining the cost function
H = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ) );
c = zeros(  n * PredHorizon + m * ( PredHorizon - 1 ), 1 );
for i = 1:PredHorizon
    H( ( ( i - 1 ) * n + 1 ):( i * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) = Q;
end
for i = 1:( PredHorizon - 1 )
    H( ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ), ( ( i - 1 ) * m + n * PredHorizon + 1 ):( i * m + n * PredHorizon ) ) = R;
end

%set initial condition 
Y = zeros(n,  length(T));
Y(:,1)= Y0 + Y_ref(:,1);
u_in = zeros( m,length( T ) - 1  ); % don't have an input at the last time step.
% u(:,1) = U_ref(:,1);

for i=1:length(T)-1

   %calculate error between Y (mpc) and Y_ref at time step i (state of linearized system)
   x= Y(:,i) - Y_ref(:,i);

   [Aeq, beq] = eq_cons(i, A, B,x,npred);

   [Lb, Ub] =  bound_cons(i,U_ref,input_range,npred);

   options = optimset('Display', 'off');
   solDV = quadprog(H,c,[],[],Aeq,beq,Lb,Ub,[],options);

%    Y( :, i ) = solDV( ( n + 1 ):( 2 * n ) ) + Y_ref(:,i);
%    Y( :, i -1) = solDV( 1:3 ) + Y_ref(:,i-1);
   u_in(:,i) = solDV( ( n * PredHorizon + 1 ):( m + n * PredHorizon ) ) + U_ref(:,i);
    
   %use ode45 to simulate nonlinear system, f, forward 1 timestep 
   %(where f is the kinematic bicycle model)
   %and T is the time span
   [~,ytemp]=ode45(@(t,Z) f_NL_mpc(t,Z,u_in(:,i),b,L),[T(i),T(i+1)],Y(:,i));
   
   %set next state  
   Y(:,i+1)=ytemp(end,:);
   
end

% figure();
% plot(T,Y_ref(1,:),'r',T,Y_ref(2,:),'r-',T,Y_ref(3,:),'r.','LineWidth',2);
% hold on;
% plot(T,Y(1,:),'g',T,Y(2,:),'g-',T,Y(3,:),'g.','LineWidth',2);
% grid on;
% title(' MPC response bicycle model with ODE45');
% xlabel('Time t');
% ylabel('Solution x,y,phi');
% legend("x_ref","y_ref","\psi_{ref}","x_mpc","y_mpc","\psi_{Mpc}");

figure();
plot(Y_ref(1,:),Y_ref(2,:),'r',Y(1,:),Y(2,:),'g','LineWidth',2);
title(' MPC xy trajectory vs ref');
xlabel('x');
ylabel('y');
legend("ref","actual")

figure();
plot(T,Y_ref(3,:),'r',T,Y(3,:),'g','LineWidth',2);
title(' MPC phi vs time of both real and actual traj');
xlabel('time');
ylabel('phi');
legend("ref","actual")

figure()
plot(T(1,1:end-1),U_ref(1,1:end-1),'r',T(1,1:end-1),u_in(1,:),'g','LineWidth',2);
title(' MPC prediction input u vs ref u');
xlabel('time');
ylabel('input u : linear velocity');
legend("ref","predict")

figure()
plot(T(1,1:end-1),U_ref(2,1:end-1),'r',T(1,1:end-1),u_in(2,:),'g','LineWidth',2);
title(' MPC prediction input delta vs ref delta');
xlabel('time');
ylabel('input delta : steering angular velocity');
legend("ref","predict")

%Calculate max distance error between the actual and nominal trajectories,
%when the x value of the actual trajectory is between or equal to 3 and 4 m

% x_index = 1;  % assuming x is the first state
Y_x = Y(1, :) >= 3 & Y(1, :) <= 4;
Y_subset = Y(:, Y_x);
% Y_subset = Y(:,282:384);

Y_ref_subset = Y_ref(:,Y_x);
% Y_ref_subset = Y_ref(:,302:401);

max_dist_error=  max(sqrt((Y_subset(1,:) - Y_ref_subset(1,:)).^2 + (Y_subset(2,:) - Y_ref_subset(2,:)).^2 ));
fprintf("Maximum distance error between actual tracjectory and reference trajectory when 3<=x <=4 is : \n");
disp(max_dist_error);

%% write functions for constraint generation and dynamics down here and 
%call them when needed above, for example,

function [Aeq, beq] = eq_cons(idx, A, B,x0,npred)

    %build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
    %in the form Aeq*z=beq
    %initial_idx specifies the time index of initial condition from the reference trajectory 
    %A and B are function handles above

    % Constants
    n = 3;
    m = 2;
    PredHorizon=npred+1;

    % Define the size of the matrices
    num_states = n * PredHorizon; % x̃, ỹ, ψ̃ at each of 11 time steps
    num_inputs = m * (PredHorizon-1); % ũ, δ̃ for each of 10 time steps
    num_constraints = num_states; % 3 initial conditions + 10 Euler integrations per state variable
    
    % Initialize the constraint matrices
    Aeq = zeros(num_constraints, num_states + num_inputs);
    beq = zeros(num_constraints, 1);
%     Aeq = zeros( n * PredHorizon, n * PredHorizon + m * ( PredHorizon - 1 ) );
%     beq = zeros( n * PredHorizon, 1 );
    
    % Initial condition constraints
    Aeq(1:3, 1:3) = eye(3);
    beq(1:3) = x0;
    %     Aeq( 1:n, 1:n ) = eye( n ); 
%     beq( 1:n ) = initial_state;
   
 
%     for i = 1:( PredHorizon - 1 )
%         Aeq((i*3+1):((i + 1)* 3), (i*3+1):((i+1)*3)) = -eye(3)  ;
%         Aeq((i*3+1):((i + 1)*3 ), ((i - 1) * 3 + 1 ):(i * 3)) =  A(idx) ;
%         Aeq((i * 3 + 1 ):((i+1)*3 ), ((i-1) * 2 + PredHorizon *3 + 1):(i * 2 + PredHorizon*3)) =   B(idx);
%     end

    for i = 1:( PredHorizon - 1 )
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( i * n + 1 ):( ( i + 1 ) * n ) ) = -eye( n );
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ( ( i - 1 ) * n + 1 ):( i * n ) ) =  A( idx ) ;
        Aeq( ( i * n + 1 ):( ( i + 1 ) * n ), ...
        ( ( i - 1 ) * m + PredHorizon * n + 1 ):( i * m + PredHorizon * n ) ) =   B( idx );
    end
   
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Lb,Ub]=bound_cons(idx,U_ref,input_range,npred)
    % %BOUND_CONS Generate lower and upper bounds for the decision variables
% %   at a given time step idx along the reference trajectory
% %
% %   Inputs:
% %       - idx: the index along the reference trajectory where the initial
% %         condition is at
% %       - U_ref: the reference input trajectory, 2x601 double
% %       - input_range: a 2x2 double specifying the min and max values for
% %         each input, [umin umax; delTmin delTmax]
% %       - Ndec: the number of decision variables
% %
% %   Outputs:
% %       - Lb: the lower bound vector, Ndecx1 double
% %       - Ub: the upper bound vector, Ndecx1 double

    %initial_idx is the index along uref the initial condition is at
    PredHorizon = npred +1;
    u = U_ref(1,:);
    delta = U_ref(2,:);
    Ub = inf(3*npred+2*(PredHorizon-1),1);
    Lb = -1*inf(3*PredHorizon+2*(PredHorizon-1),1);

        % % Set the bounds for the states
    % for i = 1:11
    %     Lb(Nstate*(i-1) + 1 : Nstate*(i-1) + 3 ) = [-Inf;-Inf;-pi];
    %     Ub(Nstate*(i-1) + 1 : Nstate*(i-1) + 3 ) = [Inf;Inf;pi];
    % end
    % 
    % % Set the bounds for the inputs
    % for i = 1:10
    % %     % u
    % %     Lb(Nstate*11 + Ninput*(i-1) + 1) = max(U_ref(1,idx),input_range(1,1));
    % %     Ub(Nstate*11 + Ninput*(i-1) + 1) = min(U_ref(1,idx),input_range(2,1));
    % %     
    % %     % delta
    % %     Lb(Nstate*11 + Ninput*(i-1) + 2) = max(U_ref(2,idx),input_range(1,2));
    % %     Ub(Nstate*11 + Ninput*(i-1) + 2) = min(U_ref(2,idx),input_range(2,2));
    %     % u
    %     Lb(Nstate*11 + Ninput*(i-1) + 1) = input_range(1,1);
    %     Ub(Nstate*11 + Ninput*(i-1) + 1) = input_range(2,1);
    %     
    %     % delta
    %     Lb(Nstate*11 + Ninput*(i-1) + 2) = input_range(1,2);
    %     Ub(Nstate*11 + Ninput*(i-1) + 2) = input_range(2,2);
    
    for i = 1:PredHorizon-1
        if i+idx-1>601
            Ub(2*i+32) = input_range(1,2) - u(601);
            Ub(2*i+33) = input_range(2,2) - delta(601);
            Lb(2*i+32) = input_range(1,1) - u(601);
            Lb(2*i+33) = input_range(2,1) - delta(601);
        else
            Ub(2*i+32) = input_range(1,2) - u(i+idx-1);
            Ub(2*i+33) = input_range(2,2) - delta(i+idx-1);
            Lb(2*i+32) = input_range(1,1) - u(i+idx-1);
            Lb(2*i+33) = input_range(2,1) - delta(i+idx-1);
        end   
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function dz = f_NL_mpc(t,Z,u_in,b,L)
    %States = z = [x,y,phi]

    phi = Z(3);
    dz = zeros(3,1);
    
    dz(1) = u_in(1)*cos(phi) - u_in(1)* (b/L) * tan(u_in(2)) * sin(phi);
    dz(2) = u_in(1)*sin(phi) + u_in(1)* (b/L) * tan(u_in(2)) * cos(phi); 
    dz(3) = (1/L) *u_in(1) * tan(u_in(2));

end