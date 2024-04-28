%% Problem 4 (4.1 and 4.3)
clc;clear;close all; format default;

% Vehicle Parameters
a   =  1.14;		% distance c.g. to front axle (m) 
b   =  1.4;            % distance c.g. to rear axle (m) 
L   =  2.54;		% wheel base (m)
m   =  1500;		% mass (kg)
Iz  =   2420.0;	% yaw moment of inertia (kg-m^2)
g   =  9.81;
u0  =  10;              %constant forward speed [m/s]
Caf = 44000*2   ;       %cornering stiffness of front wheels [N/rad]
Car = 47000*2   ;       %cornering stiffness of rear wheels [N/rad]

%Dynamics model for lateral vehicle
A = [0   ,                      1,                  0,                               0 ;
     0   , -(Caf+Car)/(m*u0)     ,       (Caf+Car)/m ,             (b*Car-a*Caf)/(m*u0);
     0   ,                      0,                  0,                               1;
     0   ,  (b*Car-a*Caf)/(Iz*u0), (a*Caf -b*Car)/Iz , -((a^2)*Caf+(b^2)*Car)/(Iz*u0)];

B = [0   ; Caf/m;     0;  a*Caf/Iz];

% Output 
C = [1 0 0 0];
D = 0;

T = 0.1;    % time discretization
tspan = 0:T:10;   % Timespan for all simulations
% delta_fun=@(t) 0.1 * sin(t);   

%Discretized state space model 
Ad = expm(A*T);
Bd = integral(@(h) expm(A*h),0,T,'ArrayValued',true) * B;
Cd = C;
Dd = D;

%% 4.1 Compute H(z), the transfer function for the discrete system

% z = tf('z',-1);
syms z
H = Cd * inv(z*eye(4) - Ad) * Bd + Dd;

%output this and then check all coefficients one by one to verify
fprintf("H(z) = \n");
pretty(simplify(vpa(H)))

[numH,denH] = ss2tf(Ad,Bd,Cd,Dd);  %only to verify

% Create the transfer function H(z)
Hz = tf(numH, denH,-1,'Variable','z');

% Display the transfer function
disp('Transfer Function H(z):');
Hz

%Evaluate both H(derived by definition) and Hz (from ss2tf) at same
%value of z to verify
z_test = 2;
fprintf("H (derived by definition) H(2) = %f \n",subs(H,z,z_test));
fprintf("Hz (from ss2tf) Hz(2)  = %f \n", ...
    subs(poly2sym(Hz.Numerator{1,1},z)/poly2sym(Hz.Denominator{1,1},z),z,z_test));

%% 4.3 Compute Q(z), the Z transform of q k . Also, compute q(kT)
syms k z
assume(k>=0)
delta = 0.1 * sin(k*T);
% Delta_z = ztrans(delta,k,z); % same as deriving by hand for 4.2
Delta_z = 0.1 * z * sin(T)/(z^2 - 2*z * cos(T) + 1);

% Compute Q(z)
Q_z = H * Delta_z;

%display Q(z)
fprintf("Q(z) = \n");
pretty(simplify(vpa(Q_z)))

% Compute q(kT) = Z^-1(Q(z)))(k)
qk = iztrans(Q_z,z,k);

%display q(k)
fprintf("q(kT) = \n");
pretty(simplify(vpa(qk)))

% Initialize an array to store the results of qk for each k
qkT = zeros(1,length(tspan));

% Substitute qk for each value of k and store the results
for i = 1:length(tspan)
    qkT(i) = subs(qk, k, i-1);
end

%plot
figure(1);
stem(tspan, qkT, 'b','LineWidth',1); grid;
xlabel('$ t[s] $','Interpreter','latex','FontSize',15);
ylabel('$ q[kT](m)$','Interpreter','latex','FontSize',15);
title('Lateral displacement versus time')

print ex4_hw3_me561_part4_3 -dpng;



















