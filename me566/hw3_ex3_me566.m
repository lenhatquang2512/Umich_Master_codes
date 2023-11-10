clc;clear;close all; format default;

% Unit Conversion factors
rps_rpm = 30/pi;     % rad/sec to rpm
rpm_rps = pi/30;
mps_mph = 2.237;     % m/s to mph
mph_mps = 1/2.2472;

% Part a - find traction power and torque
% load time [s]  and speed [mph] of EPA urban cycle
load CYC_UDDS.mat;

% load parameters
% Parameters;

% Or just paste it here-  Parameters
Rw      = 0.3;                  % Tire radius = 0.3m
Mv      = 1200;                 % Vehcle mass = 1200kg
grav    = 9.8;                  % Gravity constant (N/kg)
fr      = 0.015;                % Rolling resistance coefficient
rho     = 1.202;                % Air density (kg/m^3)
Cd      = 0.4;                  % Aerodynamic drag coefficient
Af      = 1.8;                  % Vehicle frontal area (m^2)
mps2mph = 2.2472;
slope   = 0;                    % road gradient

% resistance forces
RR = fr * Mv * grav * cos(slope); % rolling resistance
WR_coef = 0.5 * rho * Cd * Af ; % wind coef
Grade_road = 0; % mg sin(slope)

% assign time and velocities in each time step
t = cyc_mph(:,1); % time [sec]
v = cyc_mph(:,2) * mph_mps; % vehicle speed [m/s]

%initialise
m = length(cyc_mph(:,1)) ;
acc = zeros(m,1);
F_traction = zeros(m,1);
T_traction = zeros(m,1); % traction torque
Power = zeros(m,1); % power needed for this vehicle to follow the EPA urban cycle

% Backward difference , all are standard unit
for i=2:m
    acc(i)= (v(i) - v(i-1))/(t(i) - t(i-1));    % del v/ del t

    F_traction(i) = Mv * acc(i) + (RR + WR_coef * v(i)^2 + Grade_road); % N

    T_traction(i) = F_traction(i) * Rw;  %Nm

    Power(i) = F_traction(i) * v(i); % [W], or torque * angular speed

end

%plot
figure(1);
plot(t,T_traction,'g','LineWidth',2); % grid on;
% legend ({'$Motor A$','$Motor B$','$Engine speed$'},'Interpreter','latex')
xlabel('Time [s] ','FontSize',10);
ylabel(' Traction torque [Nm] ','FontSize',10);
title("Results of traction torque as a function of time");
% print hw3_ex3a_me566_torque -dpng;

figure(2);
plot(t,Power/1000,'r','LineWidth',2);
xlabel('Time [s] ','FontSize',10);
ylabel(' Power [kW] ','FontSize',10);
title("Results of power as a function of time");
% print hw3_ex3a_me566_power -dpng;


% Part b -compute total energy and distance
meter2miles = 0.0006213712;

total_energy = trapz(t/3600,Power/1000); %kWh
total_distance = trapz(t,v) * meter2miles; %miles

% if drive 300 miles of urban cycle
battery_energy = total_energy * 300/total_distance;

fprintf("Battery energy would be required to drive 300 miles of urban cycle is : \n");
disp(battery_energy);

