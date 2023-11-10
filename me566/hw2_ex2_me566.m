clc;clear all;close all; format default;

% Unit Conversion factors
rps_rpm = 30/pi;     % rad/sec to rpm
rpm_rps = pi/30;
mps_mph = 2.237;     % m/s to mph
mph_mps = 1/2.237;

%Parameters
K = 2.6; % sun and ring gear teeth ratio

rw = 0.312; % tire radius [m]
DR = 3.27; %Output differential ratio

eng_spd= 800 * rpm_rps ; %engine speed [rad/s]

vh_spd_mph = 0:0.1:90; %vehicle speed [mph]

vh_spd = vh_spd_mph * mph_mps; %vehicle speed [m/s]


% Compute ring, carrier and  sun speed for planetary gear set
%  - means "connected"
wr = (vh_spd/rw) * DR; % Ring angular velocity - Motor B [rad/s]
wc = eng_spd;   % Carrier angular velocity - Engine [rad/s]

ws = wc*(1+K) * ones(1,length(wr))- K*wr; %Sun angular velocity - Motor A[rad/s]

%plot
figure(1);
plot(vh_spd,ws,'r',vh_spd,wr,'g',vh_spd,wc * ones(1,length(wr)),'b','LineWidth',2);
legend ({'$Motor A$','$Motor B$','$Engine speed$'},'Interpreter','latex')
xlabel('Vehicle speed [m/s] ','FontSize',10);
ylabel('Planetary gear components angular speed [rad/s] ','FontSize',10);
title("Motor A, motor B, and engine speeds against vehicle speed");
grid on;
print hw2_ex2_me566 -dpng;


