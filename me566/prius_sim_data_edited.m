% Finish the code and replace all the ? marks
clc;clear all;close all; format default;
%2004 Prius Model Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ICE engine parameters
e_inertia=0.18;           % (kg*m^2), rotational inertia of the engine
eng_map_spd=[1000 2000 3000 4000 5000]; %(rpm), speed range of the engine

eng_max_trq=[90   98.33  106.67  115 101]; % ? (Nm), max torque lookup table

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% electric motors parameters, the two motor/generators are assumed
% identical in this problem
m_inertia=0.03;           % (kg*m^2), rotational inertia of the motor
m_map_spd=[0 1000 1200 2000 3000 4000 5000 6000]; % (rpm), speed range of the motor

m_max_trq=[400 400 400 238 155 115 90 72]; % ? (N*m), torque range of the motor

m_eff=0.95;  % motor efficiency

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NIMH battery parameters
ess_max_ah_cap = 6.5/3;	% (Ahr) capacity, ?

ess_vol=201.6; % (V) Nominal pack voltage
ess_kwh_cap = ess_max_ah_cap * ess_vol/1000; % (kWhr) Estimated total pack energy
ess_soc=[0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1];  % state of charge

% module's resistance to being charged, indexed by ess_soc
ess_r_dis=[	0.0377	0.0338	0.0300	0.0280	0.0275	0.0268	0.0269	0.0273	0.0283	0.0298	0.0312   ]; % discharge
ess_r_chg=[	0.0235	0.0220	0.0205	0.0198	0.0198	0.0196	0.0198	0.0197	0.0203	0.0204	0.0204   ]; % charge
% module's open-circuit (a.k.a. no-load) voltage, indexed by ess_soc
ess_voc=[7.2370	7.4047	7.5106	7.5873	7.6459	7.6909	7.7294	7.7666	7.8078	7.9143	8.3645];  


ess_module_num = 28;  % number of modules in Prius  ?

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% vehicle parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M_total = 1300 + 350 * 0.45359237; % ? Vehicle total mass (kg) = curb weight plus two passengers and cargo (350 lb)

g = 9.81;
R_tire = 0.287; % Tire radius (m) 195/55R tire
A_frontal = 2.52; % Vehicle frontal area (m^2)% estimation
rho_air = 1.2;			% (kg/m^3) Air density 

C_d = 0.26;				% ? Aerodynamic drag coefficient

f_rolling = 0.015; % Rolling resistance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% mechanical gear set parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% planetary gear set parameters
tx_pg_s=30; %number of teeth in sun gear
tx_pg_r=78; %number of teeth in ring gear
gear_no=4; %number of pinion gears

%prius planetary gear inertia assumed to be zeros
I_r=0; % ring gear inertia
I_s=0; % sun gear inertia
I_c=0; % carrier gear inertia

I_m = m_inertia; % motor inertia
I_g = m_inertia; % generator inertia
I_e = e_inertia; % engine inertia

% gear ratios
FR = 3.905; % Final drive ratio


%% Part 3b

Nm_kW = 1/9549;
% Plot curves for engine and motor rpm * Nw
engine_power = eng_max_trq .* eng_map_spd * Nm_kW;

motor_power = m_max_trq .* m_map_spd * Nm_kW;

%plot the power (kW) and torque (Nm) vs. speed (rpm) curves for engine and
% electric motor.

figure(1);
% plot(eng_map_spd,engine_power,'r',eng_map_spd,eng_max_trq,'g','LineWidth',2);

yyaxis left
plot(eng_map_spd,eng_max_trq,'g','LineWidth',2);
ylim([0 500])
xlabel('Engine speed [rpm] ','FontSize',10);
ylabel('Torque (Nm) ','FontSize',10);

yyaxis right
plot(eng_map_spd,engine_power,'r','LineWidth',2);
ylabel('Engine Power (kW)  ','FontSize',10);
ylim([0 60])

legend ({'$Torque$','$Power$'},'Interpreter','latex')
title(" Power (kW) and torque (Nm) vs. speed (rpm) curves for Engine");
grid on;
print hw2_ex3b_engine_me566 -dpng;


figure(2);
% plot(m_map_spd,motor_power,'r',m_map_spd,m_max_trq,'g','LineWidth',2);
% colororder({'b','m'})

yyaxis left
plot(m_map_spd,m_max_trq,'g','LineWidth',2);
ylim([0 500])
xlabel('Motor speed [rpm] ','FontSize',10);
ylabel('Motor Torque (Nm) ','FontSize',10);

yyaxis right
plot(m_map_spd,motor_power,'r','LineWidth',2);
ylabel('Motor Power (kW)  ','FontSize',10);
ylim([0 60])

legend ({'$Torque$','$Power$'},'Interpreter','latex')
title(" Power (kW) and torque (Nm) vs. speed (rpm) curves for Motor");
grid on;
print hw2_ex3b_Motor_me566 -dpng;


%% Part 3c

% State space model
% Assume zero pinion
% gear inertia, and neglect all vehicle dynamics except along the longitudinal direction
fprintf("Dynamic model equations of the vehicle : \n");

A = [I_s + I_g  0               0                                     -tx_pg_s;
    0           I_c+I_e         0                                      tx_pg_s+tx_pg_r;
    0           0               (R_tire/FR)^2*M_total + (I_m + I_r)   -tx_pg_r;
    -tx_pg_s    tx_pg_s+tx_pg_r -tx_pg_r                              0];

disp(A);









