clc;clear;close all; format default;

% generate fuel cell (FC) power command map using the ECMS concept
% for a fuel cell vehicle (no engine)

% Fuel cell parameters
P_fc = 1:1:100;
P_fc_sample = 0:10:100; %kW
hydroConsump_fc =  [0 0.15 0.18 0.29 0.4 0.55 0.68 0.79 0.9 1.1 1.3]; %[g/s]
fuel_fc = interp1(P_fc_sample,hydroConsump_fc,P_fc,'linear'); %interpolation to get correcr indexes
        
%SOC parameters (penalty factor)
soc_L = 0.5;
soc_H = 0.7;
soc = 0.4:0.01:0.8; % soc from 40% to 80%
x_soc =  (soc - (soc_L+soc_H)/2)/(soc_H-soc_L);
f_soc = 1-(1-0.7.*x_soc).*x_soc.^3;                % 1 by41

figure(1)
plot(soc, f_soc)
xlabel('SOC'), ylabel('f_S_O_C')
% print hw4_fsoc -dpng

SC_eng = 0.01;     % conversion factor from P_batt to equivalent fuel consumption
Eff_elec = 1.00;   %electric efficiency is ideal 100%

%Initialization
Pd = zeros(1,120);
P_motor = zeros(120,100);
P_batt = zeros(120,100);
fuel_batt = zeros(41,120,100);
fuel_total = zeros(41,120,100);

% Fuel cell motor can be powered by either fuel cell stack or electric battery or both
% FC stack has a 100kW max power capacity. The battery has a 120kW max power capacity
for i = 1:120
  Pd(i) = i*1.0;	% Pd from 1 to 120kw
  P_motor(i,:) = Pd(i)*ones(1,100) - P_fc; 
  
  for j = 1:100
    if P_motor(i,j) > 0
      P_batt(i,j) = P_motor(i,j)/Eff_elec;  % assume a constant battery/power electronics efficiency
    else                                    % Can be modified to be SOC dependent
      P_batt(i,j) = P_motor(i,j)*Eff_elec;  % Charging efficiency does not need to be 
    end                                     % the same as discharging efficiency but here we use the same
  end

  for k = 1:41
    fuel_batt(k,i,:) = f_soc(k)*SC_eng*P_batt(i,:)/Eff_elec; %equivalent hydrogen consumption for fuel cell stack to charge the battery
    fuel_total(k,i,:) = reshape(fuel_fc,1,1,100) + fuel_batt(k,i,:) ;  
  end
end

% Showing example at a particular case, where Pd and SOC are specified by
% index1 and index2, respectively
index1 = 50;    % index1 here = Pd (kW)
index2 = 21;    % index2 indicate which SOC level in [0.4:0.01:0.8] is selected  (60%)

%plot
figure(2)
subplot(411), plot(P_fc, fuel_fc)
title('Fuel Cell Power vs. Hydrogen Consumption')
xlabel('Fuel cell Power (kW)'), ylabel('Fuel_f_c')

title('Pd = 50kW, SOC = 0.6')
subplot(412), plot(P_fc, P_batt(index1,:))
xlabel('Fuel cell Power (kW)'), ylabel('Battery power(kW)')

subplot(413), plot(P_fc, squeeze(fuel_batt(index2,index1,:)))
xlabel('Fuel cell Power (kW)'), ylabel('Fuel_b_a_t_t')

subplot(414), plot(P_fc, squeeze(fuel_total(index2,index1,:)))
xlabel('Fuel cell Power (kW)'), ylabel('Total fuel') %g/s

hold on
[fuel_min, index] = min(fuel_total(index2,index1,:));
plot(P_fc(index), fuel_min, 'r*'), 
% print hw4_ex2b_me566_optim_50Pd_60soc -dpng
hold off

% Find optimal fuel cell power
P_fc_optimal = zeros(41,120);
for index1 = 1:120                 % Pd from 1 to 120 kW
    for index2 = 1:41               % SOC 0.4:0.01:0.8
        [fuel_min(index2, index1), index] = min(fuel_total(index2,index1,:));
        P_fc_optimal(index2, index1) = P_fc(index);
    end
end

%plot 3D
figure(3)
mesh(Pd, soc, P_fc_optimal)
xlabel('Pd (kW)'), ylabel('SOC'), zlabel('Optimal Fuel cell power (kW)')
% print hw4_ex2c_me566_optim_lookup_table_Pfc -dpng;

% Generate optimal decision of fuel cell stack power table
P_fc_optimal_table = zeros(41,length(10:10:120));
for index1 = 10:10:120   %Pd = 10:10:120kW
    for index2 = 1:41    %SOC = 0.4:0.01:0.8 - 40 to 80%
        P_fc_optimal_table(index2, index1/10) = P_fc_optimal(index2, index1);
    end
end

