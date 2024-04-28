%% Problem 4 - HEV - Series
% Compute fuel consumption of a thermostatic controlled 
% Series hybrid vehicle for the Highway and Urban Cycles

clc; clear; close all; format default;
ex10_1;     % Load and run ex10_1 to compute driving power  
ex10_2;     % Load and run ex10_2 to compute engine map

% figure(3)
% hold on, plot([600;4000], [18; 66], '-', 'LineWidth', 3)
% hold off
% 
% figure(4)
% hold on, plot([600;4000], [18; 66], '-', 'LineWidth', 3)
% hold off

% The optimal operation of the engine is characterized by the end points
% [600rpm, 18% throttle] and [4000rpm, 66% throttle].  

opt_eng_speed = [600: 3400/48: 4000];
opt_throttle = [18:1:66];

% Interpolate to find fuel consumption (g/sec) and power (kW) for each
% point
opt_fuel = interp2([600:200:6000], Throttle_grid, fuel_map, opt_eng_speed, opt_throttle);
opt_power = interp2([600:200:6000]*2*pi/60, Throttle_grid, engine_out_power, ...
                    opt_eng_speed*2*pi/60, opt_throttle);
% figure(5), subplot(211)
% plot(opt_eng_speed, opt_fuel), xlabel('Engine speed (rpm)'), ylabel('fuel cnsumption (g/sec)')
% subplot(212)
% plot(opt_eng_speed, opt_power), xlabel('Engine speed (rpm)'), ylabel('power (kW)')

% Simulate for the two drive cycles, series hybrid, themodtatic control
SOC_max_power = 0.25;
SOC_low = 0.35;
SOC_high = 0.50;
threshold_power = 42;   % kW - CHANGED
max_power = 84;        % kW - CHANGED
Battery_capacity = 2.5;   % kW-hr

% Simulate for the highway cycle
total_fuel_highway_hev = 0;
SOC(1) = 0.5; % CHANGED
flag = 0;       % flag tracks whether we are on the top or bottom of the hysteresis rectangle
for i = 1: (size(time_highway, 1)-1)
    if SOC(i) > SOC_high
        P_eng(i) = 0; flag = 0;
    elseif SOC(i) > SOC_low
        if  flag == 0
            P_eng(i) = 0;
        else
            P_eng(i) = threshold_power;
        end
    elseif SOC(i) > SOC_max_power
        flag = 1;
        P_eng(i) = max_power - (SOC(i)-SOC_max_power)/(SOC_low-SOC_max_power) ...
               *(max_power-threshold_power);
    else
        flag = 1;    P_eng(i) = max_power;
    end
        
    P_batt(i) = power_highway(i)/1000 - P_eng(i);
    if P_batt(i) > 0
        SOC(i+1) = SOC(i) - P_batt(i)*1.0/3600/0.92/Battery_capacity;
    else
        SOC(i+1) = SOC(i) - P_batt(i)*1.0/3600*0.92/Battery_capacity;
    end
    
    if P_eng(i) == 0
        fuel_hev(i) = 0;
    else
        fuel_hev(i) = interp1(opt_power, opt_fuel, P_eng(i), 'linear');
    end
    total_fuel_highway_hev = total_fuel_highway_hev + fuel_hev(i);
 
end

figure(6), subplot(221)
plot([1: size(time_highway,1)-1], P_eng)
xlabel('Time (sec)'), ylabel('Engine power (kW)')
subplot(222), plot([1: size(time_highway,1)-1], fuel_hev)
xlabel('Time (sec)'), ylabel('Engine fueling rate(g/s)')
subplot(223), plot([1: size(time_highway,1)], SOC)
xlabel('Time (sec)'), ylabel('SOC')
subplot(224), plot([1: size(time_highway,1)-1], P_batt)
xlabel('Time (sec)'), ylabel('Battery power (kW)')
sgtitle('Results for the highway CYC-HWFET (series HEV)','FontSize',13,'FontWeight','bold')
print p4_img/ex4a_hw3_me568_series_hev_hwfet_highway -dpng;

% Simulate for the urban cycle
clear P_eng SOC P_batt fuel_hev
total_fuel_urban_hev = 0;
SOC(1) = 0.5; % CHANGED
flag = 0;       % flag tracks whether we are on the top or bottom of the hysteresis rectangle
% P_eng = zeros(size(time_urban, 1)-1,1); % CHANGED
for i = 1: (size(time_urban, 1)-1)
    if SOC(i) > SOC_high
        P_eng(i) = 0; flag = 0;
    elseif SOC(i) > SOC_low
        if  flag == 0
            P_eng(i) = 0;
        else
            P_eng(i) = threshold_power;
        end
    elseif SOC(i) > SOC_max_power
        flag = 1;
        P_eng(i) = max_power - (SOC(i)-SOC_max_power)/(SOC_low-SOC_max_power) ...
               *(max_power-threshold_power);
    else
        flag = 1;    P_eng(i) = max_power;
    end
        
    P_batt(i) = power_urban(i)/1000 - P_eng(i);
    if P_batt(i) > 0
        SOC(i+1) = SOC(i) - P_batt(i)*1.0/3600/0.92/Battery_capacity;
    else
        SOC(i+1) = SOC(i) - P_batt(i)*1.0/3600*0.92/Battery_capacity;
    end
    if P_eng(i) == 0
        fuel_hev(i) = 0;
    else
        fuel_hev(i) = interp1(opt_power, opt_fuel, P_eng(i), 'linear');
    end
    total_fuel_urban_hev = total_fuel_urban_hev + fuel_hev(i);
end

figure(7), subplot(221)
plot([1: size(time_urban,1)-1], P_eng)
xlabel('Time (sec)'), ylabel('Engine power (kW)')
subplot(222), plot([1: size(time_urban,1)-1], fuel_hev)
xlabel('Time (sec)'), ylabel('Engine fueling rate(g/s)')
subplot(223), plot([1: size(time_urban,1)], SOC)
xlabel('Time (sec)'), ylabel('SOC')
subplot(224), plot([1: size(time_urban,1)-1], P_batt)
xlabel('Time (sec)'), ylabel('Battery power (kW)')
sgtitle('Results for the Urban CYC-US06 (series HEV)','FontSize',13,'FontWeight','bold')
print p4_img/ex4b_hw3_me568_series_hev_US06_urban -dpng;

% connvert fuel econpmy results from meter/gram to mpg
mpg_highway_HEV = (distance_highway/1602)/(total_fuel_highway_hev/0.74/3785)  
mpg_urban_HEV = (distance_urban/1602)/(total_fuel_urban_hev/0.74/3785)  

