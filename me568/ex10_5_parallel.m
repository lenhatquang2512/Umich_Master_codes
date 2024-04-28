%% Problem 4 - HEV - Parallel
% Compute fuel consumption of a load levelling 
% Parallel hybrid vehicle for the Highway and Urban Cycles

clc; clear; close all; format default;
load CYC_US06.mat              % Load the US06 cycle
% load CYC_HWFET.mat % Load the highway cycle - CHANGED

time_US06 = cyc_mph(:,1);
speed_US06 = cyc_mph(:,2);
% figure(1), subplot(211)
% plot(time_US06, speed_US06)
% xlabel('Time (Sec)')
% ylabel('Speed (mph)')
% % title('US06 cycle')
% title('HWFET cycle')

m = 1800;
g = 9.81;
rolling_resistance_coeff = 0.015;
aero_dynamic_drag_coeff = 0.4;
road_grad = 0;                  % road grade = 0 rad

N = size(time_US06, 1);
speed_US06 = speed_US06*1602/3600;  % Change the unit of speed to m/sec
distance_US06 = 0;

for i = 1:N-1
    accel_US06(i) = (speed_US06(i+1) - speed_US06(i))/1.0;  % delta_T = 1 sec
    F_resistant_US06(i) = rolling_resistance_coeff*m*g*cos(road_grad) ...
        +0.5*1.202*aero_dynamic_drag_coeff*speed_US06(i)^2 ...
        + m*g*sin(road_grad);            
    power_US06(i) = (m*accel_US06(i) + F_resistant_US06(i))*speed_US06(i)/1000;   % Power = F*V, in kW
    distance_US06 = distance_US06 + speed_US06(i);
end

% subplot(212)
% plot(time_US06(1:N-1), power_US06)
% xlabel('Time (Sec)')
% ylabel('Power (kW)')
% title('US06 cycle')

load engine_parameters
% Load engine torque (N-m)
% and fuel comsumption (g/sec)
% Both of which is a 21 by 28 matrix
% The row (throttle) index is 0:5:100
% The column (engine speed) index is 600:200:6000 (rpm)

engine_speed = [600:200:6000]*2*pi/60;    % rad/sec
Throttle_grid=[0:5:100];

% scale down the engine by 30% and use the same battery capacity
engine_torque = engine_torque * 0.7;    % Scale down engine by 30%
fuel_map = fuel_map * 0.7;

% figure(2)
% mesh([600:200:6000], Throttle_grid, fuel_map)
% xlabel('Engine speed (rpm)')
% ylabel('Throttle')
% zlabel('fuel consumption (g/sec)')

for k = 1:21
    for j = 1:28
       engine_out_power(k,j) = engine_speed(j)*engine_torque(k,j)/1000;    % engine out power (kW)
       bsfc(k,j) = fuel_map(k,j)/engine_out_power(k,j);                     % BSFC = g/sec/kW
       if (bsfc(k,j) < 0)
           bsfc(k,j) = 0.3;
       elseif (bsfc(k,j) > 0.3)
           bsfc(k,j) = 0.3;
       end
    end
end

% figure(3)
% contour([600:200:6000], Throttle_grid, bsfc, 50)
% xlabel('Engine speed (rpm)')
% ylabel('Throttle')
% title('BSFC g/sec/kW')
% hold on, plot(2350, 43, 'd'), hold off
% 
% figure(4)
% [CS,H]=contour([600:200:6000], Throttle_grid, engine_out_power);
% clabel(CS), xlabel('Engine speed (rpm)')
% ylabel('Throttle')
% title('Engine power (kW)') 
% hold on, plot(2350, 43, 'd'), hold off
% 
% figure(3)
% hold on, plot([600;4000], [18; 66], '-', 'LineWidth', 3)
% hold off
% 
% figure(4)
% hold on, plot([600;4000], [18; 66], '-', 'LineWidth', 3)
% hold off

opt_eng_speed = [600: 3400/48: 4000];
opt_throttle = [18:1:66];
opt_fuel = interp2([600:200:6000], Throttle_grid, fuel_map, opt_eng_speed, opt_throttle);
opt_power = interp2([600:200:6000]*2*pi/60, Throttle_grid, engine_out_power, ...
                    opt_eng_speed*2*pi/60, opt_throttle);

% Simulate for the US06 cycle, parallel hybrid, load leveling control
P_ev = 20;
Pe_max = 70;
P_ch = 20;
SOC_t = 0.35; % CHANGED
Battery_capacity = 2.5;   % kW-hr

% Simulate for the US06 cycle
total_fuel_US06_hev = 0;
SOC(1) = 0.5; % CHANGED

for i = 1: (size(time_US06, 1)-1)
    if SOC(i) >= SOC_t
        if  power_US06(i) <= P_ev
            P_eng(i) = 0;
        elseif power_US06(i) <= Pe_max
            P_eng(i) = power_US06(i);
        else
             P_eng(i) = Pe_max;
        end
    else
        if  power_US06(i) < 0
            P_eng(i) = 0;
        elseif power_US06(i) <= P_ev
            P_eng(i) = power_US06(i) + P_ch;
        elseif power_US06(i) <= Pe_max
            P_eng(i) = power_US06(i) + P_ch;
        else
             P_eng(i) = Pe_max;
        end
    end
    
    P_batt(i) = power_US06(i) - P_eng(i);  % unit: kW
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
    total_fuel_US06_hev = total_fuel_US06_hev + fuel_hev(i);
 
end

figure(5), subplot(221)
plot([1: size(time_US06,1)-1], P_eng)
xlabel('Time (sec)'), ylabel('Engine power (kW)')
subplot(222), plot([1: size(time_US06,1)-1], fuel_hev)
xlabel('Time (sec)'), ylabel('Engine fueling rate (g/s)')
subplot(223), plot([1: size(time_US06,1)], SOC)
xlabel('Time (sec)'), ylabel('SOC')
subplot(224), plot([1: size(time_US06,1)-1], P_batt)
xlabel('Time (sec)'), ylabel('Battery power (kW)')
% title('Results for the US06 cycle')
% sgtitle('Results for the highway CYC-HWFET (Parallel HEV)','FontSize',13,'FontWeight','bold')
sgtitle('Results for the Urban CYC-US06 (Parallel HEV)','FontSize',13,'FontWeight','bold')
% print p4_img/ex4a_hw3_me568_parallel_hev_hwfet_highway -dpng;
% print p4_img/ex4b_hw3_me568_parallel_hev_US06_urban -dpng;

% connvert fuel econpmy results from meter/gram to mpg
% mpg_US06_HEV = (distance_US06/1602)/(total_fuel_US06_hev/0.74/3785)  
% mpg_HWFET_HEV = (distance_US06/1602)/(total_fuel_US06_hev/0.74/3785)  

