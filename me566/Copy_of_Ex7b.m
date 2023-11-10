%% Minimum BSFC shift map concept 

close all; clear all; clc; format default;

% Vehicle parameters 
Pars_eng;

% Load the engine map information
% load engine_parameters
Pars_veh;

% One of the maps loaded is the engine torque matrix (N-m)
% Another is fuel comsumption (g/sec)
% Both of which is a 21 by 19 matrix
% The row (throttle) index is 0:5:100
% The column (engine speed) index is 600:200:6000 (rpm)

engine_speed  = eng_spd*2*pi/60;    % rad/sec
% Throttle_grid=[0:5:100];

for k = 1:21
    for j = 1:19
       engine_out_power(k,j) = engine_speed(j)*engine_torque(k,j)/1000;    % engine out power (kW)
       bsfc(k,j) = fuel_consumption(k,j)/engine_out_power(k,j);                     % BSFC = g/sec/kW
       if (bsfc(k,j) < 0)
           bsfc(k,j) = 0.2;
       elseif (bsfc(k,j) > 0.2)
           bsfc(k,j) = 0.2;
       end
    end
end

w_e_opt = [600,600,600,600,877,1155,1433,1711,1988, 2267:20:2487];  % RPM!!
% w_e_opt = [600,600,600,600,960,1320,1680,2040,2400, 2420:20:2640] ;  % RPM!!

figure(1)
contour(eng_spd, eng_th, bsfc, 150)

hold on

plot(w_e_opt,eng_th,'g',"LineWidth",2.5)

xlabel('Engine speed (rpm)')
ylabel('Throttle [0:5:100]')
print -dpng BSFC_with_best_BSFC_line_ex2a


%% Part 2 4-speed
% Optimal engine speed was obtained by visually examining the plot above
clear; clc;

FD_ratio = 3.8;        % Final drive ratio
Rw = 0.406 * 0.96;    % Tire radius[m] * efficiency

% Transmission ratio
% 4-Speed
gear_level = [1:4];
gear_ratio =[4.027    1.825    1.000    0.668];  % [1st 2nd...]
gear_eff = [0.963      0.993    0.995      0.993]; % gear efficiency [1st 2nd...]
gear_inertia = [0.11	0.034	0.024	0.024];

Throttle = [0,5,10,15,20,25,30,35,40,45:5:100]; 
% w_e_opt = [600,600,600,600,960,1320,1680,2040,2400, 2420:20:2640];  % RPM!!
w_e_opt = [600,600,600,600,877,1155,1433,1711,1988, 2267:20:2487];  % RPM!!


% Calculate NV ratio, with unit of RPM/MPH
NV_ratio = FD_ratio * gear_ratio(:)/Rw*60/(2*pi)*1602/3600;

for k = 1:21
  M_up(k,1) = 0.5*w_e_opt(k)*(1/NV_ratio(1)+1/NV_ratio(2));  % 1 to 2
  M_up(k,2) = 0.5*w_e_opt(k)*(1/NV_ratio(2)+1/NV_ratio(3));  % 2 to 3
  M_up(k,3) = 0.5*w_e_opt(k)*(1/NV_ratio(3)+1/NV_ratio(4));  % 3 to 4
  M_up(k,4) = 300;
   
  M_down(k,1) = 0;
  M_down(k,2) = min(M_up(k,1)*0.85, M_up(k,1)-4);
  M_down(k,3) = min(M_up(k,2)*0.85, M_up(k,2)-4);
  M_down(k,4) = min(M_up(k,3)*0.85, M_up(k,3)-4);
  
end

figure(2)
plot(M_up(:, 1:3), 0:5:100, 'ro-', M_down(:,2:4), 0:5:100, 'bx-.')
xlabel('Vehicle speed (mph)')
ylabel('Throttle (%)')
title('Upshift: red o    Down shift: blue x')
save('4_speed_shift_map_data.mat','M_up');
save('4_speed_shift_map_data.mat','M_down','-append');
print -dpng 4-speed-shift-map


%% part 2 6-speed
clear;clc;

FD_ratio = 3.8;        % Final drive ratio
Rw = 0.406 * 0.96;    % Tire radius[m] * efficiency

% % 6-Speed
gear_level = [1:6];
gear_ratio =[4.027   2.364    1.532    1.152    0.852    0.668];  % [1st 2nd...]
gear_eff = [0.963    0.971     0.993     0.993    0.995    0.993]; % gear efficiency [1st 2nd...]
gear_inertia = [0.11	0.11	0.034	0.024	0.024	0.024];


Throttle = [0,5,10,15,20,25,30,35,40,45:5:100]; 
% w_e_opt = [600,600,600,600,960,1320,1680,2040,2400, 2420:20:2640];  % RPM!!
w_e_opt = [600,600,600,600,877,1155,1433,1711,1988, 2267:20:2487];  % RPM!!


% Calculate NV ratio, with unit of RPM/MPH
NV_ratio = FD_ratio * gear_ratio(:)/Rw*60/(2*pi)*1602/3600;

for k = 1:21
  M_up(k,1) = 0.5*w_e_opt(k)*(1/NV_ratio(1)+1/NV_ratio(2));  % 1 to 2
  M_up(k,2) = 0.5*w_e_opt(k)*(1/NV_ratio(2)+1/NV_ratio(3));  % 2 to 3
  M_up(k,3) = 0.5*w_e_opt(k)*(1/NV_ratio(3)+1/NV_ratio(4));  % 3 to 4
  M_up(k,4) = 0.5*w_e_opt(k)*(1/NV_ratio(4)+1/NV_ratio(5));  % 4 to 5
  M_up(k,5) = 0.5*w_e_opt(k)*(1/NV_ratio(5)+1/NV_ratio(6));  % 5 to 6
  M_up(k,6) = 300;
   
  M_down(k,1) = 0;
  M_down(k,2) = min(M_up(k,1)*0.85, M_up(k,1)-4);
  M_down(k,3) = min(M_up(k,2)*0.85, M_up(k,2)-4);
  M_down(k,4) = min(M_up(k,3)*0.85, M_up(k,3)-4);
  M_down(k,5) = min(M_up(k,4)*0.85, M_up(k,4)-4);
  M_down(k,6) = min(M_up(k,5)*0.85, M_up(k,5)-4);
  
end

figure(3)
plot(M_up(:, 1:5), 0:5:100, 'ro-', M_down(:,2:6), 0:5:100, 'bx-.')
xlabel('Vehicle speed (mph)')
ylabel('Throttle (%)')
title('Upshift: red o    Down shift: blue x')
save('6_speed_shift_map_data.mat','M_up')
save('6_speed_shift_map_data.mat','M_down','-append')
print -dpng 6-speed-shift-map










