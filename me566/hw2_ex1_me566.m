%% Part a
% close all; clear all; clc; format default;

% Engine parameters 
Pars_eng;

% convert
engine_speed  = eng_spd*2*pi/60;    % rad/sec
Throttle_grid = eng_th;

% find max power
max_power = max(engine_speed.*engine_torque(end,:)/1000);

%create power grid
engine_power = 0:12:(round(max_power)+3);

%size
m = length(Throttle_grid); %m = 21
n = length(eng_spd); % n = 19

[Xspd,Ythrottle] = meshgrid(eng_spd, Throttle_grid);

eng_tor = zeros(1,n);

% bsfc should be = fuel consumption / engine_out_power
bsfc = fuel_consumption./(Xspd.*abs(engine_torque)/1000);
% bsfc(bsfc<=0) = 0.2;
% bsfc(bsfc>0.2) = 0.2;

for  j = 1:n
     [bsfc_opt,id_opt]=min(bsfc(:,j));
%      w_e_opt(i)=eng_spd(id_opt);
     for i = 1:m
        if  bsfc(i,j) == bsfc_opt
            eng_torr(j) = abs(engine_torque(i,j));
        end
     end
end

for j = 1:n
    min_fuel_consump = min(fuel_consumption(:,j));
    for i = 1:m
        if fuel_consumption(i,j) == min_fuel_consump
            eng_tor(j) = abs(engine_torque(i,j));
        end
    end
end

%plot
figure();
plot(eng_spd,eng_tor(1:n),'r','LineWidth',2.5)

% generate BSFC contour map

% speedgrid = repmat(engine_speed , 21,1);
% speedgrid_plot = repmat(eng_spd , 21,1); %copy eng_spd
hold on 

plot(eng_spd,eng_torr(1:n),'g--','LineWidth',2.5)

% make the constant power curves
contour(Xspd,abs(engine_torque),(Xspd.*abs(engine_torque)/1000), "b--" ,'LineWidth',2.5);

xlabel('Engine speed [rpm]')
ylabel('Engine torque [Nm]')
title('Optimal Operating Line (Best BSFC line)');
legend ({'Optimal Line','Power curves'},'Interpreter','latex')
% axis([600 6000 0 450])
print -dpng hw2_me566_ex1a_Optimal_Operating_Line

%% Part b

% Vehicle parameters
Pars_veh;

% Transmission ratio
% 4-Speed
gear_level = 1:4;
gear_ratio =[4.027    1.825    1.000    0.668];  % [1st 2nd...]
gear_eff = [0.963      0.993    0.995      0.993]; % gear efficiency [1st 2nd...]
gear_inertia = [0.11	0.034	0.024	0.024];

w_e_opt=[600,600,600,600,800,1200,1600,2100,2160,2180,2200,2220,2240,2260,2280,2300,2320,2340,2360,2380,2400];

% Calculate NV ratio, with unit of RPM/MPH
NV_ratio = FD_ratio * gear_ratio/Rw*60/(2*pi)*1602/3600;

M_up = zeros(length(Throttle_grid), length(gear_level));
M_down = zeros(length(Throttle_grid), length(gear_level));

for k = 1:length(Throttle_grid)

     M_up(k,1) = 0.5*w_e_opt(k)*(1/NV_ratio(1)+1/NV_ratio(2)); % 1 to 2
     M_up(k,2) = 0.5*w_e_opt(k)*(1/NV_ratio(2)+1/NV_ratio(3)); % 2 to 3
     M_up(k,3) = 0.5*w_e_opt(k)*(1/NV_ratio(3)+1/NV_ratio(4)); % 3 to 4
     M_up(k,4) = 300;
    
     M_down(k,1) = 0;
     M_down(k,2) = min(M_up(k,1)*0.85, M_up(k,1)-4);
     M_down(k,3) = min(M_up(k,2)*0.85, M_up(k,2)-4);
     M_down(k,4) = min(M_up(k,3)*0.85, M_up(k,3)-4);

end

figure(2); clf;
plot(M_up(:, 1:length(gear_level)-1), Throttle_grid, 'ro-', ...
    M_down(:,2:length(gear_level)), Throttle_grid, 'bx-.')

xlabel('Vehicle speed (mph)')
ylabel('Throttle (%)')
title('4-spd Transmission (Upshift: red o Down shift: blue x)');
print -dpng hw2_me566_ex1b_4-speed-shift-map

