clear all;
clc;
close all;
format default;

% INPUT YOUR 4-SPEED & 6-SPEED SHIFT MAPS DATA HERE.

% Transmission ratio
% % 4-Speed
% gear_level = [1:4];
% gear_ratio =[4.027    1.825    1.000    0.668];  % [1st 2nd...]
% gear_eff = [0.963      0.993    0.995      0.993]; % gear efficiency [1st 2nd...]
% gear_inertia = [0.11	0.034	0.024	0.024];
% 
% load('4_speed_shift_map_data.mat');

% % 6-Speed
gear_level = [1:6];
gear_ratio =[4.027   2.364    1.532    1.152    0.852    0.668];  % [1st 2nd...]
gear_eff = [0.963    0.971     0.993     0.993    0.995    0.993]; % gear efficiency [1st 2nd...]
gear_inertia = [0.11	0.11	0.034	0.024	0.024	0.024];

load('6_speed_shift_map_data.mat');

% Kindly refer to the shift block in the simulink model Ex7s.mdl and 
% name the variables in the matlab script file accordingly.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Load Parameter files: Pars_eng,Pars_TC,Pars_loss,Pars_veh,Pars_tire
Pars_eng;       % Engine fuel consumption and torque
Pars_TC;        % Torque converter Torque ratio and K-factor
Pars_tire;      % Tire slip-Fx relationship
Pars_loss;      % All accessary losses
Pars_veh;       % vehicle parameters

cycle=1;    % CHOOSE cycle=0    Constant throttle simulation
            %        cycle=1    EPA city cycle simulation
            %        cycle=2    EPA highway cycle simulation

% Switch settings
TCC_sw=0;           % TCC on/off    1/0
schedule_exist=0;   % Separate TCC schedule exist (0=no, 1=yes)
 
% % IF cycle=0, the following throttle and simulation time will be used
Throttle=100;
% Simulation_time=30;
 
Kp =  [15   15]';                  
Kp_prev = [15  40]';
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Inputs
theta=0;                  % Road grade
Bw=0.0;
Max=500; Max_t=300;
Kc=4; Kic=5; Ktc=5; 
Max_brake=20000;
 
if schedule_exist==0
    TCC_throttle=[0 100];
    TCC_app=300*ones(length(TCC_throttle),length(TCC_gear)+1);
    TCC_rel=zeros(length(TCC_throttle),length(TCC_gear)+1);
end
 
v_max_c = 100;
 
if (cycle==1)
    load_cycles;
    cyc=cyc_city(1:end);                    % City cycle = 1
    time=[0:1:length(cyc)-1]';
    fprintf("City cycle ! :\n");
elseif (cycle==2)
    load_cycles;
    cyc=cyc_hi;                             % Highway cycle = 2
    time=[0:1:length(cyc)-1]';
    fprintf("Highway cycle ! :\n");
else
    time=[0:1:Simulation_time]';
    cyc=zeros(size(time));
end
 
V_prof=cyc;
time_shift=1;
V_prof_pre=[cyc(1+time_shift:end,1); zeros(time_shift,1)];
 
samp_time=0.01;
% sim('Ex7s',[0 max(time)]);
sim('Copy_of_Ex7s',[0 max(time)]);
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation results
format compact
 
if cycle~=0
 
    MPG=Dist(end)*0.621/(FC(end)/724*264.17) %unit conversion 
end

% max(V_er)
