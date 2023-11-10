% Corvette Vehicle Parameters
grav = 9.81;                    % gravity
air_dens = 1.199;               % density of air [kg/m^3]

% Unit Conversion factors
rps_rpm = 30/pi;     % rad/sec to rpm
rpm_rps = pi/30;
mps_mph = 2.237;     % m/s to mph
mph_mps = 1/2.237;

% Driveline + Vehicle
Mv = 1300;          % Vehicle mass [kg]
f0 = 0.008;         % N/Nweight
f1 = 7.433;         % N/mph
f2 = 0.237;         % N/mph^2
Rw = 0.406 * 0.96;    % Tire radius[m] * efficiency

Tdown = 0.95;       % Percentage Loss of traction torque 
Iw = 2.0;

% Engine
Je_engine = 0.19 + 0.1761;  % engine + impeller inertia
idle_Ne=650;        % Idle Ne
idle_fuel=0.441;    % Idle fuel rate (Run Cal_idle_fuel.m)

% TC
TCC_gear=[3 4 5 6];         % TCC available gears
TCC_slip = [40 40 20 20];   % TCC slip speed for TCC gears [rpm]

% Transmission ratio
% 4-Speed
% gear_level = [1:4];
% gear_ratio =[4.027    1.825    1.000    0.668];  % [1st 2nd...]
% gear_eff = [0.963      0.993    0.995      0.993]; % gear efficiency [1st 2nd...]
% gear_inertia = [0.11	0.034	0.024	0.024];
% % 6-Speed
% gear_level = [1:6];
% gear_ratio =[4.027   2.364    1.532    1.152    0.852    0.668];  % [1st 2nd...]
% gear_eff = [0.963    0.971     0.993     0.993    0.995    0.993]; % gear efficiency [1st 2nd...]
% gear_inertia = [0.11	0.11	0.034	0.024	0.024	0.024];

FD_ratio = 3.8;        % Final drive ratio
FD_eff = 0.966;         % Final drive efficiency

% V_limit = max(eng_spd).*rpm_rps./gear_ratio./FD_ratio.*Rw.*mps_mph; % Speed limit for each gear
% V_max_limit = ceil(V_limit(end))+5; % Maximum speed limit

% Initial Conditions
theta = 0;                % road grade (assumed to be flat)