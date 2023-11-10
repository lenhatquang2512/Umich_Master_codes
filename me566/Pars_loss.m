% ACCESSORIES

% ALTERNATOR
alt_ratio = 3.23;
alt_eff = 0.98;

% 1st Column: SPEED	  2nd Column: TORQUE
alt_dat=[     0	  7.04
  1000	  7.04
  1500	  4.69
  2000	  3.52
  2500	  2.78
  3000	  2.32
  3500	  2.02
  4000	  1.80
  6000	  1.35
  8000	  1.16
 10000	  1.05
 12000	  1.07
 14000	  1.09];

% POWER STEERING
pw_str_ratio = 1.10;
pw_str_eff = 0.98;

% 1st Column: SPEED	  2nd Column: TORQUE
pw_str_dat=[     0	  0.00
   500	  1.64
   720	  2.38
  5600	  3.67
 10000	  3.67];

%FAN
fan_ratio = 1.00;
fan_eff = 1.00;

% 1st Column: SPEED	  2nd Column: TORQUE
fan_dat=[     0	  0.00
  7000	  0.00];

lbf2Nm=1.355818;
pump_speed = [ 0	800	900	1000	1500	2000	2500	3000	3500	4000	4500	5000	7000];
TPS = [0	
      100]';
Pump_Loss = [ 1.3800    1.3800    1.2500    1.1100    1.2400    1.3400    1.7800    1.9500    2.3500    2.7900    3.1400    3.0600    3.0600;
            4.6600    4.6600    3.7600    3.2900    3.5700    2.7000    2.8200    3.1300    3.3000    3.5800    4.4500    4.5100    4.5100]*lbf2Nm;

