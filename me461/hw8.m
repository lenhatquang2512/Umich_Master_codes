clc;clear;close all;format default;

%Design 
syms s 

% input transfer function here
G = 27.5 / (s^2 + 6*s+  2.75);

% convert to tf type
[numG, denG] = numden(G);
numG = sym2poly(numG);
denG = sym2poly(denG);
G_tf = tf(numG, denG);

%Bode plot of 
figure(1)
bode(G_tf);
grid on;
% print -dpng img/hw8_Bode_G_part_1
 
%still bode but with Gain margin and Phase margin
figure(2)
margin(G_tf)
[Gm,Pm,Wcg,Wcp] = margin(G_tf);
grid on;
% print -dpng img/hw8_GM_andPM_stable__part1

GM_dB = 20 * log10(Gm)
PM_deg = Pm

%% 2
%CLTF of negative unity feedback 
% H = tf;
CLTF_baseline = feedback(G_tf,1);

%% 3
[y,~] = step(CLTF_baseline)
% print -dpng img/hw8_step_part_3
sserror=abs(1-y(end))

S_baseline = stepinfo(CLTF_baseline)


%% 4
 %view poles and zeros of TF
% fprintf("Pole-zero map test: \n");
figure();
pzmap(CLTF_baseline)
grid on;
% print -dpng img/hw8_pzmap_part4

Denominator_CLTF = CLTF_baseline.Denominator{1,1};

roots_CLTF = roots(Denominator_CLTF)
s1 = roots_CLTF(2) % TUNE choose number index of dominant poles

sig_CL = abs(real(s1)); % assign constant parameters
wd =  imag(s1);
wn = sqrt(sig_CL^2 + wd^2);
zeta = sig_CL/wn;

%% 5

%find the bandwidth using CLTF
wBW = bandwidth(CLTF_baseline)
gpeak = getPeakGain(CLTF_baseline);
Mr = 20 * log10(gpeak) % convert to dB

%test to see bandwidth and resonant peak by checking Bode close loop plot
figure()
bode(CLTF_baseline)
% print -dpng img/hw8_Bode_CL_part5

%% 7
Kp = 20; %desired 
K_head = Kp * 2.75/27.5;
 G_head = K_head * G_tf;
% bode(G_head)
% grid on;
% print -dpng img/hw8_Bode_Ghead_part_7

margin(G_head)
grid on;
% print -dpng img/hw8_Bode_Ghead_part_7
[Gm,Pm,Wcg,Wcp] = margin(G_head);

% print -dpng img/hw8_GM_andPM_stable__part1

GM_dB = 20 * log10(Gm)
PM_deg = Pm


%% 9

phi_m = 15.2 %[deg]
alpha = (1- sind(phi_m + 5))/ (1 + sind(phi_m + 5))


%% 10

mag_Gjw = - 20 * log10(1/ sqrt(alpha))

%% 11
w1 = 8.06 %rad/sec
sig_z = w1 * sqrt(alpha)
sig_p = w1/sqrt(alpha)

Kc = K_head/alpha


%% 12

% input transfer function here
Gc = Kc * (s + sig_z)/(s + sig_p)

% convert to tf type
[numGc, denGc] = numden(Gc);
numGc = sym2poly(numGc);
denGc = sym2poly(denGc);
Gc_tf = tf(numGc, denGc);

G_compensated = series(Gc_tf,G_tf)

%still Bode plot but has PM and GM
margin(G_compensated)
grid on;
print -dpng img/hw8_Bode_Gcompensated_part_12
[Gm,Pm,Wcg,Wcp] = margin(G_head);

%% 13

CLTF_compensated = feedback(G_compensated,1)

%% 14
step(CLTF_compensated)
% [y,~] = step(CLTF_compensated)
print -dpng img/hw8_step_Compensated_part_14
sserror=abs(1-y(end))

S_compenstated = stepinfo(CLTF_compensated)

%% 15

% fprintf("Pole-zero map test: \n");
% figure();
pzmap(CLTF_compensated)
grid on;
% print -dpng img/hw8_pzmap_part15

Denominator_CLTF = CLTF_compensated.Denominator{1,1};

roots_CLTF = roots(Denominator_CLTF)
s1 = roots_CLTF(2) % TUNE choose number index of dominant poles

sig_CL = abs(real(s1)); % assign constant parameters
wd =  imag(s1);
wn = sqrt(sig_CL^2 + wd^2);
zeta = sig_CL/wn;


%% 16

%find the bandwidth using CLTF
wBW = bandwidth(CLTF_compensated)
gpeak = getPeakGain(CLTF_compensated);
Mr = 20 * log10(gpeak) % convert to dB

%test to see bandwidth and resonant peak by checking Bode close loop plot
% figure()
bode(CLTF_compensated)
print -dpng img/hw8_Bode_CL_compensated_part16

