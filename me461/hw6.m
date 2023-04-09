clc;clear;close all; format default;

% G = tf(15*[1 4],[1 6])
% 
% bode(G)

% bode
% [mag,phase,wout] = bode(G)


% %plot results, with frequency expressed at Hz
% figure;
% subplot(2,1,1);
% semilogx(wout(:,1)/(2*pi), 20*log10(squeeze(mag)), '-b'); zoom on; grid on; 
% title('magnitude'); xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
% subplot(2,1,2);
% semilogx(wout(:,1)/(2*pi), squeeze(phase), '-r'); zoom on; grid on; 
% title('Phase'); xlabel('Frequecy (Hz)'); ylabel('Phase (deg)');

%%

clc;clear;close all;format default;

%Design 
syms s 

% input transfer function here
G = (16 * (s+0.1))/(s * (s+0.01) * (s^2 + 3*s+9));

% convert to tf type
[numG, denG] = numden(G);
numG = sym2poly(numG);
denG = sym2poly(denG);
G_tf = tf(numG, denG);

%Bode plot of 
figure(1)
bode(G_tf);
grid on;
print -dpng img/hw6_Bode_G_part2

%---- Part 3
%still bode but with Gain margin and Phase margin
figure(2)
margin(G_tf)
[Gm,Pm,Wcg,Wcp] = margin(G_tf);
grid on;
print -dpng img/hw6_GM_andPM_stable__part3

GM_dB = 20 * log10(Gm)
PM_deg = Pm

%----Part 4
%CLTF of negative unity feedback 
% H = tf;
CLTF_baseline = feedback(G_tf,1);

%----- Part 5
%add Kc compensator
Kc = 5.17;
figure(3)
bode(Kc * G_tf)
grid on;
print -dpng img/hw6_Bode_new_Kc_compensator__part5

figure(4)
margin(Kc * G_tf)
[Gm_new,Pm_new,Wcg_new,Wcp_new] = margin(Kc * G_tf);
grid on;
print -dpng img/hw6_new_GM_andPM_unstable__part5

GM_new_dB = 20 * log10(Gm_new)
PM_new_deg = Pm_new

%----- Part 6
figure(5)
bode(CLTF_baseline)
grid on;
print -dpng img/hw6_Bode_CLTF_baseline_part6
%find the bandwidth using CLTF
wBW = bandwidth(CLTF_baseline)
gpeak = getPeakGain(CLTF_baseline);
Mr = 20 * log10(gpeak) % convert to dB


%----- Part 7
figure(6)
nichols(G_tf)
grid on;
print -dpng img/hw6_Nichols_OLTF_G_part7















