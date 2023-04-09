clc;clear;close all;format default;

%Design lead compensator
syms s Kc

% input transfer function here
G = 1/(s^2 + 7*s+12);

H = (4*s+4)/(2*s+1);

GH = G*H;

% convert to tf type
[numG, denG] = numden(G);
numG = sym2poly(numG);
denG = sym2poly(denG);
G_tf = tf(numG, denG);

[numH, denH] = numden(H);
numH = sym2poly(numH);
denH = sym2poly(denH);
H_tf = tf(numH, denH);

GH_tf = G_tf * H_tf; % OLTF init

% convert
G_func = matlabFunction(G,'Vars',s);
GH_func= matlabFunction(GH,'Vars',s);

% Desired specs
zeta = 0.8;
wn = 6;

sig_CL = abs(-zeta * wn);
sig = -zeta * wn;

% pick one desired poles
if zeta < 1
    wd = wn * sqrt(1 - zeta^2);
    s1 = sig + 1i*wd;
else
    wd = wn * sqrt(zeta^2 -1);  %not sure
    s1 = sig + wn * sqrt(zeta^2 -1);
    
end

%step 2
mag_GH = abs(GH_func(s1));
ang_GH =  -360 + rad2deg(angle(GH_func(s1)));  %angle in deg

phi_d = -180 - ang_GH;

% step3

sig_z = sig_CL + wd * tand((90 - phi_d - atand(sig_CL/wd))/2);

sig_p = sig_CL + wd * tand((90 + phi_d - atand(sig_CL/wd))/2);

T = 1/sig_z;

alpha = sig_z/sig_p;

%new controlled system
Gc = simplify(Kc * (s+(1/T))/(s + (1/(alpha*T))));
Gc_func = matlabFunction(Gc,'Vars',[s,Kc]);

GcGH = Gc * GH;
GcGH_func = matlabFunction(GcGH,'Vars',[s,Kc]);


% step 4 adjust Kc

GcGH_wo_Kc = (s+(1/T))/(s + (1/(alpha*T))) *GH;
GcGH_s1 = subs(GcGH_wo_Kc,s,s1);

mag_GcGH = Kc * abs(GcGH_s1);
% ang_GcGH =  -360 + rad2deg(angle(GCGH_func(s1)))  %angle in deg
Kc_sol = double(vpasolve(mag_GcGH == 1, Kc));

%step 5 close and verify

% G=tf(1,conv([1 1],[1 2]));
% G = tf(1,[1 7 12])

% Gc=tf(Kc_sol*[1 2.38],[1 3.78])
Gc_tf=tf(Kc_sol *[1 1/T],[1 1/(alpha*T)])

OLTF_new =series(Gc_tf,G_tf * H_tf)
CLTF_compensated=feedback(Gc_tf * G_tf,H_tf)

CLTF_baseline = feedback(G_tf,H_tf)

rltool(OLTF_new)


figure()
step(CLTF_baseline,'ro',CLTF_compensated,"b+")
legend ('Baseline','Compensated')
title('Step responses of the baseline and compensated systems')
% print -dpng img/hw4_part1b

figure()
pzmap(CLTF_compensated)
grid on;

%% second way of designing using rltool
% 
% OLTF_init = series(G_tf,H_tf);
% rltool(OLTF_init);

%then edit design requirement, add damping ratio and natural frequency
% then edit axis and add poles/zeros -> lead then tune the gain square
% correct;y -> read C tf , this is a compensator 


