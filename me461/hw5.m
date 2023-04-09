clc;clear;close all;format default;

%Design lead compensator
syms s Kc

% input transfer function here, assume unity feedback H = 1
% G = 10*(s+6)/((s+10)*(s^2+2*s+2)); %type 0
G = 100/(s^3 + 47*s^2 + 70*s) % type 1
% G = 8*(s+2) /(s^3 + 27*s^2 + 12*s +24)
% G = 3/(s+3)
% G = 10*(s+5)*(s+8)/((s+1)*(s^2 +4*s + 13))
% G = 8.5/(s+2)

% H = 2/(s+4);
H = s*1/s;

% convert to tf type
[numG, denG] = numden(G);
numG = sym2poly(numG);
denG = sym2poly(denG);
G_tf = tf(numG, denG);

[numH, denH] = numden(H);
numH = sym2poly(numH);
denH = sym2poly(denH);
H_tf = tf(numH, denH);

CLTF_init = feedback(G_tf,H_tf)

%%
% convert to function type
G_func = matlabFunction(G,'Vars',s);
sG_func = matlabFunction(simplify(s*G),'Vars',s);
ssG_func =  matlabFunction(simplify(s^2 *G),'Vars',s);

% step 1 : Draw Root Locus and Locate Dominant Poles when 𝐾_𝑅𝐿=1
figure(1)
pzmap(CLTF_init)

Denominator_CLTF = CLTF_init.Denominator{1,1};

roots_CLTF = roots(Denominator_CLTF)

%%
s1 = roots_CLTF(2) % TUNE choose number index of dominant poles

sig_CL = abs(real(s1)); % assign constant parameters
wd =  imag(s1);
wn = sqrt(sig_CL^2 + wd^2);
zeta = sig_CL/wn;

%%
% step 2 : Calculate appropriate static error constant

% (TUNE) Tracking step input -> Kp, ramp ;> Kv, parabolic -> Ka
% Kp = G_func(0) % s  = 0
Kv = sG_func(0);
% Ka = ssG_func(0)

%steady state error, choose base on type 0,1 or 2
% ess = 1/(1+Kp) %type 0
ess = 1/Kv ;% type 1
% ess = 1/Ka %type 2

% step3 : Determine desired increase in error constant
% Treat this as a value for 𝛽
beta = 10;  %choose beta TUNE

% step 4: Select a pole/zero pair 
% 𝜎_𝐶𝐿 and 𝜔_𝑑 come from the dominant poles in Step 1
% you pick a 𝜙_(𝑑_𝑙𝑎𝑔 ) value in the range: 0<𝜙_(𝑑_𝑙𝑎𝑔 )<5°
phi_dLag = 2.5; % [degree] TUNE

sig_z = sig_CL - wd * tand((atand(sig_CL/wd)) - 2 * phi_dLag);

sig_p = sig_z / beta;

T = 1/sig_z;

%%
% Step 5 : Redraw the root locus with the new pole/zero pair added (𝐾_𝑐=1)
% new controlled system
GcG_Kc_eq_1 = simplify((s+sig_z)/(s + sig_p)) * G;
[num, den] = numden(GcG_Kc_eq_1);

OLTF_new = tf(sym2poly(num), sym2poly(den));


% Step 6 : Adjust 𝐾_𝑐 from magnitude condition (keep 𝜁 zeta the same as Step 1)
rltool(OLTF_new)

%%
Kc = 0.91134; %TUNE

%finding new constant errors depending on each type, similarly for type 2
GcG_with_Kc = GcG_Kc_eq_1 * Kc;
GcG_with_Kc_func = matlabFunction(GcG_with_Kc,'Vars',s); %type 0
sGcG_with_Kc_func = matlabFunction(simplify(s*GcG_with_Kc),'Vars',s); %type 1

%new Kp,v or a
Kv_new = sGcG_with_Kc_func(0); %type 1

% Step 7 : Close loop and verify performance
Gc = tf(Kc *[1 sig_z],[1 sig_p]);

CLTF_final = feedback(Gc*G_tf,H_tf)

%%
% check the step or any kind of response
figure();
% step(CLTF_final)
step(CLTF_init,'ro',CLTF_final,"b+")
legend ('Baseline','Compensated')
title('Step responses of the baseline and compensated systems')
% print -dpng img/hw5_part6a

S_baseline = stepinfo(CLTF_init);
S_compensated = stepinfo(CLTF_final);

%ramp response
t = 0:0.1:18;
u = t;
figure();
[y_baseline,t,x] = lsim(CLTF_init,u,t);
[y_compensated,t,x] = lsim(CLTF_final,u,t);
plot(t, y_baseline,'ro',t,y_compensated,'b+');
legend ('Baseline','Compensated')
xlabel('Time (sec)')
ylabel('Amplitude')
title('Ramp responses of the baseline and compensated systems')
% print -dpng img/hw5_part8a

%steady state error for ramp
fprintf("steady state error for baseline system ess_init =  \n")
disp(ess);
fprintf("steady state error for baseline system ess_final =  \n")
disp(1/Kv_new);


























