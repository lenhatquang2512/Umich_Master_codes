clc;clear;close all; format default;

%spring-mass-damper system parameters
m = 1;
k = 1;
b = 0.5;
% F = 1;

%state space model
A = [0 1; -k/m -b/m];
B = [0 1/m]';
C = [1 0];
D = 0;

sys = ss(A,B,C,D);
% transfer
H_tf = tf(sys);

%convert tf type to sym type
syms s
Numerator = poly2sym(H_tf.Numerator{1,1},s);
Denominator = poly2sym(H_tf.Denominator{1,1},s);
Hsym = Numerator/Denominator;

% convert to matlab function
H_func = matlabFunction(Hsym,'Vars',s);

%check stability
[V,D_eig] = eig(A);
evalues = diag(D_eig)
s1 = evalues(1);
s2 = evalues(2);

%DC gain
DC_gain = H_func(0);

%damping quality
sig_CL = abs(real(s1)); % assign constant parameters
wd =  imag(s1); % characteristic oscillation frequency
wn = sqrt(sig_CL^2 + wd^2);
zeta = sig_CL/wn;
disp('underdamped');

%time constant (reciprocal)
Time_constant = 1/sig_CL

%check controllable
% Kalman rank test
Co_hand = [B A*B];
Co = ctrb(A,B);
% disp(Co);
unco = length(A) - rank(Co)
disp('fully controllable');

%check observale
% Observable test
Ob_hand = [C;C*A];
Ob = obsv(A,C);
unobsv = length(A) - rank(Ob)
disp('fully observable');



% --------Forward Euler approximation------------------
T = 0.05;   % time step

Ad = eye(2) + A * T;
Bd = B * T;
Cd = C;
Dd = D;

sys_euler = ss(Ad,Bd,Cd,Dd)

%state transition matrix (STM)
syms t
STM = simplify(expm(A*t))
pretty(STM)

% zero order approximation
Ad_zo = expm(A*T)
Bd_zo = integral(@(h) expm(A*h),0,T,'ArrayValued',true) * B
Cd_zo = C;
Dd_zo = D;

%use c2d to verify

sys_zo = c2d(sys,T,'zoh')
% [X,Y,Z,W] = ssdata(sys_zo) % get the discrete matrices back

%assess
%the stability, time constant(s) and characteristic oscillation frequency
fprintf('for Euler :');
eig(Ad)
 abs(0.9875 + 0.0484i)  % check stability
-T/(log(abs(0.9875 + 0.0484i))) % time constant for continuous
angle(0.9875 + 0.0484i)/T
new_e_values = 1 + s1 * T

if real(new_e_values) > 0 
    fprintf("Unstable \n");
else
    fprintf("Stable \n");
end

new_time_constant = 1 / abs(real(new_e_values))
new_char_osc_freq = imag(new_e_values) %% ??

% fprintf(" These results do not agree with what I obtained in part (b) \n");

fprintf('for ZOH :');
eig(Ad_zo)
abs(0.9864 + 0.0478i)
-T/(log(abs(0.9864 + 0.0478i)))
angle(0.9864 + 0.0478i)/T

 new_time_constant_zoh2cts = -T/(log(abs(0.9864 + 0.0478i)))



