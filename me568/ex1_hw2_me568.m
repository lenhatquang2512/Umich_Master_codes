%% Problem 1d Verification of magnitude of zs
clc; clear; close all; format default;

syms s t

G = (1000 * s + 50000) * (0.8 * pi) / ((500 * s^2 + 1000 * s + 50000) * (s^2 + 64 * pi ^2));

f = ilaplace(G);

% steady_state_value = limit(f, t, inf)

% pretty(simplify(f))

% convert
f_func = matlabFunction(f,'Vars',t);
f_func(4.65)

t = -0:0.00001:10;

for i = 1:length(t)
    f_plot(i) = f_func(t(i));
end

figure(1);
plot(t,f_plot,'r');grid;

print ex1d_hw2_me568 -dpng;

% 
% syms s
% 
% G = (1000 * s + 50000) * (0.8 * pi) / ((500 * s^2 + 1000 * s + 50000) * (s^2 + 64 * pi ^2));
% 
% steady_state_value = limit(s* G, s, 0)