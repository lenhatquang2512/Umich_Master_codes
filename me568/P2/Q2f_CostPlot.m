close all;
clear all;
clc;
w_y = [1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1,10];

% TODO value of cost function
cost = [93.24, 65.966, 44.373,35.183,33.798,33.495, 33.135]; 


semilogx(w_y, cost)
set(gca, "Fontsize", 14)
xlabel("w_y")
ylabel("Cost")

print img/plot2f -dpng;