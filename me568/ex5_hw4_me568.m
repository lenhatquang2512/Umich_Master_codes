%% Problem 5 - Vehicle Stability Control
clc; clear; close all; format default;

% The Magic Formula for Fz = 3225N
By = 0.27; Cy = 1.2; Dy = 2921;
Ey = -1.6; Shy = 0; Svy = 0;
% Vehicle parameters
a = 1.14; L = 2.54;
b = L - a;
m = 1500; Iz = 2420.0; g = 9.81;
Fz_f = m*g*b/L; Fz_r = m*g*a/L;
Dy_f = Dy*Fz_f/3225; Dy_r = Dy*Fz_r/3225;

for i=1:101
    alpha(i) = (i-1)*0.14;
    % slip angle 0 to 14 deg
    phi_y = (1-Ey)*(alpha(i)+Shy) + Ey/By*atan(By*(alpha(i)+Shy));
    fy_f(i) = Dy_f*sin(Cy*atan(By*phi_y))+Svy;
    fy_r(i) = Dy_r*sin(Cy*atan(By*phi_y))+Svy;
end

% figure(1)
figure()
subplot(311)
plot(alpha, fy_f, '-r', alpha, fy_r,'-.b')
xlabel('Tire slip angle (deg)'); ylabel('Fy (N)')
legend('Front axle', 'Rear axle'); 

% Part II, calculate Yaw moment & lateral force
r_over_u = 0;
% yaw rate = 0
for i = 1:10
    delta = i-5;
    % steering angle -4 to 5 deg
    for j = 1:9
        beta(j) = j-1;
        % side slip angle 0 to 8 deg
        alpha_f = delta - (beta(j) + 180/pi*a*r_over_u);
        alpha_r = - (beta(j) - 180/pi*b*r_over_u);
        phi_yf = (1-Ey)*(alpha_f+Shy) + Ey/By *atan(By*(alpha_f+Shy));
        Fyf = Dy_f*sin(Cy*atan(By*phi_yf))+Svy;
        phi_yr = (1-Ey)*(alpha_r+Shy) + Ey/By *atan(By*(alpha_r+Shy));
        Fyr = Dy_r*sin(Cy*atan(By*phi_yr))+Svy;
        yaw_moment(j, i) = a * Fyf - b * Fyr;
        lateral_force(j,i) = Fyf + Fyr;
    end
end

% figure(2)
subplot(312)
plot(beta, yaw_moment)
xlabel('Vehicle side Slip angle (deg)')
ylabel(' Yaw moment (N-m)');
% figure(3)
subplot(313)
plot(beta, lateral_force)
xlabel('Vehicle side Slip angle (deg)')
ylabel('Total lat. force(N)')

% print p5_img/ex5_hw4_me568_ex_14_1_side_slip_angle_control_authorities -dpng;








