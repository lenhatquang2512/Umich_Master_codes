function dXdt = ex4b_hw4_me599_dynamics_withRoadgrade(X, u)
    x = X(1);
    v = X(2);
    
    m = 1000;
    rho = 1.2;
    Cd = 0.4;
    Aref = 5;
    Crr = 0.01;
    g = 9.8;

    dXdt = zeros(2,1);

    if 0 <= x && x <= 400
        theta = 0;   % deg
    elseif 400 < x && x <= 800
        theta = 5;
    elseif 800 < x && x <= 1200
        theta =  -5;
    elseif x > 1200
        theta = 0;
    end

    dXdt(1) = v;  % x_dot
    dXdt(2) = (1/m)*(u - (0.5*rho*Cd*Aref*v^2) - Crr*m*g*cos(deg2rad(theta)) - m*g * sin(deg2rad(theta)));  %v_dot

end