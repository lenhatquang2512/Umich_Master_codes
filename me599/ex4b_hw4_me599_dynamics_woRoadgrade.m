function dXdt = ex4b_hw4_me599_dynamics_woRoadgrade(X, u)
    x = X(1);
    v = X(2);
    
    m = 1000;
    rho = 1.2;
    Cd = 0.4;
    Aref = 5;
    Crr = 0.01;
    g = 9.8;

    dXdt = zeros(2,1);
   
    dXdt(1) = v;
    dXdt(2) = (1/m)*(u - (0.5*rho*Cd*Aref*v^2) - Crr*m*g); 
end