%% kinematic bicycle model with (x,y) located in the center of gravity
function dstates = kinematics(states, ctrls)
    global la;
    lb = la;
    lb = 2.2;
    L = la+lb;
    
    x = states(1);
    y = states(2);
    psi = states(3);
    ux = states(4);
    ax = ctrls(1);
    sa = ctrls(2);

    beta = atan(lb/L*tan(sa));
    dx = ux*cos(psi + beta);
    dy = ux*sin(psi + beta);
    dpsi = ux*tan(sa)*cos(beta)/L;
    dux = ax;

    if ux <= 0 && dux <0 
        dux = 0;
    end

    dstates = [dx;dy;dpsi;dux];
end

