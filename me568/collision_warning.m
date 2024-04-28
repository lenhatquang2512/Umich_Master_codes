function [col_flag, TTC] = collision_warning(t, thetaR, thetaD, angle_threshold)
    global la;
    % la is the half length of vehicle, or the radius of the circle
    % t is the current time in simulation
    % thetaR is the estimated variables for R, it is 3x1 variables. 
    % Assume thetaR is [a;b;c], then R_est = a * t^2 + b * t + c 
    % thetaD is the estimated variables for D, it is 3x1 variables. 
    % D_est can be calculated similarly
    % angle_threshold is the critical angle that you have calculated 
    % in Q1(a)

    %% TODO: 
%     col_flag = false; % please modify col_flag, determine if 
%     % two vehicle will collide or not. true means collide, vice versa
%     TTC = Inf; % please modify TTC. TTC means Time to Collision
    
    global psi_his
    global threshold_his
    
    % Compute R , V, A
    aR = thetaR(1); bR = thetaR(2); cR = thetaR(3);
    R_est = aR * t^2 + bR * t + cR - 2* la;
    V_est = 2 * aR * t + bR; % R dot 
    A_est = 2 * aR; 

    % Compute D and omega
    aD = thetaD(1); bD = thetaD(2); cD = thetaD(3);
    D_est = aD * t^2 + bD * t + cD;
    omega_est = 2 * aD * t + bD;

    % Compute absolute value angle between Vrel and -R
%     psi_est = atan2(omega_est*(R_est + 2 * la),V_est) + D_est;
     psi_est = atan((omega_est*(R_est + 2 * la))/V_est);
%     fprintf("psi_est = %f , angle_threshold = %f \n", psi_est, angle_threshold);

     psi_his = [psi_his;abs(psi_est)];
     threshold_his = [threshold_his;abs(angle_threshold)];
    
    if( R_est >= 0)
%     if (psi_est <= angle_threshold)
    if (abs(psi_est) <= angle_threshold)
%         fprintf("t = %f Crash \n",t);
        col_flag = true; % raise flag collision
        if A_est == 0
            TTC = R_est/V_est;
            fprintf("t = %f, R_est =  %f, TTC = %f, A_est = 0 \n",t, R_est, TTC);
        else 
%             (V_est^2 > 2 * A_est * R_est )
            TTC1 = (-V_est + sqrt(V_est^2 - 2 * A_est * R_est))/A_est;
            TTC2 = (-V_est - sqrt(V_est^2 - 2 * A_est * R_est))/A_est;

            if TTC1 < 0 
                fprintf("t = %f, R_est =  %f, TTC2 = %f,TTC1 = %f Negative TTC1 \n",t, R_est, TTC2, TTC1);
                TTC = TTC2;
            elseif TTC2 < 0
                fprintf("Negative TTC2 \n");
                TTC = TTC1;
            end

            if TTC1 > 0 && TTC2 >0
                if TTC1 > TTC2 
                    fprintf("t = %f, R_est =  %f, TTC2 = %f TTC1 = %f, Both positive and TTC1 > TTC2 \n",t, R_est, TTC2, TTC1);
                    TTC = TTC2;
                else
                    fprintf("Both positive and TTC1 < TTC2 \n");
                    TTC = TTC1;
                end
            end

            if TTC < 0
                TTC = 0;
                fprintf("Negative time \n");
            end
        
        end

    else
        fprintf("t =  %f R_est =  %f Not Crash \n",t, R_est);
        col_flag = false;
        TTC = Inf;
    end

    else
       fprintf("t = %f R_est =  %f Crash \n",t, R_est);
       col_flag = true;
       TTC = 0;
    end

end

