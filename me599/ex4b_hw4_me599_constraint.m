function [c_ineq,c_eq] = ex4b_hw4_me599_constraint(u,x,v,N,T_MPC)

    %   Parameters
    %     m = 1000;       % kg
    %     g = 9.8;        % m/s^2
    %     Crr = 0.01;     % dimensionless
    %     rho = 1.2;      % kg/m^3
    %     Cd = .4;       % dimensionless
    %     A_ref = 5;      % m^2
        
%     N = 300;         % horizon length
%     delta_T = 0.2;    % time step
%     v_final_des = 25;       % m/s (desired final velocity)
    
%         f = @ex4b_hw4_me599_dynamics_withRoadgrade;
        f = @ex4b_hw4_me599_dynamics_woRoadgrade;
        v_vec = v * ones(N,1);
        x_vec = x * ones(N,1);
%         v(1) = 25;     %   Initial velocity
%         x(1) = 0;     %   Initial position
        state = [x_vec(1) v_vec(1)]';
    
        c_eq = [];
        c_ineq = zeros(2 *N-2,1);
        
        %   Step through the horizon to calculate the final position
        for i=2:N
            K1 = f(state,u(i-1));
            K2 = f(state+K1*T_MPC/2,u(i-1));
            K3 = f(state+K2*T_MPC/2,u(i-1));
            K4 = f(state+K3*T_MPC,u(i-1));
            K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);
        
            x_vec(i) =  x_vec(i-1) + K_weighted(1)*T_MPC;
            v_vec(i) =  v_vec(i-1) + K_weighted(2)*T_MPC;
            state = [x_vec(i) v_vec(i)]';
            
           c_ineq(2*i-3) = -v_vec(i) + 20;
           c_ineq(2*i-2) = v_vec(i) - 30;
        end
      

end
