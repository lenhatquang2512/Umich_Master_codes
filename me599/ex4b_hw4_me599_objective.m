function J = ex4b_hw4_me599_objective(u,x,v,N,T_MPC)

    x_vec = x*ones(N,1);
    v_vec = v * ones(N,1);
%     f = @ex4b_hw4_me599_dynamics_withRoadgrade;
    f = @ex4b_hw4_me599_dynamics_woRoadgrade;

    v_final_des = 25;       % m/s (desired final velocity)

    state = [x_vec(1) v_vec(1)]';

    %   Step through the horizon to calculate the objective function value
    J = 0;
    for i=2:N
        K1 = f(state,u(i-1));
        K2 = f(state+K1*T_MPC/2,u(i-1));
        K3 = f(state+K2*T_MPC/2,u(i-1));
        K4 = f(state+K3*T_MPC,u(i-1));
        K_weighted = 1/6*(K1 + 2*K2 + 2*K3 + K4);
    
        x_vec(i) =  x_vec(i-1) + K_weighted(1)*T_MPC;
        v_vec(i) =  v_vec(i-1) + K_weighted(2)*T_MPC;
        state = [x_vec(i) v_vec(i)]';
    
        J = J + (v_vec(i) - v_final_des)^2  * T_MPC;
    end
end
