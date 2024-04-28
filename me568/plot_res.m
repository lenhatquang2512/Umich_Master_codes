%% Plotting function
function plot_res(step_idx, t_his, states_p_his, states_e_his, R_his, D_his, thetaR_his, thetaD_his, col_flag, TTC)
        global la;
        global dt_sim;
        figure(1)
        %% plots for R evaluation
        subplot(2,3,2)
        dR_his = (R_his(2:step_idx) - R_his(1:step_idx-1))./dt_sim;
        ddR_his = (dR_his(2:step_idx-1) - dR_his(1:step_idx-2))./dt_sim;
        hold off
        plot(t_his(1:step_idx-1), dR_his, 'b--')
        hold on
        plot(t_his(1:step_idx-1), 2*(thetaR_his(1,1:step_idx-1).*t_his(1:step_idx-1)) + thetaR_his(2,1:step_idx-1), 'r--')
        xlabel('$t [s]$','Interpreter','latex')
        ylabel('$\dot{R}$ $[m/s]$','Interpreter','latex')
        legend({'Actual','Estimated'},'Location','northoutside','Orientation','horizontal')
        title('RLS for $\dot{R}$','Interpreter','latex')

        subplot(2,3,3)
        hold off
        plot(t_his(1:step_idx-2), ddR_his, 'b--')
        hold on
        plot(t_his(1:step_idx-2), 2*(thetaR_his(1,1:step_idx-2)), 'r--')
        xlabel('$t [s]$','Interpreter','latex')
        ylabel('$\ddot{R}$ $[m/s^2]$','Interpreter','latex')
        legend({'Actual','Estimated'},'Location','northoutside','Orientation','horizontal')
        title('RLS for $\ddot{R}$','Interpreter','latex')

        %% plots for D evaluation
        subplot(2,3,5)
        dD_his = (D_his(2:step_idx) - D_his(1:step_idx-1))./dt_sim;
        ddD_his = (dD_his(2:step_idx-1) - dD_his(1:step_idx-2))./dt_sim;
        hold off
        plot(t_his(1:step_idx-1), dD_his, 'b--')
        hold on
        plot(t_his(1:step_idx-1), 2*(thetaD_his(1,1:step_idx-1).*t_his(1:step_idx-1)) + thetaD_his(2,1:step_idx-1), 'r--')
        xlabel('$t [s]$','Interpreter','latex')
        ylabel('$\dot{D}$ $[rad/s]$','Interpreter','latex')
        legend({'Actual','Estimated'},'Location','northoutside','Orientation','horizontal')
        title('RLS for $\dot{D}$','Interpreter','latex')

        subplot(2,3,6)
        hold off
        plot(t_his(1:step_idx-2), ddD_his, 'b--')
        hold on
        plot(t_his(1:step_idx-2), 2*(thetaD_his(1,1:step_idx-2)), 'r--')
        xlabel('$t [s]$','Interpreter','latex')
        ylabel('$\ddot{D}$ $[rad/s]$','Interpreter','latex')
        legend({'Actual','Estimated'},'Location','northoutside','Orientation','horizontal')
        title('RLS for $\ddot{D}$','Interpreter','latex')

        set(gcf, 'Renderer', 'painters', 'Position', [0 0 1080 900])
        set(findall(gcf,'-property','FontSize'),'FontSize',15)

        %% Simulation plot
        subplot(2,3,1)
        hold off
        plot(states_p_his(1,1:step_idx), states_p_his(2,1:step_idx), 'b--')
        hold on
        plot(states_e_his(1,1:step_idx), states_e_his(2,1:step_idx), 'r--')
        
        [x_p_shape, y_p_shape] = circle(states_p_his(1,step_idx),states_p_his(2,step_idx),la);
        [x_e_shape, y_e_shape] = circle(states_e_his(1,step_idx),states_e_his(2,step_idx),la);
        
        fill(x_p_shape, y_p_shape,'b','FaceAlpha',0.3)
        scatter(states_p_his(1,step_idx),states_p_his(2,step_idx), 'b.')
        fill(x_e_shape, y_e_shape,'r','FaceAlpha',0.3)
        scatter(states_e_his(1,step_idx),states_e_his(2,step_idx), 'r.')
        xlabel('x [m]')
        ylabel('y [m]')
        axis([-10 10 -2 50])
        axis equal
        title(['Est Collide: ', num2str(col_flag),' TTC: ',num2str(TTC), ' s; ','Time: ',num2str(t_his(step_idx)), ' s; '], 'FontSize',10)




        %% Simulation in the polar coordinates
        subplot(2,3,4)
        hold off

        R_list = zeros(1, step_idx);
        D_list = zeros(1, step_idx);
        
        for i = 1:1:step_idx
            [R_list(i),D_list(i)] = polar_coord(states_p_his(:,i), states_e_his(:,i));
        end
       
        polarplot(D_list,R_list,'b-o')
        hold on
        BoundaryD = 0:0.1:2*pi;
        BoundaryR = ones(size(BoundaryD))*2*la;
        polarplot(BoundaryD,BoundaryR,'r--')
        title(['Polar Coordinates'], 'FontSize',10)


        
end

