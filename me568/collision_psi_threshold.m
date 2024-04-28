%% Problem 1a : Given the size of vehicle and each states, what are the critical values for collision psi
function psi_c = collision_psi_threshold(states_p, states_e)
    global la;
    % la is the half length of vehicle, or the radius of the circle
    % states_p is the states of the preceding vehicle. It is a 4x1 vectors expressed as [x;y;psi;ux]. 
    % x is the x position of the center of the mass of vehicle, y is the y position, psi is the yaw angle
    % and ux is the speed
    % states_e is the states of the ego vehicle.
    %% TODO:
    [R,D] = polar_coord(states_p, states_e);
    psi_c = asin(2 * la / R); 
end