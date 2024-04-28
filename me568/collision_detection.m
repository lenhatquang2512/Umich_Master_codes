%% collision detection
function flag = collision_detection(states_p, states_e, la)
    dist = sqrt( sum((states_p(1:2) - states_e(1:2)).^2 ) );
    if dist <= 2*la
        flag = true;
    else
        flag = false;
    end
end
