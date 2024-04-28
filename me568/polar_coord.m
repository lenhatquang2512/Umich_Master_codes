function [R,D] = polar_coord(states_p, states_e)
    pos_p = states_p(1:2); % x and y
    pos_e = states_e(1:2);
    vec = pos_p - pos_e;

    R = sqrt( sum((vec).^2 ));
    D = atan2(vec(2),vec(1));
end

