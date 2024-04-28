%% Problem 1b: Implement rls based on lecture slides
function [theta, P] = rls(theta,P, phi, z, lambda)
    % z is the observation (1x1 variable),  it can be either R or D
    % theta is the parameters you want to estimate (3x1 variable)
    % phi is the time variables expressed as [t^2; t; 1] (3x1 variable)
    % P is (3x3 matrix)
    % lamda is the forgetting factor (1x1)
    %% TODO:
    K = P * phi* inv(lambda+ (phi.') * P * phi );
    theta = theta + K * (z -  (phi.')* theta); %theta: the estimation of parameters,which is a 3x1 vector
    P = ((eye(3) - K * (phi.'))* P)/ lambda; % P: it should be 3x3 matrix
end
