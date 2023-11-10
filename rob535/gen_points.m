function [p1, p2, R, t] = gen_points(num_points, sigma)
if nargin < 3
    % std of noise
    sigma = 0;
end

%% generate a random transformation
A = randn(3, 3);
[R, ~] = eig(A * A');

if det(R) < 0
    R(:, 1) = -R(:, 1);
end

t = randn(3, 1);

%% generate random points
x1 = randn(1, num_points);
p1 = [x1; log1p(x1.^2); sin(x1)];

p2 = R * p1 + t;

if sigma
    p2 = p2 + sigma * randn(3, num_points);
end

end
