clc; clear ; close all; format default;

[p1, p2, R, t] = gen_points(1000, 0.01);
disp([R,t]); % ground truth
[R1, t1] = rigid_fit(p1, p2);
disp([R1, t1]); %should be close to the ground truth

% 3.1 Fit Rigid Body Transformations
function [R, t] = rigid_fit(p1, p2, weight)
% Fit a rigid body transformation [R, t] by solving
%
%       \min    sum(w(i)* norm(R * p1(:, i) + t - p2(:, i) )^2)
%
%   Reference: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf

%assert(isequal(size(p1), size(p2)))
%assert(size(p1, 1) <= size(p1, 2))

if nargin < 3
    weight = ones(size(p1, 2), 1);
end

%assert(all(weight >= 0))

% reshape and normalize
weight = reshape(weight, [], 1);
weight = weight / sum(weight);

% Write your code here:

%p1 is  m xn (m=3 usually for point cloud)
%p2 us also mxn
% w is 1xn
%Compute the weighted centroids of both point sets
n = size(p1,2);
m = size(p1,1);
wp1 = zeros(m,1); 
wp2 = zeros(m,1);
for i =  1:n
    wp1 = wp1 + weight(i) * p1(:,i);
    wp2 = wp2 + weight(i) * p2(:,i);
end

p1_bar = wp1/sum(weight);
p2_bar = wp2/sum(weight);

% Compute the centered vectors
X = zeros(m,n); Y = zeros(m,n);
for i =  1:n
    X(:,i) = p1(:,i) - p1_bar;
    Y(:,i) = p2(:,i) - p2_bar;
end

%Compute the d × d covariance matrix
S = X * diag(weight) * Y.';

% Compute the singular value decomposition , U is mxm, V is nxn
[U,~,V] = svd(S,"econ");

% rotation we are looking for is then
I_mod = eye(size(U.',1));
I_mod(size(U.',1),size(U.',1)) = det(V* U.');

R = V * I_mod * U.';

% Compute the optimal translation as
t = p2_bar - R * p1_bar;

end
