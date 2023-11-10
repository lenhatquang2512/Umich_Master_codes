clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

load('variables.mat')

%% show 3D points (x = camera right, y = camera down, z = camera forward)

%%%% Uncomment following block on your local Matlab to visualize%%%%
%%%% Visualization Start %%%%
figure(2)
set(gcf, 'position', [200, 200, 600, 800])
pcshow(X0.', 'r'); hold on
pcshow(X1.', 'b'); hold off
axis([-25, 25, -10, 10, 0, 50])
% print -dpng ex3_4_RANSAC_Visual_Odom
%%% Visualization End %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

%% RANSAC
[T, idx_inliers] = visual_odom_ransac(...
    X0(:, idx_match(:, 1)), X1(:, idx_match(:, 2)), ...
    x0_left(:, idx_match(:, 1)), x1_left(:, idx_match(:, 2)), ...
    P_left,0);

fprintf('Done.\nT =\n')
disp(T)

disp(num2str(mean(idx_inliers) * 100, 'Percentage of inliers = %.2f'))

%% for debug only
% X0 = X0(:, idx_match(:, 1));
% X1 = X1(:, idx_match(:, 2));
% x0 = x0_left(:, idx_match(:, 1));
% x1 = x1_left(:, idx_match(:, 2));
% seed =0;

%%
function [T, idx_inliers] = visual_odom_ransac(X0, X1, x0, x1, P_left, seed)
% Visual Odometry with RANSAC:
% estimate the relative motion (R, t) between two frames (i.e. t0 and t1)

%assert(isequal(size(X0), size(X1)))
%assert(isequal(size(x0), size(x1)))
%assert(isequal(size(X0, 2), size(x0, 2)))

if nargin == 6
    rng(seed)
end

% hyperparameters for RANSAC
% smallest number of points required to fit the model
s = 3;
% Number of iterations required to find transformation matrix using RANSAC
num_iter = 500;
% Threshold of the reprojection error
d_thresh = 12.5;

%
num_points = size(X0, 2);
X0 = [X0; ones(1, num_points)];
X1 = [X1; ones(1, num_points)];

% reprojection error of all points at each iteration
d = nan(num_iter, num_points);

for k = 1:num_iter
    % draw samples from the two clouds
    idx_sample = randsample(num_points, s);
    X0_sample = X0(:, idx_sample);
    X1_sample = X1(:, idx_sample);

    % Performing Rigid Fit
    [R, t] = rigid_fit(X1_sample(1:3,:), X0_sample(1:3,:));
    T = [R t;zeros(1,size(R,2)) 1];

    % Transform point clouds X0 and X1 to frame 1 and 0 respectively
    X0_in_1 = T\ X0;
    X1_in_0 = T * X1;

    % Projecting X1_in_0 and X0_in_1 to the respective 2D image planes
    x0_hat = P_left * X1_in_0;
    x1_hat = P_left * X0_in_1;
    x0_hat = x0_hat(1:2,:)./x0_hat(3,:); % convert from homogeneous to Cartesian
    x1_hat = x1_hat(1:2,:)./x1_hat(3,:);

    % Calculating reprojection error d(:, k)
    d(k, :) = vecnorm(x0 - x0_hat,2,1).^2 + vecnorm(x1 - x1_hat,2,1).^2; %vecnorm because I need a vector of distances error of all points 

end

% obtain indices of inliers corresponding to the iteration with the most
% number of inliers
count_inliers = sum(d < d_thresh,2); % I want a count vector to use max function
[~,idx_iter_best] = max(count_inliers); % which iteration has most inliers 
idx_inliers = d(idx_iter_best,:) < d_thresh; % where all inliers in "that best" iteration

% Fit using all inliers
[R, t] = rigid_fit(X1(1:3,idx_inliers), X0(1:3,idx_inliers)); %perform rigid fit but with inliers only
T = [R t;zeros(1,size(R,2)) 1];

end

%%
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

% Reference solution:
mu1 = p1 * weight;
mu2 = p2 * weight;

[U, ~, V] = svd((p1 - mu1) .* weight' * (p2 - mu2)');

D = eye(size(p1, 1));
if det(U) * det(V) < 0
    D(end, end) = -D(end, end);
end

R = V * D * U';
t = mu2 - R * mu1;
end