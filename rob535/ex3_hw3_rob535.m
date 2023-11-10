clc; clear ; close all; format default;

% [p1, p2, R, t] = gen_points(1000, 0.01);
% disp([R,t]);
% % ground truth
% [R1, t1] = rigid_fit(p1, p2);
% disp([R1, t1]); %should be close to the ground truth
% [T_rel, T_total] = kitti_icp('velodyne_turn_small.mat')

[T_rel, T_total] = kitti_icp();
print -dpng kitti_ICP

%% 3.1 Fit Rigid Body Transformations

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

%%  3.2 Naive and Weighted ICP

function T_final = icp(p_t, p_s, use_naive, T_init)

m = size(p_t, 1);

if nargin < 4
    T_init = eye(m + 1);
    if nargin < 3
        use_naive = true;
    end
end

delta = inf;
T_final = T_init;
while delta > sqrt(eps)
    p1 = T_final(1:m, 1:m) * p_s + T_final(1:m, m + 1);
    % part 1.2
    % Write your code here:
    % p2 = ?;

%     nearest neighbor in X for each query point in Y 
% find the nearest point in p t for each point in p 1
% gives index and correspondences distances between corresponding points
    [pt_Idx,D_ptNear_p1] = knnsearch(p_t.',p1.');
    p2 = zeros(m,length(pt_Idx));
    for i = 1:length(pt_Idx)
        idx = pt_Idx(i);
        p2(:,i) = p_t(:,idx); 
    end

    if use_naive
        weight = ones(size(p1, 2), 1);
    else
        % part 1.3 
        % Write your code here:
        sigma_d = std(D_ptNear_p1);
        for i = 1:size(p1, 2)
            weight(i) = exp(-D_ptNear_p1(i)^2*(1/sigma_d^2));
        end

    end

    % incremental transformation
    [delta_R, delta_t] = rigid_fit(p1, p2, weight);

    % Write your code here:
    % update T_final with `delta_R` and `delta_t`
    T_final = T_final * [delta_R delta_t;zeros(1,size(delta_R,2)) 1];

    delta = norm([delta_R - eye(m), delta_t]);
end

end


%% 3.3 Odometry

function [T_rel, T_total] = kitti_icp(file)
% Load 3D point clouds in the body frame
if nargin == 0
   file = 'velodyne_turn.mat';
end
velodyne = load(file);
pc_body = velodyne.data;
num_clouds = length(pc_body);

% Relative motions
T_rel = repmat(eye(4), [1, 1, num_clouds]);

% initial guess of each relative motion
T_init = eye(4);
T_init(1, 4) = 1.5;

% Write your code here:
for k = 2:num_clouds
    T_rel(:, :, k) = icp(pc_body{k-1}, pc_body{k}, 0, T_init);
end

% Total motion
T_total = repmat(T_rel(:, :, 1), [1, 1, num_clouds]);

% Write your code here:
for k = 2:num_clouds
  T_total(:, :, k) =  T_total(:,:,k-1) * T_rel(:,:,k);
end

disp(T_total)

% Visulization
figure(1)
clf()

pc_world = cell(num_clouds, 1);

for k = 1:num_clouds
    R = T_total(1:3, 1:3, k);
    t = T_total(1:3, 4, k);
    pc_world{k} = R * pc_body{k} + t;
end

% uncomment follow code block on your local machine for visualization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%% VISUALIZATION START %%%%%%%%%%%%%%%%%%%%%
pc_world = cell2mat(pc_world');
pcshow(pc_world')
caxis(prctile(pc_world(3, :), [1, 99]))
grid on
axis equal

zlim([-10, 10])
view(2)

traj_xyz = squeeze(T_total(1:3, 4, :));
hold on
plot3(traj_xyz(1, :), traj_xyz(2, :), traj_xyz(3, :), ...
   'r.-', 'MarkerSize', 8)
hold off
%%%%%%%%%%%%% VISUALIZATION END %%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end