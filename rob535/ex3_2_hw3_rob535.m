clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw3/func')

%% Load 3D point clouds in the body frame
velodyne = load('velodyne_turn.mat');
pc_body = velodyne.data;
num_clouds = size(pc_body, 1);
%initialize
T_init = eye(4);
T_init(1, 4) = 1;
%Point Clouds
idx = randi([1,num_clouds-1]);
sprintf('Check ICP with point clouds %d and %d ', [idx,idx+1])
pc1 = pc_body{idx};
pc2 = pc_body{idx+1};
% Learner Solution
% T_final=icp(pc1,pc2,1,T_init);
T_final=icp(pc1,pc2,0,T_init);

% %Reference Solution
% T_ref=reference.icp(pc1,pc2,1,T_init);
% % Assesment
% assessVariableEqual('T_final', T_ref);

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