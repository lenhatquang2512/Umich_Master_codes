clc; clear ; close all; format default;

%%
n = randn(3, 1);

% Run learner solution.
T = mtx_rotate(n)

a = [1 1 1].';
s = [1 2 4].';
T_s = mtx_scale(s, a)

%% P1.1 Homography Matrix - Translate

function T_t = mtx_translate(t)
    n = length(t);
    T_t = [eye(n) t ; zeros(1,n) 1];

end

%% 1.2 Homography Matrix - Rotate

function T_r = mtx_rotate(n, a)

    %  if anchor point not given use origin
    if nargin < 2
        a = [0 0 0].';
    end
    
    % Convert rotation vector to rotation matrix
    theta = norm(n,2); %Euclidean norm i.e norm-2
    
    %roll,pitch,yaw
%     gamma = n(1);
%     beta = n(2);
%     alpha = n(3);
    
    %Rotation matrices
%     Rx = [1 0 0;0 cos(gamma) -sin(gamma);0 sin(gamma) cos(gamma)];
%     Ry = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)];
%     Rz = [cos(alpha) -sin(alpha) 0;sin(alpha) cos(alpha) 0; 0 0 1];
   

    %Rodrigues rotation formula
    if theta > 0
        
        n = n/theta;
        W = [0 -n(3) n(2); n(3) 0 -n(1); -n(2) n(1) 0];
        M = [n(1)^2 n(1)*n(2) n(1)*n(3) ;n(2)*n(1) n(2)^2 n(2)*n(3);n(3)*n(1) n(3)*n(2) n(3)^2];
    %     R_rod = eye(3) + sin(theta)*W + 2* sin(theta/2)^2 * W;
        R_rod = eye(3) * cos(theta) + (1-cos(theta)) * M + sin(theta) * W;
    else
        R_rod = eye(3);
    end
   
    
%     combine 3 rotations in homogeneous coordinate
%     R = [Rx*Ry*Rz zeros(3,1);zeros(1,3) 1];
    R = [R_rod zeros(3,1);zeros(1,3) 1];
    
%   translate a to get correct rotation axis
    T_r = mtx_translate(a) * R * mtx_translate(-a);

end

%% 1.3 Homography Matrix - Scale
function T_s = mtx_scale(s, a)
    %  if anchor point not given use origin
    if nargin < 2
        a = [0 0 0].';
    end
    
%     t = [a(1) *(1-s(1)) ; a(2)*(1-s(2)); a(3) * (1-s(3))];
%     T_s = [diag(s) t; zeros(1,3) 1];
      
      S = [diag(s) zeros(3,1); zeros(1,3) 1];
      T_s = mtx_translate(a) * S * mtx_translate(-a);
    

end
