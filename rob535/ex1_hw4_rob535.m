clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

w_radius = 4;
max_d = 64;
min_d = 0;
% I1 = imread('t0_left.png');
% I2 = imread('t0_right.png');
I1 = imread('t1_left.png');
I2 = imread('t1_right.png');
% Run learner solution.
[D]=genDisparityMap(I1, I2, min_d, max_d, w_radius);
% print -dpng ex1_disparity_sol
% Run reference solution.
[D_ref]=genDisparityMap_ref(I1, I2, min_d, max_d, w_radius);
% Assesment.
accuracy = sum(sum((D - D_ref).^2 < eps)) / numel(D);
% assessVariableEqual('accuracy', 0.99, 'AbsoluteTolerance', 0.01+eps,'Feedback','More than 2 percent of the points have incorrect Disparity values')
eva = ismembertol(accuracy,0.99,0.01+eps);
fprintf("Compare result is %d\n",eva);


function [D] = genDisparityMap_ref(I1, I2, min_d, max_d, w_radius)
% INPUT
%   I1 the left stereo image
%   I2 the right stereo image
%   min_d minimum disparity
%   max_d maximum disparity
%   w_radius the radius of the window to do the AD aggeration
%
% OUTPUT
%   D disparity values

if nargin < 5, w_radius = 4; end % 9x9 window
if nargin < 4, max_d = 64; end
if nargin < 3, min_d = 0; end

% Grayscale Images are sufficient for stereo matching
% The Green channel is actually a good approximation of the grayscale, we
% could instad do I1 = I1(:,:,2);  
if size(I1, 3) > 1, I1 = rgb2gray(I1); end
if size(I2, 3) > 1, I2 = rgb2gray(I2); end

% convert to double/single
I1 = double(I1);
I2 = double(I2);

% Calculate SAD values for each pixel in the Left Image.
kernel_size = 2 * w_radius +1;
% Create kernel for weighting pixel differences
% kernel = ones(kernel_size);

% Initialize the disparity map
[h, w] = size(I1);
D = zeros(h, w); % initialize disparity map

% % Pad images with zeros
paddedI1 = padarray(I1,[w_radius w_radius]);
paddedI2 = padarray(I2,[w_radius w_radius]);

for i = 1+w_radius : h+w_radius
    left_window = zeros(w,kernel_size,kernel_size);
    right_window = zeros(w,kernel_size,kernel_size);
    for j = 1+w_radius : w+w_radius
        left_window(j-w_radius,:,:) = paddedI1(i-w_radius:i+w_radius, j-w_radius:j+w_radius);
        right_window(j-w_radius,:,:) = paddedI2(i-w_radius:i+w_radius, j-w_radius:j+w_radius);
%             SAD(k) = sum(sum(abs(left_window - shiftedI2).*kernel));
    end
    
    for k = 1:w
        
        diff  = abs(repmat(left_window(k,:,:),min(max_d+1,k),1,1) - right_window(max(min_d+1,k-max_d):k,:,:));
        all_sum = sum(flip(diff,1),[2,3]);
        [~,idx] = min(all_sum);
        D(i - w_radius, k) = idx-1;
    end

end

% Visualize disparity map
figure;
imagesc(D, [0, max_d]);
colormap(gray);
colorbar;
axis image;
end



function [D] = genDisparityMap(I1, I2, min_d, max_d, w_radius)
% INPUT
%   I1 the left stereo image
%   I2 the right stereo image
%   min_d minimum disparity
%   max_d maximum disparity
%   w_radius the radius of the window to do the AD aggeration
%
% OUTPUT
%   D disparity values

if nargin < 5, w_radius = 4; end % 9x9 window
if nargin < 4, max_d = 64; end
if nargin < 3, min_d = 0; end

% Grayscale Images are sufficient for stereo matching
% The Green channel is actually a good approximation of the grayscale, we
% could instad do I1 = I1(:,:,2);  
if size(I1, 3) > 1, I1 = rgb2gray(I1); end
if size(I2, 3) > 1, I2 = rgb2gray(I2); end

% convert to double/single
I1 = double(I1);
I2 = double(I2);

% Calculate SAD values for each pixel in the Left Image.
kernel_size = 2 * w_radius +1;
% Create kernel for weighting pixel differences
kernel = ones(kernel_size);

% Initialize the disparity map
[h, w] = size(I1);
D = zeros(h, w); % initialize disparity map

% % Pad images with zeros
paddedI1 = padarray(I1,[w_radius w_radius]);
paddedI2 = padarray(I2,[w_radius w_radius]);

% % % min_d:max_d the range of disparity values from min_d to max_d inclusive
d_vals = min_d:max_d;
% D is the Disparity Matrix and is the same size as that of the Images
% SAD = zeros(1,length(d_vals));
% Calculate SAD values for each pixel in the Left Image
for i = 1+w_radius : h+w_radius
    for j = 1+w_radius : w+w_radius
        left_window = paddedI1(i-w_radius:i+w_radius, j-w_radius:j+w_radius);
%         for k = 1:length(d_vals)
%             if (j-w_radius-d_vals(k)) < 1  % check if the window is out of the image
%                 continue;
%             end
%             right_window = paddedI2(i-w_radius:i+w_radius, j-w_radius-d_vals(k):j+w_radius-d_vals(k));
%             SAD(k) = conv2(abs(right_window - left_window), kernel, 'valid');
% %               SAD(k) = sum(sum(abs(left_window - right_window).*kernel));
% %             if SAD < best_match
% %                 best_match = SAD;
% %                 D(i-w_radius, j-w_radius) = d;
% %             end
%         end
% 
%         % Find disparity with minimum SAD value
%         [~, min_index] = min(SAD);
%         D(i - w_radius, j - w_radius) = d_vals(min_index);
        best_match = Inf;
        for d = d_vals
            if (j-w_radius-d) < 1 % check if the window is out of the image
                continue;
            end
            right_window = paddedI2(i-w_radius:i+w_radius, j-w_radius-d:j+w_radius-d);
%             SAD = sum(sum(abs(right_window - left_window).*kernel));
            SAD = conv2(abs(right_window - left_window), kernel, 'valid');
            if SAD < best_match
                best_match = SAD;
                D(i-w_radius, j-w_radius) = d;
            end
        end
    end
end


% Visualize disparity map
figure;
imagesc(D, [0, max_d]);
colormap(gray);
colorbar;
axis image;
end
