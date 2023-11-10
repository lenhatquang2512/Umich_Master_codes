clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

seed = 0;
n_c = 100;
features = get_features(rgb2gray(imread('test_image.png')));
codewords = get_codewords(features, n_c, seed);
disp(codewords);

%%
function codewords = get_codewords(feature_all, num_codewords, seed)
if nargin == 3
    rng(seed);
end
% Write your code here:
[~,codewords ] = kmeans(feature_all,num_codewords,'MaxIter',100000);

end

% function features = get_features(image)
% 
% % Extract SURF features from the input image
% % INPUTS:
% %   image: grayscale input image
% % OUTPUTS:
% %   features: matrix with 64 columns, each row is a SURF descriptor
% 
% % detect SURF features
% points = detectSURFFeatures(image);
% 
% % extract SURF descriptors
% [features, ~] = extractFeatures(image, points);
% 
% features = double(features);
% end