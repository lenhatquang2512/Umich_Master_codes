clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

features = get_features(rgb2gray(imread('test_image.png')));
seed = 0;
n_c = 100; 
codewords = get_codewords(features, n_c, seed);
h = get_hist(codewords, features)

%%
% function h = get_hist(codewords, features)
% num_codewords = size(codewords, 1);
% % Write your code here:
% % h = ?;
% 
% % `h` must be a row vector
% %assert(isequal(size(h), [1, num_codewords]))
% 
% % `h` must be normalized
% %assert((sum(h) - 1)^2 < eps)
% end

function h = get_hist(codewords, features)
num_codewords = size(codewords, 1);

% Compute the distances between each feature and all codewords
distances = pdist2(features, codewords);

% Find the index of the nearest codeword for each feature
[~, idx] = min(distances, [], 2);

% Compute the histogram of codeword indices
h = histcounts(idx, 1:num_codewords+1);

% Normalize the histogram
h = h / sum(h);
end