clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

% image = rgb2gray(imread('test_image.png'))
image = rgb2gray(imread('query.png'));
features = get_features(image);

%%
function features = get_features(image)

% Extract SURF features from the input image
% INPUTS:
%   image: grayscale input image
% OUTPUTS:
%   features: matrix with 64 columns, each row is a SURF descriptor

% detect SURF features
points = detectSURFFeatures(image);

% extract SURF descriptors
[features, ~] = extractFeatures(image, points);

features = double(features);
end