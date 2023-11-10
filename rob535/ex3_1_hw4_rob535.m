clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

% Run the Pretest
I1 = imread('t0_left.png');
I2 = imread('t0_right.png');
% Run learner solution.
[matchedPoints1, matchedPoints2, matchedFeatures1, matchedFeatures2] = detectFeaturePoints(I1, I2);

%%
function [matchedPoints1, matchedPoints2, matchedFeatures1, matchedFeatures2] = detectFeaturePoints(I1, I2)
if(size(I1,3) == 3)
    I1 = rgb2gray(I1);
end

if(size(I2,3) == 3)
    I2 = rgb2gray(I2);
end

% detect SURF points
points1 = detectSURFFeatures(I1);
points2 = detectSURFFeatures(I2);

% extract SURF features
[features1, validPoints1] = extractFeatures(I1, points1);
[features2, validPoints2] = extractFeatures(I2, points2);

% match features
indexPairs = matchFeatures(features1, features2);

% obtain matched points
matchedPoints1 = validPoints1(indexPairs(:,1), :);
matchedPoints2 = validPoints2(indexPairs(:,2), :);

% obtain matched features
matchedFeatures1 = features1(indexPairs(:,1), :);
matchedFeatures2 = features2(indexPairs(:,2), :);

% Write your code here:
% Hint: use the function `matchFeatures`
% matchedPoints1 = ??;
% matchedPoints2 = ??;
% matchedFeatures1 = ??;
% matchedFeatures2 = ??;

end