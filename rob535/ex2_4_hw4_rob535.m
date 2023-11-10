clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw4/func')

[idx, dist] = query('kitti_bow.mat');


%%
function [idx, dist] = query(bow_file)

if nargin < 1
    % calculate everythin from scratch
    files = dir('kitti/*.png');
    num_images = numel(files);

    features = cell(num_images, 1);
    for i = 1:num_images
        img = rgb2gray(imread(['kitti/', files(i).name]));
        features{i} = get_features(img);
    end

    features_all = cell2mat(features);
    num_codewords = 100;
    seed = 0;
    codewords = get_codewords(features_all, num_codewords, seed);

    % the i-th row of `h_train` is the BoW representation of the i-th image
    h_train = zeros(num_images, num_codewords);
    for i = 1:num_images
        h_train(i, :) = get_hist(codewords, features{i});
    end

    save('kitti_bow.mat', 'h_train', 'codewords', 'seed')
else
    % load existing file
    bow = load(bow_file);

    codewords = bow.codewords;
    h_train = bow.h_train;
end

% start from here
img_q = rgb2gray(imread('query.png'));

features_q = get_features(img_q);
% codewords_q = get_codewords(features_q,100,0);
h_q = get_hist(codewords,features_q);

num_images = size(h_train,1);

dist_list = zeros(num_images,1);
for i  = 1:num_images
    dist_list(i) = chi_sq_dist(h_q,h_train(i,:));
end

[~,idx_nearest_img] = min(dist_list);

% Write your code here:
dist = dist_list;
idx = idx_nearest_img;

figure(1)
clf()
plot(dist)
xlim([0, numel(dist) + 1])
ylim([0, 0.3])
xlabel('Image Index $i$', 'interpreter', 'latex')
ylabel('dist$(h_q,h_i)$', 'interpreter', 'latex')
% print -dpng ex2_4_dist_bow
end

function d = chi_sq_dist(h1, h2)
d = sum((h1 - h2).^2 ./ (h1 + h2 + eps), 2) / 2;
end