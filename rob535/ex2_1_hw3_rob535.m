clc; clear ; close all; format default;

% Add paths
addpath('/home/quang_le/Documents/Michigan/Second_semester/ROB535/Hw/Hw3/func')


% T = apple_transformations();
% T = apple_transformations('apple.png');
T = apple_transformations('apple.png', true);

%% 2.1 2D Transformations

function T_final = apple_transformations(file_name, save_fig)
if nargin < 2
    save_fig = false;
    if nargin < 1
        % use the default image
        file_name = 'apple.png';
    end
end

[apple, xyz0, height, width] = read_apple(file_name);

% Write your code here:
%Translate the apple by 150 pixels along x and 100 pixels along y
T1 = mtx_translate([150;100;0]) ;

% ii. Rotate the apple 15 degrees about the point (120, 150).
T2 = mtx_rotate([0 ;0 ;deg2rad(15)], [120;150;0]);

% iii. Scale the apple by 200% in both x and y while fixing the point (200, 200).
T3 = mtx_scale([2 ;2; 1], [200;200;0]);

% iv. Rotate the apple - 30 degrees about the point (200, 250)
T4 = mtx_rotate([0 ;0 ;deg2rad(-30)], [200;250;0]);

% Combine all transformations
T_final = T4 * T3 * T2 * T1;

% visualization
figure(1); clf();
set(gcf, 'Position', [100, 100, 800, 400])

corners = [
    0, width - 1, width - 1, 0, 0
    0, 0, height - 1, height - 1, 0
    zeros(1, 5)
    ones(1, 5)
];

ax1 = subplot(1, 2, 1);
visualize_apple(apple, xyz0, corners)
title('Original')

ax2 = subplot(1, 2, 2);
visualize_apple(apple, T_final * xyz0, T_final * corners)
title('Transformed')

linkaxes([ax1, ax2])

%
if save_fig
    set(gcf, ...
        'PaperPositionMode', 'Auto', ...
        'PaperUnits', 'Inches', ...
        'PaperSize', [8, 4], ...
        'Renderer', 'Painters')
    print('hw1_p4_1.pdf', '-dpdf', '-fillpage')
    set(gcf, 'Renderer', 'opengl')
end
end

%%
function [apple, xyz, height, width] = read_apple(file_name)
apple = rgb2gray(imread(file_name));
[height, width] = size(apple);

% put everything in homogeneous coordinates
[y, x] = meshgrid(0:(height - 1), 0:(width - 1));
xyz = [
    reshape(x, 1, [])
    reshape(y, 1, [])
    zeros(1, height * width)
    ones(1, height * width)
];
end

%%
function visualize_apple(image, xyz, corners)
[height, width] = size(image);
x = reshape(xyz(1, :), [width, height]);
y = reshape(xyz(2, :), [width, height]);

h = pcolor(x, y, image.');
set(h, 'EdgeColor', 'none')
set(gca, 'Ydir', 'reverse')

colormap gray
caxis([0, 255])

hold on
plot(corners(1, :), corners(2, :), 'r')
hold off

axis equal; axis([0, 400, 0, 450])
xlabel('x'); ylabel('y')
end

