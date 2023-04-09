
% ex2

clear;clc;close all; format default;

numG1 = [1 2];
denG1 = 4*[1 2 8];
numG2 = 8;
denG2 = [1 8];
numG3 = 6;
denG3 = [1 0 0];
numH1 = 3*[1 5];
denH1 = 1;
numH2 = 50;
denH2 = [1 50];

G1 = tf(numG1,denG1);
G2 = tf(numG2,denG2);
G3 = tf(numG3,denG3);
H1 = tf(numH1,denH1);
H2 = tf(numH2,denH2);

%G and H
G = G1- G2*G3;
H = H1 + H2;

% --------Using conv--------------------------------------
fprintf("Finding OLTF and CLTF using conv method: \n");
%OLTF
numG = G.Numerator{1,1};
denG = G.Denominator{1,1};
numH = H.Numerator{1,1};
denH = H.Denominator{1,1};
num_GH = conv(numG,numH);
den_GH = conv(denG,denH);

OLTF = tf(num_GH,den_GH)

%CLTF
num_CLTF = conv(numG,denH);
DGDH = conv(denG,denH);
NGNH = conv(numG,numH);
den_CLTF = DGDH + NGNH; %DGDH and NGNH have same size ->no need to zeropadd

CLTF = tf(num_CLTF,den_CLTF)

%Ex3
%--------Using series,paraller,feedback command------------
fprintf("Finding OLTF and CLTF using series,paraller,feedback command: \n");

%OLTF
OLTF = series(parallel(G1,-series(G2,G3)),parallel(H1,H2))
%in short, just use 
% OLTF = series(G,H)

%CLTF
CLTF = feedback(parallel(G1,-series(G2,G3)),parallel(H1,H2))
%in short, just use 
% OLTF = feedback(G,H)

%%

%Ex4
clear;clc;close all;
fprintf("K inside stable range (K= 10): \n");
K = 10;
p4 = [1 9 12 (15+3*K)]
r4 = roots(p4)

fprintf("K outside stable range (K= 40): \n");
K = 40;
p4 = [1 9 12 (15+3*K)]
r4 = roots(p4)

%Ex5
% clear;clc; close all;
p5 = [3 6 4 8 2 2]
r5 = roots(p5)

%Ex6
p6 = [1 4 2 8 6 24]
r6 = roots(p6)



