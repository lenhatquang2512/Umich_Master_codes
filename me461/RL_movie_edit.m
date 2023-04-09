% numG=[1 2 -3];
% denG=[1 8 12];
% numH=[1 5];
% denH=[1 8];
clc;clear;close all;format default;
numG=[1 4 3];
denG=[1 0 1];
numH=[1];
denH=[1];

% num_sym = expand((x+1) *(x+3))
% den_sym = expand((x+i) *(x-i))
% num = [1 4 3];
% den = [1 0 1];


G=tf(numG,denG);
H=tf(numH,denH);
OLTF=series(G,H);
pzmap(OLTF);
sgrid
axis([-10 2 -2 2]);
hold on
for m=1:1000
    K=m/100;
    den=conv(denG,denH)+K*conv(numG,numH);
    R=roots(den);
    pause
%     plot(real(R(1)),imag(R(1)),'bs',real(R(2)),imag(R(2)),'bd',...
%         real(R(3)),imag(R(3)),'b.')
    plot(real(R(1)),imag(R(1)),'bs',real(R(2)),imag(R(2)),'bd')
end
       