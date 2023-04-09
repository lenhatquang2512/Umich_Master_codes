% Ex2
clc;clear;close all;format default;

syms x
% expand a polynomial expression using MATLAB symbolic, you can use the "expand" function
num_sym = expand((x+3)*(x+2));
den_sym = expand((x+i-1) *(x-i-1));

num = [1 5 6];
den = [1 -2 2];
[r,p,k] = residue(num,den);

H = tf(num,den);

fprintf("Pole-zero map test: \n");
figure(1);
pzmap(H)
sgrid
axis([-4 4 -1.5 1.5])

%root locus
fprintf("Root-locus test: \n");
figure(2);
rlocus(H);
grid on

% Open loop poles and zeros
p1 = 1+1i;
p2 = 1-1i;
z1 = -2;
z2 = -3;

% for pole 1
fprintf("For pole 1: \n");
angles_to_zeros = rad2deg(angle(p1-z1)) + rad2deg(angle(p1-z2));
angles_to_poles = rad2deg(angle(p1-p2));
angles_of_departure = 180 + angles_to_zeros - angles_to_poles


%for pole 2
fprintf("For pole 2: \n");
angles_to_zeros = rad2deg(angle(p2-z1)) + rad2deg(angle(p2-z2))
angles_to_poles = rad2deg(angle(p1-p2))
angles_of_departure = 180 + angles_to_zeros - angles_to_poles

%%
% Ex3
clc;clear;format default;close all;

numG = [1 4];
denG = [5 8 14 5];
numH = 5;
denH = [1 6];
G = tf(numG,denG);
H = tf(numH,denH);

% or G*H
OLsys = series(G,H)

figure();
fprintf("Root-locus test: \n");
rlocus(OLsys);
grid on
axis([-10 10 -10 10])
print -dpng hw3_ex3a_root_locus

rltool(OLsys)



%%
% Still ex3 
format default;
%  Solving inequalities
clear;
syms x
% eqn = (125*x^2 + 21550*x- 126759)/(25*x - 1911) > 0;

S1 = solve((125*x^2 + 21550*x- 126759) < 0,x,'ReturnConditions',true);

S2 = solve(25*x - 1911 < 0,x,'ReturnConditions',true);

simplify(S1.conditions)
simplify(S2.conditions)


%%

% Ex4 
clc;clear;close all;

numG = 2*[1 4];
denG = [5 8 14 5];
numH = 5;
denH = [1 6];
G = tf(numG,denG);
H = tf(numH,denH);

OLsys = G*H

figure();
fprintf("Root-locus test: \n");
rlocus(OLsys);
grid on
axis([-10 10 -10 10])
print -dpng hw3_ex4a_root_locus

rltool(OLsys);

%

Krl = 4;
CLTF_sys = feedback(Krl*G,H)

% p4 = [1 9 12 (15+3*K)]
r4 = roots( CLTF_sys.Denominator{1,1})



%%
% Ex1
% Plot any root locus 

clc;clear;close all;format default;

syms x

% expand a polynomial expression using MATLAB symbolic, you can use the "expand" function

%1a
% num_sym = expand(x-2)
% den_sym = expand((x+1) *(x+3))
% num = [1 -2];
% den = [1 4 3];

%1b
% num_sym = expand(1)
% den_sym = expand((x+1) *(x+i) *(x-i))
% num = [1];
% den = [1 1 1 1];

% 1c
% num_sym = expand((x+1) *(x+3))
% den_sym = expand((x+i) *(x-i))
% num = [1 4 3];
% den = [1 0 1];

% 1d
% num_sym = expand(1)
% den_sym = expand((x+i) *(x-i))
% num = [1];
% den = [1 0 1];

% 1e
% num_sym = expand(1)
% den_sym = expand((x+1) *(x+2)*(x+3)*(x+4))
% num = [1];
% den = [1 10 35 50 24];

% 1f
% num_sym = expand((x+1) *(x+3))
% den_sym = expand((x-1)*(x-3))
num = [1 4 3];
den = [1 -4 3];

[r,p,k] = residue(num,den);
H = tf(num,den);

%root locus
fprintf("Root-locus test: \n");
figure();
rlocus(H);
grid on
axis([-5 5 -5 5])















