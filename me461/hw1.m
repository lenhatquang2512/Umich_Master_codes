%% 
clear;clc; close all;
% p = [1 16 63];
p = [1 5 6]
r = roots(p)


%% strictly proper TF with real non-repearting roots (case 1)

clear;clc; close all;
format rat;
num = 6;
den = [1 16 63 0];
[r,p,k] = residue(num,den)

%%
clear;clc; close all;
format rat;

A = [1 1 0;6 15 1; 25 0 15];
b = [0 1 7].';

%sol c1,c2,c3
c = inv(A) *b

%% strictly proper TF with complex roots (Case 2)

clear;clc; close all;
format rat;
syms x;

% expand a polynomial expression using MATLAB symbolic, you can use the "expand" function
den_sym = expand((x+15) *(x^2 + 6*x + 25))

num = [1 7];
den = [1 21 115 375];
[r,p,k] = residue(num,den)

%just imaginary parts
Y = + r(2)*(1/(x - p(2))) + r(3)*(1/(x - p(3)));

%simplify to check 
simplify(Y)

%make tf
H = tf(num,den)

%convert tf type to sym type
syms s
Numerator = poly2sym(H.Numerator{1,1},s);
Denominator = poly2sym(H.Denominator{1,1},s);
Hsym = Numerator/Denominator

% To convert a transfer function in the s-domain to the time domain using MATLAB
H_tf_t = ilaplace(Hsym)

%%  strictly proper TF with real repeated roots (Case 3)
clear;clc; close all;
format rat;
syms x;

% syms x;

% expand a polynomial expression using MATLAB symbolic, you can use the "expand" function
den_sym = expand((x+6) *(x^2 + 3*x + 3))

num = [1 -4];
den = [1 9 21 18];
[r,p,k] = residue(num,den)

H = tf(num,den)

%convert tf type to sym type
syms s
Numerator = poly2sym(H.Numerator{1,1},s);
Denominator = poly2sym(H.Denominator{1,1},s);
Hsym = Numerator/Denominator;

% To convert a transfer function in the s-domain to the time domain using MATLAB
H_tf_t = ilaplace(Hsym)






