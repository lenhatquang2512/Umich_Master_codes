% clear;clc;close;
syms Kp s
% G = (5*s + 2)/(s*(s^2 + 3*s + 2)) % plant TF
% Gc = (Kp*(s + 3))/(s + 5) % controller TF
% chareq = 1+G*Gc
chareq = 5* s^4 + 38* s^3 + 62*s^2 + (89+10* Kp)*s + (30+40*Kp);
cheq = expand(simplify(chareq))
% haven't figure out how to extract char equation from symbolic, but you can simply copy coefs
% or adapt to you existing char.eq.
%% Update in 2020 - have figured out how to extract coefficients out of char eq
[n, d] = numden(cheq)
cheq = n == 0
cheq = collect(n,s) == 0
R = coeffs(n,s)
%% Now coefficients can be accessed from the vector R and put into Rouht Table
clc;
% R = [0 R]
RT = [R(1,5) R(1,3) R(1,1);
      R(1,4) R(1,2) 0]
% Routh table first two rows from coefs of char.eq. (from cheq)
% RT = [1 17+5*Kp 6*Kp;
%       8 10+17*Kp 0];
% the rest of the table
b1 = (RT(2,1)*RT(1,2)-RT(1,1)*RT(2,2))/RT(2,1);
b2 = (RT(2,1)*RT(1,3)-RT(1,1)*RT(2,3))/RT(2,1);
b3 = 0;
c1 = (b1*RT(2,2)-RT(2,1)*b2)/b1;
c2 = (b1*RT(2,3)-RT(2,1)*b3)/b1;
c3 = 0;
d1 = (c1*b2-b1*c2)/c1;
d2 = (c1*b3-b1*c3)/c1;
d3 = 0;
% full Routh table
RT = [R(1,5) R(1,3) R(1,1);
      R(1,4) R(1,2) 0;
      simplify(b1) b2 b3;
      simplify(c1) c2 c3;
      simplify(d1) d2 d3]
% coeficient Kp values for stability to satisfy condition when b1=0, c1=0 and d1=0
K1 = vpasolve(b1, Kp)
K2 = vpasolve(c1, Kp)
K3 = vpasolve(d1, Kp)