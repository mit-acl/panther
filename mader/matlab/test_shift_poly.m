clc; clear; close all;
% addpath(genpath('./../../submodules/minvo/src/utils'));
% addpath(genpath('./../../submodules/minvo/src/solutions'));
% addpath(genpath('./more_utils'));


coeff_p=[5.0, 1.0, 2.0, 3.0, 6.0]';
syms t real; T=(t.^[(numel(coeff_p)-1):-1:0])';
p=coeff_p'*T; a=7.0; b=3.0; q=subs(p,t,a*t+b);
coeffs(q,'All')

syms tq1 tq2 tp1 tp2 a b;


tp1=3.0;
tp2=50;
tq1=1.0;
tq2=7.0;
syms a b
% syms tq1 tq2 tp1 tp2 a b;
s=solve([tp1==a*tq1+b, tp2==a*tq2+b],[a,b])

coeff_p=[5.0, 1.0, 2.0, 3.0, 6.0]';
syms t real; T=(t.^[(numel(coeff_p)-1):-1:0])';
p=coeff_p'*T;
q=subs(p,t,s.a*t+s.b);

assert(subs(p, t,tp1) - subs(q, t,tq1) ==0)
assert(subs(p, t,tp2) - subs(q, t,tq2) ==0)

vpa(coeffs(q,'All'),6)