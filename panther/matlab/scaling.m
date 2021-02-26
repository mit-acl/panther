
close all; clc; clear;

syms t real


f=sin(t);

alpha=1.5;

diff1=alpha*diff(f,t)

diff2=diff(subs(f,t,alpha*t),t)

fplot()
%%


t0=1;
tf=8;
figure; subplot(2,1,1); hold on; 
fplot(f,[t0,tf])

alpha=0.2;
f_scaled=sin(alpha*t);
fplot(f_scaled,[t0/alpha,tf/alpha])

subplot(2,1,2); hold on
derivative=diff(f,t);
fplot(derivative,[t0,tf])


fplot(derivative/alpha,[t0/alpha,tf/alpha])