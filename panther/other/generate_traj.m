close all; clear; clc;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
syms t real

slower=1.0

offset=0.0;
s=[1.0;1.0;1.0];
c=[0.0;0.0;0.0]
dc=0.01;

tt=t/slower;
x=s(1)*sin(tt+offset)+2*sin(2*tt+offset)+c(1);
y=s(2)*cos(tt+offset)-2*cos(2*tt+offset)+c(2);
z=s(3)*(-sin(3*tt+offset))+c(3);

all_t=0:dc:2*pi;
p=[];
for t_i=all_t
    t_i
    p=[p  double([subs(x,t,t_i);subs(y,t,t_i);subs(z,t,t_i)])];
end

figure(1); hold on
h=colormapline(p(1,:),p(2,:),p(3,:),winter);