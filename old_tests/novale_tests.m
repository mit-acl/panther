
clear; clc; 

import casadi.*
addpath(genpath('./utils'));

t0=0;
tf=4;

s=MyClampedUniformSpline(t0, tf, 3, 4);


a{1}=rand(3,1);
for i=1:s.N
    a{end+1}=rand(3,1);
end

s.setCPoints(a)

% syms t real
% s.evalDerivative(0,t,1)
% 
% pos=s.getPos(t,1);



s.plotPosVelAccelJerk();
s.plotPosVelAccelJerkFiniteDifferences()



