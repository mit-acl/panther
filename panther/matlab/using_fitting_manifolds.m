close all;
clear all;
clc;


addpath(genpath('./../../../manint/manopt'));
addpath(genpath('./../../../manint/methods'));

import casadi.*
addpath(genpath('./../../submodules/minvo/src/utils'));
addpath(genpath('./../../submodules/minvo/src/solutions'));
addpath(genpath('./more_utils'));
addpath(genpath('./hopf_visualization'));

set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'

opti = casadi.Opti();


deg_pos=3;
deg_yaw=2;
num_seg =4; %number of segments
num_max_of_obst=10; %This is the maximum num of the obstacles 

basis="MINVO"; %MINVO OR B_SPLINE or BEZIER. This is the basis used for collision checking (in position, velocity, accel and jerk space), both in Matlab and in C++

t0=0; 
tf=10.5;

dim_pos=3;
dim_yaw=1;

sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti); %spline position.
sy=MyClampedUniformSpline(t0,tf,deg_yaw, dim_yaw, num_seg, opti); %spline yaw.

pCPs=[   -4.0000   -4.0000   4.0000    0.7111    3    3   4;
         -2         0         0   5   -0.0131   2   -0.0131;
         0         2         -2    0.6275    0.0052    3    0.0052];
     
sp.setCPoints(mat2cell(pCPs,[3],[ones(1,size(pCPs,2))]))



sp.plotPos3D(); grid on;


my_colormap=autumn;

b_T_c= [roty(90)*rotz(-90) zeros(3,1); zeros(1,3) 1];
w_fevar=[1 -2 3]';

for t=t0:0.6:tf
    
    pos=sp.getPosT(t);
    a=sp.getAccelT(t);
    xi=a+[0 0 9.81]';
    tmp=pos+xi/norm(xi);
    
    arrow3d(pos',tmp',20,'cylinder',[0.2,0.1]);
    
    qabc=qabcFromAccel(a, 9.81);
    
    
    w_T_b=[toRotMat(qabc) pos; 0 0 0 1];
    plot3dcircleXY(w_T_b,0.3,my_colormap(2,:),0.5)
    
    
    n_psi_samples=30;
    all_psi=linspace(0,2*pi,n_psi_samples);
    
%     my_map=phasemap(n_psi_samples);
    
    my_map=autumn;
    my_map=[my_map];%; flip(my_map)]
       
    all_values=[];
    all_projs=[];
    all_tmp=[];
    for i=1:numel(all_psi)%=0:2*pi
        psi=all_psi(i);
        q=quatmultiply(qabc, [cos(psi/2.0) 0.0 0.0 sin(psi/2.0)]);
        R=toRotMat(q);
        
        b0=R(:,1)*norm(a);
        ee=w_fevar-pos;
        value=norm(ee)*cosd(80)*norm(a)-b0'*ee; %This is the cost (the more negative it is--> the more in the FOV it is)
        all_values=[all_values value];

        all_projs=[all_projs R(:,1)+pos]; %Assumming here that
%          c_T_b=invPose(b_T_c);
%          b_T_w=invPose(w_T_b);
%     
%          c_P=c_T_b*b_T_w*[w_fevar;1]; %Position of the feature in the camera frame
%          s=c_P(1:2)/(c_P(3));  %Note that here we are not using f (the focal length in meters) because it will simply add a constant factor in ||s|| and in ||s_dot||
%          
%          norm(s)
%         
    end
    
%     all_values=all_projs(1,:);
    
    all_values= (all_values - min(all_values)) / ( max(all_values) - min(all_values) ); %normalize \in [0,1]
    
    n=size(my_map,1);
    all_values=  ceil((n-1)*all_values + 1); %normalize \in [1,n]
    colors=my_map(all_values',:);
%     for i=1:numel(all_psi)
%          index=ceil(all_values*numel(my_phasemap))+1;
%          index=max(index,numel(my_phasemap));
%          colors=[colors my_phasemap(index)]
%     end

h=colormapline(all_projs(1,:),all_projs(2,:),all_projs(3,:),colors); axis equal;
set(h,'linewidth',4,'linestyle','--')
    
    
end

plotSphere(w_fevar,0.2,'g');



%%


n_d=100; %number of datapoints
f = @(phi) [cos(phi) , sin(phi)];
phi     = linspace(0,2*pi-0.5,n_d)'+2*rand(1,n_d)';
dataPoints = mat2cell(f(phi),ones(1,length(phi)),2);
nData   = length(dataPoints);
dataCoords = linspace(0,1,nData);
M = spherefactory(2);
t = linspace(0,1,(nData-1)*30);
lambda = 3; % fitting
S = blendedCurve(M,dataPoints,dataCoords,t,'lambda',lambda);

figure;

angles=[];
for i=1:size(S,1)
    angles=[angles atan2(S(i,1,2), S(i,1,1))];
end

% angles_datapoints=[];
% for i=1:size(S,1)
%     angles_datapoints=[angles_datapoints atan2(S(i,1,1), S(i,1,2))]
% end

plot(t,angles,'-o'); hold on;
plot(dataCoords,phi,'o')