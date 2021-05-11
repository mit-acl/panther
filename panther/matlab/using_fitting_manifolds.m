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
set(0,'defaultfigurecolor',[1 1 1])
opti = casadi.Opti();


deg_pos=3;
deg_yaw=2;
num_seg =4; %number of segments
num_max_of_obst=10; %This is the maximum num of the obstacles 

basis="MINVO"; %MINVO OR B_SPLINE or BEZIER. This is the basis used for collision checking (in position, velocity, accel and jerk space), both in Matlab and in C++

t0=0; 
tf=8.5;

dim_pos=3;
dim_yaw=1;

sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti); %spline position.
sy=MyClampedUniformSpline(t0,tf,deg_yaw, dim_yaw, num_seg, opti); %spline yaw.

pCPs=[   -4.0000   -4.0000   8    5    3   2     -4;
         -2         0         0   -5   -7   -8   -9;
         0         2         -2    7    0.0052    3    0.0052];
     
sp.setCPoints(mat2cell(pCPs,[3],[ones(1,size(pCPs,2))]))



sp.plotPos3D(); grid on;


my_colormap=autumn;

b_T_c= [roty(90)*rotz(-90) zeros(3,1); zeros(1,3) 1];
w_fevar=[1 -2 3]';

all_t=t0:0.1:tf;
index_t=1;

n_psi_samples=30;
all_psi=linspace(0,2*pi,n_psi_samples);

all_circles_all_values=zeros(numel(all_t),numel(all_psi)); %Each row is a different t. Each column in one row is the value for a specific psi

for t=all_t
    
%     index_t
    
    pos=sp.getPosT(t);
    a=sp.getAccelT(t);
    ee=w_fevar-pos;
    xi=a+[0 0 9.81]';
    tmp=pos+xi/norm(xi);
    
    arrow3d(pos',tmp',20,'cylinder',[0.2,0.1]);
    qabc=qabcFromAccel(a, 9.81);
    
    
    w_T_b=[toRotMat(qabc) pos; 0 0 0 1];
    plot3dcircleXY(w_T_b,0.3,my_colormap(2,:),0.5)
           
    
    
    w_T_abc=[toRotMat(qabc) pos; 0 0 0 1];
    
    all_values=[];
    all_projs=[];
    all_tmp=[];
%     X=[];
%     Y=[];
    for i=1:numel(all_psi)%=0:2*pi
        psi=all_psi(i);
        q=quatmultiply(qabc, [cos(psi/2.0) 0.0 0.0 sin(psi/2.0)]);
        R=toRotMat(q);
        w_R_b=R;
        w_T_b=[w_R_b pos; 0 0 0 1];
       
%         abc_T_b=(w_T_abc)^(-1)*w_T_b;
%         assert(sum(abs(abc_T_b(3,1:2)))<1e-4);
%         X=[X  abc_T_b(1:2,1)];
%         Y=[Y  abc_T_b(1:2,2)];
%          
        
        b0=R(:,1)*norm(a);
        
        value=norm(ee)*cosd(80)*norm(a)-b0'*ee; %This is the cost (the more negative it is--> the more in the FOV it is)
        value=value/(norm(a)*norm(ee)); %This doesn't affect the constraint, and makes the value non-dimensional 
        all_values=[all_values value];
        all_circles_all_values(index_t,i)=value;

        all_projs=[all_projs R(:,1)+pos]; %Assumming here that
       
    end
    
    colors=getColors(all_values);
   

h=colormapline(all_projs(1,:),all_projs(2,:),all_projs(3,:),colors); axis equal;
set(h,'linewidth',4,'linestyle','--')
    
index_t=index_t+1;
end

plotSphere(w_fevar,0.2,'g');
p1=[w_fevar(1:2);0];
p2=w_fevar;
plot3([ p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)],'--');
% plot([w_fevar(1:2);0],w_fevar,'--' )

camlight




angles_datapoints=[];
dataPoints={};
for t=all_t
    
    pos=sp.getPosT(t);
    a=sp.getAccelT(t);
    ee=w_fevar-pos;
    xi=a+[0 0 9.81]';

    r0_star=(ee*norm(xi)^2 - (ee'*xi)*xi);
    r0_star=r0_star/norm(r0_star);
    
%     r3=xi/norm(xi);
%     r2=cross(r3,r0_star);
    
    tmp=pos+r0_star;
    arrow3dWithColor(pos',tmp',30,'cylinder',[0.2,0.1],'b');

    qabc=qabcFromAccel(a, 9.81);
    Rabc=toRotMat(qabc);
    Tabc=[Rabc pos; 0 0 0 1];
    
    b_r0star=inv(Tabc)*[tmp;1];
    b_r0star=b_r0star(1:3);
    assert(abs(b_r0star(3))<0.0001)
    
    dataPoints{end+1}=b_r0star(1:2)';
    angles_datapoints=[angles_datapoints atan2(b_r0star(2),b_r0star(1))];
%     
%     u=r0_star; v=Rabc(:,1);
%     CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
%     angles=[angles real(acos(CosTheta))];    
%     
%     dataPoints=[dataPoints  (u'*v)*v u-(u'*v)*v];
%     
%     
%     w_T_b=[toRotMat(qabc) pos; 0 0 0 1];
    
end

dataPoints=dataPoints';





figure; hold on;
[r,theta] = meshgrid(ones(1,numel(all_t)),all_psi);
r=r'; theta=theta'; %Every row of the matrices r, theta, all_t_grid corresponds to a slice of the cilinder (i.e. a circunference)
                    %Every column of the matrices .................corresponds to a longitudinal line of the cylinder
all_t_grid=repmat(all_t',1,size(r,2));
surf(all_t_grid,r.*cos(theta),r.*sin(theta),all_circles_all_values); xlabel('t'); axis equal
colormap jet; colorbar; shading interp; % caxis([20 50])
curve=[all_t;cell2mat(dataPoints)'];
plot3(curve(1,:), curve(2,:), curve(3,:),'r','LineWidth',3)



%%


n_d=100; %number of datapoints
f = @(phi) [cos(phi) , sin(phi)];
angles_datapoints     = linspace(0,2*pi-0.5,n_d)'+2*rand(1,n_d)';
dataPoints = mat2cell(f(angles_datapoints),ones(1,length(angles_datapoints)),2);

nData   = length(dataPoints);
dataCoords = linspace(0,1,nData);
M = spherefactory(2);
t = linspace(0,1,(nData-1)*30);
lambda = 300; % fitting
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

scatter(dataCoords,angles_datapoints,80,'o','filled');hold on;
% plot(t,angles,'-o'); 
yline(pi,'--'); yline(-pi,'--'); xlabel('t'); ylabel('psi');

function colors=getColors(all_values)
    my_map=jet;
    all_values= (all_values - min(all_values)) / ( max(all_values) - min(all_values) ); %normalize \in [0,1]
    n=size(my_map,1);
    all_values=  ceil((n-1)*all_values + 1); %normalize \in [1,n]
    colors=my_map(all_values',:);
end