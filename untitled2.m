
close all; clc;clear;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
%Let us change now the usual grey background of the matlab figures to white
set(0,'defaultfigurecolor',[1 1 1])

import casadi.*
addpath(genpath('./minvo/src/utils'));
addpath(genpath('./minvo/src/solutions'));

t0=0;
tf=4;

deg_pos=3;
dim_pos=3;

deg_yaw=2;
dim_yaw=1;

num_seg =5; %number of segments


p0=[-4;0;0];
v0=[0;0;0];
a0=[0;0;0];

y0=0;


pf=[4;0;0];
vf=[0;0;0];
af=[0;0;0];

num_of_obst_=1;


opti = casadi.Opti();
sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti); %spline position.

sy=MyClampedUniformSpline(t0,tf,deg_yaw, dim_yaw, num_seg, opti); %spline position.

%This comes from the initial guess, set as decision variables for now
for i=1:(num_of_obst_*sp.num_seg)
    n_{i}=opti.variable(3,1); 
    d_{i}=opti.variable(1,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initial conditions
opti.subject_to( sp.getPosT(t0)== p0 );
opti.subject_to( sp.getVelT(t0)== v0 );
opti.subject_to( sp.getAccelT(t0)== a0 );
opti.subject_to( sy.getPosT(t0)== y0 );

%Final conditions
opti.subject_to( sp.getPosT(tf)== pf );
opti.subject_to( sp.getVelT(tf)== vf );
opti.subject_to( sp.getAccelT(tf)== af );

%Cost
jerk_cost=sp.getControlCost();

yaw_cost=sy.getControlCost();
%PLANE CONSTRAINTS
epsilon=1;


for j=0:(sp.num_seg-1)
    for obst_index=0:(num_of_obst_-1)
        
      ip = obst_index * sp.num_seg + j;  % index plane
       
      init_int=min(sp.timeSpanOfInterval(j)); 
      end_int=max(sp.timeSpanOfInterval(j)); 
      %impose that all the vertexes of the obstacle are on one side of the plane
      vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself

      for r=1:size(vertexes,2) %vertex=vertexes
          vertex=vertexes(:,r);
          opti.subject_to( (-(n_{tm(ip)}'*vertex + d_{tm(ip)} - epsilon))<= 0);
      end
      
      %and the control points on the other side
      Q_Mv=sp.getMINVOCPsofInterval(j);
      for kk=1:size(Q_Mv,2)
        opti.subject_to( n_{tm(ip)}'*Q_Mv{kk} + d_{tm(ip)} + epsilon <= 0);
      end
    end   
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

jit_compilation=false;
opti.minimize( jerk_cost +0.00000000000001*yaw_cost);
opti.solver('ipopt',struct('jit',jit_compilation));
sol = opti.solve();
sp.updateCPsWithSolution(sol)
sy.updateCPsWithSolution(sol)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING! %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sp.plotPosVelAccelJerk()
sy.plotPosVelAccelJerk()


sp.plotPos3D();
plotSphere(p0,0.05,'r'); plotSphere(pf,0.05,'b'); %plotSphere(w_fe(1:3),0.05,'g');

view([45,45]); axis equal
% 
disp("Plotting")
for t_i=t0:0.3:tf  %t_constrained

    w_t_b = sp.getPosT(t_i);
    accel = sp.getAccelT(t_i);% sol.value(A{n})*Tau_i;
%         psiT=sol.value(Psi{n})*Tau_i;

    qabc=qabcFromAccel(accel, 9.81);

    qpsi=[0 0 0 1]';%[cos(psiT/2), 0, 0, sin(psiT/2)]; %Note that qpsi has norm=1
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1

    w_R_b=toRotMat(q);
    w_T_b=[w_R_b w_t_b; 0 0 0 1];
    plotAxesArrowsT(0.2,w_T_b)

end

grid on; xlabel('x'); ylabel('y'); zlabel('z'); 
camlight
lightangle(gca,45,0)

for j=0:(sp.num_seg-1) % i  is the interval (\equiv segment)
    
      init_int=min(sp.timeSpanOfInterval(j)); 
      end_int=max(sp.timeSpanOfInterval(j)); 
      vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself
    
    x=vertexes(1,:);     y=vertexes(2,:);    z=vertexes(3,:);
    
    [k1,av1] = convhull(x,y,z);
    trisurf(k1,x,y,z,'FaceColor','cyan')
    
end


