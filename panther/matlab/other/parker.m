close all; clc; clear;
t0=0.0;
tf=5.0;
deg_pos=3; %degree of the polynomial for each interval. TODO: modify computeMatrixForAnyBSpline() function so that it accepts degrees>=3
num_seg=15;
dim_pos = 3 %1D, 2D, 3D, ...

c_smooth=0.0;
c_closeness=1.0;

v_max=3.0*ones(3,1);
a_max=6.0*ones(3,1);
j_max=10.0*ones(3,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import casadi.*
addpath(genpath('./../../submodules/minvo/src/utils'));
addpath(genpath('./../../submodules/minvo/src/solutions'));
addpath(genpath('./more_utils'));
opti = casadi.Opti();
basis="MINVO"; %MINVO OR B_SPLINE or BEZIER. This is the basis used for collision checking (in position, velocity, accel and jerk space), both in Matlab and in C++
sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti); %spline position.


p=[0.0, 0.0, 2.0, 2.0, 4.0, 4.0;
   0.0, 2.0, 2.0, 0.0, 0.0, 2.0;
   0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

p0=p(:,1);
pf=p(:,end);

opti.subject_to( sp.getPosT(t0)== p0 );
opti.subject_to( sp.getPosT(tf)== pf );

%Max vel constraints (position)
for j=1:sp.num_seg
    vel_cps=sp.getCPs_XX_Vel_ofInterval(basis, j);
    dim=size(vel_cps, 1);
    for u=1:size(vel_cps,2)
        for xyz=1:3
            opti.subject_to( vel_cps{u}(xyz) <= v_max(xyz)  )
            opti.subject_to( vel_cps{u}(xyz) >= -v_max(xyz) )
        end
    end
end

%Max accel constraints (position)
for j=1:sp.num_seg
    accel_cps=sp.getCPs_XX_Accel_ofInterval(basis, j);
    dim=size(accel_cps, 1);
    for u=1:size(accel_cps,2)
        for xyz=1:3
            opti.subject_to( accel_cps{u}(xyz) <= a_max(xyz)  )
            opti.subject_to( accel_cps{u}(xyz) >= -a_max(xyz) )
        end
    end
end

%Max jerk constraints (position)
for j=1:sp.num_seg
    jerk_cps=sp.getCPs_MV_Jerk_ofInterval(j);
    dim=size(jerk_cps, 1);
    for u=1:size(jerk_cps,2)
        for xyz=1:3
            opti.subject_to( jerk_cps{u}(xyz) <= j_max(xyz)  )
            opti.subject_to( jerk_cps{u}(xyz) >= -j_max(xyz) )
        end
    end
end

pos_smooth_cost=sp.getControlCost();

pos_closeness_cost=0.0;

t=linspace(t0, tf, size(p,2));
ppx=pchip(t,p(1,:)); ppy=pchip(t,p(2,:)); ppz=pchip(t,p(3,:));
for ti=linspace(t0, tf, 100)
    tmp=sp.getPosT(ti)- [ppval(ppx, ti), ppval(ppy, ti), ppval(ppz, ti)]';
    pos_closeness_cost = pos_closeness_cost +  tmp'*tmp;
end

total_cost= c_smooth*pos_smooth_cost + c_closeness*pos_closeness_cost;
opti.minimize(simplify(total_cost));

opts = struct;
opts.expand=true; %When this option is true, it goes WAY faster!
opts.print_time=true;
opts.ipopt.print_frequency_iter=1;%1e10 %Big if you don't want to print all the iteratons


opti.solver('ipopt',opts); %{"ipopt.hessian_approximation":"limited-memory"} 
sol = opti.solve();

sp.updateCPsWithSolution(opti.value(sp.getCPsAsMatrix()));
sp.plotPosVelAccelJerk()

all_xy=[]
for ti=linspace(t0, tf, 100)
    all_xy=[all_xy sp.getPosT(ti)];
end

figure
plot(all_xy(1,:), all_xy(2,:))
%%
% close all; clc; clear;
% x=[0.0, 0.0, 2.0, 2.0, 4.0, 4.0];
% y=[0.0, 2.0, 2.0, 0.0, 0.0, 2.0];
% % x=[zeros(1,5) linspace(0.2,1.7,5) 2*ones(1,5)]
% % y=[zeros(1,5) 0.2:0.5:1.7 2*ones(1,5)]
% t=linspace(0.0, 500, numel(x));
% 
% ppx=pchip(t,x);
% ppy=pchip(t,y);
% 
% 
% tq=linspace(min(t), max(t), 300);
% plotStuff(ppx, ppy, x, y, tq)



%%
% close all;
% p=0.03
% ppx=csaps(t,x,p);
% ppy=csaps(t,y,p);
% 
% plotStuff(ppx, ppy, x, y, tq)

% 
% function plotStuff(ppx, ppy, x, y, tq)
%     figure; hold on;
%     xq=ppval(ppx,tq);
%     yq=ppval(ppy,tq);
%     plot(xq,yq); plot(x,y,'o'); xlabel('x'); ylabel('y')
% 
%     figure; subplot (3,1,1); hold on;
%     plot(tq,ppval(ppx,tq)); title('Pos x');
%     subplot (3,1,2); hold on;
%     plot(tq,ppval(fnder(ppx),tq)); title('Vel x')
%     subplot (3,1,3); hold on;
%     plot(tq,ppval(fnder(fnder(ppx)),tq)); title('Accel x')
% 
%     figure; subplot (2,1,1); hold on;
%     plot(tq,ppval(ppy,tq)); title('Pos y');
%     subplot (2,1,2); hold on;
%     plot(tq,ppval(fnder(ppy),tq)); title('Vel y')
% 
%     
% end