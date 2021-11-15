% /* ----------------------------------------------------------------------------
%  * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */



close all; clc;clear;

set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
%Let us change now the usual grey background of the matlab figures to white
set(0,'defaultfigurecolor',[1 1 1])

import casadi.*
addpath(genpath('./../../submodules/minvo/src/utils'));
addpath(genpath('./../../submodules/minvo/src/solutions'));
addpath(genpath('./more_utils'));


for pos_is_fixed=[true, false] %you need to run this file twice to produce the necessary casadi files: both with pos_is_fixed=false and pos_is_fixed=true. 

clearvars -except pos_is_fixed
const_p={};
const_y={};
opti = casadi.Opti();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTANTS! %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pos_is_fixed=true;
do_plots=false;
deg_pos=3;
deg_yaw=2;
num_seg =4; %number of segments
num_max_of_obst=10; %This is the maximum num of the obstacles 
num_samples_simpson=14;  %This will also be the num_of_layers in the graph yaw search of C++
num_of_yaw_per_layer=40; %This will be used in the graph yaw search of C++
                         %Note that the initial layer will have only one yaw (which is given) 
basis="MINVO"; %MINVO OR B_SPLINE or BEZIER. This is the basis used for collision checking (in position, velocity, accel and jerk space), both in Matlab and in C++
linear_solver_name='ma27'; %mumps [default, comes when installing casadi], ma27, ma57, ma77, ma86, ma97 
print_level=5; %From 0 (no verbose) to 12 (very verbose), default is 5

t0_n=0.0; 
tf_n=1.0;

dim_pos=3;
dim_yaw=1;

offset_vel=0.3;

assert(tf_n>t0_n);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% DEFINITION
%%%%% factors for the cost
c_pos_smooth= opti.parameter(1,1);
c_yaw_smooth= opti.parameter(1,1);
c_fov=        opti.parameter(1,1);
c_final_pos = opti.parameter(1,1);
c_final_yaw = opti.parameter(1,1);
% c_costs.dist_im_cost=         opti.parameter(1,1);

Ra=opti.parameter(1,1);

thetax_FOV_deg=opti.parameter(1,1);    %total angle of the FOV in the x direction
thetay_FOV_deg=opti.parameter(1,1);    %total angle of the FOV in the y direction

thetax_half_FOV_deg=thetax_FOV_deg/2.0; %half of the angle of the cone
thetax_half_FOV_rad=thetax_half_FOV_deg*pi/180.0;

thetay_half_FOV_deg=thetay_FOV_deg/2.0; %half of the angle of the cone
thetay_half_FOV_rad=thetay_half_FOV_deg*pi/180.0;

%%%%% Transformation matrix camera/body b_T_c
b_T_c=opti.parameter(4,4);

%%%%% Initial and final conditions
p0=opti.parameter(3,1); v0=opti.parameter(3,1); a0=opti.parameter(3,1);
pf=opti.parameter(3,1); vf=opti.parameter(3,1); af=opti.parameter(3,1);
y0=opti.parameter(1,1); ydot0=opti.parameter(1,1); 
yf=opti.parameter(1,1); ydotf=opti.parameter(1,1);

%%%%% Planes
n={}; d={};
for i=1:(num_max_of_obst*num_seg)
    n{i}=opti.parameter(3,1); 
    d{i}=opti.parameter(1,1);
end

%%%% Positions of the feature in the times [t0,t0+XX, ...,tf-XX, tf] (i.e. uniformly distributed and including t0 and tf)
for i=1:num_samples_simpson
    w_fe{i}=opti.parameter(3,1); %Positions of the feature in world frame
    w_velfewrtworld{i}=opti.parameter(3,1);%Velocity of the feature wrt the world frame, expressed in the world frame
end

%%% Min/max x, y ,z

x_lim=opti.parameter(2,1); %[min max]
y_lim=opti.parameter(2,1); %[min max]
z_lim=opti.parameter(2,1); %[min max]

%%% Maximum velocity and acceleration
v_max=opti.parameter(3,1);
a_max=opti.parameter(3,1);
j_max=opti.parameter(3,1);
ydot_max=opti.parameter(1,1);

total_time=opti.parameter(1,1); %This allows a different t0 and tf than the one above 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CREATION OF THE SPLINES! %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sy=MyClampedUniformSpline(t0_n,tf_n,deg_yaw, dim_yaw, num_seg, opti); %spline yaw.

if(pos_is_fixed==true)
    sp=MyClampedUniformSpline(t0_n,tf_n,deg_pos, dim_pos, num_seg, opti, false); %spline position, cPoints are fixed
else
    sp=MyClampedUniformSpline(t0_n,tf_n,deg_pos, dim_pos, num_seg, opti); %spline position.
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_time_n=(tf_n-t0_n);

alpha=total_time/total_time_n;  %Please read explanation_normalization.svg

v0_n=v0*alpha;
a0_n=a0*(alpha^2);
ydot0_n=ydot0*alpha;


vf_n=vf*alpha;
af_n=af*(alpha^2);
ydotf_n=ydotf*alpha;

v_max_n=v_max*alpha;
a_max_n=a_max*(alpha^2);
j_max_n=j_max*(alpha^3);

ydot_max_n=ydot_max*alpha;

%Initial conditions
const_p{end+1}= sp.getPosT(t0_n)== p0 ;
const_p{end+1}= sp.getVelT(t0_n)== v0_n ;
const_p{end+1}= sp.getAccelT(t0_n)== a0_n ;
const_y{end+1}= sy.getPosT(t0_n)== y0 ;
const_y{end+1}= sy.getVelT(t0_n)== ydot0_n ;

%Final conditions
% opti.subject_to( sp.getPosT(tf)== pf );
const_p{end+1}= sp.getVelT(tf_n)== vf_n ;
const_p{end+1}= sp.getAccelT(tf_n)== af_n ;
const_y{end+1}= sy.getVelT(tf_n)==ydotf_n ; % Needed: if not (and if you are minimizing ddyaw), dyaw=cte --> yaw will explode


% epsilon=1;
for j=1:(sp.num_seg)

    %Get the control points of the interval
    Q=sp.getCPs_XX_Pos_ofInterval(basis, j);

    %Plane constraints
    for obst_index=1:num_max_of_obst
      ip = (obst_index-1) * sp.num_seg + j;  % index plane
       
%       init_int=min(sp.timeSpanOfInterval(j)); 
%       end_int=max(sp.timeSpanOfInterval(j)); 
%       %impose that all the vertexes of the obstacle are on one side of the plane
%       vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself
% 
%       for r=1:size(vertexes,2) %vertex=vertexes
%           vertex=vertexes(:,r);
%           opti.subject_to( (-(n{tm(ip)}'*vertex + d{tm(ip)} - epsilon))<= 0);
%       end
      
      %and the control points on the other side
      for kk=1:size(Q,2)
        const_p{end+1}= n{ip}'*Q{kk} + d{ip} <= 0;
      end
    end  
 
%     %Sphere constraints
%     for kk=1:size(Q,2) 
%         tmp=(Q{kk}-p0);
%         const_p{end+1}= (tmp'*tmp)<=(Ra*Ra) ;
%     end
    
    %Min max xyz constraints
    for kk=1:size(Q,2) 
        tmp=Q{kk};
%         const_p{end+1}= x_lim(1)<=tmp(1) ;
%         const_p{end+1}= x_lim(2)>=tmp(1) ;
%         
        const_p{end+1}= y_lim(1)<=tmp(2) ;
        const_p{end+1}= y_lim(2)>=tmp(2) ;

        const_p{end+1}= z_lim(1)<=tmp(3) ; 
        const_p{end+1}= z_lim(2)>=tmp(3) ;
    end
end


[const_p,const_y]=addDynLimConstraints(const_p,const_y, sp, sy, basis, v_max_n, a_max_n, j_max_n, ydot_max_n);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECTIVE! %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% g=9.81;
%Compute perception cost
dist_im_cost=0;
vel_im_cost=0;
fov_cost=0;

clear i
t_simpson_n=linspace(t0_n,tf_n,num_samples_simpson);

delta_simpson_n=total_time_n/num_samples_simpson;
delta_simpson =total_time/num_samples_simpson;


u=MX.sym('u',1,1); %it must be defined outside the loop (so that then I can use substitute it regardless of the interval
w_fevar=MX.sym('w_fevar',3,1); %it must be defined outside the loop (so that then I can use substitute it regardless of the interval
w_velfewrtworldvar=MX.sym('w_velfewrtworld',3,1);
yaw= MX.sym('yaw',1,1);  
simpson_index=1;
simpson_coeffs=[];

all_target_isInFOV=[];

s_logged={};

for j=1:sp.num_seg
    
    w_t_b{j} = sp.getPosU(u,j);
    accel_n = sp.getAccelU(u,j);
    accel=accel_n/(alpha^2);
%     yaw= sy.getPosU(u,j);%%%%%%%%%%%%

    qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1

    
      %%%%% Option A
%     qabc=qabcFromAccel(accel,g);
%     q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
%     w_R_b=toRotMat(q);
%     %%%%% 
    
    
    %%%%% Option B (same as option 1, but this saves ~0.2 seconds of computation (ONLY IF expand=FALSE) (due to the fact that Casadi doesn't simplify, and simply keeps concatenating operations)     
    %if expand=true, option A and B give very similar comp. time
    t=[accel(1); accel(2); accel(3)+9.81];
    norm_t=sqrt(t(1)^2+t(2)^2+t(3)^2);
    
    q_tmp= [qpsi(1)*(norm_t+t(3));
            -qpsi(1)*t(2)+qpsi(4)*t(1);
            qpsi(4)*t(2)+qpsi(1)*t(1);
            qpsi(4)*(norm_t+t(3))];

    w_R_b=(1/(2*norm_t*(norm_t+t(3))))*toRotMat(q_tmp);
    %%%%%%    
   
    
    w_T_b=[w_R_b w_t_b{j}; zeros(1,3) 1];
   
    
    w_T_c=w_T_b*b_T_c;
    c_T_b=invPose(b_T_c);
    b_T_w=invPose(w_T_b);
    
    c_P=c_T_b*b_T_w*[w_fevar;1]; %Position of the feature in the camera frame
    s=c_P(1:2)/(c_P(3));  %Note that here we are not using f (the focal length in meters) because it will simply add a constant factor in ||s|| and in ||s_dot||
    
    %FOV is a cone:  (See more possible versions of this constraint at the end of this file)
    gamma=5;
%     w_beta=w_fevar(1:3)-w_T_c(1:3,4);
%     w_beta=w_beta/norm(w_beta);
%     is_in_FOV1=-cos(thetax_half_FOV_deg*pi/180.0)+w_beta'*w_T_c(1:3,3); %This has to be >=0
    is_in_FOV1=-cos(thetax_half_FOV_deg*pi/180.0) + (c_P(1:3)'/norm(c_P((1:3))))*[0;0;1];
    isInFOV_smooth=  (   1/(1+exp(-gamma*(is_in_FOV1)))  );
   
    

    target_isInFOV{j}=isInFOV_smooth; %This one will be used for the graph search in yaw
    
    %I need to substitute it here because s_dot should consider also the velocity caused by the fact that yaw=yaw(t)
    s=substitute(s, yaw, sy.getPosU(u,j));
    target_isInFOV_substituted_yawcps{j}=substitute(target_isInFOV{j}, yaw, sy.getPosU(u,j));
    
    %Note that s=f(t, posfeature(t))
    %See, e.g.,  Eq. 11.3 of https://www2.math.upenn.edu/~pemantle/110-public/notes11.pdf
    %Hence, s_dot = partial_s_partial_t + partial_s_partial_posfeature*partial_posfeature_partial_t
    
    partial_s_partial_t=jacobian(s,u)*(1/(alpha*sp.delta_t));% partial_s_partial_u * partial_u_partial_t  
    
    partial_s_partial_posfeature=jacobian(s,w_fevar);
    partial_posfeature_partial_t=w_velfewrtworldvar*alpha;
    s_dot=partial_s_partial_t  + partial_s_partial_posfeature*partial_posfeature_partial_t; 
    
    
    s_dot2=s_dot'*s_dot;
    s2=(s'*s);
    
    %Costs (following the convention of "minimize" )
    isInFOV=(target_isInFOV_substituted_yawcps{j});
    fov_cost_j=-isInFOV/(offset_vel+0.00001*s_dot2);
%     fov_cost_j=-isInFOV + 1500000*(isInFOV)*s_dot2;
%     fov_cost_j=100000*s_dot2/(isInFOV);
%     fov_cost_j=-isInFOV+1e6*(1-isInFOV)*s_dot2;
    
      %%%%%%%%%%%%%%%%%%
      
    span_interval_n=sp.timeSpanOfInterval(j);
    t_init_interval_n=min(span_interval_n);   
    t_final_interval_n=max(span_interval_n);
    delta_interval_n=t_final_interval_n-t_init_interval_n;
    
    tsf_n=t_simpson_n; %tsf is a filtered version of  t_simpson_n
    tsf_n=tsf_n(tsf_n>=min(t_init_interval_n));
    if(j==(sp.num_seg))
        tsf_n=tsf_n(tsf_n<=max(t_final_interval_n));
    else
        tsf_n=tsf_n(tsf_n<max(t_final_interval_n));
    end
    u_simpson{j}=(tsf_n-t_init_interval_n)/delta_interval_n;

    
    for u_i=u_simpson{j}
                
        simpson_coeff=getSimpsonCoeff(simpson_index,num_samples_simpson);
        
       
        fov_cost=fov_cost + (delta_simpson/3.0)*simpson_coeff*substitute( fov_cost_j,[u;w_fevar;w_velfewrtworldvar],[u_i;w_fe{simpson_index};w_velfewrtworld{simpson_index}]); 

        all_target_isInFOV=[all_target_isInFOV  substitute(target_isInFOV{j},[u;w_fevar],[u_i;w_fe{simpson_index}])];
        
        simpson_coeffs=[simpson_coeffs simpson_coeff]; %Store simply for debugging. Should be [1 4 2 4 2 ... 4 2 1]
        
        
        s_logged{simpson_index}=substitute( s,[u;w_fevar;w_velfewrtworldvar],[u_i;w_fe{simpson_index};w_velfewrtworld{simpson_index}]);
        
        simpson_index=simpson_index+1;
        
    end
end

%Cost
pos_smooth_cost=sp.getControlCost()/(alpha^(sp.p-1));
yaw_smooth_cost=sy.getControlCost()/(alpha^(sy.p-1));

final_pos_cost=(sp.getPosT(tf_n)- pf)'*(sp.getPosT(tf_n)- pf);
final_yaw_cost=(sy.getPosT(tf_n)- yf)^2;

total_cost=c_pos_smooth*pos_smooth_cost+...
           c_yaw_smooth*yaw_smooth_cost+... 
           c_fov*fov_cost+...
           c_final_pos*final_pos_cost+...
           c_final_yaw*final_yaw_cost;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% opti.callback(@(i) stairs(opti.debug.value(total_cost)));


%%%%%%%%%%%%%%%% Example of how to create a casadi function from the solver and then call it
all_nd=[];
for i=1:(num_max_of_obst*num_seg)
    all_nd=[all_nd [n{i};d{i}]];
end

all_w_fe=[]; %all the positions of the feature, as a matrix. Each column is the position of the feature at each simpson sampling point
all_w_velfewrtworld=[];
for i=1:num_samples_simpson
    all_w_fe=[all_w_fe w_fe{i}];
    all_w_velfewrtworld=[all_w_velfewrtworld w_velfewrtworld{i}];
end

pCPs=sp.getCPsAsMatrix();
yCPs=sy.getCPsAsMatrix();

% my_function = opti.to_function('panther_casadi_function',...
%     [ {pCPs},     {yCPs},     {thetax_FOV_deg}, {thetay_FOV_deg},{Ra},{p0},{v0},{a0},{pf},{vf},{af},{y0}, {ydot0}, {ydotf}, {v_max}, {a_max}, {j_max}, {ydot_max}, {total_time}, {all_nd}, {all_w_fe}, {all_w_velfewrtworld}, {c_pos_smooth}, {c_yaw_smooth}, {c_fov}, {c_final_pos}], {pCPs,yCPs},...
%     {'guess_CPs_Pos','guess_CPs_Yaw', 'thetax_FOV_deg','thetay_FOV_deg','Ra','p0','v0','a0','pf','vf','af','y0', 'ydot0', 'ydotf', 'v_max', 'a_max', 'j_max', 'ydot_max', 'total_time', 'all_nd', 'all_w_fe', 'all_w_velfewrtworld', 'c_pos_smooth', 'c_yaw_smooth', 'c_fov', 'c_final_pos'}, {'pCPs','yCPs'}...
%                                );

for i=1:num_samples_simpson
    x0_feature=[1;1;1];
    v0_feature=0.2; %Set to 0 if you want constant poistion
    syms t real;
    x_feature=x0_feature+v0_feature*(t-t0_n)*ones(3,1);
    v_feature=diff(x_feature,t);
    all_w_fe_value{i}=double(subs(x_feature,t,t_simpson_n(i)));
    all_w_velfewrtworld_value{i}=double(subs(v_feature,t,t_simpson_n(i)));
end
all_w_fe_value=cell2mat(all_w_fe_value);
all_w_velfewrtworld_value=cell2mat(all_w_velfewrtworld_value);

v_max_value=1.6*ones(3,1);
a_max_value=5*ones(3,1);
j_max_value=50*ones(3,1);
ydot_max_value=1.0; 
total_time_value=10.5;
thetax_FOV_deg_value=80;
thetay_FOV_deg_value=80;
Ra_value=12.0;
y0_value=0.0;
yf_value=0.0;
ydot0_value=0.0;
ydotf_value=0.0;
b_T_c_value= [roty(90)*rotz(-90) zeros(3,1); zeros(1,3) 1];

p0_value=[-4;0.0;0.0];
v0_value=[0;0;0];
a0_value=[0;0;0];

pf_value=[4.0;0.0;0.0];
vf_value=[0;0;0];
af_value=[0;0;0];

x_lim_value=[-100;100];
y_lim_value=[-100;100];
z_lim_value=[-100;100];

all_params= [ {createStruct('thetax_FOV_deg', thetax_FOV_deg, thetax_FOV_deg_value)},...
              {createStruct('thetay_FOV_deg', thetay_FOV_deg, thetay_FOV_deg_value)},...
              {createStruct('b_T_c', b_T_c, b_T_c_value)},...
              {createStruct('Ra', Ra, Ra_value)},...
              {createStruct('p0', p0, p0_value)},...
              {createStruct('v0', v0, v0_value)},...
              {createStruct('a0', a0, a0_value)},...
              {createStruct('pf', pf, pf_value)},...
              {createStruct('vf', vf, vf_value)},...
              {createStruct('af', af, af_value)},...
              {createStruct('y0', y0, y0_value)},...
              {createStruct('ydot0', ydot0, ydot0_value)},...
              {createStruct('yf', yf, yf_value)},...
              {createStruct('ydotf', ydotf, ydotf_value)},...
              {createStruct('v_max', v_max, v_max_value)},...
              {createStruct('a_max', a_max, a_max_value)},...
              {createStruct('j_max', j_max, j_max_value)},...
              {createStruct('ydot_max', ydot_max, ydot_max_value)},... 
              {createStruct('x_lim', x_lim, x_lim_value)},...
              {createStruct('y_lim', y_lim, y_lim_value)},...
              {createStruct('z_lim', z_lim, z_lim_value)},...
              {createStruct('total_time', total_time, total_time_value)},...
              {createStruct('all_nd', all_nd, zeros(4,num_max_of_obst*num_seg))},...
              {createStruct('all_w_fe', all_w_fe, all_w_fe_value)},...
              {createStruct('all_w_velfewrtworld', all_w_velfewrtworld, all_w_velfewrtworld_value)},...
              {createStruct('c_pos_smooth', c_pos_smooth, 0.001)},...
              {createStruct('c_yaw_smooth', c_yaw_smooth, 0.0)},...
              {createStruct('c_fov', c_fov, 1.0)},...
              {createStruct('c_final_pos', c_final_pos, 100)},...
              {createStruct('c_final_yaw', c_final_yaw, 0.0)}];


tmp1=[   -4.0000   -4.0000   -4.0000    0.7111    3.9997    3.9997    3.9997;
         0         0         0   -1.8953   -0.0131   -0.0131   -0.0131;
         0         0         0    0.6275    0.0052    0.0052    0.0052];
     
tmp2=[   -0.0000   -0.0000    0.2754    2.1131    2.6791    2.6791];

all_params_and_init_guesses=[{createStruct('pCPs', pCPs, tmp1)},...
                             {createStruct('yCPs', yCPs, tmp2)},...
                             all_params];

vars=[];
names=[];
for i=1:numel(all_params_and_init_guesses)
    vars=[vars {all_params_and_init_guesses{i}.param}];
    names=[names {all_params_and_init_guesses{i}.name}];
end

names_value={};
for i=1:numel(all_params_and_init_guesses)
    names_value{end+1}=all_params_and_init_guesses{i}.name;
    names_value{end+1}=double2DM(all_params_and_init_guesses{i}.value); 
end


opts = struct;
opts.expand=true; %When this option is true, it goes WAY faster!
opts.print_time=true;
opts.ipopt.print_level=print_level; 
opts.ipopt.print_frequency_iter=1e10;%1e10 %Big if you don't want to print all the iteratons
opts.ipopt.linear_solver=linear_solver_name;
opti.solver('ipopt',opts); %{"ipopt.hessian_approximation":"limited-memory"} 
% if(strcmp(linear_solver_name,'ma57'))
%    opts.ipopt.ma57_automatic_scaling='no';
% end
%opts.ipopt.hessian_approximation = 'limited-memory';
% jit_compilation=false; %If true, when I call solve(), Matlab will automatically generate a .c file, convert it to a .mex and then solve the problem using that compiled code
% opts.jit=jit_compilation;
% opts.compiler='clang';
% opts.jit_options.flags='-O0';  %Takes ~15 seconds to generate if O0 (much more if O1,...,O3)
% opts.jit_options.verbose=true;  %See example in shallow_water.cpp
% opts.enable_forward=false; %Seems this option doesn't have effect?
% opts.enable_reverse=false;
% opts.enable_jacobian=false;
% opts.qpsol ='qrqp';  %Other solver
% opti.solver('sqpmethod',opts);

if(pos_is_fixed==true)
    opti.subject_to([const_y]); %The control points are fixed (i.e., parameters)
else
    opti.subject_to([const_p, const_y]);
end

opti.minimize(simplify(total_cost));
results_vars={pCPs,yCPs, pos_smooth_cost, yaw_smooth_cost, fov_cost, final_pos_cost, final_yaw_cost};
results_names={'pCPs','yCPs','pos_smooth_cost','yaw_smooth_cost','fov_cost','final_pos_cost','final_yaw_cost'};

my_function = opti.to_function('my_function', vars, results_vars,...
                                              names, results_names);
if(pos_is_fixed==true)
    my_function.save('./casadi_generated_files/op_fixed_pos.casadi') %Optimization Problam. The file generated is quite big
else
    my_function.save('./casadi_generated_files/op.casadi') %Optimization Problam. The file generated is quite big
end


% opti_tmp=opti.copy;
% opti_tmp.subject_to( sp.getPosT(tf)== pf );
% my_function_tmp = opti_tmp.to_function('my_function_tmp', vars, results_vars,...
%                                                         names, results_names);
% my_function_tmp.save('./casadi_generated_files/op_force_final_pos.casadi') %The file generated is quite big
                                                    
% my_function=my_function.expand();
tic();
sol=my_function( names_value{:});
toc();
if(pos_is_fixed==false)
    statistics=get_stats(my_function); %See functions defined below
end
full(sol.pCPs)
full(sol.yCPs)

%Write param file with the characteristics of the casadi function generated
my_file=fopen('./casadi_generated_files/params_casadi.yaml','w'); %Overwrite content. This will clear its content
fprintf(my_file,'#DO NOT EDIT. Automatically generated by MATLAB\n');
fprintf(my_file,'#If you want to change a parameter, change it in main.m and run the main.m again\n');
fprintf(my_file,'deg_pos: %d\n',deg_pos);
fprintf(my_file,'deg_yaw: %d\n',deg_yaw);
fprintf(my_file,'num_seg: %d\n',num_seg);
fprintf(my_file,'num_max_of_obst: %d\n',num_max_of_obst);
fprintf(my_file,'num_samples_simpson: %d\n',num_samples_simpson);
fprintf(my_file,'num_of_yaw_per_layer: %d\n',num_of_yaw_per_layer); % except in the initial layer, that has only one value
fprintf(my_file,'basis: "%s"\n',basis);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% FUNCTION TO GENERATE VISIBILITY AT EACH POINT  %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

yaw_samples=MX.sym('yaw_samples',1,num_of_yaw_per_layer); 

all_target_isInFOV_for_different_yaw=[];
for yaw_sample_i=yaw_samples
    all_target_isInFOV_for_different_yaw=  [all_target_isInFOV_for_different_yaw;
                                        substitute(all_target_isInFOV, yaw, yaw_sample_i)];  
end
all_target_isInFOV_for_different_yaw=all_target_isInFOV_for_different_yaw'; % Each row will be a layer. Each column will have yaw=constat

pCPs=sp.getCPsAsMatrix();
g = Function('g',{pCPs,  total_time,    all_w_fe,    thetax_FOV_deg,  thetay_FOV_deg,  b_T_c,  yaw_samples},{all_target_isInFOV_for_different_yaw},...
                 {'pCPs', 'total_time', 'all_w_fe', 'thetax_FOV_deg', 'thetay_FOV_deg','b_T_c', 'yaw_samples'},{'result'});
g=g.expand();

g.save('./casadi_generated_files/visibility.casadi') %The file generated is quite big

g_result=g('pCPs',full(sol.pCPs),...
                         'all_w_fe', all_w_fe_value,...
                         'thetax_FOV_deg',thetax_FOV_deg_value,...  
                         'thetay_FOV_deg',thetay_FOV_deg_value,...
                         'b_T_c',b_T_c_value,...
                         'yaw_samples', linspace(0,2*pi,numel(yaw_samples)));
full(g_result.result);


sp_cpoints_var=sp.getCPsAsMatrix();
sy_cpoints_var=sy.getCPsAsMatrix();

sp.updateCPsWithSolution(full(sol.pCPs))
sy.updateCPsWithSolution(full(sol.yCPs))

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING! %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(do_plots)
import casadi.*
alpha_value=convertMX2Matlab(substitute(alpha,total_time, total_time_value));

v_max_n_value= v_max_value*alpha_value;
a_max_n_value= a_max_value*(alpha_value^2);
j_max_n_value= j_max_value*(alpha_value^3);

ydot_max_n_value= ydot_max_value*alpha_value;

sp.plotPosVelAccelJerk(v_max_n_value, a_max_n_value, j_max_n_value)
% sp.plotPosVelAccelJerkFiniteDifferences();
sy.plotPosVelAccelJerk(ydot_max_n_value)
% sy.plotPosVelAccelJerkFiniteDifferences();

sp.plotPos3D();
plotSphere( sp.getPosT(t0_n),0.2,'b'); plotSphere( sp.getPosT(tf_n),0.2,'r'); 

view([280,15]); axis equal
% 
disp("Plotting")



for t_i=t_simpson_n %t0:0.3:tf  
    
    w_t_b = sp.getPosT(t_i);
%     accel = sp.getAccelT(t_i);% sol.value(A{n})*Tau_i;
    
    accel_n = sp.getAccelT(t_i);
    accel = accel_n/(alpha_value^2);
    
    yaw = sy.getPosT(t_i);
%         psiT=sol.value(Psi{n})*Tau_i;

    qabc=qabcFromAccel(accel, 9.81);

    qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1

    w_R_b=toRotMat(q);
    w_T_b=[w_R_b w_t_b; 0 0 0 1];
    plotAxesArrowsT(0.5,w_T_b)
    
    %Plot the FOV cone
    w_T_c=w_T_b*b_T_c_value;
    position=w_T_c(1:3,4);
    direction=w_T_c(1:3,3);
    length=1;
    plotCone(position,direction,thetax_FOV_deg_value,length); 

end

for i=1:num_samples_simpson
    plotSphere(all_w_fe_value(:,i),0.2,'g');
end

grid on; xlabel('x'); ylabel('y'); zlabel('z'); 
camlight
lightangle(gca,45,0)



figure; hold on; import casadi.*

all_s_logged=[];
for i=1:num_samples_simpson
    tmp=s_logged{i};
    
    %Substitute all the params
    for ii=1:numel(all_params)
        tmp=substitute(tmp,all_params{ii}.param, all_params{ii}.value);
    end
    
%   sol.value(tmp)

    %Substitute the solution
    tmp=substitute(tmp, sp_cpoints_var, sp.getCPsAsMatrix());
    tmp=substitute(tmp, sy_cpoints_var, sy.getCPsAsMatrix());
    
    all_s_logged=[all_s_logged convertMX2Matlab(tmp)];
end

% plot(t_simpson, all_s_logged,'o');legend('u','v');
% plot(all_s_logged(1,:), all_s_logged(2,:),'o');
x=all_s_logged(1,:); y=all_s_logged(2,:);
scatter(x,y,60,1:numel(x),'Filled')
title('Image projection of the feature');
xlim([-max(abs(x)),max(abs(x))]); ylim([-max(abs(y)),max(abs(y))]); yline(0.0,'--');xline(0.0,'--')
%  set(gca, 'XAxisLocation', 'origin', 'YAxisLocation', 'origin')
end

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%    FUNCTION TO FIT A SPLINE TO SAMPLES     %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sy_tmp=MyClampedUniformSpline(t0_n,tf_n,deg_yaw, dim_yaw, num_seg, opti);  %creating another object to not mess up with sy

all_yaw=MX.sym('all_yaw',1,numel(t_simpson_n));
cost_function=0;
for i=1:numel(t_simpson_n)
    cost_function = cost_function + (sy_tmp.getPosT(t_simpson_n(i))-all_yaw(i))^2; 
end

lambda1=MX.sym('lambda1',1,1);
lambda2=MX.sym('lambda2',1,1);
lambda3=MX.sym('lambda3',1,1);

%Note that y0 \equiv all_yaw(1)
c1= sy_tmp.getPosT(t0_n) - all_yaw(1); %==0
c2= sy_tmp.getVelT(t0_n) - ydot0_n; %==0
c3= sy_tmp.getVelT(tf_n) - ydotf_n; %==0

lagrangian = cost_function  +  lambda1*c1 + lambda2*c2 + lambda3*c3;

variables=[sy_tmp.getCPsAsMatrix() lambda1 lambda2  lambda3];

kkt_eqs=jacobian(lagrangian, variables)'; %I want kkt=[0 0 ... 0]'

%Obtain A and b
b=-casadi.substitute(kkt_eqs, variables, zeros(size(variables))); %Note the - sign
A=jacobian(kkt_eqs, variables);

solution=A\b;  %Solve the system of equations

f= Function('f', {all_yaw, total_time, ydot0, ydotf }, {solution(1:end-3)}, ...
                 {'all_yaw', 'total_time', 'ydot0', 'ydotf'}, {'result'} );
% f=f.expand();
all_yaw_value=linspace(0,pi,numel(t_simpson_n));


solution=f(all_yaw_value, total_time_value, ydot0_value, ydotf_value);
sy_tmp=MyClampedUniformSpline(t0_n,tf_n,deg_yaw, dim_yaw, num_seg, opti);  %creating another object to not mess up with sy
sy_tmp.updateCPsWithSolution(full(solution)');
sy_tmp.plotPosVelAccelJerk();
subplot(4,1,1); hold on;
plot(t_simpson_n, all_yaw_value, 'o')


f.save('./casadi_generated_files/fit_yaw.casadi') %The file generated is quite big

% solution=convertMX2Matlab(A)\convertMX2Matlab(b);  %Solve the system of equations
% sy.updateCPsWithSolution(solution(1:end-3)');

end

%% Functions


function [const_p,const_y]=addDynLimConstraints(const_p,const_y, sp, sy, basis, v_max_n, a_max_n, j_max_n, ydot_max_n)

    const_p=[const_p sp.getMaxVelConstraints(basis, v_max_n)];      %Max vel constraints (position)
    const_p=[const_p sp.getMaxAccelConstraints(basis, a_max_n)];    %Max accel constraints (position)
    const_p=[const_p sp.getMaxJerkConstraints(basis, j_max_n)];     %Max jerk constraints (position)
    const_y=[const_y sy.getMaxVelConstraints(basis, ydot_max_n)];   %Max vel constraints (yaw)

end

%Taken from https://gist.github.com/jgillis/9d12df1994b6fea08eddd0a3f0b0737f
%See discussion at https://groups.google.com/g/casadi-users/c/1061E0eVAXM/m/dFHpw1CQBgAJ
function [stats] = get_stats(f)
  dep = 0;
  % Loop over the algorithm
  for k=0:f.n_instructions()-1
%      fprintf("Trying with k= %d\n", k)
    if f.instruction_id(k)==casadi.OP_CALL
      fprintf("Found k= %d\n", k)
      d = f.instruction_MX(k).which_function();
      if d.name()=='solver'
        my_file=fopen('./casadi_generated_files/index_instruction.txt','w'); %Overwrite content
        fprintf(my_file,'%d\n',k);
        dep = d;
        break
      end
    end
  end
  if dep==0
    stats = struct;
  else
    stats = dep.stats(1);
  end
end

function a=createStruct(name,param,value)
    a.name=name;
    a.param=param;
    a.value=value;
end

function result=mySig(gamma,x)
    result=(1/(1+exp(-gamma*x)));
end
%%

        % See https://en.wikipedia.org/wiki/Cone#Equation_form:~:text=In%20implicit%20form%2C%20the%20same%20solid%20is%20defined%20by%20the%20inequalities
    %%%%%%%%%%%%%%%%%%%
%     %FOV version 1 (cone):
%     % See https://en.wikipedia.org/wiki/Cone#Equation_form:~:text=In%20implicit%20form%2C%20the%20same%20solid%20is%20defined%20by%20the%20inequalities
%       gamma1=100; gamma2=100;
%     xx{j}=c_P(1); yy{j}=c_P(2); zz{j}=c_P(3);
%     x=c_P(1);y=c_P(2); z=c_P(3);
%     is_in_FOV1{j}=-((x^2+y^2)*(cos(theta_half_FOV_deg*pi/180.0))^2 -(z^2)*(sin(theta_half_FOV_deg*pi/180.0))^2); %(if this quantity is >=0)
%     is_in_FOV2{j}=c_P(3); %(and this quantity is >=0)
%     isInFOV_smooth=  (   1/(1+exp(-gamma1*is_in_FOV1{j}))  )*(   1/(1+exp(-gamma2*is_in_FOV2{j}))  );
%     target_isInFOV{j}=isInFOV_smooth; %/(0.1+f_vel_im{n})
%     
%     %Costs (all of them following the convention of "minimize" )
%     f_vel_im{j}=(s_dot'*s_dot);
%     f_dist_im{j}=(s'*s); %I wanna minimize the integral of this funcion. Approx. using symp. Rule
%     f_vel_isInFOV_im{j}=-(target_isInFOV{j}) /(offset_vel+f_vel_im{j});
%       %End of one possible version
      %%%%%%%%%%%%%%%%%%%
      
      %FOV version 2: (the four planes of the FOV (i.e. a pyramid) + sigmoid to smooth)
      %See also Mathematica notebook
%       gamma=30;
%       tthx=tan(thetax_half_FOV_rad);
%       tthy=tan(thetay_half_FOV_rad);
%       v0h=[0.0, 0.0, 0.0, 1.0]';     %Vertex of the pyramid
%       v1h=[-tthx, -tthy, 1.0, 1.0]'; %Point in the base of the pyramid
%       v2h=[-tthx, tthy, 1.0, 1.0]';  %Point in the base of the pyramid
%       v3h=[tthx, tthy, 1.0, 1.0]';   %Point in the base of the pyramid
%       v4h=[tthx, -tthy, 1.0, 1.0]';  %Point in the base of the pyramid
%       x=c_P(1);y=c_P(2); z=c_P(3);
%       xyzh=[x, y, z, 1.0]';
%       dA21=computeDet([xyzh'; v0h'; v2h'; v1h'] );%This has to be >=0
%       dA32=computeDet([xyzh'; v0h'; v3h'; v2h'] );%This has to be >=0
%       dA43=computeDet([xyzh'; v0h'; v4h'; v3h'] );%This has to be >=0
%       dA14=computeDet([xyzh'; v0h'; v1h'; v4h'] );%This has to be >=0
%       isInFOV_smooth=(mySig(gamma, dA21)*mySig(gamma, dA32)*mySig(gamma, dA43)*mySig(gamma, dA14));
      
      
%%%%%%%%%%%%%%%%%%


    %     %%%%%%%%%%%%%%%%%%%
      %FOV version 3: [Squicular cone]
%     gamma1=0.6; %If too big, the warning "solver:nlp_grad_f failed: NaN detected for output grad_f_x" will appear
%     gamma2=1.0;
%     tthx2=(tan(thetax_half_FOV_rad))^2;
%     tthy2=(tan(thetay_half_FOV_rad))^2;
%     
%     x=c_P(1);y=c_P(2); z=c_P(3);  ss=0.95;
% %%%     is_in_FOV1=-(    (x^2)*(z^2)/tthx2  +  (y^2)*(z^2)/tthy2   -(ss^2)*(x^2)*(y^2) -(z^4)     ); %(if this quantity is >=0). See https://en.wikipedia.org/wiki/Squircle, Note that  a^2=tan^2(theta_half_x)     b^2=tan^2(theta_half_y)   
%     is_in_FOV1= -(    (x^4)/tthx2  +  (y^4)/tthy2   -(z^4)     ); %(if this quantity is >=0). See https://en.wikipedia.org/wiki/Squircle, , Note that  a^2=tan^2(theta_half_x)     b^2=tan^2(theta_half_y)   
%     is_in_FOV2=z; %(and this quantity is >=0) 
%     isInFOV_smooth=  (   1/(1+exp(-gamma1*is_in_FOV1))  )*(   1/(1+exp(-gamma2*is_in_FOV2))  );
%     %%%%%%%%%%%%%%%%%%%



%%
% %% EXAMPLE OF CALLING THE PROBLEM DIRECTLY (WITHOUT FUNCTION)
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL GUESSES  %%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% for i=1:sp.num_cpoints
%     opti.set_initial(sp.CPoints{i}, zeros(3,1)); %Control points
% end
% 
% for i=1:sy.num_cpoints
%     opti.set_initial(sy.CPoints{i}, zeros(1,1)); %Control points
% end
%                     
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%% ASSIGNMENT for the parameters %%%%%%%%%%%%%%%
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% opti.set_value(c_pos_smooth, 0.0);
% opti.set_value(c_yaw_smooth, 0.0);
% opti.set_value(c_fov, 1.0);
% opti.set_value(c_final_pos, 100);
% 
% opti.set_value(theta_FOV_deg, 80);
% 
% opti.set_value(p0, [-4;0;0]);
% opti.set_value(v0, [0;0;0]);
% opti.set_value(a0, [0;0;0]);
% 
% opti.set_value(pf, [4;0;0]);
% opti.set_value(vf, [0;0;0]);
% opti.set_value(af, [0;0;0]);
% 
% opti.set_value(y0, 0);
% opti.set_value(ydot0, 0);
% 
% for i=1:(num_max_of_obst*num_seg)
%     opti.set_value(n{i}, rand(3,1));
%     opti.set_value(d{i}, rand(1,1));
% end
% 
% %Positions of the feature
% for i=1:num_samples_simpson
%     opti.set_value(w_fe{i}, [1 1 1]');
% end
% 
% %Max vel and accel
% opti.set_value(v_max, 1.6*ones(3,1));
% opti.set_value(a_max, 5*ones(3,1));
% 
% opti.set_value(total_time, 10.5);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE!
% sol = opti.solve();
% sp.updateCPsWithSolution(sol)
% sy.updateCPsWithSolution(sol)
% 
% 
% figure; hold on;
% semilogy(sol.stats.iterations.inf_du)
% semilogy(sol.stats.iterations.inf_pr)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%% OLD CODE %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%% In case you wanna generate c code
% opts_nlpsol.x=opti.x;
% opts_nlpsol.f=opti.f;
% opts_nlpsol.g=opti.g;
% opts_nlpsol.p=opti.p;
% solver = nlpsol('my_solver', 'ipopt', opts_nlpsol);
% solver.generate_dependencies('example.c')
%%% Other stuff
% arg = opti.advanced.arg() % get nlpsol args
% solution = solver(lbg=arg['lbg'], ubg=arg['ubg'], lam_g0=arg['lam_g0'], p=arg['p'], x0=arg['x0']);

% disp ("SOLUTION 1")
% disp("Translation")
% for i=1:sp.num_cpoints
%     sol.value(sp.CPoints{tm(i)})'
% %      opti.set_initial(sp.CPoints{tm(i)},sol.value(sp.CPoints{tm(i)})); %Control points
% end



% 
% disp("Yaw")
% for i=1:sy.num_cpoints
%     sol.value(sy.CPoints{tm(i)});
%     opti.set_initial(sy.CPoints{tm(i)},sol.value(sy.CPoints{tm(i)})); %Control points
% end
% 
% % https://stackoverflow.com/questions/43104254/ipopt-solution-is-not-optimal
% opts = struct;
% opts.ipopt.warm_start_init_point = 'yes';
% opts.ipopt.warm_start_bound_push=1e-9;
% opts.ipopt.warm_start_bound_frac=1e-9;
% opts.ipopt.warm_start_slack_bound_frac=1e-9;
% opts.ipopt.warm_start_slack_bound_push=1e-9;
% opts.ipopt.warm_start_mult_bound_push=1e-9;
% opti.solver('ipopt',opts);
% 
% sol = opti.solve();
% 
% sp.updateCPsWithSolution(sol)
% sy.updateCPsWithSolution(sol)
% 
% disp ("SOLUTION 2")
% disp("Translation")
% sp.printCPs();
% disp("Yaw")
% sy.printCPs();



% 
% simpson_index=1;
% for j=1:sp.num_seg 
%      for u_i=u_simpson{j}
%          
%         tmp=s_logged{j};
%         
%         %Substitute all the params
%         for ii=1:numel(all_params)
%             tmp=substitute(tmp,all_params{ii}.param, all_params{ii}.value);
%         end
%         
%         %Substitute the solution
%         tmp=substitute(tmp, sp_cpoints_var, sp.getCPsAsMatrix());
%         tmp=substitute(tmp, sy_cpoints_var, sy.getCPsAsMatrix());
%         
%         %Substitute ui
%         tmp=substitute(tmp, u, u_i);
%         tmp=substitute(tmp, u, u_i);
%          
% %         tmp=substitute(     substitute(s_logged{j},u,u_i)    ,w_fevar, w_fe{simpson_index}  );
% %         
% % %         
% % %         sp_cpoints_var=sp.getCPsAsMatrix();
% % % sy_cpoints_var=sy.getCPsAsMatrix();
% %         
% %         tmp=substitute(tmp, sp_cpoints_var, sp.getCPsAsMatrix());
% %         
% %         tmp=substitute(tmp, sy_cpoints_var, sy.getCPsAsMatrix());
% %         tmp=substitute(tmp,w_velfewrtworldvar, w_velfewrtworld{simpson_index});
% %         
% %         for (ii=1:size(all_params_value,2))
% %          tmp=substitute(tmp,all_params{ii}, all_params_value{ii})
% %         end
% %         
% %         simpson_index=simpson_index+1;
%      end
% end

% for j=1:sp.num_seg 
%     
%     for index_obs=1:num_max_of_obst
%         init_int=min(sp.timeSpanOfInterval(j)); 
%         end_int=max(sp.timeSpanOfInterval(j)); 
%         vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself
% 
%         x=vertexes(1,:);     y=vertexes(2,:);    z=vertexes(3,:);
% 
%         [k1,av1] = convhull(x,y,z);
%         trisurf(k1,x,y,z,'FaceColor','cyan')
%     end
%     
% end



% figure; hold on; 
% subplot(3,1,1);hold on; title('isInFOV()')
% subplot(3,1,2); hold on; title('Cost v')
% subplot(3,1,3); hold on; title('Cost -isInFOV()/(e + v)')
% simpson_index=1;
% for j=1:sp.num_seg
%     for u_i=u_simpson{j}
%         t_i=sp.u2t(u_i,j);
%         subplot(3,1,1);    ylim([0,1]);        
%         stem(t_i, sol.value(substitute(     substitute(target_isInFOV{j},u,u_i)    ,w_fevar, w_fe{simpson_index}  )),'filled','r')
%         subplot(3,1,2);
%         stem(t_i, sol.value(substitute(     substitute(f_vel_im{j},u,u_i)             ,w_fevar, w_fe{simpson_index}  )),'filled','r')
%         subplot(3,1,3);
%         stem(t_i, sol.value(substitute(     substitute(f_vel_isInFOV_im{j},u,u_i)     ,w_fevar, w_fe{simpson_index}  )),'filled','r') 
%         simpson_index=simpson_index+1;
%     end
% end
