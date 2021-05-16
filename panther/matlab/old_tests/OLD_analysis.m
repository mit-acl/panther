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

opti = casadi.Opti('nlp');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTANTS! %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg_pos=3;
deg_yaw=2;
num_seg =5; %number of segments
num_max_of_obst=10; %This is the maximum num of the obstacles 
num_samples_simpson=7;  %This will also be the num_of_layers in the graph yaw search of C++
num_of_yaw_per_layer=40; %This will be used in the graph yaw search of C++
                         %Note that the initial layer will have only one yaw (which is given) 
basis="MINVO"; %MINVO OR B_SPLINE or BEZIER. This is the basis used for collision checking (in position, velocity, accel and jerk space), both in Matlab and in C++
linear_solver_name='mumps'; %mumps [default, comes when installing casadi], ma27, ma57, ma77, ma86, ma97 
print_level=5; %From 0 (no verbose) to 12 (very verbose), default is 5
t0=0; 
tf=10.5;

dim_pos=3;
dim_yaw=1;

offset_vel=0.1;

assert(tf>t0);

all_constraints=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% DEFINITION
%%%%% factors for the cost
c_pos_smooth=            opti.parameter(1,1);
c_yaw_smooth=             opti.parameter(1,1);
c_fov=  opti.parameter(1,1);
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
sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti,'sp'); %spline position.
sy=MyClampedUniformSpline(t0,tf,deg_yaw, dim_yaw, num_seg, opti,'sy'); %spline yaw.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scaling=(tf-t0)/total_time;

v0_scaled=v0/scaling;
a0_scaled=a0/(scaling^2);
ydot0_scaled=ydot0/scaling;


vf_scaled=vf/scaling;
af_scaled=af/(scaling^2);
ydotf_scaled=ydotf/scaling;

v_max_scaled=v_max/scaling;
a_max_scaled=a_max/(scaling^2); 
j_max_scaled=j_max/(scaling^3);

ydot_max_scaled=ydot_max/scaling; %v_max for yaw

%Initial conditions
all_constraints=[all_constraints; ...
                 sp.getPosT(t0)== p0];
                 %sp.getVelT(t0)== v0_scaled;...
                 %sp.getAccelT(t0)== a0_scaled
                 %sy.getPosT(t0)== y0;...
                 %sy.getVelT(t0)== ydot0_scaled];
% opti.subject_to( sp.getPosT(t0)== p0 );
% opti.subject_to( sp.getVelT(t0)== v0_scaled );
% opti.subject_to( sp.getAccelT(t0)== a0_scaled );
% opti.subject_to( sy.getPosT(t0)== y0 );
% opti.subject_to( sy.getVelT(t0)== ydot0_scaled );

%Final conditions
% opti.subject_to( sp.getPosT(tf)== pf );
all_constraints=[all_constraints];
                 %sp.getVelT(tf)== vf_scaled ;...
                 %sp.getAccelT(tf)== af_scaled
                 %sy.getVelT(tf)==ydotf_scaled];% Needed: if not (and if you are minimizing ddyaw), dyaw=cte --> yaw will explode
             
% opti.subject_to( sp.getVelT(tf)== vf_scaled );
% opti.subject_to( sp.getAccelT(tf)== af_scaled );
% opti.subject_to( sy.getVelT(tf)==ydotf_scaled); % Needed: if not (and if you are minimizing ddyaw), dyaw=cte --> yaw will explode

%addDynLimConstraints(all_constraints, sp, sy, basis, v_max_scaled, a_max_scaled, j_max_scaled, ydot_max_scaled);


g=9.81;
%Compute perception cost
dist_im_cost=0;
vel_im_cost=0;
fov_cost=0;

clear i
t_simpson=linspace(t0,tf,num_samples_simpson);
delta_simpson=(t_simpson(2)-t_simpson(1));



u=MX.sym('u',1,1); %it must be defined outside the loop (so that then I can use substitute it regardless of the interval
w_fevar=MX.sym('w_fevar',3,1); %it must be defined outside the loop (so that then I can use substitute it regardless of the interval
w_velfewrtworldvar=MX.sym('w_velfewrtworld',3,1);
yaw= MX.sym('yaw',1,1);  
simpson_index=1;
simpson_coeffs=[];

all_target_isInFOV=[];

s_logged={};

f=0.05;%focal length in meters

for j=1:sp.num_seg
    
    w_t_b{j} = sp.getPosU(u,j);
    accel = sp.getAccelU(u,j);
    qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1

    
    %%%%%
    qabc=qabcFromAccel(accel,g);
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
    w_R_b=toRotMat(q);

    %%%%%
    w_T_b=[w_R_b w_t_b{j}; zeros(1,3) 1];
    w_T_c=w_T_b*b_T_c;
    c_T_b=invPose(b_T_c);
    b_T_w=invPose(w_T_b);
    c_P=c_T_b*b_T_w*[w_fevar;1]; %Position of the feature in the camera frame
    s=f*c_P(1:2)/(c_P(3));  %Note that here we are not using f (the focal length in meters) because it will simply add a constant factor in ||s|| and in ||s_dot||


    % In FOV:
    is_in_FOV1=-cos(thetax_half_FOV_deg*pi/180.0) + (c_P(1:3)'/norm(c_P((1:3))))*[0;0;1];%This has to be >=0
    isInFOV_smooth= mySig(1,is_in_FOV1);

    target_isInFOV{j}=isInFOV_smooth; %This one will be used for the graph search in yaw
    
    %I need to substitute it here because s_dot should consider also the velocity caused by the fact that yaw=yaw(t)
    s=substitute(s, yaw, sy.getPosU(u,j));
    target_isInFOV_substituted_yawcps{j}=substitute(isInFOV_smooth, yaw, sy.getPosU(u,j));
     
    %VELOCITY ON THE IMAGE PLANE
    partial_s_partial_t=jacobian(s,u)*(1/sp.delta_t);% partial_s_partial_u * partial_u_partial_t 
    partial_s_partial_posfeature=jacobian(s,w_fevar); %See Eq. 11.3 of https://www2.math.upenn.edu/~pemantle/110-public/notes11.pdf
    partial_posfeature_partial_t=w_velfewrtworldvar/scaling;
    s_dot=partial_s_partial_t  + partial_s_partial_posfeature*partial_posfeature_partial_t; % partial_s_partial_u * partial_u_partial_t
    s_dot2=s_dot'*s_dot;
    s2=(s'*s);
    
    %Costs (following the convention of "minimize" )
    isInFOV=(target_isInFOV_substituted_yawcps{j});
    fov_cost_j=-isInFOV /(offset_vel+s_dot2);
%   fov_cost_j=-isInFOV + 1500000*(isInFOV)*s_dot2;
%   fov_cost_j=100000*s_dot2/(isInFOV);
%   fov_cost_j=-isInFOV+1e6*(1-isInFOV)*s_dot2;

    fov_cost_j=-isInFOV;
    
    %%%%%%%%%%%%%%%%%%
      
    span_interval=sp.timeSpanOfInterval(j);
    
    u_simpson{j}=getUSimpsonJ(span_interval, sp,j, t_simpson);    
    
    for u_i=u_simpson{j}
                
        simpson_coeff=getSimpsonCoeff(simpson_index,num_samples_simpson);
        fov_cost=fov_cost + (delta_simpson/3.0)*simpson_coeff*substitute( fov_cost_j,[u;w_fevar;w_velfewrtworldvar],[u_i;w_fe{simpson_index};w_velfewrtworld{simpson_index}]); 

       % all_target_isInFOV=[all_target_isInFOV  substitute(target_isInFOV{j},[u;w_fevar],[u_i;w_fe{simpson_index}])];
       % simpson_coeffs=[simpson_coeffs simpson_coeff]; %Store simply for debugging. Should be [1 4 2 4 2 ... 4 2 1]
        s_logged{simpson_index}=substitute( s,[u;w_fevar;w_velfewrtworldvar],[u_i;w_fe{simpson_index};w_velfewrtworld{simpson_index}]);
        
        simpson_index=simpson_index+1;
        
    end
end

%Cost
pos_smooth_cost=sp.getControlCost(); yaw_smooth_cost=sy.getControlCost();

final_pos_cost=(sp.getPosT(tf)- pf)'*(sp.getPosT(tf)- pf);
final_yaw_cost=(sy.getPosT(tf)- yf)^2;

cost=c_pos_smooth*pos_smooth_cost+...
           c_yaw_smooth*yaw_smooth_cost+... 
           c_fov*fov_cost+...
           c_final_pos*final_pos_cost+...
           c_final_yaw*final_yaw_cost;
       
       
cost=c_pos_smooth*pos_smooth_cost+...
           c_final_pos*final_pos_cost;
       
pCps=sp.getCPsAsMatrix();     pCps_par=opti.parameter(size(pCps,1),size(pCps,2));
yCps=sy.getCPsAsMatrix();     yCps_par=opti.parameter(size(yCps,1),size(yCps,2));
       
cost_p=substitute(cost,yCps, yCps_par);
cost_y=substitute(cost,pCps, pCps_par);

all_constraints_p=substitute(all_constraints, yCps, yCps_par);
all_constraints_y=substitute(all_constraints, pCps, pCps_par);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% opti.callback(@(i) stairs(opti.debug.value(total_cost)));


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

for i=1:num_samples_simpson
    x0_feature=[1;1;1];
    v0_feature=0.2; %Set to 0 if you want constant position
    syms t real;
    x_feature=x0_feature+v0_feature*(t-t0)*ones(3,1);
    v_feature=diff(x_feature,t);
    all_w_fe_value{i}=double(subs(x_feature,t,t_simpson(i)));
    all_w_velfewrtworld_value{i}=double(subs(v_feature,t,t_simpson(i)));
end
all_w_fe_value=cell2mat(all_w_fe_value);
all_w_velfewrtworld_value=cell2mat(all_w_velfewrtworld_value);

v_max_value=1.6*ones(3,1);
a_max_value=5*ones(3,1);
j_max_value=50*ones(3,1);
ydot_max_value=1000.0; 
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



tmp1=[   -4.0000   -4.0000   -4.0000    0.7111    3.9997    3.9997    3.9997;
         0         0         0   -1.8953   -0.0131   -0.0131   -0.0131;
         0         0         0    0.6275    0.0052    0.0052    0.0052];
     
tmp2=[   -0.0000   -0.0000    0.2754    2.1131    2.6791    2.6791];


tmp1=[   -4.0000   -4.0000   -4.0000    0.7111   2.5     3.9997    3.9997    3.9997;
         0         0         0   -1.8953      -0.5                -0.0131   -0.0131   -0.0131;
         0         0         0    0.6275    0.2           0.0052    0.0052    0.0052];
     
tmp2=[   -0.0000   -0.0000    0.2754  1.0  2.1131    2.6791    2.6791];


all_params_and_init_guesses= [ {createStruct('thetax_FOV_deg', thetax_FOV_deg, thetax_FOV_deg_value)},...
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
                              {createStruct('c_pos_smooth', c_pos_smooth, 1.0)},...
                              {createStruct('c_yaw_smooth', c_yaw_smooth, 1.0)},...
                              {createStruct('c_fov', c_fov, 10.0)},...
                              {createStruct('c_final_pos', c_final_pos, 1.0)},...
                              {createStruct('c_final_yaw', c_final_yaw, 1.0)}];

names_value={};
vars=[]; names=[];
for i=1:numel(all_params_and_init_guesses)
    names_value{end+1}=all_params_and_init_guesses{i}.name;
    names_value{end+1}=double2DM(all_params_and_init_guesses{i}.value); 
    vars=[vars {all_params_and_init_guesses{i}.param}];
    names=[names {all_params_and_init_guesses{i}.name}];
end

names_value{end+1}='pCps';
names_value{end+1}=tmp1;
names_value{end+1}='yCps';
names_value{end+1}=tmp2;
                          

opts = struct;
opts.expand=true; %When this option is true, it goes WAY faster!
opts.print_time=true;
% opti.solver('ipopt',opts); %{"ipopt.hessian_approximation":"limited-memory"} 
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

results_vars={pos_smooth_cost, yaw_smooth_cost, fov_cost, final_pos_cost, final_yaw_cost};
results_names={'pos_smooth_cost','yaw_smooth_cost','fov_cost','final_pos_cost','final_yaw_cost'};


results_vars_p=substituteCellArray(results_vars,yCps,yCps_par)


% %%%%%%%%%% Create opti structure for POSITION
opti_p=opti.copy; %opti_p.change_problem_type('nlp') %Note that if you don't use opti.copy (and simply do opti_p=opti) --> it's not a deep copy (it's a reference)
opti_p.subject_to(all_constraints_p);
opti_p.minimize(simplify(cost_p));
opti_p.solver('ipopt',opts);
function_opt_p = opti_p.to_function('function_p', [vars   {yCps_par} {pCps}    ],     [results_vars_p   {pCps}    {yCps_par} {cost_p}],...
                                                  [names, 'yCps' 'pCps',    ],         [results_names  'pCps'  'yCps' 'cost']);
                                              
sol=function_opt_p( names_value{:}); %Dummy call to avoid the overhead of the first iteration (not sure why it happens)

%%
% 
% %%%%%%%%%% Create opti structure for YAW 
% opti_y=opti.copy; opti_y.change_problem_type('nlp')
% %Add some options specific to ipopt
% opts.ipopt.print_level=print_level; 
% opts.ipopt.print_frequency_iter=1e10;%1e10 %Big if you don't want to print all the iteratons
% opts.ipopt.linear_solver=linear_solver_name;
% opts.ipopt.print_timing_statistics='no'; %If yes, it prints more info about timing
% opti_y.solver('ipopt',opts);
% opti_y.subject_to(all_constraints_y);
% opti_y.minimize(simplify(cost_y));
% function_opt_y = opti_y.to_function('function_y',  [vars    {pCps_par}    {yCps}],     [results_vars  {pCps_par}    {yCps} {cost_y}],...
%                                                    [names, 'pCps',     'yCps'],        [results_names  'pCps'  'yCps' 'cost']);
                                            
%%%%%%%%%% Create opti structure for YAW/Position 
opti.solver('ipopt',opts); %opti.change_problem_type('nlp')
opti.subject_to(all_constraints);
opti.minimize(simplify(cost));
function_opt = opti.to_function('function_opt',  [vars    {pCps}    {yCps}],         [results_vars  {pCps}    {yCps} {cost}],...
                                                 [names, 'pCps',     'yCps'],        [results_names  'pCps'  'yCps' 'cost']);

                                           
%%
                     
% my_function=my_function.expand();
 %SOLVE THE PROBLEM

sol=function_opt_p( names_value{:}); %Dummy call to avoid the overhead of the first iteration (not sure why it happens)
sol=function_opt_y( names_value{:}); %Dummy call to avoid the overhead of the first iteration (not sure why it happens)
 
all_costs=[];
ms_p=[];
ms_y=[];
for i=1:5
    tic()
    sol=function_opt_p( names_value{:});
    ms_p=[ms_p 1000*toc()];
    names_value{end-2}= sol.pCps;%Update Pos

    % all_costs=[all_costs full(sol.cost)];

    tic()
    sol=function_opt_y( names_value{:});
    ms_y=[ms_y 1000*toc()];
    names_value{end}= sol.yCps; %Update Yaw

    all_costs=[all_costs full(sol.cost)];

end

toc();


full(sol.pCps)
full(sol.yCps)

tic();
sol_single=function_opt( names_value{:} );
ms_single=1000*toc();

figure; hold on;
yline(full(sol_single.cost),'--')

full(sol_single.pCps)
full(sol_single.yCps)

plot(all_costs); xlabel('Iteration'); ylabel('Cost');

figure;
plot(ms_p); hold on
plot(ms_y); ylabel('time (ms)');
yline(ms_single,'--')
legend('opt. pos','opt. yaw','joint')
%%
% %Solve for yaw, pos is fixed:
% all_params_and_init_guesses_y=[{createStruct('pCps', pCps_par, tmp1)},...
%                                {createStruct('yCps', yCps, tmp2)},...
%                                 all_params];
%                             
% %Solve for pos, yaw is fixed:
% all_params_and_init_guesses_p=[{createStruct('pCps', pCps, tmp1)},...
%                                {createStruct('yCps', yCps_par, tmp2)},...
%                                 all_params];

my_function.save('./casadi_generated_files/op.casadi') %Optimization Problam. The file generated is quite big

statistics=get_stats(my_function); %See functions defined below
full(sol.all_pCPs)
full(sol.all_yCPs)

%%
sp_cpoints_var=sp.getCPsAsMatrix();
sy_cpoints_var=sy.getCPsAsMatrix();
sp.updateCPsWithSolution(full(sol.all_pCPs))
sy.updateCPsWithSolution(full(sol.all_yCPs))


all_s_logged=[];
for i=1:num_samples_simpson
    tmp=s_logged{i};
    
    %Substitute all the params
    for ii=1:numel(all_params)
        tmp=substitute(tmp,all_params{ii}.param, all_params{ii}.value);
    end
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

hold on;

width=f*tand(thetay_FOV_deg_value);
height=f*tand(thetax_FOV_deg_value);
x = [-width/2 width/2 width/2 -width/2];
y = [-height/2 -height/2 height/2 height/2];
patch(x,y,'red','EdgeColor','green','FaceColor','none','LineWidth',2)
d=0.0; xlim([min(x)-d,max(x)+d]); axis equal; ylim([min(y)-d,max(y)+d])

%%



function result=getUSimpsonJ(span_interval, sp,j, t_simpson)
    t_init_interval=min(span_interval);   
    t_final_interval=max(span_interval);
    delta_interval=t_final_interval-t_init_interval;
    
    tsf=t_simpson; %tsf is a filtered version of  t_simpson
    tsf=tsf(tsf>=min(t_init_interval));
    if(j==(sp.num_seg))
        tsf=tsf(tsf<=max(t_final_interval));
    else
        tsf=tsf(tsf<max(t_final_interval));
    end
    result=(tsf-t_init_interval)/delta_interval;
end

function all_constraints=addDynLimConstraints(all_constraints, sp, sy, basis, v_max_scaled, a_max_scaled, j_max_scaled, ydot_max_scaled)
%Max vel constraints (position)
for j=1:sp.num_seg
    vel_cps=sp.getCPs_XX_Vel_ofInterval(basis, j);
    dim=size(vel_cps, 1);
    for u=1:size(vel_cps,2)
        for xyz=1:3
            all_constraints=[all_constraints;...
                            vel_cps{u}(xyz) <= v_max_scaled(xyz);...
                            vel_cps{u}(xyz) >= -v_max_scaled(xyz)];
%             opti.subject_to( vel_cps{u}(xyz) <= v_max_scaled(xyz)  )
%             opti.subject_to( vel_cps{u}(xyz) >= -v_max_scaled(xyz) )
        end
    end
end

%Max accel constraints (position)
for j=1:sp.num_seg
    accel_cps=sp.getCPs_XX_Accel_ofInterval(basis, j);
    dim=size(accel_cps, 1);
    for u=1:size(accel_cps,2)
        for xyz=1:3
            all_constraints=[all_constraints;...
                            accel_cps{u}(xyz) <= a_max_scaled(xyz) ;...
                            accel_cps{u}(xyz) >= -a_max_scaled(xyz)];
%             opti.subject_to( accel_cps{u}(xyz) <= a_max_scaled(xyz)  )
%             opti.subject_to( accel_cps{u}(xyz) >= -a_max_scaled(xyz) )
        end
    end
end

%Max jerk constraints (position)
for j=1:sp.num_seg
    jerk_cps=sp.getCPs_MV_Jerk_ofInterval(j);
    dim=size(jerk_cps, 1);
    for u=1:size(jerk_cps,2)
        for xyz=1:3
            all_constraints=[all_constraints;...
                            jerk_cps{u}(xyz) <= j_max_scaled(xyz) ;...
                            jerk_cps{u}(xyz) >= -j_max_scaled(xyz)];
%             opti.subject_to( jerk_cps{u}(xyz) <= j_max_scaled(xyz)  )
%             opti.subject_to( jerk_cps{u}(xyz) >= -j_max_scaled(xyz) )
        end
    end
end

%Max vel constraints (yaw)
for j=1:sy.num_seg
    minvo_vel_cps=sy.getCPs_MV_Vel_ofInterval(j);
    dim=size(minvo_vel_cps, 1);
    for u=1:size(minvo_vel_cps,2)
            all_constraints=[all_constraints;...
                            minvo_vel_cps{u} <= ydot_max_scaled ;...
                            minvo_vel_cps{u}  >= -ydot_max_scaled];
%         opti.subject_to( minvo_vel_cps{u} <= ydot_max_scaled  )
%         opti.subject_to( minvo_vel_cps{u}  >= -ydot_max_scaled )
    end
end

end

%% Functions

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