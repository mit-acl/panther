
close all; clc;clear;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
%Let us change now the usual grey background of the matlab figures to white
set(0,'defaultfigurecolor',[1 1 1])

import casadi.*
addpath(genpath('./minvo/src/utils'));
addpath(genpath('./more_utils'));
addpath(genpath('./minvo/src/solutions'));

opti = casadi.Opti();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTANTS! %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

num_eval_simpson=15;

t0=0;
tf=10;

deg_pos=3;
dim_pos=3;

deg_yaw=2;
dim_yaw=1;

num_seg =4; %number of segments

num_of_obst=0; %This is actually the maximum num of the obstacles 

beta1=100;
beta2=100;
offset_vel=0.1;

%Transformation matrix camera/body
b_T_c=[roty(90)*rotz(-90) zeros(3,1); zeros(1,3) 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PARAMETERS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% DEFINITION
%%%%% factors for the cost
c_jerk=            opti.parameter(1,1);
c_yaw=             opti.parameter(1,1);
c_vel_isInFOV=  opti.parameter(1,1);
% c_costs.dist_im_cost=         opti.parameter(1,1);


theta_FOV_deg=opti.parameter(1,1);    %total angle of the cone
theta_half_FOV_deg=theta_FOV_deg/2.0; %half of the angle of the cone

%%%%% Initial and final conditions
p0=opti.parameter(3,1); v0=opti.parameter(3,1); a0=opti.parameter(3,1);
pf=opti.parameter(3,1); vf=opti.parameter(3,1); af=opti.parameter(3,1);
y0=opti.parameter(1,1); ydot0=opti.parameter(1,1);

%%%%% Planes
n={}; d={};
for i=1:(num_of_obst*num_seg)
    n{i}=opti.parameter(3,1); 
    d{i}=opti.parameter(1,1);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CREATION OF THE SPLINES! %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti); %spline position.
sy=MyClampedUniformSpline(t0,tf,deg_yaw, dim_yaw, num_seg, opti); %spline yaw.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL GUESSES  %%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=0:(sp.num_cpoints-1)
    opti.set_initial(sp.CPoints{tm(i)}, zeros(3,1)); %Control points
end

for i=0:(sy.num_cpoints-1)
    opti.set_initial(sy.CPoints{tm(i)}, zeros(1,1)); %Control points
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% CONSTRAINTS! %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initial conditions
opti.subject_to( sp.getPosT(t0)== p0 );
opti.subject_to( sp.getVelT(t0)== v0 );
opti.subject_to( sp.getAccelT(t0)== a0 );
opti.subject_to( sy.getPosT(t0)== y0 );
opti.subject_to( sy.getVelT(t0)== ydot0 );

%Final conditions
opti.subject_to( sp.getPosT(tf)== pf );
opti.subject_to( sp.getVelT(tf)== vf );
opti.subject_to( sp.getAccelT(tf)== af );

%Plane constraints
epsilon=1;

for j=0:(sp.num_seg-1)
    for obst_index=0:(num_of_obst-1)
        
      ip = obst_index * sp.num_seg + j;  % index plane
       
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
      Q_Mv=sp.getMINVOCPsofInterval(j);
      for kk=1:size(Q_Mv,2)
        opti.subject_to( n{tm(ip)}'*Q_Mv{kk} + d{tm(ip)} + epsilon <= 0);
      end
    end   
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% OBJECTIVE! %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Cost
jerk_cost=sp.getControlCost();
yaw_cost=sy.getControlCost();

g=9.81;
%Compute perception cost
dist_im_cost=0;
vel_im_cost=0;
vel_isInFOV_im_cost=0;

clear i
t_simpson=linspace(t0,tf,num_eval_simpson);
delta_simpson=(t_simpson(2)-t_simpson(1));



u=opti.variable(1,1); %it must be defined outside the loop (so that then I can use substitute(...,u,..) regardless of the interval

simpson_index=1;
simpson_coeffs=[];

for j=0:(sp.num_seg-1)
    
%     syms u real
    w_t_b{tm(j)} = sp.evalDerivativeU(0,u,j); % sp.getPosU(u,j)
    accel = sp.getAccelU(u,j);
    yaw= sy.getPosU(u,j);
    qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1

    
      %%%%% Option 1
%     qabc=qabcFromAccel(accel,g);
%     q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
%     w_R_b=toRotMat(q);
%     %%%%% 
    
    
    %%%%% Option 2 (same as option 1, but this saves ~0.2 seconds of computation (ONLY IF expand=FALSE) (due to the fact that Casadi doesn't simplify, and simply keeps concatenating operations)     
    %if expand=true, option 1 and two give very similar comp. time
    t=[accel(1); accel(2); accel(3)+9.81];
    norm_t=sqrt(t(1)^2+t(2)^2+t(3)^2);
    
    q_tmp= [qpsi(1)*(norm_t+t(3));
            -qpsi(1)*t(2)+qpsi(4)*t(1);
            qpsi(4)*t(2)+qpsi(1)*t(1);
            qpsi(4)*(norm_t+t(3))];

    w_R_b=(1/(2*norm_t*(norm_t+t(3))))*toRotMat(q_tmp);
    %%%%%%
    
    
    w_T_b=[w_R_b w_t_b{tm(j)}; zeros(1,3) 1];
    w_fe=[1 1 1 1]';   %feature in world frame
   
    
    w_T_c=w_T_b*b_T_c;
    c_T_b=invPose(b_T_c);
    b_T_w=invPose(w_T_b);
    
    c_P=c_T_b*b_T_w*w_fe; %Position of the feature in the camera frame
    s=c_P(1:2)/(c_P(3));  
  
    s_dot=jacobian(s,u);
    
    % See https://en.wikipedia.org/wiki/Cone#Equation_form:~:text=In%20implicit%20form%2C%20the%20same%20solid%20is%20defined%20by%20the%20inequalities

      %%One possible version:
%     xx{tm(j)}=c_P(1); yy{tm(j)}=c_P(2); zz{tm(j)}=c_P(3);
%     x=c_P(1);y=c_P(2); z=c_P(3);
%     is_in_FOV1{tm(j)}=-((x^2+y^2)*(cosd(theta_half_FOV_deg))^2 -(z^2)*(sind(theta_half_FOV_deg))^2); %(if this quantity is >=0)
%     is_in_FOV2{tm(j)}=c_P(3); %(and this quantity is >=0)
%     isInFOV_smooth=  (   1/(1+exp(-beta1*is_in_FOV1{tm(j)}))  )*(   1/(1+exp(-beta2*is_in_FOV2{tm(j)}))  );
%     target_isInFOV_im{tm(j)}=isInFOV_smooth; %/(0.1+f_vel_im{n})
%     
%     %Costs (all of them following the convention of "minimize" )
%     f_vel_im{tm(j)}=(s_dot'*s_dot);
%     f_dist_im{tm(j)}=(s'*s); %I wanna minimize the integral of this funcion. Approx. using symp. Rule
%     f_vel_isInFOV_im{tm(j)}=-(target_isInFOV_im{tm(j)}) /(offset_vel+f_vel_im{tm(j)});
      %%End of one possible version

    %Simpler version:
    w_beta=w_fe(1:3)-w_T_c(1:3,4);
    w_beta=w_beta/norm(w_beta);
    is_in_FOV1=-cos(theta_half_FOV_deg*3.14159/180.0)+w_beta'*w_T_c(1:3,3); %This has to be >=0
    isInFOV_smooth=  (   1/(1+exp(-beta1*is_in_FOV1))  );
    target_isInFOV_im{tm(j)}=isInFOV_smooth;

    %Costs (all of them following the convention of "minimize" )
    f_vel_im{tm(j)}=(s_dot'*s_dot);
    f_dist_im{tm(j)}=(s'*s); %I wanna minimize the integral of this funcion. Approx. using Sympson's Rule
    f_vel_isInFOV_im{tm(j)}=-(target_isInFOV_im{tm(j)}) /(offset_vel+f_vel_im{tm(j)});
    %End of simpler version
    
    span_interval=sp.timeSpanOfInterval(j);
    t_init_interval=min(span_interval);   
    t_final_interval=max(span_interval);
    delta_interval=t_final_interval-t_init_interval;
    
    tsf=t_simpson; %tsf \eqiv t_simpson_filtered
    tsf=tsf(tsf>=min(t_init_interval));
    if(j==(sp.num_seg-1))
        tsf=tsf(tsf<=max(t_final_interval));
    else
        tsf=tsf(tsf<max(t_final_interval));
    end
    
    u_simpson{tm(j)}=(tsf-t_init_interval)/delta_interval;

    fun1 = Function('fun1',{u},{f_dist_im{tm(j)}});

    for u_i=u_simpson{tm(j)}
        
        simpson_coeff=getSimpsonCoeff(simpson_index,num_eval_simpson);

        dist_im_cost=dist_im_cost               + (delta_simpson/3.0)*simpson_coeff*substitute(f_dist_im{tm(j)},u,u_i);
        vel_im_cost=vel_im_cost                 + (delta_simpson/3.0)*simpson_coeff*substitute(f_vel_im{tm(j)},u,u_i);
        vel_isInFOV_im_cost=vel_isInFOV_im_cost + (delta_simpson/3.0)*simpson_coeff*substitute(f_vel_isInFOV_im{tm(j)},u,u_i);
        
        simpson_coeffs=[simpson_coeffs simpson_coeff]; %Store simply for debugging. Should be [1 4 2 4 2 ... 4 2 1]
        
        simpson_index=simpson_index+1;
        
    end
end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

jit_compilation=false; %If true, when I call solve(), Matlab will automatically generate a .c file, convert it to a .mex and then solve the problem using that compiled code

c=c_jerk;
f=jerk_cost;
total_cost=        c_jerk*           jerk_cost+...
                    c_yaw*            yaw_cost+... % c_costs.dist_im_cost*        dist_im_cost+...
           c_vel_isInFOV* vel_isInFOV_im_cost;
       
total_cost=simplify(total_cost); 

% opti.callback(@(i) stairs(opti.debug.value(total_cost)));

opti.minimize(total_cost);
opts = struct;
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.jit=jit_compilation;

opts.compiler='clang';
opts.jit_options.flags='-O0';  %Takes ~15 seconds to generate if O0 (much more if O1,...,O3)
opts.jit_options.verbose=true;  %See example in shallow_water.cpp
opts.expand=true; %When this option is true, it goes WAY faster!
% opts.enable_forward=false; %Seems this option doesn't have effect?
% opts.enable_reverse=false;
% opts.enable_jacobian=false;

opti.solver('ipopt',opts); %{"ipopt.hessian_approximation":"limited-memory"} 


% opts.qpsol ='qrqp';  %Other solver
% opti.solver('sqpmethod',opts);

%%%%%%%%%%%%%%%% Example of how to create a casadi function from the solver and then call it
all_nd=[];
for i=1:(num_of_obst*num_seg)
    all_nd=[all_nd [n{i};d{i}]];
end

all_nd_tmp=[];
for i=1:(num_of_obst*num_seg)
    all_nd_tmp=[all_nd_tmp rand(4,1)];
end

all_pCPs=sp.getCPsAsMatrix();
all_yCPs=sy.getCPsAsMatrix();

my_function = opti.to_function('mader_casadi_function',[{theta_FOV_deg},{p0},{v0},{a0},{pf},{vf},{af},{y0}, {ydot0}, {all_nd}, {c_jerk}, {c_yaw}, {c_vel_isInFOV}],  {all_pCPs,all_yCPs},...
                                                        {'theta_FOV_deg','p0','v0','a0','pf','vf','af','y0', 'ydot0', 'all_nd', 'c_jerk', 'c_yaw', 'c_vel_isInFOV'}, {'all_pCPs','all_yCPs'});

sol_tmp=my_function('theta_FOV_deg',80, ...  
                      'p0',  [-4;0;0], ... 
                      'v0',  [0;0;0], ... 
                      'a0',  [0;0;0], ... 
                      'pf',  [4;0;0], ... 
                      'vf',  [0;0;0], ... 
                      'af',  [0;0;0], ... 
                      'y0',  0.0 ,...
                      'ydot0',  0 ,...
                      'c_jerk', 0.0,...
                      'c_yaw', 0.0,...
                      'c_vel_isInFOV', 1.0,...
                      'all_nd',  all_nd_tmp);
full(sol_tmp.all_pCPs)
full(sol_tmp.all_yCPs)

%%
                    

%%%%%%%%%%%%%%%%%%%%%%%%%%% ASSIGNMENT for the parameters
opti.set_value(c_jerk, 0.0);
opti.set_value(c_yaw, 0.0);
opti.set_value(c_vel_isInFOV, 1.0);

opti.set_value(theta_FOV_deg, 80);

opti.set_value(p0, [-4;0;0]);
opti.set_value(v0, [0;0;0]);
opti.set_value(a0, [0;0;0]);

opti.set_value(pf, [4;0;0]);
opti.set_value(vf, [0;0;0]);
opti.set_value(af, [0;0;0]);

opti.set_value(y0, 0);
opti.set_value(ydot0, 0);

for i=1:(num_of_obst*num_seg)
    opti.set_value(n{i}, rand(3,1));
    opti.set_value(d{i}, rand(1,1));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLVE!
sol = opti.solve();
sp.updateCPsWithSolution(sol)
sy.updateCPsWithSolution(sol)


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
% for i=0:(sp.num_cpoints-1)
%     sol.value(sp.CPoints{tm(i)})'
% %      opti.set_initial(sp.CPoints{tm(i)},sol.value(sp.CPoints{tm(i)})); %Control points
% end



% 
% disp("Yaw")
% for i=0:(sy.num_cpoints-1)
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


figure; hold on;
semilogy(sol.stats.iterations.inf_du)
semilogy(sol.stats.iterations.inf_pr)

%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING! %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sp.plotPosVelAccelJerk()
% sp.plotPosVelAccelJerkFiniteDifferences();
sy.plotPosVelAccelJerk()
% sy.plotPosVelAccelJerkFiniteDifferences();

sp.plotPos3D();
plotSphere(opti.value(p0),0.2,'b'); plotSphere(opti.value(pf),0.2,'r'); plotSphere(w_fe(1:3),0.2,'g');

view([280,15]); axis equal
% 
disp("Plotting")
for t_i=t_simpson %t0:0.3:tf  
    
    w_t_b = sp.getPosT(t_i);
    accel = sp.getAccelT(t_i);% sol.value(A{n})*Tau_i;
    yaw = sy.getPosT(t_i);
%         psiT=sol.value(Psi{n})*Tau_i;

    qabc=qabcFromAccel(accel, 9.81);

    qpsi=[cos(yaw/2), 0, 0, sin(yaw/2)]; %Note that qpsi has norm=1
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1

    w_R_b=toRotMat(q);
    w_T_b=[w_R_b w_t_b; 0 0 0 1];
    plotAxesArrowsT(0.5,w_T_b)
    
    %Plot the FOV cone
    w_T_c=w_T_b*b_T_c;
    position=w_T_c(1:3,4);
    direction=w_T_c(1:3,3);
    length=1;
    plotCone(position,direction,opti.value(theta_FOV_deg),length);

end

grid on; xlabel('x'); ylabel('y'); zlabel('z'); 
camlight
lightangle(gca,45,0)

for j=0:(sp.num_seg-1) % i  is the interval (\equiv segment)
    
    for index_obs=0:(num_of_obst-1)
        init_int=min(sp.timeSpanOfInterval(j)); 
        end_int=max(sp.timeSpanOfInterval(j)); 
        vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself

        x=vertexes(1,:);     y=vertexes(2,:);    z=vertexes(3,:);

        [k1,av1] = convhull(x,y,z);
        trisurf(k1,x,y,z,'FaceColor','cyan')
    end
    
end


figure; hold on; 
subplot(3,1,1);hold on; title('isInFOV()')
subplot(3,1,2); hold on; title('Cost v')
subplot(3,1,3); hold on; title('Cost -isInFOV()/(e + v)')



for j=0:(sp.num_seg-1)
    for u_i=u_simpson{tm(j)}
        t_i=sp.u2t(u_i,j);
        subplot(3,1,1);    ylim([0,1]);        
        stem(t_i, sol.value(substitute(target_isInFOV_im{tm(j)},u,u_i)),'filled','r')
        subplot(3,1,2);
        stem(t_i, sol.value(substitute(f_vel_im{tm(j)},u,u_i)),'filled','r')
        subplot(3,1,3);
        stem(t_i, sol.value(substitute(f_vel_isInFOV_im{tm(j)},u,u_i)),'filled','r') 
    end
end
%%
% clc
% p=randi([1 20],1,1);
% order=randi([1 20],1,1);
% syms tmp real;
% Tmp=(tmp.^[p:-1:0])';
% quiero=diff(Tmp,tmp,order)'
% % diffTmp=([polyder(ones(1,p+1)) zeros(1,p-order+1)].*[(tmp.^[p-order:-1:0]) zeros(1,order)])';
% 
% 
% pto0=p:-1:0;
% if(order>p)
%     tengo=zeros(1,p+1);
% else
%     tengo=(factorial(pto0)./factorial(max(pto0-order,0))).*[(tmp.^[p-order:-1:0]) zeros(1,order)];
% end
% 
% % var_exponents
% % tengo
% assert(nnz(quiero-tengo)==0)

% factorial(p)/factorial(p-order)

% (diag([p:-1:+1])^order)

% if(order>=p)
%     tengo=sort([nonzeros(diag([1:p],-1)^order); zeros(order,1)] , 'descend').*[(tmp.^[(p-order):-1:0])' ; zeros(order,1)]
% else
%     tengo=sort([nonzeros(diag([1:p],-1)^order); zeros(order,1)] , 'descend').*[(tmp.^[(p-order):-1:0])' ; zeros(order,1)]
% end
% 
% quiero-tengo

% %This generates sampling without including the initial and final points of
% %the interval
% num_att_cons_per_interval=4; %Number of attitude evaluations (Simpson rule)
% delta_u_bet_att_cons=1/(num_att_cons_per_interval+1);
% u_constrained= linspace(delta_u_bet_att_cons,1-delta_u_bet_att_cons,num_att_cons_per_interval);
