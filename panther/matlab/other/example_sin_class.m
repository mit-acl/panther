
close all; clc;clear;
set(0,'DefaultFigureWindowStyle','docked') %'normal' 'docked'
set(0,'defaulttextInterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex'); set(groot, 'defaultLegendInterpreter','latex');
%Let us change now the usual grey background of the matlab figures to white
set(0,'defaultfigurecolor',[1 1 1])

import casadi.*
addpath(genpath('./utils'));

t_init=0;
t_final=4;

deg_pol_=3;
num_pol_ =5;

p0=[-4;0;0];
v0=[0;0;0];
a0=[0;0;0];

pf=[4;0;0];
vf=[0;0;0];
af=[0;0;0];

num_of_obst_=1;

p_ = deg_pol_;
M_ = num_pol_ + 2 * p_;
N_ = M_ - p_ - 1;
num_of_segments_ = (M_ - 2 * p_);  % this is the same as num_pol_

deltaT_ = (t_final - t_init) / (1.0 * (M_ - 2 * p_ - 1 + 1));

t_final = t_init + (1.0 * (M_ - 2 * p_ - 1 + 1)) * deltaT_;

t_init_ = t_init;
t_final_ = t_final;

%%

opti = casadi.Opti();




A_pos_bs_seg0 =[  -1.0000,    3.0000,   -3.0000,    1.0000;
                    1.7500,   -4.5000,    3.0000,         0;
                   -0.9167,    1.5000,         0,         0;
                    0.1667,         0,         0,         0;];

A_pos_bs_seg1 = [   -0.2500,    0.7500,   -0.7500,    0.2500;
                    0.5833,   -1.2500,    0.2500,    0.5833;
                   -0.5000,    0.5000,    0.5000,    0.1667;
                    0.1667,         0,         0,         0;];

A_pos_bs_rest =[  -0.1667,    0.5000,   -0.5000,    0.1667,
                    0.5000,   -1.0000,         0,    0.6667,
                   -0.5000,    0.5000,    0.5000,    0.1667,
                    0.1667,         0,         0,         0;];

A_pos_bs_seg_last2 =[  -0.1667,    0.5000,   -0.5000,    0.1667;
                        0.5000,   -1.0000,    0.0000,    0.6667;
                       -0.5833,    0.5000,    0.5000,    0.1667;
                        0.2500,         0,         0,         0;];

A_pos_bs_seg_last =[   -0.1667,    0.5000,   -0.5000,   0.1667;
                        0.9167,   -1.2500,   -0.2500,   0.5833;
                       -1.7500,    0.7500,    0.7500,   0.2500;
                        1.0000,         0,         0,        0;];

                    


A_pos_bs_{tm(0)}= A_pos_bs_seg0;
A_pos_bs_{tm(1)}= A_pos_bs_seg1;
for i=0:(num_pol_ - 5)
   A_pos_bs_{end+1}= A_pos_bs_rest;
end
A_pos_bs_{end+1}=A_pos_bs_seg_last2;
A_pos_bs_{end+1}=A_pos_bs_seg_last;


M_pos_bs2mv_seg0 =[
 1.1023313949144333268037598827505,   0.34205724556666972091534262290224, -0.092730934245582874453361910127569, -0.032032766697130621302846975595457;
-0.049683556253749178166501110354147,   0.65780347324677179710050722860615,   0.53053863760186903419935333658941,   0.21181027098212013015654520131648;
-0.047309044211162346038612724896666,  0.015594436894155586093013710069499,    0.5051827557159349613158383363043,   0.63650059656260427054519368539331;
-0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,   0.18372189915240558222286892942066];

M_pos_bs2mv_seg1 =[
0.27558284872860833170093997068761,  0.085514311391667430228835655725561, -0.023182733561395718613340477531892, -0.0080081916742826553257117438988644;
 0.6099042761975865811763242163579,   0.63806904207840509091198555324809,   0.29959938009132258684985572472215,    0.12252106674808682651445224109921;
0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594;
-0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066];

M_pos_bs2mv_rest =[
0.18372189915240555446729331379174,  0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988;
0.70176522577378919187651717948029,   0.66657381254229419731416328431806,   0.29187180223752384744528853843804,    0.11985166952332582113172065874096;
0.11985166952332682033244282138185,   0.29187180223752445806795208227413,   0.66657381254229419731416328431806,    0.70176522577378930289881964199594;
-0.0053387944495217444854096022766043, -0.015455155707597083292181849856206,  0.057009540927778303009976212933907,    0.18372189915240558222286892942066];


M_pos_bs2mv_seg_last2 =[
0.18372189915240569324517139193631,  0.057009540927778309948870116841135, -0.015455155707597145742226985021261, -0.0053387944495218164764338553140988;
0.70176522577378952494342456702725,   0.66657381254229453038107067186502,   0.29187180223752412500104469472717,    0.11985166952332593215402312125661;
 0.1225210667480875342816304396365,   0.29959938009132280889446064975346,   0.63806904207840497988968309073243,    0.60990427619758624810941682881094;
-0.0080081916742826154270717964323012, -0.023182733561395621468825822830695,  0.085514311391667444106623463540018,    0.27558284872860833170093997068761];

M_pos_bs2mv_seg_last =[
0.18372189915240555446729331379174, 0.057009540927778309948870116841135, -0.015455155707597117986651369392348, -0.0053387944495218164764338553140988;
0.63650059656260415952289122287766,   0.5051827557159349613158383363043,  0.015594436894155294659469745965907,  -0.047309044211162887272337229660479;
0.21181027098212068526805751389475,  0.53053863760186914522165579910506,   0.65780347324677146403359984105919,  -0.049683556253749622255710960416764;
-0.032032766697130461708287185729205, -0.09273093424558248587530329132278,   0.34205724556666977642649385416007,     1.1023313949144333268037598827505];


M_pos_bs2mv_{tm(0)}= M_pos_bs2mv_seg0;
M_pos_bs2mv_{tm(1)}= M_pos_bs2mv_seg1;
for i=0:(num_pol_ - 5)
   M_pos_bs2mv_{end+1}= M_pos_bs2mv_rest;
end
M_pos_bs2mv_{end+1}=M_pos_bs2mv_seg_last2;
M_pos_bs2mv_{end+1}=M_pos_bs2mv_seg_last;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%This comes from the initial guess, set as decision variables for now
for i=1:(num_of_obst_*num_of_segments_)
    n_{i}=opti.variable(3,1); 
    d_{i}=opti.variable(1,1);
end

knots=[];
for i=0:p_
    knots=[knots t_init_];
end

for i=(p_ + 1):(M_ - p_ - 1)
    knots=[knots knots(tm(i - 1)) + deltaT_];
end

for i=(M_ - p_):M_
    knots=[knots t_final_];
end

knots_=knots;

t1 = knots_(tm(1));
t2 = knots_(tm(2));
tpP1 = knots_(tm(p_ + 1));
t1PpP1 = knots_(tm(1 + p_ + 1));

tN = knots_(tm(N_));
tNm1 = knots_(tm(N_ - 1));
tNPp = knots_(tm(N_ + p_));
tNm1Pp = knots_(tm(N_ - 1 + p_));


q0_ = p0;
q1_ = p0 + (-t1 + tpP1) * v0 / p_;
q2_ = (p_ * p_ * q1_ - (t1PpP1 - t2) * (a0 * (t2 - tpP1) + v0) - p_ * (q1_ + (-t1PpP1 + t2) * v0)) / ((-1 + p_) * p_);

qN_ = pf;
qNm1_ = pf + ((tN - tNPp) * vf) / p_;
qNm2_ = (p_ * p_ * qNm1_ - (tNm1 - tNm1Pp) * (af * (-tN + tNm1Pp) + vf) - p_ * (qNm1_ + (-tNm1 + tNm1Pp) * vf)) / ((-1 + p_) * p_);


%%Add Constraints

for i=0:N_ 
    q_exp_{tm(i)}=opti.variable(3,1); %Control points
end

opti.subject_to( q_exp_{tm(0)}== q0_ );
opti.subject_to( q_exp_{tm(1)}== q1_ );
opti.subject_to( q_exp_{tm(2)}== q2_ );


%Final velocity and acceleration are zero \equiv q_exp_[N_]=q_exp_[N_-1]=q_exp_[N_-2]
opti.subject_to( q_exp_{tm(N_)}== q_exp_{tm(N_-1)} );
opti.subject_to( q_exp_{tm(N_-1)}== q_exp_{tm(N_-2)} );


%%%%%%PLANE CONSTRAINTS

epsilon=1;

for i=0:(N_-3) % i  is the interval (\equiv segment)
    
    Qbs=[q_exp_{tm(i)} q_exp_{tm(i+1)} q_exp_{tm(i+2)} q_exp_{tm(i+3)}];
    Qmv=Qbs*M_pos_bs2mv_{tm(i)}; %transformPosBSpline2otherBasis(Qbs,i); TODO

    init_int=knots(tm(p_+i));
    end_int=knots(tm(p_+i+1));

    for obst_index=0:(num_of_obst_-1)
        
       ip = obst_index * num_of_segments_ + i;  % index plane
       
      %impose that all the vertexes of the obstacle are on one side of the plane
      vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself

      for r=1:size(vertexes,2) %vertex=vertexes
          vertex=vertexes(:,r);
          opti.subject_to( (-(n_{tm(ip)}'*vertex + d_{tm(ip)} - epsilon))<= 0);
      end
      
      %and the control points on the other side
      for u=0:3
      
        q_ipu = Qmv(:,tm(u));
        opti.subject_to( n_{tm(ip)}'*q_ipu + d_{tm(ip)} + epsilon <= 0);
      
      end
        
    end   
    
end

%TODO: add here the sphere, velocity and acceleration constraints


%Adding objective:

cost=0;

for i=0:(num_of_segments_-1)   
    
    Q=[q_exp_{tm(i)} q_exp_{tm(i+1)} q_exp_{tm(i+2)} q_exp_{tm(i+3)}];
    jerk=Q*A_pos_bs_{tm(i)}*[6 0 0 0]';
    cost=cost + jerk'*jerk;
    
end



%For now, force final poition
opti.subject_to( q_exp_{tm(N_)}== pf );

weight_=30;
% cost= cost  + weight_*(q_exp_{tm(N_)}-pf)'*(q_exp_{tm(N_)}-pf);


jit_compilation=false;
%Solve first without perception cost (to get an initial guess)
opti.minimize( cost );
opti.solver('ipopt',struct('jit',jit_compilation));
sol = opti.solve();

figure; 
subplot(4,1,1);hold on; title('p')
subplot(4,1,2); hold on; title('v')
subplot(4,1,3); hold on; title('a')
subplot(4,1,4); hold on; title('j')
for j=0:(num_of_segments_-1)  

    Qj=sol.value([q_exp_{tm(j)} q_exp_{tm(j+1)} q_exp_{tm(j+2)} q_exp_{tm(j+3)}]);
    
    syms t_m real
    
    init_int=knots(tm(p_+j));
    end_int=knots(tm(p_+j+1));
    
    
    uj=(t_m-init_int)/(end_int - init_int   );
   
    Uj=[uj^3 uj^2 uj 1]';
    dUj=[3*uj^2 2*uj 1 0]';
    ddUj=[6*uj   2  0 0]';
    dddUj=[6   0  0 0]';
    
    pos=Qj*A_pos_bs_{tm(j)}*Uj;
    vel= (1/(deltaT_))   *Qj*A_pos_bs_{tm(j)}  * dUj;
    accel= (1/(deltaT_^2)) *Qj*A_pos_bs_{tm(j)}  * ddUj;
    jerk= (1/(deltaT_^3)) *Qj*A_pos_bs_{tm(j)}  * dddUj;
    

    interval=[init_int, end_int];
        
    subplot(4,1,1);
    fplot(pos, interval)
    subplot(4,1,2);
    fplot(vel, interval)
    subplot(4,1,3);
    fplot(accel, interval)
    subplot(4,1,4);
    fplot(jerk, interval)

end


figure; hold on;
plotSphere(p0,0.05,'r'); plotSphere(pf,0.05,'b'); % plotSphere(w_fe(1:3),0.05,'g');


for j=0:(num_of_segments_-1)  

    Qj=sol.value([q_exp_{tm(j)} q_exp_{tm(j+1)} q_exp_{tm(j+2)} q_exp_{tm(j+3)}]);
    
    syms t_m real
    
    init_int=knots(tm(p_+j));
    end_int=knots(tm(p_+j+1));
    
    uj=(t_m-init_int)/(end_int - init_int   );
   
    Uj=[uj^3 uj^2 uj 1]';
    dUj=[3*uj^2 2*uj 1 0]';
    ddUj=[6*uj   2  0 0]';
    dddUj=[6   0  0 0]';
    
    pos=Qj*A_pos_bs_{tm(j)}*Uj;
    vel= (1/(deltaT_))   *Qj*A_pos_bs_{tm(j)}  * dUj;
    accel= (1/(deltaT_^2)) *Qj*A_pos_bs_{tm(j)}  * ddUj;
    jerk= (1/(deltaT_^3)) *Qj*A_pos_bs_{tm(j)}  * dddUj;

    interval=[init_int, end_int];

    fplot3(pos(1),pos(2),pos(3),interval); 

    for t_m_i=init_int:0.5:end_int  %t_constrained
        
        w_t_b=subs(pos,t_m,t_m_i);
        qabc=qabcFromAccel(subs(accel,t_m,t_m_i), 9.81);
        qpsi=[0 0 0 1];%[cos(psiT/2), 0, 0, sin(psiT/2)]; %Note that qpsi has norm=1
        q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
        w_R_b=toRotMat(q);
        w_T_b=[w_R_b w_t_b; 0 0 0 1];
        plotAxesArrowsT(0.2,double(w_T_b))
    end
    
%     subplot(4,1,1);
%     fplot(pos, interval)
%     subplot(4,1,2);
%     fplot(vel, interval)
%     subplot(4,1,3);
%     fplot(accel, interval)
%     subplot(4,1,4);
%     fplot(jerk, interval)

end
view([45,45]); axis equal; xlabel('x'); ylabel('y'); zlabel('z');


% disp("Plotting")
% for n=1:NoI
%     for tau_i=t0:0.05:tf  %t_constrained
% 
%         Tau_i=[tau_i^3 tau_i^2 tau_i 1]';
% 
%         w_t_b = sol.value(P{n})*Tau_i;
%         accel = sol.value(A{n})*Tau_i;
%         psiT=sol.value(Psi{n})*Tau_i;
% 
%         qabc=qabcFromAccel(accel, g);
%         qpsi=[cos(psiT/2), 0, 0, sin(psiT/2)]; %Note that qpsi has norm=1
%         q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
%         w_R_b=toRotMat(q);
%         w_T_b=[w_R_b w_t_b; 0 0 0 1];
%         plotAxesArrowsT(0.2,w_T_b)
% 
%     end
%     disp(['Done plotting interval ', num2str(n)])
% end

for i=0:(N_-3) % i  is the interval (\equiv segment)
    init_int=knots(tm(p_+i));
    end_int=knots(tm(p_+i+1));
 
    for obst_index=0:(num_of_obst_-1)
       vertexes=getVertexesMovingObstacle(init_int,end_int); %This call should depend on the obstacle itself
    end
    
    x=vertexes(1,:);     y=vertexes(2,:);    z=vertexes(3,:);
    
    [k1,av1] = convhull(x,y,z);
    trisurf(k1,x,y,z,'FaceColor','cyan')
    
end

fprintf('Final Cost=')
sol.value((q_exp_{tm(N_)}-pf)'*(q_exp_{tm(N_)}-pf))

%%

for n=1:NoI

P{n} = opti.variable(3,4);

V{n}=[0 3*P{n}(1,1) 2*P{n}(1,2) P{n}(1,3);
      0 3*P{n}(2,1) 2*P{n}(2,2) P{n}(2,3);
      0 3*P{n}(3,1) 2*P{n}(3,2) P{n}(3,3);
];

A{n}=[0    0     2*V{n}(1,2) V{n}(1,3);
      0    0     2*V{n}(2,2) V{n}(2,3);
      0    0     2*V{n}(3,2) V{n}(3,3);
];

J{n}=[
   0    0    0   6*V{n}(1,2);
   0    0    0   6*V{n}(2,2);
   0    0    0   6*V{n}(3,2);
];

Psi{n} = [0 opti.variable(1,3)]; %0t^3 + at^2 + bt + c 

VPsi{n}=[0    0     2*Psi{n}(1,2) Psi{n}(1,3)];

APsi{n}=[0    0    0   VPsi{n}(1,3)];

jerk=J{n}(:,end);
apsi=APsi{n}(:,end);

psi_cost=psi_cost+apsi*apsi; %apsi is a scalar
jerk_cost = jerk_cost + jerk'*jerk;

end





%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

NoI=4; %number of intervals

jerk_cost=0;
psi_cost=0;
opti = casadi.Opti();

%Start and end of each interval
t0=0.0;
tf=1.0;

syms t_m %matlab symbolic t
T_m=[t_m^3 t_m^2 t_m 1]';
T0=double(subs(T_m,t_m,t0));
Tf=double(subs(T_m,t_m,tf));

init_pos=[0;0;0];
init_vel=[0;0;0];
init_accel=[0;0;0];
final_pos=[2;0;0];
final_vel=[0;0;0];
final_accel=[0;0;0];

t=opti.variable(1,1);
T=[t*t*t;t*t;t;1];

for n=1:NoI

P{n} = opti.variable(3,4);

V{n}=[0 3*P{n}(1,1) 2*P{n}(1,2) P{n}(1,3);
      0 3*P{n}(2,1) 2*P{n}(2,2) P{n}(2,3);
      0 3*P{n}(3,1) 2*P{n}(3,2) P{n}(3,3);
];

A{n}=[0    0     2*V{n}(1,2) V{n}(1,3);
      0    0     2*V{n}(2,2) V{n}(2,3);
      0    0     2*V{n}(3,2) V{n}(3,3);
];

J{n}=[
   0    0    0   6*V{n}(1,2);
   0    0    0   6*V{n}(2,2);
   0    0    0   6*V{n}(3,2);
];

Psi{n} = [0 opti.variable(1,3)]; %0t^3 + at^2 + bt + c 

VPsi{n}=[0    0     2*Psi{n}(1,2) Psi{n}(1,3)];

APsi{n}=[0    0    0   VPsi{n}(1,3)];

jerk=J{n}(:,end);
apsi=APsi{n}(:,end);

psi_cost=psi_cost+apsi*apsi; %apsi is a scalar
jerk_cost = jerk_cost + jerk'*jerk;

end

%Initial and final constraints
opti.subject_to( P{1}*T0==init_pos );
opti.subject_to( V{1}*T0==init_vel );
opti.subject_to( A{1}*T0==init_accel );
opti.subject_to( P{NoI}*Tf==final_pos );
opti.subject_to( V{NoI}*Tf==final_vel );
opti.subject_to( A{NoI}*Tf==final_accel );

opti.subject_to( Psi{1}*T0==0 );
% opti.subject_to( sin(Psi{NoI}*Tf)==sin(0) );%  (wrapping psi)

%Continuity constraints
for n=1:(NoI-1)
    opti.subject_to( P{n+1}*T0==P{n}*Tf );
    opti.subject_to( V{n+1}*T0==V{n}*Tf );
    opti.subject_to( A{n+1}*T0==A{n}*Tf );

    opti.subject_to( sin(Psi{n+1}*T0)==sin(Psi{n}*Tf) ); %   (wrapping psi)
    opti.subject_to( cos(Psi{n+1}*T0)==cos(Psi{n}*Tf) ); %   (wrapping psi)
      
    opti.subject_to( VPsi{n+1}*T0==VPsi{n}*Tf );
end


g=9.81;
%Compute perception cost
dist_im_cost=0;
vel_im_cost=0;
vel_isInFOV_im_cost=0;

for n=1:(NoI)

    deltat=0.4;
    t_constrained=0.0:deltat:1.0;
    j=1;

    At=A{n}*T;
    Pt=P{n}*T;
    axt=At(1,:); ayt=At(2,:); azt=At(3,:);
    psit=Psi{n}*T;

    qabc=qabcFromAccel([axt ayt azt],g);
    qpsi=[cos(psit/2), 0, 0, sin(psit/2)]; %Note that qpsi has norm=1
    q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1
    w_R_b=toRotMat(q);
    w_fe=[1 1 1 1]';   %feature in world frame
    w_T_b=[w_R_b Pt; zeros(1,3) 1];

    b_T_c=[roty(90)*rotz(90) zeros(3,1); zeros(1,3) 1];

    c_P=inv(b_T_c)*invPose(w_T_b)*w_fe; %Position of the feature in the camera frame
    s=c_P(1:2)/c_P(3);  
   

  
    s_dot=jacobian(s,t);
    
    % See https://en.wikipedia.org/wiki/Cone#Equation_form:~:text=In%20implicit%20form%2C%20the%20same%20solid%20is%20defined%20by%20the%20inequalities
    theta_deg=45;
    is_in_FOV1=-(c_P(1)^2+c_P(2)^2)*(cosd(theta_deg))^2 +(c_P(3)^2)*(sind(theta_deg))^2; %(if this quantity is >=0)
    is_in_FOV2=c_P(3); %(and this quantity is >=0)
    
%     isInFOV=is_in_FOV1*is_in_FOV2; % be careful because if both are
%     negative, this is also satisfied
    
    beta=10;
    isInFOV_smooth=  (   1/(1+exp(-beta*is_in_FOV1))  )*(   1/(1+exp(-beta*is_in_FOV2))  );

    f_vel_im{n}=(s_dot'*s_dot);
    f_dist_im{n}=(s'*s); %I wanna minimize the integral of this funcion. Approx. using symp. Rule
    f_isInFOV_im{n}=(isInFOV_smooth); %/(0.1+f_vel_im{n})
    f_vel_isInFOV_im{n}=(-isInFOV_smooth)/(0.1+f_vel_im{n});
    
    for t_i=t_constrained
        
        simpson=getSimpsonCoeff(j,numel(t_constrained));

        dist_im_cost=dist_im_cost               + (deltat/3.0)*simpson*substitute(f_dist_im{n},t,t_i);
        vel_im_cost=vel_im_cost                 + (deltat/3.0)*simpson*substitute(f_vel_im{n},t,t_i);
        vel_isInFOV_im_cost=vel_isInFOV_im_cost + (deltat/3.0)*simpson*substitute(f_vel_isInFOV_im{n},t,t_i);
    end
   
end

jit_compilation=false;

%Solve first without perception cost (to get an initial guess)
opti.minimize(jerk_cost + 33333*psi_cost );
opti.solver('ipopt',struct('jit',jit_compilation));
sol = opti.solve();


for n=1:NoI
    P_guess{n}=sol.value(P{n});
    Psi_guess{n}=sol.value(Psi{n});
end

%Use the initial guess to solve with all costs
for n=1:NoI
    opti.set_initial(P{n}, P_guess{n})
    opti.set_initial(Psi{n}, Psi_guess{n})
end


opti.minimize(0.0000005*jerk_cost+...
              0.0*psi_cost+...
              0.0*dist_im_cost+...
              1.0*vel_isInFOV_im_cost);
sol = opti.solve();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% PLOTTING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure; hold on;

for n=1:NoI
    position=sol.value(P{n})*T_m;
    fplot3(position(1),position(2),position(3),[t0,tf]); 
end

plotSphere(init_pos,0.05,'r'); plotSphere(final_pos,0.05,'b'); plotSphere(w_fe(1:3),0.05,'g');
% plotSphere(double(subs(position,t,0.5)),0.02,'r');

view([45,45]); axis equal
% 
disp("Plotting")
for n=1:NoI
    for tau_i=t_constrained %t0:0.05:tf  %t_constrained

        Tau_i=[tau_i^3 tau_i^2 tau_i 1]';

        w_t_b = sol.value(P{n})*Tau_i;
        accel = sol.value(A{n})*Tau_i;
        psiT=sol.value(Psi{n})*Tau_i;

        qabc=qabcFromAccel(accel, g);

        qpsi=[cos(psiT/2), 0, 0, sin(psiT/2)]; %Note that qpsi has norm=1
        q=multquat(qabc,qpsi); %Note that q is guaranteed to have norm=1

        w_R_b=toRotMat(q);
        w_T_b=[w_R_b w_t_b; 0 0 0 1];
        plotAxesArrowsT(0.2,w_T_b)

    end
    disp(['Done plotting interval ', num2str(n)])
end

grid on; xlabel('x'); ylabel('y'); zlabel('z'); 
camlight
lightangle(gca,45,0)


figure; 
subplot(3,1,1);hold on; title('yaw')
subplot(3,1,2); hold on; title('vyaw')
subplot(3,1,3); hold on; title('ayaw')
for n=1:NoI
    init_interval=t0+(n-1)*(tf-t0);
    interval=[init_interval, tf+(n-1)*(tf-t0)];
    tau=t_m-init_interval;
    
    Tau=[tau^3 tau^2 tau 1]';
    
    subplot(3,1,1);
    fplot(sol.value(Psi{n})*Tau, interval)
    subplot(3,1,2);
    fplot(sol.value(VPsi{n})*Tau, interval)
    subplot(3,1,3);
    fplot(sol.value(APsi{n})*Tau, interval)
end


figure; hold on; 
subplot(3,1,1);hold on; title('isInFOV()')
subplot(3,1,2); hold on; title('Cost v')
subplot(3,1,3); hold on; title('Cost -isInFOV()/(e + v)')

for n=1:NoI
    for tau_i=t_constrained %t0:0.05:tf  %t_constrained
        init_interval=t0+(n-1)*(tf-t0);

      
    subplot(3,1,1);    ylim([0,1]);
    stem(init_interval+tau_i, sol.value(substitute(f_isInFOV_im{n},t,tau_i)),'filled','r')
    subplot(3,1,2);
    stem(init_interval+tau_i, sol.value(substitute(f_vel_im{n},t,tau_i)),'filled','r')
    subplot(3,1,3);
    stem(init_interval+tau_i, sol.value(substitute(f_vel_isInFOV_im{n},t,tau_i)),'filled','r')
       
    end
    
end

figure; 
subplot(3,1,1);hold on; title('p')
subplot(3,1,2); hold on; title('v')
subplot(3,1,3); hold on; title('a')
for n=1:NoI
    init_interval=t0+(n-1)*(tf-t0);
    interval=[init_interval, tf+(n-1)*(tf-t0)];
    tau=t_m-init_interval;
    
    Tau=[tau^3 tau^2 tau 1]';
    
    subplot(3,1,1);
    fplot(sol.value(P{n})*Tau, interval)
    subplot(3,1,2);
    fplot(sol.value(V{n})*Tau, interval)
    subplot(3,1,3);
    fplot(sol.value(A{n})*Tau, interval)
end

%convert c index to matlab index
function result=tm(x)
    result=x+1;
end
