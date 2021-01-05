
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

opti = casadi.Opti();
deg_pos=3;
dim_pos=3;
num_seg =4; %number of segments

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%    PREDICTION     %%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_observations=10;
all_t=linspace(0.5,10,n_observations);

      
all_pos=MX.sym('all_pos',dim_pos,n_observations);


t0=min(all_t);
tf=max(all_t);

sp_tmp=MyClampedUniformSpline(t0,tf,deg_pos, dim_pos, num_seg, opti);  %creating another object to not mess up with sy

% all_yaw=MX.sym('lambda1',1,numel(t_simpson));
cost_function=0;
for i=1:size(all_pos,2)
    dist=sp_tmp.getPosT(all_t(i))-all_pos(:,i);
    cost_function = cost_function + dist'*dist; 
end

% lambda1=MX.sym('lambda1',1,1);
% lambda2=MX.sym('lambda2',1,1);
% lambda3=MX.sym('lambda3',1,1);

% c1= sy_tmp.getPosT(t0) - y0; %==0
% c2= sy_tmp.getVelT(t0) - ydot0; %==0
% c3= sy_tmp.getVelT(tf) - ydotf; %==0

lagrangian = cost_function; %  +  lambda1*c1 + lambda2*c2 + lambda3*c3;

variables=[sp_tmp.getCPsAsMatrix()];% lambda1 lambda2  lambda3];

kkt_eqs=jacobian(lagrangian, variables)'; %I want kkt=[0 0 ... 0]'

%Obtain A and b
b=-casadi.substitute(kkt_eqs, variables, zeros(size(variables))); %Note the - sign
A=jacobian(kkt_eqs, variables);

solution=A\b;  %Solve the system of equations

solution=reshape(solution,dim_pos,numel(solution)/dim_pos);

sp_tmp.updateCPsWithSolution(full(solution));

xf=sp_tmp.getPosT(tf);
vf=sp_tmp.getVelT(tf);
af=sp_tmp.getAccelT(tf);

%Obtain the coefficients
% xf+vf*(t-tf) + 0.5*af*(t-tf)^2   \equiv 
% xf+vf*t - vf*tf + 0.5*af*t^2 -af*tf*t + 0.5*af*tf^2;  \equiv 
% (0.5*af)*t^2 + (vf - af*tf)*t +  (xf+0.5*af*tf^2 - vf*tf )      

a=0.5*af;
b=vf-af*tf;
c=xf+0.5*af*tf^2- vf*tf;
    
%obtain the coefficients (second way to do it, slower)
% t=MX.sym('t',1,1);
% poly_predicted=xf+vf*(t-tf) + 0.5*af*(t-tf)^2;
% c=substitute(poly_predicted, t, 0.0);
% b=substitute(jacobian(poly_predicted,t), t, 0.0);
% a=(1/2)*substitute(jacobian(jacobian(poly_predicted,t),t), t, 0.0);

coeff_predicted=[a b c];


f= Function('f', {all_pos}, [{solution, coeff_predicted}], ...
                 {'all_pos'}, {'solution', 'coeff_predicted'} );
% f=f.expand();

all_pos_value= [linspace(0.0,10,n_observations);
                linspace(0.0,10,n_observations);
                linspace(0.0,10,n_observations)] + 4*rand(dim_pos,n_observations);

tic
sol=f('all_pos',all_pos_value);
toc

sp_tmp.updateCPsWithSolution(full(sol.solution));
sp_tmp.plotPosVelAccelJerk();
subplot(4,1,1); hold on;
plot(all_t, all_pos_value, 'o')
t=sym('t');
Pt=full(sol.coeff_predicted)*[t^2;t;1];
fplot(Pt,[tf,tf+4],'--')
subplot(4,1,2); hold on;
fplot(diff(Pt,t),[tf,tf+4],'--')
subplot(4,1,3); hold on;
fplot(diff(Pt,t,2),[tf,tf+4],'--')


f.save('predictor.casadi') %The file generated is quite big