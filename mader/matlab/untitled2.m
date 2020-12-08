
clc; clear; close all;
 

import casadi.*
t0=0;
tf=10;
dim=3;


   
opti = casadi.Opti();

deg=randi([2,3]);

num_seg=randi([2,10]);

my_spline=MyClampedUniformSpline(t0, tf, deg, dim, num_seg, opti);

order_derivative=randi([0,deg-1]);

for (interval=0:(num_seg-1))

mia=my_spline.getA_BS_Interval(order_derivative,interval);


otra=computeMatrixForClampedUniformBSpline(deg-order_derivative,interval,[0,1]);

assert(nnz(mia-otra)==0);

interval
num_seg-1

    
end

%%
    
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


