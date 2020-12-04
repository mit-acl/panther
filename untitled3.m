% import casadi.*
% opti = casadi.Opti();
% 
% Qj=opti.variable(3,4);
% uu=opti.variable(1,1);
% p=3;
% j=0;
% A=computeMatrixForClampedUniformBSpline(3,j,[0,1])
% 
% expression=Qj*A*[uu^3;uu^2;uu;1];
% 
% substitute(expression,uu,0.0)
% 
% % substitute(w_t_b{1},u,0.0)


clc; clear;
import casadi.*
opti = casadi.Opti();

x=opti.variable(1,1);

substitute([x^2 x^1 x^0],x,0) %This gives horzcat(0, 0, 1) --> correct
substitute(x.^[2,1,0],x,0) %This gives zeros(1x3), why? 

%%
clc
syms u real; p=10; order=0; Tmp=(u.^[p:-1:0])'; for (order=0:10) diff(Tmp,u,order)'
end

%%

x=sym('x','real')

expres=2*x;

u=opti.variable(1,1);

subs(expres,x,u)

%% 
syms s
exp_sym = (s+2)/(s^2+5*s+9);    
s = opti.variable(1,1);
eval(char(exp_sym))

%% 
