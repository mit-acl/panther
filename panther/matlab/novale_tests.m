clc; clear; close all;
p=sym('p',[4,1],'real');
syms t;
pt=p'*[t^3; t^2; t; 1];
at=diff(pt,t);

% t=5;
t_value=rand();
pt=subs(pt,t,t_value);
at=subs(at,t,t_value);

termino=(at'*at)*(pt'*pt);

h=hessian(termino,p);

eig(double(subs(h,p,rand(4,1))))



%%
syms s t real
% s=sym('s',[1,1],'real');
% t=sym('t',[1,1],'real');

p_s=(5*s^3+2*s^2+3*s+1);
s_t=(2*t+4);

p_t=subs(p_s,s,s_t)

diff(p_t,t,3)

diff(p_s,s,3)*(diff(s_t,t,1))^3


%%
clc
syms qlp2 qlp1 ql real;
syms dt1 dt2 dt3 dt4 real;

t=[0 dt1 (dt1+dt2) (dt1+dt2+dt3) (dt1+dt2+dt3+dt4)];
p=3;
l=0;

vl=p*(qlp1-ql)/(t(l+p+1)-t(l+1));

vlp1=p*(qlp2-qlp1)/(t(l+1+p+1)-t(l+1+1));

al=(p-1)*(vlp1-vl)/(t(l+p+1)-t(l+2))
% simplify(expand(al))

%%
q=sym('q',[1,4]);
syms u real
A=getA_BS(3, [0,1]);
px=q*A*[u^3; u^2; u; 1];

syms d1 %inv deltaT
syms d2 %inv deltaT
pxd1=subs(px,u,1/d1);
pxd2=subs(px,u,1/d2);
