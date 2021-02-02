%%
% clc;
% import casadi.*
% t=MX.sym('t',1,1);
% a= 6.0 %MX.sym('a',1,1);
% b= 7.0 %MX.sym('b',1,1);
% c= 2.0 %MX.sym('c',1,1);
% d= 1.5 %MX.sym('d',1,1);
% poly=a*t^3 + b*t^2 + c*t +d;
% 
% coeff=getCoeffPolyCasadi(poly,t, 3)
% coeff(1)
% coeff(2)
% coeff(3)
% coeff(4)
function result=getCoeffPolyCasadi(poly, var, deg)
import casadi.*
coeff=[];
for i=0:(deg)  %for each of the coefficients
    deriv=poly;
    for k=0:(i-1)
       deriv=jacobian(deriv,var);
    end

    coeff=[coeff casadi.substitute(deriv,var,0.0)/factorial(i)];
end
result=flip(coeff); %[a b c ...] --> a^degree + b^(degree-1) + ....
end