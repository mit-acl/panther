% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */


 %Assumes that A is expressed in the interval [-1,1]
function [W V]=findWVgivenA(A_solution)
    
    deg=size(A_solution,1)-1;

    deg_is_even = (rem(deg, 2) == 0);

    if(deg_is_even==1)
        d=deg/2;
        size_Wi=d+1;
        size_Vi=d;
    else
        d=(deg-1)/2;
        size_Wi=d+1;
        size_Vi=d+1;
    end
    
    W=[]; V=[];
    for i=1:(deg+1)
       W=[W sdpvar(size_Wi,size_Wi)];
       V=[V sdpvar(size_Vi,size_Vi)];
    end

    sdpvar t %This is not optimized, but needed to handle symbolic expressions
    constraints=[];
    A=[];

    for i=1:(deg+1)
        Wi=W(:,(i-1)*size_Wi+1:i*size_Wi);
        Vi=V(:,(i-1)*size_Vi+1:i*size_Vi);

        %Wi and Vi are psd matrices
        constraints=[constraints, Wi>=0, Vi>=0];

        Tvi=[];
        for i=0:(size(Vi,1)-1)
            Tvi=[Tvi; t^i]; 
        end

        Twi=[];
        for i=0:(size(Wi,1)-1)
            Twi=[Twi; t^i]; 
        end

        if(deg_is_even==1)
            lambdai=Twi'*Wi*Twi + (t+1)*(1-t)*Tvi'*Vi*Tvi; %Assumming here that A is expressed in the interval [-1,1]
        else
            lambdai=(t+1)*Twi'*Wi*Twi + (1-t)*Tvi'*Vi*Tvi;  %Assumming here that A is expressed in the interval [-1,1]
        end
        coeffs_lambdai=flip(coefficients(lambdai,t))';
        A=[A; coeffs_lambdai]; 

    end

    constraints=[constraints, A_solution==A];

    obj=0; %Feasibility problem

    disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
    result=optimize(constraints,obj,sdpsettings('usex0',0,'solver','mosek','showprogress',0,'verbose',0,'debug',0 ))
    
    W=value(W);
    
    V=value(V);
    
end