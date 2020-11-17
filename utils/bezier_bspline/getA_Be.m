% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

% Returns the A matrix for the Bernstein basis

function A=getA_Be(deg,interval)

syms t real

A=[];
tmp=bernsteinMatrix(deg, t);
for i=1:length(tmp)
    A=[A; double(coeffs(tmp(i),t,'All'))];

end

%A is expressed in t\in[0,1] at this point 

A=convertCoeffMatrixFromABtoCD(A,[0,1],interval); 
    
end