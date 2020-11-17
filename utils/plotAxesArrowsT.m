% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */


%T is the 
function plotAxesArrowsT(length, w_T_b)

w_t_b=w_T_b(1:3,4);

%Let D be a point along x_body, E along y_body,...

b_D=[length 0 0 1]';
b_E=[0 length 0 1]';
b_F=[0 0 length 1]';

w_D=w_T_b*b_D;
w_E=w_T_b*b_E;
w_F=w_T_b*b_F;



% arrow3dWithColor(w_t_b',w_D(1:3)',20,'cylinder',[0.2,0.1],'r');
% arrow3dWithColor(w_t_b',w_E(1:3)',20,'cylinder',[0.2,0.1],'g');
arrow3dWithColor(w_t_b',w_F(1:3)',20,'cylinder',[0.2,0.1],'b');
end