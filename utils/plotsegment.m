% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function plotsegment(a,b,color,linewidth)
   AB=[a b];
   plot3(AB(1,:),AB(2,:),AB(3,:),color,'LineWidth',linewidth)
end