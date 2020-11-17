% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function plotAxesArrows(length)
arrow3d([0 0 0],[0 0 length],20,'cylinder',[0.2,0.1]);
arrow3d([0 0 0],[0 length 0],20,'cylinder',[0.2,0.1]);
arrow3d([0 0 0],[length 0 0],20,'cylinder',[0.2,0.1]);
end