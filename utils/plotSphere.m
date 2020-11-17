% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function handle=plotSphere(position, radius, color)


    [x,y,z] = sphere(50);

    handle = surf(x*radius+position(1),y*radius+position(2),z*radius+position(3),'FaceColor',color,'LineStyle','none' );
%     s2 = surf(x*radius+v2(1),y*radius+v2(2),z*radius+v2(3),'FaceColor',color_vertex,'LineStyle','none');
%     s3 = surf(x*radius+v3(1),y*radius+v3(2),z*radius+v3(3),'FaceColor',color_vertex,'LineStyle','none');
%     s4 = surf(x*radius+v4(1),y*radius+v4(2),z*radius+v4(3),'FaceColor',color_vertex,'LineStyle','none');

end