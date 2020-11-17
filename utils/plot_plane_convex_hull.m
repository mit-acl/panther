% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */



function area=plot_plane_convex_hull(pol_x,pol_y,pol_z,A,color,radius_sphere)

    
    P=[pol_x'; pol_y'; pol_z'];
    V=P*inv(A);
    
    vx=V(1,:);
    vy=V(2,:);
    vz=V(3,:);
    

    v1=V(:,1);
    v2=V(:,2);  
    v3=V(:,3);

    

    color_vertex=[.98 .45 .02];
    radius=radius_sphere;
s1=plotSphere(v1,radius, color_vertex);
s2=plotSphere(v2,radius, color_vertex);
s3=plotSphere(v3,radius, color_vertex);


    
       alpha(s1,1.0)
    alpha(s2,1.0)
    alpha(s3,1.0)

    
    patch(vx,vy,vz,color,'FaceAlpha',.3) 
    
    area=norm(cross(V(:,2)-V(:,1),V(:,3)-V(:,1)))/2.0;
    
     axis equal
     tmp=gca;
     if (size(findobj(tmp.Children,'Type','Light'))<1) %If still no light in the subplot
         camlight %create light
     end
     lighting phong

end
