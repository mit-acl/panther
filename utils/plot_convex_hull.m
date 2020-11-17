% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */



function volume=plot_convex_hull(pol_x,pol_y,pol_z,A,color,radius_sphere)

    hold on;
    P=[pol_x'; pol_y'; pol_z'];
    V=P*inv(A);

    v1=V(:,1);
    v2=V(:,2);
    v3=V(:,3);
    v4=V(:,4);
    
    vx=V(1,:)';
    vy=V(2,:)';
    vz=V(3,:)';
    

    color_vertex=[.98 .45 .02];
    radius=radius_sphere;
    s1=plotSphere(v1,radius, color_vertex);
    s2=plotSphere(v2,radius, color_vertex);
    s3=plotSphere(v3,radius, color_vertex);
    s4=plotSphere(v4,radius, color_vertex);

%     
%     
%     set(s1,'facecolor',color_vertex);
%     set(s2,'facecolor',color_vertex);
%     set(s3,'facecolor',color_vertex);
%     set(s4,'facecolor',color_vertex);
    
    alpha(s1,1.0)
    alpha(s2,1.0)
    alpha(s3,1.0)
    alpha(s4,1.0)
    
%     shading faceted
%     hold on
     axis equal
     tmp=gca;
     if (size(findobj(tmp.Children,'Type','Light'))<1) %If still no light in the subplot
         camlight %create light
     end
     lighting phong
%    
    [k1,volume] = convhull(vx,vy,vz);

    s2=trisurf(k1,vx,vy,vz,'LineWidth',1,'FaceColor',color);
   
%     xlabel('$x$')
%     ylabel('y')
%     zlabel('z')
    alpha(s2,0.1)

end
