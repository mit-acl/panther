% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function projected=projectPoint(p, v, q1,q2,q3,frame)
% Performs a perspective projection of the point p (expressed in the world coordinates), 
% doing raycasting from the viewpoint v, using the plane formed by 
%the points q1,q2,q3 and using the direction of cross(q2-q1,q3-q1) as the
%direction of projection. 

%If frame is 'w'(world) --> Returns the coordinates of the point in world
%frame (3x1 vector)

%If frame is 'c'(camera) --> Returns the coordinates of the point in the
%camera plane (2x1 vector)

%Math: see Lecture 11 from VNAV (Image Formation), Luca Carlone

n=cross(q1-q3,q2-q3);
n=n/norm(n);

if((q1-v)'*n<0)
    n=-n; %Ensure n is pointing in the direction v --> plane
end

z_c=n;
x_c=(q1-q3);x_c=x_c/norm(x_c);
y_c=cross(z_c,x_c);

f=abs((q1-v)'*z_c) %focal length;


w_R_c=[x_c y_c z_c];
w_t_c=v;

w_T_c=[w_R_c w_t_c; 0 0 0 1];
c_T_w=inv(w_T_c);

c_projected=[f 0 0; 0 f 0; 0 0 1]*[eye(3) zeros(3,1)]*c_T_w*[p ; 1];

u=c_projected(1)/c_projected(3); %u and v are the coordinates in the image plane
v=c_projected(2)/c_projected(3);

if(frame=='w')
    projected=w_T_c*[u v f 1]'; 
else if(frame=='c')
    projected=[u v]';  
end

end