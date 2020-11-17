% /* ----------------------------------------------------------------------------
%  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

%A needs to be expressed in t \in interv
function [volume, num_vertexes]=plot_splitted_convex_hulls(P,A,interv,num_of_intervals,color,radius_sphere);

samples=[];
samples_t=linspace(min(interv),max(interv),num_of_intervals+1);
all_vertexes=[];%Its columns are the vertexes
for i=1:(length(samples_t)-1)
    a=samples_t(i);
    b=samples_t(i+1);
    P_converted=convertCoeffMatrixFromABtoCD(P,[a,b],interv);
    V=P_converted*inv(A);
    all_vertexes=[all_vertexes V];
    %plot_convex_hull(P_converted(1,:)',P_converted(2,:)',P_converted(3,:)',A,'b',0.0017);    
end

color_vertex=[.98 .45 .02];

for i=1:size(all_vertexes,2)
    s1=plotSphere(all_vertexes(:,i),radius_sphere, color_vertex);
end

axis equal
tmp=gca;
if (size(findobj(tmp.Children,'Type','Light'))<1) %If still no light in the subplot
 camlight %create light
end
lighting phong

 
x=all_vertexes(1,:); y=all_vertexes(2,:); z=all_vertexes(3,:);
[k1,volume] = convhull(x,y,z);
s2=trisurf(k1,x,y,z,'LineWidth',1,'FaceColor',color);
alpha(s2,0.1)

k_all=k1(:);
k_all_unique=unique(k1);
num_vertexes=length(k_all_unique); %points that are in the frotier of the convex hull

end