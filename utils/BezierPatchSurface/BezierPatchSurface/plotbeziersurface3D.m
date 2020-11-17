% % See also plotbezierpatch3D.m

% % function to plot 3D cubic Bezier surface in many ways.

% % A Bezier surface is composed of one or more patches.
% % A patch is defined by 16 control points arranged in 4 x 4 x 3 matrix. 
% % A control point has three coordinates (x,y,z)

% % Details
% % -> Input Matrix S stores all the control points of all the patches of
% %    a Bezier surface such that
% % -> S(:,:,:,k) holds control points of kth patch, 
% %    where k=1..np and np is number of patches in the surface 
% % -> Size of S(:,:,:,k) is 4 x 4 x 3, i.e., 16 control points and each
% %    control point has three coordinates (x,y,z)
% %    S(:,:,1,k): x-coordates of control points of kth patch as 4 x 4 matrix 
% %    S(:,:,2,k): y-coordates of control points of kth patch as 4 x 4 matrix 
% %    S(:,:,3,k): z-coordates of control points of kth patch as 4 x 4 matrix
% % -> Input Matrix Q stores interpolated values between control points
% %    Q is similar to S in format but has more values, i.e., it stores 
% %    end control points and interpolated values. 
% %    see the function bezierpatchinterp.m for more details

function plotbeziersurface3D(S,Q)

[r c dim np]=size(S);
% % np: number of patches

if dim > 3 or dim < 3 
    error ('plotbeziersurface3D.m function handles only 3-D points')
end

az=21;  %azimuth
el=19;  %elevation. 

lw=1; %plotting linewidth

str1='\bf Control Point';
str2='\bf Control Polygon';
str3='\bf Surface (bi-directional Bezier curve)';

% %-----------------------------------------------
% % For plotting it is convenient if we reshape the data
% % into vecotor format and separate (X,Y,Z) coordinates

xS=[]; yS=[]; zS=[];
for k=1:np
     xS =horzcat(xS, reshape(S(:,:,1,k),1,[])); 
     yS =horzcat(yS, reshape(S(:,:,2,k),1,[]));
     zS =horzcat(zS, reshape(S(:,:,3,k),1,[]));     
end

xQ=[]; yQ=[]; zQ=[];
for k=1:np
     xQ =horzcat(xQ, reshape(Q(:,:,1,k),1,[])); 
     yQ =horzcat(yQ, reshape(Q(:,:,2,k),1,[])); 
     zQ =horzcat(zQ, reshape(Q(:,:,3,k),1,[]));    
end
% %------------------------------------------------
% % Wireframe Plot of Bezier Surface using interpolation of control points
% % Control Points and Control Polygon are marked 
figure, hold on
plot3(xS,yS,zS,'ro','LineWidth',lw)
plot3(xS,yS,zS,'g','LineWidth',lw)
plot3(xQ,yQ,zQ,'b','LineWidth',lw)
legend(str1,str2,str3);
title('\bf Wireframe Plot of a Bezier Surface with Control Points and Control Polygon')
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Wireframe Plot of Bezier Surface using interpolation of control points
figure, hold on
plot3(xQ,yQ,zQ,'b','LineWidth',lw)
legend(str3);
title('\bf Wireframe Plot of a Bezier Surface')
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Plot of Surface using control points 
figure, hold on
for k=1:np
    surface(S(:,:,1,k),S(:,:,2,k),S(:,:,3,k),'FaceColor','green')
end
title('\bf Bezier Surface using Control Points');
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Plot of Surface using interpolation of control points
figure, hold on
for k=1:np
    surface(Q(:,:,1,k),Q(:,:,2,k),Q(:,:,3,k),'FaceColor','green')
end
title('\bf Bezier Surface using Interpolated Points');
view(3); box;  view(az,el)

% %----------------------------------------------- 
% % Surface with interpolated shading
figure, hold on
for k=1:np
    surface(Q(:,:,1,k),Q(:,:,2,k),Q(:,:,3,k))
end
shading interp
title('\bf Bezier Surface using Interpolated Shading');
view(3); box;  view(az,el)
% %-----------------------------------------------% 
% % Surface with faceted shading
figure, hold on
for k=1:np
    surface(Q(:,:,1,k),Q(:,:,2,k),Q(:,:,3,k))
end
shading faceted
title('\bf Bezier Surface using Faceted Shading');
view(3); box;  view(az,el)
% %-----------------------------------------------% 

% % --------------------------------
% % This program or any other program(s) supplied with it does not provide any
% % warranty direct or implied.
% % This program is free to use/share for non-commerical purpose only. 
% % Kindly reference the author.
% % Author: Dr. Murtaza Khan
% % URL : http://www.linkedin.com/pub/dr-murtaza-khan/19/680/3b3
% % --------------------------------
