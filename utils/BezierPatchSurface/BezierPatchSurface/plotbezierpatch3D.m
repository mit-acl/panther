% % See also plotbeziersurface3D.m

% % function to plot 3D cubic Bezier patch in many ways.

% % A Bezier patch is defined by 16 control points arranged in 4 x 4 x 3 matrix. 
% % A control point has three coordinates (x,y,z)

% % Details
% % -> Input Matrix P stores 16 control points of a patches, each
% %    control point has three coordinates (x,y,z)
% % -> Size of P is 4 x 4 x 3 
% % -> P(:,:,k) holds control points of kth dimension, where k=1..3 
% % -> Size of P(:,:,k) is 4 x 4 i.e., 16 control points
% %    P(:,:,1): x-coordates of control points as 4 x 4 matrix 
% %    P(:,:,2): y-coordates of control points as 4 x 4 matrix 
% %    P(:,:,3): z-coordates of control points as 4 x 4 matrix
% % -> Input Matrix Q stores interpolated values between control points
% %    Q is similar to P in format but has more values, i.e., it stores 
% %    end control points and interpolated values. 
% %    see the function bezierpatchinterp.m for more details

function plotbezierpatch3D(P,Q)

[r c dim]=size(P);
% % d: dimension of data

if dim > 3 or dim < 3 
    error ('plotbezierpatch3D.m function handles only 3-D points')
end

az=21;  %azimuth
el=19;  %elevation. 

lw=1; %plotting linewidth

str1='\bf Control Point';
str2='\bf Control Polygon';
str3='\bf Patch (bi-directional Bezier curve)';

% %-----------------------------------------------
% % For plotting it is convenient if we reshape the data
% % into vecotor format and separate (X,Y,Z) coordinates

xP=[]; yP=[]; zP=[];
 xP =horzcat(xP, reshape(P(:,:,1),1,[])); 
 yP =horzcat(yP, reshape(P(:,:,2),1,[]));
 zP =horzcat(zP, reshape(P(:,:,3),1,[]));     


xQ=[]; yQ=[]; zQ=[];
xQ =horzcat(xQ, reshape(Q(:,:,1),1,[])); 
yQ =horzcat(yQ, reshape(Q(:,:,2),1,[])); 
zQ =horzcat(zQ, reshape(Q(:,:,3),1,[]));    

% %------------------------------------------------
% % Wireframe Plot of Bezier Patch using interpolation of control points
% % Control Points and Control Polygon are marked 
figure, hold on
plot3(xP,yP,zP,'ro','LineWidth',lw)
plot3(xP,yP,zP,'g','LineWidth',lw)
plot3(xQ,yQ,zQ,'b','LineWidth',lw)
legend(str1,str2,str3);
title('\bf Wireframe Plot of a Bezier Patch with Control Points and Control Polygon')
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Wireframe Plot of Bezier Patch using interpolation of control points
figure, hold on
plot3(xQ,yQ,zQ,'b','LineWidth',lw)
legend(str3);
title('\bf Wireframe Plot of a Bezier Patch')
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Plot of Patch using control points 
figure, hold on
surface(P(:,:,1),P(:,:,2),P(:,:,3),'FaceColor','green')
title('\bf Bezier Patch using Control Points');
view(3); box;  view(az,el)
% %-----------------------------------------------
% % Plot of Patch plot interpolation of control points
figure, hold on
surface(Q(:,:,1),Q(:,:,2),Q(:,:,3),'FaceColor','green')
title('\bf Bezier Patch using Interpolated Points');
view(3); box;  view(az,el)

% %----------------------------------------------- 
% % Patch with interpolated shading
figure, hold on
surface(Q(:,:,1),Q(:,:,2),Q(:,:,3))
shading interp
title('\bf Bezier Patch using Interpolated Shading');
view(3); box;  view(az,el)
% %-----------------------------------------------% 
% % Patch with faceted shading
figure, hold on
surface(Q(:,:,1),Q(:,:,2),Q(:,:,3))
shading faceted
title('\bf Bezier Patch using Faceted Shading');
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
