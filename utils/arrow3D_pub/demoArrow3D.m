%% arrow3D - creates a single volumized arrow based on a cylinder/cone combination

%% Basic Setup
    arrow3D([0,0,0] ,[1,2,3]);    
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Basic arrow3D call');
    
%% Enhancing Visual Appearance    
    lighting phong;
    camlight right;
    title('');
    
    
%% Stem Ratio Options
    subplot(131); 
    arrow3D([0,0,0], [0,0,3] , 'r', 0.25);
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    lighting phong;
    camlight right;
    title('Stem Ratio = 0.25');

    subplot(132); 
    arrow3D([0,0,0], [0,0,3], 'g', 0.5);
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    lighting phong;
    camlight right;
    title('Stem Ratio = 0.5');
    
    subplot(133); 
    arrow3D([0,0,0], [0,0,3], 'b', 0.75);
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    lighting phong;
    camlight right;
    title('Stem Ratio = 0.75');

%% arrow3D handle Options
% The form of the arrowHandle is the same as 'surf'.  arrowHandle = [arrowStem, arrowHead]
    subplot(131); 
    hold off;
    arrow3D([0,0,0], [0,0,3], 'r');
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    lighting phong;
    camlight right;

    subplot(132); 
    hold off;
    arrowHandle = arrow3D([0,0,0], [0,0,3], 'r');
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    set(arrowHandle(1), 'FaceColor', 'c');
    lighting phong;
    camlight right;
        
    subplot(133); 
    hold off;
    arrowHandle = arrow3D([0,0,0], [0,0,3], 'r');
    hold on;   axis equal;   xlabel('X'); ylabel('Y'); zlabel('Z');
    set(arrowHandle(1), 'FaceColor', 'c');
    set(arrowHandle(2), 'FaceColor', 'b');
    lighting phong;
    camlight right;

    
%% Credits
% Author: Shawn Arseneau
% 
% Created: September 14, 2006


