%theta_deg is the TOTAL angle of the cone (and not half of it)
function plotCone(position,direction,theta_deg,length)

    X1=position;
    X2=position+length*direction/norm(direction);
    start_radius=0.0;
    end_radius=tand(theta_deg/2.0)*length;
    r=[start_radius end_radius]; %starting radius and end radius
    n=20;
    cyl_color='b';
    closed=1;
    lines=0;
    
    Cone(X1,X2,r,n,cyl_color,closed,lines, [0.1,0.3]);
    axis equal;

end
