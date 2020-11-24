function position=getPosMovingObstacle(t)

    pos_x=sin(t)+2*sin(2*t);
    pos_y=(cos(t)-2*cos(2*t))/3;
    pos_z=-sin(3*t);
    
    position=[pos_x;pos_y;pos_z];


end