% /* ----------------------------------------------------------------------------
%  * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
%  * Massachusetts Institute of Technology
%  * All Rights Reserved
%  * Authors: Jesus Tordesillas, et al.
%  * See LICENSE file for the license information
%  * -------------------------------------------------------------------------- */

function vertexes=getVertexesMovingObstacle(t_init, t_end)

  hs=0.2; %half side of the box
 
  all_vertexes=[];
  
  for t=linspace(t_init,t_end,6)
       
    p=getPosMovingObstacle(t);
    
    all_vertexes=[all_vertexes p+[hs hs hs]' ];
    
    all_vertexes=[all_vertexes p+[-hs hs -hs]' ];
    all_vertexes=[all_vertexes p+[-hs hs hs]' ];
    all_vertexes=[all_vertexes p+[-hs -hs hs]' ];
    all_vertexes=[all_vertexes p+[-hs -hs -hs]' ];
    
    all_vertexes=[all_vertexes p+[hs hs -hs]' ];
    all_vertexes=[all_vertexes p+[hs -hs hs]' ];
    all_vertexes=[all_vertexes p+[hs -hs -hs]' ];

    
  end
  
  x=all_vertexes(1,:);
  y=all_vertexes(2,:);
  z=all_vertexes(3,:);

  [k1,av1] = convhull(x,y,z);
  k_all=k1(:);
  k_all_unique=unique(k1);

  vertexes=[];
    
  for i=1:length(k_all_unique)
      tmp=[x(k_all_unique(i)) y(k_all_unique(i)) z(k_all_unique(i))]';
      vertexes=[vertexes tmp];
%         plotSphere([x(k_all_unique(i)) y(k_all_unique(i)) z(k_all_unique(i))], 0.03, color)
 end

end