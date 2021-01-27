#!/usr/bin/env python


import numpy as np
import rospy

import sympy as sp
import rospkg
import collections

def getTrefoil(tt,offset,slower,lim_x, lim_y, lim_z):
    x=(sp.sin(tt/slower+offset)+2*sp.sin(2*(tt/slower)+offset)+3)/6; # in [0,1] approx
    y=(sp.cos(tt/slower+offset)-2*sp.cos(2*(tt/slower)+offset)+3)/6;
    z=((-sp.sin(3*(tt/slower)+offset))+1.0)/2.0; # in [0,1] approx

    x=min(lim_x)+(max(lim_x)-min(lim_x))*x
    y=min(lim_y)+(max(lim_y)-min(lim_y))*y
    z=min(lim_z)+(max(lim_z)-min(lim_z))*z

    return [x, y, z]

#bbox has three elements: [hx, hy, hz] (bbox of size hx x hy x hz)
Drone = collections.namedtuple('Drone', ["name","bbox", "slower", "offset", "lim_x", "lim_y", "lim_z"])

all_drones=[     #"name",     "bbox",      "slower", "offset", "lim_x",    "lim_y",    "lim_z"
            Drone("SQ01s", [0.5, 0.5, 2.5],  3.0,   0.0,    [-3.0,3.0],   [-3.0,3.0],  [1.0,3.0]),
            Drone("SQ02s", [0.5, 0.5, 2.5],  3.0,   2*np.pi/4,    [-3.0,3.0],   [-3.0,3.0],  [1.0,3.0]),
            Drone("SQ03s", [0.5, 0.5, 2.5],  3.0,   2*2*np.pi/4,    [-3.0,3.0],   [-3.0,3.0],  [1.0,3.0]),
            Drone("SQ04s", [1.5, 1.5, 1.5],  3.0,   3*2*np.pi/4,    [-3.0,3.0],   [-3.0,3.0],  [1.0,3.0])
            ]


tmp = rospkg.RosPack()
pwd_package=tmp.get_path('mader')

t=sp.symbols('t')

for i in range(len(all_drones)):
    drone_i=all_drones[i];
    traj=getTrefoil(t, drone_i.offset, drone_i.slower, drone_i.lim_x, drone_i.lim_y, drone_i.lim_z)
    # print traj
    name_file=pwd_package+"/obstacles/"+drone_i.name+".yaml"
    f = open(name_file, "w")
    f.write("# DO NOT EDIT. RUN THE PYTHON FILE INSTEAD TO GENERATE THIS .yaml FILE \n")
    f.write("traj_x: "+str(traj[0])+"\n")
    f.write("traj_y: "+str(traj[1])+"\n")
    f.write("traj_z: "+str(traj[2])+"\n")
    f.write("bbox: "+str(drone_i.bbox)+"\n")
    f.close()
    print "Writing to " + name_file 

# import matplotlib.pyplot as plt
# import matplotlib as mpl
# from mpl_toolkits.mplot3d import Axes3D
# from snapstack_msgs.msg import Goal, State

#class bbox
# class Bbox:
#   self.cent_top=[0,0,0]; #centroid of the top face of the bbox
#   self.wx=1.0;
#   self.wy=1.0;
#   self.wz=1.0;
#   def getBounds(self):
#       a_minX=self.cent_top[0]-self.wx/2.0;
#       a_minY=self.cent_top[1]-self.wy/2.0;
#       a_minZ=self.cent_top[2]-self.wz;

#       a_maxX=self.cent_top[0]+self.wx/2.0;;
#       a_maxY=self.cent_top[1]+self.wy/2.0;
#       a_maxZ=self.cent_top[2];

#       return [a_minX, a_minY, a_minZ, a_maxX, a_maxY, a_maxZ]

# class Bbox:
#     def __init__(self, wx, wy, wz):
#         self.wx=1.0;
#         self.wy=1.0;
#         self.wz=1.0;

# #https://developer.mozilla.org/en-US/docs/Games/Techniques/3D_collision_detection#aabb_vs._aabb:~:text=And%20in%20JavaScript%2C%20we'd%20use%20this%3A
# def bboxesCollide(a,b):
#     [a_minX, a_minY, a_minZ, a_maxX, a_maxY, a_maxZ]=a.getBounds();
#     [b_minX, b_minY, b_minZ, b_maxX, b_maxY, b_maxZ]=b.getBounds();


    # return ((a_minX <= b_maxX and a_maxX >= b_minX) and
    #      (a_minY <= b_maxY and a_maxY >= b_minY) and
    #      (a_minZ <= b_maxZ and a_maxZ >= b_minZ));

# def getStraightLine(tt, p_init, p_final):
#     v=((p_final-p_init)/5.0)
#     return p_init+v * tt

# class Obstacle:
#     def __init__(self, name, offset, wx, wy, wz, lim_x, lim_y, lim_z):
#         self.t=sp.symbols('t');
#         self.traj=getTrefoil(self.t,offset,lim_x,lim_y,lim_z)
#         self.bbox=Bbox(wx, wy, wz);
#         self.lim_x=lim_x;
#         self.lim_y=lim_y;
#         self.lim_z=lim_z;
#         self.name=name;
#         self.goal_pub = rospy.Publisher('/'+self.name+'/goal', Goal, queue_size=1, latch=False)
#         rospy.Subscriber("state", State, self.stateCB)
#         self.state_initialized=False;
#     def pubGoal(self):
#         goal=Goal()
#         goal.header.stamp=rospy.Time.now();
#         t=rospy.get_time(); #Same as before, but it's float
#         traj_eval=self.evalTraj(t)
#         goal.p.x=traj_eval[0];
#         goal.p.y=traj_eval[1];
#         goal.p.z=traj_eval[2];
#         self.goal_pub.publish(goal)
#     def stateCB(self,data):
#         self.state=data;
#         self.state_initialized=True;
#     def evalTraj(self,ti):
#         return [self.traj[0].subs(self.t,ti), self.traj[1].subs(self.t,ti), self.traj[2].subs(self.t,ti)]
#     def goToInitialPoint();


# class Misson:
#     def __init__(self):
#         obs1=Obstacle("SQ01s",0.0, 1.0,1.0,1.0, [1.0,2.0], [1.0,3.0], [1.0,5.0]);
#         obs1.pubGoal()

# if __name__ == '__main__':


#     ns = rospy.get_namespace()
#     try:
#         rospy.init_node('trajectories_obs_hw_exps')
#         mission=Misson()
        
#     except rospy.ROSInterruptException:
#         pass

# mpl.rcParams['legend.fontsize'] = 10

# fig = plt.figure()
# ax = fig.gca(projection='3d')

# lim_x=[1.0,2.0];
# lim_y=[1.0,3.0];
# lim_z=[1.0,5.0];


# T=2*3.14;
# t=np.linspace(0, T, 100)
# offset=0.0;
# slower=1.0;
# tt=t/slower;

# [x,y,z]=getTrefoil(tt,offset,lim_x,lim_y,lim_z)
# ax.plot(x, y, z, label='parametric curve')

# [xs,ys,zs]=getTrefoil(0.0,offset,lim_x,lim_y,lim_z)
# ax.scatter(xs, ys, zs, marker='o')

# [xs,ys,zs]=getTrefoil(0.0,T/4.0,lim_x,lim_y,lim_z)
# ax.scatter(xs, ys, zs, marker='o')

# [xs,ys,zs]=getTrefoil(0.0,2*T/4.0,lim_x,lim_y,lim_z)
# ax.scatter(xs, ys, zs, marker='o')


# [xs,ys,zs]=getTrefoil(0.0,3*T/4.0,lim_x,lim_y,lim_z)
# ax.scatter(xs, ys, zs, marker='o')

# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')
# ax.legend()

# plt.show()


