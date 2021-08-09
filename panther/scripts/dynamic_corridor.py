#!/usr/bin/env python

# /* ----------------------------------------------------------------------------
#  * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import random
import roslib
import rospy
import math
from panther_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np
from numpy import linalg as LA
import random
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from math import sin, cos, tan, floor
from numpy import sign as sgn #Because https://github.com/ArashPartow/exprtk  uses sgn, not sign
import os
import copy 
import sys
import glob
import rospkg

def getColorJet(v, vmin, vmax): 

  c=ColorRGBA()

  c.r = 1;
  c.g = 1;
  c.b = 1;
  c.a = 1;

  if (v < vmin):
    v = vmin;
  if (v > vmax):
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)):
    c.r = 0;
    c.g = 4 * (v - vmin) / dv;
  
  elif (v < (vmin + 0.5 * dv)):
    c.r = 0;
    c.b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  
  elif (v < (vmin + 0.75 * dv)):
    c.r = 4 * (v - vmin - 0.5 * dv) / dv;
    c.b = 0;
  
  else:
    c.g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    c.b = 0;

  return c;


class DynCorridor:

    def getTrajectoryPosMeshBBox(self, i):
        delta_beginning=2.0;
        delta=(self.x_max-self.x_min-delta_beginning)/(self.total_num_obs);
        x=delta_beginning + self.x_min + i*delta #random.uniform(self.x_min, self.x_max);
        y=random.uniform(self.y_min, self.y_max);
        z=random.uniform(self.z_min, self.z_max);
        offset=random.uniform(-2*math.pi, 2*math.pi);
        slower=random.uniform(self.slower_min, self.slower_max);
        s=self.scale
        if(self.getType(i)=="dynamic"):
            mesh=random.choice(self.available_meshes_dynamic);
            bbox=self.bbox_dynamic;
            [x_string, y_string, z_string] = self.trefoil(x,y,z, self.scale[0],self.scale[1],self.scale[2], offset, slower)
        else:
            mesh=random.choice(self.available_meshes_static);
            bbox=self.bbox_static_vert;
            z=bbox[2]/2.0;
            [x_string, y_string, z_string] = self.wave_in_z(x, y, z, self.scale[2], offset, 1.0)
        return [x_string, y_string, z_string, x, y, z, mesh, bbox]

    def getType(self,i):
        if(i<self.num_of_dyn_objects):
            return "dynamic"
        else:
            return "static"

    def __init__(self, total_num_obs,gazebo):
        self.state=State()

        name = rospy.get_namespace()
        self.name = name[1:-1]

        print(total_num_obs)
        self.total_num_obs=total_num_obs;
        self.num_of_dyn_objects=int(1.0*total_num_obs);
        self.num_of_stat_objects=total_num_obs-self.num_of_dyn_objects; 
        self.x_min= 2.0 
        self.x_max= 40.0
        self.y_min= -1.0 
        self.y_max= 1.0
        self.z_min= 0.5 
        self.z_max= 0.5
        # self.scale= [(self.x_max-self.x_min)/self.total_num_obs, 5.0, 1.0]
        self.scale= [2.0, 2.0, 2.0]
        self.slower_min=1.5   #1.2 or 2.3
        self.slower_max=1.5   #1.2 or 2.3
        self.bbox_dynamic=[0.8, 0.8, 0.8] 
        self.bbox_static_vert=[0.4, 0.4, 4]
        self.bbox_static_horiz=[0.4, 8, 0.4]
        self.percentage_vert=0.0;
        self.name_obs="obs_"
        self.max_vel_obstacles=-10.0;
   
        #HACK
        # self.num_of_dyn_objects=1;
        # self.num_of_stat_objects=0;
        # self.x_min= 4.0
        # self.x_max= 4.0
        # self.y_min= 4.0 
        # self.y_max= 4.0
        # self.z_min= 1.0 
        # self.z_max= 1.0
        # self.scale= [1.0, 1.0, 2.5]
        # self.bbox_dynamic=[0.2, 0.2, 0.2] 
        #END OF HACK

        self.available_meshes_static=["package://panther/meshes/ConcreteDamage01b/model3.dae", "package://panther/meshes/ConcreteDamage01b/model2.dae"]
        self.available_meshes_dynamic=["package://panther/meshes/ConcreteDamage01b/model4.dae"]

        self.marker_array=MarkerArray();
        self.all_dyn_traj=[]
        self.all_dyn_traj_zhejiang=[]

        self.total_num_obs=self.num_of_dyn_objects + self.num_of_stat_objects

        for i in range(self.total_num_obs): 
            [traj_x, traj_y, traj_z, x, y, z, mesh, bbox]=self.getTrajectoryPosMeshBBox(i);           
            self.marker_array.markers.append(self.generateMarker(mesh, bbox, i));

            dynamic_trajectory_msg=DynTraj(); 
            dynamic_trajectory_msg.use_pwp_field=False;
            dynamic_trajectory_msg.is_agent=False;
            dynamic_trajectory_msg.header.stamp= rospy.Time.now();
            dynamic_trajectory_msg.s_mean = [traj_x, traj_y, traj_z]
            dynamic_trajectory_msg.s_var = ["0.001", "0.001", "0.001"] #TODO (a nonzero variance is needed to choose the obstacle to focus on, see panther.cpp)
            dynamic_trajectory_msg.bbox = [bbox[0], bbox[1], bbox[2]];
            dynamic_trajectory_msg.pos.x=x #Current position, will be updated later
            dynamic_trajectory_msg.pos.y=y #Current position, will be updated later
            dynamic_trajectory_msg.pos.z=z #Current position, will be updated later
            dynamic_trajectory_msg.id = 4000+ i #Current id 4000 to avoid interference with ids from agents #TODO

            self.all_dyn_traj.append(dynamic_trajectory_msg);

        self.all_dyn_traj_zhejiang=copy.deepcopy(self.all_dyn_traj);

        self.pubTraj = rospy.Publisher('/trajs', DynTraj, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh = rospy.Publisher('/obstacles_mesh', MarkerArray, queue_size=1, latch=True)

        # self.pubShapes_dynamic_mesh_zhejiang = rospy.Publisher('/obstacles_mesh_zhejiang', MarkerArray, queue_size=1, latch=True)
        self.pubShapes_dynamic_mesh_colored = rospy.Publisher('/obstacles_mesh_colored', MarkerArray, queue_size=1, latch=True)
        self.pubTraj_zhejiang = rospy.Publisher('/SQ01s/trajs_zhejiang', DynTraj, queue_size=1, latch=True)

        #self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=100)


        if(gazebo):
            # Spawn all the objects in Gazebo
            for i in range(self.total_num_obs):
                self.spawnGazeboObstacle(i)


        rospy.sleep(0.5)

    def generateMarker(self, mesh, bbox, i):
        marker=Marker();
        marker.id=i;
        marker.ns="mesh";
        marker.header.frame_id="world"
        marker.type=marker.MESH_RESOURCE;
        marker.action=marker.ADD;

        marker.pose.position.x=0.0 #Will be updated later
        marker.pose.position.y=0.0 #Will be updated later
        marker.pose.position.z=0.0 #Will be updated later
        marker.pose.orientation.x=0.0;
        marker.pose.orientation.y=0.0;
        marker.pose.orientation.z=0.0;
        marker.pose.orientation.w=1.0;
        marker.lifetime = rospy.Duration.from_sec(0.0);
        marker.mesh_use_embedded_materials=True
        marker.mesh_resource=mesh

        marker.scale.x=bbox[0];
        marker.scale.y=bbox[1];
        marker.scale.z=bbox[2];
        return marker

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()

        marker_array_static_mesh=MarkerArray();
        marker_array_dynamic_mesh=MarkerArray();

        for i in range(self.total_num_obs): 
            t_ros=rospy.Time.now()
            t=rospy.get_time(); #Same as before, but it's float

            marker=self.marker_array.markers[i];
          
            x = eval(self.all_dyn_traj[i].s_mean[0])
            y = eval(self.all_dyn_traj[i].s_mean[1])
            z = eval(self.all_dyn_traj[i].s_mean[2])

            # Set the stamp and the current pos
            self.all_dyn_traj[i].header.stamp= t_ros;
            self.all_dyn_traj[i].pos.x=x #Current position
            self.all_dyn_traj[i].pos.y=y #Current position
            self.all_dyn_traj[i].pos.z=z #Current position

            self.pubTraj.publish(self.all_dyn_traj[i])
            br.sendTransform((x, y, z), (0,0,0,1), t_ros, self.name_obs+str(self.all_dyn_traj[i].id), "world")


            self.marker_array.markers[i].pose.position.x=x;
            self.marker_array.markers[i].pose.position.y=y;
            self.marker_array.markers[i].pose.position.z=z;



            #If you want to move the objets in gazebo. BETTER WITH THE PLUGIN
            # gazebo_state = ModelState()
            # gazebo_state.model_name = str(i)#"all_"+str(i)
            # gazebo_state.pose.position.x = x
            # gazebo_state.pose.position.y = y
            # gazebo_state.pose.position.z = z
            # gazebo_state.reference_frame = "world" 
            # self.pubGazeboState.publish(gazebo_state) 


            # x=self.x_all[i];
            # y=self.y_all[i];
            # z=self.z_all[i];

            # s="""
            # rosservice call /gazebo/set_model_state "model_state:
            #   model_name: '"""+str(i)+"""'
            #   pose:
            #     position:
            #       x: """+str(x)+"""
            #       y: """+str(y)+"""
            #       z: """+str(z)+"""
            #     orientation:
            #       x: 0.0
            #       y: 0.0
            #       z: 0.0
            #       w: 0.0
            #   twist:
            #     linear:
            #       x: 0.0
            #       y: 0.0
            #       z: 0.0
            #     angular:
            #       x: 0.0
            #       y: 0.0
            #       z: 0.0
            #   reference_frame: 'world'" 
            # """
            # os.system(s) 

            #If you want to see the objects in rviz

        self.pubShapes_dynamic_mesh.publish(self.marker_array)

        # #####################################################
        # # START (This is for the benchmark with the code from zhejiang)
        # # In their code, the function EGOReplanFSM::marker_callback has access to pos/vel from the obstacles through the marker array. The code below publishes the marker array with that info
        # ###################################################
        # marker_array_zhejiang=MarkerArray();
        # for i in range(len(self.marker_array.markers)): 

        #     marker=Marker();
        #     delta=1e-6; #Don't make it smaller than this
        #     t0=rospy.get_time();
        #     t=t0
        #     # print "t= ", t
        #     x = eval(self.all_dyn_traj[i].s_mean[0])
        #     y = eval(self.all_dyn_traj[i].s_mean[1])
        #     z = eval(self.all_dyn_traj[i].s_mean[2])
        #     t=t+delta;
        #     xd = eval(self.all_dyn_traj[i].s_mean[0])
        #     yd = eval(self.all_dyn_traj[i].s_mean[1])
        #     zd = eval(self.all_dyn_traj[i].s_mean[2])
        #     vel=[(xd-x)/delta, (yd-y)/delta, (zd-z)/delta];


        #     # print "vel= ", vel
        #     # print self.all_dyn_traj[i].s_mean

        #     marker.pose.position.x=x;
        #     marker.pose.position.y=y;
        #     marker.pose.position.z=z;

        #     marker.pose.orientation.x=vel[0];
        #     marker.pose.orientation.y=vel[1];
        #     marker.pose.orientation.z=vel[2];
        #     marker.pose.orientation.w=1.0;

        #     marker.color.r=vel[0];
        #     marker.color.g=vel[1];
        #     marker.color.b=vel[2];
        #     marker.color.a=0.2;

        #     marker.scale=self.marker_array.markers[i].scale;

        #     marker_array_zhejiang.markers.append(marker)


   
        #     self.all_dyn_traj_zhejiang[i].s_mean[0]=str(x)+'+'+str(vel[0])+'*(t-'+str(t0)+')';
        #     self.all_dyn_traj_zhejiang[i].s_mean[1]=str(y)+'+'+str(vel[1])+'*(t-'+str(t0)+')';
        #     self.all_dyn_traj_zhejiang[i].s_mean[2]=str(z)+'+'+str(vel[2])+'*(t-'+str(t0)+')';

        #     self.all_dyn_traj_zhejiang[i].pos.x=x;
        #     self.all_dyn_traj_zhejiang[i].pos.y=y;
        #     self.all_dyn_traj_zhejiang[i].pos.z=z;

        #     self.pubTraj_zhejiang.publish(self.all_dyn_traj_zhejiang[i])

        #     self.max_vel_obstacles=max(self.max_vel_obstacles, np.linalg.norm(np.array(vel)))
        #     # print("self.max_vel_obstacles= ", self.max_vel_obstacles)


        # self.pubShapes_dynamic_mesh_zhejiang.publish(marker_array_zhejiang)       
        # #####################################################
        # # END
        # ###################################################


        # #####################################################
        # # Publish the colored markers
        # ###################################################
        # id_marker_tmp=0
        # marker_array_colored=MarkerArray();
        # for i in range(len(self.marker_array.markers)): 

        #     tinit=rospy.get_time();
        #     tfinal=rospy.get_time()+5;
        #     for t in np.linspace(tinit,tfinal,21).tolist():
        #          # print "t= ", t
        #         x = eval(self.all_dyn_traj[i].s_mean[0])
        #         y = eval(self.all_dyn_traj[i].s_mean[1])
        #         z = eval(self.all_dyn_traj[i].s_mean[2])    
                
        #         marker=Marker();           

        #         marker.pose.position.x=x;
        #         marker.pose.position.y=y;
        #         marker.pose.position.z=z;

        #         marker.pose.orientation.x=0;
        #         marker.pose.orientation.y=0;
        #         marker.pose.orientation.z=0;
        #         marker.pose.orientation.w=1.0;

        #         marker.color=getColorJet(t,tinit,tfinal)

        #         marker.scale=self.marker_array.markers[i].scale;

        #         marker.header.frame_id = "/world"
        #         marker.type = marker.CUBE
        #         marker.action = marker.ADD
        #         marker.id = id_marker_tmp
        #         marker.ns = 'colored_dyn_corridor'

        #         marker_array_colored.markers.append(marker)
        #         id_marker_tmp=id_marker_tmp+1;


        #         marker2=copy.deepcopy(marker);
        #         marker2.color.r=0.0;
        #         marker2.color.g=0.0;
        #         marker2.color.b=0.0;
        #         marker2.color.a=0.3;
        #         marker2.id = id_marker_tmp
        #         marker2.scale.x =  marker2.scale.x+0.05;
        #         marker2.scale.y =  marker2.scale.y+0.05;
        #         marker2.scale.z =  marker2.scale.z+0.05;


        #         marker_array_colored.markers.append(marker2)
        #         id_marker_tmp=id_marker_tmp+1;
            
        # self.pubShapes_dynamic_mesh_colored.publish(marker_array_colored)       
        # #####################################################
        # # END
        # ###################################################



    def static(self,x,y,z):
        return [str(x), str(y), str(z)]

    # Trefoil knot, https://en.wikipedia.org/wiki/Trefoil_knot
    def trefoil(self,x,y,z,scale_x, scale_y, scale_z, offset, slower):

        #slower=1.0; #The higher, the slower the obstacles move" 
        tt='t/' + str(slower)+'+';

        x_string=str(scale_x/6.0)+'*(sin('+tt +str(offset)+') + 2 * sin(2 * '+tt +str(offset)+'))' +'+' + str(x); #'2*sin(t)' 
        y_string=str(scale_y/5.0)+'*(cos('+tt +str(offset)+') - 2 * cos(2 * '+tt +str(offset)+'))' +'+' + str(y); #'2*cos(t)' 
        z_string=str(scale_z/2.0)+'*(-sin(3 * '+tt +str(offset)+'))' + '+' + str(z);                               #'1.0'        

        # x_string='1';
        # y_string='0';
        # z_string='1';

        return [x_string, y_string, z_string]

    def square(self,x,y,z,scale_x, scale_y, scale_z, offset, slower):

        #slower=1.0; #The higher, the slower the obstacles move" 
        tt='t/' + str(slower)+'+';

        tt='(t+'+str(offset)+')'+'/' + str(slower);
        cost='cos('+tt+')';
        sint='sin('+tt+')';



        #See https://math.stackexchange.com/questions/69099/equation-of-a-rectangle
        x_string=str(scale_x)+'*0.5*(abs('+cost+')*'+cost + '+abs('+sint+')*'+sint+')';
        y_string=str(scale_x)+'*0.5*(abs('+cost+')*'+cost + '-abs('+sint+')*'+sint+')';
        z_string=x_string;

        #Now let's rotate around the z axis
        costheta='cos('+str(offset)+')';
        sintheta='sin('+str(offset)+')';
        x_rotated_string='('+costheta+'*'+x_string +'-'+ sintheta+'*'+y_string+')'; 
        y_rotated_string='('+sintheta+'*'+x_string +'+'+ costheta+'*'+y_string+')'; 
        z_rotated_string=z_string; 

        x_rotated_string=x_rotated_string+'+' + str(x);
        y_rotated_string=y_rotated_string+'+' + str(y);
        z_rotated_string=z_rotated_string+'+' + str(z);

        # x_rotated_string='1';
        # y_rotated_string='0';
        # z_rotated_string='1';

        return [x_rotated_string, y_rotated_string, z_rotated_string]

    def wave_in_z(self,x,y,z,scale, offset, slower):

        tt='t/' + str(slower)+'+';

        x_string=str(x);
        y_string=str(y)
        z_string=str(scale)+'*(-sin( '+tt +str(offset)+'))' + '+' + str(z);  

        # x_string='1';
        # y_string='1';
        # z_string='1';                   

        return [x_string, y_string, z_string]

    def spawnGazeboObstacle(self, i):

            rospack = rospkg.RosPack();
            path_panther=rospack.get_path('panther');
            path_file=path_panther+"/meshes/tmp_"+str(i)+".urdf"

            f = open(path_file, "w") #TODO: This works, but it'd better not having to create this file
            scale=self.marker_array.markers[i].scale;
            scale='"'+str(scale.x)+" "+str(scale.y)+" "+str(scale.z)+'"';


            x=self.all_dyn_traj[i].pos.x;
            y=self.all_dyn_traj[i].pos.y;
            z=self.all_dyn_traj[i].pos.z;

            #Remember NOT to include de <collision> tag (Gazebo goes much slower if you do so)
            f.write("""
<robot name="name_robot">
  <link name="name_link">
    <inertial>
      <mass value="0.200" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="5.8083e-4" ixy="0" ixz="0" iyy="3.0833e-5" iyz="0" izz="5.9083e-4" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="""+'"'+self.marker_array.markers[i].mesh_resource+'"'+""" scale="""+scale+"""/>
      </geometry>
    </visual>
  </link>
  <gazebo>
    <plugin name="move_model" filename="libmove_model.so">
    <traj_x>"""+self.all_dyn_traj[i].s_mean[0]+"""</traj_x>
    <traj_y>"""+self.all_dyn_traj[i].s_mean[1]+"""</traj_y>
    <traj_z>"""+self.all_dyn_traj[i].s_mean[2]+"""</traj_z>
    </plugin>
  </gazebo>
</robot>
                """)
  # <plugin name="pr2_pose_test" filename="libpr2_pose_test.so"/>
            f.close()

            # os.system("rosrun gazebo_ros spawn_model -file `rospack find panther`/meshes/tmp.urdf -urdf -x " + str(x) + " -y " + str(y) + " -z " + str(z) + " -model "+self.name_obs+str(i)+" &"); #all_
            os.system("rosrun gazebo_ros spawn_model -file "+path_file+" -urdf -x " + str(x) + " -y " + str(y) + " -z " + str(z) + " -model "+self.name_obs+str(i)+" && rm "+path_file + " &"); #all_
            # os.remove(path_file)

             

def startNode(total_num_obs, gazebo):
    c = DynCorridor(total_num_obs,gazebo)
    rospy.Timer(rospy.Duration(0.01), c.pubTF)
    rospy.spin()

if __name__ == '__main__':

    # TODO: use https://docs.python.org/3.3/library/argparse.html
    print("********************************")
    print(sys.argv)
    if(len(sys.argv)<=1):
        # print("Usage: python dynamic_corridor.py [Num_of_obstacles]")
        total_num_obs=45; 
    else:
        total_num_obs=int(sys.argv[1])


    gazebo=((sys.argv[2]=='True') or (sys.argv[2]=='true'))

    # total_num_obs=140
    ns = rospy.get_namespace()
    try:
        rospy.init_node('dynamic_obstacles')
        startNode(total_num_obs,gazebo)
    except rospy.ROSInterruptException:
        pass



        # See https://en.wikipedia.org/wiki/Triangle_wave
        # x_string='2*abs('+tt+str(offset+0.5)+'+''-floor('+tt+str(offset+0.5)+'+'+'0.5))'+'+' + str(x); #'2*sin(t)' 
        # y_string='2*abs('+tt+str(offset)+'+''-floor('+tt+str(offset)+'+'+'0.5))'+'+' + str(y); #'2*sin(t)' 
        # z_string='2*abs('+tt+str(offset+1.0)+'+''-floor('+tt+str(offset+1.0)+'+'+'0.5))'+'+' + str(z); #'2*sin(t)' 

        # x_string=x_string+'+'+x_string;
        # y_string=y_string+'+'+y_string;
        # z_string=z_string+'+'+z_string;

        
        # x_string=str(scale_x)+'*0.5*sgn(cos('+tt+str(offset)+'))';
        # y_string=str(scale_y)+'*0.5*sgn(cos('+tt+str(offset)+'))';
        # z_string=str(scale_z)+'*0.5*sgn(cos('+tt+str(offset)+'))';




        # p=rotz(45)*[0.5*w*sign(cos(t));0.5*h*sign(sin(t));0.5*w*sign(cos(t))]