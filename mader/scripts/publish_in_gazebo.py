#!/usr/bin/env python

import roslib
import rospy
import copy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from snapstack_msgs.msg import State


class GazeboPublisher:

    def __init__(self):
        self.gazebo_state = ModelState()
        self.pubState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        name = rospy.get_namespace()
        self.name = name[1:-1]


    def stateCB(self, data):
        self.gazebo_state.model_name = self.name
        self.gazebo_state.pose.orientation = data.quat
        self.gazebo_state.pose.position = data.pos
        self.gazebo_state.reference_frame = "world"   
        self.pubState.publish(self.gazebo_state)        

def startNode():
    c = GazeboPublisher()
    rospy.Subscriber("state", State, c.stateCB)
    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('relay')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=mQ01 $ rosrun quad_control joy.py")
        else:
            print "Starting joystick teleop node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
