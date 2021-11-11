import math
import os
import time
import rospy
import rosgraph
from geometry_msgs.msg import PoseStamped
from snapstack_msgs.msg import Goal
import numpy as np

class TermGoalSender:

    def __init__(self):
        self.term_goal=PoseStamped();
        self.term_goal.header.frame_id='world';
        self.pubTermGoal = rospy.Publisher('/SQ01s/term_goal', PoseStamped, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCB)
        self.sign=1.0;

        self.sendGoal(0.0, self.sign)

        self.goal_pos=np.array([0.0, 0.0, 0.0])


        self.time_init=time.time()

        self.total_secs=60;


    def timerCB(self, tmp):
        self.term_goal_pos=np.array([self.term_goal.pose.position.x,self.term_goal.pose.position.y,self.term_goal.pose.position.z])

        dist=np.linalg.norm(self.term_goal_pos-self.goal_pos)

        print("dist=", dist)

        if(dist<4.0):
            self.sendGoal(0.0, self.sign)
            self.sign=-self.sign;


    def sendGoal(self, t, sign):
        center=[6.0, 4.5, 1.0];
        radius=5.0
        scale=6.0;
        t=0.0;
        x=center[0]+sign*radius
        y=center[1]#-radius
        # x=center[0]+sign*radius*math.cos(2*math.pi*t/(scale*self.total_secs));
        # y=center[1]+sign*radius*math.sin(2*math.pi*t/(scale*self.total_secs));
        z=1.0

        self.term_goal.pose.position.x=x;
        self.term_goal.pose.position.y=y;
        self.term_goal.pose.position.z=z;

        self.pubTermGoal.publish(self.term_goal)  

        print ("Goal sent!!")      

        return

    def goalCB(self, data):
        # print("IN GOAL CB")
        self.goal_pos=np.array([data.p.x, data.p.y, data.p.z])

       

def startNode():
    c = TermGoalSender()
    rospy.Subscriber("/SQ01s/goal", Goal, c.goalCB)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('TermGoalSender')
    startNode()


