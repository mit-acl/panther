#!/usr/bin/env python
# coding=utf-8

# /* ----------------------------------------------------------------------------
#  * Copyright 2020, Jesus Tordesillas Torres, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Jesus Tordesillas, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */


import math
import os
import sys
import time
import rospy
from snapstack_msgs.msg import State
import subprocess

if __name__ == '__main__':


    num_of_sims=10;
    num_of_obs=[4, 6, 8, 10, 12, 14, 16, 18, 20]#[1000]#[50,400,500,600,700]#[150, 200, 250, 300, 350] #[340,380,420,460,500]; #140,180,220,260,300
    commands = []

    seconds_replanning=5

    estimated_total_min = seconds_replanning*num_of_sims*len(num_of_obs)/60
    print(f"Estimated total time={estimated_total_min} min")

    # folder_bags="/home/jtorde/Desktop/ws/src/panther/panther/bags";
    folder_bags="/home/jtorde/Dropbox\ \(MIT\)/Research/Planning_project/PANTHER/simulation_complexity/bags";
    all_modes=["panther"] #   "zhejiang", "panther", "noPA", "py", "ysweep"
    name_node_record="bag_recorder"
    kill_all="tmux kill-server & killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roscore & killall -9 rosmaster & pkill panther_node & pkill -f dynamic_obstacles & pkill -f rosout & pkill -f behavior_selector_node & pkill -f rviz & pkill -f rqt_gui & pkill -f perfect_tracker & pkill -f panther_commands"

    #make sure ROS (and related stuff) is not running
    os.system(kill_all)


    for k in range(len(num_of_obs)):
        for j in range(len(all_modes)):
            mode=all_modes[j]

            for s in range(num_of_sims):

                commands = []

                seconds_sleep_before=max(2*num_of_obs[k], 2.0)

                if(mode=="zhejiang"):
                   commands.append("roslaunch ego_planner swarm.launch rviz:=false num_of_obs:="+str(num_of_obs[k]));
                else:
                   commands.append("roslaunch panther simulation.launch gazebo:=true perfect_tracker:=true perfect_prediction:=true quad:=SQ01s gui_mission:=false rviz:=false mode:="+mode+" num_of_obs:="+str(num_of_obs[k]));

                commands.append("sleep "+str(seconds_sleep_before)+" && cd "+folder_bags+" && rosbag record -O "+mode+"_obs_"+str(num_of_obs[k])+"_sim_"+str(s)+" /SQ01s/goal /SQ01s/state /SQ01s/panther/log /SQ01s/panther/traj_obtained /tf /tf_static /SQ01s/panther/fov /obstacles_mesh __name:="+name_node_record);
                #publishing the goal should be the last command
                commands.append("sleep "+str(seconds_sleep_before)+" && rostopic pub /SQ01s/term_goal geometry_msgs/PoseStamped \'{header: {stamp: now, frame_id: \"world\"}, pose: {position: {x: 6.0, y: 0, z: 1}, orientation: {w: 1.0}}}\'");

                print("len(commands)= " , len(commands))
                session_name="run_many_sims_single_agent_session"
                os.system("tmux kill-session -t" + session_name)

                os.system("tmux new -d -s "+str(session_name)+" -x 300 -y 300")

                for i in range(len(commands)):
                    print('splitting ',i)
                    os.system('tmux split-window ; tmux select-layout tiled')
               
                for i in range(len(commands)):
                    os.system('tmux send-keys -t '+str(session_name)+':0.'+str(i) +' "'+ commands[i]+'" '+' C-m')

                print("Commands sent for "+"[Sim "+str(s)+", with mode="+mode+", with num_of_obs="+str(num_of_obs[k])+"]")

                time.sleep(seconds_sleep_before + seconds_replanning)
                # pos_string=""
                # while (pos_string.find('39.')==-1 and pos_string.find('40.')==-1): #note the point after the 49, to make sure they are the first two numbers, and not the decimals
                #     try:
                #         pos_string =str(subprocess.check_output(['rostopic', 'echo', '/SQ01s/state/pos/x', '-n', '1']))
                #         print("Currently at ",pos_string, "[Sim "+str(s)+", with mode="+mode+", with num_of_obs="+str(num_of_obs[k])+"]");
                #     except:
                #         print("An Error occurred")


                print("Timeout reached, killing the bag node")
                os.system("rosnode kill "+name_node_record);
                time.sleep(0.5)
                print("Killing the rest")
                os.system(kill_all)

    time.sleep(3.0)
    # os.system("sed -i '/visual/s/^#//g' $(rospack find panther)/param/panther.yaml") #comment out visual param
    # os.system("sed -i '/mode/s/^#//g' $(rospack find panther)/param/panther.yaml") #comment out mode param