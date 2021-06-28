/* ----------------------------------------------------------------------------
 * Copyright 2021, Jesus Tordesillas Torres, Aerospace Controls Laboratory
 * Massachusetts Institute of Technology
 * All Rights Reserved
 * Authors: Jesus Tordesillas, et al.
 * See LICENSE file for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <snapstack_msgs/State.h>
#include <snapstack_msgs/Goal.h>

#include <panther_msgs/WhoPlans.h>
#include <panther_msgs/DynTraj.h>

#include "utils.hpp"
#include "panther.hpp"
#include "panther_types.hpp"

#include "timer.hpp"

// #define WHOLE 1  // Whole trajectory (part of which is planned on unkonwn space)
// #define SAFE 2   // Safe path

namespace rvt = rviz_visual_tools;

class PantherRos
{
public:
  PantherRos(ros::NodeHandle nh1, ros::NodeHandle nh2, ros::NodeHandle nh3);
  ~PantherRos();

private:
  std::unique_ptr<Panther> panther_ptr_;

  void verify(bool cond, std::string info_if_false);

  void publishOwnTraj(const mt::PieceWisePol& pwp);
  void publishPlanes(std::vector<Hyperplane3D>& planes);

  // class methods
  void pubTraj(const std::vector<mt::state>& data);
  void terminalGoalCB(const geometry_msgs::PoseStamped& msg);
  void pubState(const mt::state& msg, const ros::Publisher pub);
  void stateCB(const snapstack_msgs::State& msg);
  void whoPlansCB(const panther_msgs::WhoPlans& msg);
  void pubCB(const ros::TimerEvent& e);
  void replanCB(const ros::TimerEvent& e);
  void trajCB(const panther_msgs::DynTraj& msg);

  // void clearMarkerSetOfArrows();
  void clearMarkerActualTraj();
  void clearMarkerColoredTraj();

  void pubActualTraj();
  visualization_msgs::MarkerArray clearArrows();
  // geometry_msgs::Vector3 vectorNull();

  void clearMarkerArray(visualization_msgs::MarkerArray* tmp, ros::Publisher* publisher);

  void publishPoly(const vec_E<Polyhedron<3>>& poly);
  // visualization_msgs::MarkerArray Matrix2ColoredMarkerArray(Eigen::MatrixXd& X, int type);

  void publishText();

  void publishFOV();

  void pubObstacles(mt::Edges edges_obstacles);

  void constructFOVMarker();

  mt::state state_;

  std::string world_name_ = "world";

  rvt::RvizVisualToolsPtr visual_tools_;

  visualization_msgs::Marker E_;
  visualization_msgs::Marker A_;
  visualization_msgs::Marker setpoint_;

  ros::NodeHandle nh1_;
  ros::NodeHandle nh2_;
  ros::NodeHandle nh3_;

  ros::Publisher pub_point_G_;
  ros::Publisher pub_point_G_term_;
  ros::Publisher pub_goal_;
  ros::Publisher pub_traj_safe_;
  ros::Publisher pub_setpoint_;
  ros::Publisher pub_actual_traj_;

  ros::Publisher pub_point_A_;

  ros::Publisher pub_traj_safe_colored_;

  ros::Publisher pub_text_;
  ros::Publisher pub_traj_;

  ros::Publisher poly_safe_pub_;

  ros::Publisher pub_fov_;
  ros::Publisher pub_obstacles_;
  ros::Publisher pub_log_;

  ros::Subscriber sub_term_goal_;
  ros::Subscriber sub_whoplans_;
  ros::Subscriber sub_state_;
  ros::Subscriber sub_traj_;

  ros::Timer pubCBTimer_;
  ros::Timer replanCBTimer_;

  mt::parameters par_;  // where all the parameters are

  std::string name_drone_;

  std::vector<std::string> traj_;  // trajectory of the dynamic obstacle

  visualization_msgs::MarkerArray traj_safe_colored_;

  int actual_trajID_ = 0;

  int num_of_LPs_run_ = 0;
  int num_of_QCQPs_run_ = 0;

  int id_;  // id of the drone

  bool published_initial_position_ = false;

  // Eigen::Affine3d W_T_B_;

  mt::PieceWisePol pwp_last_;

  PANTHER_timers::Timer timer_stop_;

  visualization_msgs::Marker marker_fov_;

  std::string name_camera_depth_optical_frame_tf_;
};
